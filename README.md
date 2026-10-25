# Autonomous Exploration and Airdrop for UAVs

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![PX4](https://img.shields.io/badge/PX4-MAVROS%202-1A5CFF)](https://px4.io/)
[![LiDAR](https://img.shields.io/badge/LiDAR-Livox%20Mid--360-00A0E9)](https://www.livoxtech.com/mid-360)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20Orin%20NX-76B900?logo=nvidia)](https://developer.nvidia.com/embedded)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

An autonomous **exploration + vision-guided precision airdrop** system for a PX4 multicopter,
running fully onboard (no GPS, no ground station in the loop) on a Jetson Orin NX.

一套面向**无 GPS 场地**的无人机**自主探索 + 视觉引导精确空投**系统：机载运行（Jetson Orin NX），
由 FAST-LIO（Livox Mid-360）提供里程计与点云，PX4 经 MAVROS 接收位置/速度设定点，
自主探索场地、发现标靶后精确投放、投满后穿越带窄口的走廊返回降落点。

<p align="center">
  <img src="docs/drone.jpg" alt="Experimental airframe" width="640">
</p>

<p align="center">
  <i>实物平台：Livox Mid-360（顶部，前向倾斜 15°）+ Jetson Orin NX / PX4 机载栈
  + 前视相机（下倾 55°）+ 三路投放机构 + 桨叶保护罩</i>
</p>

---

## Overview 项目简介

* **自主探索** — 用激光点云构建二维占据栅格地图，基于 frontier（未知区域边界）自动选择
  视点，用 A\* 规划路径、B 样条生成平滑轨迹，并把相机视野（FOV）覆盖的区域标记为已探索。
* **视觉投放** — 视觉节点检测地面标靶并把坐标发给本节点；本节点飞近目标、悬停采样、
  降到投放高度、精确对准后通过串口触发放投机构。
* **返航降落** — 投满（或探索饱和）后飞到场地角点，降到通道高度，穿过带窄口的走廊，
  最后在远端降落点着陆。
* **纯机载、无 GPS** — 飞行控制器以激光里程计作为唯一位置/高度来源，
  因此所有高度指令都在 FAST-LIO 的世界系下表达。

---

## Key Features 主要功能

| 模块 | 功能 |
|---|---|
| 建图 | 增量式二维占据栅格、射线式自由空间更新、障碍物膨胀 |
| 视点选择 | frontier 边界点 + 距离/朝向/密度评分、远端优先、近点惩罚 |
| FOV 覆盖 | 前视下倾相机扇形（可选向下矩形），遮挡感知（射线遇障即停） |
| 点云高度带 | 按**相对无人机当前高度**的上下非对称带过滤，抗雷达 Z 漂移 |
| 轨迹 | A\* + 二维 B 样条，速度/加速度/偏航角速度限制，独立 50 Hz 设定点线程 |
| 目标管理 | 多类别优先级、按探索进度解锁的补充类别、起飞点附近误检过滤、同类/同位置去重 |
| 精确投放 | 机体系三投放口偏移、视觉基准点、低空精细对准、重复触发与稳定判定 |
| 走廊返航 | 角点 → 入口 → 窄口中点检测与穿越（安全余量自动放宽）→ 远端降落 |
| 任务保护 | 执行视觉任务期间不允许"覆盖率达标返航"打断，保证在途投放不中断 |

---

## System Architecture 系统架构

```
        ┌──────────────────────┐    /livox/imu, /livox/lidar
        │ Livox Mid-360 (15°↧) │───────────────┐
        └──────────────────────┘               │
                                               ▼
   ┌────────────┐   /Odometry, /cloud_registered   ┌─────────────────────┐
   │  FAST-LIO  │──────────────────────────────────▶│ px4_exploration_node│
   └────────────┘                                   │    (this repo)      │
                                                    └──────┬──────────────┘
   ┌────────────┐  /detected_object_class{0,1,2,3,5}      │
   │ Vision node│───────────────────────────────────────▶│
   └────────────┘                                         │ setpoint_raw/local (50 Hz)
                                                          ▼
                                                    ┌──────────┐      ┌──────────────┐
                                                    │  MAVROS  │─────▶│     PX4      │
                                                    └──────────┘      └──────┬───────┘
                                                                             │
                                                    ┌────────────────────────▼───────┐
                                                    │ Payload board (serial)         │
                                                    │ bomb_0 / bomb_1 / bomb_2       │
                                                    └────────────────────────────────┘
```

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/mavros/state` | `mavros_msgs/State` | 解锁 / OFFBOARD 监控 |
| 订阅 | `/Odometry` | `nav_msgs/Odometry` | FAST-LIO 位姿 |
| 订阅 | `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO 配准点云 |
| 订阅 | `/detected_object_class{0,1,2,3,5}` | `geometry_msgs/Point` | 视觉标靶坐标 |
| 发布 | `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | 位置/速度设定点 |
| 串口 | `/dev/mcu @115200` | — | 投放触发（`bomb_N`） |

---

## Repository Layout 目录结构

```
.
├── src/
│   ├── px4_exploration/                   # 主节点：探索 + 视觉任务 + 返航
│   │   ├── src/
│   │   │   ├── exploration_main.cpp       # ROS 节点、状态机、视觉任务、投放流程
│   │   │   └── Arduino.cpp                # 投放机构串口驱动
│   │   ├── include/px4_exploration/
│   │   │   ├── Px4ExplorationPlanner.hpp  # 探索状态机、视点选择、返航与走廊
│   │   │   ├── GridMap.hpp                # 占据栅格地图 + FOV 覆盖标记
│   │   │   ├── FrontierDetector.hpp       # frontier 提取与更新
│   │   │   ├── PathPlanner.hpp            # 栅格 A* 路径规划
│   │   │   ├── BsplineTrajectory2D.hpp    # 二维 B 样条轨迹
│   │   │   ├── BsplineOptimizer2D.hpp     # 轨迹优化/约束
│   │   │   ├── GuiRenderer.hpp            # 可选 SDL2 地图可视化
│   │   │   ├── Arduino.h
│   │   │   └── Types.hpp
│   │   ├── launch/explore.launch.py       # 任务参数入口
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── serial_drop_test/                  # 投放机构单独测试节点
├── docs/drone.jpg                         # 实物平台照片
├── README.md
└── LICENSE
```

> 本仓库只包含 `src/` 下的两个 ROS 2 包。视觉节点（TensorRT YOLO）、FAST-LIO 配置、
> 机载脚本与离线仿真**不在**本仓库内。
> This repository contains only the two ROS 2 packages under `src/`; the vision node,
> FAST-LIO configuration, onboard scripts and the offline simulator are not included.

---

## Hardware 硬件

| 项目 | 型号 / 说明 |
|---|---|
| 机载计算机 | Jetson Orin NX，Ubuntu 22.04 + ROS 2 Humble |
| 激光雷达 | Livox Mid-360，**前向倾斜 15° 安装**，内置 IMU |
| 相机 | USB 相机，**下倾 55°**，1280×720 |
| 飞控 | PX4 + MAVROS 2，位置/高度参考 = 激光里程计 |
| 投放机构 | Arduino 板，串口 `/dev/mcu @115200`，三路释放口 |

---

## Software Stack 软件栈

* ROS 2 **Humble**
* [FAST-LIO 2](https://github.com/hku-mars/FAST_LIO) + `livox_ros_driver2`（里程计与点云）
* `mavros` / `mavros_msgs` + PX4
* PCL、Eigen
* OpenCV + TensorRT（视觉节点，独立包）
* SDL2（可选，`use_gui:=1` 显示地图窗口）

---

## Build 构建

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-msgs libpcl-dev

cd ~/fly_demo
colcon build --base-paths src --packages-select px4_exploration serial_drop_test
source install/setup.bash
```

---

## Run 运行

```bash
# 1) 雷达驱动 + FAST-LIO（独立工作空间）
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fast_lio mapping.launch.py

# 2) 等里程计稳定后再启动任务节点

# 3) 探索 + 投放节点
cd ~/fly_demo && source install/setup.bash
ros2 launch px4_exploration explore.launch.py
```

节点自动起飞、探索、执行视觉投放任务，然后返航降落；随时可切换到 PX4 `POSCTL` 接管。

> ⚠️ 首次调试请**拆下桨叶**；该节点会请求解锁并切入 OFFBOARD。

---

## Mission Flow 任务流程

```
TAKEOFF ─▶ EXPLORE ─┬─(发现标靶)─▶ VISION_TASK ─▶ DROP ─┬─▶ EXPLORE（下一个标靶）
                    │                                    └─▶ RETURN（投满）
                    └─(探索饱和/需要返航)─────────────────▶ RETURN

RETURN：角点 ─▶ 降到通道高度 ─▶ 走廊入口 ─▶ 穿越窄口 ─▶ 远端降落点 ─▶ 降落
```

---

## License

[MIT](LICENSE)

## Contributors

- **aaalixin** — 系统设计、算法实现、机载集成与实飞测试。
