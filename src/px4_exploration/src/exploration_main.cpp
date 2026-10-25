#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mavros_msgs/srv/command_tol.hpp>  // 添加这行
#include <thread>
#include <atomic>
#include <mutex>
#include <set>
#include <map>
#include <array>
#include <algorithm>

#include "px4_exploration/Types.hpp"
#include "px4_exploration/Arduino.h"   // 投放(串口)
int INFLATE_RADIUS = 3;
double MAP_X_MIN=-1,MAP_X_MAX=10,MAP_Y_MIN=-6,MAP_Y_MAX=6;
// Exploration sub-region (defaults to the full map rectangle; overwritten from
// launch).  Exploration only happens inside this sub-rectangle: free-space
// marking, coverage %, and viewpoint targets.  The landing corridor east of
// the internal wall (x>7.5) must stay OUTSIDE it (leave explore_x_max ≤ 7.4).
double EXPLORE_X_MIN=-1, EXPLORE_X_MAX=10, EXPLORE_Y_MIN=-6, EXPLORE_Y_MAX=6;
double EDGE_MARGIN = 0.7;
// A→B(走廊段 = 非探索区)规划时的边缘距离(m): 探索区外不加膨胀光晕, 只用这个值。
// 探索区内的膨胀(INFLATE_RADIUS*GRID_RES + DRONE_RADIUS)不受它影响。
thread_local double AB_EDGE_MARGIN = 0.25;
double DRONE_SPEED=0.5, MAX_ACCEL=1.5, MAX_YAW_RATE=2.0;

#include "px4_exploration/GridMap.hpp"
#include "px4_exploration/Px4ExplorationPlanner.hpp"
#include "px4_exploration/GuiRenderer.hpp"

class ExplorationNode : public rclcpp::Node {
public:
    ExplorationNode() : Node("px4_exploration_node") {
        // params
        this->declare_parameter("sensor_topic", "/Laser_map");  // FastLIO global map
        this->declare_parameter("position_topic", "/Odometry");
        this->declare_parameter("cam_width", 1280);
        this->declare_parameter("cam_height", 720);
        this->declare_parameter("cam_fx", 689.74);
        this->declare_parameter("cam_fy", 690.02);
        this->declare_parameter("cam_cx", 643.12);
        this->declare_parameter("cam_cy", 361.61);
        this->declare_parameter("max_speed", 0.5);
        this->declare_parameter("max_accel", 1.5);
        this->declare_parameter("max_yaw_rate", 2.0);
        this->declare_parameter("lidar_range", 8.0);
        this->declare_parameter("height_band", 0.35);
        // ★规划线程模式: 1=同步(原行为, 算路时主循环会卡住几秒); 0=后台线程规划(不卡)
        //   探索/选目标都跟着这个开关; A→B(走廊)本来就强制后台 ✓
        this->declare_parameter("planner_sync", 1);
        // ★A→B(通道)段专用: 用【无人机自身高度 ± 这个带宽】取点云(不是固定高度范围 ✗)
        //   原因: 通道里 Z 估计会有偏移(命令 0.5 实际快着地) → 固定范围会把点云滤没/滤错
        //   独立参数, 不和探索段的 height_band 共用 ✓
        this->declare_parameter("ab_cloud_h_band", 0.30);
        // ★★ 现场要求(2026-10-17): 点云 z 用【无人机当前高度 ± 上下范围】✗
        //    (雷达 z 会漂 → 用固定高度会越飞越高 ✗ → 改用相对当前高度 ✓)
        //    探索段: 上 +0.10 / 下 -0.40 ✓    A→B段: 上 +0.00 / 下 -0.20 ✓
        this->declare_parameter("explore_cloud_h_up",   0.10);
        this->declare_parameter("explore_cloud_h_down", 0.40);
        this->declare_parameter("ab_cloud_h_up",        0.00);
        this->declare_parameter("ab_cloud_h_down",      0.20);
        // ★返航段固定高度范围点云: 过探索区角点(开始降通道高度)后, 建图不再跟随
        //   无人机当前高度, 而是只用固定高度带内的点 —— 否则降高过程中"高度带"
        //   往下扫, 每帧清掉前向扇区重标, 地图会变得不完整 → 到 A 附近容易失误。
        this->declare_parameter("use_return_cloud", 1);                             // 1=启用上面的切换, 0=始终用无人机当前高度附近
        this->declare_parameter("return_cloud_h_min", 0.30);                        // 固定高度范围下限(世界系, m)
        this->declare_parameter("return_cloud_h_max", 0.60);                        // 固定高度范围上限(世界系, m)
        this->declare_parameter("return_cloud_clear_map", 1);                       // 1=切换瞬间整张障碍图清空重建(只保留新范围的点云)
        // ★虚拟通道右墙(自建场地没有围墙时用): 给定"通道入口处右边墙边缘"的世界系坐标点,
        //   算法从该点沿 -y 一直到地图 y 边界整条线都当障碍物(同时 GUI 会直接显示出来),
        //   这样无人机到左上角后不会从场地外面绕过去飞 B, 只能按通道路径规划。
        this->declare_parameter("use_corridor_wall", 0);                           // 1=启用虚拟右墙(自建场地测试用)
        this->declare_parameter("corridor_wall_x", 0.0);                           // 墙的世界系 x (m)
        this->declare_parameter("corridor_wall_y", 0.0);                           // 墙起始的世界系 y (m), 从这里往 -y 标到地图下边界
        // ★第二道虚拟墙(生成逻辑与第一道完全相同: 从该点往 -y 标到地图下边界)
        this->declare_parameter("use_corridor_wall2", 0);
        this->declare_parameter("corridor_wall2_x", 0.0);
        this->declare_parameter("corridor_wall2_y", 0.0);
        this->declare_parameter("takeoff_height", 1.0);
        this->declare_parameter("fov_max_range", 4.0);
        this->declare_parameter("gui_fov_range", 5.0);  // GUI 绿色扇形射线长度(m), 仅显示用
        this->declare_parameter("vision_topic_prefix", std::string("/detected_object_class")); // ★每类一个话题: <前缀><类别号>
        this->declare_parameter("vision_class_list", std::string("0,1,2,3,5"));              // ★要订阅的类别(今年5个目标; 4号没有目标, 视觉不发)
        this->declare_parameter("vision_hold_secs", 1.5);     // 0.7m 处悬停时长(s)
        this->declare_parameter("vision_reach_tol", 0.10);
        // ★先稳一下再对准: 飞到离目标这么近(0.7m)先悬停 hold 秒稳定, 再取坐标飞中心。
        //   防止前视相机快到位时目标偏到视野边缘 → 投影中心偏 → 基准点偏。
        // ★飞目标→降落前 的流程切换:
        //   0 = 旧流程: 直接飞到目标中心(容差 vision_reach_tol) → 悬停 → 采 drop_align_frames
        //       帧【视觉坐标】中位数=基准点 → 原地下降
        //   1 = 新流程: 先飞到距目标 vision_near_dist(0.7m) → 纯悬停 vision_near_hold_secs(0.5s,
        //       不采样)稳定 → 采 drop_align_frames 帧【视觉坐标】中位数=基准点 → 再飞中心 → 下降
        // ★视觉瞄准点滤波(消除"在目标附近徘徊"): 死区(m, 小于它不动) + 一阶低通系数(0~1)
        // ★基准点来源: 1=用视觉上报坐标中位数(理论"投放口压标靶"); 0=用机位中位数(旧版)
        //   视觉坐标若存在视角相关偏差, 用它会让"偏移点"几乎落在飞机当前位置 → 看着像不飞偏移。
        this->declare_parameter("drop_base_from_vision", 1);
        this->declare_parameter("vision_aim_deadband", 0.0);   // 0=不过滤(旧版行为)
        this->declare_parameter("vision_aim_alpha", 1.0);      // 1.0=不过滤(旧版行为)
        this->declare_parameter("vision_approach_mode", 0);   // 默认0=旧流程(直接飞中心→悬停→采样→降落; 现场验证过的那套)
        // ★无可用路径时的"近距离免规划"阈值(m): 距目标点在这个范围内就直接位置环靠近,
        //   不再反复重规划(否则视觉坐标抖动会导致目标附近剧烈来回)
        this->declare_parameter("vision_nopath_free_m", 0.60);
        // ★进入这个距离后改用速度环, 以 vision_slow_speed 慢慢蹭到中心
        //   (视角稳定→视觉坐标稳定; 也不会冲过头)
        // ★到中心 10cm 内后, 飞"投放口偏移"时的横向速度上限(m/s)
        // ★是否锁定机头: 1=到目标10cm内锁死(默认); 0=旧行为(机头跟路径方向变)
        this->declare_parameter("vision_lock_yaw", 1);
        this->declare_parameter("vision_offset_speed", 0.10);
        // ★★ 高空对准(投放口压目标中心)与低空对准是两套判据/两套超时:
        //   高空: drop_offset_tol(0.06m) + drop_offset_try_secs(超时后不再等, 直接用当前机位
        //         采基准点并下降 —— 低空那套会更严格地对准)
        //   低空: drop_final_tol(0.04m) + drop_try_secs(超时放宽3倍, 再3倍放弃)
        this->declare_parameter("drop_offset_tol", 0.06);
        this->declare_parameter("drop_offset_try_secs", 10.0);
        this->declare_parameter("vision_slow_dist", 0.30);
        this->declare_parameter("vision_near_kff", 1.2);
        this->declare_parameter("vision_near_vmax", 0.50);
        this->declare_parameter("vision_near_vmin", 0.10);
        this->declare_parameter("vision_slow_speed", 0.05);
        this->declare_parameter("vision_near_dist", 0.50);
        this->declare_parameter("vision_near_hold_secs", 0.5);    // 水平到达判定(m)
        this->declare_parameter("vision_timeout", 1.5);
        this->declare_parameter("vision_jump_tol", 1.5);         // ★同一类别相邻帧位置突变超过此值(m)判为坏帧, 丢弃
        // ★目标暂时不在视野(FOV边缘时有时无)时, 继续朝"最后一次收到的位置"飞多久才放弃。
        //   比 vision_timeout 大得多 —— 飞过去往往就重新进视野了; 真丢了才恢复探索。
        this->declare_parameter("vision_lost_secs", 6.0);       // 视觉数据中断超时(s)
        this->declare_parameter("vision_dedup_radius", 1.0);
        // ★★ 原点附近【5号】过滤(现场要求 ✓): 起降点的"H"标志会被视觉误识别成5号 ✗
        //    → 这个半径内(雷达系, 起飞点=原点)的 5 号目标【不飞】✗
        this->declare_parameter("origin_reject_radius", 1.00);  // 已投放点位去重半径(m): 新目标落在该半径内视为同一物体 → 不再重复投放(SLAM漂移+误分类, 0.5太小)
        // ===== 视觉标靶类别策略(顺序 = 优先级) =====
        this->declare_parameter("vision_total", 3);
        // ★★ 飞视觉目标期间【是否继续做 FOV 覆盖标记】✗
        //    1 = 继续标(默认 ✓ 投放完不会再走老路 ✓)  0 = 不标(旧行为 ✗ 效率低 ✓)
        this->declare_parameter("fov_mark_while_vision", 1);           // 总任务: 飞满几个标靶就直接返航(现场只投3个)
        this->declare_parameter("vision_fallback_pct", 80.0); // 探索进度达到该%后, 才允许飞"回退类别"补齐总数
        this->declare_parameter("vision_priority_classes", std::string("0,1,5")); // 优先类别
        this->declare_parameter("vision_fallback_classes", std::string("3,2"));   // 回退类别(补齐用)
        this->declare_parameter("vision_filter_classes",   std::string("4"));     // 直接过滤(现场没有=误识别)
        // ===== 投放(串口 + 三个投放口) =====
        this->declare_parameter("use_serial", 0);                                   // 1=真机: 节点启动即打开串口; 0=仿真: 不连串口, 投放只打日志
        this->declare_parameter("serial_port", std::string("/dev/ttyS3"));          // 串口设备
        this->declare_parameter("serial_baud", 115200);                             // 波特率
        this->declare_parameter("serial_ready_secs", 3.0);                          // 打开后要等多久才能用(真机 MCU 要求3s)
        this->declare_parameter("drop_hold_secs", 1.0);                             // 两次投放指令最短间隔(原 send() 里的 sleep(1))
        this->declare_parameter("bomb_offsets", std::string("0,-0.2;0.2,0;0,0.2")); // 三个投放口相对机体的位置(m): "口0;口1;口2", 每项 "x,y"
        // ★★ 5号(最小标靶)专用投放口位置 ✓: 空字符串=不启用(退回 bomb_offsets) ✓
        //    为什么单独一套: 实测别的目标用 0.2 准 ✓ 只有 5 号要 0.28 才落中心 ✓
        //    → 说明【5号的视觉坐标偏了约 8cm】✗ (5号框最小, 取点偏差最大 ✓)
        //    → 用这个参数补, 而不是改 bomb_offsets(那是口的真实物理位置 ✓ 不能动 ✓)
        this->declare_parameter("bomb_offsets_small", std::string(""));
        this->declare_parameter("bomb_y_sign", 1.0);
        // ★实物投放微调(机体系, m): "口0;口1;口2" 每项 "x,y", 叠加到 bomb_offsets 上
        this->declare_parameter("bomb_trim", std::string("0,0;0,0;0,0"));                                // 机体 y 轴方向: ROS(左正)=+1; 若真机按 FRD(右正)填 -1
        this->declare_parameter("drop_final_tol", 0.03);                            // 【降落后】对准"降落前记录的位置"判定(m): 严格(小标靶调更小)
        this->declare_parameter("drop_final_tol_small", 0.020);                     // ★5号(最小标靶)专用: 比别的类别更严 ✓
        this->declare_parameter("drop_relax_mult_small", 1.5);                      // ★5号专用: 超时放宽倍数(别的类别 3.0) ✓
        this->declare_parameter("drop_height", 0.7);
        // ★最终降落: 到降落点后先在"位置环悬停对准"保持这么久, 再真正调 land 服务降落
        //   (和投放前的 drop_settle_secs 一个思路: 不是一进范围就落, 给对准/稳定留时间)
        this->declare_parameter("land_settle_secs", 2.0);                                // 降落到的高度(世界系, m)
        this->declare_parameter("descent_speed", 0.3);                              // 降落速度(m/s): 别太快, 否则降落后偏得多
        this->declare_parameter("ascend_speed", 0.3);                               // 投放后升回探索高度的速度(m/s); 升到位才算完成本趟
        this->declare_parameter("ascend_hold_secs", 2.0);                           // 升回探索高度后"位置环悬停稳定"时长(s), 之后才恢复探索/飞下一个
        this->declare_parameter("drop_align_frames", 20);                           // 降落前记录对准位置取多少帧的中位数
        this->declare_parameter("drop_try_secs", 10.0);                             // ★只给"低空严格对准"计时(从到达降落高度开始): 超时放宽到3倍容差投放, 再超3倍放弃
        this->declare_parameter("drop_repeat", 4);                                  // 投放时同一口连发次数(防单次没触发)
        this->declare_parameter("drop_repeat_gap_ms", 30);                          // 连发之间的间隔(ms)
        this->declare_parameter("drop_ignore_radius", 1.2);                         // 投放后"飞机还在投放点这么近"时, 视觉数据一律不记录(m)
        this->declare_parameter("drop_ignore_secs", 15.0);                          // 上面这条只在该时长内生效(避免以后路过同一片区域一直不记录)
        this->declare_parameter("drop_vel_tol", 0.05);                              // 投放前速度刹住判定(m/s)
        this->declare_parameter("drop_settle_secs", 0.3);                           // 【低空/投放前】在降落高度上"位置+高度+速度都达标"持续这么久才投放(给无人机这段时间继续对准)
        this->declare_parameter("drop_height_tol", 0.05);
        // ★投放判据可选: 1=只看水平(x,y)对准(连续 drop_xy_hold_ticks 个控制帧就投, 忽略
        //   高度差与刹车速度)。低空投放(0.12m)炸弹近似直线下落时用它更快。
        this->declare_parameter("drop_xy_only", 0);
        this->declare_parameter("drop_xy_hold_ticks", 20);        // 50Hz 控制帧, 20 帧 ≈ 0.4s
        // ★下降末段提前开始横向偏移: 0=关闭(严格"降完再偏移"); >0=距投放高度还剩这么多米时
        //   就开始飞偏移(缓解"实际高度最后几厘米 settle 好几秒、看着原地不动")
        this->declare_parameter("drop_offset_start_dz", 0.0);
        this->declare_parameter("drop_align_kff", 1.0);           // ★低空对准 误差→速度增益(1/s)  (1.0=旧版力度)
        this->declare_parameter("drop_align_vmax", 0.30);         // ★低空对准 横向限速(m/s)  (0.30=旧版)                           // "到降落高度"判定(m): 高度没到位绝不投放
        this->declare_parameter("corridor_height", 0.7);      // 返航角点后的通道飞行高度(世界系, m)
        // ★过角点后"降到通道高度"这一步也判 z(和降落一样必须到位), 但只判一次:
        //   到位/超时都锁存, 之后不再因 z 漂移或高度振荡把航段闸门反复关回来
        //   (以前会在到 padA 后重新卡住 → 不转 -y、不规划走廊、最后原地降落)
        this->declare_parameter("corner_h_tol", 0.10);           // 到位判定(m)
        this->declare_parameter("corner_h_wait_secs", 6.0);      // 最多等多久(s): <0=一直等, 0=不等待
        this->declare_parameter("ab_edge_margin", 0.25);      // A→B段(非探索区)规划边缘距离(m); 非探索区不加膨胀, 只用这个
        // ★通道路径规划方式: 0 = 保守A*(斜线直接穿口中点, 现在的做法)
        //                    1 = 先x再y 轴对齐(每个口中点过 y 后再多走 ab_axis_overshoot)
        this->declare_parameter("ab_path_mode", 0);
        this->declare_parameter("ab_axis_overshoot", 0.7);
        this->declare_parameter("ab_axis_lead", 1.2);   // ★mode1 提前并线距离(m): 离口子这么远就斜着并到中线 ✓
        this->declare_parameter("ab_axis_x_speed", 0.25);          // mode1: 横移(x)段限速(m/s), 否则拐回-y刹不住
        this->declare_parameter("ab_vel_ff", 1.0);            // A→B速度前馈比例(0=只发位置点会很慢, 1=按channel_speed飞)
        this->declare_parameter("fov_h_deg", -1.0);   // 前视相机水平FOV(度); ≤0 时按相机内参自动计算
        this->declare_parameter("fov_tilt_deg", 55.0);// 前视相机光轴下倾角(度,相对水平面)
        this->declare_parameter("explore_threshold", 98.0);
        this->declare_parameter("inflation_radius_dm", 3.0);   // ★障碍物膨胀半径(dm = 0.1m, 与分辨率无关): 3.0 = 0.30m
        this->declare_parameter("distance_threshold", 0.2);
        this->declare_parameter("fov_divider", 5);            // FOV覆盖标记每几拍跑一次(控制50Hz → 5=10Hz; 1=50Hz, 2=25Hz)
        this->declare_parameter("use_gui", 1);
        this->declare_parameter("use_global_cloud", 1);
        this->declare_parameter("cloud_max_hz", 1.0);
        // ★★ 全场累加点云(现场要求 ✓): 1=全场都累加(不再清探索区外 ✗); 0=旧行为(探索区外每帧重建)
        //    注意: 累加后【走过的人/临时物会被永久记住】✗ → 只有确定场地无人穿行时才用 1 ✓
        this->declare_parameter("accumulate_all_cloud", 1);
        this->declare_parameter("cloud_subsample", 8);   // ★建图点云抽样(每N点取1): 膨胀圆盘开销是N²级 → 抽样8倍建图快8倍 ✓; 1=不抽   // 点云处理限频(Hz): /cloud_registered 是10Hz, 每帧建图几十ms会拖死主循环(设定点断流→丢OFFBOARD) ✗ → 限到1Hz ✓; 0=不限   // 1=全局点云, 0=仅前向180°点云
        this->declare_parameter("map_x_min", -1.0);
        this->declare_parameter("map_x_max", 10.0);
        this->declare_parameter("map_y_min", -6.0);
        this->declare_parameter("map_y_max", 6.0);
        this->declare_parameter("margin_x", 0.5);
        this->declare_parameter("margin_y", 0.5);
        this->declare_parameter("edge_margin", 0.7);

        // ★探索策略: 目标点 x >= 该值的一律优先探索; x 更小的(起飞点附近)放最后。设 -999 = 关闭
        this->declare_parameter("explore_far_first_x", 1.0);
        // ★选点安全/绕远: 实际评估前 N 个候选(A*路径长度+沿途离障余量+航向)
        this->declare_parameter("explore_pick_topn", 6);
        this->declare_parameter("explore_safe_clear", 0.40);   // 期望沿途离障余量(m, 仅当 safe_w>0 才起作用)
        this->declare_parameter("explore_safe_w", 0.0);        // ★安全惩罚权重: 0=不用(膨胀+机体半径已保证安全), >0 才额外偏好"离障远"
        this->declare_parameter("extend_ratio", 0.3);  // extend when remaining < ratio × path_len
        this->declare_parameter("replan_interval", 0.5);  // EGO-style replan period (s) while MOVE
        this->declare_parameter("takeoff_speed", 0.5);    // vertical ascent/descent speed (m/s)
        this->declare_parameter("go_home_speed", 2.0);    // cruise speed when flying to HOME (m/s, faster than explore)
        this->declare_parameter("corner_speed", 0.5);     // 探索完→探索区角点的巡航速度(m/s)
        // ★探索完后飞的那个"左上角点"(世界系; -999 = 自动=探索区边界内缩0.5m)
        this->declare_parameter("home_corner_x", -999.0);
        this->declare_parameter("home_corner_y", -999.0);
        // ===== 探索子区(可选) =====
        // 不配置时默认 = 整个 map_* 矩形(保持旧行为)。配置后探索只发生在此
        // 子矩形内(隔墙以东 x>7.5 的降落走廊应排除, 即 explore_x_max ≤ 7.4)。
        this->declare_parameter("explore_x_min", -1.0);
        this->declare_parameter("explore_x_max", 10.0);
        this->declare_parameter("explore_y_min", -6.0);
        this->declare_parameter("explore_y_max", 6.0);
        // ===== 返航/降落模式 =====
        // flight_mode 3 → 探索完/投放完 直接 A* 回**起飞点**原地降落(不进走廊, 通道过不去时最保险)
        // flight_mode 0 → 探索完直飞 pad_a 降落(go_home_speed)
        // flight_mode 1 → 探索完先 go_home_speed 飞到 pad_a(走廊入口),
        //                 再以 channel_speed 沿走廊飞到 pad_b 降落
        this->declare_parameter("flight_mode", 0);
        this->declare_parameter("pad_a_x", 8.5);   // 降落走廊入口/左上pad
        this->declare_parameter("pad_a_y", 4.5);
        this->declare_parameter("pad_b_x", 8.5);   // 真正降落pad(右下/mode1)
        this->declare_parameter("pad_b_y", -4.5);
        this->declare_parameter("channel_speed", 0.5);  // A→B 通道段巡航速度
        // 向下FOV开关参数
        this->declare_parameter("use_downward_fov", 1);   // 1=使用向下FOV, 0=使用前向FOV

        // read
        std::string st = this->get_parameter("sensor_topic").as_string();
        std::string pt = this->get_parameter("position_topic").as_string();
        cam_w_    = this->get_parameter("cam_width").as_int();
        cam_h_    = this->get_parameter("cam_height").as_int();
        cam_fx_   = this->get_parameter("cam_fx").as_double();
        cam_fy_   = this->get_parameter("cam_fy").as_double();
        cam_cx_   = this->get_parameter("cam_cx").as_double();
        cam_cy_   = this->get_parameter("cam_cy").as_double();
        DRONE_SPEED=this->get_parameter("max_speed").as_double();
        MAX_ACCEL  =this->get_parameter("max_accel").as_double();
        MAX_YAW_RATE=this->get_parameter("max_yaw_rate").as_double();
        lidar_range_=this->get_parameter("lidar_range").as_double();
        height_band_=this->get_parameter("height_band").as_double();
        ab_cloud_h_band_ = this->get_parameter("ab_cloud_h_band").as_double();
        explore_cloud_h_up_   = this->get_parameter("explore_cloud_h_up").as_double();
        explore_cloud_h_down_ = this->get_parameter("explore_cloud_h_down").as_double();
        ab_cloud_h_up_        = this->get_parameter("ab_cloud_h_up").as_double();
        ab_cloud_h_down_      = this->get_parameter("ab_cloud_h_down").as_double();
        use_return_cloud_   = this->get_parameter("use_return_cloud").as_int() != 0;
        return_cloud_h_min_ = this->get_parameter("return_cloud_h_min").as_double();
        return_cloud_h_max_ = this->get_parameter("return_cloud_h_max").as_double();
        return_cloud_clear_map_ = this->get_parameter("return_cloud_clear_map").as_int() != 0;
        use_corridor_wall_  = this->get_parameter("use_corridor_wall").as_int() != 0;
        corridor_wall_x_    = this->get_parameter("corridor_wall_x").as_double();
        corridor_wall_y_    = this->get_parameter("corridor_wall_y").as_double();
        use_corridor_wall2_ = this->get_parameter("use_corridor_wall2").as_int() != 0;
        corridor_wall2_x_   = this->get_parameter("corridor_wall2_x").as_double();
        corridor_wall2_y_   = this->get_parameter("corridor_wall2_y").as_double();
        hgit_       =this->get_parameter("takeoff_height").as_double();
        fov_max_range_=this->get_parameter("fov_max_range").as_double();
        gui_fov_range_ =this->get_parameter("gui_fov_range").as_double();
        vision_topic_prefix_ = this->get_parameter("vision_topic_prefix").as_string();
        vis_class_list_      = parseClassList(this->get_parameter("vision_class_list").as_string());
        vision_hold_secs_  = this->get_parameter("vision_hold_secs").as_double();
        vision_reach_tol_  = this->get_parameter("vision_reach_tol").as_double();
        vision_timeout_    = this->get_parameter("vision_timeout").as_double();
        vision_dedup_r_    = this->get_parameter("vision_dedup_radius").as_double();
        origin_reject_radius_ = this->get_parameter("origin_reject_radius").as_double();
        vision_total_       = this->get_parameter("vision_total").as_int();
        fov_mark_while_vision_ = this->get_parameter("fov_mark_while_vision").as_int() != 0;
        vision_fallback_pct_= this->get_parameter("vision_fallback_pct").as_double();
        vis_priority_ = parseClassList(this->get_parameter("vision_priority_classes").as_string());
        vis_fallback_ = parseClassList(this->get_parameter("vision_fallback_classes").as_string());
        vis_filter_   = parseClassList(this->get_parameter("vision_filter_classes").as_string());
        // ===== 投放(串口) =====
        use_serial_       = this->get_parameter("use_serial").as_int();
        serial_port_      = this->get_parameter("serial_port").as_string();
        serial_baud_      = this->get_parameter("serial_baud").as_int();
        serial_ready_     = this->get_parameter("serial_ready_secs").as_double();
        drop_hold_secs_   = this->get_parameter("drop_hold_secs").as_double();
        bomb_offsets_     = parseBombOffsets(this->get_parameter("bomb_offsets").as_string());
        {   // ★5号专用偏移: 非空才启用 ✓
            std::string ssm = this->get_parameter("bomb_offsets_small").as_string();
            bomb_offsets_small_ = parseBombOffsets(ssm);
            bomb_small_enabled_ = (ssm.find_first_not_of(" \t\r\n") != std::string::npos);
            if (bomb_small_enabled_)
                RCLCPP_INFO(this->get_logger(), "★5号专用投放口偏移已启用: %s", ssm.c_str());
        }
        bomb_y_sign_      = this->get_parameter("bomb_y_sign").as_double();
        drop_final_tol_   = this->get_parameter("drop_final_tol").as_double();
        drop_final_tol_small_  = this->get_parameter("drop_final_tol_small").as_double();
        drop_relax_mult_small_ = this->get_parameter("drop_relax_mult_small").as_double();
        drop_height_      = this->get_parameter("drop_height").as_double();
        land_settle_secs_ = this->get_parameter("land_settle_secs").as_double();
        descent_speed_    = this->get_parameter("descent_speed").as_double();
        ascend_speed_     = this->get_parameter("ascend_speed").as_double();
        ascend_hold_secs_ = this->get_parameter("ascend_hold_secs").as_double();
        drop_align_frames_= this->get_parameter("drop_align_frames").as_int();
        drop_try_secs_    = this->get_parameter("drop_try_secs").as_double();
        drop_repeat_      = this->get_parameter("drop_repeat").as_int();
        drop_repeat_gap_ms_ = this->get_parameter("drop_repeat_gap_ms").as_int();
        drop_ignore_radius_ = this->get_parameter("drop_ignore_radius").as_double();
        drop_ignore_secs_   = this->get_parameter("drop_ignore_secs").as_double();
        drop_vel_tol_     = this->get_parameter("drop_vel_tol").as_double();
        drop_settle_secs_ = this->get_parameter("drop_settle_secs").as_double();
        drop_height_tol_  = this->get_parameter("drop_height_tol").as_double();
        drop_xy_only_     = this->get_parameter("drop_xy_only").as_int() != 0;
        drop_xy_hold_ticks_ = (int)this->get_parameter("drop_xy_hold_ticks").as_int();
        drop_offset_start_dz_ = this->get_parameter("drop_offset_start_dz").as_double();
        drop_align_kff_   = this->get_parameter("drop_align_kff").as_double();
        drop_align_vmax_  = this->get_parameter("drop_align_vmax").as_double();
        if (drop_align_kff_  < 0.1) drop_align_kff_  = 0.1;
        if (drop_align_vmax_ < 0.05) drop_align_vmax_ = 0.05;
        if (drop_xy_hold_ticks_ < 1) drop_xy_hold_ticks_ = 1;
        // 真机: 节点启动就打开串口(打开后需等 serial_ready_secs 才能用, 见 Arduino::send)
        Arduino::configure(serial_port_, serial_baud_, use_serial_ != 0,
                           serial_ready_, drop_hold_secs_);
        Arduino::serinit();
        corridor_height_   = this->get_parameter("corridor_height").as_double();
        corner_h_tol_      = this->get_parameter("corner_h_tol").as_double();
        corner_h_wait_secs_= this->get_parameter("corner_h_wait_secs").as_double();
        AB_EDGE_MARGIN     = this->get_parameter("ab_edge_margin").as_double();
        ab_path_mode_      = this->get_parameter("ab_path_mode").as_int();
        ab_axis_overshoot_ = this->get_parameter("ab_axis_overshoot").as_double();
        ab_axis_lead_      = this->get_parameter("ab_axis_lead").as_double();
        ab_axis_x_speed_   = this->get_parameter("ab_axis_x_speed").as_double();
        // ★边缘距离在规划里是"整格量化 + 以格为单位的圆盘判定" → 实际生效值 ≈ floor(v/GRID_RES)*GRID_RES。
        //   换分辨率时同一个参数值会变松/变紧, 所以这里把实际值打出来(便于现场调通道窄口)。
        {
            auto qcells = [](double v){ return (int)std::floor(v / GRID_RES + 1e-9); };
            RCLCPP_INFO(this->get_logger(),
                "边缘距离量化(栅格 %.3f m): ab_edge_margin %.2f → 实际≈%.2f m (%d格) | edge_margin %.2f → 实际≈%.2f m (%d格)",
                GRID_RES, AB_EDGE_MARGIN, qcells(AB_EDGE_MARGIN)*GRID_RES, qcells(AB_EDGE_MARGIN),
                EDGE_MARGIN, qcells(EDGE_MARGIN)*GRID_RES, qcells(EDGE_MARGIN));
        }
        planner_.setStage1VelFF(this->get_parameter("ab_vel_ff").as_double());
        fov_h_deg_=this->get_parameter("fov_h_deg").as_double();
        fov_tilt_deg_=this->get_parameter("fov_tilt_deg").as_double();
        planner_.home_pct_=this->get_parameter("explore_threshold").as_double();
        planner_.waypoint_tol_=this->get_parameter("distance_threshold").as_double();
        {   // ★膨胀半径按 dm 给出, 内部换算成"栅格数"(分辨率改了物理距离不变)
            const double infl_dm = this->get_parameter("inflation_radius_dm").as_double();
            INFLATE_RADIUS = std::max(0, (int)std::lround(infl_dm * 0.1 / GRID_RES));
            RCLCPP_INFO(this->get_logger(),
                "膨胀半径 %.1f dm = %.3f m → %d 格 (GRID_RES=%.3f m/格)",
                infl_dm, infl_dm * 0.1, INFLATE_RADIUS, GRID_RES);
        }
        // Launch values are WORLD coordinates. They are stored raw in the w_*
        // members and get +origin applied later (see applyOriginShift) so they
        // line up with the radar-frame odom / point cloud.
        w_map_x_min_ = this->get_parameter("map_x_min").as_double();
        w_map_x_max_ = this->get_parameter("map_x_max").as_double();
        w_map_y_min_ = this->get_parameter("map_y_min").as_double();
        w_map_y_max_ = this->get_parameter("map_y_max").as_double();
        w_explore_x_min_ = this->get_parameter("explore_x_min").as_double();
        w_explore_x_max_ = this->get_parameter("explore_x_max").as_double();
        w_explore_y_min_ = this->get_parameter("explore_y_min").as_double();
        w_explore_y_max_ = this->get_parameter("explore_y_max").as_double();
        w_flight_mode_   = this->get_parameter("flight_mode").as_int();
        w_pad_a_x_ = this->get_parameter("pad_a_x").as_double();
        w_pad_a_y_ = this->get_parameter("pad_a_y").as_double();
        w_pad_b_x_ = this->get_parameter("pad_b_x").as_double();
        w_pad_b_y_ = this->get_parameter("pad_b_y").as_double();
        w_channel_speed_ = this->get_parameter("channel_speed").as_double();

        // Until origin_ is known we assume origin=(0,0) → launch values are the
        // working values too (single-frame / origin≈0 case stays correct).
        MAP_X_MIN=w_map_x_min_; MAP_X_MAX=w_map_x_max_;
        MAP_Y_MIN=w_map_y_min_; MAP_Y_MAX=w_map_y_max_;
        map_x_min_=MAP_X_MIN; map_x_max_=MAP_X_MAX;
        map_y_min_=MAP_Y_MIN; map_y_max_=MAP_Y_MAX;
        marg_x_=this->get_parameter("margin_x").as_double();
        marg_y_=this->get_parameter("margin_y").as_double();
        EDGE_MARGIN=this->get_parameter("edge_margin").as_double();

        {   // ★探索策略: x>=阈值的目标优先(起飞点附近 x 小的放最后)
            const double ffx = this->get_parameter("explore_far_first_x").as_double();
            planner_.setFarFirstX(ffx);
            planner_.setPickSafety((int)this->get_parameter("explore_pick_topn").as_int(),
                                   this->get_parameter("explore_safe_clear").as_double(),
                                   this->get_parameter("explore_safe_w").as_double());
            RCLCPP_INFO(this->get_logger(),
                "探索策略: x >= %.2f 的目标优先探索, 更小的(起飞点附近)放最后%s",
                ffx, ffx < -100.0 ? " (已关闭)" : "");
        }
        planner_.extend_ratio_=this->get_parameter("extend_ratio").as_double();
        planner_.replan_interval_=this->get_parameter("replan_interval").as_double();
        takeoff_speed_ = this->get_parameter("takeoff_speed").as_double();
        planner_.go_home_speed_ = this->get_parameter("go_home_speed").as_double();
        planner_.corner_speed_  = this->get_parameter("corner_speed").as_double();
        {
            const double hcx = this->get_parameter("home_corner_x").as_double();
            const double hcy = this->get_parameter("home_corner_y").as_double();
            // 世界系 → odom系(+origin); -999 保持自动
            const double ox = (hcx < -900.0) ? -999.0 : (hcx + origin_x_);
            const double oy = (hcy < -900.0) ? -999.0 : (hcy + origin_y_);
            planner_.setHomeCorner(ox, oy);
            RCLCPP_INFO(this->get_logger(),
                "返航角点: %s  世界=(%.2f,%.2f) → odom=(%.2f,%.2f)",
                (hcx < -900.0 || hcy < -900.0) ? "自动(探索区边界内缩0.5m)" : "launch 指定",
                hcx, hcy, ox, oy);
        }

        // 探索子区(可选): 未配置(值等于全图边界)时默认为整个 map 矩形。
        EXPLORE_X_MIN = std::max(w_explore_x_min_, MAP_X_MIN);
        EXPLORE_X_MAX = std::min(w_explore_x_max_, MAP_X_MAX);
        EXPLORE_Y_MIN = std::max(w_explore_y_min_, MAP_Y_MIN);
        EXPLORE_Y_MAX = std::min(w_explore_y_max_, MAP_Y_MAX);

        // 返航/降落配置 (world frame; shift applied once origin_ is known)
        if (w_flight_mode_ == 3)   // mode3: 起飞点 = 世界原点(0,0), 等 origin_ 到了再设一次
            planner_.setReturnConfig(0.0, 0.0, 0.0, 0.0, w_flight_mode_, w_channel_speed_);
        else
            planner_.setReturnConfig(w_pad_a_x_, w_pad_a_y_, w_pad_b_x_, w_pad_b_y_,
                                     w_flight_mode_, w_channel_speed_);

        RCLCPP_INFO(this->get_logger(),
            "Return mode=%d padA=(%.1f,%.1f) padB=(%.1f,%.1f) ch=%.2f | Explore x:[%.1f,%.1f] y:[%.1f,%.1f]",
            w_flight_mode_, w_pad_a_x_, w_pad_a_y_, w_pad_b_x_, w_pad_b_y_, w_channel_speed_,
            EXPLORE_X_MIN, EXPLORE_X_MAX, EXPLORE_Y_MIN, EXPLORE_Y_MAX);

        // 读取FOV模式选择
        use_downward_fov_ = this->get_parameter("use_downward_fov").as_int() != 0;

        fov_divider_ = std::max(1, (int)this->get_parameter("fov_divider").as_int());
        use_gui_=this->get_parameter("use_gui").as_int() != 0;
        use_global_cloud_=this->get_parameter("use_global_cloud").as_int() != 0;
        cloud_max_hz_ = this->get_parameter("cloud_max_hz").as_double();
        cloud_subsample_ = std::max(1, (int)this->get_parameter("cloud_subsample").as_int());
        acc_all_cloud_   = this->get_parameter("accumulate_all_cloud").as_int() != 0;
        if(use_gui_){
            gui_=std::make_unique<GuiRenderer>(map_x_min_,map_x_max_,map_y_min_,map_y_max_);
            // GUI 前向扇形用 launch 配置的水平FOV; 未配置(≤0)时按内参推算
            double hfov_angle = (fov_h_deg_ > 0.0)
                ? fov_h_deg_ * M_PI / 180.0
                : 2.0 * std::atan(cam_w_ / (2.0 * cam_fx_));
            // 绿色扇形射线长度: 仅 GUI 显示用(gui_fov_range), 方便观察前方哪些
            // 射线能穿过窄口、哪些被挡。真实探索覆盖区域仍按 fov_max_range_
            // (内参+安装角推算)标记, 两者互不影响。
            gui_->setFovParams(hfov_angle, gui_fov_range_);
            // Drone marker & frame live in the shared world frame.
            gui_->start(&planner_, &gui_drone_x_, &gui_drone_y_, &odom_yaw_, &gui_mutex_);
            RCLCPP_INFO(this->get_logger(),"GUI started");
        }

        auto qos=rclcpp::SensorDataQoS();
        state_sub_=this->create_subscription<mavros_msgs::msg::State>("/mavros/state",qos,std::bind(&ExplorationNode::state_cb,this,std::placeholders::_1));
        odom_sub_=this->create_subscription<nav_msgs::msg::Odometry>(pt,qos,std::bind(&ExplorationNode::odom_cb,this,std::placeholders::_1));
        cloud_sub_=this->create_subscription<sensor_msgs::msg::PointCloud2>(st,qos,std::bind(&ExplorationNode::cloud_cb,this,std::placeholders::_1));
        pos_pub_=this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local",10);
        // 视觉目标: 一个类别一个话题 <prefix><类别号>, 消息 geometry_msgs/Point
        //   x,y = 目标全局坐标(雷达系), z = 类别号(冗余, 以话题为准)
        //   5Hz/类, 仅探索阶段使用; 4号(现场没有该目标)视觉端根本不发
        for (int c : vis_class_list_) {
            const std::string tp = vision_topic_prefix_ + std::to_string(c);
            vision_subs_.push_back(this->create_subscription<geometry_msgs::msg::Point>(
                tp, qos,
                [this, c](const geometry_msgs::msg::Point::SharedPtr m){ vision_cb(m, c); }));
            RCLCPP_INFO(this->get_logger(), "视觉订阅: %s (类别%d)", tp.c_str(), c);
        }
        arming_client_=this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_=this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        landing_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

        // 打印当前使用的FOV模式
        RCLCPP_INFO(this->get_logger(),"FOV Mode: %s", use_downward_fov_ ? "DOWNWARD (ground rectangle)" : "FORWARD (sector)");
        RCLCPP_INFO(this->get_logger(),
            "[VIS] 飞目标流程 = %s | 基准点=视觉坐标中位数(drop_align_frames=%d帧) | 稳定悬停=%.1fs | 预稳定距离=%.2fm",
            vision_approach_mode_ == 1 ? "新模式(0.7m先纯悬停稳定→采样→再飞中心)"
                                       : "旧模式(直接飞中心→悬停→采样)",
            (int)this->get_parameter("drop_align_frames").as_int(),
            vision_near_hold_secs_, vision_near_dist_);

        // Log camera ground footprint
        double H = hgit_ > 0.3 ? hgit_ : 0.3;
        double fov_w = (cam_w_ / cam_fx_) * H;
        double fov_h = (cam_h_ / cam_fy_) * H;
        RCLCPP_INFO(this->get_logger(),"Ready. FOV=%.1fx%.1fm on ground H=%.1fm LaserMap=%s",
            fov_w,fov_h,hgit_,st.c_str());
    }

    void run(){
        rclcpp::Rate rate(50);          // 控制主循环 50Hz
        // 50Hz 独立设定点发布线程: 从连接阶段就持续发(进入 OFFBOARD 前的
        // 心跳/后续所有指令都经它发布), 主循环再慢也不会掉 OFFBOARD。
        startSpThread();
        // connect
        while(rclcpp::ok()&&!state_.connected){publishHeartbeat();rclcpp::spin_some(this->get_node_base_interface());rate.sleep();}
        RCLCPP_INFO(this->get_logger(),"Connected");
        // wait for odometry origin baseline (average of first 10 frames)
        while(rclcpp::ok()&&!origin_ready_){rclcpp::spin_some(this->get_node_base_interface());rate.sleep();}
        for(int i=0;i<20;i++){publishHeartbeat();rclcpp::spin_some(this->get_node_base_interface());rate.sleep();}
        // OFFBOARD
        while(rclcpp::ok()&&state_.mode!="OFFBOARD"){publishHeartbeat();rclcpp::spin_some(this->get_node_base_interface());rate.sleep();}
        RCLCPP_INFO(this->get_logger(),"OFFBOARD");
        // ★★ 一收到 OFFBOARD 就【立刻】解锁(不再等 2 秒), 并且:
        //    ① 解锁请求每 1 秒重发一次(第一次可能被飞控丢弃/没收到 ✓)
        //    ② 期间持续发心跳(维持 OFFBOARD) + 持续取状态 ✓
        //       → 如果程序解锁被飞控忽略, 你【用遥控器解锁】这里也能立刻读到 armed=true 放行 ✓✓
        auto arm=std::make_shared<mavros_msgs::srv::CommandBool::Request>();arm->value=true;
        {
            int  arm_tries = 0;
            auto last_arm  = this->now();
            while(rclcpp::ok() && !state_.armed){
                if(arm_tries==0 || (this->now()-last_arm).seconds()>=1.0){
                    if(arm_tries<60){                       // 最多重发 60 次(约 60s), 之后只等遥控器 ✓
                        arming_client_->async_send_request(arm);
                        ++arm_tries;
                        RCLCPP_INFO(this->get_logger(),"[ARM] 解锁请求 #%d (等待中 — 也可用遥控器直接解锁 ✓)",arm_tries);
                    }
                    last_arm=this->now();
                }
                publishHeartbeat();
                rclcpp::spin_some(this->get_node_base_interface());
                rate.sleep();
            }
        }
        // armed 后再稳 0.5s(等状态收敛) 就起飞 ✓
        auto t1=this->now();while((this->now()-t1).seconds()<0.5){publishHeartbeat();rclcpp::spin_some(this->get_node_base_interface());rate.sleep();}
        RCLCPP_INFO(this->get_logger(),"Armed");
        // takeoff + build map.
        // /Odometry 本来就是雷达系坐标, 不加 origin; 只有 launch 里的场地/高度参数
        // 才 +origin 换算到雷达系 → 位置环目标 z = 世界高度 + origin_z。
        RCLCPP_INFO(this->get_logger(),"Takeoff to %.1fm... (origin=(%.3f,%.3f,%.3f))",hgit_,origin_x_,origin_y_,origin_z_);
        const double hk=0.7, hz=takeoff_speed_;
        const double target_z = origin_z_ + hgit_;      // 世界高度 hgit → 雷达系 z
        // 起飞爬升阶段不更新地图/障碍(机还在垂直爬升, 点云视角不稳定):
        // 起飞完成进入悬停后才开始 processLaserMap 建图。
        while(rclcpp::ok()&&odom_z_<target_z-0.1){      // odom_z_ 已是雷达系, 不再 +origin
            if (posctlActive()) {          // 飞手接管 → 停 offboard 流后直接退出程序
                RCLCPP_WARN(this->get_logger(),
                    "POSCTL during takeoff climb → stop offboard & EXIT (no land; pilot has control)");
                stopSpThread();
                return;
            }
            double cz = odom_z_;                    // 当前雷达系高度(不加 origin)
            double vz=(cz<target_z-hgit_*(1.0-hk))?hz:(target_z-cz)*hz/(1.0-hk)/hgit_;
            // 水平修正: 拉回起飞点正上方。起飞点在雷达系的位置 = origin_(起飞前10帧均值),
            // 当前点 = odom_, 所以偏差 = origin_ - odom_ (odom 本身是雷达系, 不再加 origin)
            double vx = (origin_x_ - odom_x_) * 2.5;
            double vy = (origin_y_ - odom_y_) * 2.5;
            flyVel(vx, vy, vz, 0);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
        // 起飞完成后悬停 2 秒 —— 速度悬停(参考之前可正常工作的实现):
        // 循环内保持 OFFBOARD 检查(掉线立即中止),flyVel 把飞机拉回原点
        // 并保持高度,每拍下发+建图+50Hz sleep,计时到 2.0s 才退出。
        RCLCPP_INFO(this->get_logger(),"Takeoff done. Hovering for 2.0s to stabilize (velocity hold)...");
        // ★★ 现场要求: 悬停【一开始】就建图(悬停就是用来稳定+等裁判离场的 ✓)
        //    此前(起飞过程)不使用点云 ✓
        map_started_ = true;
        RCLCPP_INFO(this->get_logger(),"[MAP] 悬停开始 → 开始使用点云建图 ✓");
        auto hover_start=this->now();
        double hover_secs=0.0; int hover_last_s=-1;
        while(rclcpp::ok() && (this->now()-hover_start).seconds() < 2.0){
            if(state_.mode != "OFFBOARD"){
                RCLCPP_WARN(this->get_logger(),"Exited OFFBOARD mode during hover! Aborting...");
                if (posctlActive()) {      // 切到 POSCTL → 停 offboard 流后退出程序, 不降落
                    RCLCPP_WARN(this->get_logger(),
                        "POSCTL during hover → stop offboard & EXIT (no land)");
                    stopSpThread();
                }
                return;
            }
            // 速度悬停: 高度用 0.3 分段控高公式 + 水平拉回原点
            double cz = odom_z_;                        // 当前雷达系高度(不加 origin)
            double vz = (target_z - cz) * 0.3 / (0.3 * hgit_);
            double vx = (origin_x_ - odom_x_) * 3.0;   // 偏差 = 起飞点(origin_) - 当前点(odom_)
            double vy = (origin_y_ - odom_y_) * 3.0;
            flyVel(vx, vy, vz, 0);
            rclcpp::spin_some(this->get_node_base_interface());
            processLaserMap();   // 悬停期间继续建图(点云1Hz / FOV覆盖每拍)
            rate.sleep();
            hover_secs=(this->now()-hover_start).seconds();
            int hs=(int)hover_secs;
            if(hs!=hover_last_s){ hover_last_s=hs;
                RCLCPP_INFO(this->get_logger(),"[hover] stabilizing... %ds / 2s", hs); }
        }
        hover_secs=(this->now()-hover_start).seconds();
        RCLCPP_INFO(this->get_logger(),"Hover complete after %.2fs. Exploring...", hover_secs);
        planner_.triggerStart();
        auto last=this->now();
        while(rclcpp::ok()&&!planner_.finished()){
            // 飞手把模式切到 POSCTL → 停止 offboard 流并直接退出程序(不降落,
            // PX4 已由飞手接管; 也不尝试抢回 OFFBOARD, 抢模式会和飞手打架)。
            if (posctlActive()) {
                RCLCPP_WARN(this->get_logger(),
                    "POSCTL detected → stop offboard & EXIT program (no land) at (%.2f,%.2f,%.2f)",
                    odom_x_, odom_y_, odom_z_);
                break;                     // 跳出主循环 → 收尾处直接退出
            }
            auto now=this->now();
            double dt=(now-last).seconds();
            if(dt<=0)dt=0.01; if(dt>0.1)dt=0.1; last=now;
            Px4ExplorationPlanner::Cmd cmd;
            bool vis_active = false;
            bool alt_hold = false;      // 过角点后正在降到通道高度(降高期间先不推进航段)
            double world_h = hgit_;     // 当前使用的高度(世界系)
            double algo_ms = 0.0;      // 算法迭代耗时(建图+视觉/规划, ms)
            {std::lock_guard<std::mutex> lk(gui_mutex_);
                auto ta = this->now();
                // 建图只在"收到新点云"时做一次: FAST_LIO 全局点云是 1Hz, 没必要
                // 跟着 50Hz 控制周期反复跑(障碍 + FOV覆盖 + 自由空间一起更新)。
                // 点云频率变了(比如 10Hz)会自动适应, 因为这里看的是"新点云到达"。
                // 只有"任务全部飞完后的最后一次降落"不建图 —— 那次 land() 在主循环
                // 退出后执行, 循环已经不在跑; 任务中间的所有降落照常建图。
                // 每拍都调: 函数内部把"点云→障碍图"(1Hz)和"FOV覆盖"(每拍)分开处理
                processLaserMap();
                // 返航已过探索区角点 → 位置环高度改用通道高度(探索区 hgit_ ↔ 通道 corridor_height)。
                // ★这一步(降到通道高度)要判 z —— 但它只判"一次": 到位(或等待超时)就锁存,
                //   之后不再因为 SLAM 的 z 漂移/高度振荡把闸门重新关回来。否则会出现
                //   "到 padA 后闸门又关了 → 不转 -y、不规划走廊、一直悬停、最后原地降落"。
                if (planner_.state() == Px4ExplorationPlanner::GO_HOME
                    && planner_.cornerDone()) {
                    world_h = corridor_height_;
                    if (!corner_h_done_) {
                        const double dz = std::fabs(odom_z_ - (corridor_height_ + origin_z_));
                        if (dz > corner_h_tol_) {
                            alt_hold = true;                 // 原地降高, 暂不推进航段
                            alt_hold_t_ += dt;
                            if (corner_h_wait_secs_ >= 0.0 && alt_hold_t_ > corner_h_wait_secs_
                                && corner_h_wait_secs_ > 0.0) {
                                RCLCPP_WARN(this->get_logger(),
                                    "[corner-h] 通道高度 %.2fm 等 %.1fs 仍未到位(差%.3fm) → 放弃等待, 继续航段",
                                    corridor_height_, alt_hold_t_, dz);
                                corner_h_done_ = true;
                                alt_hold = false;
                            } else if (++alt_hold_log_ % 50 == 1) {
                                RCLCPP_INFO(this->get_logger(),
                                    "[corner-h] 降到通道高度中: z=%.2f 目标=%.2f 差%.3fm 已等 %.1fs",
                                    odom_z_, corridor_height_ + origin_z_, dz, alt_hold_t_);
                            }
                        } else {
                            alt_hold_t_ = 0.0;
                            corner_h_done_ = true;           // ★到位 → 锁存, 后面不再卡航段
                            RCLCPP_INFO(this->get_logger(),
                                "[corner-h] 已到通道高度 %.2fm (z=%.2f) → 继续航段",
                                corridor_height_, odom_z_);
                        }
                    }
                } else {
                    alt_hold_t_ = 0.0;
                    corner_h_done_ = false;                  // 离开该阶段 → 复位(下次返航再判)
                }

                // ★A→B(通道)点云模式切换: 一旦"过探索区角点"(开始降通道高度):
                //   建图改用【无人机自身高度 ± ab_cloud_h_band】的点云带(跟随高度, 不用固定范围 ✗),
                //   并可整图清空重建 —— 通道里 Z 会有偏移, 固定范围会把点云滤没/滤错。
                //   控制开关只有 use_return_cloud(要不要切这个模式) + return_cloud_clear_map(要不要清图) ✓
                {
                    const bool want_fixed = use_return_cloud_
                        && (planner_.state() == Px4ExplorationPlanner::GO_HOME)
                        && planner_.cornerDone();
                    if (want_fixed && !fixed_cloud_on_) {
                        RCLCPP_INFO(this->get_logger(),
                            "过角点 → 通道模式: 点云改用【无人机高度±%.2fm】(跟随高度)%s",
                            ab_cloud_h_band_,
                            return_cloud_clear_map_ ? " + 障碍图清空重建(探索区内累加保留 ✓)" : "");
                        if (return_cloud_clear_map_) {
                            // ★★ 探索区内【累加】(见过就留) / 探索区外(通道)每帧重建 ✓ 现场要求 ✓
                if (!acc_all_cloud_) planner_.clearObstaclesOutsideExplore();  // ★全场累加时不清理 ✓
                            applyVirtualWall();          // 清图后立刻补回虚拟墙
                        }
                    }
                    fixed_cloud_on_ = want_fixed;
                }
                // ★每拍先取后台规划结果(视觉任务期间 planner_.update 不跑, 这里必须收)
                planner_.setVisionActive(vis_target_id_ >= 0);   // 任务结束→丢弃迟到的旧路径
                planner_.pollBgPlan();
                // 视觉目标任务优先接管控制; 没有视觉任务时才跑探索状态机
                vis_active = visionStep(dt, cmd);
                // 标靶没飞满且还有可飞目标(或进度还没到回退阈值) → 暂缓探索器的
                // "覆盖率达标自动返航", 先把视觉目标处理完; 飞满 vision_total 个后
                // visionStep 会直接 forceGoHome()。
                planner_.setHoldHome(visionHoldHome());
                if (!vis_active && !alt_hold) {
                    // 雷达 SLAM 系(原始 odom): 点云/无人机都不加 origin。
                    // 位置环高度 = 世界高度 + origin(世界1.2, origin_z=-0.5 → 0.7)
                    cmd=planner_.update(dt,odom_x_,odom_y_,odom_z_,odom_yaw_,
                                        odom_vx_,odom_vy_,odom_vz_,odom_vyaw_,
                                        world_h + origin_z_);
                }
                algo_ms = (this->now() - ta).seconds() * 1000.0;
            }
            if (alt_hold) {
                // ★过角点降高度期间【同时把机头转到 -y(走廊方向)】—— 在开阔的角点处转完,
                //   避免进通道后才转身: 通道里受气流影响, 转身不稳/容易撞墙。
                flyTraj(odom_x_, odom_y_, world_h + origin_z_, 0, 0, 0, -M_PI / 2.0, dt);
            } else if (vis_active) {
                // cmd 已由 visionStep 直接通过 flyTraj 下发(含高度 0.7/1.2 切换),
                // 这里不再重复下发, 避免覆盖视觉任务的高度指令。
            } else if (cmd.valid) {
                // ★★ 通用防越界: 把指令位置夹在【地图范围内】(留 0.10m 边)
                //    原因: A* 贴边 + B样条平滑会往外鼓 → 指令自己就飞出地图 ✗
                //    (实测返航去角点时指令鼓到 y=3.99, 飞机跟着冲到 y=4.255 ✗)
                const double mx0 = map_x_min_ + 0.10, mx1 = map_x_max_ - 0.10;
                const double my0 = map_y_min_ + 0.10, my1 = map_y_max_ - 0.10;
                if (cmd.pos_x < mx0) cmd.pos_x = mx0;
                if (cmd.pos_x > mx1) cmd.pos_x = mx1;
                if (cmd.pos_y < my0) cmd.pos_y = my0;
                if (cmd.pos_y > my1) cmd.pos_y = my1;
                flyTraj(cmd.pos_x, cmd.pos_y, world_h + origin_z_,
                        cmd.vel_x, cmd.vel_y, cmd.vel_z, cmd.yaw, dt,
                        cmd.acc_x, cmd.acc_y);   // ★加速度前馈(惯性补偿 ✓)
            }
            else
                // Hover: hold current pose in position mode (px=current, 0 vel).
                flyTraj(odom_x_, odom_y_, world_h + origin_z_,
                        0, 0, 0, navYaw(), dt);
            // OFFBOARD 掉线监控: 尝试重新进入 OFFBOARD(最多 5 次, 每 1s 一次),
            // 每次打印 PX4 返回结果(success/result)以定位拒绝原因:
            // 常见 = 位置估计失效(EV 中断)、RC/GCS failsafe、已落地 disarm。
            if (state_.mode != "OFFBOARD") {
                auto now3 = this->now();
                if (!last_offboard_try_.nanoseconds() ||
                    (now3 - last_offboard_try_).seconds() > 1.0) {
                    last_offboard_try_ = now3;
                    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    req->custom_mode = "OFFBOARD";
                    auto fut = set_mode_client_->async_send_request(req);
                    auto rc  = rclcpp::spin_until_future_complete(
                                   this->get_node_base_interface(), fut,
                                   std::chrono::milliseconds(500));
                    bool ok = (rc == rclcpp::FutureReturnCode::SUCCESS) && fut.get()->mode_sent;
                    ++offboard_retry_;
                    RCLCPP_WARN(this->get_logger(),
                        "!! OFFBOARD lost (mode=%s) armed=%d at (%.2f,%.2f,%.2f) — re-enter try %d/5 → %s",
                        state_.mode.c_str(), (int)state_.armed,
                        odom_x_, odom_y_, odom_z_,
                        offboard_retry_, ok ? "accepted" : "REJECTED");
                    if (offboard_retry_ >= 5) {
                        RCLCPP_ERROR(this->get_logger(),
                            "OFFBOARD re-enter failed 5x — giving up, landing");
                        break;
                    }
                }
            } else {
                offboard_retry_ = 0;   // 已回到 OFFBOARD, 复位重试计数
            }
            if(++report_%50==0&&!planner_.finished()){   // 50Hz → 每1秒打印一次
                auto nowl=this->now();
                double dwin = map_t_.nanoseconds()? (nowl-map_t_).seconds() : 0.0;
                double map_hz = (dwin>1e-6)? map_cnt_/dwin : 0.0;
                double fov_hz = (dwin>1e-6)? fov_cnt_/dwin : 0.0;
                map_cnt_=0; fov_cnt_=0; map_t_=nowl;
                RCLCPP_INFO(this->get_logger(),
                    "Explored:%.1f%% Ftrs:%zu St:%d Pos:(%.2f,%.2f,%.2f) | ctrl: iter=%.1fms(%.1fHz) algo=%.1fms map(点云)=%.1fHz fov=%.1fHz",
                    planner_.exploredPct(),planner_.frontiers().size(),(int)planner_.state(),
                    odom_x_,odom_y_,odom_z_,
                    dt*1000.0, (dt>1e-6?1.0/dt:0.0), algo_ms, map_hz, fov_hz);
            }
            rclcpp::spin_some(this->get_node_base_interface()); rate.sleep();
        }
        // 飞手切了 POSCTL → 停 offboard 流后直接退出程序, 不发降落指令
        // (PX4 已在飞手的位置控制下, 由飞手决定怎么落)。
        if (posctlActive()) {
            RCLCPP_WARN(this->get_logger(),
                "=== EXIT (POSCTL takeover) === Explored:%.1f%% St:%d Pos:(%.2f,%.2f,%.2f) — no land, exiting",
                planner_.exploredPct(),(int)planner_.state(),
                odom_x_,odom_y_,odom_z_);
            stopSpThread();
            return;
        }
        RCLCPP_INFO(this->get_logger(),"=== DONE === Explored:%.1f%% St:%d goal=(%.2f,%.2f) Pos:(%.2f,%.2f,%.2f)",
            planner_.exploredPct(),(int)planner_.state(),
            planner_.currentGoal().x, planner_.currentGoal().y,
            odom_x_,odom_y_,odom_z_);
        // ★到降落点后先"位置环悬停对准"保持 land_settle_secs, 再真正降落 ——
        //   和投放前那套一样(不是一进范围就直接落)。保持的位置 = 本段目标点
        //   (planner 的 goal_, 与到达判定同源), 高度保持当前高度(不额外升降)。
        if (land_settle_secs_ > 0.0) {
            const Point lg = planner_.currentGoal();
            const bool goal_ok = (std::hypot(lg.x - odom_x_, lg.y - odom_y_) < 2.0);   // 目标点合理才用它
            const double hx = goal_ok ? lg.x : odom_x_;
            const double hy = goal_ok ? lg.y : odom_y_;
            const double hz = odom_z_;                      // 保持当前高度, 交给 LAND 去降
            RCLCPP_INFO(this->get_logger(),
                "到达降落点 → 位置环悬停对准保持 %.1fs 后再降落 (保持(%.2f,%.2f) 当前偏差 %.3fm)",
                land_settle_secs_, hx, hy, std::hypot(hx - odom_x_, hy - odom_y_));
            auto ts = this->now();
            while (rclcpp::ok() && (this->now() - ts).seconds() < land_settle_secs_) {
                if (posctlActive()) {                       // 飞手接管 → 停 offboard 流后退出(不降落)
                    RCLCPP_WARN(this->get_logger(),
                        "POSCTL during landing settle → stop offboard & EXIT (no land)");
                    stopSpThread();
                    return;
                }
                if (state_.mode != "OFFBOARD") {            // 掉出 OFFBOARD → 直接交给 land 收尾
                    RCLCPP_WARN(this->get_logger(),
                        "Exited OFFBOARD during landing settle → 直接降落");
                    break;
                }
                // ★这里【不发偏航】(原来发 odom_yaw_ = 追测量值 → 落地前会自己慢慢转一点 ✗)
                //   yaw 传 NaN → 屏蔽偏航字段, PX4 保持当前机头 → 原地不动、直接降落 ✓
                flyTraj(hx, hy, hz, 0, 0, 0,
                        std::numeric_limits<double>::quiet_NaN(), 0.02);
                rclcpp::spin_some(this->get_node_base_interface());
                rate.sleep();
            }
            RCLCPP_INFO(this->get_logger(),
                "悬停对准完成 (最终水平偏差 %.3fm) → 开始降落",
                std::hypot(hx - odom_x_, hy - odom_y_));
        }
        // 已平飞到降落点正上方 → 直接用 land 服务垂直降落(原地, 不改变水平位置)。
        // 先停掉 50Hz OFFBOARD 设定点流, 避免与 LAND 模式互相打架。
        stopSpThread();
        land(rate);
    }

private:

	rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr landing_client_;

    /// launch gives WORLD coordinates (walls, explore region, landing pads).
    /// The radar SLAM odometry / point cloud live in the radar frame whose
    /// origin is origin_.  Once origin_ is known, add it to every world value
    /// so the planner map, explore box and pads all sit in the same frame as
    /// the drone (odom + origin) and the cloud points (p + origin).
    void applyOriginShift() {
        if (origin_shift_applied_) return;
        origin_shift_applied_ = true;

        // Map / explore globals used by worldToGrid & markFreeSpace etc.
        MAP_X_MIN = w_map_x_min_ + origin_x_;
        MAP_X_MAX = w_map_x_max_ + origin_x_;
        MAP_Y_MIN = w_map_y_min_ + origin_y_;
        MAP_Y_MAX = w_map_y_max_ + origin_y_;
        map_x_min_ = MAP_X_MIN; map_x_max_ = MAP_X_MAX;
        map_y_min_ = MAP_Y_MIN; map_y_max_ = MAP_Y_MAX;

        EXPLORE_X_MIN = std::max(w_explore_x_min_ + origin_x_, MAP_X_MIN);
        EXPLORE_X_MAX = std::min(w_explore_x_max_ + origin_x_, MAP_X_MAX);
        EXPLORE_Y_MIN = std::max(w_explore_y_min_ + origin_y_, MAP_Y_MIN);
        EXPLORE_Y_MAX = std::min(w_explore_y_max_ + origin_y_, MAP_Y_MAX);

        // Landing pads (planner return config) → shifted frame.
        if (w_flight_mode_ == 3) {
            // mode3: 返航目标 = 起飞点。起飞点在雷达系里的位置就是 origin_
            // (起飞前地面那 10 帧 odom 均值), 不走探索区角点、不进走廊。
            planner_.setReturnConfig(origin_x_, origin_y_, origin_x_, origin_y_,
                                     w_flight_mode_, w_channel_speed_);
            RCLCPP_INFO(this->get_logger(),
                "Return mode=3: 探索完/投放完直接回起飞点 (%.2f,%.2f) 原地降落 (不进走廊)",
                origin_x_, origin_y_);
        } else {
            planner_.setReturnConfig(
                w_pad_a_x_ + origin_x_, w_pad_a_y_ + origin_y_,
                w_pad_b_x_ + origin_x_, w_pad_b_y_ + origin_y_,
                w_flight_mode_, w_channel_speed_);
        }

        // ★通道路径规划方式(0=保守A* 1=先x再y) —— planner 内部走廊规划用
        planner_.setAbPathMode(ab_path_mode_, ab_axis_overshoot_, ab_axis_x_speed_, ab_axis_lead_);
        {
            const bool sync = this->get_parameter("planner_sync").as_int() != 0;
            planner_.setSyncPlanning(sync);
            planner_.setSyncPlanningTarget(sync);
            RCLCPP_INFO(this->get_logger(), "规划模式: %s",
                        sync ? "同步(算路时主循环会卡住)" : "后台线程(不卡, 结果晚1~2拍)");
        }

        // GUI: the drawn frame is map+origin.
        if (use_gui_) {
            std::lock_guard<std::mutex> lk(gui_mutex_);
            gui_->setBounds(map_x_min_, map_x_max_,
                            map_y_min_, map_y_max_);
        }

        RCLCPP_INFO(this->get_logger(),
            "Origin shift applied: map x:[%.2f,%.2f] y:[%.2f,%.2f] | explore x:[%.2f,%.2f] y:[%.2f,%.2f] | padA=(%.2f,%.2f)",
            MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX,
            EXPLORE_X_MIN, EXPLORE_X_MAX, EXPLORE_Y_MIN, EXPLORE_Y_MAX,
            w_pad_a_x_ + origin_x_, w_pad_a_y_ + origin_y_);
    }

    void state_cb(const mavros_msgs::msg::State::SharedPtr m){
        // 模式变化日志 + 飞手接管检测: 一旦切到 POSCTL, 各控制循环会立刻停止
        // offboard 设定点流并直接退出程序(见 posctlActive() 的调用处)。
        if (m->mode != state_.mode) {
            if (m->mode == "POSCTL")
                RCLCPP_WARN(this->get_logger(),
                    "mode: %s → POSCTL (pilot takeover) → will stop offboard & EXIT program (no land)",
                    state_.mode.c_str());
            else
                RCLCPP_INFO(this->get_logger(), "mode: %s → %s",
                    state_.mode.c_str(), m->mode.c_str());
        }
        state_=*m;
    }
    /// 飞手把模式切到 POSCTL(手动位置控制) → 视为接管: 停止 offboard 流后
    /// 直接退出程序, 不发降落指令(PX4 已由飞手接管)。
    /// 注意只在"已经进入过 OFFBOARD"之后的循环里判定 —— 启动前 FC 可能本来
    /// 就在 POSCTL, 那时要等 OFFBOARD, 不能误触发退出。
    bool posctlActive() const { return state_.mode == "POSCTL"; }
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr m){
        odom_x_=m->pose.pose.position.x; odom_y_=m->pose.pose.position.y; odom_z_=m->pose.pose.position.z;
        double qw=m->pose.pose.orientation.w,qx=m->pose.pose.orientation.x,qy=m->pose.pose.orientation.y,qz=m->pose.pose.orientation.z;
        odom_yaw_=std::atan2(2.0*(qw*qz+qx*qy),1.0-2.0*(qy*qy+qz*qz));
        odom_vx_=m->twist.twist.linear.x; odom_vy_=m->twist.twist.linear.y; odom_vz_=m->twist.twist.linear.z;
        odom_vyaw_=m->twist.twist.angular.z;
        // FastLIO often leaves angular.z = 0 → estimate yaw rate from heading diff.
        {
            double ts = this->now().seconds();
            if (std::abs(odom_vyaw_) < 1e-4 && odom_yaw_ts_ > 0.0) {
                double dT = ts - odom_yaw_ts_;
                if (dT > 1e-3 && dT < 1.0) {
                    double dy = odom_yaw_ - odom_yaw_prev_;
                    while (dy >  PI) dy -= 2.0 * PI;
                    while (dy < -PI) dy += 2.0 * PI;
                    odom_vyaw_ = dy / dT;
                }
            }
            odom_yaw_ts_ = ts;
            odom_yaw_prev_ = odom_yaw_;
        }
        // Origin baseline = mean of the first ORIGIN_FRAMES odometry frames.
        // Everything (cloud points, drone pos, targets) is shifted into the
        // origin frame: coord = odom + origin. So the whole map and the setpoints
        // live in the same shifted frame, and the drifted initial z=-0.5 still
        // means "fly to 1m" = target 0.5.
        if (!origin_ready_) {
            sum_x_ += odom_x_; sum_y_ += odom_y_; sum_z_ += odom_z_;
            if (++origin_frames_ >= ORIGIN_FRAMES) {
                origin_x_ = sum_x_ / ORIGIN_FRAMES;
                origin_y_ = sum_y_ / ORIGIN_FRAMES;
                origin_z_ = sum_z_ / ORIGIN_FRAMES;
                origin_ready_ = true;
                applyOriginShift();
                RCLCPP_INFO(this->get_logger(),
                    "Origin=(%.3f,%.3f,%.3f) from %d odom frames. Whole pipeline in origin frame (odom+origin).",
                    origin_x_, origin_y_, origin_z_, ORIGIN_FRAMES);
            }
        }
        // GUI drone marker: 雷达 SLAM 系(不加 origin, 与地图参数 shift 后一致)
        gui_drone_x_ = odom_x_;
        gui_drone_y_ = odom_y_;
    }
    void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr m){last_cloud_=m;cloud_ready_=true;}

    /// Process FastLIO global Laser_map: mark obstacles + FOV + free space.
    /// Clears old obstacles each frame, re-marks from latest cloud → true real-time.
    /// 坐标系: 点云直接用雷达 SLAM 原始坐标(不加 origin); 无人机位置也用原始 odom。
    /// launch 的世界系参数(地图范围/探索区/pad)才 +origin 转到雷达系(见 applyOriginShift)。
    /// ★虚拟通道右墙: 从 (corridor_wall_x_, corridor_wall_y_) 沿 -y 到地图 y 下边界,
    /// 整条线标为障碍(它不在雷达点云里, 所以每次重标地图后都要补一遍)。
    /// 用途: 自建场地没有通道围墙时, 防止无人机从场地外面绕到 B, 强制走通道路径规划。
    /// 标一道虚拟墙(共用逻辑): 从世界系 (wx, wy) 往 -y 标到地图 y 下边界
    void markVirtualWall(bool use, double wx, double wy, const char* tag){
        if (!use) return;
        const double x  = wx + origin_x_;   // 世界系 → 雷达系(与其它场地参数同一规则)
        const double y0 = wy + origin_y_;
        if (x < map_x_min_ || x > map_x_max_ ||
            y0 < map_y_min_ || y0 > map_y_max_) {
            if (++vwall_log_ % 200 == 1)
                RCLCPP_WARN(this->get_logger(),
                    "%s 点 (%.2f,%.2f) 世界系 → 雷达系 (%.2f,%.2f) 不在地图范围 x[%.2f,%.2f] y[%.2f,%.2f] 内 → 未标",
                    tag, wx, wy, x, y0,
                    map_x_min_, map_x_max_, map_y_min_, map_y_max_);
            return;
        }
        const double step = GRID_RES * 0.5;               // 半步长, 保证不留缝
        for (double y = y0; y >= map_y_min_; y -= step)
            planner_.markObstacle(x, std::clamp(y, map_y_min_, map_y_max_));
        if (++vwall_log_ % 20 == 1)
            RCLCPP_INFO(this->get_logger(),
                "%s: x=%.2f 从 y=%.2f 一直标到地图下边界 y=%.2f (世界系 x=%.2f y0=%.2f)",
                tag, x, y0, map_y_min_, wx, wy);
    }

    void applyVirtualWall(){
        markVirtualWall(use_corridor_wall_,  corridor_wall_x_,  corridor_wall_y_,  "虚拟通道右墙");
        markVirtualWall(use_corridor_wall2_, corridor_wall2_x_, corridor_wall2_y_, "虚拟墙2");
    }

    void processLaserMap(){
        // ★★ 起飞/悬停阶段不使用点云(现场要求 ✓): 直接返回, 障碍图保持全空 ✓
        if (!map_started_) return;
        // 先刷新障碍(用本拍点云),再做 FOV 射线标记 —— 保证遮挡判断用的是
        // 当前障碍:墙/柱后面的格子不会被相机扫到并标成已覆盖。
        //
        // 两级频率(刻意分开):
        //   * 障碍/自由空间 ← 只在"收到新点云"时重算(FAST_LIO 全局点云 1Hz);
        //   * FOV 覆盖标记 ← 每个控制周期都做(跟控制频率, 转向/移动时覆盖更细)。
        bool have_cloud = cloud_ready_ && last_cloud_;
        if (have_cloud && cloud_max_hz_ > 0.0) {
            const double dt = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_cloud_proc_t_).count();
            if (dt < 1.0 / cloud_max_hz_ - 1e-6) have_cloud = false;   // 本拍不处理(不清 cloud_ready_, 下一拍用最新帧 ✓)
        }
        if (have_cloud) {
            last_cloud_proc_t_ = std::chrono::steady_clock::now();
            cloud_ready_ = false;
            ++map_cnt_;          // 统计"点云→障碍图"实际频率

            // 无人机在雷达 SLAM 系中的位置(原始 odom)
            double rx = odom_x_, ry = odom_y_, rz = odom_z_;

            // 清除旧的障碍物标记 → 重新标记最新的全局地图
            if (!use_global_cloud_) {
                // Frontal 180° only: clear obstacles in forward sector, leave rear intact
                planner_.clearObstaclesInSector(rx, ry, odom_yaw_);
            } else {
                // ★★ 现场要求: 探索区内【障碍累加】(见过就留着 ✓ 薄柱子不会因某帧没扫到而消失 ✗
                //    → 根治"停一下又撞"✓); 探索区外(通道)仍然每帧全清重建 ✓
                //    → 裁判走进通道, 走开即消失 ✓✓
                if (!acc_all_cloud_) planner_.clearObstaclesOutsideExplore();  // ★全场累加时不清理 ✓
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*last_cloud_,*c);
            // ★★ 点云抽样(默认每 8 个点取 1 个): 建图对每个点要标一个【膨胀圆盘】
            //    (4dm→半径16格→33x33=1089 格/点 ✗), 全密度 1.15万点/帧 → 1250 万次写入
            //    → 一次建图 0.5~0.8 秒 ✗ → 每秒冻住 0.8 秒 → 飞机飞不动/原地悬停 ✓✓
            //    抽样 8 倍 → 开销降 8 倍 ✓ 墙/柱子由很多点命中, 抽掉 7/8 照样在图里 ✓
            size_t pt_idx = 0;
            const size_t skip = (cloud_subsample_ > 1) ? (size_t)cloud_subsample_ : 1;
            for(auto& p:c->points){
                if (skip > 1 && (pt_idx++ % skip) != 0) continue;
                // Shift cloud point into origin frame (FastLIO drift → same baseline as drone)
                double px = p.x, py = p.y, pz = p.z;   // 原始雷达系点云, 不加 origin
                if (fixed_cloud_on_) {
                    // ★★ A→B(通道)段: 改用【无人机自身高度附近】的带 —— 不用固定高度范围 ✗
                    //   为什么: 通道里 Z 估计有偏移(命令 0.5m、实际快着地) → 固定范围(如0.10~0.40)
                    //   会把该看的点云全滤掉 → 地图缺块/穿墙。跟随高度就没这个问题。
                    //   带宽用独立参数 ab_cloud_h_band(默认0.30), 和探索段的 height_band 分开 ✓
                    // ★非对称: 只用【无人机当前高度 +up / -down】✓ (现场要求 ✓)
                    { const double dzc = pz - rz;
                      if (dzc > ab_cloud_h_up_ || dzc < -ab_cloud_h_down_) continue; }
                } else {
                    // ★非对称: 探索段 上 +0.10 / 下 -0.40 ✓ (相对无人机当前高度 ✓)
                    { const double dze = pz - rz;
                      if (dze > explore_cloud_h_up_ || dze < -explore_cloud_h_down_) continue; }
                }
                double hd = std::sqrt((px-rx)*(px-rx) + (py-ry)*(py-ry));
                if(hd > lidar_range_) continue;
                // Frontal 180° filter: only process points ahead of drone
                if (!use_global_cloud_) {
                    double ang = std::atan2(py - ry, px - rx);
                    double diff = ang - odom_yaw_;
                    while (diff > M_PI) diff -= 2.0 * M_PI;
                    while (diff < -M_PI) diff += 2.0 * M_PI;
                    if (std::abs(diff) > M_PI / 2.0) continue;
                }
                double wx = std::clamp(px, map_x_min_, map_x_max_);
                double wy = std::clamp(py, map_y_min_, map_y_max_);
                planner_.markObstacle(wx, wy);
            }
            // 细缝填充: 补薄墙/挡板点云残缺的 1~2 格小洞(不增孤立噪点)
            planner_.fillObstacleHoles();
            // ★虚拟通道右墙(启用时): 点云重标后补一遍, 防止被本次清图擦掉
            applyVirtualWall();
        }

        // 根据配置选择使用向下FOV或前向FOV(射线+遮挡,基于刚更新的障碍图)
        // 按 fov_divider 分频(控制 50Hz → 默认 5 = 10Hz):
        // 无人机 20ms 只走 1cm, FOV 覆盖是 2.2x1.3m, 50Hz 相邻帧几乎完全重叠,
        // 重复标记同一批格子纯属浪费。新点云那一拍强制跑一次(障碍图刚变)。
        if (have_cloud || ++fov_tick_ >= fov_divider_) {
            fov_tick_ = 0;
            ++fov_cnt_;
            if (use_downward_fov_) {
                if (fov_mark_while_vision_ || vis_target_id_ < 0)
                    markDownwardFovOnly();   // 向下摄像头FOV — 地面矩形
            } else {
                if (fov_mark_while_vision_ || vis_target_id_ < 0)   // ★视觉任务期间可选是否标FOV
                markFovOnly();       // 前向摄像头FOV — 前方扇形
            }
        }

        // 重新标记自由空间(障碍更新后回填)
        if (have_cloud)
            planner_.markFreeSpace();
    }

    /// 向下摄像头FOV — 标记地面矩形区域为已探索
    void markDownwardFovOnly(){
        planner_.markDownwardFov(odom_x_, odom_y_, odom_yaw_,
                                 cam_fx_, cam_fy_, cam_cx_, cam_cy_,
                                 cam_w_, cam_h_, hgit_);
    }

    /// 前向摄像头FOV — 标记前方下倾相机看到的地面梯形区域为已探索
    void markFovOnly(){
        planner_.markForwardFov(odom_x_, odom_y_, odom_yaw_,
                                cam_fx_, cam_fy_, cam_cx_, cam_cy_,
                                cam_w_, cam_h_, hgit_, fov_max_range_,
                                fov_h_deg_, fov_tilt_deg_);
    }

    /// 设定点统一入口: 只更新"最新指令", 由独立 50Hz 线程持续发布 ——
    /// 主循环里的规划/GUI 再慢也不会让 OFFBOARD 掉线(以前主循环卡到 4Hz,
    /// PX4 判定 setpoint 超时 → 掉 ALTCTL → 自动降落)。
    void setSp(const mavros_msgs::msg::PositionTarget& m){
        std::lock_guard<std::mutex> lk(sp_mtx_);
        sp_msg_ = m; sp_ready_ = true;
    }
    void spLoop(){
        rclcpp::Rate r(50.0);           // 设定点发布 50Hz
        while(rclcpp::ok() && sp_run_.load()){
            mavros_msgs::msg::PositionTarget m; bool have=false;
            { std::lock_guard<std::mutex> lk(sp_mtx_);
              if(sp_ready_){ m = sp_msg_; have = true; } }
            if(have) pos_pub_->publish(m);
            r.sleep();
        }
    }
    void startSpThread(){ sp_run_.store(true); sp_thread_ = std::thread(&ExplorationNode::spLoop, this); }
    void stopSpThread(){ sp_run_.store(false); if(sp_thread_.joinable()) sp_thread_.join(); }

    // ★★ 心跳必须发【有效】设定点: 原来 type_mask=255(全部忽略) 会被 PX4 视为无效 ✗
//    → offboard 失效保护(0.5s)照常计时 → 掉出 OFFBOARD → 解锁被拒 ✓✓
//    现在改为"零速度保持"(速度=0, 帧 LOCAL_NED), 是 PX4 认的有效 setpoint ✓
void publishHeartbeat(){
    mavros_msgs::msg::PositionTarget m;
    m.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    m.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX|
                  mavros_msgs::msg::PositionTarget::IGNORE_PY|
                  mavros_msgs::msg::PositionTarget::IGNORE_PZ|
                  mavros_msgs::msg::PositionTarget::IGNORE_AFX|
                  mavros_msgs::msg::PositionTarget::IGNORE_AFY|
                  mavros_msgs::msg::PositionTarget::IGNORE_AFZ|
                  mavros_msgs::msg::PositionTarget::IGNORE_YAW|
                  mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    m.velocity.x = 0.0; m.velocity.y = 0.0; m.velocity.z = 0.0;   // 零速度 = 悬停保持 ✓
    setSp(m);
}
    /// Velocity in WORLD frame on setpoint_raw/local (same as flyTraj), used by
    /// takeoff/hover/descend so every stage shares one topic & frame.
    void flyVel(double vx,double vy,double vz,double vyaw){
        // ★vyaw = NaN → 本次不发偏航(让 PX4 保持机头), 与 flyTraj 一致
        const bool send_yaw = !std::isnan(vyaw);
        if (send_yaw && std::fabs(vyaw - odom_yaw_) > 1e-9) { nav_yaw_ = vyaw; nav_yaw_ok_ = true; }
        if (!send_yaw) { vyaw = odom_yaw_; }
        mavros_msgs::msg::PositionTarget m;
        m.coordinate_frame=mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        m.type_mask=mavros_msgs::msg::PositionTarget::IGNORE_PX|
                    mavros_msgs::msg::PositionTarget::IGNORE_PY|
                    mavros_msgs::msg::PositionTarget::IGNORE_PZ|
                    mavros_msgs::msg::PositionTarget::IGNORE_AFX|
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY|
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ|
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW;
        m.velocity.x=vx; m.velocity.y=vy; m.velocity.z=vz;
        m.yaw_rate=static_cast<float>(vyaw);
        setSp(m);
    }
    /// POSITION-mode tracking with velocity feedforward + yaw ramp (EGO-style).
    /// PX4 runs its own position loop on (px,py,pz); velocity.x/y are only a
    /// feedforward.  This is the previously-working flight controller.
    void flyTraj(double px, double py, double pz,
                 double vx, double vy, double vz, double yaw, double dt,
                 double ax_ff = 0.0, double ay_ff = 0.0){
        // ★yaw 传 NaN = 【这一次不发偏航】: 位置/速度照发, 但把 yaw 字段屏蔽掉,
        //   让 PX4 自己保持当前机头 —— 现场实测: 每拍都发同一个 yaw 反而会让机头轻微
        //   来回晃(因为每拍都带 yaw_rate 前馈), 不发就稳。
        const bool send_yaw = !std::isnan(yaw);
        if (send_yaw && std::fabs(yaw - odom_yaw_) > 1e-9) { nav_yaw_ = yaw; nav_yaw_ok_ = true; }
        mavros_msgs::msg::PositionTarget m;
        m.coordinate_frame=mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        // ★★ 注意: MAVLink 里 afx/afy 【不是位置环前馈】✗ 而是"加速度/力控制模式" ✓
        //    一开就会让 PX4 丢掉位置/速度控制 → 仿真里飞机只会爬(30s 挪 0.2m) ✗✗
        //    真机上这样发可能直接失控 ✗ → 所以【继续屏蔽】✓ 惯性问题改走 PX4 增益/降速 ✓
        (void)ax_ff; (void)ay_ff;
        m.type_mask=mavros_msgs::msg::PositionTarget::IGNORE_AFX|
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY|
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ;
        // 注意: 视觉段"只发一次偏航、之后不发"是用 visYawArg() 传 NaN 实现的 ✓ (见那里)
        //       这里不再另加门限 —— 否则会破坏探索/返航段"机头跟路径"的持续跟踪 ✗
        if (!send_yaw) {
            m.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW
                         |  mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
        }
        const bool send_yaw_eff = send_yaw;
        m.position.x=px; m.position.y=py; m.position.z=pz;
        m.velocity.x=vx; m.velocity.y=vy;
        if (std::abs(m.velocity.x) == std::abs(m.velocity.y)) m.velocity.x += 0.01;
        m.velocity.z=0.0;   // z is position-controlled (PX4 altitude loop holds it)
        // Yaw: internal commanded-yaw ramp, slew ≤ MAX_YAW_RATE*dt per tick.
        // Send both the yaw angle and its derivative as yaw_rate feedforward.
        if (!send_yaw_eff) { setSp(m); return; }    // ★屏蔽偏航: 只发位置/速度(机头交给PX4保持 ✓)
        rclcpp::Time yt = this->now();
        if (!yaw_cmd_init_ || (yt - yaw_cmd_last_).seconds() > 0.25)
            yaw_cmd_ = odom_yaw_;          // (re)anchor on mode transitions
        if (!std::isnan(yaw_tgt_prev_)) {
            double dj = yaw - yaw_tgt_prev_;
            while (dj >  PI) dj -= 2.0 * PI;
            while (dj < -PI) dj += 2.0 * PI;
            if (std::abs(dj) > M_PI / 2.0)
                yaw_cmd_ = odom_yaw_;      // big target jump → re-anchor
        }
        yaw_tgt_prev_ = yaw;
        double yaw_err = yaw - yaw_cmd_;
        while (yaw_err >  PI) yaw_err -= 2.0 * PI;
        while (yaw_err < -PI) yaw_err += 2.0 * PI;
        double max_step = MAX_YAW_RATE * dt;
        if (yaw_err >  max_step) yaw_err =  max_step;
        if (yaw_err < -max_step) yaw_err = -max_step;
        yaw_cmd_ += yaw_err;
        while (yaw_cmd_ >  PI) yaw_cmd_ -= 2.0 * PI;
        while (yaw_cmd_ < -PI) yaw_cmd_ += 2.0 * PI;
        m.yaw = static_cast<float>(yaw_cmd_);
        m.yaw_rate = (dt > 1e-4) ? static_cast<float>(yaw_err / dt) : 0.0f;
        yaw_cmd_last_ = yt;
        yaw_cmd_init_ = true;
        setSp(m);
    }

    // ================= 视觉目标任务 =================
    // 话题: geometry_msgs/Point (x,y=坐标 odom系, z=类别id), 5Hz。
    // 类别策略(launch 可配):
    //   vision_priority_classes = "0,1,5" → 优先飞的 3 个类别; **不按类别号排序**,
    //                                       这 3 个里谁先被识别到就先飞谁(FIFO)
    //   vision_fallback_classes = "3,2"   → 回退目标; 只有探索进度 >= vision_fallback_pct
    //                                       之后才允许飞(用来补齐总数), 按 3 → 2 的顺序
    //   vision_filter_classes   = "4"     → 直接丢弃(现场没这个目标 = 误识别)
    //   不在上面三类里的类别 → 丢弃
    // 总数: vision_total(默认3) = 现场只投 3 个标靶 → 飞满 3 个就"直接返航"。
    //   优先类别凑满 3 个 → 返航, 不再飞回退类别;
    //   进度到 vision_fallback_pct 还没凑满 3 个 → 用回退类别按 3,2 补齐;
    //   凑满 3 个 → 立即返航(不等探索阈值)。
    // 每趟流程(不再降高): A*规划飞到目标中心 → 对准中心 → 飞到"投放口对准中心"
    //   的偏移点 → 位置对准且速度刹住 → 调串口投放(**一个目标只投一个口**)
    //   → 原地停留 vision_hold_secs(等投放机构动作完成) → 记该类别已完成。
    // 投放口: 相对机体的位置(launch bomb_offsets, x前 y左), 按当前 yaw 旋转到
    //   雷达 SLAM 系; 三个口按"飞到的顺序"用 0→1→2(投放口编号, 与目标类别号无关)。
    enum VisState { VIS_IDLE, VIS_FLY, VIS_OFFSET, VIS_DESCEND, VIS_ALIGN, VIS_HOLD, VIS_ASCEND, VIS_SETTLE };
    /// @param cls 来自话题的类别号(<prefix><cls>), 以此为准; m->z 只做一致性检查
    void vision_cb(const geometry_msgs::msg::Point::SharedPtr m, int cls){
        std::lock_guard<std::mutex> lk(vis_mtx_);
        int id = cls;
        int zid = (int)std::llround(m->z);
        if (zid != id && ++vis_mismatch_ % 50 == 1)
            RCLCPP_WARN(get_logger(),
                "视觉消息类别不一致: 话题类别=%d 消息z=%d → 以话题为准", id, zid);
        // 过滤: 现场不存在的类别(误识别) + 不在类别策略里的类别 → 丢弃
        if (inList(vis_filter_, id) ||
            (!inList(vis_priority_, id) && !inList(vis_fallback_, id))) return;
        // ★任务收尾阶段(降落→投放→上升回巡航高度)完全不处理视觉数据:
        //   这段视觉节点还在发, 若把"同一物体的另一个类别"塞进暂存槽, 本趟一结束
        //   就会被当成新目标 → 同一目标投第二次。这里直接丢弃(等飞完再重新识别)。
        if (vis_state_ == VIS_DESCEND || vis_state_ == VIS_ALIGN ||
            vis_state_ == VIS_HOLD    || vis_state_ == VIS_ASCEND ||
            vis_state_ == VIS_SETTLE) return;
        // ★ 记录前过滤(已投放过的目标: 识别到了也不记录) —— 两道:
        //   ① 目标位置落在任一"已投放点"的去重半径内 → 过滤(即使这次被识别成别的类别)
        //   ② 飞机本身还在刚投放点附近(刚投完还没飞走) → 一律过滤: 近距离/大角度下
        //      深度投影误差可能 >1m, 光靠①会漏, 这样从源头杜绝"刚投过的目标又被记录"
        if (visionNearLanded(m->x, m->y)) {
            if (++vis_skip_log_ % 20 == 1)
                RCLCPP_INFO(this->get_logger(),
                    "vision: id=%d (%.2f,%.2f) 落在已投放点 %.2fm 内 → 过滤, 不记录",
                    id, m->x, m->y, visionDedupDist(m->x, m->y));
            return;
        }
        if (nearRecentDropPoint()) {
            if (++vis_skip_log_ % 20 == 1)
                RCLCPP_INFO(this->get_logger(),
                    "vision: id=%d (%.2f,%.2f) 飞机仍在刚投放点 %.2fm 内 → 过滤, 不记录",
                    id, m->x, m->y,
                    std::hypot(odom_x_ - vis_drop_pt_x_, odom_y_ - vis_drop_pt_y_));
            return;
        }
        if (vis_target_id_ >= 0 && id == vis_target_id_) {
            // ★跳变过滤: 同一类别相邻帧位置突变(>1.5m, 且距上一帧 <0.5s) → 判为"投影跳点"
            //   (框中心深度打到很近的东西/误检), 丢弃该帧。否则位置会瞬间跳到机下 →
            //   "中心对准"瞬间误判完成、或把飞机带向错误方向(实测遇到过一次)。
            //   无人机最大 0.8m/s → 0.5s 最多走 0.4m, 所以 1.5m 的突变必是坏帧。
            const double jump = std::hypot(m->x - vis_x_, m->y - vis_y_);
            // 从未收到过帧时 vis_last_rx_ 还是默认构造值 → 与 now() 时间源不同, 相减会抛
            // "can't subtract times with different time sources", 所以先判 vis_rx_ever_。
            const double gap  = vis_rx_ever_ ? (this->now() - vis_last_rx_).seconds() : 1e9;
            if (vis_live_seen_ && jump > vision_jump_tol_ && gap < 0.5) {
                vis_last_rx_ = this->now();          // 帧是活的, 只是这一帧不可信
                if (++vis_jump_skip_ % 10 == 1)
                    RCLCPP_WARN(get_logger(),
                        "vision: id=%d 位置跳变 %.2fm(距上一帧 %.2fs) → 丢弃该帧(疑似投影跳点)",
                        id, jump, gap);
                return;
            }
            // 锁定类别的数据: 直接用视觉实时坐标(不做噪点平滑), 并刷新超时
            vis_x_ = m->x;
            vis_y_ = m->y;
            vis_live_seen_ = true;
            vis_last_rx_ = this->now();
            vis_rx_ever_ = true;
            ++vis_rx_seq_;               // ★视觉帧序号: 高空采中位数按"视觉帧"采(不是按控制周期)
            return;
        }
        if (vis_done_.count(id)) return;         // 该类别的标靶已经飞过
        // 正在飞另一个目标时: 只有"离当前目标足够远"的其它类别才暂存 ——
        // 离得近的极可能是同一物体的误分类, 存下来就会重复投放。
        if (vis_target_id_ >= 0 &&
            std::hypot(m->x - vis_tx_, m->y - vis_ty_) < vision_dedup_r_) return;
        // 每个类别只保留最新坐标; 首次识别到的先后决定优先类别里的飞行顺序
        auto it = vis_slot_.find(id);
        if (it == vis_slot_.end()) {
            vis_slot_[id] = VisSlot{m->x, m->y, vis_seq_++};
            RCLCPP_INFO(this->get_logger(),
                "vision: id=%d stashed (%.2f,%.2f) seq=%ld%s", id, m->x, m->y,
                vis_slot_[id].seq,
                vis_target_id_ >= 0 ? " — 当前任务飞完后再选" : "");
        } else {
            it->second.x = m->x;
            it->second.y = m->y;
        }
    }
    /// 释放锁(断流中止 / 任务完成恢复探索 / 探索结束): 之后新的数据
    /// 才能成为下一个目标。
    void visionRelease(){
        vis_target_id_ = -1;
        vis_rx_ever_   = false;
        vis_live_seen_ = false;
        vis_yaw_locked_ = false;          // 低空偏航锁定解除(交回探索/下一个目标)
        vis_near_done_ = false;           // 下一个目标重新走"先稳一下"
        vis_near_t_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        vis_near_x_.clear(); vis_near_y_.clear(); vis_near_seq_ = -1;
        vis_offset_hold_logged_ = false;
        vis_hold_latched_ = false;
        vis_last_rx_   = this->now();     // 用节点时钟初始化, 避免时间源不一致
    }
    /// 该点是否已经降落过(与任一已完成降落点距离 < vision_dedup_radius)。
    /// 用于防止"同一个物体换了个类别 id 又被当成新目标, 在原地点再降一次"。
    bool visionNearLanded(double x, double y) const {
        for (const auto& p : vis_landed_xy_) {
            double dx = x - p.first, dy = y - p.second;
            if (dx*dx + dy*dy < vision_dedup_r_ * vision_dedup_r_) return true;
        }
        return false;
    }
    /// 返回 true = 本拍控制权归视觉任务(cmd 已填)。
    bool visionStep(double dt, Px4ExplorationPlanner::Cmd& cmd){
        // ★诊断(不改行为): 每次视觉状态切换打一行, 现场就能看出"到底跑的是哪个状态"
        static VisState dbg_last = VIS_IDLE;
        if (vis_state_ != dbg_last) {
            const char* nm = (vis_state_==VIS_IDLE?"IDLE":vis_state_==VIS_FLY?"FLY":
                              vis_state_==VIS_OFFSET?"OFFSET":vis_state_==VIS_DESCEND?"DESCEND":
                              vis_state_==VIS_ALIGN?"ALIGN":vis_state_==VIS_HOLD?"HOLD":
                              vis_state_==VIS_ASCEND?"ASCEND":vis_state_==VIS_SETTLE?"SETTLE":"?");
            RCLCPP_INFO(this->get_logger(),
                "[VIS] state %s → %s | id=%d 机=(%.2f,%.2f,%.2f) 视=(%.2f,%.2f) base=(%.2f,%.2f) valid=%d",
                (dbg_last==VIS_IDLE?"IDLE":dbg_last==VIS_FLY?"FLY":dbg_last==VIS_OFFSET?"OFFSET":
                 dbg_last==VIS_DESCEND?"DESCEND":dbg_last==VIS_ALIGN?"ALIGN":dbg_last==VIS_HOLD?"HOLD":
                 dbg_last==VIS_ASCEND?"ASCEND":dbg_last==VIS_SETTLE?"SETTLE":"?"),
                nm, vis_target_id_, odom_x_, odom_y_, odom_z_, vis_tx_, vis_ty_, drop_px_, drop_py_,
                (int)drop_point_valid_);
            dbg_last = vis_state_;
        }
        // 位置环在雷达 SLAM 系: setpoint = 世界高度 + origin
        const double sp_h = hgit_ + origin_z_;   // 探索高度(投放也在这个高度做)
        int st = (int)planner_.state();
        bool exploring = (st != (int)Px4ExplorationPlanner::GO_HOME &&
                          st != (int)Px4ExplorationPlanner::FINISHED);
        if (!exploring) {
            if (vis_state_ != VIS_IDLE) {     // 探索结束/返航 → 中止视觉任务
                RCLCPP_WARN(this->get_logger(),
                    "vision: task aborted (exploration ended) at id=%d", vis_target_id_);
                vis_state_ = VIS_IDLE;
                visionRelease();
            }
            return false;
        }
        // ★记录"进入当前视觉状态"的时刻(状态变化时刷新) —— 断流兜底要用它。
        if (vis_state_ != vis_prev_state_) { vis_state_t_ = this->now(); vis_prev_state_ = vis_state_; }
        // 距最近一次"锁定类别的实时帧"多久: 若从来没收到过实时帧(常见于"目标已离开视野/
        // 只靠暂存点锁定的情况"), 就按"进入本状态以来的时长"算 —— 否则 vis_rx_ever_=false
        // 会让下面的断流判定永远不触发 → 一直原地悬停出不来。
        const double rx_silent = vis_rx_ever_
            ? (this->now() - vis_last_rx_).seconds()
            : (this->now() - vis_state_t_).seconds();

        // ★视觉数据"变旧"时的策略(按现场需求): 目标常常刚好在 FOV 边缘 → 时有时无。
        //   这种情况**继续朝"最后一次收到的位置"飞**(vis_x_/vis_y_ 只在收到帧时更新,
        //   所以设定点本来就是粘住的) —— 飞过去往往就重新进视野了。
        //   只有真的失联 vision_lost_secs(默认6s) 才放弃, 恢复探索。
        //   (vision_timeout 仍用于"已到最后已知点仍无数据"等其它判定。)
        if (vis_state_ == VIS_FLY && rx_silent > vision_lost_secs_) {
            RCLCPP_WARN(this->get_logger(),
                "vision: id=%d 失联 >%.1fs%s → 放弃该目标, 恢复探索(此前一直朝最后已知点飞)",
                vis_target_id_, vision_lost_secs_,
                vis_rx_ever_ ? "" : "(锁定后从未收到实时帧, 一直用暂存点)");
            vis_state_ = VIS_IDLE;
            visionRelease();          // 断流 → 释放锁
            planner_.restartExplore();
            return false;
        }
        switch (vis_state_) {
        case VIS_IDLE: {
            // 标靶数已飞满 → 不再接新目标(返航由收尾处 forceGoHome 触发)
            if (vis_count_ >= vision_total_) return false;
            int id = -1; double wx = 0, wy = 0;
            // 回退类别是否已解锁(探索进度达到 vision_fallback_pct)
            const bool fallback_ok = (planner_.exploredPct() >= vision_fallback_pct_);
            {
                std::lock_guard<std::mutex> lk(vis_mtx_);
                if (!pickVisionTarget(fallback_ok, id, wx, wy)) return false;
                vis_slot_.erase(id);     // 选中即出槽, 避免同一目标反复选中
            }
            if (vis_done_.count(id) || visionNearLanded(wx, wy)) {
                const bool same_id = vis_done_.count(id) != 0;
                double dmin = 1e9;
                for (const auto& p : vis_landed_xy_)
                    dmin = std::min(dmin, std::hypot(wx - p.first, wy - p.second));
                RCLCPP_INFO(this->get_logger(),
                    "vision: id=%d at (%.2f,%.2f) 已投放过 %s (最近已投点 %.2fm < %.2fm) → 跳过, 不再重复投放",
                    id, wx, wy, same_id ? "(same class)" : "(同位置)", dmin, vision_dedup_r_);
                vis_done_.insert(id);    // 一并记下, 后续同类别数据直接丢
                return false;
            }
            // ★★ 原点附近的【5号】过滤掉 ✗ (现场: 起降点"H"被误识别成5号 ✓)
            if (id == 5 && std::hypot(wx, wy) < origin_reject_radius_) {
                RCLCPP_WARN(this->get_logger(),
                    "vision: id=5 (%.2f,%.2f) 距原点 %.2fm < %.2fm → 【原点H标志误识别, 忽略不飞】✓",
                    wx, wy, std::hypot(wx, wy), origin_reject_radius_);
                vis_done_.insert(id);   // 记为已处理 → 之后同类直接丢, 不反复刷日志 ✓
                return false;
            }
            vis_target_id_ = id;
            vis_port_ = vis_count_ % 3;  // ★端口在锁定时就固定(之后 vis_count_ 提前自增也不影响)
            vis_drop_called_ = false;    // 新目标 → 复位"已投放"标记(一个目标只投一个口)
            drop_point_valid_ = false;   // 复位降落基准点
            vis_align_x_.clear(); vis_align_y_.clear();
            // ★防重复投放(根因): 同一个物体常被识别成两个类别(如 0 和 5)。锁定 A 时
            // 把"当前已暂存、位置在去重半径内的其它类别"一并记为已处理 —— 否则等飞完
            // A、过一会儿再看到 B 时 SLAM 已经漂移, 位置差可能超过半径 → 同一目标被
            // 投第二次。
            {
                std::lock_guard<std::mutex> lk(vis_mtx_);
                for (auto it = vis_slot_.begin(); it != vis_slot_.end(); ) {
                    double d = std::hypot(it->second.x - wx, it->second.y - wy);
                    if (it->first != id && d < vision_dedup_r_) {
                        RCLCPP_INFO(this->get_logger(),
                            "vision: 类别%d 与本目标%d 相距 %.2fm < %.2fm → 判为同一物体, 一并标记(不再单独投放)",
                            it->first, id, d, vision_dedup_r_);
                        vis_done_.insert(it->first);
                        it = vis_slot_.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
            vis_tx_ = wx;                // 视觉是 odom(雷达)系 → 与地图同系, 不加 origin
            vis_ty_ = wy;
            // ★把"实时坐标"也初始化成暂存点: VIS_FLY 每拍用的是 vis_x_/vis_y_, 若这里不初始化
            //   它会保持默认 (0,0) → 目标瞬间跳到原点 → 飞机在原点附近时会立刻误判"已对准"
            //   (实测: err=0.058 一个周期就"中心对准完成"), 离原点远时则会朝原点飞。
            vis_x_ = wx;
            vis_y_ = wy;
            vis_live_seen_ = false;      // 还没收到过实时帧(用暂存点先飞)
            // 用 A* 规划一条绕开中间障碍的路径(不是直线硬飞) —— ★扔给后台线程算,
            // 控制线程本拍不阻塞(以前这里 planToPoint 会卡住控制循环 → 飞机"顿一下")。
            // 可达性判定挪到 VIS_FLY: 路径没算好之前先在原地悬停等结果。
            planner_.setDronePose(odom_x_, odom_y_, odom_yaw_);
            // ★必须先置"视觉任务激活": 否则同步规划当场就把结果丢掉了
            //   (applyVisionPath 会因 !vision_active_ 把结果清成 0 → status 永远 0 →
            //    3s 后"后台规划超过3s"放弃该目标)。异步版只是靠"任务算得慢"碰巧躲过。
            planner_.setVisionActive(true);
            planner_.planToPointAsync(vis_tx_, vis_ty_);
            vis_path_ready_ = false;
            vis_near_done_ = false;              // ★新目标: 复位"先稳一下"标志
            vis_near_t_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            vis_near_x_.clear(); vis_near_y_.clear(); vis_near_seq_ = -1;
            vis_offset_hold_logged_ = false; vis_offset_timeout_logged_ = false;
            vis_hold_latched_ = false;           // ★新目标: 悬停锁存点复位
            vis_sample_logged_ = false;
            vis_yaw_sent_ = false;
            vis_near_unsafe_logged_ = false;
            vis_slow_logged_ = false;            // ★新目标: 减速日志复位
            vis_stable_t_ = 0.0; vis_stable_ok_ = false;
            vis_aim_init_ = false;               // ★新目标: 瞄准点滤波重新初始化
            vis_path_wait_t_ = this->now();   // 等后台路径的超时计时
            vis_replan_t_ = 0.0;
            // 最近一个已降落点的距离(诊断: 若这次仍是同一物体, 该值会很小)
            double d_landed = -1.0;
            for (const auto& p : vis_landed_xy_) {
                double d = std::hypot(vis_tx_ - p.first, vis_ty_ - p.second);
                if (d_landed < 0 || d < d_landed) d_landed = d;
            }
            // 记录目标点，供到达判定使用（路径末端就是它）
            RCLCPP_INFO(this->get_logger(),
                "vision: new target id=%d radar=(%.2f,%.2f) 端口=%d 已投放%d/%d 已完成类别[%s] "
                "→ A* plan & fly (landed_pts=%zu nearest=%.2fm)",
                id, vis_tx_, vis_ty_, vis_port_, vis_count_, vision_total_,
                doneSetStr().c_str(), vis_landed_xy_.size(), d_landed);
            vis_state_ = VIS_FLY;
            return false;
        }
        case VIS_FLY: {
            // 飞行阶段始终使用"锁定 id 的最新视觉坐标"(实时跟随目标)
            {
                std::lock_guard<std::mutex> lk(vis_mtx_);
                vis_tx_ = aimFilter(vis_tx_, vis_x_);   // ★低通+死区, 不追逐帧抖动
                vis_ty_ = aimFilter(vis_ty_, vis_y_);
                vis_aim_init_ = true;
            }
            // ★后台规划结果: ①还没拿到路径 → 原地悬停等(通常几十ms);
            //                  ②失败(不可达) → 放弃该目标; ③成功 → 开始飞。
            if (!vis_path_ready_) {
                const int pr = planner_.planToPointStatus();
                if (pr == -1) {
                    RCLCPP_WARN(this->get_logger(),
                        "vision: id=%d target (%.2f,%.2f) unreachable → skip",
                        vis_target_id_, vis_tx_, vis_ty_);
                    vis_done_.insert(vis_target_id_);   // 不可达: 记为已处理
                    vis_state_ = VIS_IDLE;
                    visionRelease();
                    planner_.restartExplore();
                    return false;
                }
                if (pr == 1) {
                    vis_path_ready_ = true;
                    RCLCPP_INFO(this->get_logger(),
                        "vision: id=%d 路径就绪(同步规划) → 沿 A* 路径飞向目标", vis_target_id_);
                } else {
                    // 在算: 原地位置环悬停(不推进航段), 不阻塞控制循环
                    if ((this->now() - vis_path_wait_t_).seconds() > 3.0) {
                        RCLCPP_WARN(this->get_logger(),
                            "vision: id=%d 后台规划超过 3s 仍未返回 → 放弃该目标", vis_target_id_);
                        vis_done_.insert(vis_target_id_);
                        vis_state_ = VIS_IDLE;
                        visionRelease();
                        planner_.restartExplore();
                        return false;
                    }
                    flyTraj(odom_x_, odom_y_, sp_h, 0, 0, 0, navYaw(), dt);
                    cmd.pos_x = odom_x_; cmd.pos_y = odom_y_; cmd.pos_z = sp_h;
                    cmd.vel_x = 0; cmd.vel_y = 0; cmd.vel_z = 0;
                    cmd.yaw = std::isnan(visYawArg()) ? navYaw() : visYawArg(); cmd.valid = true;   // 发的yaw: 锁定值
                    return true;
                }
            }
            // 沿 A* 路径飞; 每 0.5s 让后台用最新地图/最新坐标重规划一次
            // (重规划期间继续飞当前路径, 算好后下一拍无缝换新)
            vis_replan_t_ += dt;
            if (vis_replan_t_ >= 0.5) {
                vis_replan_t_ = 0.0;
                planner_.setDronePose(odom_x_, odom_y_, odom_yaw_);
                planner_.planToPointAsync(vis_tx_, vis_ty_);
            }
            if (vis_near_done_ && drop_point_valid_) {     // ★已记录基准点 → 瞄准基准点飞中心
                vis_tx_ = drop_px_; vis_ty_ = drop_py_;
            }
            planner_.setDronePose(odom_x_, odom_y_, odom_yaw_);
            Px4ExplorationPlanner::Cmd pc = planner_.trackPlanned(dt);
            if (!pc.valid) {
                // ★没有可用路径时【绝不直线飞目标】—— 直线上可能有障碍(实飞撞过)。
                //   改为: 原地位置环悬停 + 立刻重新规划; 连续 2s 都规划不出来 → 放弃该目标
                //   (安全优先: 宁可少投一个, 也不冒险直线穿障)。
                if (vis_nopath_t_.nanoseconds() == 0) vis_nopath_t_ = this->now();
                // ★★ 关键: 绝不每拍重规划! 目标附近路径很短时 posTrack 会返回无效, 若每 20ms
                //    重规划一次, 而视觉坐标每帧抖动几厘米 → 每 20ms 生成一条方向不同的新路径
                //    → 飞机在目标中心附近【剧烈来回】("路径规划层面的徘徊")。
                //    这里: (a) 重规划节流到 0.4s 一次; (b) 已经很近(<=vis_nopath_free_m)时
                //    干脆不规划, 直接用位置环平滑靠近目标点(限幅, 不会猛冲)。
                const double d_np = std::hypot(vis_tx_ - odom_x_, vis_ty_ - odom_y_);
                if (d_np > vision_nopath_free_m_
                    && (this->now() - vis_nopath_last_).seconds() > 0.4) {
                    vis_nopath_last_ = this->now();
                    planner_.setDronePose(odom_x_, odom_y_, odom_yaw_);
                    planner_.planToPointAsync(vis_tx_, vis_ty_);
                }
                // 位置环直接朝目标点(限幅 0.30m 防猛冲; 近处自然收敛)
                {
                    double ddx = vis_tx_ - odom_x_, ddy = vis_ty_ - odom_y_;
                    const double dd = std::hypot(ddx, ddy);
                    constexpr double MAXJ = 0.30;
                    if (dd > MAXJ) { ddx *= MAXJ / dd; ddy *= MAXJ / dd; }
                    pc.pos_x = odom_x_ + ddx; pc.pos_y = odom_y_ + ddy;
                }
                pc.yaw = lowAltYaw(); pc.vel_x = pc.vel_y = 0; pc.valid = true;
                // ★只在"确实离目标还远"时才放弃: 接近目标时 pc 本来就会常判无效(路径很短),
                //   若不加这个条件, 会在刚到目标中心时把目标放弃掉。
                if ((this->now() - vis_nopath_t_).seconds() > 2.0 && d_np > 0.35) {
                    RCLCPP_WARN(this->get_logger(),
                        "vision: id=%d 无可用路径且 2s 内重规划都失败 → 放弃该目标(不直线穿障)",
                        vis_target_id_);
                    vis_done_.insert(vis_target_id_);
                    vis_state_ = VIS_IDLE;
                    visionRelease();
                    planner_.restartExplore();
                    return false;
                }
            } else {
                vis_nopath_t_ = rclcpp::Time(0, 0, RCL_ROS_TIME);   // 有路径 → 复位计时
            }
            cmd = pc;
            flyTraj(pc.pos_x, pc.pos_y, sp_h, pc.vel_x, pc.vel_y, 0, pc.yaw, dt);
            double dx = vis_tx_ - odom_x_;
            double dy = vis_ty_ - odom_y_;
            double err = std::hypot(dx, dy);
            // ★★ "先稳一下再对准": 前视相机(下倾55°)在快到位时机体还在移动/前冲,
            //    目标容易偏到视野边缘 → 投影中心偏 → 记录的基准点偏(实测投放偏差十几厘米)。
            //    所以: 飞到离目标 vision_near_dist(0.7m) 内 → 原地位置环悬停
            //    vision_near_hold_secs(0.5s) 稳定 → 这期间视觉持续更新坐标(即"重新计算坐标")
            //    → 然后再飞目标中心(vision_reach_tol 判定)。
            // ★"进入阈值就锁存": 只要进过一次 vision_near_dist, 这段悬停/采样就继续做完,
            //   悬停期间飘出去也不回去重新靠近(位置环原地保持即可, 采的是视觉坐标不受影响)。
            const bool near_started = (vis_near_t_.nanoseconds() != 0);
            if (vision_approach_mode_ == 1 && !vis_near_done_
                && (near_started || err < vision_near_dist_)) {
                if (vis_near_t_.nanoseconds() == 0) {
                    vis_near_t_ = this->now();
                    vis_near_x_.clear(); vis_near_y_.clear(); vis_near_seq_ = vis_rx_seq_;
                    RCLCPP_INFO(this->get_logger(),
                        "vision: id=%d 距目标 %.2fm → 先悬停 %.1fs 稳定后再取坐标对准",
                        vis_target_id_, err, vision_near_hold_secs_);
                }
                const double held = (this->now() - vis_near_t_).seconds();
                // ① 先纯悬停 vision_near_hold_secs(0.5s): 【不采样】—— 无人机/姿态还没稳,
                //    这时采到的坐标带抖动, 取中位数也白搭。
                if (held < vision_near_hold_secs_) {
                    // ★同样只锁存一次悬停点/偏航(不要每拍读自己的位置 → 会随机游走)
                    if (!vis_hold_latched_) {
                        vis_hold_x_ = odom_x_; vis_hold_y_ = odom_y_; vis_hold_yaw_ = odom_yaw_;
                        vis_hold_latched_ = true;
                    }
                    flyTraj(vis_hold_x_, vis_hold_y_, sp_h, 0, 0, 0, vis_hold_yaw_, dt);
                    cmd.pos_x = vis_hold_x_; cmd.pos_y = vis_hold_y_; cmd.pos_z = sp_h;
                    cmd.vel_x = 0; cmd.vel_y = 0; cmd.vel_z = 0;
                    cmd.yaw = vis_hold_yaw_; cmd.valid = true;
                    return true;
                }
                // ② 稳定够了才开始采样: 每收到一帧视觉采一个点(机体此时还在 0.7m 外,
                //    所以采的是【视觉上报坐标】, 不是机体位置), 采满 drop_align_frames 帧。
                // ★只在飞机基本静止时采样: 飞机还在飞的时候, 视觉坐标随视角变(实测同一目标
                //   移动中/静止差 0.14~0.21m), 把移动期间的帧混进中位数会让基准点偏 ✗。
                if (!posStableNow()) {
                    flyTraj(odom_x_, odom_y_, sp_h, 0, 0, 0, navYaw(), dt);
                    cmd.pos_x = odom_x_; cmd.pos_y = odom_y_; cmd.pos_z = sp_h;
                    cmd.vel_x = 0; cmd.vel_y = 0; cmd.vel_z = 0;
                    cmd.yaw = navYaw(); cmd.valid = true;
                    vis_near_seq_ = vis_rx_seq_;      // 没停稳的帧不计入
                    return true;
                }
                if (vis_near_x_.empty()) {                 // 进入采样阶段的第一拍: 从当前帧开始数
                    vis_near_seq_ = vis_rx_seq_;
                    RCLCPP_INFO(this->get_logger(),
                        "vision: id=%d 悬停 %.1fs 稳定完成 → 开始采 %d 帧视觉坐标",
                        vis_target_id_, vision_near_hold_secs_, std::max(3, drop_align_frames_));
                } else if (vis_rx_seq_ != vis_near_seq_) {
                    vis_near_seq_ = vis_rx_seq_;
                    vis_near_x_.push_back(vis_x_);
                    vis_near_y_.push_back(vis_y_);
                }
                const size_t need = (size_t)std::max(3, drop_align_frames_);
                const bool enough = (vis_near_x_.size() >= need);
                if (!enough && (held - vision_near_hold_secs_) < 6.0) {   // 20帧@5Hz≈4s, 窗口给6s
                    // 采样期间: 原地位置环悬停(同样锁存固定点, 不读自己位置)
                    if (!vis_hold_latched_) {
                        vis_hold_x_ = odom_x_; vis_hold_y_ = odom_y_; vis_hold_yaw_ = odom_yaw_;
                        vis_hold_latched_ = true;
                    }
                    flyTraj(vis_hold_x_, vis_hold_y_, sp_h, 0, 0, 0, vis_hold_yaw_, dt);
                    cmd.pos_x = vis_hold_x_; cmd.pos_y = vis_hold_y_; cmd.pos_z = sp_h;
                    cmd.vel_x = 0; cmd.vel_y = 0; cmd.vel_z = 0;
                    cmd.yaw = vis_hold_yaw_; cmd.valid = true;
                    return true;
                }
                // 采到足够帧(或超时) → 取中位数 = 基准点(视觉系), 然后飞到该点正上方
                drop_px_ = medianOf(vis_near_x_);
                drop_py_ = medianOf(vis_near_y_);
                drop_point_valid_ = true;
                vis_tx_ = drop_px_; vis_ty_ = drop_py_;      // 瞄准基准点(不再被实时抖动带走)
                vis_near_done_ = true;
                planner_.setDronePose(odom_x_, odom_y_, odom_yaw_);
                planner_.planToPointAsync(vis_tx_, vis_ty_);
                vis_path_ready_ = false;    // 重新等这次规划就绪
                vis_path_wait_t_ = this->now();
                RCLCPP_INFO(this->get_logger(),
                    "vision: id=%d 在 %.2fm 处 悬停%.1fs + 采%zu帧视觉坐标 → 基准点=(%.3f,%.3f) → 飞到该点正上方(容差%.2fm)",
                    vis_target_id_, err, held, vis_near_x_.size(), drop_px_, drop_py_, vision_reach_tol_);
                return true;
            }
            if (err < vision_reach_tol_) {
                // ★★ 到阈值(中心)这一瞬间: 【锁存当时的机位】, 之后一直给这个点悬停 0.5s
                //    —— 不再检查是否飘出阈值(飘出也不管)。悬停期间按视觉帧采基准点,
                //    悬停满 + 采够帧 → 原地垂直下降(投放口偏移留到低空)。
                if (!vis_hold_latched_) {
                    vis_hold_latched_ = true;
                    vis_hold_x_ = odom_x_; vis_hold_y_ = odom_y_; vis_hold_yaw_ = odom_yaw_;
                    vis_arrive_t_ = this->now();
                    if (!vis_yaw_locked_) {               // 已在 0.5m 处锁过就不再改
                        vis_lock_yaw_   = odom_yaw_;
                        vis_yaw_locked_ = true;
                    }
                    RCLCPP_INFO(this->get_logger(),
                        "vision: id=%d 到目标中心(err=%.3f < %.2f) → 锁存机位(%.3f,%.3f) yaw=%.1f° "
                        "→ 原地悬停 %.1fs (期间采基准点, 不再看是否飘出阈值)",
                        vis_target_id_, err, vision_reach_tol_, vis_hold_x_, vis_hold_y_,
                        vis_lock_yaw_ * 180.0 / M_PI, vision_near_hold_secs_);
                }
                flyTraj(vis_hold_x_, vis_hold_y_, sp_h, 0, 0, 0, visYawArg(), dt);   // ★一直给锁存点
                cmd.pos_x = vis_hold_x_; cmd.pos_y = vis_hold_y_; cmd.pos_z = sp_h;
                cmd.vel_x = 0; cmd.vel_y = 0; cmd.vel_z = 0;
                cmd.yaw = std::isnan(visYawArg()) ? navYaw() : visYawArg(); cmd.valid = true;
                // ★悬停 0.5s 期间【不采样】(飞机可能还在微调/姿态未稳), 稳完再采
                const double held = (this->now() - vis_arrive_t_).seconds();
                if (held < vision_near_hold_secs_) return true;
                if (vis_align_x_.empty() && !vis_sample_logged_) {
                    vis_sample_logged_ = true;
                    RCLCPP_INFO(this->get_logger(),
                        "vision: id=%d 悬停 %.1fs 结束 → 开始采 %d 帧视觉坐标(基准点)",
                        vis_target_id_, vision_near_hold_secs_, std::max(3, drop_align_frames_));
                }
                // 悬停结束后: 每来一帧视觉采一个点
                if (vis_rx_seq_ != vis_align_last_seq_) {
                    vis_align_last_seq_ = vis_rx_seq_;
                    if (drop_base_from_vision_) { vis_align_x_.push_back(vis_x_); vis_align_y_.push_back(vis_y_); }
                    else                        { vis_align_x_.push_back(odom_x_); vis_align_y_.push_back(odom_y_); }
                }
                const bool enough = (int)vis_align_x_.size() >= std::max(3, drop_align_frames_);
                // ★★ 采样窗口内【一帧都没收到】→ 绝不能继续(原来会走 medianOf(空)=0 →
                //    基准点变 (0,0) → 低空对准把飞机朝原点拽 4m ✗ 实测就是这么失控的 ✗)
                if (vis_align_x_.empty() && held > vision_near_hold_secs_ + 3.0) {
                    RCLCPP_WARN(this->get_logger(),
                        "vision: id=%d 悬停 %.1fs 内一帧视觉都没收到(目标已不在视野/被过滤) → "
                        "【放弃该目标, 不下降不投放】恢复探索", vis_target_id_, held);
                    vis_done_.insert(vis_target_id_);
                    vis_state_ = VIS_IDLE;
                    visionRelease();
                    planner_.restartExplore();
                    return false;
                }
                if (enough || held > vision_near_hold_secs_ + 3.0) {
                    // ★再保一道: 采样到的基准点必须落在机位附近(半个身位内), 否则也放弃
                    const double bx = medianOf(vis_align_x_), by = medianOf(vis_align_y_);
                    const double bd = std::hypot(bx - odom_x_, by - odom_y_);
                    if (vis_align_x_.size() < 3 || bd > 0.60) {
                        RCLCPP_WARN(this->get_logger(),
                            "vision: id=%d 基准点异常(采 %zu 帧, 基准(%.2f,%.2f) 离机位 %.2fm > 0.60m) "
                            "→ 【放弃该目标, 不下降不投放】恢复探索",
                            vis_target_id_, vis_align_x_.size(), bx, by, bd);
                        vis_done_.insert(vis_target_id_);
                        vis_state_ = VIS_IDLE;
                        visionRelease();
                        planner_.restartExplore();
                        return false;
                    }
                    drop_px_ = bx;
                    drop_py_ = by;
                    drop_point_valid_ = true;
                    double wxo, wyo; bombOffsetWorld(vis_port_, wxo, wyo);
                    // ★格式/参数顺序修正(原来 held 吃到 %zu → 整行数值错位 ✗)
                    RCLCPP_INFO(this->get_logger(),
                        "vision: id=%d 悬停 %.1fs 采 %zu 帧 → 基准点=(%.3f,%.3f) 机位=(%.3f,%.3f) "
                        "两者差%.3fm → 原地垂直到 %.2fm (投放口偏移%.2fm 留到低空)",
                        vis_target_id_, held, vis_align_x_.size(), drop_px_, drop_py_,
                        odom_x_, odom_y_, bd,
                        drop_height_, std::hypot(wxo, wyo));
                    vis_descend_z_ = odom_z_;
                    vis_descend_x_ = odom_x_;             // 原地垂直下降(水平锁当前机位)
                    vis_descend_y_ = odom_y_;
                    vis_align_t_      = this->now();
                    vis_drop_settle_t_ = this->now();
                    vis_state_ = VIS_DESCEND;
                }
                return true;
            } else if (err < vision_near_dist_) {
                // ★距目标 vision_near_dist(0.5m) 内: 直接位置环飞 + 【比例式速度前馈】。
                //   为什么不用"固定速度 + 到阈值急停": 急停会让机身/云台甩动, 视觉坐标随视角变
                //   (实测差 0.1~0.2m) → 采到的基准点就偏。比例前馈 = 速度随误差自然衰减
                //   (e=0.5→上限 0.30m/s; e=0.1→0.12m/s; e→0 速度→0), 平滑滑入不甩动。
                // ★一进 0.5m 就把偏航【锁存成固定值】(只锁一次): 否则 lowAltYaw() 会退回
                //   返回 odom_yaw_(测量值) → 每拍追噪声 → 机头轻微来回晃 → 视觉坐标跟着偏。
                if (!vis_yaw_locked_) {
                    vis_lock_yaw_   = odom_yaw_;
                    vis_yaw_locked_ = true;
                    RCLCPP_INFO(this->get_logger(),
                        "vision: id=%d 进入 %.2fm → 锁定偏航 yaw=%.1f° (此后固定, 不再追测量值)",
                        vis_target_id_, vision_near_dist_, vis_lock_yaw_ * 180.0 / M_PI);
                }
                // ★★ 实时安全判断(你的要求): 位置环直线只能"安全时"用。
                //   从当前机位到目标画一条直线, 用地图算沿途最小离障距离;
                //   ≥ 机体半径+5cm → 安全, 直接位置环直线滑入 ✓
                //   否则 → 不直线穿, 改用路径规划(节流重规划 + 沿现有路径跟踪) ✓
                const bool direct_ok = planner_.straightLineSafe(odom_x_, odom_y_, vis_tx_, vis_ty_);
                if (!direct_ok) {
                    if (!vis_near_unsafe_logged_) {
                        vis_near_unsafe_logged_ = true;
                        RCLCPP_WARN(this->get_logger(),
                            "vision: id=%d 距目标 %.2fm → 【直线不安全】(沿途离障<%.2fm) → 改用路径规划",
                            vis_target_id_, err, 0.25 + 0.05);
                    }
                    if (vis_replan_t_ >= 0.5) { vis_replan_t_ = 0.0; planner_.planToPointAsync(vis_tx_, vis_ty_); }
                    Px4ExplorationPlanner::Cmd pc2 = planner_.trackPlanned(dt);
                    if (pc2.valid) {
                        flyTraj(pc2.pos_x, pc2.pos_y, pc2.pos_z, pc2.vel_x, pc2.vel_y, pc2.vel_z,
                                visYawArg(), dt);
                        cmd = pc2;
                        cmd.yaw = std::isnan(visYawArg()) ? navYaw() : visYawArg();
                    } else {
                        flyTraj(odom_x_, odom_y_, sp_h, 0, 0, 0, visYawArg(), dt);   // 无路径 → 原地等重规划
                        cmd.pos_x = odom_x_; cmd.pos_y = odom_y_; cmd.pos_z = sp_h;
                        cmd.vel_x = 0; cmd.vel_y = 0; cmd.vel_z = 0;
                        cmd.yaw = std::isnan(visYawArg()) ? navYaw() : visYawArg(); cmd.valid = true;
                    }
                    return true;
                }
                const double KFF = vision_near_kff_;
                double vx = KFF * (vis_tx_ - odom_x_);
                double vy = KFF * (vis_ty_ - odom_y_);
                const double vm = std::hypot(vx, vy);
                if (vm > vision_near_vmax_) { vx *= vision_near_vmax_ / vm; vy *= vision_near_vmax_ / vm; }
                else if (vm < vision_near_vmin_) {          // ★最小速度 0.1m/s(按方向补足)
                    if (vm > 1e-6) { vx *= vision_near_vmin_ / vm; vy *= vision_near_vmin_ / vm; }
                    else           { vx = 0.0; vy = 0.0; }
                }
                if (!vis_slow_logged_) {
                    vis_slow_logged_ = true;
                    RCLCPP_INFO(this->get_logger(),
                        "vision: id=%d 距目标 %.2fm → 【直线安全】直接位置环+比例前馈滑入(kff=%.1f, 上限%.2fm/s, 判到中心<%.2fm)",
                        vis_target_id_, err, KFF, vision_near_vmax_, vision_reach_tol_);
                }
                flyTraj(vis_tx_, vis_ty_, sp_h, vx, vy, 0, visYawArg(), dt);   // ★位置点 + 速度前馈
                cmd.pos_x = vis_tx_; cmd.pos_y = vis_ty_; cmd.pos_z = sp_h;
                cmd.vel_x = vx; cmd.vel_y = vy; cmd.vel_z = 0;
                cmd.yaw = std::isnan(visYawArg()) ? navYaw() : visYawArg(); cmd.valid = true;
                return true;
            }
            return true;
        }
        case VIS_OFFSET: {
            // ★★ 按你的要求: 【机体在目标中心采样】, 降落后再在低空飞偏移。
            //   到中心后悬停在目标正上方 → 按视觉帧采 drop_align_frames 帧基准点
            //   → 【原地垂直下降】; 投放口偏移对准全部留到低空 VIS_ALIGN 做。
            {
                std::lock_guard<std::mutex> lk(vis_mtx_);
                vis_tx_ = vis_x_;
                vis_ty_ = vis_y_;
            }
            cmd.pos_x = vis_tx_; cmd.pos_y = vis_ty_; cmd.pos_z = sp_h;   // 设定点 = 目标中心(机体压在上面)
            cmd.vel_x = 0; cmd.vel_y = 0; cmd.vel_z = 0;
            cmd.yaw = std::isnan(visYawArg()) ? navYaw() : visYawArg(); cmd.valid = true;
            flyTraj(vis_tx_, vis_ty_, sp_h, 0, 0, 0, visYawArg(), dt);
            const double e = std::hypot(vis_tx_ - odom_x_, vis_ty_ - odom_y_);
            // 高空失联超时(保留)
            if (rx_silent > vision_lost_secs_) {
                RCLCPP_WARN(this->get_logger(),
                    "vision: id=%d 高空对准期间失联 >%.1fs%s → 放弃该目标恢复探索",
                    vis_target_id_, vision_lost_secs_,
                    vis_rx_ever_ ? "" : "(锁定后从未收到实时帧, 目标已不在视野)");
                vis_done_.insert(vis_target_id_);
                vis_state_ = VIS_IDLE;
                visionRelease();
                planner_.restartExplore();
                return false;
            }
            // 门槛: 机体已在中心容差内(从 VIS_FLY 过来时已满足, 这里继续伺服收敛);
            // 一旦开始采就一直采完, 中途飘出容差也不作废
            if (!vis_align_x_.empty() || e < vision_reach_tol_) {
                if (vis_rx_seq_ != vis_align_last_seq_) {      // 收到新视觉帧才采一个点
                    vis_align_last_seq_ = vis_rx_seq_;
                    if (drop_base_from_vision_) {              // 基准点来源开关
                        vis_align_x_.push_back(vis_x_);
                        vis_align_y_.push_back(vis_y_);
                    } else {
                        vis_align_x_.push_back(odom_x_);       // ★默认: 机体位置(此时就在目标中心)
                        vis_align_y_.push_back(odom_y_);
                    }
                    if (static_cast<int>(vis_align_x_.size()) >= std::max(3, drop_align_frames_)) {
                        drop_px_ = medianOf(vis_align_x_);
                        drop_py_ = medianOf(vis_align_y_);
                        drop_point_valid_ = true;
                        double wxo, wyo; bombOffsetWorld(vis_port_, wxo, wyo);
                        RCLCPP_INFO(this->get_logger(),
                            "vision: id=%d 记录对准位置中位数(%zu帧) 基准点=(%.3f,%.3f) 机体=(%.3f,%.3f) "
                            "中心偏差%.3fm → 原地垂直到 %.2fm (投放口偏移留到低空, 口偏移%.2fm)",
                            vis_target_id_, vis_align_x_.size(), drop_px_, drop_py_,
                            odom_x_, odom_y_, e, drop_height_, std::hypot(wxo, wyo));
                        vis_descend_z_ = odom_z_;              // 降落设定点从当前高度开始
                        vis_descend_x_ = odom_x_;              // ★原地下降: 水平锁当前机位
                        vis_descend_y_ = odom_y_;
                        vis_align_t_      = this->now();
                        vis_drop_settle_t_ = this->now();
                        vis_state_ = VIS_DESCEND;
                    }
                }
            } else {
                vis_hit_t_ = this->now();
            }
            return true;
        }
        case VIS_DESCEND: {
            // ★★ 边降落边朝目标飞(直接偏移对准):
            //   下降过程中就把水平位置伺服到"基准点 − 口偏移"那个点(x,y 合成方向 ✓),
            //   这样降到投放高度时水平偏差已经很小 → 低空只需飞一点点残差。
            const double z_tgt = drop_height_ + origin_z_;
            vis_descend_z_ -= descent_speed_ * dt;
            if (vis_descend_z_ < z_tgt) vis_descend_z_ = z_tgt;
            double ox, oy;
            recordedDropDronePos(ox, oy);                     // ★水平目标 = 基准点 − 口偏移(合成)
            // 比例速度前馈(与接近段同一套): 速度∝误差, 平滑不甩动
            const double KFF = drop_align_kff_;
            double ffx = KFF * (ox - odom_x_);
            double ffy = KFF * (oy - odom_y_);
            const double VMAX = std::min(drop_align_vmax_, descent_speed_);   // 横向别比下降还快
            const double vm = std::hypot(ffx, ffy);
            if (vm > VMAX) { ffx *= VMAX / vm; ffy *= VMAX / vm; }
            else if (vm < 1e-3) { ffx = 0.0; ffy = 0.0; }
            cmd.pos_x = ox; cmd.pos_y = oy; cmd.pos_z = vis_descend_z_;
            cmd.vel_x = ffx; cmd.vel_y = ffy; cmd.vel_z = 0;
            cmd.yaw = std::isnan(visYawArg()) ? navYaw() : visYawArg(); cmd.valid = true;
            flyTraj(ox, oy, vis_descend_z_, ffx, ffy, 0, visYawArg(), dt);
            // ★ 必须"飞机实际到了降落高度"才进 ALIGN
            if (odom_z_ <= z_tgt + drop_height_tol_) {
                double tx2, ty2; recordedDropDronePos(tx2, ty2);
                double e = std::hypot(tx2 - odom_x_, ty2 - odom_y_);
                RCLCPP_INFO(this->get_logger(),
                    "vision: id=%d 已到达降落高度 (odom_z=%.2f 目标z=%.2f, 高度差 %.3fm, 剩余水平 %.3fm) → 低空严格对准(%.3fm)+稳定(%.1fs)再投放",
                    vis_target_id_, odom_z_, z_tgt, std::fabs(odom_z_ - z_tgt), e,
                    drop_final_tol_, drop_settle_secs_);
                vis_state_ = VIS_ALIGN;
                vis_align_t_ = this->now();
                vis_hit_t_ = this->now();
                vis_drop_settle_t_ = this->now();
            }
            return true;
        }
        case VIS_ALIGN: {
            // 降落后对准(严): 降到指定高度后位置会偏, 先对准"降落前记录的位置"
            // (基准点 = 中位数记录), 满足 drop_final_tol 且速度刹住才投放。
            double ox, oy;
            recordedDropDronePos(ox, oy);
            // ★低空横向偏移加"偏差比例速度前馈": 只发位置设定点+vel=0 时, 全靠 PX4 内部
            //   位置环(P 项 + MPC_XY_ERR_MAX 误差限幅) → 20cm 偏差要爬好几秒, 现场像
            //   "飞不动/慢慢屯屯"。这里按偏差给一个限速速度前馈(到位自然减速):
            //   e=0.2m → 0.20m/s(约1秒到位); e→0.04m → 0.04m/s(仍满足 drop_vel_tol 0.05)
            double ffx = 0, ffy = 0;
            {
                // ★增益/限速可调: 位置环纯P(无积分), 持续扰动下稳态误差≈扰动/增益 →
                //   加大 drop_align_kff 能明显缩小"降落后还差十几厘米"的稳态偏差。
                const double KFF = drop_align_kff_;
                const double VMAX_FF = drop_align_vmax_;
                // ★接近目标时按误差收力: e<0.1m 后增益线性衰减(下限0.25) —— 保证最后几厘米
                //   能同时满足 e<drop_final_tol(0.04) 且 v<drop_vel_tol(0.05) 并停稳。
                const double eraw = std::hypot(ox - odom_x_, oy - odom_y_);
                const double kff = KFF * std::clamp(eraw / 0.10, 0.25, 1.0);
                ffx = kff * (ox - odom_x_);
                ffy = kff * (oy - odom_y_);
                const double vm = std::hypot(ffx, ffy);
                if (vm > VMAX_FF) { ffx *= VMAX_FF / vm; ffy *= VMAX_FF / vm; }
                else if (vm < 1e-3) { ffx = 0; ffy = 0; }
            }
            cmd.pos_x = ox; cmd.pos_y = oy; cmd.pos_z = drop_height_ + origin_z_;
            cmd.vel_x = ffx; cmd.vel_y = ffy; cmd.vel_z = 0;
            cmd.yaw = std::isnan(visYawArg()) ? navYaw() : visYawArg(); cmd.valid = true;
            flyTraj(ox, oy, drop_height_ + origin_z_, ffx, ffy, 0, visYawArg(), dt);
            // ★低空对准每秒诊断: 一次飞行就能看出"为什么不飞偏移"
            //   (基准点/偏移/目标点/机位/误差/前馈/高度差/速度/是否锁定)
            {
                const double now_s = this->now().seconds();
                if (now_s - vis_align_diag_t_ > 1.0) {
                    vis_align_diag_t_ = now_s;
                    double wxo, wyo; bombOffsetWorld(vis_port_, wxo, wyo);
                    double bx = drop_point_valid_ ? drop_px_ : vis_tx_;
                    double by = drop_point_valid_ ? drop_py_ : vis_ty_;
                    RCLCPP_INFO(this->get_logger(),
                        "[ALIGN] 口%d 基准=(%.3f,%.3f)%s 偏移=(%.3f,%.3f) 目标=(%.3f,%.3f) 机体=(%.3f,%.3f) "
                        "e=%.3f dz=%.3f v=%.2f ff=(%.3f,%.3f) yaw锁=%d 锁定yaw=%.1f° 实际yaw=%.1f°(差%.1f°) 已对准%.1fs",
                        vis_port_, bx, by, drop_point_valid_ ? "" : "(用实时视觉)", wxo, wyo, ox, oy,
                        odom_x_, odom_y_,
                        std::hypot(ox - odom_x_, oy - odom_y_), std::fabs(odom_z_ - (drop_height_ + origin_z_)),
                        std::hypot(odom_vx_, odom_vy_), ffx, ffy,
                        (int)vis_yaw_locked_, lowAltYaw() * 180.0 / M_PI,
                        odom_yaw_ * 180.0 / M_PI,
                        (odom_yaw_ - lowAltYaw()) * 180.0 / M_PI,
                        (this->now() - vis_align_t_).seconds());
                }
            }
            if (vis_drop_called_) { vis_state_ = VIS_HOLD; vis_hold_t_ = this->now(); return true; }
            const double z_drop = drop_height_ + origin_z_;
            // ★e = 水平(x,y)误差: 机体到"投放口偏移点"的平面距离。
            //   满足 e<drop_final_tol(0.04) ⇔ 投放口在 x,y 上落在基准点 4cm 内。
            //   (高度差 dz 单独用 drop_height_tol 判, 不混进 e; 速度 v 单独用 drop_vel_tol)
            double e  = std::hypot(ox - odom_x_, oy - odom_y_);
            double dz = std::fabs(odom_z_ - z_drop);
            double v  = std::sqrt(odom_vx_*odom_vx_ + odom_vy_*odom_vy_ + odom_vz_*odom_vz_);  // ★含垂直速度
            // ★超时用"进入 ALIGN 之后"的独立时钟(以前用的是进入 OFFSET 的时刻, 降落前
            //   多花几秒就会让这里一进来就 give_up → 位置/高度/速度都不管直接投放)
            const bool align_timeout = (this->now() - vis_align_t_).seconds() > drop_try_secs_;
            const bool height_ok  = (dz < drop_height_tol_);              // ★必须真到高度
            // ★★ 现场要求: 5号(最小标靶)低空对准【用更严格的判据】, 和别的类别区分开 ✓
            //    别的类别: 0.04m / 超时放宽 3 倍
            //    5 号    : 0.020m / 超时放宽 1.5 倍  → 宁可多等, 也不投到边上 ✓✓
            const bool  strict5   = (vis_target_id_ == 5);
            const double tol_use  = strict5 ? drop_final_tol_small_ : drop_final_tol_;
            const double v_tol_use= strict5 ? drop_vel_tol_ * 0.5   : drop_vel_tol_;
            // ★★ 兜底必须【可达】: 5号的严格判据(0.02)本身很难达到, 若放宽倍数也按 0.02 算
            //    (0.02*1.5=0.03) → 永远触发不了 → 飞机会【一直悬在 0.12m 不投】✗✗
            //    (实测 id=5 卡在 0.12m 超过 15 秒 ✗)
            //    → 所以放宽的【基准】用普通判据 drop_final_tol(0.04), 只是倍数按 5 号收紧 ✓
            const double relax_m  = strict5 ? drop_relax_mult_small_ : 3.0;
            const double relax_base = drop_final_tol_;      // ★兜底基准统一用普通值 ✓ 保证可达 ✓
            const bool pos_vel_ok = (e < tol_use && v < v_tol_use);
            // 正常路径: e<0.04m 且 v<0.05m/s 且高度到位, 并稳定 drop_settle_secs → 投。
            // ★超时兜底"照旧保留": 对准超过 drop_try_secs 后放宽到 3 倍容差也投 ——
            //   投出去还有机会命中, 不投就完全没机会(现场要求)。偏差会打在日志里。
            const bool relaxed_ok = height_ok && (e < relax_base*relax_m) && (v < drop_vel_tol_*relax_m);
            // ★现场要求: 【不等 yaw 转到位】✓ 对准位置本来就是按【实际 yaw】实时算的
            //   (见 bombOffsetWorld: 默认用 odom_yaw_ ✓) → 机头转到哪, 口就按哪算 ✓
            //   → 所以只要【口到目标的误差 e】够小就可以投 ✓ 不需要额外等 yaw ✓
            // ★可选模式 drop_xy_only=1: 只看【水平(x,y)对准】, 忽略高度差 dz 与刹车速度 v。
            //   理由: 投放高度很低(如0.12m), 炸弹近似直线下落, 不会被机身晃动甩偏, 所以
            //   只要口在目标上方对准并连续保持 drop_xy_hold_ticks 个控制帧(50Hz, 20帧≈0.4s)
            //   就投放 —— 比等"高度+速度+稳定"快很多。
            // ★硬上限: 等超过 drop_try_secs 的 4 倍(≈5s) 无论如何都投 ✓ 绝不卡在半空 ✓
            const bool hard_cap = (this->now() - vis_align_t_).seconds() > drop_try_secs_ * 4.0;
            bool allow_drop = false;
            if (drop_xy_only_) {
                if (e < tol_use) { if (vis_xy_ok_cnt_ < 100000) ++vis_xy_ok_cnt_; }
                else                       vis_xy_ok_cnt_ = 0;          // 飘出容差 → 重新数帧
                allow_drop = (vis_xy_ok_cnt_ >= std::max(1, drop_xy_hold_ticks_));
                if (allow_drop && ++vis_xy_log_ % 5 == 1)
                    RCLCPP_INFO(this->get_logger(),
                        "vision: id=%d [仅x,y对准模式] 水平误差%.3fm 连续%d帧(≈%.2fs) → 投放(忽略高度/速度)",
                        vis_target_id_, e, vis_xy_ok_cnt_, vis_xy_ok_cnt_ * 0.02);
            } else {
                allow_drop = height_ok
                          && ((pos_vel_ok || (align_timeout && relaxed_ok))
                              && ((this->now() - vis_drop_settle_t_).seconds() >= drop_settle_secs_
                                  || (align_timeout && relaxed_ok))
                              || hard_cap);      // ★硬上限: 到点就投, 绝不卡死 ✓
            }
            if (allow_drop) {
                if (true) {
                    if (e >= drop_final_tol_ || v >= drop_vel_tol_)
                        RCLCPP_WARN(this->get_logger(),
                            "vision: id=%d 对准超时: 水平(x,y)误差=%.3fm 速度=%.3fm/s (高度已到位 dz=%.3fm) → 按当前精度投放",
                            vis_target_id_, e, v, dz);
                    const int port = vis_port_;           // 投放口编号(锁定目标时已固定)
                    vis_drop_called_ = true;              // ★ 一次目标只投一个口
                    // ★★ 投放"之前"就登记, 而不是投完再登记:
                    //    - 该类别写入 vis_done_ → 之后同类别数据/同类别目标一律不再飞
                    //    - 记录投放点(中位数基准点 + 视觉点) → 位置去重也能拦住
                    //    - 计数 +1(端口已固定, 不影响本次剩余动作)
                    //    这样即使随后的收尾/上升/悬停任一环节出问题, 也不会再把同一
                    //    目标当成新目标飞一次 → 从根上杜绝"一个目标投两次"。
                    vis_done_.insert(vis_target_id_);
                    if (drop_point_valid_) vis_landed_xy_.push_back({drop_px_, drop_py_});
                    vis_landed_xy_.push_back({vis_tx_, vis_ty_});
                    ++vis_count_;
                    // ★★ 打开"先离开投放点"窗口: 飞机此刻就停在刚投过的目标正上方,
                    //    相机近距离/大角度看同一个目标时深度投影误差可能 >1m, 位置去重
                    //    拦不住 → 会又降下去投一次。所以这段时间内不接收任何视觉目标。
                    vis_drop_pt_valid_ = true;
                    vis_drop_pt_x_ = drop_point_valid_ ? drop_px_ : vis_tx_;
                    vis_drop_pt_y_ = drop_point_valid_ ? drop_py_ : vis_ty_;
                    vis_drop_pt_t_ = this->now();
                    logBombDrop("投放", e, v);
                    // 同一个口连发 drop_repeat 次: 防止单次指令没被 MCU 触发
                    Arduino::bombRepeat(port, drop_repeat_, drop_repeat_gap_ms_);
                    vis_state_ = VIS_HOLD;                // 投放后原地停留
                    vis_hold_t_ = this->now();
                }
            } else if (!drop_xy_only_) {
                vis_drop_settle_t_ = this->now();         // 没稳住 → 重新计时
                if (align_timeout) {
                    if (++vis_align_warn_ % 20 == 1)
                        RCLCPP_WARN(this->get_logger(),
                            "vision: id=%d 水平(x,y)误差未达 %.3fm 仍在对准 (水平e=%.3fm 速度=%.3fm/s 高度差dz=%.3fm; 超过 %.0fs 按放宽精度投)",
                            vis_target_id_, drop_final_tol_, e, v, dz, drop_try_secs_);
                    // 极端情况: 长时间既到不了高度也对不准 → 放弃该目标, 避免卡死
                    if ((this->now() - vis_align_t_).seconds() > drop_try_secs_ * 3.0) {
                        RCLCPP_ERROR(this->get_logger(),
                            "vision: id=%d 投放前对准/降高失败超过 %.0fs → 放弃该目标恢复探索",
                            vis_target_id_, drop_try_secs_ * 3.0);
                        vis_done_.insert(vis_target_id_);
                        vis_state_ = VIS_IDLE;
                        visionRelease();
                        planner_.restartExplore();
                    }
                }
            }
            return true;
        }
        case VIS_HOLD: {
            // 投放后原地停留 vision_hold_secs(等投放机构动作完成), 然后结束本目标
            const double zt = drop_height_ + origin_z_;
            double ox, oy;
            recordedDropDronePos(ox, oy);
            // ★保持段同样加偏差比例速度前馈(见 VIS_ALIGN 注释): 否则漂出去一点要爬很久
            double ffx = 0, ffy = 0;
            {
                constexpr double KFF = 1.0;
                constexpr double VMAX_FF = 0.30;
                ffx = KFF * (ox - odom_x_);
                ffy = KFF * (oy - odom_y_);
                const double vm = std::hypot(ffx, ffy);
                if (vm > VMAX_FF) { ffx *= VMAX_FF / vm; ffy *= VMAX_FF / vm; }
                else if (vm < 1e-3) { ffx = 0; ffy = 0; }
            }
            cmd.pos_x = ox; cmd.pos_y = oy; cmd.pos_z = zt;
            cmd.vel_x = ffx; cmd.vel_y = ffy; cmd.vel_z = 0;
            cmd.yaw = std::isnan(visYawArg()) ? navYaw() : visYawArg(); cmd.valid = true;
            flyTraj(ox, oy, zt, ffx, ffy, 0, visYawArg(), dt);
            if ((this->now() - vis_hold_t_).seconds() >= vision_hold_secs_) {
                vis_ascend_z_ = odom_z_;
                vis_state_ = VIS_ASCEND;
                RCLCPP_INFO(this->get_logger(),
                    "vision: id=%d 投放后停留完成 → 以 %.2fm/s 升回探索高度(期间不处理视觉数据)",
                    vis_target_id_, ascend_speed_);
            }
            return true;
        }
        case VIS_ASCEND: {
            // 匀速升回探索高度: 升到位才算"完成本趟"(否则会出现"边爬边飞下一个目标")。
            // 水平保持在投放点正上方, 期间 vision_cb 丢弃所有视觉数据。
            const double z_tgt = sp_h;
            vis_ascend_z_ += ascend_speed_ * dt;
            if (vis_ascend_z_ > z_tgt) vis_ascend_z_ = z_tgt;
            double ox, oy;
            recordedDropDronePos(ox, oy);
            // ★保持段同样加偏差比例速度前馈(见 VIS_ALIGN 注释): 否则漂出去一点要爬很久
            double ffx = 0, ffy = 0;
            {
                constexpr double KFF = 1.0;
                constexpr double VMAX_FF = 0.30;
                ffx = KFF * (ox - odom_x_);
                ffy = KFF * (oy - odom_y_);
                const double vm = std::hypot(ffx, ffy);
                if (vm > VMAX_FF) { ffx *= VMAX_FF / vm; ffy *= VMAX_FF / vm; }
                else if (vm < 1e-3) { ffx = 0; ffy = 0; }
            }
            cmd.pos_x = ox; cmd.pos_y = oy; cmd.pos_z = vis_ascend_z_;
            cmd.vel_x = ffx; cmd.vel_y = ffy; cmd.vel_z = 0;
            cmd.yaw = std::isnan(visYawArg()) ? navYaw() : visYawArg(); cmd.valid = true;
            flyTraj(ox, oy, vis_ascend_z_, ffx, ffy, 0, visYawArg(), dt);
            if (odom_z_ >= z_tgt - 0.05 || vis_ascend_z_ >= z_tgt - 1e-6) {
                vis_hold_t_ = this->now();
                vis_state_ = VIS_SETTLE;
                RCLCPP_INFO(this->get_logger(),
                    "vision: id=%d 已升回探索高度 %.2fm → 位置环悬停 %.1fs 稳定后再继续",
                    vis_target_id_, hgit_, ascend_hold_secs_);
            }
            return true;
        }
        case VIS_SETTLE: {
            // 回到巡航高度后先"位置环悬停"稳定(位置保持 + 速度前馈0), 不处理视觉数据;
            // 稳定结束才完成本趟 → 恢复探索 / 飞下一个目标。
            double ox, oy;
            recordedDropDronePos(ox, oy);
            cmd.pos_x = ox; cmd.pos_y = oy; cmd.pos_z = sp_h;
            cmd.vel_x = 0; cmd.vel_y = 0; cmd.vel_z = 0;
            cmd.yaw = std::isnan(visYawArg()) ? navYaw() : visYawArg(); cmd.valid = true;
            flyTraj(ox, oy, sp_h, 0, 0, 0, visYawArg(), dt);
            if ((this->now() - vis_hold_t_).seconds() >= ascend_hold_secs_) {
                RCLCPP_INFO(this->get_logger(),
                    "vision: id=%d 悬停稳定完成 → 完成本趟, 恢复探索/视觉处理",
                    vis_target_id_);
                visionFinishTarget();
            }
            return true;
        }
    }
    return false;
    }

    /// 到任一已投放点的最近距离(m), 仅用于日志
    double visionDedupDist(double x, double y) const {
        double dmin = 1e9;
        for (const auto& q : vis_landed_xy_) dmin = std::min(dmin, std::hypot(x - q.first, y - q.second));
        return dmin;
    }

    /// 飞机是否还在"最近一次投放点"附近(投完还没飞走)。
    /// 用途: 这种时候相机正贴着刚投过的目标看, 近距离/大角度下深度投影误差可能
    /// >1m, 光靠"目标位置去重"会漏 → 干脆一律不记录, 从源头避免同一目标再被记录。
    /// 只在投放后 drop_ignore_secs_ 内生效(避免以后路过同一片区域一直不记录)。
    bool nearRecentDropPoint() const {
        if (!vis_drop_pt_valid_) return false;
        if ((this->now() - vis_drop_pt_t_).seconds() > drop_ignore_secs_) return false;
        return std::hypot(odom_x_ - vis_drop_pt_x_, odom_y_ - vis_drop_pt_y_) < drop_ignore_radius_;
    }

    /// 已完成类别集合的字符串(诊断日志用)
    std::string doneSetStr() const {
        std::string t;
        for (int c : vis_done_) { if (!t.empty()) t += ","; t += std::to_string(c); }
        return t.empty() ? "-" : t;
    }

    /// 中位数(拷贝后 nth_element; 不取平均: 可能有误差很大的点)
    static double medianOf(std::vector<double> v) {
        if (v.empty()) return 0.0;
        const size_t n = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + n, v.end());
        return v[n];
    }
    /// 用"降落前记录的目标点(中位数)"反算无人机应该到的位置:
    /// 记录的是投放口位置, 所以无人机中心 = 记录点 - R(yaw)*offset
    void recordedDropDronePos(double& dx, double& dy) const {
        if (!drop_point_valid_) {
            double wx, wy;
            bombOffsetWorld(vis_port_, wx, wy);          // ★内部用【实际yaw】旋转口偏移
            dx = vis_tx_ - wx; dy = vis_ty_ - wy;
            return;
        }
        double wx, wy;
        bombOffsetWorld(vis_port_, wx, wy);              // ★内部用【实际yaw】旋转口偏移
        dx = drop_px_ - wx;
        dy = drop_py_ - wy;
    }
    void logBombDrop(const char* tag, double e, double v) {
        const int port = vis_port_;
        RCLCPP_INFO(this->get_logger(),
            "vision: id=%d %s: 口%d 水平(x,y)误差=%.3fm 速度=%.3fm/s 高度%.2fm → %s bomb_%d ×%d",
            vis_target_id_, tag, port, e, v, drop_height_,
            Arduino::isSim() ? "[仿真]" : "调用串口", port, drop_repeat_);
        // ★投放口落点校验: 机体位置 + 口偏移(按锁定的低空yaw转到SLAM系) = 投放口实际落点,
        //   和"降落前记录的目标基准点"比 —— 现场一眼确认"口是否真的对上了标靶"。
        {
            double wx, wy;
            bombOffsetWorld(port, wx, wy);
            const double px = odom_x_ + wx, py = odom_y_ + wy;
            const double bx = drop_point_valid_ ? drop_px_ : vis_tx_;
            const double by = drop_point_valid_ ? drop_py_ : vis_ty_;
            RCLCPP_INFO(this->get_logger(),
                "vision: id=%d 投放口落点: 机体(%.2f,%.2f) + 偏移(%.2f,%.2f) = 口(%.2f,%.2f) | 目标基准(%.2f,%.2f) 偏差 %.3fm",
                vis_target_id_, odom_x_, odom_y_, wx, wy, px, py, bx, by,
                std::hypot(px - bx, py - by));
        }
    }

    /// 本目标流程结束: 记已投放点/类别, 计数 +1; 飞满则直接返航, 否则回探索。
    void visionFinishTarget() {
        // ★ 注意: 类别(vis_done_)/投放点(vis_landed_xy_)/计数(vis_count_) 已经在
        //   VIS_ALIGN 调用串口"之前"登记过了(见那里的 ★★ 注释), 这里只做收尾,
        //   绝对不能再登记一次 —— 否则每投一次计数会 +2、端口会跳号。
        const bool full = (vis_count_ >= vision_total_);
        RCLCPP_INFO(this->get_logger(),
            "vision: id=%d 完成 (%d/%d 已投放) 端口%d 目标(%.2f,%.2f) 已完成类别[%s] → %s",
            vis_target_id_, vis_count_, vision_total_, vis_port_, vis_tx_, vis_ty_,
            doneSetStr().c_str(),
            full ? "标靶已飞满, 直接返航" : "resume exploration");
        vis_state_ = VIS_IDLE;
        visionRelease();                                // 任务完成 → 释放锁
        if (full) {
            planner_.setHoldHome(false);                // 解除暂缓
            planner_.forceGoHome();                     // 不等探索阈值, 直接返航
        } else {
            planner_.restartExplore();                  // 恢复探索: 重新检测/规划
        }
    }

    /// 投放口在雷达 SLAM 系下的偏移: 机体偏移 (bx,by) 按当前 yaw 旋转。
    /// port = 投放口编号(0/1/2), 与目标类别号无关, 按飞到的顺序 0→1→2。
    /// 低空段使用的偏航: 一旦锁定(进入降落后)就不再跟随测量值 ——
    /// ★命令"测量到的yaw"等于让 PX4 追着估计值跑(SLAM yaw漂移会变成持续慢转),
    ///   而且世界系投放偏移点会随 yaw 画圈 → "边转边对准、对准了又被转偏"。
    /// ★视觉段发偏航用这个: 锁存后【只发一次】, 之后返回 NaN(不发偏航字段)
    ///   —— 现场实测: 每拍都发同一个 yaw 反而会让机头轻微来回晃; 不发就不变。
    double visYawArg() {
        if (vision_lock_yaw_ && vis_yaw_locked_) {
            if (!vis_yaw_sent_) { vis_yaw_sent_ = true; return vis_lock_yaw_; }
            return std::numeric_limits<double>::quiet_NaN();
        }
        return lowAltYaw();
    }

    double lowAltYaw() const {
        // ★发给飞控的偏航必须是【固定值】, 绝不能每拍发测量值 —— 那等于追自己的噪声,
        //   机头会小幅来回晃 → 视觉坐标跟着晃 → 投放偏。
        //   vision_lock_yaw=1(推荐): 到中心锁存那一刻的角度; 未锁存前用上一次导航角(也是固定值)。
        //   vision_lock_yaw=0: 旧行为(发 odom_yaw_ 测量值, 会晃 ✗)。
        if (!vision_lock_yaw_) return odom_yaw_;
        return vis_yaw_locked_ ? vis_lock_yaw_ : navYaw();
    }

    /// ★位置稳定判据: 每 0.5s 检查一次, 这 0.5s 内位置变化 < 0.02m 视为"停稳"。
    ///   注意必须用【位置】而不是【速度估计】: 位置估计漂移时速度读数≈0, 会误判为静止 ✗。
    bool posStableNow() {
        const double t = this->now().seconds();
        if (t - vis_stable_t_ >= 0.5) {
            const double moved = std::hypot(odom_x_ - vis_stable_x_, odom_y_ - vis_stable_y_);
            vis_stable_x_ = odom_x_; vis_stable_y_ = odom_y_; vis_stable_t_ = t;
            vis_stable_ok_ = (moved < 0.02 && std::hypot(odom_vx_, odom_vy_) < 0.08);
        }
        return vis_stable_ok_;
    }

    /// ★视觉瞄准点滤波: 逐帧视觉坐标有几厘米抖动(5Hz投影噪声), 直接拿来当设定点会让
    ///   飞机在目标附近来回蹭("在中心附近徘徊")。这里做 死区(小于 deadband 不动) + 一阶低通。
    double aimFilter(double cur, double meas) {
        if (!vis_aim_init_) { cur = meas; return cur; }
        const double d = meas - cur;
        if (std::fabs(d) < vision_aim_deadband_) return cur;   // 死区: 小抖动不跟
        return cur + vision_aim_alpha_ * d;                    // 低通: 大变化也只走一部分
    }

    /// 悬停/保持时用的偏航: 上一次"明确指定过的"导航方向(不是测量值)
    double navYaw() const { return nav_yaw_ok_ ? nav_yaw_ : odom_yaw_; }

    /// 投放口相对机体的偏移 → 世界系。yaw_use < -1e8 时用"锁定的低空yaw"
    /// ★低空(yaw锁住)后必须用锁定的 yaw 算偏移: 否则 yaw 一漂/一转, 世界系目标点就
    ///   跟着画圈 → "边转边对准、对准了又被转偏"。
    void bombOffsetWorld(int port, double& wx, double& wy, double yaw_use = -1e9) const {
        const int p = std::clamp(port, 0, 2);
        // ★实物微调(机体系 x前/y左, m): 在标定偏移上叠加, 用来校掉现场那种"每次都往
        //   同一个方向偏"的系统性误差(视觉报的是目标近表面 / 口相对机体量得不准等)。
        //   调法: 实物落在中心外 10cm(假设偏在机头方向) → bomb_trim[口] 的 x 填 -0.10。
        // ★5号(最小标靶)用专用偏移 ✓ 别的目标用标准偏移 ✓
        const bool use_small = bomb_small_enabled_ && (vis_target_id_ == 5);
        const auto& off = use_small ? bomb_offsets_small_[p] : bomb_offsets_[p];
        const double bx = off.first  + bomb_trim_[p].first;
        const double by = off.second * bomb_y_sign_ + bomb_trim_[p].second;
        // ★默认用【实际测量 yaw】(odom_yaw_): 锁定 yaw 只是"不再主动转", 实际机头与锁存值
        //   可能有偏差 —— 而投放口相对机体的偏移必须按实际机头旋转到世界系, 否则口会按
        //   锁存角度偏过去。锁存 yaw 仅用于"悬停时保持机头指向"。
        const double y  = (yaw_use < -1e8) ? odom_yaw_ : yaw_use;
        const double cy = std::cos(y), sy = std::sin(y);
        wx = bx * cy - by * sy;
        wy = bx * sy + by * cy;
    }
    /// 解析 "0,1,5" → {0,1,5}(保持参数里的顺序 = 优先级顺序)
    static std::vector<int> parseClassList(const std::string& s) {
        std::vector<int> out;
        std::string cur;
        for (char ch : s + ",") {
            if (ch == ',' || ch == ' ' || ch == ';') {
                if (!cur.empty()) { out.push_back(std::stoi(cur)); cur.clear(); }
            } else {
                cur.push_back(ch);
            }
        }
        return out;
    }
    static bool inList(const std::vector<int>& v, int id) {
        return std::find(v.begin(), v.end(), id) != v.end();
    }
    /// 解析投放口机体偏移 "0,-0.2;0.2,0;0,0.2" → {{0,-0.2},{0.2,0},{0,0.2}}
    /// (顺序就是投放口编号 0/1/2; 少于 3 项的用 (0,0) 兜底)
    static std::array<std::pair<double,double>, 3> parseBombOffsets(const std::string& s) {
        std::array<std::pair<double,double>, 3> out{{{0.0,0.0},{0.0,0.0},{0.0,0.0}}};
        size_t idx = 0;
        std::string cur;
        auto flush = [&]() {
            if (cur.empty() || idx >= 3) { cur.clear(); return; }
            size_t comma = cur.find(',');
            if (comma != std::string::npos) {
                try {
                    out[idx].first  = std::stod(cur.substr(0, comma));
                    out[idx].second = std::stod(cur.substr(comma + 1));
                    ++idx;
                } catch (...) { }
            }
            cur.clear();
        };
        for (char ch : s) {
            if (ch == ';') flush();
            else if (ch != ' ' && ch != '\t') cur.push_back(ch);
        }
        flush();
        return out;
    }
    /// 按类别策略挑下一个要飞的视觉目标(调用时必须已持 vis_mtx_)。
    ///  * 优先类别(0,1,5): 不按类别号排序 —— 谁**先被识别到**就先飞谁(FIFO,
    ///    用首次入槽的 seq 比较);
    ///  * 回退类别(3,2): 仅在探索进度达标解锁后启用, 按参数里的顺序(3 先于 2);
    ///  * 都没有 → 返回 false(没有可飞目标)。
    bool pickVisionTarget(bool fallback_ok, int& id, double& x, double& y) const {
        // 1) 优先类别: 取"首次识别时间最早"的那个
        int  best = -1;
        long best_seq = 0;
        for (int c : vis_priority_) {
            if (vis_done_.count(c)) continue;
            auto it = vis_slot_.find(c);
            if (it == vis_slot_.end()) continue;
            if (best < 0 || it->second.seq < best_seq) { best = c; best_seq = it->second.seq; }
        }
        if (best >= 0) {
            const auto& s = vis_slot_.at(best);
            id = best; x = s.x; y = s.y;
            return true;
        }
        // 2) 回退类别: 按参数顺序(默认 3 → 2)
        if (fallback_ok) {
            for (int c : vis_fallback_) {
                if (vis_done_.count(c)) continue;
                auto it = vis_slot_.find(c);
                if (it == vis_slot_.end()) continue;
                id = c; x = it->second.x; y = it->second.y;
                return true;
            }
        }
        return false;
    }
    /// 是否还有"未飞且已知坐标"的目标可按策略飞。
    bool visionHasCandidate() const {
        const bool fallback_ok = (planner_.exploredPct() >= vision_fallback_pct_);
        std::lock_guard<std::mutex> lk(vis_mtx_);
        for (int c : vis_priority_) if (!vis_done_.count(c) && vis_slot_.count(c)) return true;
        if (fallback_ok)
            for (int c : vis_fallback_) if (!vis_done_.count(c) && vis_slot_.count(c)) return true;
        return false;
    }
    /// 是否要"暂缓自动返航": 标靶没飞满, 且(还有可飞目标 或 进度还没到回退阈值)。
    /// 进度过阈值又没有任何可飞目标 → 不暂缓, 让探索器按原逻辑返航(不会死等);
    /// 已经到探索器自己的返航阈值(explore_threshold)时也不再拖。
    bool visionHoldHome() const {
        // ★★ 正在执行视觉任务(飞向目标/投放中) → 绝不让探索器启动返航 ✓
        //    原因: 目标被选中后就从 vis_slot_ 移除 ✗ → visionHasCandidate() 变 false ✗
        //    → 覆盖率一旦 ≥ explore_threshold 就会把【正在飞的任务】中止 ✗ (现场必然发生 ✓)
        if (vis_target_id_ >= 0) return true;
        if (vis_count_ >= vision_total_) return false;
        if (visionHasCandidate()) return true;
        if (planner_.exploredPct() >= planner_.home_pct_) return false;
        return planner_.exploredPct() < vision_fallback_pct_;
    }

    /// 原地降落: 发 /mavros/cmd/land(MAV_CMD_NAV_LAND, lat/lon=0 表示当前位置,
    /// 所以是垂直原地降落)。仅用于任务正常结束(padB / 飞完第一个口)那一次;
    /// 若被拒则退回用 set_mode 切 AUTO.LAND。等待 disarm 有 60s 超时, 避免卡死。
    void land(rclcpp::Rate& rate){
	    RCLCPP_INFO(this->get_logger(), "Initiating MAVROS land (in place)...");
	    
	    // 构造降落请求
	    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
	    req->min_pitch = 0;           // 最小俯仰角
	    req->yaw = 0;                 // 偏航角（可选）
	    req->latitude = 0;            // 0 表示当前位置
	    req->longitude = 0;
	    req->altitude = 0;
	    
	    // 发送异步请求
	    auto future = landing_client_->async_send_request(req);
	    bool accepted = false;
	    // 等待服务响应
	    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == 
		rclcpp::FutureReturnCode::SUCCESS) {
		auto resp = future.get();
		if (resp->success) {
		    accepted = true;
		    RCLCPP_INFO(this->get_logger(), "Land command accepted, descending...");
		} else {
		    RCLCPP_WARN(this->get_logger(), "Land command failed: %d", resp->result);
		}
	    } else {
		RCLCPP_ERROR(this->get_logger(), "Land service call timed out");
	    }
	    if (!accepted) {
		// 兜底: 直接切 PX4 的 AUTO.LAND 模式(同样垂直原地降落)
		RCLCPP_WARN(this->get_logger(), "land cmd rejected → try set_mode AUTO.LAND");
		auto mreq = std::make_shared<mavros_msgs::srv::SetMode::Request>();
		mreq->custom_mode = "AUTO.LAND";
		set_mode_client_->async_send_request(mreq);
	    }
	    
	    // 等待状态变为 disarmed(带超时, 避免服务/模式异常时死等)
	    auto lt0 = this->now();
	    while(rclcpp::ok() && state_.armed){
		rclcpp::spin_some(this->get_node_base_interface());
		rate.sleep();
		if ((this->now() - lt0).seconds() > 60.0) {
		    RCLCPP_ERROR(this->get_logger(),
			"still armed after 60s (mode=%s, z=%.2f) — giving up waiting; check FC/RC",
			state_.mode.c_str(), odom_z_);
		    return;
		}
	    }
	    RCLCPP_INFO(this->get_logger(), "Landed and disarmed.");
	}

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    mavros_msgs::msg::State state_;
    // ===== 视觉目标(geometry_msgs/Point, x,y=坐标, z=类别id) =====
    std::string vision_topic_prefix_ = "/detected_object_class";
    std::vector<int> vis_class_list_;                       // 订阅的类别(默认 0,1,2,3,5)
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr> vision_subs_;
    int vis_mismatch_ = 0;                                 // 话题类别 vs 消息z 不一致计数
    mutable std::mutex vis_mtx_;
    double vis_x_ = 0, vis_y_ = 0;
    VisState vis_state_ = VIS_IDLE;
    int    vis_target_id_ = -1;
    double vis_tx_ = 0, vis_ty_ = 0;
    rclcpp::Time vis_hold_t_;
    std::set<int> vis_done_;          // 已飞过的类别(=id)
    // 已降落过的点位(雷达系). 只按类别去重不够: 同一物体换个类别就是新 id,
    // 会在原地再降一次 → 用位置去重兜住。
    std::vector<std::pair<double,double>> vis_landed_xy_;
    double origin_reject_radius_ = 1.00;   // ★原点附近(雷达系)的5号不飞(H标志误识别)
    double vision_dedup_r_     = 0.5;   // 位置去重半径(m)
    // 各类别的"最新坐标"槽(未飞的候选目标); 优先类别按"首次识别时间"选
    struct VisSlot { double x; double y; long seq; };
    std::map<int, VisSlot> vis_slot_;
    long   vis_seq_            = 0;     // 首次识别序号(越小越先被识别到)
    // ===== 视觉类别策略(launch 可配, 顺序=优先级) =====
    std::vector<int> vis_priority_;     // 优先类别, 默认 {0,1,5}
    std::vector<int> vis_fallback_;     // 回退类别, 默认 {3,2}
    std::vector<int> vis_filter_;       // 直接过滤(误识别), 默认 {4}
    int    vision_total_       = 3;     // 总任务: 飞满几个标靶就返航
    bool   fov_mark_while_vision_ = true;   // ★飞视觉目标期间是否继续做FOV覆盖标记
    double vision_fallback_pct_ = 80.0; // 探索进度达到该值后才允许飞回退类别
    int    vis_count_          = 0;     // 已完成(投放)的标靶数
    int    vis_port_           = 0;     // 本次目标的投放口编号(锁定目标时固定)
    bool   vis_drop_pt_valid_  = false; // 刚投放完(离开窗口用)
    double vis_drop_pt_x_ = 0, vis_drop_pt_y_ = 0;   // 投放点(目标位置)
    rclcpp::Time vis_drop_pt_t_;        // 投放时刻
    double drop_ignore_radius_ = 1.2;   // 飞机距投放点这么近时视觉数据不记录(m)
    double drop_ignore_secs_   = 15.0;  // 上面这条的生效时长(s)
    int    vis_skip_log_       = 0;     // 过滤提示节流
    // ===== 投放(串口 + 三个投放口) =====
    int         use_serial_       = 0;      // 1=真机开串口
    std::string serial_port_      = "/dev/ttyS3";
    int         serial_baud_      = 115200;
    double      serial_ready_     = 3.0;    // 打开后多久才能用
    double      drop_hold_secs_   = 1.0;    // 两次投放最短间隔
    std::array<std::pair<double,double>, 3> bomb_offsets_small_ = {{{0.0,0.0},{0.0,0.0},{0.0,0.0}}};  // ★5号专用
    bool bomb_small_enabled_ = false;   // ★5号专用偏移是否启用
    std::array<std::pair<double,double>, 3> bomb_offsets_ =
        {{{0.0, -0.2}, {0.2, 0.0}, {0.0, 0.2}}};   // 投放口相对机体位置(m)
    // ★实物微调(机体系 x前/y左, m): 叠加到 bomb_offsets_ 上, 校掉系统性投偏
    std::array<std::pair<double,double>, 3> bomb_trim_ =
        {{{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}};
    double      bomb_y_sign_      = 1.0;    // 机体 y 轴方向(ROS左正=+1, FRD右正=-1)
    double      drop_final_tol_small_   = 0.020;  // ★5号(最小标靶)专用: 更严
    double      drop_relax_mult_small_  = 1.5;    // ★5号专用: 超时放宽倍数
    double      drop_final_tol_   = 0.03;   // 降落后对准判定(m, 严格)
    double      drop_height_      = 0.7;
    double      land_settle_secs_ = 2.0;   // 最终降落到点后, 位置环悬停对准保持时长(s)    // 降落到的目标高度(世界系, m)
    double      descent_speed_    = 0.3;    // 降落速度(m/s)
    double      ascend_speed_     = 0.3;    // 升回探索高度速度(m/s)
    double      ascend_hold_secs_ = 2.0;    // 升回高度后位置环悬停稳定时长(s)
    double      vis_ascend_z_     = 0.0;    // 上升设定点(按 ascend_speed 推进)
    int         drop_align_frames_= 10;     // 降落前记录对准位置取几帧
    std::vector<double> vis_align_x_, vis_align_y_;  // 降落前对准位置采样
    double      drop_px_ = 0, drop_py_ = 0; // 记录的目标点(投放口位置中位数)
    bool        drop_point_valid_ = false;
    double      vis_descend_z_    = 0.0;    // 降落设定点(按 descent_speed 推进)
    double      drop_try_secs_    = 10.0;   // 【低空】严格对准预算(s), 从进入 ALIGN 计时
    int         drop_repeat_      = 4;      // 投放时同一口连发次数(防单次没触发)
    int         drop_repeat_gap_ms_ = 30;   // 连发间隔(ms)
    double      drop_vel_tol_     = 0.05;   // 投放前速度刹住判定(m/s)
    double      drop_settle_secs_ = 0.3;    // 【低空/投放前】稳定时长(s): 达标持续这么久才投放
    double      drop_height_tol_  = 0.05;   // 到降落高度判定(m): 没到绝不投放
    rclcpp::Time vis_align_t_;              // 进入 VIS_ALIGN 的时刻(独立超时)
    int    vis_align_warn_ = 0;             // ALIGN 警告节流
    double alt_hold_t_ = 0.0;               // 降到通道高度已等待时长(s)
    int    alt_hold_log_ = 0;               // 该阶段日志节流
    bool   corner_h_done_ = false;          // ★降到通道高度是否已判过(锁存, 不再反复关闸门)
    double corner_h_tol_ = 0.10;            // 到位判定(m)
    double corner_h_wait_secs_ = 6.0;       // 最多等多久(s)
    bool   vis_path_ready_ = false;         // 视觉目标的后台A*路径是否已就绪
    rclcpp::Time vis_path_wait_t_;          // 提交后台规划的时刻(等路径超时用)
    rclcpp::Time vis_state_t_;              // 进入当前视觉状态的时刻(断流兜底用)
    int    vis_prev_state_ = -1;            // 上一拍的状态(用于检测状态变化)
    long   vis_rx_seq_ = 0;                 // 锁定类别的视觉帧序号(每收到一帧+1)
    long   vis_align_last_seq_ = 0;         // 上次采中位数时的视觉帧序号
    bool        vis_drop_called_  = false;  // ★ 本目标是否已经投放过(一个目标只投一个口)
    rclcpp::Time vis_drop_settle_t_;        // VIS_DROP 稳定计时
    rclcpp::Time vis_hit_t_;                // VIS_OFFSET 稳定计时
    double vision_hold_secs_   = 1.5;
    double vision_reach_tol_   = 0.10;
    double vision_timeout_     = 1.5;   // 视觉数据中断超时(s), 用于其它判定
    double vision_lost_secs_   = 6.0;   // ★失联多久才放弃目标(s): 期间一直朝最后已知点飞
    double corridor_height_    = 0.7;   // 返航角点后(通道段)的飞行高度(世界系, m)
    rclcpp::Time vis_last_rx_;          // 最近一次收到视觉数据的时间
    double drop_offset_start_dz_ = 0.0;                  // ★下降末段提前开始横向偏移(m)
    double drop_align_kff_ = 1.0, drop_align_vmax_ = 0.30;  // ★低空对准 增益/限速(1.0/0.30=旧版力度)
    bool   drop_xy_only_ = false;                        // ★只看x,y对准就投(忽略高度/速度)
    int    drop_xy_hold_ticks_ = 20;                    // 连续多少控制帧(50Hz)
    int    vis_xy_ok_cnt_ = 0, vis_xy_log_ = 0;          // 连续达标帧计数/日志节流
    bool   drop_base_from_vision_ = true;                // ★基准点=视觉坐标中位数
    double vision_aim_deadband_ = 0.0;                   // ★瞄准点死区(m): 0=不过滤(旧版)
    double vision_aim_alpha_    = 1.0;                   // ★瞄准点低通(1.0=不过滤, 旧版)
    bool   vis_aim_init_        = false;                 // 瞄准点滤波是否已初始化
    int    vision_approach_mode_ = 0;                    // ★默认0=旧流程(现场验证过的) 1=新流程(0.7m先稳)
    std::vector<double> vis_near_x_, vis_near_y_;       // ★0.7m稳定后采的【视觉坐标】样本
    long   vis_near_seq_ = -1;                           // 采样用的视觉帧号
    double drop_offset_tol_ = 0.06;                      // ★高空: 投放口↔目标中心 判据(m)
    double drop_offset_try_secs_ = 10.0;                 // ★高空对准独立超时(s)
    bool   vision_lock_yaw_ = true;                      // ★1=锁定机头(默认) 0=旧行为(yaw跟着路径变)
    double vision_offset_speed_ = 0.10;                  // ★高空飞偏移的横向速度上限(m/s)
    double vision_near_kff_  = 1.2;                      // ★比例前馈增益(1/s)
    double vision_near_vmax_ = 0.50;                     // ★前馈速度上限(m/s)
    double vision_near_vmin_ = 0.10;                     // ★前馈速度下限(m/s)
    rclcpp::Time vis_arrive_t_;                          // ★到阈值那一瞬间(悬停计时)
    bool   vis_sample_logged_ = false;                   // "悬停结束→开始采样"日志节流
    bool   vis_yaw_sent_ = false;                        // ★锁存后的 yaw 是否已发过一次
    bool   vis_near_unsafe_logged_ = false;              // ★"近距直线不安全→改规划"日志节流
    double vision_slow_dist_ = 0.30;                     // (旧: 慢蹭距离, 已不用)
    double vision_slow_speed_ = 0.05;                    // ★慢蹭速度(m/s)
    bool   vis_slow_logged_ = false;                     // 减速日志只打一次
    double vis_stable_x_ = 0.0, vis_stable_y_ = 0.0;     // 位置稳定判据: 上次采样点
    double vis_stable_t_ = 0.0;                          // 上次检查时刻
    bool   vis_stable_ok_ = false;                       // 是否停稳
    double vision_near_dist_ = 0.70;                     // ★到目标这么近先悬停稳定
    double vision_near_hold_secs_ = 0.5;                 // ★稳定时长(s)
    bool   vis_near_done_ = false;                       // 本次目标是否已做过"先稳一下"
    rclcpp::Time vis_near_t_;                            // 稳定起始时刻
    double vis_hold_x_ = 0.0, vis_hold_y_ = 0.0, vis_hold_yaw_ = 0.0;  // ★悬停锁存点(不追自己位置)
    bool   vis_hold_latched_ = false;
    rclcpp::Time vis_offset_t_;                          // ★旧模式进入VIS_OFFSET时刻(先悬停计时)
    bool   vis_offset_hold_logged_ = false;
    bool   vis_offset_timeout_logged_ = false;   // 高空对准超时日志只打一次
    double vision_nopath_free_m_ = 0.60;                 // ★近距离免规划阈值(m)
    double vis_align_diag_t_ = 0.0;                      // ★低空对准每秒诊断计时
    rclcpp::Time vis_nopath_last_;                       // ★无路径时重规划节流
    rclcpp::Time vis_nopath_t_;                          // ★无可用路径的起始时刻(2s后放弃)
    double vis_descend_x_ = 0.0, vis_descend_y_ = 0.0;   // ★降落时锁存的水平机位(原地垂直下降)
    double nav_yaw_ = 0.0;                  // ★"导航偏航"(明确指定过的方向, 悬停时重复它)
    bool   nav_yaw_ok_ = false;
    bool   vis_yaw_locked_ = false;         // ★低空段偏航是否已锁定(不再跟随测量yaw)
    double vis_lock_yaw_   = 0.0;           // 锁定时的偏航(rad)
    bool   vis_live_seen_ = false;          // 当前锁定目标是否收到过实时帧
    int    vis_jump_skip_ = 0;              // 跳变丢弃计数(日志节流)
    double vision_jump_tol_ = 1.5;          // 位置跳变判定(m)
    bool   vis_rx_ever_ = false;
    double vis_replan_t_ = 0.0;         // VIS_FLY 期间的重规划节流
    // 50Hz 独立设定点发布(防 OFFBOARD 超时掉线): 主循环只更新 sp_msg_,
    // 由 spLoop 线程持续发最新指令。
    std::mutex sp_mtx_;
    mavros_msgs::msg::PositionTarget sp_msg_;
    bool sp_ready_ = false;
    std::atomic<bool> sp_run_{false};
    std::thread sp_thread_;
    int map_cnt_ = 0;            // 点云→障碍图 次数(统计实际频率, 应≈点云频率)
    int fov_cnt_ = 0;            // FOV覆盖标记 次数(统计实际频率)
    int fov_divider_ = 5;        // FOV覆盖每几拍跑一次(50Hz/5 = 10Hz)
    int fov_tick_ = 0;           // 分频计数
    rclcpp::Time map_t_;         // 上次统计时刻
    rclcpp::Time last_offboard_try_;   // OFFBOARD 重入节流
    int  offboard_retry_ = 0;
    double odom_x_=0,odom_y_=0,odom_z_=0,odom_yaw_=0;
    double odom_vx_=0,odom_vy_=0,odom_vz_=0,odom_vyaw_=0;
    // Yaw-ramp state for position-mode flyTraj (EGO-style yaw + rate feedforward).
    bool   yaw_cmd_init_ = false;
    double yaw_cmd_ = 0.0;
    double yaw_tgt_prev_ = NAN;   // last target yaw handed to flyTraj (jump detect)
    rclcpp::Time yaw_cmd_last_;
    int    yaw_dbg_ = 0;   // TEMP debug counter, remove after fix
    double odom_yaw_ts_ = 0.0;    // yaw-rate-from-heading-diff state
    double odom_yaw_prev_ = 0.0;
    // Origin baseline = mean of the first ORIGIN_FRAMES odometry frames.
    // Map/planner/GUI stay in raw odom frame; setpoint targets add the origin
    // back: setpoint = target + origin (e.g. z: origin_z + rel_height).
    static constexpr int ORIGIN_FRAMES = 10;
    double origin_x_=0, origin_y_=0, origin_z_=0;
    double sum_x_=0, sum_y_=0, sum_z_=0;
    int origin_frames_=0;
    bool origin_ready_=false;
    sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_; bool cloud_ready_=false;
    double lidar_range_,height_band_,hgit_,fov_max_range_,takeoff_speed_;
    double explore_cloud_h_up_ = 0.10, explore_cloud_h_down_ = 0.40;  // ★探索段 相对当前高度 上/下
    double ab_cloud_h_up_ = 0.00, ab_cloud_h_down_ = 0.20;             // ★A→B段 相对当前高度 上/下
    double ab_cloud_h_band_ = 0.30;   // ★A→B段: 无人机高度附近取点云的半带宽(m)
    // ★返航固定高度范围点云
    bool   use_return_cloud_ = true;
    double return_cloud_h_min_ = 0.30, return_cloud_h_max_ = 0.60;
    bool   return_cloud_clear_map_ = true;
    bool   fixed_cloud_on_ = false;      // 当前这一拍是否用固定高度带建图
    int    ret_cloud_warn_ = 0;          // 无效范围警告节流
    // ★虚拟通道右墙(自建场地无围墙时)
    bool   use_corridor_wall_ = false;
    double corridor_wall_x_ = 0.0, corridor_wall_y_ = 0.0;
    bool   use_corridor_wall2_ = false;              // ★第二道虚拟墙(逻辑同第一道)
    double corridor_wall2_x_ = 0.0, corridor_wall2_y_ = 0.0;
    int    vwall_log_ = 0;               // 虚拟墙日志节流
    double gui_fov_range_ = 5.0;   // GUI 绿色扇形射线长度(仅显示, 不影响覆盖标记)
    double fov_h_deg_=-1.0;   // 前视水平FOV(度), ≤0 = 按内参自动
    double fov_tilt_deg_=55.0;// 前视光轴下倾角(度,相对水平面)
    double map_x_min_,map_x_max_,map_y_min_,map_y_max_,marg_x_,marg_y_;
    // World-frame (launch) values of map / explore / pads, BEFORE the origin
    // shift is applied. launch gives WORLD coordinates; the radar SLAM odom /
    // point cloud live in the radar frame. Once origin_ (radar origin in world)
    // is known we add it to every launch value so all entities share one frame.
    double w_map_x_min_=-1, w_map_x_max_=10, w_map_y_min_=-6, w_map_y_max_=6;
    double w_explore_x_min_=-1, w_explore_x_max_=10;
    double w_explore_y_min_=-6, w_explore_y_max_=6;
    double w_pad_a_x_=8.5, w_pad_a_y_=4.5, w_pad_b_x_=8.5, w_pad_b_y_=-4.5;
    int    w_flight_mode_=0;
    int    ab_path_mode_=0;        // 通道路径方式: 0=保守A*, 1=先x再y
    double ab_axis_overshoot_=0.7;
    double ab_axis_lead_=1.2;    // ★提前并线距离(m) // mode1: 过口后再多走多少米
    double ab_axis_x_speed_=0.25;  // mode1: 横移(x)段限速(m/s)
    double w_channel_speed_=0.5;
    bool   origin_shift_applied_=false;
    // GUI drone marker, kept in the shared world frame (odom+origin).
    double gui_drone_x_=0, gui_drone_y_=0;
    int cam_w_, cam_h_;
    double cam_fx_, cam_fy_, cam_cx_, cam_cy_;
    Px4ExplorationPlanner planner_; int report_=0;
    std::unique_ptr<GuiRenderer> gui_; std::mutex gui_mutex_; bool use_gui_=false;
    bool use_global_cloud_=true;
    double cloud_max_hz_=1.0;
    int    cloud_subsample_=8;   // ★建图抽样倍数
    bool   acc_all_cloud_=true;   // ★全场累加点云(1=不清探索区外 ✓)
    bool   map_started_=false;    // ★起飞悬停结束后才允许使用点云建图 ✓
    std::chrono::steady_clock::time_point last_cloud_proc_t_{};
    bool use_downward_fov_=true;  // true=向下FOV, false=前向FOV
};

int main(int argc,char**argv){rclcpp::init(argc,argv);auto n=std::make_shared<ExplorationNode>();n->run();rclcpp::shutdown();return 0;}
