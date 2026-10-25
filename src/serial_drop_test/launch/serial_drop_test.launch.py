"""
投放串口测试 launch —— 单独验证串口 + 三个投放口(1→2→0 依次投放)。

用法:
    ros2 launch serial_drop_test serial_drop_test.launch.py
    ros2 launch serial_drop_test serial_drop_test.launch.py use_serial:=0        # 只打日志(不连串口)
    ros2 launch serial_drop_test serial_drop_test.launch.py serial_port:=/dev/ttyUSB0

协议(与 px4_exploration/src/Arduino.cpp 一致): 4 字节帧 {0xfe,0xfd,id,id%2}
    bomb_0 → id=1, bomb_1 → id=2, bomb_2 → id=3
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_drop_test',
            executable='serial_drop_test_node',
            name='serial_drop_test_node',
            output='screen',
            parameters=[{
                # ===== 串口配置 =====
                'use_serial': 1,                # 1=真机: 打开串口投放; 0=只打日志(桌面自测)
                'serial_port': '/dev/ttyS3',    # 串口设备名
                'serial_baud': 115200,          # 波特率
                'serial_ready_secs': 3.0,       # 打开后等这么久才允许发送(真机 MCU 要求 3s)

                # ===== 测试流程 =====
                'test_gap_secs': 1.5,           # 两次投放之间的间隔(秒)
                'test_repeat': 1,               # 每个口投几轮(1 → 1/2/0 各一次, 共3次)
                'test_repeat_times': 4,         # 每轮同一个口连发几次(防单次没触发)
                'test_repeat_gap_ms': 30,       # 连发之间的间隔(ms)
                'exit_after': 1,                # 1=测完自动退出; 0=保持节点运行
            }],
        ),
    ])
