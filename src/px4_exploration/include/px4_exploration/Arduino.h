#pragma once
// ============================================================================
//  投放/外设串口控制(接口与原 Arduino.cpp 一致)
//  协议: 4 字节帧 {0xfe, 0xfd, id, id%2}
//        bomb_0 → id=1, bomb_1 → id=2, bomb_2 → id=3
//  串口: 用 POSIX termios 直接读写 —— 原来的 serial/serial.h 是 ROS1 的包,
//        ROS2 环境下没有, 所以这里不依赖任何外部串口库(仿真/真机都能编译)。
//
//  用法(节点启动时):
//      Arduino::configure(port, baud, use_serial, ready_secs, hold_secs);
//      Arduino::serinit();          // use_serial=true 时打开串口
//      ...                          // 真机要求"打开后 >= ready_secs(3s) 才能用"
//      Arduino::bomb_0();           // 投放口 0
//
//  use_serial=false(仿真): 不打开串口, bomb_* 只打印日志, 方便仿真验证流程。
// ============================================================================

#include <string>

class Arduino {
public:
    /// @param port        串口设备, 例如 "/dev/ttyS3"
    /// @param baud        波特率, 例如 115200
    /// @param use_serial  false = 仿真: 不开串口, bomb_* 只打日志
    /// @param ready_secs  打开串口后要等多久才能用(真机 MCU 要求 3s)
    /// @param hold_secs   两次投放指令之间的最短间隔(原代码 send() 里 sleep(1))
    static void configure(const std::string& port, int baud, bool use_serial,
                          double ready_secs = 3.0, double hold_secs = 1.0);

    static bool isSim();        // true = 仿真(没开串口)
    static bool portOpen();     // 串口是否已打开
    static int  lastCommand();  // 最近一次发送的 id(-1 = 还没发过)

    static void serinit();      // 打开串口(use_serial=false 时只打印)
    static void send();         // 发送当前 arduinoID 对应的帧
    static void bomb_0();       // 投放口 0
    static void bomb_1();       // 投放口 1
    static void bomb_2();       // 投放口 2
    /// 同一个投放口连发 times 次(每两次之间隔 gap_ms) —— 防止单次没被 MCU 触发。
    /// port: 0/1/2; times<=0 时按 1 次。
    static void bombRepeat(int port, int times, int gap_ms = 30);
    static void led_1s();
    static void led_on();
    static void loud();
    static void laser();
    static void oak_up();
    static void oak_down();
    static void bigbomb_1();
    static void bigbomb_2();
    static void bigbomb_3();
};
