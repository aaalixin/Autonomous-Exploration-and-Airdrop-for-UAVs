// 投放/外设串口实现(ROS2 语法)。
// 协议与帧格式与原 Arduino.cpp 完全一致:
//   arduinolink[4] = { 0xfe, 0xfd, arduinoID, arduinoID%2 }
//   bomb_0 → id=1, bomb_1 → id=2, bomb_2 → id=3
// 串口: 用 ROS2 下的 POSIX termios 直接读写(原 serial/serial.h 是 ROS1 的包,
//       ROS2 环境没有该头文件), 因此这里不引入额外依赖, 真机/仿真都能编译。
// 日志: 统一用 RCLCPP_*(rclcpp::get_logger("Arduino"))。
// 时序: serinit() 打开串口; 真机 MCU 要求"打开满 ready_secs(默认3s)后才能用"
//       → send() 里做门控; 原实现 send() 末尾 sleep(1) 改成 hold_secs 内的
//       "只发一次"约束, 不阻塞飞控主循环(投放时 20Hz 设定点线程仍在按住位置)。
#include "px4_exploration/Arduino.h"

#include <rclcpp/rclcpp.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>

namespace {

int         fd_          = -1;
bool        use_serial_  = false;
std::string port_        = "/dev/ttyS3";
int         baud_        = 115200;
double      ready_secs_  = 3.0;    // 打开后多久才能用
double      hold_secs_   = 1.0;    // 两次投放的最短间隔
bool        ever_sent_   = false;
int         last_id_     = -1;
unsigned char arduinoID  = 0;
std::chrono::steady_clock::time_point open_t_{};
std::chrono::steady_clock::time_point last_send_t_{};
std::mutex  mtx_;

rclcpp::Logger logger() { return rclcpp::get_logger("Arduino"); }

double since(const std::chrono::steady_clock::time_point& t) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}

speed_t baudFlag(int b) {
    switch (b) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B115200;
    }
}

}  // namespace

void Arduino::configure(const std::string& port, int baud, bool use_serial,
                        double ready_secs, double hold_secs) {
    std::lock_guard<std::mutex> lk(mtx_);
    port_       = port;
    baud_       = baud;
    use_serial_ = use_serial;
    ready_secs_ = ready_secs;
    hold_secs_  = hold_secs;
}

bool Arduino::isSim()       { return !use_serial_; }
bool Arduino::portOpen()    { return fd_ >= 0; }
int  Arduino::lastCommand() { return last_id_; }

void Arduino::serinit() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!use_serial_) {
        RCLCPP_INFO(logger(), "仿真模式: 不打开串口(%s), bomb_* 只打印日志",
                    port_.c_str());
        return;
    }
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        RCLCPP_ERROR(logger(), "打开串口 %s 失败: %s (投放指令将无法发送)",
                     port_.c_str(), std::strerror(errno));
        return;
    }
    int fl = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, fl & ~O_NONBLOCK);   // 4 字节写不会阻塞

    struct termios tio;
    std::memset(&tio, 0, sizeof(tio));
    if (::tcgetattr(fd_, &tio) != 0)
        RCLCPP_WARN(logger(), "tcgetattr 失败: %s", std::strerror(errno));
    cfsetospeed(&tio, baudFlag(baud_));
    cfsetispeed(&tio, baudFlag(baud_));
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;            // 无硬件流控
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;                 // 8 数据位
    tio.c_cflag &= ~PARENB;             // 无校验
    tio.c_cflag &= ~CSTOPB;             // 1 停止位
    tio.c_iflag &= ~(INLCR | ICRNL | IGNCR | IXON | IXOFF | IXANY);
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_oflag &= ~OPOST;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1;
    if (::tcsetattr(fd_, TCSANOW, &tio) != 0)
        RCLCPP_WARN(logger(), "tcsetattr 失败: %s", std::strerror(errno));
    ::tcflush(fd_, TCIOFLUSH);

    open_t_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(logger(), "串口已打开: %s @ %d 8N1 (打开后 %.1fs 内不发送)",
                port_.c_str(), baud_, ready_secs_);
}

void Arduino::send() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!use_serial_) {
        last_id_ = static_cast<int>(arduinoID);
        RCLCPP_INFO(logger(), "[SIM] 投放指令 id=%u (仿真, 未开串口)", arduinoID);
        return;
    }
    if (fd_ < 0) {
        RCLCPP_ERROR(logger(), "串口未打开, 投放指令 id=%u 被丢弃", arduinoID);
        return;
    }
    // 真机 MCU: 串口打开满 ready_secs_ 后才能用
    double el = since(open_t_);
    if (el < ready_secs_) {
        double w = ready_secs_ - el;
        RCLCPP_WARN(logger(), "串口仅打开 %.2fs (< %.2fs), 等 %.2fs 后再发",
                    el, ready_secs_, w);
        std::this_thread::sleep_for(std::chrono::duration<double>(w));
    }
    // 两次投放的最短间隔(替代原实现的 sleep(1), 但不阻塞主循环)
    if (ever_sent_ && since(last_send_t_) < hold_secs_)
        RCLCPP_WARN(logger(), "距上次投放仅 %.2fs (< %.2fs)",
                    since(last_send_t_), hold_secs_);

    unsigned char arduinolink[4] = {0xfe, 0xfd, 0, 0};
    arduinolink[2] = arduinoID;
    arduinolink[3] = static_cast<unsigned char>(arduinoID % 2);
    ssize_t n = ::write(fd_, arduinolink, sizeof(arduinolink));
    ::tcdrain(fd_);
    last_send_t_ = std::chrono::steady_clock::now();
    ever_sent_   = true;
    last_id_     = static_cast<int>(arduinoID);
    if (n == static_cast<ssize_t>(sizeof(arduinolink)))
        RCLCPP_INFO(logger(), "%s ← 投放帧已发送: id=%u", port_.c_str(), arduinoID);
    else
        RCLCPP_ERROR(logger(), "串口写入异常: 期望 4 字节, 实际 %zd (%s)",
                     n, std::strerror(errno));
}

void Arduino::bomb_0() { arduinoID = 1; Arduino::send(); }
void Arduino::bomb_1() { arduinoID = 2; Arduino::send(); }
void Arduino::bomb_2() { arduinoID = 3; Arduino::send(); }

void Arduino::bombRepeat(int port, int times, int gap_ms) {
    if (times <= 0) times = 1;
    for (int i = 0; i < times; ++i) {
        if      (port <= 0) Arduino::bomb_0();
        else if (port == 1) Arduino::bomb_1();
        else                Arduino::bomb_2();
        if (i + 1 < times && gap_ms > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(gap_ms));
    }
    RCLCPP_INFO(logger(), "投放口%d 连发 %d 次完成", port, times);
}
void Arduino::led_1s() { arduinoID = 4;  Arduino::send(); }
void Arduino::led_on() { arduinoID = 5;  Arduino::send(); }
void Arduino::loud()   { arduinoID = 7;  Arduino::send(); }
void Arduino::laser()  { arduinoID = 8;  Arduino::send(); }
void Arduino::oak_up() { arduinoID = 9;  Arduino::send(); }
void Arduino::oak_down(){ arduinoID = 10; Arduino::send(); }
void Arduino::bigbomb_1(){ arduinoID = 11; Arduino::send(); }
void Arduino::bigbomb_2(){ arduinoID = 12; Arduino::send(); }
void Arduino::bigbomb_3(){ arduinoID = 14; Arduino::send(); }
