// ============================================================================
//  投放串口测试节点(独立小包, 不依赖 px4_exploration)
//  目的: 单独验证"串口通不通 + 三个投放口能不能依次触发"。
//  行为: 启动 → (use_serial=1 时)打开串口 → 等 serial_ready_secs(默认3s, 真机
//        MCU 要求) → 按 1,2,0 顺序投放, 每个口连发 repeat 次 → 打印汇总 → 退出。
//  协议与 px4_exploration/src/Arduino.cpp 完全一致(4 字节帧 {0xfe,0xfd,id,id%2},
//        id: bomb_0→1, bomb_1→2, bomb_2→3)。
//  用法:
//    ros2 launch serial_drop_test serial_drop_test.launch.py
//    ros2 launch serial_drop_test serial_drop_test.launch.py use_serial:=0   # 只打日志
// ============================================================================
#include <rclcpp/rclcpp.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace {

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

class SerialDropTest : public rclcpp::Node {
public:
    SerialDropTest() : Node("serial_drop_test_node") {
        port_       = this->declare_parameter("serial_port", std::string("/dev/ttyS3"));
        baud_       = this->declare_parameter("serial_baud", 115200);
        use_serial_ = this->declare_parameter("use_serial", 1);
        ready_secs_ = this->declare_parameter("serial_ready_secs", 3.0);
        gap_secs_   = this->declare_parameter("test_gap_secs", 1.5);   // 两次投放之间的间隔
        repeat_     = this->declare_parameter("test_repeat", 1);       // 每个口投放几轮(1 → 共3次)
        repeat_n_   = this->declare_parameter("test_repeat_times", 4); // 每轮同一个口连发几次
        repeat_gap_ = this->declare_parameter("test_repeat_gap_ms", 30);
        exit_after_ = this->declare_parameter("exit_after", 1);
        RCLCPP_INFO(this->get_logger(),
            "串口测试: port=%s baud=%d use_serial=%d ready=%.1fs 顺序1→2→0 × %d轮, 每轮连发%d次",
            port_.c_str(), baud_, use_serial_, ready_secs_, repeat_, repeat_n_);
    }

    ~SerialDropTest() override {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    void run() {
        if (!openPort()) return;
        // 真机 MCU: 打开后必须等一段时间才能用
        if (use_serial_ && ready_secs_ > 0.0) {
            RCLCPP_INFO(this->get_logger(), "等待 %.1fs 让 MCU 就绪...", ready_secs_);
            std::this_thread::sleep_for(std::chrono::duration<double>(ready_secs_));
        }
        int total = 0;
        const int order[3] = {1, 2, 0};   // 投放顺序: 1 → 2 → 0
        for (int round = 0; round < std::max(1, repeat_); ++round) {
            for (int idx = 0; idx < 3; ++idx) {
                const int port = order[idx];
                RCLCPP_INFO(this->get_logger(),
                    "→ 投放口 %d (第 %d/%d 轮, 连发 %d 次)",
                    port, round + 1, std::max(1, repeat_), std::max(1, repeat_n_));
                for (int k = 0; k < std::max(1, repeat_n_); ++k) {
                    sendBomb(port);
                    if (k + 1 < std::max(1, repeat_n_) && repeat_gap_ > 0)
                        std::this_thread::sleep_for(std::chrono::milliseconds(repeat_gap_));
                }
                ++total;
                if (gap_secs_ > 0.0)
                    std::this_thread::sleep_for(std::chrono::duration<double>(gap_secs_));
            }
        }
        RCLCPP_INFO(this->get_logger(),
            "=== 测试完成: 共投放 %d 次(1/2/0 各 %d 轮) ===", total, std::max(1, repeat_));
        if (exit_after_) {
            RCLCPP_INFO(this->get_logger(), "exit_after=1 → 退出");
            rclcpp::shutdown();
        }
    }

private:
    bool openPort() {
        if (!use_serial_) {
            RCLCPP_WARN(this->get_logger(),
                "use_serial=0 → 不打开串口, 只打印将要发送的帧(仿真/桌面自测用)");
            return true;
        }
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "打开串口 %s 失败: %s",
                         port_.c_str(), std::strerror(errno));
            return false;
        }
        int fl = ::fcntl(fd_, F_GETFL, 0);
        ::fcntl(fd_, F_SETFL, fl & ~O_NONBLOCK);

        struct termios tio;
        std::memset(&tio, 0, sizeof(tio));
        if (::tcgetattr(fd_, &tio) != 0)
            RCLCPP_WARN(this->get_logger(), "tcgetattr: %s", std::strerror(errno));
        cfsetospeed(&tio, baudFlag(baud_));
        cfsetispeed(&tio, baudFlag(baud_));
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~CRTSCTS;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_iflag &= ~(INLCR | ICRNL | IGNCR | IXON | IXOFF | IXANY);
        tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tio.c_oflag &= ~OPOST;
        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 1;
        if (::tcsetattr(fd_, TCSANOW, &tio) != 0)
            RCLCPP_WARN(this->get_logger(), "tcsetattr: %s", std::strerror(errno));
        ::tcflush(fd_, TCIOFLUSH);
        RCLCPP_INFO(this->get_logger(), "串口已打开: %s @ %d 8N1", port_.c_str(), baud_);
        return true;
    }

    void sendBomb(int port) {
        const unsigned char id = static_cast<unsigned char>(port + 1);   // 0→1, 1→2, 2→3
        unsigned char frame[4] = {0xfe, 0xfd, id, static_cast<unsigned char>(id % 2)};
        if (!use_serial_ || fd_ < 0) {
            RCLCPP_INFO(this->get_logger(),
                "[SIM] 帧 %02x %02x %02x %02x (口%d)", frame[0], frame[1], frame[2], frame[3], port);
            return;
        }
        ssize_t n = ::write(fd_, frame, sizeof(frame));
        ::tcdrain(fd_);
        if (n == static_cast<ssize_t>(sizeof(frame)))
            RCLCPP_INFO(this->get_logger(),
                "口%d 已发送: %02x %02x %02x %02x", port, frame[0], frame[1], frame[2], frame[3]);
        else
            RCLCPP_ERROR(this->get_logger(), "口%d 写入失败: %zd (%s)",
                         port, n, std::strerror(errno));
    }

    std::string port_;
    int    baud_        = 115200;
    int    use_serial_  = 1;
    double ready_secs_  = 3.0;
    double gap_secs_    = 1.5;
    int    repeat_      = 1;    // 每个口几轮
    int    repeat_n_    = 4;    // 每轮连发几次
    int    repeat_gap_  = 30;   // 连发间隔 ms
    int    exit_after_  = 1;
    int    fd_          = -1;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialDropTest>();
    node->run();
    if (rclcpp::ok()) rclcpp::spin(node);   // exit_after=0 时保持节点活着
    rclcpp::shutdown();
    return 0;
}
