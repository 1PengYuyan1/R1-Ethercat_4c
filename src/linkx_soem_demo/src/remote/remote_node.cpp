// ROS2 节点: 从 /dev/rc_receiver 串口 (115200 8N1) 读 FS-i6S iBus 帧,
// 经 Class_FSi6S 解算后发布 /cmd_vel + /robot_buttons, 与原 F710 节点 topic 兼容,
// robot.cpp 那一侧不必改任何东西。

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include "linkx_soem_demo/remote/dvc_fsi6s.h"
#include "linkx_soem_demo/remote/ibus_parser.h"

namespace
{
// 给 termios 设 115200 8N1, 无校验, 无流控, 完全 raw 模式 (read 不阻塞超时由 VMIN/VTIME 控制)
bool Configure_Serial(int fd)
{
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) return false;

    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~PARENB;        // 无校验
    tio.c_cflag &= ~CSTOPB;        // 1 停止位
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;            // 8 数据位

    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_iflag |= IGNPAR;

    tio.c_cc[VMIN]  = 0;   // 非阻塞 read; 真正阻塞由外层 poll() 实现, 便于优雅退出
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) return false;
    tcflush(fd, TCIFLUSH);
    return true;
}
}  // namespace

class TeleopFSi6SNode : public rclcpp::Node
{
public:
    TeleopFSi6SNode() : Node("remote_node_cpp")
    {
        // 参数: max_speed / max_omega / deadzone 默认值与原 F710 节点一致, 保持兼容
        this->declare_parameter("max_speed", 0.5);
        this->declare_parameter("max_omega", 0.5);
        this->declare_parameter("deadzone",  0.03);
        // 串口路径: 默认走 udev 软链 /dev/rc_receiver, 也允许命令行覆盖
        this->declare_parameter("serial_port", std::string("/dev/rc_receiver"));
        // 失联超时 (ms): 串口长时间没收到合法 iBus 帧 -> 发零速 + 零按键
        this->declare_parameter("rx_timeout_ms", 200);

        const float max_speed = static_cast<float>(this->get_parameter("max_speed").as_double());
        const float max_omega = static_cast<float>(this->get_parameter("max_omega").as_double());
        const float deadzone  = static_cast<float>(this->get_parameter("deadzone").as_double());
        serial_path_ = this->get_parameter("serial_port").as_string();
        rx_timeout_ms_ = this->get_parameter("rx_timeout_ms").as_int();

        fsi6s_.Set_Control_Params(max_speed, max_omega, deadzone);

        pub_chassis_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_buttons_ = this->create_publisher<std_msgs::msg::UInt32>("/robot_buttons", 10);
        pub_vra_     = this->create_publisher<std_msgs::msg::Float32>("/remote_vra", 10);

        RCLCPP_INFO(this->get_logger(),
                    "remote_node started: serial=%s max_speed=%.2f max_omega=%.2f deadzone=%.3f",
                    serial_path_.c_str(), max_speed, max_omega, deadzone);

        running_.store(true);
        rx_thread_ = std::thread(&TeleopFSi6SNode::Rx_Loop, this);
    }

    ~TeleopFSi6SNode() override
    {
        running_.store(false);
        if (rx_thread_.joinable()) rx_thread_.join();
        if (fd_ >= 0) ::close(fd_);
    }

private:
    static constexpr bool kEnableDebugLog = false;

    Class_FSi6S  fsi6s_;
    IbusParser   parser_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_chassis_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr     pub_buttons_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr    pub_vra_;

    std::string  serial_path_;
    int          rx_timeout_ms_ = 200;
    int          fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread  rx_thread_;

    // 打开串口失败时: 不让节点直接 crash, 而是 100ms 重试, 让用户能热插拔
    int Open_Serial_With_Retry()
    {
        while (running_.load() && rclcpp::ok()) {
            int fd = ::open(serial_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd >= 0) {
                if (Configure_Serial(fd)) return fd;
                ::close(fd);
                RCLCPP_WARN(this->get_logger(), "termios 配置 %s 失败, 100ms 后重试", serial_path_.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "打开 %s 失败 (%s), 100ms 后重试",
                            serial_path_.c_str(), std::strerror(errno));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return -1;
    }

    // 串口接收线程: poll 等数据, 喂给 IbusParser; 拿到完整帧就解算并发布
    void Rx_Loop()
    {
        fd_ = Open_Serial_With_Retry();
        if (fd_ < 0) return;

        std::vector<uint16_t> channels;
        auto last_frame_t = std::chrono::steady_clock::now();
        bool last_publish_was_zero = false;
        uint8_t buf[128];

        // 关掉旧 fd, 重连; 若 running_ 已 false 返回 false 通知调用方退出循环
        auto reconnect = [this]() -> bool {
            if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
            fd_ = Open_Serial_With_Retry();
            return fd_ >= 0;
        };

        while (running_.load() && rclcpp::ok()) {
            pollfd pfd{ fd_, POLLIN, 0 };
            const int pret = ::poll(&pfd, 1, 50);   // 50ms 轮询, 配合 rx 超时判定
            if (pret < 0) {
                if (errno == EINTR) continue;
                RCLCPP_ERROR(this->get_logger(), "poll(%s) 错误: %s; 100ms 后重连",
                             serial_path_.c_str(), std::strerror(errno));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if (!reconnect()) return;
                continue;
            }

            if (pret > 0 && (pfd.revents & POLLIN)) {
                const ssize_t n = ::read(fd_, buf, sizeof(buf));
                if (n > 0) {
                    const int decoded = parser_.Feed(buf, static_cast<size_t>(n), channels);
                    if (decoded > 0) {
                        Publish_From_Channels(channels);
                        last_frame_t = std::chrono::steady_clock::now();
                        last_publish_was_zero = false;
                    }
                } else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                    RCLCPP_WARN(this->get_logger(), "%s read() 返回 %zd (%s), 重连",
                                serial_path_.c_str(), n, std::strerror(errno));
                    if (!reconnect()) return;
                    continue;
                }
            }

            // 超时: 长时间无帧, 发一次零速 + 零按键, 让下游进入安全态
            const auto now = std::chrono::steady_clock::now();
            const auto since_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_t).count();
            if (since_ms > rx_timeout_ms_ && !last_publish_was_zero) {
                Publish_Failsafe();
                last_publish_was_zero = true;
                RCLCPP_WARN(this->get_logger(),
                            "iBus 失联 %lld ms (>%d), 已发零速 + 零按键",
                            static_cast<long long>(since_ms), rx_timeout_ms_);
            }
        }
    }

    // 失联/超时时下发的安全态: 零速 Twist + 零按键码
    void Publish_Failsafe()
    {
        pub_chassis_->publish(geometry_msgs::msg::Twist{});
        std_msgs::msg::UInt32 btn;
        btn.data = 0U;
        pub_buttons_->publish(btn);
    }

    void Publish_From_Channels(const std::vector<uint16_t> &channels)
    {
        const auto chassis = fsi6s_.Resolve_Chassis(channels);
        const uint32_t buttons = fsi6s_.Resolve_Button_Code(channels);
        const float scale = fsi6s_.Resolve_Speed_Scale(channels);
        const float vra   = fsi6s_.Resolve_VrA_Unit(channels);

        geometry_msgs::msg::Twist twist;
        twist.linear.x  = chassis.vx * scale;
        twist.linear.y  = chassis.vy * scale;
        twist.angular.z = chassis.omega;
        pub_chassis_->publish(twist);

        std_msgs::msg::UInt32 btn;
        btn.data = buttons;
        pub_buttons_->publish(btn);

        std_msgs::msg::Float32 vra_msg;
        vra_msg.data = vra;
        pub_vra_->publish(vra_msg);

        if (kEnableDebugLog &&
            (std::abs(chassis.vx) > 0.01f || std::abs(chassis.vy) > 0.01f ||
             std::abs(chassis.omega) > 0.01f || buttons != 0U)) {
            RCLCPP_INFO(this->get_logger(),
                        "vx=%.2f vy=%.2f omega=%.2f buttons=0x%05X frames=%llu drop=%llu",
                        chassis.vx, chassis.vy, chassis.omega, buttons,
                        static_cast<unsigned long long>(parser_.Frames_Decoded()),
                        static_cast<unsigned long long>(parser_.Frames_Dropped()));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopFSi6SNode>());
    rclcpp::shutdown();
    return 0;
}
