// =============================================================================
// fsi6s_monitor_main.cpp
//
// FS-i6S 遥控通道实时监视器 (纯只读, 不依赖 EtherCAT / ROS2).
//   - 打开 /dev/rc_receiver (115200 8N1) 读 iBus 帧
//   - IbusParser 解出 14 通道 PWM (us)
//   - Class_FSi6S 解算 vx/vy/omega/speed_scale/VrA/button code
//   - 用 ANSI 控制码就地刷新打印所有原始通道 + 各解算结果, 方便核对通道映射
//
// 用法:
//   ./build/linkx_soem_demo/fsi6s_monitor                # 默认 /dev/rc_receiver
//   ./build/linkx_soem_demo/fsi6s_monitor /dev/ttyUSB0   # 显式串口
//
// 退出: Ctrl+C
// =============================================================================

#include "linkx_soem_demo/remote/dvc_fsi6s.h"
#include "linkx_soem_demo/remote/ibus_parser.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

std::atomic<bool> g_running{true};

void On_Sigint(int) { g_running.store(false); }

bool Configure_Serial(int fd) {
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) return false;
    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_iflag |= IGNPAR;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio) != 0) return false;
    tcflush(fd, TCIFLUSH);
    return true;
}

// 把 3 段 SwB/SwC 当前 pwm 翻译成 "Up/Mid/Down"
const char* Pos3_Name(uint16_t pwm) {
    if (pwm < 1250U) return "Down";
    if (pwm > 1750U) return "Up  ";
    return "Mid ";
}

// 把 2 段 SwA/SwD/Key1 当前 pwm 翻译成 "Up/Down"
const char* Pos2_Name(uint16_t pwm) {
    return (pwm >= 1500U) ? "Up  " : "Down";
}

// 把 button code 翻译成可读字符串
std::string Decode_Button_Code(uint32_t code) {
    if (code == FSi6S_Key_IDLE) return "IDLE";
    std::string out;
    auto append = [&](const char* s) {
        if (!out.empty()) out += " | ";
        out += s;
    };
    if (code & FSi6S_SwA_Up)   append("SwA_Up(DISABLE)");
    if (code & FSi6S_SwA_Down) append("SwA_Down(ENABLE)");

    const uint32_t swb = code & FSi6S_SwB_MASK;
    if (swb == FSi6S_SwB_Up)   append("SwB_Up");
    else if (swb == FSi6S_SwB_Mid) append("SwB_Mid");
    else if (swb == FSi6S_SwB_Down) append("SwB_Down");

    const uint32_t swc = code & FSi6S_SwC_MASK;
    if (swc == FSi6S_SwC_Up)   append("SwC_Up(edge)");
    else if (swc == FSi6S_SwC_Mid) append("SwC_Mid(edge)");
    else if (swc == FSi6S_SwC_Down) append("SwC_Down(edge)");

    if (code & FSi6S_SwD_Up)   append("SwD_Up(edge)");
    if (code & FSi6S_SwD_Down) append("SwD_Down(edge)");
    if (code & FSi6S_Key1_UP)   append("Key1_UP(press)");
    if (code & FSi6S_Key1_DOWN) append("Key1_DOWN(release)");
    return out;
}

// 仅当 code 有非常驻 bit (除 SwA/SwB 之外) 时, "粘住" 让用户看到边沿
struct SavedEdge {
    uint32_t code = 0;
    std::chrono::steady_clock::time_point t{};
};

}  // namespace

int main(int argc, char** argv) {
    const std::string serial_path = (argc > 1) ? argv[1] : "/dev/rc_receiver";

    signal(SIGINT, On_Sigint);

    int fd = ::open(serial_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "open(%s) failed: %s\n", serial_path.c_str(), strerror(errno));
        return 1;
    }
    if (!Configure_Serial(fd)) {
        fprintf(stderr, "termios %s failed: %s\n", serial_path.c_str(), strerror(errno));
        ::close(fd);
        return 1;
    }

    IbusParser parser;
    Class_FSi6S fsi6s;
    // 用与 remote_node 一致的 0.5 m/s, 0.5 rad/s, 死区 0.03 (Apply_Stick_Channel 用)
    fsi6s.Set_Control_Params(0.5f, 0.5f, 0.03f);

    std::vector<uint16_t> channels;
    uint8_t buf[128];
    auto last_frame_t = std::chrono::steady_clock::now();
    auto last_resolve_t = last_frame_t;
    bool first_paint = true;

    // 边沿 sticky: 拿到一次后保留 800ms, 方便看清是哪一拍触发了哪个 Sw*/Key1 bit
    SavedEdge edge;
    constexpr int kEdgeStickMs = 800;

    printf("\033[2J");  // 全屏清屏一次, 之后每帧用 \033[H 光标回顶
    fflush(stdout);

    while (g_running.load()) {
        pollfd pfd{ fd, POLLIN, 0 };
        const int pret = ::poll(&pfd, 1, 20);
        if (pret < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "poll: %s\n", strerror(errno));
            break;
        }

        bool got_frame = false;
        if (pret > 0 && (pfd.revents & POLLIN)) {
            const ssize_t n = ::read(fd, buf, sizeof(buf));
            if (n > 0) {
                const int decoded = parser.Feed(buf, static_cast<size_t>(n), channels);
                if (decoded > 0) {
                    got_frame = true;
                    last_frame_t = std::chrono::steady_clock::now();
                }
            }
        }

        const auto now = std::chrono::steady_clock::now();
        const long since_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  now - last_frame_t).count();

        // 没收到帧就先不解算 (沿用上一帧 channels), 但仍刷新一次显示让用户能看到失联
        Struct_FSi6S_Command cmd{};
        float scale = 0.30f;
        float vra   = 0.0f;
        uint32_t code = 0;

        if (!channels.empty()) {
            cmd   = fsi6s.Resolve_Chassis(channels);
            scale = fsi6s.Resolve_Speed_Scale(channels);
            vra   = fsi6s.Resolve_VrA_Unit(channels);
            // 只在新帧上前进 button 边沿状态机, 旧帧重复算会让 SwC/SwD/Key1
            // 误判成"位置又变了"。
            if (got_frame) {
                code = fsi6s.Resolve_Button_Code(channels);
                if ((code & ~(FSi6S_SwB_MASK)) != 0U) {
                    // 有事件 bit (含 SwA/SwC/SwD/Key1), sticky 记录
                    edge.code = code;
                    edge.t = now;
                }
            }
        }

        // sticky edge: 保留 800ms 让人眼能看清
        uint32_t edge_view = 0;
        long edge_age_ms = 0;
        if (edge.code != 0U) {
            edge_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              now - edge.t).count();
            if (edge_age_ms <= kEdgeStickMs) edge_view = edge.code;
        }

        // === 渲染 ===
        printf("\033[H");  // 光标回顶
        if (first_paint) first_paint = false;

        const bool link_ok = since_ms < 200 && !channels.empty();
        printf("\033[K=== FS-i6S 遥控通道监视器  (%s, %s) ===\n",
               serial_path.c_str(),
               link_ok ? "\033[32mLINK OK\033[0m " : "\033[31mNO SIGNAL\033[0m");
        printf("\033[K  frames decoded=%llu  dropped=%llu  last_frame=%ldms ago\n\n",
               static_cast<unsigned long long>(parser.Frames_Decoded()),
               static_cast<unsigned long long>(parser.Frames_Dropped()),
               since_ms);

        // ---- 原始通道 ----
        auto ch = [&](size_t i) -> uint16_t {
            return (i < channels.size()) ? channels[i] : 0U;
        };
        printf("\033[K--- 原始通道 PWM (us) ---\n");
        printf("\033[K  CH1  RH/Aileron  = %4u   (vy)       CH8  SwD       = %4u   %s\n",
               ch(0), ch(7), Pos2_Name(ch(7)));
        printf("\033[K  CH2  RV/Elevator = %4u   (vx)       CH9  VrA       = %4u\n",
               ch(1), ch(8));
        printf("\033[K  CH3  LV/Throttle = %4u   (scale)    CH10 Key1      = %4u   %s\n",
               ch(2), ch(9), Pos2_Name(ch(9)));
        printf("\033[K  CH4  LH/Rudder   = %4u   (omega)    CH11           = %4u\n",
               ch(3), ch(10));
        printf("\033[K  CH5  SwA         = %4u   %s        CH12           = %4u\n",
               ch(4), Pos2_Name(ch(4)), ch(11));
        printf("\033[K  CH6  SwB         = %4u   %s        CH13           = %4u\n",
               ch(5), Pos3_Name(ch(5)), ch(12));
        printf("\033[K  CH7  SwC         = %4u   %s        CH14           = %4u\n\n",
               ch(6), Pos3_Name(ch(6)), ch(13));

        // ---- 解算结果 ----
        printf("\033[K--- 解算结果 (chassis 0.5/0.5/0.03) ---\n");
        printf("\033[K  vx    = %+6.3f m/s        (CH2 RV 推前为 +)\n", cmd.vx);
        printf("\033[K  vy    = %+6.3f m/s        (CH1 RH 推右为 - 与 OPS 右手系)\n", cmd.vy);
        printf("\033[K  omega = %+6.3f rad/s      (CH4 LH 推右为 - / CW 为 +)\n\n", cmd.omega);

        printf("\033[K--- Throttle / VrA ---\n");
        printf("\033[K  speed_scale = %.3f        (CH3 0.30~1.00, 中位 0.65)\n", scale);
        printf("\033[K  VrA_unit    = %.3f        (CH9 旋钮 0..1)\n\n", vra);

        // ---- 按键 code ----
        printf("\033[K--- 按键 code (Class_FSi6S::Resolve_Button_Code) ---\n");
        printf("\033[K  本帧 code   = 0x%05X  %s\n",
               code, Decode_Button_Code(code).c_str());
        printf("\033[K  控制使能    = %s  (SwA_Down 持位 500ms 翻 ENABLE; SwA_Up 100ms 翻 DISABLE)\n",
               fsi6s.Is_Control_Enabled() ? "\033[32mENABLED \033[0m" : "\033[33mDISABLED\033[0m");
        if (edge_view != 0U) {
            printf("\033[K  最近边沿    = 0x%05X  %s   (%ldms ago)\n",
                   edge_view, Decode_Button_Code(edge_view).c_str(), edge_age_ms);
        } else {
            printf("\033[K  最近边沿    = (无, 拨动 SwA/SwC/SwD 或按 Key1 试试)\n");
        }

        // 喂 enable 状态机: 用真实两次新帧间隔 (iBus ~7ms), 失联时 clamp 防乱跳
        if (!channels.empty() && got_frame) {
            const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                                now - last_resolve_t).count();
            uint32_t dt_ms = static_cast<uint32_t>(dt);
            if (dt_ms > 50U) dt_ms = 50U;
            if (dt_ms < 1U)  dt_ms = 1U;
            fsi6s.Update_Control_Enable(code, dt_ms);
            last_resolve_t = now;
        }

        printf("\n\033[K(Ctrl+C 退出)\033[J\n");  // \033[J 清掉光标以下残留行
        fflush(stdout);
    }

    ::close(fd);
    printf("\nbye.\n");
    return 0;
}
