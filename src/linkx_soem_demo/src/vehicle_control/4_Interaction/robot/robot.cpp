//
// 机器人交互层实现 (全向轮工况)
// ----------------------------------------------------------------------------
// 文件组织 (与舵轮版保持类似):
//   1. 内部常量 (匿名 namespace)
//   2. 生命周期     —— Init / Start_ROS2_Remote_Bridge / Stop_ROS2_Remote_Bridge
//   3. 周期回调     —— TIM_1ms / TIM_2ms / TIM_100ms
//   4. CAN 接收分发 —— CAN_Rx_Callback
//   5. 控制核心     —— _Chassis_Control 及其辅助函数
//   6. ROS2 桥接内部 —— _ROS2_Remote_Spin_Loop / _Update_Remote_*
//
// 与舵轮版差异:
//   * 不再 Init 舵向校准、不挂 Manual_Yaw_Hold、不写 manual_correction CSV
//   * CAN_Rx_Callback ch0 改派给 DJI (0x201..0x204), 不再分发 ODrive/Encoder
//   * _Chassis_Control 中航向直传 snapshot.omega, 不做慢漂纠偏
// ----------------------------------------------------------------------------
#include "robot.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include "linkx4c_handler.h"

namespace {
// 遥控命令超时阈值: 超过 200ms 未刷新则视为失联, 强制零速
constexpr int64_t kRemoteCmdTimeoutNs = 200LL * 1000LL * 1000LL;

/** @brief 取当前 steady_clock 时间戳 (纳秒) */
inline int64_t Steady_Now_Ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
}  // namespace

// ============================================================================
//  生命周期
// ============================================================================

/**
 * @brief 机器人初始化
 * @param __LinkX_Handler     经典 LinkX (slave_id=1), 挂 DJI 3508 ×4 (ch0) + OPS (ch3)
 * @param __LinkX_FD_Handler  CAN-FD LinkX (slave_id=2), 挂夹爪 DM + 气夹爪 (均在 ch0); 可为 nullptr
 */
void Class_Robot::Init(linkx_t* __LinkX_Handler, linkx_t* __LinkX_FD_Handler) {
    LinkX_Handler    = __LinkX_Handler;
    LinkX_FD_Handler = __LinkX_FD_Handler;
    Chassis.Init(LinkX_Handler);
    Chassis.Init_Motor_Params();

    if (LinkX_FD_Handler != nullptr) {
        Clamp.Init(LinkX_FD_Handler, /*CAN_Channel=*/0);
        std::cout << "[ROBOT] Clamp + gripper initialized on FD-LinkX ch0 "
                     "(slave_id=2, gripper id=0x13)."
                  << std::endl;
    } else {
        std::cout << "[ROBOT] WARN: FD LinkX handle is null, Clamp disabled."
                  << std::endl;
    }

    // ============== 半自动路径跟随 (OPS 纠偏) ==============
    Auto_Pilot.Init(&Chassis);

    // 横向 PID (沿用舵轮版调过的参数)
    Auto_Pilot.Set_Lateral_PID(/*kp=*/0.5f, /*ki=*/0.05f, /*kd=*/0.0f,
                               /*out_max_mm_s=*/30.0f, /*i_out_max_mm_s=*/15.0f);
    // 航向 PID (沿用舵轮版调过的参数)
    Auto_Pilot.Set_Heading_PID(/*kp=*/0.01f, /*ki=*/0.0f, /*kd=*/0.0f,
                               /*out_max_rad_s=*/0.05f);

    // 路径 0 (按键 Y 触发): 与舵轮版一致, 蓝方斜对角线
    static const Struct_Waypoint kPath0[] = {
        {-400.0f, -400.0f, 00.0f, 180.0f, 30.0f},
        {-600.0f, -600.0f, 00.0f, 100.0f, 30.0f},
        {-1000.0f, -600.0f, 00.0f, 180.0f, 30.0f},
        {-1400.0f, -600.0f, 00.0f, 180.0f, 20.0f},
        {-1800.0f, -600.0f, 00.0f, 180.0f, 20.0f},
        {-2000.0f, -600.0f, 00.0f, 100.0f, 0.0f},
        {-2140.0f, -600.0f, 00.0f, 50.0f, 0.0f},
    };
    Auto_Pilot.Register_Path(0, kPath0, sizeof(kPath0) / sizeof(kPath0[0]));
}

void Class_Robot::Start_ROS2_Remote_Bridge() {
    if (ros_bridge_running_.load()) return;

    if (!rclcpp::ok()) {
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
        ros_initialized_here_ = true;
    }

    ros_bridge_running_.store(true);
    ros_spin_thread_ = std::thread(&Class_Robot::_ROS2_Remote_Spin_Loop, this);
    std::cout << "[ROBOT] ROS2 remote bridge started. Subscribing /cmd_vel, "
                 "/robot_buttons, /remote_vra."
              << std::endl;
}

void Class_Robot::Stop_ROS2_Remote_Bridge() {
    if (!ros_bridge_running_.load()) return;

    ros_bridge_running_.store(false);
    if (ros_spin_thread_.joinable()) ros_spin_thread_.join();

    if (ros_initialized_here_ && rclcpp::ok()) {
        rclcpp::shutdown();
    }
    ros_initialized_here_ = false;
    std::cout << "[ROBOT] ROS2 remote bridge stopped." << std::endl;
}

// ============================================================================
//  周期回调
// ============================================================================

void Class_Robot::TIM_1ms_Calculate_Callback() { _Chassis_Control(); }

void Class_Robot::TIM_2ms_Calculate_PeriodElapsedCallback() {
    Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
    Chassis.TIM_2ms_Control_PeriodElapsedCallback();
    Clamp.TIM_Calculate_PeriodElapsedCallback();
}

void Class_Robot::TIM_100ms_Alive_PeriodElapsedCallback() {
    Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
    Clamp.TIM_100ms_Alive_PeriodElapsedCallback();
}

// ============================================================================
//  CAN 接收分发
// ============================================================================

/**
 * @brief CAN 帧接收统一入口
 *        按 (module_id, channel) 二级分发:
 *          module_id=0 (经典 LinkX, slave_id=1):
 *              ch0 = DJI 3508 ×4 反馈 (0x201..0x204)
 *              ch3 = OPS (CAN-COM 透传, ID 不固定)
 *              通道映射失配时按 ID 全分发兜底 (仅 DJI, OPS 不参与兜底)
 *          module_id=1 (CAN-FD LinkX, slave_id=2):
 *              ch0 = Clamp DM (Rx 0x11/0x12)
 *              不参与跨模块兜底
 */
void Class_Robot::CAN_Rx_Callback(uint8_t Module_Id, uint8_t CAN_Channel,
                                  uint32_t CAN_ID, uint8_t* CAN_Data,
                                  uint8_t CAN_DLen) {
    const uint32_t can_id_std = (CAN_ID & 0x7FFU);

    // —— DJI 3508: 按 0x201..0x204 匹配 ——
    auto dispatch_dji = [&](uint32_t id_std) -> bool {
        for (int i = 0; i < 4; ++i) {
            if (id_std == Chassis.Motor_DJI[i].DJI_CAN_Rx_ID) {
                Chassis.Motor_DJI[i].CAN_RxCpltCallback(CAN_Data);
                return true;
            }
        }
        return false;
    };

    // —— OPS-9: CAN-COM 透传, 所有片段共用一个 ID; DLC 必须透传 ——
    auto dispatch_ops = [&](uint32_t /*id_std*/) -> bool {
        Chassis.OPS.CAN_RxCpltCallback(CAN_Data, CAN_DLen);
        return true;
    };

    // —— 夹爪 DM: Rx_ID = 0x11 (大 Pitch) / 0x12 (小 Pitch) ——
    auto dispatch_clamp = [&](uint32_t id_std) -> bool {
        if (id_std == 0x11U) {
            Clamp.Motor_Pitch_Large.CAN_RxCpltCallback(CAN_Data);
            return true;
        }
        if (id_std == 0x12U) {
            Clamp.Motor_Pitch_Small.CAN_RxCpltCallback(CAN_Data);
            return true;
        }
        return false;
    };

    if (Module_Id == 1U) {
        if (CAN_Channel == 0U) (void)dispatch_clamp(can_id_std);
        return;
    }

    // module_id == 0: 按通道分发
    bool handled = false;
    switch (CAN_Channel) {
        case 0:
            handled = dispatch_dji(can_id_std);
            break;
        case 3:
            handled = dispatch_ops(can_id_std);
            break;
        default:
            break;
    }

    // 通道映射失配时按 ID 兜底 (OPS 不参与, 避免吞掉其他帧)
    if (!handled) {
        (void)dispatch_dji(can_id_std);
    }
}

// ============================================================================
//  控制核心
// ============================================================================

/**
 * @brief 底盘控制主流程 (每 1ms 调用一次)
 *        编排: 取快照 → 更新使能 → 处理功能键 → 下发目标速度 → 刷新调试观测。
 *
 *        全向轮版本与舵轮版差异: 不再做 Manual_Yaw_Hold 航向慢漂纠偏,
 *        snapshot.omega 直接透传到 _Apply_Chassis_Velocity。
 */
void Class_Robot::_Chassis_Control() {
    // 1) 取 ROS 命令快照
    Ros_Remote_Command snapshot;
    bool is_recent = false;
    _Snapshot_Remote_Command(snapshot, is_recent);

    // 2) 按键长按更新手动使能
    const uint32_t key_code =
        snapshot.has_buttons ? snapshot.buttons : FSi6S_Key_IDLE;
    const bool is_enabled = _Update_Manual_Enable_State(key_code);

    // 3) 功能键边沿处理 (Key1/SwB/SwC/SwD)
    _Process_Function_Key_Edge(key_code, is_enabled);

    // 3b) 气夹爪 push (FD-LinkX ch0 / id=0x13 / byte0)
    Clamp.Push_Gripper_Frame();

    // ============== AUTO-PILOT (OPS 半自动路径跟随) ==============
    if (Auto_Pilot.Is_Active()) {
        Auto_Pilot.TIM_Tick(0.001f);
        debug_remote_key_code_   = key_code;
        debug_remote_is_recent_  = is_recent;
        debug_remote_is_enabled_ = is_enabled;
        debug_remote_vx_    = is_recent ? snapshot.vx    : 0.0f;
        debug_remote_vy_    = is_recent ? snapshot.vy    : 0.0f;
        debug_remote_omega_ = is_recent ? snapshot.omega : 0.0f;
        return;
    }

    // 4) 手动模式: 速度直传 (不做航向纠偏)
    const bool active = is_recent && is_enabled;
    _Apply_Chassis_Velocity(snapshot.vx, snapshot.vy, snapshot.omega,
                            snapshot.vra, active);

    // 4b) 夹爪跟随全局手动使能
    _Clamp_Control(key_code, is_enabled);

    // 5) 调试观测 (不受使能门控影响)
    debug_remote_key_code_   = key_code;
    debug_remote_is_recent_  = is_recent;
    debug_remote_is_enabled_ = is_enabled;
    debug_remote_vx_    = is_recent ? snapshot.vx    : 0.0f;
    debug_remote_vy_    = is_recent ? snapshot.vy    : 0.0f;
    debug_remote_omega_ = is_recent ? snapshot.omega : 0.0f;
}

bool Class_Robot::_Snapshot_Remote_Command(Ros_Remote_Command& out_cmd,
                                           bool& out_is_recent) {
    {
        std::lock_guard<std::mutex> lock(ros_cmd_mutex_);
        out_cmd = ros_cmd_;
    }
    const int64_t now_ns = Steady_Now_Ns();
    out_is_recent = out_cmd.has_twist && (out_cmd.last_update_ns > 0) &&
                    ((now_ns - out_cmd.last_update_ns) <= kRemoteCmdTimeoutNs);
    return out_is_recent;
}

bool Class_Robot::_Update_Manual_Enable_State(uint32_t key_code) {
    const bool was_enabled = fsi6s_remote_.Is_Control_Enabled();
    fsi6s_remote_.Update_Control_Enable(key_code, 1U);
    const bool is_enabled = fsi6s_remote_.Is_Control_Enabled();

    if (!was_enabled && is_enabled)
        std::cout << "[ROBOT] ROS manual control enabled by SwA-Up long press."
                  << std::endl;
    else if (was_enabled && !is_enabled)
        std::cout << "[ROBOT] ROS manual control disabled by SwA-Down long press."
                  << std::endl;

    return is_enabled;
}

/**
 * @brief 功能键边沿动作 (按 SwB 模式分发, 与舵轮版逻辑一致)
 *        SwB_Up   : OPS + CAN 信号 (SwC 三态 → ID=0x14 / SwD → Auto_Pilot)
 *        SwB_Mid  : Clamp + 气夹爪 (Key1 控 gripper / SwC+SwD 控 Clamp)
 *        SwB_Down : 机械臂 (Key1 → ID=0x16)
 */
void Class_Robot::_Process_Function_Key_Edge(uint32_t key_code,
                                             bool is_enabled) {
    if (!is_enabled) return;

    uint32_t rising_key = FSi6S_Key_IDLE;
    if (!fsi6s_remote_.Check_Key_Rising_Edge(key_code, &rising_key)) return;

    const uint32_t swb_state = rising_key & FSi6S_SwB_MASK;
    const uint32_t swc_state = rising_key & FSi6S_SwC_MASK;
    const uint32_t swd_state = rising_key & FSi6S_SwD_MASK;
    const bool key1_up_edge   = (rising_key & FSi6S_Key1_UP)   != 0U;
    const bool key1_down_edge = (rising_key & FSi6S_Key1_DOWN) != 0U;

    const bool swc_edge = (swc_state != 0U);
    const bool swd_edge = (swd_state != 0U);
    const bool has_event = key1_up_edge || key1_down_edge || swc_edge || swd_edge;

    if (swc_edge) last_swc_level_ = swc_state;
    if (swd_edge) last_swd_level_ = swd_state;

    if (!has_event) {
        if (rising_key == FSi6S_SwA_Up)
            std::cout << "[ROBOT] SwA_Up (short press / disable arming)" << std::endl;
        return;
    }

    // ============ SwB_Up: OPS + CAN ============
    if (swb_state == FSi6S_SwB_Up) {
        if (swc_edge && LinkX_FD_Handler != nullptr) {
            uint8_t buf[8] = {0};
            const char *label = "?";
            if (swc_state == FSi6S_SwC_Up)        { buf[0] = 0x01U; label = "X"; }
            else if (swc_state == FSi6S_SwC_Mid)  { buf[0] = 0x02U; label = "Y"; }
            else /* SwC_Down */                   { buf[0] = 0x03U; label = "B"; }
            linkx_quick_can_send(LinkX_FD_Handler, 0U, 0x14U, buf);
            std::cout << "[ROBOT] SwB_Up+SwC (" << label
                      << "): FD-LinkX ch0 ID=0x14 payload=0x" << std::hex
                      << static_cast<int>(buf[0]) << std::dec << std::endl;
        }
        if (swd_state == FSi6S_SwD_Down) {
            Auto_Pilot.Start(0);
            std::cout << "[ROBOT] SwB_Up+SwD_Down: Auto-Pilot START path0"
                      << std::endl;
        } else if (swd_state == FSi6S_SwD_Up) {
            Auto_Pilot.Stop();
            std::cout << "[ROBOT] SwB_Up+SwD_Up: Auto-Pilot STOP" << std::endl;
        }
        return;
    }

    // ============ SwB_Mid: Clamp + 气夹爪 ============
    if (swb_state == FSi6S_SwB_Mid) {
        if (key1_up_edge) {
            Clamp.Set_Gripper_Byte0(0x00U);
            std::cout << "[CLAMP] SwB_Mid+Key1_UP: gripper OPEN (byte0=0x00)"
                      << std::endl;
        }
        if (key1_down_edge) {
            Clamp.Set_Gripper_Byte0(0x01U);
            std::cout << "[CLAMP] SwB_Mid+Key1_DOWN: gripper CLOSE (byte0=0x01)"
                      << std::endl;
        }
        if (swc_edge || swd_edge) {
            const uint32_t c = last_swc_level_;
            const uint32_t d = last_swd_level_;
            if (c == FSi6S_SwC_Mid && d == FSi6S_SwD_Down) {
                Clamp.Set_Pitch_Large_State(L_PITCH_POS2);
                Clamp.Set_Pitch_Small_State(S_PITCH_POS2);
                std::cout << "[CLAMP] SwB_Mid+SwC_Mid+SwD_Down: arm -> POS2/POS2"
                          << std::endl;
            } else if (c == FSi6S_SwC_Up && d == FSi6S_SwD_Down) {
                Clamp.Set_Pitch_Large_State(L_PITCH_POS2);
                Clamp.Set_Pitch_Small_State(S_PITCH_POS3);
                std::cout << "[CLAMP] SwB_Mid+SwC_Up+SwD_Down: arm -> POS2/POS3"
                          << std::endl;
            } else if (c == FSi6S_SwC_Up && d == FSi6S_SwD_Up) {
                Clamp.Set_Pitch_Large_State(L_PITCH_POS1);
                Clamp.Set_Pitch_Small_State(S_PITCH_POS1);
                std::cout << "[CLAMP] SwB_Mid+SwC_Up+SwD_Up: arm -> POS1/POS1"
                          << std::endl;
            } else if (c == FSi6S_SwC_Down && d == FSi6S_SwD_Down) {
                if (LinkX_FD_Handler != nullptr) {
                    uint8_t buf[8] = {0};
                    buf[0] = 0x02U;
                    linkx_quick_can_send(LinkX_FD_Handler, 0U, 0x15U, buf);
                    std::cout
                        << "[ROBOT] SwB_Mid+SwC_Down+SwD_Down: ID=0x15 payload=0x02"
                        << std::endl;
                }
            } else if (c == FSi6S_SwC_Down && d == FSi6S_SwD_Up) {
                if (LinkX_FD_Handler != nullptr) {
                    uint8_t buf[8] = {0};
                    buf[0] = 0x01U;
                    linkx_quick_can_send(LinkX_FD_Handler, 0U, 0x15U, buf);
                    std::cout << "[ROBOT] SwB_Mid+SwC_Down+SwD_Up: ID=0x15 payload=0x01"
                              << std::endl;
                }
            }
        }
        return;
    }

    // ============ SwB_Down: 机械臂 ============
    if (swb_state == FSi6S_SwB_Down) {
        if (key1_up_edge && LinkX_FD_Handler != nullptr) {
            uint8_t buf[8] = {0};
            buf[0] = 0x01U;
            linkx_quick_can_send(LinkX_FD_Handler, 0U, 0x16U, buf);
            std::cout << "[ROBOT] SwB_Down+Key1_UP: ID=0x16 payload=0x01"
                      << std::endl;
        }
        if (key1_down_edge && LinkX_FD_Handler != nullptr) {
            uint8_t buf[8] = {0};
            buf[0] = 0x02U;
            linkx_quick_can_send(LinkX_FD_Handler, 0U, 0x16U, buf);
            std::cout << "[ROBOT] SwB_Down+Key1_DOWN: ID=0x16 payload=0x02"
                      << std::endl;
        }
        return;
    }
}

void Class_Robot::_Clamp_Control(uint32_t /*key_code*/, bool is_enabled) {
    Clamp.Set_Clamp_Control_Type(is_enabled ? CLAMP_CONTROL_ENABLE
                                            : CLAMP_CONTROL_DISABLE);
}

/**
 * @brief 将目标速度下发到底盘
 *        active=true 时 slew rate 限幅并 ENABLE; 否则零速 + DISABLE。
 *        VrA 旋钮 (0..1) 线性映射加速度上限。
 */
void Class_Robot::_Apply_Chassis_Velocity(float vx, float vy, float omega,
                                          float vra, bool active) {
    if (!active) {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        Chassis.Set_Target_Velocity_X(0.0f);
        Chassis.Set_Target_Velocity_Y(0.0f);
        Chassis.Set_Target_Omega(0.0f);
        chassis_cmd_vx_    = 0.0f;
        chassis_cmd_vy_    = 0.0f;
        chassis_cmd_omega_ = 0.0f;
        return;
    }

    // VrA -> 加速度上限
    constexpr float kALinMin   = 0.2f;
    constexpr float kALinMax   = 5.0f;
    constexpr float kAOmegaMin = 0.5f;
    constexpr float kAOmegaMax = 10.0f;
    constexpr float kDt        = 0.001f;

    const float u = std::clamp(vra, 0.0f, 1.0f);
    const float a_lin   = kALinMin   + u * (kALinMax   - kALinMin);
    const float a_omega = kAOmegaMin + u * (kAOmegaMax - kAOmegaMin);
    const float dv_max  = a_lin   * kDt;
    const float dw_max  = a_omega * kDt;

    auto slew = [](float current, float target, float step) {
        const float delta = target - current;
        if (delta >  step) return current + step;
        if (delta < -step) return current - step;
        return target;
    };

    chassis_cmd_vx_    = slew(chassis_cmd_vx_,    vx,    dv_max);
    chassis_cmd_vy_    = slew(chassis_cmd_vy_,    vy,    dv_max);
    chassis_cmd_omega_ = slew(chassis_cmd_omega_, omega, dw_max);

    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ENABLE);
    Chassis.Set_Target_Velocity_X(chassis_cmd_vx_);
    Chassis.Set_Target_Velocity_Y(chassis_cmd_vy_);
    Chassis.Set_Target_Omega(chassis_cmd_omega_);
}

// ============================================================================
//  ROS2 桥接
// ============================================================================

void Class_Robot::_ROS2_Remote_Spin_Loop() {
    auto node = std::make_shared<rclcpp::Node>("soem_remote_bridge_node");

    auto sub_cmd = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 20, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            _Update_Remote_Twist(static_cast<float>(msg->linear.x),
                                 static_cast<float>(msg->linear.y),
                                 static_cast<float>(msg->angular.z));
        });

    auto sub_buttons = node->create_subscription<std_msgs::msg::UInt32>(
        "/robot_buttons", 20, [this](const std_msgs::msg::UInt32::SharedPtr msg) {
            _Update_Remote_Buttons(msg->data);
        });

    auto sub_vra = node->create_subscription<std_msgs::msg::Float32>(
        "/remote_vra", 20, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            _Update_Remote_Vra(msg->data);
        });

    auto pub_ops = node->create_publisher<geometry_msgs::msg::Twist>("/ops", 10);

    (void)sub_cmd;
    (void)sub_buttons;
    (void)sub_vra;

    rclcpp::WallRate rate(200);
    while (ros_bridge_running_.load() && rclcpp::ok()) {
        rclcpp::spin_some(node);

        geometry_msgs::msg::Twist ops_msg;
        ops_msg.linear.x  = Chassis.OPS.Get_Pos_X_Mm();
        ops_msg.linear.y  = Chassis.OPS.Get_Pos_Y_Mm();
        ops_msg.angular.z = Chassis.OPS.Get_Yaw_Deg();
        pub_ops->publish(ops_msg);

        rate.sleep();
    }
}

void Class_Robot::_Update_Remote_Twist(float vx, float vy, float omega) {
    std::lock_guard<std::mutex> lock(ros_cmd_mutex_);
    ros_cmd_.vx = vx;
    ros_cmd_.vy = vy;
    ros_cmd_.omega = omega;
    ros_cmd_.has_twist = true;
    ros_cmd_.last_update_ns = Steady_Now_Ns();
}

void Class_Robot::_Update_Remote_Buttons(uint32_t buttons) {
    std::lock_guard<std::mutex> lock(ros_cmd_mutex_);
    ros_cmd_.buttons = buttons;
    ros_cmd_.has_buttons = true;
    ros_cmd_.last_update_ns = Steady_Now_Ns();
}

void Class_Robot::_Update_Remote_Vra(float vra) {
    std::lock_guard<std::mutex> lock(ros_cmd_mutex_);
    ros_cmd_.vra = std::clamp(vra, 0.0f, 1.0f);
    ros_cmd_.has_vra = true;
    ros_cmd_.last_update_ns = Steady_Now_Ns();
}
