//
// 全向轮 X 型四驱底盘实现
// ----------------------------------------------------------------------------
// CAN 通道映射 (与舵轮版有差异,见 robot.cpp CAN_Rx_Callback 顶部文档):
//   * ch0: 4 × DJI 3508 / C620
//          - 反馈 ID 0x201..0x204 (ESC ID 1..4)
//          - 控制 ID 0x200 一帧驱动 4 个
//   * ch1, ch2: 闲置 (舵轮版用作 ODrive / Encoder)
//   * ch3: OPS-9 (与舵轮版一致, Auto_Pilot 依赖)
// ----------------------------------------------------------------------------

#include "crt_chassis.h"

#include <algorithm>
#include <cmath>

/* ============================================================================
 *   生命周期
 * ========================================================================== */

void Class_Chassis::Init(linkx_t *__LinkX_Handler) {
    LinkX_Handler = __LinkX_Handler;

    // 4 个 DJI 3508 共享 ch0 + Tx_ID=0x200, Rx_ID = 0x201..0x204, Slot = 0..3
    for (int i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        const uint16_t rx_id = static_cast<uint16_t>(0x201U + static_cast<uint16_t>(i));
        Motor_DJI[i].Init(LinkX_Handler, /*can_channel=*/0,
                          /*rx_id=*/rx_id,
                          /*tx_id=*/0x200U,
                          /*slot=*/static_cast<uint8_t>(i),
                          /*reduction=*/Reduction_Ratio);
    }

    // OPS 沿用舵轮版的 ch3 + ID=0x01 约定
    OPS.Init(LinkX_Handler, /*can_channel=*/3, /*can_id=*/0x01);
}

void Class_Chassis::Init_Motor_Params() {
    // 每轮 RPM 闭环: error = target_rpm - now_rpm → 电流 raw (±MAX_CURRENT_RAW)
    // 种子参数 (空载经验起点, 现场需用 /pid-loop-tuning 调):
    //   K_P=8.0, K_I=0.2, K_D=0, D_T=0.002s (匹配 2ms 控制周期),
    //   I_Out_Max=8000, Out_Max=MAX_CURRENT_RAW
    for (int i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        wheel_rpm_pid_[i].Init(/*K_P=*/8.0f, /*K_I=*/0.2f, /*K_D=*/0.0f, /*K_F=*/0.0f,
                               /*I_Out_Max=*/8000.0f,
                               /*Out_Max=*/static_cast<float>(MAX_CURRENT_RAW),
                               /*D_T=*/0.002f,
                               /*Dead_Zone=*/0.0f);
    }
}

/* ============================================================================
 *   周期回调
 * ========================================================================== */

void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback() {
    for (int i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        Motor_DJI[i].TIM_Alive_PeriodElapsedCallback();
    }
    OPS.TIM_Alive_PeriodElapsedCallback();
}

void Class_Chassis::TIM_2ms_Resolution_PeriodElapsedCallback() {
    Self_Resolution();
}

void Class_Chassis::TIM_2ms_Control_PeriodElapsedCallback() {
    if (Chassis_Control_Type == Chassis_Control_Type_DISABLE) {
        // 失能直接清电流 + 清 PID 积分, 避免恢复时 windup
        for (int i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
            Target_Wheel_Omega[i] = 0.0f;
            Target_Rotor_RPM[i]   = 0.0f;
            wheel_rpm_pid_[i].Set_Target(0.0f);
            wheel_rpm_pid_[i].Set_Now(0.0f);
            wheel_rpm_pid_[i].Set_Integral_Error(0.0f);
            Motor_DJI[i].Set_Control_Current(0);
        }
    } else {
        // 速度安全限幅 (与舵轮版同量级)
        const float vx = std::clamp(Target_Velocity_X, -MAX_CHASSIS_SPEED, MAX_CHASSIS_SPEED);
        const float vy = std::clamp(Target_Velocity_Y, -MAX_CHASSIS_SPEED, MAX_CHASSIS_SPEED);
        const float wz = std::clamp(Target_Omega,      -MAX_CHASSIS_OMEGA, MAX_CHASSIS_OMEGA);

        // 暂存 clamp 后的目标供 IK 用
        Target_Velocity_X = vx;
        Target_Velocity_Y = vy;
        Target_Omega      = wz;

        Kinematics_Inverse_Resolution();
        Compute_Wheel_PID_And_Cache();
    }

    Send_Group_Frame();
}

/* ============================================================================
 *   逆运动学: (vx, vy, ω) → 每轮 rotor RPM 目标
 * ========================================================================== */

void Class_Chassis::Kinematics_Inverse_Resolution() {
    for (int i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        const float alpha = Roll_Direction[i];
        const float v_wheel = Target_Velocity_X * std::cos(alpha) +
                              Target_Velocity_Y * std::sin(alpha) +
                              Target_Omega       * Wheel_To_Core_Distance[i];
        Target_Wheel_Omega[i] = v_wheel / Wheel_Radius;                       // 轮边 rad/s
        Target_Rotor_RPM[i]   = Target_Wheel_Omega[i] * Reduction_Ratio       // 电机轴 RPM
                                * 60.0f / (2.0f * PI);
    }
}

/* ============================================================================
 *   每轮 PID: 把 target_rotor_rpm 闭环到 now_rotor_rpm, 出 ±16000 电流 raw
 * ========================================================================== */

void Class_Chassis::Compute_Wheel_PID_And_Cache() {
    for (int i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        wheel_rpm_pid_[i].Set_Target(Target_Rotor_RPM[i]);
        wheel_rpm_pid_[i].Set_Now(Motor_DJI[i].Get_Now_RPM());
        wheel_rpm_pid_[i].TIM_Calculate_PeriodElapsedCallback();
        const float out = wheel_rpm_pid_[i].Get_Out();
        int16_t cur = static_cast<int16_t>(out);
        if (cur >  MAX_CURRENT_RAW) cur =  MAX_CURRENT_RAW;
        if (cur < -MAX_CURRENT_RAW) cur = -MAX_CURRENT_RAW;
        Motor_DJI[i].Set_Control_Current(cur);
    }
}

/* ============================================================================
 *   一帧 CAN 控制 4 个电机
 * ========================================================================== */

void Class_Chassis::Send_Group_Frame() {
    Class_Motor_DJI_3508 *group[4] = {&Motor_DJI[0], &Motor_DJI[1],
                                      &Motor_DJI[2], &Motor_DJI[3]};
    // ch=0, tx_id=0x200 — 与 Init 中保持一致
    Class_Motor_DJI_3508::Send_Group(LinkX_Handler, /*can_channel=*/0,
                                     /*tx_can_id=*/0x200U, group);
}

/* ============================================================================
 *   反解: 4 轮 ω → (vx, vy, ω_chassis), 用于遥测/调试。
 *   X 型 4 轮全向轮的伪逆解析解 (L 相同时):
 *     vx = (sum  cos(α_i)·v_wheel_i) / 2
 *     vy = (sum  sin(α_i)·v_wheel_i) / 2
 *     ω  = (sum  v_wheel_i) / (4·L)
 * ========================================================================== */

void Class_Chassis::Self_Resolution() {
    float sum_x = 0.0f, sum_y = 0.0f, sum_w = 0.0f;
    for (int i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        const float alpha = Roll_Direction[i];
        const float v_wheel = Motor_DJI[i].Get_Now_Wheel_Omega() * Wheel_Radius;
        sum_x += v_wheel * std::cos(alpha);
        sum_y += v_wheel * std::sin(alpha);
        sum_w += v_wheel;
    }
    Now_Velocity_X = sum_x / 2.0f;
    Now_Velocity_Y = sum_y / 2.0f;
    Now_Omega      = sum_w / (4.0f * Wheel_To_Core_Distance[0]);
}
