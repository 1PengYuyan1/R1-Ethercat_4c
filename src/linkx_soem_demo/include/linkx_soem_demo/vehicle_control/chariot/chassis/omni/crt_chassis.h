//
// 全向轮 X 型四驱底盘 (4 × DJI 3508 + C620, 标准 RoboMaster CAN 协议)
// ----------------------------------------------------------------------------
// 公共接口刻意与舵轮版同名 Class_Chassis, 以便 Auto_Pilot / robot.cpp 等
// 共享源文件无需改类型;通过 target_include_directories 的搜索顺序选型,
// 每个 add_executable 编译时只能挂一份 chassis 头目录 + 一份源文件。
// ----------------------------------------------------------------------------
// 机械约定 (与舵轮版相同的 X 型布置):
//   Wheel 0 @  45° (左前)
//   Wheel 1 @ 135° (左后)
//   Wheel 2 @ 225° (右后)
//   Wheel 3 @ 315° (右前)
//   每个轮的滚动方向 α = 安装方位 + π/2 (切向)
//
// 逆运动学:
//   v_wheel_i = vx·cos(α_i) + vy·sin(α_i) + ω·L_i           [m/s]
//   ω_wheel_i = v_wheel_i / R                                [rad/s, 轮边]
//   target_rotor_rpm_i = ω_wheel_i · Reduction · 60/(2π)     [RPM, 电机轴]
//
// 控制环:
//   C620 ESC 自带电流环;速度环在本类用 Class_PID(target rpm, now rpm) 出
//   ±16384 的电流指令, 通过 Class_Motor_DJI_3508::Send_Group 一帧发出。
// ----------------------------------------------------------------------------

#ifndef CRT_CHASSIS_OMNI_H
#define CRT_CHASSIS_OMNI_H

#include "alg_pid.h"
#include "dvc_motor_dji.h"
#include "dvc_ops.h"
#include "linkx.h"
#include "math.h"

#ifndef CHASSIS_WHEEL_NUM
#define CHASSIS_WHEEL_NUM 4
#endif

/** @brief 底盘使能状态 */
enum Enum_Chassis_Control_Type {
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_ENABLE,
};

/** @brief 驱动模式 (保留接口与舵轮版兼容, 全向轮当前两态行为一致) */
enum Enum_Drive_Mode {
    Drive_Mode_MANUAL = 0,
    Drive_Mode_SEMI_AUTO,
};

/**
 * @brief 4 轮全向轮底盘
 *
 * 公共字段 Motor_DJI[4] + OPS 与舵轮版兼容:
 *   * Auto_Pilot 直接读写 chassis_->OPS.Get_* (见 dvc_auto_pilot.cpp)
 *   * robot.cpp CAN_Rx_Callback 通过 Motor_DJI[i].DJI_CAN_Rx_ID 做匹配
 */
class Class_Chassis {
public:
    Class_Motor_DJI_3508 Motor_DJI[CHASSIS_WHEEL_NUM];
    Class_OPS OPS;

    // ---- 生命周期 ----
    void Init(linkx_t *__LinkX_Handler);
    void Init_Motor_Params();

    // ---- 周期回调 (节拍由 task 层对齐, 同舵轮版) ----
    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_2ms_Resolution_PeriodElapsedCallback();
    void TIM_2ms_Control_PeriodElapsedCallback();

    // ---- Getter ----
    inline float Get_Now_Velocity_X() const  { return Now_Velocity_X; }
    inline float Get_Now_Velocity_Y() const  { return Now_Velocity_Y; }
    inline float Get_Now_Omega()      const  { return Now_Omega; }
    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type() const { return Chassis_Control_Type; }
    inline float Get_Target_Velocity_X() const { return Target_Velocity_X; }
    inline float Get_Target_Velocity_Y() const { return Target_Velocity_Y; }
    inline float Get_Target_Omega() const      { return Target_Omega; }
    inline Enum_Drive_Mode Get_Drive_Mode() const { return drive_mode_; }

    // ---- Setter ----
    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type t) { Chassis_Control_Type = t; }
    inline void Set_Target_Velocity_X(float v) { Target_Velocity_X = v; }
    inline void Set_Target_Velocity_Y(float v) { Target_Velocity_Y = v; }
    inline void Set_Target_Omega(float v)      { Target_Omega = v; }
    inline void Set_Drive_Mode(Enum_Drive_Mode m) { drive_mode_ = m; }

protected:
    linkx_t *LinkX_Handler = nullptr;

    // ---- 机械常数 ----
    // 轮直径 152.2mm → R = 0.0761m。3508 P19 减速比 19。
    const float Wheel_Radius = 0.0761f;
    const float Reduction_Ratio = 19.0f;
    // X 型布置, 4 轮到底盘几何中心距离 (m)。这里沿用舵轮版的 0.707 (= sqrt(2)/2),
    // 实际 L 需按车架尺寸校准;不影响平移, 只影响 ω 项的轮速贡献。
    const float Wheel_To_Core_Distance[CHASSIS_WHEEL_NUM] = {0.707f, 0.707f, 0.707f, 0.707f};
    // 安装方位 (从车体 +x 轴逆时针起算)
    const float Wheel_Azimuth[CHASSIS_WHEEL_NUM] = {
        PI / 4.0f,            //  45° 左前
        3.0f * PI / 4.0f,     // 135° 左后
        5.0f * PI / 4.0f,     // 225° 右后
        7.0f * PI / 4.0f,     // 315° 右前
    };
    // 滚动方向 α = 安装方位 + π/2 (切向);若实际安装某轮反向, 把对应元素 ±PI。
    float Roll_Direction[CHASSIS_WHEEL_NUM] = {
        PI / 4.0f       + PI / 2.0f,
        3.0f * PI / 4.0f + PI / 2.0f,
        5.0f * PI / 4.0f + PI / 2.0f,
        7.0f * PI / 4.0f + PI / 2.0f,
    };

    const float MAX_CHASSIS_SPEED = 0.5f;     // m/s
    const float MAX_CHASSIS_OMEGA = 0.5f;     // rad/s
    const int16_t MAX_CURRENT_RAW = 16000;    // 留 384 余量到 ±16384 上限

    // ---- 目标与状态 ----
    float Target_Velocity_X = 0.0f;
    float Target_Velocity_Y = 0.0f;
    float Target_Omega      = 0.0f;

    float Now_Velocity_X = 0.0f;
    float Now_Velocity_Y = 0.0f;
    float Now_Omega      = 0.0f;

    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    Enum_Drive_Mode drive_mode_ = Drive_Mode_MANUAL;

    // ---- 控制中间量 ----
    float   Target_Wheel_Omega[CHASSIS_WHEEL_NUM]    = {0};   // 轮边 rad/s
    float   Target_Rotor_RPM  [CHASSIS_WHEEL_NUM]    = {0};   // 电机轴 RPM
    Class_PID wheel_rpm_pid_  [CHASSIS_WHEEL_NUM];            // RPM → 电流 raw

    // ---- 内部函数 ----
    void Kinematics_Inverse_Resolution();
    void Self_Resolution();
    void Compute_Wheel_PID_And_Cache();
    void Send_Group_Frame();
};

#endif // CRT_CHASSIS_OMNI_H
