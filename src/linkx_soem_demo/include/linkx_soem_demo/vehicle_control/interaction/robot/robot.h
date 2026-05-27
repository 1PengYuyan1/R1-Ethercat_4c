#ifndef ROBOT_H
#define ROBOT_H

#include "crt_chassis.h"
#include "crt_clamp.h"
#include "dvc_auto_pilot.h"
#include "linkx_soem_demo/remote/dvc_fsi6s.h"
#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>

/**
 * @brief 机器人交互层 (全向轮工况)
 *        整合底盘控制、ROS2 遥控桥接、CAN 接收分发与定时回调,
 *        作为 task 层与执行层 (Chassis) 之间的唯一入口。
 *
 *        与舵轮版差异:
 *          * Chassis 公共字段从 Motor_Steer/ODrive/Encoder 改为 Motor_DJI[4]
 *          * 移除 Manual_Yaw_Hold (航向慢漂纠偏) 与对应 CSV 日志
 *          * 移除 Steer_Calibration 初始化
 *          * CAN_Rx_Callback ch0 改派给 DJI 反馈帧 (0x201..0x204)
 */
class Class_Robot
{
public:
    /* ===== 子系统 ===== */
    Class_Chassis Chassis;
    Class_Auto_Pilot Auto_Pilot;
    Class_Clamp Clamp;

    /* ===== 生命周期 ===== */
    void Init(linkx_t *__LinkX_Handler, linkx_t *__LinkX_FD_Handler = nullptr);
    void Start_ROS2_Remote_Bridge();
    void Stop_ROS2_Remote_Bridge();

    /* ===== 周期回调 (由 task 层按节拍触发) ===== */
    void TIM_1ms_Calculate_Callback();
    void TIM_2ms_Calculate_PeriodElapsedCallback();
    void TIM_100ms_Alive_PeriodElapsedCallback();

    /* ===== CAN 接收 =====
     * @param Module_Id  0 = 经典 LinkX (slave_id=1, DJI 3508 ×4 / OPS)
     *                   1 = CAN-FD LinkX (slave_id=2, ch0=Clamp DM)
     */
    void CAN_Rx_Callback(uint8_t Module_Id, uint8_t CAN_Channel, uint32_t CAN_ID, uint8_t *CAN_Data, uint8_t CAN_DLen);

    /* ===== 调试观测接口 (只读) ===== */
    inline uint32_t Get_Debug_Remote_Key_Code() const { return debug_remote_key_code_; }
    inline bool     Get_Debug_Remote_Is_Recent() const { return debug_remote_is_recent_; }
    inline bool     Get_Debug_Remote_Is_Enabled() const { return debug_remote_is_enabled_; }
    inline float    Get_Debug_Remote_Vx() const { return debug_remote_vx_; }
    inline float    Get_Debug_Remote_Vy() const { return debug_remote_vy_; }
    inline float    Get_Debug_Remote_Omega() const { return debug_remote_omega_; }

protected:
    /**
     * @brief ROS2 → 主线程之间的命令快照
     *        ros_spin_thread_ 写入, 主控线程读出;通过 ros_cmd_mutex_ 保护。
     */
    struct Ros_Remote_Command
    {
        float    vx = 0.0f;
        float    vy = 0.0f;
        float    omega = 0.0f;
        float    vra = 0.65f;       ///< VrA 旋钮 0..1, 决定底盘加减速 slew rate
        uint32_t buttons = 0;
        int64_t  last_update_ns = 0;
        bool     has_twist = false;
        bool     has_buttons = false;
        bool     has_vra = false;
    };

    /* ===== 资源句柄与子设备 ===== */
    linkx_t          *LinkX_Handler    = nullptr;
    linkx_t          *LinkX_FD_Handler = nullptr;
    Class_FSi6S    fsi6s_remote_;

    /* ===== ROS2 桥接状态 ===== */
    std::mutex             ros_cmd_mutex_;
    Ros_Remote_Command     ros_cmd_;
    std::thread            ros_spin_thread_;
    std::atomic<bool>      ros_bridge_running_{false};
    bool                   ros_initialized_here_ = false;

    /* ===== 底盘速度 slew rate 状态 (VrA 旋钮控制加减速) =====
     * 在 _Apply_Chassis_Velocity 中按每 tick (1ms) 限幅 vx/vy/omega 的变化率;
     * 加速度上限由 VrA (0..1) 线性映射到 [a_min, a_max]。
     * DISABLE 或失能时复位为 0, 避免恢复时还按上次速度起步。 */
    float chassis_cmd_vx_    = 0.0f;
    float chassis_cmd_vy_    = 0.0f;
    float chassis_cmd_omega_ = 0.0f;

    /* ===== SwC/SwD 当前位置缓存 (用于 SwB_Mid 模式下 SwC+SwD 组合分发) ===== */
    uint32_t last_swc_level_ = FSi6S_SwC_Mid;
    uint32_t last_swd_level_ = FSi6S_SwD_Up;

    /* ===== 调试观测缓存 ===== */
    uint32_t debug_remote_key_code_   = 0;
    bool     debug_remote_is_recent_  = false;
    bool     debug_remote_is_enabled_ = false;
    float    debug_remote_vx_         = 0.0f;
    float    debug_remote_vy_         = 0.0f;
    float    debug_remote_omega_      = 0.0f;

    /* ===== 私有控制流程 ===== */
    void _Chassis_Control();
    void _Clamp_Control(uint32_t key_code, bool is_enabled);
    bool _Snapshot_Remote_Command(Ros_Remote_Command &out_cmd, bool &out_is_recent);
    bool _Update_Manual_Enable_State(uint32_t key_code);
    void _Process_Function_Key_Edge(uint32_t key_code, bool is_enabled);
    void _Apply_Chassis_Velocity(float vx, float vy, float omega, float vra, bool active);

    /* ===== ROS2 桥接内部 ===== */
    void _ROS2_Remote_Spin_Loop();
    void _Update_Remote_Twist(float vx, float vy, float omega);
    void _Update_Remote_Buttons(uint32_t buttons);
    void _Update_Remote_Vra(float vra);
};

#endif // ROBOT_H
