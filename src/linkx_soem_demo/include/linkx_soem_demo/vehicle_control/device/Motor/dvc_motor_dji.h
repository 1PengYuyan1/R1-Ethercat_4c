//
// DJI M3508 / C620 ESC 驱动 (标准 RoboMaster 协议)
// ----------------------------------------------------------------------------
// 协议要点:
//   * 反馈帧 ID 0x201..0x208 (电调 ID 1..8), 8 字节大端:
//       [0..1] 机械角度 0..8191 → 0..2π (单圈)
//       [2..3] int16 转子 RPM (电机轴, 未除减速比)
//       [4..5] int16 实际扭矩电流 (C620 内部电流环测量)
//       [6]    转子温度 °C
//       [7]    保留
//   * 控制帧 ID 0x200 控制 ESC ID 1-4; ID 0x1FF 控制 ESC ID 5-8。
//     8 字节 = 4 × int16 big-endian, 单位为电流指令 -16384..16384 (= ±20A peak)
//   * C620 自带电流闭环, 上位机仅需输出电流; 速度/位置闭环需在 chassis 层实现
//   * 无 enable/disable/save-zero 帧; 上电即工作。Alive 判定靠 Rx flag 增量。
// ----------------------------------------------------------------------------

#ifndef DVC_MOTOR_DJI_H
#define DVC_MOTOR_DJI_H

#include <cstdint>
#include "linkx.h"
#include "linkx4c_handler.h"

/** @brief 电机存活状态 (由 TIM_Alive_PeriodElapsedCallback 维护) */
enum Enum_Motor_DJI_Status {
    Motor_DJI_Status_DISABLE = 0,
    Motor_DJI_Status_ENABLE,
};

/** @brief 反馈帧字段布局 (8 字节, big-endian on the wire) */
struct Struct_Motor_DJI_CAN_Rx_Data {
    uint8_t angle_hi;
    uint8_t angle_lo;
    uint8_t rpm_hi;
    uint8_t rpm_lo;
    uint8_t current_hi;
    uint8_t current_lo;
    uint8_t temperature;     // °C
    uint8_t reserved;
} __attribute__((packed));

/** @brief 解析后状态 (对外通过 Get_Now_* 读取) */
struct Struct_Motor_DJI_Rx_Data {
    float    Now_Rad;              // 累计 (multi-turn) 转子角度 [rad]
    float    Now_RPM;              // 转子转速 [RPM]
    float    Now_Current;          // 实际电流, 原始 int16 单位 (≈ ±20A 对应 ±16384)
    float    Now_Temperature_C;    // 转子温度 [°C]
    uint16_t Pre_Angle_Raw;        // 上一次原始 0..8191 (用于 unwrap)
    int32_t  Total_Round;          // 累计圈数
    uint8_t  First_Update_Flag;
};

/**
 * @brief Reusable, DJI M3508 + C620 电调, 标准 RoboMaster CAN 协议
 *
 * 一个对象对应一个电机/电调。4 个电机的电流指令由 Send_Group 静态方法
 * 打包成一帧发出 (CAN 协议规定每帧驱动 4 个电调)。
 */
class Class_Motor_DJI_3508 {
public:
    /** @brief 反馈 CAN ID (0x201..0x208) */
    uint16_t DJI_CAN_Rx_ID = 0;
    /** @brief 发送 CAN ID (0x200 for ID 1-4 / 0x1FF for ID 5-8) */
    uint16_t DJI_CAN_Tx_ID = 0x200;
    /** @brief 本电机在发送帧 4 槽位中的索引 (0..3) */
    uint8_t  Slot_In_Group = 0;

    /**
     * @brief 电机初始化
     * @param __LinkX_Handler    LinkX 句柄
     * @param __CAN_Channel      LinkX 上的 CAN 通道
     * @param __CAN_Rx_ID        反馈帧 CAN ID (0x201..0x208)
     * @param __CAN_Tx_ID        控制帧基地址 (0x200 或 0x1FF)
     * @param __Slot_In_Group    在控制帧中的槽位 (0..3, 对应 Rx_ID 个位)
     * @param __Reduction_Ratio  减速比 (P19 标配 = 19.0)
     */
    void Init(linkx_t *__LinkX_Handler, uint8_t __CAN_Channel,
              uint16_t __CAN_Rx_ID, uint16_t __CAN_Tx_ID,
              uint8_t __Slot_In_Group, float __Reduction_Ratio = 19.0f);

    // ---- 状态读取 ----
    inline Enum_Motor_DJI_Status Get_Status() const     { return Motor_DJI_Status; }
    inline float Get_Now_Rad() const                     { return data.Now_Rad; }                       // 转子轴
    inline float Get_Now_RPM() const                     { return data.Now_RPM; }                       // 转子轴
    inline float Get_Now_Rotor_Omega() const             { return data.Now_RPM * 2.0f * 3.14159265358979f / 60.0f; }
    inline float Get_Now_Wheel_Omega() const             { return Get_Now_Rotor_Omega() / Reduction_Ratio; }
    inline float Get_Now_Current() const                 { return data.Now_Current; }
    inline float Get_Now_Temperature() const             { return data.Now_Temperature_C; }
    inline float Get_Reduction_Ratio() const             { return Reduction_Ratio; }

    // ---- 控制设定 ----

    /**
     * @brief 写入电流指令 (raw int16, ±16384 = ±20A peak)
     *        clamp 在 [-16384, 16384], 由 chassis 层 PID 闭环输出。
     */
    void Set_Control_Current(int16_t cmd_raw);
    inline int16_t Get_Control_Current() const           { return ctrl_current_raw_; }

    /**
     * @brief CAN 接收回调 — SOEM 主循环在收到本电机帧时调用
     * @param Rx_Data 8 字节原始数据
     */
    void CAN_RxCpltCallback(uint8_t *Rx_Data);

    /** @brief 存活检测回调 — 通过 Flag 增量判断本周期是否收过帧 */
    void TIM_Alive_PeriodElapsedCallback();

    /**
     * @brief 群发: 把 4 个电机的电流缓存打包成一帧 CAN 发出
     *        4 个电机必须挂同一 LinkX_Handler + 同一 CAN_Channel + 同一 Tx_ID。
     *        motors[i]==nullptr 的槽位填 0。
     */
    static void Send_Group(linkx_t *handler, uint8_t can_channel,
                           uint16_t tx_can_id,
                           Class_Motor_DJI_3508 *motors[4]);

protected:
    linkx_t *LinkX_Handler = nullptr;
    uint8_t  CAN_Channel = 0;
    float    Reduction_Ratio = 19.0f;

    int16_t  ctrl_current_raw_ = 0;
    Enum_Motor_DJI_Status Motor_DJI_Status = Motor_DJI_Status_DISABLE;
    Struct_Motor_DJI_Rx_Data data{};

    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    void Data_Process(uint8_t *Rx_Data);
};

#endif // DVC_MOTOR_DJI_H
