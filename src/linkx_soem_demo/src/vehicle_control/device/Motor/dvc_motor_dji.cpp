//
// DJI M3508 / C620 ESC 驱动实现
// ----------------------------------------------------------------------------
// 反馈帧字段是 big-endian。解析时直接位拼接。
// multi-turn unwrap: 比较本次 raw (0..8191) 与上次 raw, |Δ|>4096 视为越界,
// Total_Round ±1 调整。Now_Rad = (Total_Round * 8192 + raw) * 2π / 8192。
// ----------------------------------------------------------------------------

#include "dvc_motor_dji.h"

#include <cmath>
#include <cstring>

namespace {
constexpr int    DJI_ANGLE_RAW_RANGE = 8192;
constexpr float  DJI_TWO_PI          = 6.28318530717958647692f;
}  // namespace

/* ============================================================================
 *   初始化
 * ========================================================================== */

void Class_Motor_DJI_3508::Init(linkx_t *__LinkX_Handler, uint8_t __CAN_Channel,
                                uint16_t __CAN_Rx_ID, uint16_t __CAN_Tx_ID,
                                uint8_t __Slot_In_Group, float __Reduction_Ratio) {
    LinkX_Handler   = __LinkX_Handler;
    CAN_Channel     = __CAN_Channel;
    DJI_CAN_Rx_ID   = __CAN_Rx_ID;
    DJI_CAN_Tx_ID   = __CAN_Tx_ID;
    Slot_In_Group   = (__Slot_In_Group > 3) ? 3 : __Slot_In_Group;
    Reduction_Ratio = __Reduction_Ratio;

    ctrl_current_raw_ = 0;
    Motor_DJI_Status  = Motor_DJI_Status_DISABLE;
    std::memset(&data, 0, sizeof(data));
    Flag = 0;
    Pre_Flag = 0;
}

/* ============================================================================
 *   控制指令
 * ========================================================================== */

void Class_Motor_DJI_3508::Set_Control_Current(int16_t cmd_raw) {
    if (cmd_raw >  16384) cmd_raw =  16384;
    if (cmd_raw < -16384) cmd_raw = -16384;
    ctrl_current_raw_ = cmd_raw;
}

/* ============================================================================
 *   群发: 一帧 CAN 控制 4 个电机
 * ========================================================================== */

void Class_Motor_DJI_3508::Send_Group(linkx_t *handler, uint8_t can_channel,
                                     uint16_t tx_can_id,
                                     Class_Motor_DJI_3508 *motors[4]) {
    if (handler == nullptr) return;

    uint8_t buf[8] = {0};
    for (int i = 0; i < 4; ++i) {
        int16_t cur = (motors[i] != nullptr) ? motors[i]->ctrl_current_raw_ : 0;
        // big-endian 打包
        buf[i * 2 + 0] = static_cast<uint8_t>((cur >> 8) & 0xFF);
        buf[i * 2 + 1] = static_cast<uint8_t>( cur       & 0xFF);
    }
    linkx_quick_can_send(handler, can_channel, tx_can_id, buf);
}

/* ============================================================================
 *   接收 / 周期回调
 * ========================================================================== */

void Class_Motor_DJI_3508::CAN_RxCpltCallback(uint8_t *Rx_Data) {
    Flag += 1;
    Data_Process(Rx_Data);
}

void Class_Motor_DJI_3508::TIM_Alive_PeriodElapsedCallback() {
    Motor_DJI_Status = (Flag == Pre_Flag) ? Motor_DJI_Status_DISABLE : Motor_DJI_Status_ENABLE;
    Pre_Flag = Flag;
}

/* ============================================================================
 *   解析反馈帧 (big-endian, 多圈展开)
 * ========================================================================== */

void Class_Motor_DJI_3508::Data_Process(uint8_t *Rx_Data) {
    const uint16_t angle_raw = (static_cast<uint16_t>(Rx_Data[0]) << 8) |
                                static_cast<uint16_t>(Rx_Data[1]);
    const int16_t  rpm_raw   = static_cast<int16_t>(
        (static_cast<uint16_t>(Rx_Data[2]) << 8) |
         static_cast<uint16_t>(Rx_Data[3]));
    const int16_t  cur_raw   = static_cast<int16_t>(
        (static_cast<uint16_t>(Rx_Data[4]) << 8) |
         static_cast<uint16_t>(Rx_Data[5]));
    const uint8_t  temp_c    = Rx_Data[6];

    // multi-turn unwrap
    if (data.First_Update_Flag == 0) {
        data.Pre_Angle_Raw     = angle_raw;
        data.Total_Round       = 0;
        data.First_Update_Flag = 1;
    } else {
        const int32_t delta = static_cast<int32_t>(angle_raw) -
                              static_cast<int32_t>(data.Pre_Angle_Raw);
        if (delta >  DJI_ANGLE_RAW_RANGE / 2) {
            data.Total_Round -= 1;  // 反向越界 (例如 50 -> 8100, 实际倒退)
        } else if (delta < -DJI_ANGLE_RAW_RANGE / 2) {
            data.Total_Round += 1;  // 正向越界 (例如 8100 -> 50, 实际前进)
        }
        data.Pre_Angle_Raw = angle_raw;
    }

    const float turns = static_cast<float>(data.Total_Round) +
                        static_cast<float>(angle_raw) / static_cast<float>(DJI_ANGLE_RAW_RANGE);
    data.Now_Rad           = turns * DJI_TWO_PI;
    data.Now_RPM           = static_cast<float>(rpm_raw);
    data.Now_Current       = static_cast<float>(cur_raw);
    data.Now_Temperature_C = static_cast<float>(temp_c);
}
