#include "linkx_soem_demo/remote/dvc_fsi6s.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr uint32_t kStartLongPressMs = 500;
constexpr uint32_t kBackLongPressMs  = 100;

// iBus PWM 规格: 通道值范围 1000-2000us (扩展可到 988-2012us), 中位 1500us。
constexpr uint16_t kChMid      = 1500U;
constexpr uint16_t kChHalfSwing = 500U;   // 单边摆程,做归一化用

// i6S 通道索引 (0-based; CH1 = idx 0)
constexpr size_t kIdx_RH       = 0U;  // CH1 Aileron  -> vy
constexpr size_t kIdx_RV       = 1U;  // CH2 Elevator -> vx
constexpr size_t kIdx_LV       = 2U;  // CH3 Throttle -> speed_scale
constexpr size_t kIdx_LH       = 3U;  // CH4 Rudder   -> omega
constexpr size_t kIdx_SwA      = 4U;  // CH5 ENABLE
constexpr size_t kIdx_SwB      = 5U;  // CH6 Modifier
constexpr size_t kIdx_SwC      = 6U;  // CH7 Up/Down 边沿
constexpr size_t kIdx_SwD      = 7U;  // CH8 Left/Right 边沿
constexpr size_t kIdx_VrA      = 8U;  // CH9 旋钮, 辅助 0..1 标量
constexpr size_t kIdx_Key1     = 9U;  // CH10 背面按键 1, 按下 -> Key_X 边沿

inline uint16_t Get_Channel(const std::vector<uint16_t> &ch, size_t idx)
{
    // 缺通道按中位返回 (= 不触发任何动作)
    return (idx < ch.size()) ? ch[idx] : kChMid;
}
}  // namespace

void Class_FSi6S::Set_Control_Params(float max_v, float max_w, float deadzone)
{
    robot_velocity_max_ = max_v;
    robot_rotation_max_ = max_w;
    deadzone_ = deadzone;
}

float Class_FSi6S::Apply_Stick_Channel(uint16_t pwm_us) const
{
    // 1500±500us -> [-1, +1], 加死区
    const float raw = (static_cast<float>(pwm_us) - static_cast<float>(kChMid)) /
                      static_cast<float>(kChHalfSwing);
    const float clamped = std::clamp(raw, -1.0f, 1.0f);
    const float abs_v = std::abs(clamped);
    if (abs_v < deadzone_) return 0.0f;
    const float sign = (clamped > 0.0f) ? 1.0f : -1.0f;
    return sign * (abs_v - deadzone_) / (1.0f - deadzone_);
}

float Class_FSi6S::Channel_To_Unit(uint16_t pwm_us)
{
    // 1000us -> 0.0, 2000us -> 1.0
    const float clamped = std::clamp(static_cast<float>(pwm_us), 1000.0f, 2000.0f);
    return (clamped - 1000.0f) / 1000.0f;
}

uint8_t Class_FSi6S::Switch_Pos_2(uint16_t pwm_us)
{
    return (pwm_us >= kChMid) ? 2U : 0U;
}

uint8_t Class_FSi6S::Switch_Pos_3(uint16_t pwm_us)
{
    if (pwm_us < 1250U) return 0U;   // 下位
    if (pwm_us > 1750U) return 2U;   // 上位
    return 1U;                        // 中位
}

Struct_FSi6S_Command Class_FSi6S::Resolve_Chassis(const std::vector<uint16_t> &channels) const
{
    Struct_FSi6S_Command cmd;

    // 摇杆方向约定:
    //   右摇杆: 推前 vx>0, 推右 vy<0 (右手坐标系, 与底盘 OPS 帧对齐)
    //   左摇杆: 推右 omega<0 (CW 为正, 与底盘 omega 约定一致)
    //
    // i6S 默认 1500us 居中, 推到末端 2000us。Aileron/Rudder 推右数值升高,
    // 因此 vy/omega 取符号反相, 保持"摇杆动作 = 车辆动作"直觉。
    const float rv = Apply_Stick_Channel(Get_Channel(channels, kIdx_RV));   // CH2 推前 +1
    const float rh = Apply_Stick_Channel(Get_Channel(channels, kIdx_RH));   // CH1 推右 +1
    const float lh = Apply_Stick_Channel(Get_Channel(channels, kIdx_LH));   // CH4 推右 +1

    cmd.vx    =  rv * robot_velocity_max_;
    cmd.vy    = -rh * robot_velocity_max_;
    cmd.omega = -lh * robot_rotation_max_;
    return cmd;
}

float Class_FSi6S::Resolve_Speed_Scale(const std::vector<uint16_t> &channels) const
{
    // Throttle 0..1 -> 速度缩放 0.30..1.00, 中位 (0.5) 对应 0.65
    // Channel_To_Unit 已钳位 th 到 [0,1], 故 scale 必在 [0.30, 1.00]
    const float th = Channel_To_Unit(Get_Channel(channels, kIdx_LV));
    return 0.30f + th * 0.70f;
}

float Class_FSi6S::Resolve_VrA_Unit(const std::vector<uint16_t> &channels) const
{
    // VrA 旋钮原始 1000-2000us, 归一到 [0, 1]; 未连通道 (默认 1500us) 返回 0.5
    return Channel_To_Unit(Get_Channel(channels, kIdx_VrA));
}

uint32_t Class_FSi6S::Resolve_Button_Code(const std::vector<uint16_t> &channels)
{
    // ===== 1. SwA (DISABLE/ENABLE) 高优先级单键 (语义已翻转: Up=DISABLE, Down=ENABLE) =====
    const uint8_t swa = Switch_Pos_2(Get_Channel(channels, kIdx_SwA));
    if (swa == 2U) {
        // SwA 上位 -> 强制返回 SwA_Up, 由 Update_Control_Enable 翻 disable
        // 拨向上时直接断使能, 不允许同时还触发 SwB/SwC/SwD
        return FSi6S_SwA_Up;
    }

    uint32_t code = FSi6S_Key_IDLE;

    // ===== 2. SwB modifier (3 段 enum, 常驻) =====
    // SwB 在 code 中永远反映当前位置 (Up / Mid / Down), 不做边沿过滤。
    // robot.cpp 用 (code & SwB_MASK) == SwB_Up 这种方式判 modifier 在位。
    const uint8_t swb = Switch_Pos_3(Get_Channel(channels, kIdx_SwB));
    if (swb == 2U)       code |= FSi6S_SwB_Up;
    else if (swb == 1U)  code |= FSi6S_SwB_Mid;
    else                 code |= FSi6S_SwB_Down;

    // ===== 3. SwC: 3 段拨杆的边沿 (上/中/下都会 emit) =====
    //   拨向上 (X->上): 触发 SwC_Up
    //   回中位 (上/下->中): 触发 SwC_Mid (新增)
    //   拨向下 (X->下): 触发 SwC_Down
    // 仅在状态变化的那一拍 emit, 维持稳态时 SwC 不入 code。
    const uint8_t swc = Switch_Pos_3(Get_Channel(channels, kIdx_SwC));
    if (!swc_initialized_) { last_swc_pos_ = swc; swc_initialized_ = true; }
    if (swc != last_swc_pos_) {
        if (swc == 2U)       code |= FSi6S_SwC_Up;
        else if (swc == 1U)  code |= FSi6S_SwC_Mid;
        else                 code |= FSi6S_SwC_Down;
        last_swc_pos_ = swc;
    }

    // ===== 4. SwD: 2 段拨杆的 Up/Down 边沿 =====
    const uint8_t swd = Switch_Pos_2(Get_Channel(channels, kIdx_SwD));
    if (!swd_initialized_) { last_swd_pos_ = swd; swd_initialized_ = true; }
    if (swd != last_swd_pos_) {
        if (swd == 2U)       code |= FSi6S_SwD_Up;
        else                 code |= FSi6S_SwD_Down;
        last_swd_pos_ = swd;
    }

    // ===== 4b. Key1 (CH10, 背面按键): 按下边沿 + 弹起边沿 =====
    //   按下 (pos: 0 -> 2)  -> Key1_UP  (0x01)
    //   弹起 (pos: 2 -> 0)  -> Key1_DOWN (0x02)
    const uint8_t key1 = Switch_Pos_2(Get_Channel(channels, kIdx_Key1));
    if (!key1_initialized_) { last_key1_pos_ = key1; key1_initialized_ = true; }
    if (key1 != last_key1_pos_) {
        if (key1 == 2U)      code |= FSi6S_Key1_UP;
        else                 code |= FSi6S_Key1_DOWN;
        last_key1_pos_ = key1;
    }

    // ===== 5. SwA 下位时, 若无任何边沿则叠加 SwA_Down 让长按计时累计 (ENABLE) =====
    //   "无边沿" = code 里只有 SwB 常驻状态, 没有 Key1/SwC/SwD bit
    //   Update_Control_Enable 见到 code 只含 SwA_Down + SwB 状态时累加 start_press_ms_
    if (swa == 0U) {
        const uint32_t event_bits = code & ~(FSi6S_SwB_MASK | FSi6S_SwA_MASK);
        if (event_bits == 0U) {
            code |= FSi6S_SwA_Down;
        }
        // else: 有 SwC/SwD/Key1 边沿同帧, 不叠 SwA_Down, 长按计数本帧清零;
        //       下一帧边沿清掉后, SwA_Down 又会重新进 code, 计数恢复累加。
    }

    return code;
}

void Class_FSi6S::Update_Control_Enable(uint32_t key_code, uint32_t dt_ms)
{
    // 语义翻转后:
    //   SwA 下位且无任何边沿时, code 含 SwA_Down bit -> start_press_ms_ 累加, 达 500ms 翻 ENABLE;
    //   有其他边沿同帧时 SwA_Down bit 不在, 计数清零 (防误触);
    //   SwA 上位短路 code = SwA_Up 单独, back_press_ms_ 累加, 达 100ms 翻 DISABLE。
    const bool swa_down_held =
        (key_code & FSi6S_SwA_Down) != 0U &&
        ((key_code & ~(FSi6S_SwA_MASK | FSi6S_SwB_MASK)) == 0U);
    if (swa_down_held) {
        start_press_ms_ += dt_ms;
        if (start_press_ms_ >= kStartLongPressMs) is_ros_control_enabled_ = true;
    } else {
        start_press_ms_ = 0;
    }

    if (key_code == FSi6S_SwA_Up) {
        back_press_ms_ += dt_ms;
        if (back_press_ms_ >= kBackLongPressMs) is_ros_control_enabled_ = false;
    } else {
        back_press_ms_ = 0;
    }
}

bool Class_FSi6S::Check_Key_Rising_Edge(uint32_t key_code, uint32_t *rising_key)
{
    // "rising" = 本帧 code 与上一帧不同。返回 *rising_key = 整个 code, robot.cpp
    // 再用 mask + equal / bit-and 提取具体 switch 的位置。SwB 常驻意味着只要 SwB
    // 改了位置也会触发 rising; 但 rising_key 里若没有 Key1/SwC/SwD bit, 调用方
    // 应识别"纯 SwB 状态变化"并跳过分发。
    const bool is_rising = (key_code != FSi6S_Key_IDLE) && (key_code != last_button_code_);
    if (is_rising && rising_key) *rising_key = key_code;
    last_button_code_ = key_code;
    return is_rising;
}
