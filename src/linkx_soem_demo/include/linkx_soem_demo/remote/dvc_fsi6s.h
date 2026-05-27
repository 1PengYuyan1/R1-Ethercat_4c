#ifndef DVC_FSI6S_H
#define DVC_FSI6S_H

#include <cstdint>
#include <vector>

// =============================================================================
// 富斯 FS-i6S 遥控器 -> 接收机 (FS-iA6B) iBus 输出 -> CH340 USB-UART
// (/dev/rc_receiver, 115200 8N1, 32 字节/帧, 14 通道) 的解析层。
//
// 接线/链路:
//   FS-i6S Tx (2.4G AFHDS-2A) --> FS-iA6B Rx --> iBus(SERVO 接口)
//        --> CH340 USB-UART --> /dev/rc_receiver (udev 软链到 /dev/ttyUSB0)
//
// 期望的 i6S 通道配置 (Tx 菜单 Functions -> Aux Channels 里设置):
//   CH1 (1500±500 us):  Right-H / Aileron   ->  vy
//   CH2 (1500±500 us):  Right-V / Elevator  ->  vx
//   CH3 (1000-2000 us): Left-V  / Throttle  ->  速度缩放 (0.30 ~ 1.00)
//   CH4 (1500±500 us):  Left-H  / Rudder    ->  omega
//   CH5 (1000/2000):    SwA  (2 段)         ->  ENABLE 主开关 (上=SwA_Up=DISABLE,
//   下=SwA_Down=ENABLE) CH6 (1000/1500/2000): SwB (3 段)        ->  Modifier
//   (SwB_Up/SwB_Mid/SwB_Down 三态常驻) CH7 (1000/1500/2000): SwC (3 段) ->
//   上中下边沿 (SwC_Up/SwC_Mid/SwC_Down) CH8 (1000/2000):    SwD  (2 段) ->
//   边沿 (SwD_Up/SwD_Down) CH9 (1000-2000):    VrA  (旋钮)         ->  辅助
//   0..1 标量, 暂未挂任何动作 CH10 (1000/2000):   Key1 (背面按键)     ->
//   按下边沿触发 Key1
//
// 按键编码 (uint32_t) — "槽位编码":
//   bit 0      (0x00001): Key1                          (背面按键 1 按下边沿)
//   bits 4-5   (0x00010 / 0x00020): SwA_Up / SwA_Down   (2 段, 各占 1 bit)
//   bits 8-9   (0x00100 / 0x00200): SwD_Up / SwD_Down   (2 段, 各占 1 bit)
//   bits 12-13 (0x01000 / 0x02000 / 0x03000): SwB enum  (3 段, Up/Mid/Down)
//   bits 16-17 (0x10000 / 0x20000 / 0x30000): SwC enum  (3 段, Up/Mid/Down)
//
// SwB / SwC 是 enum 编码 (3 个值占同一 nibble 的 2 个 bit), 用 MASK + 等号判:
//   if ((code & FSi6S_SwB_MASK) == FSi6S_SwB_Up) { ... m1 modifier 在位 ... }
//   if ((code & FSi6S_SwC_MASK) == FSi6S_SwC_Up) { ... SwC 拨向上的边沿 ... }
//
// 其余 (SwA / SwD / Key1) 都是独立 bit, 直接用 & 检测即可。
//
// 行为约定:
//   - SwB 永远反映当前 3 段位置, 在每帧 code 中常驻 (Up / Mid / Down 三选一)
//   - SwC 仅在拨杆位置变化的那一拍 emit (上/中/下 都会 emit, 包括回中)
//   - SwD / Key1 仅在 2 段拨杆变化的那一拍 emit
//   - SwA 上位短路 (DISABLE): 立即返回 SwA_Up 单独, 不与其他位共存
//   - SwA 下位 (ENABLE): 无其他边沿时 code = SwB_state | SwA_Down; 有边沿时
//   SwA_Down 不入 code (避免长按计数被无关帧打断,后续 ms 再恢复累计)
// =============================================================================

#define FSi6S_Key_IDLE 0x00000

#define FSi6S_Key1_UP 0x00001    // 背面按键 1 (CH10 按下边沿)
#define FSi6S_Key1_DOWN 0x00002  // 背面按键 1 (CH10 弹起边沿)

#define FSi6S_SwA_Up 0x00010    // SwA 上位 (Disable, fast)
#define FSi6S_SwA_Down 0x00020  // SwA 下位 (Enable, slow long-press)
#define FSi6S_SwA_MASK 0x00030

#define FSi6S_SwD_Up 0x00100    // SwD 上位边沿
#define FSi6S_SwD_Down 0x00200  // SwD 下位边沿
#define FSi6S_SwD_MASK 0x00300

#define FSi6S_SwB_Up 0x01000    // SwB 上位 (modifier1, 常驻)
#define FSi6S_SwB_Mid 0x02000   // SwB 中位 (无 modifier, 常驻)
#define FSi6S_SwB_Down 0x03000  // SwB 下位 (modifier2, 常驻)
#define FSi6S_SwB_MASK 0x03000

#define FSi6S_SwC_Up 0x10000    // SwC 上位边沿
#define FSi6S_SwC_Mid 0x20000   // SwC 中位边沿 (回中)
#define FSi6S_SwC_Down 0x30000  // SwC 下位边沿
#define FSi6S_SwC_MASK 0x30000

struct Struct_FSi6S_Command {
  float vx = 0.0f;
  float vy = 0.0f;
  float omega = 0.0f;
};

/**
 * @brief FS-i6S 遥控器解算器
 *        输入 14 个通道的 PWM 微秒值 (典型 1000-2000us, 中位 1500us),
 *        输出底盘 Twist + 按键码 + 速度缩放。按键码为 uint32_t 槽位编码。
 */
class Class_FSi6S {
 public:
  void Set_Control_Params(float max_v, float max_w, float deadzone);

  /// 解算底盘速度 (vx, vy, omega), 单位与 max_v/max_w 一致
  Struct_FSi6S_Command Resolve_Chassis(
      const std::vector<uint16_t>& channels) const;

  /// 解算按键码, 内部维护 SwC/SwD/Key1 的边沿状态 (非 const)
  uint32_t Resolve_Button_Code(const std::vector<uint16_t>& channels);

  /// 左摇杆 (Throttle, CH3) 推到顶 -> 1.0, 静止中位 -> 0.65, 压到底 -> 0.30
  float Resolve_Speed_Scale(const std::vector<uint16_t>& channels) const;

  /// VrA (CH9) 旋钮归一到 [0, 1]; robot 端按线性映射决定底盘 vx/vy/omega 加减速上限
  float Resolve_VrA_Unit(const std::vector<uint16_t>& channels) const;

  /// SwA Up 持位累计达 kStartLongPressMs 翻 enable; SwA Down 持位累计达
  /// kBackLongPressMs 翻 disable。"持位" 判定基于 SwA bit 是否 set +
  /// 无其他边沿。
  void Update_Control_Enable(uint32_t key_code, uint32_t dt_ms = 1U);
  bool Is_Control_Enabled() const { return is_ros_control_enabled_; }

  /// 相对上一次 key_code 是否发生变化; 用于功能键边沿分发。
  bool Check_Key_Rising_Edge(uint32_t key_code, uint32_t* rising_key = nullptr);

 private:
  float robot_velocity_max_ = 1.0f;
  float robot_rotation_max_ = 1.0f;
  float deadzone_ = 0.05f;

  bool is_ros_control_enabled_ = false;
  uint32_t start_press_ms_ = 0;
  uint32_t back_press_ms_ = 0;
  uint32_t last_button_code_ = FSi6S_Key_IDLE;

  // SwC/SwD 边沿检测的上一拍位置 (3 段: 0/1/2; 2 段: 0/2)
  uint8_t last_swc_pos_ = 1U;
  uint8_t last_swd_pos_ = 1U;
  uint8_t last_key1_pos_ = 0U;
  bool swc_initialized_ = false;
  bool swd_initialized_ = false;
  bool key1_initialized_ = false;

  /// 1500±500us -> [-1, +1] 钳位后加死区
  float Apply_Stick_Channel(uint16_t pwm_us) const;
  /// 1000-2000us -> [0, 1] 线性 (用于 Throttle)
  static float Channel_To_Unit(uint16_t pwm_us);
  /// 2 段: <1500=0, >=1500=2
  static uint8_t Switch_Pos_2(uint16_t pwm_us);
  /// 3 段: <1250=0(下), 1250-1750=1(中), >1750=2(上)
  static uint8_t Switch_Pos_3(uint16_t pwm_us);
};

#endif  // DVC_FSI6S_H
