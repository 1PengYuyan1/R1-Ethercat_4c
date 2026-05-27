/**
 * @file task.cpp
 * @brief 机器人主控制循环 (task 层, 全向轮工况)。
 *
 *  职责 (仅做调度, 业务全部托管给 Class_Robot / Class_Chassis):
 *    - 初始化 EtherCAT 主站与 LinkX (4×CAN) 适配器;
 *    - 1 ms 节拍:CAN 收 → Robot 周期任务 → CAN 发;
 *    - 周期诊断:CAN 统计 / LIVE 仪表盘;
 *    - 退出时把所有 DJI/夹爪电机置安全态后再关 ROS 桥。
 */

#include "task.h"

#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "robot.h"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include "math.h"

// ============================================================================
//   全局对象 —— 与 main.cpp 共享（main.cpp 中通过 extern 引用 master）
// ============================================================================

Class_Robot   robot;          ///< 机器人逻辑入口
ecat_master_t master;         ///< EtherCAT 主站（信号处理读 is_running）
linkx_t       linkx_dev;      ///< LinkX 4 通道 CAN 适配器（slave_id=1，经典 1Mbps）
linkx_t       linkx_dev_fd;   ///< LinkX 4 通道 CAN-FD 适配器（slave_id=2，ch0 经典 1M / ch1-3 FD）

// ============================================================================
//                             编译期可调参数
// ============================================================================
namespace {

constexpr int      kChannelCount                  = 4;        ///< CAN 通道数
constexpr uint32_t kControlLoopPeriodMs           = 1;        ///< 主循环周期 1 ms (1 kHz)
constexpr uint32_t kTaskPeriod2Ms                 = 2;        ///< 2 ms 任务节拍
constexpr uint32_t kTaskPeriod100Ms               = 100;      ///< 100 ms 任务节拍
constexpr uint32_t kCanStatPrintPeriodMs          = 5000;     ///< CAN 统计打印周期
constexpr uint32_t kLiveDashboardPeriodMs         = 200;      ///< LIVE 仪表盘刷新周期

constexpr bool kEnableCanStatPrint        = false;  // [DIAG] temporarily off to isolate stdout backpressure
constexpr bool kEnableLiveDashboard       = false;  // [DIAG] temporarily off to isolate stdout backpressure

constexpr const char *kDefaultVarDataFile = "var_data/live_variables.log";

std::ofstream g_var_data_stream;        ///< 仪表盘镜像日志 (懒打开)
bool          g_var_data_stream_inited = false;

} // namespace

// ============================================================================
//                                工具函数
// ============================================================================

/**
 * @brief 懒打开变量日志文件流；优先取环境变量 VAR_DATA_FILE 指定的路径。
 *        失败仅 stderr 警告，不影响其他逻辑。
 */
static void Ensure_Var_Data_Stream()
{
    if (g_var_data_stream_inited)
        return;

    const char *path = std::getenv("VAR_DATA_FILE");
    if (path == nullptr || path[0] == '\0')
        path = kDefaultVarDataFile;

    g_var_data_stream.open(path, std::ios::app);
    g_var_data_stream_inited = true;
    if (!g_var_data_stream.is_open())
        std::cerr << "[WARN] Failed to open variable data file: " << path << std::endl;
}

// ============================================================================
//                          诊断打印 —— CAN 统计
// ============================================================================

/**
 * @brief 打印 4 个 CAN 通道的累计 TX/RX/丢帧/丢包率，以及全局合计。
 *        每 kCanStatPrintPeriodMs 调用一次。
 */
static void Print_CAN_Stats(linkx_t *linkx)
{
    std::cout << "\n[CAN-TOTAL]\n";

    uint64_t sum_tx_req_f = 0, sum_tx_req_b = 0;
    uint64_t sum_tx_sent_f = 0, sum_tx_sent_b = 0;
    uint64_t sum_tx_drop_f = 0;
    uint64_t sum_rx_f = 0,      sum_rx_b = 0;

    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        const auto &s = linkx->can_stats[ch];
        const uint16_t q_fill  = linkx->tx_queues[ch].size;
        const uint64_t loss_f  = (s.tx_frames > s.rx_frames) ? (s.tx_frames - s.rx_frames) : 0;
        const double   loss_rt = (s.tx_frames > 0) ? (100.0 * (double)loss_f / (double)s.tx_frames) : 0.0;

        std::cout << "  CH" << ch
                  << " TX_REQ=" << s.tx_enqueued_frames << " frames (" << s.tx_enqueued_bytes << " B)"
                  << " TX_SENT=" << s.tx_frames << " frames (" << s.tx_bytes << " B)"
                  << " TX_DROP=" << s.tx_dropped_frames
                  << " Q=" << q_fill
                  << " RX_TOTAL=" << s.rx_frames << " frames (" << s.rx_bytes << " B)"
                  << " TOTAL_LOSS=" << loss_f
                  << " TOTAL_LOSS_RATE=" << loss_rt << "%"
                  << std::endl;

        sum_tx_req_f  += s.tx_enqueued_frames;
        sum_tx_req_b  += s.tx_enqueued_bytes;
        sum_tx_sent_f += s.tx_frames;
        sum_tx_sent_b += s.tx_bytes;
        sum_tx_drop_f += s.tx_dropped_frames;
        sum_rx_f      += s.rx_frames;
        sum_rx_b      += s.rx_bytes;
    }

    const uint64_t sum_loss_f  = (sum_tx_sent_f > sum_rx_f) ? (sum_tx_sent_f - sum_rx_f) : 0;
    const double   sum_loss_rt = (sum_tx_sent_f > 0) ? (100.0 * (double)sum_loss_f / (double)sum_tx_sent_f) : 0.0;
    std::cout << "  ALL TX_REQ=" << sum_tx_req_f << " frames (" << sum_tx_req_b << " B)"
              << " TX_SENT=" << sum_tx_sent_f << " frames (" << sum_tx_sent_b << " B)"
              << " TX_DROP=" << sum_tx_drop_f
              << " RX_TOTAL=" << sum_rx_f << " frames (" << sum_rx_b << " B)"
              << " TOTAL_LOSS=" << sum_loss_f
              << " TOTAL_LOSS_RATE=" << sum_loss_rt << "%"
              << std::endl;
}

// ============================================================================
//                       诊断打印 —— LIVE 仪表盘
// ============================================================================

/**
 * @brief 清屏并打印仪表盘头，统一两路输出的浮点格式。
 */
static void Render_Dashboard_Header()
{
    std::cout << "\033[2J\033[H";       // 清屏并回到左上角
    std::cout << "[LIVE-DASHBOARD] (refresh " << kLiveDashboardPeriodMs << " ms)\n";
    std::cout << std::fixed << std::setprecision(3);
    if (g_var_data_stream.is_open())
        g_var_data_stream << std::fixed << std::setprecision(3);
}

/**
 * @brief 打印遥控/底盘目标量/底盘当前态（仅终端，方便排障）。
 */
static void Render_Dashboard_Remote()
{
    std::cout << "\n[REMOTE]\n"
              << "  chassis_ctrl=" << static_cast<int>(robot.Chassis.Get_Chassis_Control_Type())
              << " key=0x" << std::hex << robot.Get_Debug_Remote_Key_Code() << std::dec
              << " recent="    << static_cast<int>(robot.Get_Debug_Remote_Is_Recent())
              << " enabled="   << static_cast<int>(robot.Get_Debug_Remote_Is_Enabled())
              << " recv_vx="   << robot.Get_Debug_Remote_Vx()
              << " recv_vy="   << robot.Get_Debug_Remote_Vy()
              << " recv_omega="<< robot.Get_Debug_Remote_Omega()
              << " target_vx=" << robot.Chassis.Get_Target_Velocity_X()
              << " target_vy=" << robot.Chassis.Get_Target_Velocity_Y()
              << " target_omega=" << robot.Chassis.Get_Target_Omega()
              << " now_vx="    << robot.Chassis.Get_Now_Velocity_X()
              << " now_vy="    << robot.Chassis.Get_Now_Velocity_Y()
              << " now_omega=" << robot.Chassis.Get_Now_Omega()
              << "\n";
}

/**
 * @brief 打印 4 个 DJI 3508 / C620 电机状态 (终端 + 文件)。
 */
static void Render_Dashboard_DJI()
{
    std::cout << "\n[DJI]\n";
    if (g_var_data_stream.is_open())
        g_var_data_stream << "\n[DJI]\n";

    for (int i = 0; i < kChannelCount; ++i)
    {
        auto &m = robot.Chassis.Motor_DJI[i];
        const int alive = (m.Get_Status() == Motor_DJI_Status_ENABLE) ? 1 : 0;

        std::cout << "  M" << i
                  << " rx=0x"     << std::hex << m.DJI_CAN_Rx_ID << std::dec
                  << " rpm="      << m.Get_Now_RPM()
                  << " rotor_rad="<< m.Get_Now_Rad()
                  << " wheel_w="  << m.Get_Now_Wheel_Omega()
                  << " cur="      << m.Get_Now_Current()
                  << " cmd_cur="  << static_cast<int>(m.Get_Control_Current())
                  << " temp="     << m.Get_Now_Temperature()
                  << " alive="    << alive
                  << "\n";

        if (g_var_data_stream.is_open())
        {
            g_var_data_stream << "  M" << i
                              << " rx=0x"     << std::hex << m.DJI_CAN_Rx_ID << std::dec
                              << " rpm="      << m.Get_Now_RPM()
                              << " rotor_rad="<< m.Get_Now_Rad()
                              << " wheel_w="  << m.Get_Now_Wheel_Omega()
                              << " cur="      << m.Get_Now_Current()
                              << " cmd_cur="  << static_cast<int>(m.Get_Control_Current())
                              << " temp="     << m.Get_Now_Temperature()
                              << " \n";
        }
    }
}

/**
 * @brief 打印 OPS-9 状态（终端 + 文件）：连接状态 / 位姿 / 角速度 / 帧统计。
 *        停车排障时主要看 status 是否 ENABLE、rx_frames 是否在持续增长。
 */
static void Render_Dashboard_OPS()
{
    auto &ops = robot.Chassis.OPS;
    const char *status_str = (ops.Get_Status() == OPS_Status_ENABLE) ? "ENABLE " : "DISABLE";
    const auto data = ops.Get_Data();

    std::cout << "\n[OPS]\n"
              << "  status="   << status_str
              << " yaw_deg="   << data.yaw_deg
              << " pitch_deg=" << data.pitch_deg
              << " roll_deg="  << data.roll_deg
              << " pos_x_mm="  << data.pos_x_mm
              << " pos_y_mm="  << data.pos_y_mm
              << " yaw_rate_dps=" << data.yaw_rate_dps
              << " rx_frames=" << ops.Get_Rx_Frame_Count()
              << " resync="    << ops.Get_Resync_Count()
              << " tail_err="  << ops.Get_Tail_Mismatch_Count()
              << "\n";

    if (g_var_data_stream.is_open())
    {
        g_var_data_stream << "\n[OPS]\n"
                          << "  status="   << status_str
                          << " yaw_deg="   << data.yaw_deg
                          << " pitch_deg=" << data.pitch_deg
                          << " roll_deg="  << data.roll_deg
                          << " pos_x_mm="  << data.pos_x_mm
                          << " pos_y_mm="  << data.pos_y_mm
                          << " yaw_rate_dps=" << data.yaw_rate_dps
                          << " rx_frames=" << ops.Get_Rx_Frame_Count()
                          << " resync="    << ops.Get_Resync_Count()
                          << " tail_err="  << ops.Get_Tail_Mismatch_Count()
                          << " \n";
    }
}

/**
 * @brief 渲染整张 LIVE 仪表盘并刷出两路流。
 */
static void Print_Live_Dashboard()
{
    Ensure_Var_Data_Stream();
    Render_Dashboard_Header();
    Render_Dashboard_Remote();
    Render_Dashboard_DJI();
    Render_Dashboard_OPS();

    std::cout.flush();
    if (g_var_data_stream.is_open())
        g_var_data_stream.flush();
}

// ============================================================================
//                                初始化
// ============================================================================

/**
 * @brief 完成 EtherCAT 主站 + LinkX 适配器初始化，并把网络带入 OP 状态。
 *
 *  slave_id 绑定策略:
 *    优先按 Station Alias 绑（alias=1 -> FD, alias=2 -> classic）;
 *    若 alias 未生效，按当前实测物理拓扑回退:
 *        PC -> slave1(FD,挂夹爪) -> slave2(经典,挂底盘 DM/ODrive/Encoder/OPS)
 *        即 fd_slave_id=1, classic_slave_id=2
 *
 * @return true 成功；false 表示主站启动失败，调用方应直接返回。
 */
static bool Init_Ethercat_And_Linkx(const char *ifname)
{
    constexpr uint16_t kAliasFd      = 1;   ///< FD 那片(挂夹爪)
    constexpr uint16_t kAliasClassic = 2;   ///< 经典那片(挂底盘电机/编码器)

    if (!ecat_master_init(&master, ifname))
        return false;

    if (master.ctx.slavecount < 2)
    {
        std::cerr << "[TASK] FATAL: expected >=2 EtherCAT slaves (LinkX classic + LinkX FD), found "
                  << master.ctx.slavecount
                  << ". Check downstream LinkX cabling (old.OUT -> new.IN)." << std::endl;
        return false;
    }

    // 按 alias 绑 slave_id
    int classic_slave_id = 0;
    int fd_slave_id      = 0;
    for (int i = 1; i <= master.ctx.slavecount; ++i)
    {
        uint16_t alias = master.ctx.slavelist[i].aliasadr;
        if      (alias == kAliasFd)      fd_slave_id      = i;
        else if (alias == kAliasClassic) classic_slave_id = i;
    }

    if (fd_slave_id > 0 && classic_slave_id > 0)
    {
        std::cout << "[ECAT] alias-based binding: classic=slave " << classic_slave_id
                  << " (alias=" << kAliasClassic << "), fd=slave " << fd_slave_id
                  << " (alias=" << kAliasFd << ")" << std::endl;
    }
    else
    {
        std::cerr << "[ECAT][WARN] Station Alias not properly set "
                  << "(fd_slave_id=" << fd_slave_id
                  << " classic_slave_id=" << classic_slave_id
                  << "). Falling back to fixed topology: classic=2, fd=1." << std::endl;
        classic_slave_id = 2;
        fd_slave_id      = 1;
    }

    // 经典 CAN 1Mbps（DM/ODrive/Encoder/OPS 都跑这里）
    linkx_init(&linkx_dev, (uint32_t)classic_slave_id, &master.ctx);
    linkx_hw_wakeup(&linkx_dev);
    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_set_can_baudrate(&linkx_dev, ch, 0, 2, 31, 8, 8, 1, 31, 8, 8);

    // 第二片 LinkX:ch0 + ch1 经典 CAN 1M(ch0 挂夹爪 DM,ch1 透传 0x20 帧);ch2-3 保留 CAN-FD(仲裁 1M / 数据 5M)
    linkx_init(&linkx_dev_fd, (uint32_t)fd_slave_id, &master.ctx);
    linkx_hw_wakeup(&linkx_dev_fd);
    linkx_set_can_baudrate(&linkx_dev_fd, 0, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    linkx_set_can_baudrate(&linkx_dev_fd, 1, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    for (int ch = 2; ch < kChannelCount; ++ch)
        linkx_set_can_baudrate(&linkx_dev_fd, ch, 1, 2, 31, 8, 8, 1, 6, 1, 1);

    return ecat_master_bring_online(&master);
}

/**
 * @brief 启动 Robot 业务层（含 ROS2 桥接），不阻塞返回。
 */
static void Init_Robot_Logic()
{
    robot.Init(&linkx_dev, &linkx_dev_fd);
    robot.Start_ROS2_Remote_Bridge();
    std::cout << "[TASK] Robot Logic Initialized." << std::endl;
}

// ============================================================================
//                            主循环每帧子步骤
// ============================================================================

/**
 * @brief 抽干两片 LinkX 共 8 个 CAN 通道的接收队列，逐帧分发给 Robot。
 *        module_id=0 -> 经典 LinkX (slave_id=1)；module_id=1 -> CAN-FD LinkX (slave_id=2)
 */
static void Pump_CAN_Receive()
{
    can_msg_t msg;
    for (uint8_t ch = 0; ch < kChannelCount; ++ch)
        while (linkx_quick_recv(&linkx_dev, ch, &msg))
            robot.CAN_Rx_Callback(0, ch, msg.id, msg.data, msg.dlen);
    for (uint8_t ch = 0; ch < kChannelCount; ++ch)
        while (linkx_quick_recv(&linkx_dev_fd, ch, &msg))
            robot.CAN_Rx_Callback(1, ch, msg.id, msg.data, msg.dlen);
}

/**
 * @brief 按 1 ms / 2 ms / 100 ms 节拍触发 Robot 周期任务。
 */
static void Dispatch_Robot_Tick(uint32_t tick)
{
    robot.TIM_1ms_Calculate_Callback();
    if ((tick % kTaskPeriod2Ms)   == 0)
        robot.TIM_2ms_Calculate_PeriodElapsedCallback();
    if ((tick % kTaskPeriod100Ms) == 0)
        robot.TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 收到 Ctrl+C 等关停信号后,把所有电机置安全态再退出。
 *
 *  对象 (共 6 个执行器):
 *    - 4× DJI 3508 (linkx_dev ch0, Tx 0x200)         → 电流指令清零, 持续重发若干周期
 *    - 2× 夹爪 DM   (linkx_dev_fd ch0, ID 0x01/0x02) → CAN_Send_Exit
 *
 *  每个 tick 重新入队一次零电流/失能帧;linkx_send_pdos 每 ch 只 pop 1 帧,
 *  所以共跑 kDisableSendCycles ≈ 50ms, 确保多 ID 全部出栈到总线。
 *  期间继续 ecat_master_sync + linkx_recv_pdos, 保持 EtherCAT 链路活动。
 */
static void Disable_All_Devices()
{
    constexpr int kDisableSendCycles = 50;   // 50 × 1ms ≈ 50ms

    std::cout << "\n[TASK] Disabling all devices (Ctrl+C safety)..." << std::endl;

    // 提前把 4 个 DJI 电流缓存清零, 群发会拿这些 0 打包
    for (int i = 0; i < kChannelCount; ++i)
        robot.Chassis.Motor_DJI[i].Set_Control_Current(0);

    // 同时拉 chassis 到 DISABLE, 让 2ms 控制回调也停止往电流缓存里写非零值
    robot.Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

    Class_Motor_DJI_3508 *dji_group[4] = {
        &robot.Chassis.Motor_DJI[0], &robot.Chassis.Motor_DJI[1],
        &robot.Chassis.Motor_DJI[2], &robot.Chassis.Motor_DJI[3]
    };

    for (int cycle = 0; cycle < kDisableSendCycles; ++cycle)
    {
        // 保持 EtherCAT/PDO 活动, 顺带消化 RX 队列
        ecat_master_sync(&master);
        linkx_recv_pdos(&linkx_dev);
        linkx_recv_pdos(&linkx_dev_fd);
        Pump_CAN_Receive();

        // 4 个 DJI 3508 群发零电流帧 (Tx 0x200, ch0)
        Class_Motor_DJI_3508::Send_Group(&linkx_dev, /*can_channel=*/0,
                                         /*tx_can_id=*/0x200U, dji_group);

        // 2 个夹爪 DM 失能
        robot.Clamp.Motor_Pitch_Large.CAN_Send_Exit();
        robot.Clamp.Motor_Pitch_Small.CAN_Send_Exit();

        // 把入队的帧推到总线
        linkx_send_pdos(&linkx_dev);
        linkx_send_pdos(&linkx_dev_fd);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "[TASK] All devices disabled." << std::endl;
}

/**
 * @brief CAN 统计 / LIVE 仪表盘 / OPS 状态探针的周期触发。
 *        每个动作各自带开关与节拍, 互不影响。
 */
static void Dispatch_Periodic_Diagnostics(uint32_t tick)
{
    if (kEnableCanStatPrint && (tick % kCanStatPrintPeriodMs) == 0 && tick != 0)
    {
        std::cout << "\n[CAN-STATS] === LinkX classic (slave_id=1) ===";
        Print_CAN_Stats(&linkx_dev);
        std::cout << "\n[CAN-STATS] === LinkX FD      (slave_id=2) ===";
        Print_CAN_Stats(&linkx_dev_fd);
    }

    if (kEnableLiveDashboard && (tick % kLiveDashboardPeriodMs) == 0)
        Print_Live_Dashboard();

    // [OPS-PROBE] 1Hz 状态探针:dashboard 关闭时也能看到 OPS 是否在收帧
    if ((tick % 1000U) == 0U)
    {
        auto &ops = robot.Chassis.OPS;
        std::cout << "[OPS] status="
                  << (ops.Get_Status() == OPS_Status_ENABLE ? "ENABLE " : "DISABLE")
                  << " rx_frames=" << ops.Get_Rx_Frame_Count()
                  << " resync="    << ops.Get_Resync_Count()
                  << " yaw="       << ops.Get_Yaw_Deg()
                  << " x="         << ops.Get_Pos_X_Mm()
                  << " y="         << ops.Get_Pos_Y_Mm()
                  << std::endl;
    }
}

// ============================================================================
//                                  主入口
// ============================================================================

/**
 * @brief 机器人主控制阻塞循环 (全向轮工况):
 *           初始化 → 1 kHz 主循环 (CAN 收 → Robot 节拍 → CAN 发 → 诊断)
 *           → 收到关闭信号后把所有电机置安全态 → 关闭 ROS 桥。
 *
 * @param ifname EtherCAT 网卡名 (如 "enp86s0")。
 */
void Robot_Control_Loop(const char *ifname)
{
    if (!Init_Ethercat_And_Linkx(ifname))
        return;
    Init_Robot_Logic();

    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;

    while (master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kControlLoopPeriodMs);

        // ---- 进站: EtherCAT 同步 + CAN 接收分发 ----
        ecat_master_sync(&master);
        linkx_recv_pdos(&linkx_dev);
        linkx_recv_pdos(&linkx_dev_fd);
        Pump_CAN_Receive();

        // ---- 计算: Robot 周期任务 ----
        Dispatch_Robot_Tick(tick);

        // ---- 出站: 提交 CAN 发送 ----
        linkx_send_pdos(&linkx_dev);
        linkx_send_pdos(&linkx_dev_fd);

        // ---- 旁路: 诊断 ----
        Dispatch_Periodic_Diagnostics(tick);

        ++tick;
        std::this_thread::sleep_until(next_wakeup);
    }

    // 退出钩子:先把所有电机置安全态, 再关 ROS 桥
    Disable_All_Devices();
    robot.Stop_ROS2_Remote_Bridge();
}
