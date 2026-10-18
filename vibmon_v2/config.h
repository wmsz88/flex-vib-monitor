// 配置模块：引脚、量程、采样率、滤波参数、阈值、串口参数。所有可调参数只在这里改。
#pragma once
#include <stdint.h>

/* ---------- 引脚（依据 Netlist_PCB1_2026-09-24.tel） ---------- */
#define PIN_SDA        8
#define PIN_SCL        9
#define PIN_LED_G      10        // 绿：心跳
#define PIN_LED_Y      5         // 黄：振动中 / 故障快闪
#define LED_ACTIVE_LOW 0         // 1 = 灌电流接法（GPIO 低电平点亮）

/* ---------- 调试口（Serial） ---------- */
#define UART_SWAP      0         // 1 = 交换 TX/RX
#if UART_SWAP
  #define PIN_DEBUG_TX  20
  #define PIN_DEBUG_RX  21
#else
  #define PIN_DEBUG_TX  21
  #define PIN_DEBUG_RX  20
#endif
#define BAUD           115200

/* ---------- 比赛口（Serial1 / UART1） ----------
 * 引脚可自由映射：任何可用 GPIO 都行，改下面两个宏即可，无需改 .cpp。
 * 默认与调试口同脚：CDC 模式下 Serial 走 USB，不占 GPIO20/21，
 * 因此比赛口可以独占。非 CDC 模式会自动让位（见 output.cpp outBegin）。
 */
#ifndef PIN_COMP_TX
#define PIN_COMP_TX    21
#endif
#ifndef PIN_COMP_RX
#define PIN_COMP_RX    20
#endif
#ifndef COMP_BAUD
#define COMP_BAUD      115200
#endif
/* 比赛帧开关：协议/口径确定前保持 0，比赛口完全不发送任何字节 */
#ifndef COMP_FRAME_ENABLED
#define COMP_FRAME_ENABLED 0
#endif
#ifndef COMP_PERIOD_MS
#define COMP_PERIOD_MS 20        // 50Hz
#endif

/* ---------- MPU6050 ---------- */
#define MPU_ADDR         0x68    // AD0 接地
#define I2C_CLOCK        200000UL
#define I2C_TIMEOUT_MS   5       // 采样回调里不能长时间阻塞
#ifndef ACCEL_FS_SEL
#define ACCEL_FS_SEL     0       // 0=±2g 1=±4g 2=±8g 3=±16g
#endif
#ifndef DLPF_CFG
#define DLPF_CFG         0       // 0 = 加速度带宽 260Hz（保住 180Hz）
#endif
#define MPU_CLKSEL       0       // 0 = 内部 8MHz；1 = PLL(陀螺X)，频率更稳
#ifndef ACCEL_CLIP_RAW
#define ACCEL_CLIP_RAW   32700   // |raw| 超过即视为削顶（±2g 下 ≈1996mg）
#endif

/* ---------- 采样 ---------- */
#ifndef FS_HZ
#define FS_HZ            1000
#endif
#ifndef RING_N
#define RING_N           512     // 环形缓冲深度（2 的幂）
#endif
#define SMP_JITTER_TOL_US 500    // 相邻样本间隔偏离标称超过此值视为时间不连续
// 缺口超过此时长：启停周期历史作废。用时间而非样本数，改 FS_HZ 时物理含义不变。
#ifndef SMP_BIG_GAP_MS
#define SMP_BIG_GAP_MS   20
#endif
#define SENSOR_LOST_N    20      // 连续读失败次数 → 判定传感器断线
#define SENSOR_STALE_MS  100     // 这么久没有成功样本 → 判定断线
#define RECOVER_PERIOD_MS 1000   // 故障后重连间隔

// 停止握手超时：等正在执行的采样回调退出。
// 依据：单次 I2C 事务上限 5ms（i2cWriteReadNonStop）+ 调度抖动 + 短时 Wire 锁竞争。
// 注意 Wire.requestFrom 内部用 portMAX_DELAY 取锁，锁等待无上界，
// 所以这个值不是"最坏耗时"的推导，而是工程折中：真的锁死时靠它进故障。
#ifndef SMP_STOP_TIMEOUT_MS
#define SMP_STOP_TIMEOUT_MS 15
#endif

/* ---------- 信号处理 ---------- */
#ifndef BLOCK_N
#define BLOCK_N          128     // 主频估计窗长（2 的幂）
#endif
#ifndef HOP
#define HOP              64      // 窗步长 → 主频更新 15.6Hz
#endif
#define HP_FC_HZ         70.0f   // 二阶高通截止
#define HP_Q             0.7071f
#ifndef FREQ_LO_HZ
#define FREQ_LO_HZ       60.0f   // 有效周期对应频率范围
#endif
#ifndef FREQ_HI_HZ
#define FREQ_HI_HZ       220.0f
#endif
#define MIN_PERIODS      5       // 窗内有效周期数下限
#define ZCL_K            3.6f    // 过零阈值 = K × 带内噪声 sigma
#define ZCL_MIN_MG       1.0f
#ifndef ENV_WIN
#define ENV_WIN          30      // 包络 RMS 窗（样本）
#endif
#ifndef ENV_HOP
#define ENV_HOP          10      // 包络输出间隔 → 100Hz
#endif
#define CT_LPF_TAU_S     0.2f    // 姿态（低频）支路每轴低通时间常数

/* ---------- 标定 ---------- */
#define CAL_BIAS_MS      2000
#define CAL_NOISE_MS     1200
#define CAL_MIN_SAMPLES  200     // 阶段A 最少样本数，不足即失败
#define CAL_MAX_STD_MG   20.0f   // 阶段A 原始数据标准差上限：超过说明标定时被碰过
#define CAL_G_TOL_MG     150.0f  // 零偏模长与 1g 的允许偏差
#define CAL_MAX_ENV_MG   50.0f   // 阶段B 静置包络上限：超过说明标定时有振动
#define CAL_MAX_CT_MG    50.0f   // 阶段B 静置姿态偏差上限：超过说明标定时被转动
#define CAL_RETRY_MS     5000    // 无有效标定时自动重试间隔

/* ---------- 状态判断 ---------- */
#define TH_HI_K          5.0f    // 启停高阈值 = K × 静置包络
#define TH_LO_K          2.5f
#define TH_HI_MIN_MG     8.0f
#define TH_LO_MIN_MG     4.0f
#ifndef SS_CONFIRM
#define SS_CONFIRM       3       // 状态翻转需连续 3 个包络点（30ms）
#endif
#ifndef SS_EDGE_MAX
#define SS_EDGE_MAX      16      // 记录的"起"边沿数
#endif
#ifndef SS_T_MIN_MS
#define SS_T_MIN_MS      200     // 启停周期下限（5Hz）：更短视为抖动
#endif
// 启停周期上限：0.1Hz 的周期是 10000ms，留 20% 余量避免边沿抖动越界。
// 更低频率（<0.083Hz）需要延长观察，不能笼统宣称覆盖任意非零频率。
#ifndef SS_T_MAX_MS
#define SS_T_MAX_MS      12000   // 启停周期上限（≈0.083Hz）
#endif
#ifndef SS_MIN_PERIODS
#define SS_MIN_PERIODS   2       // 至少 2 个完整周期才给启停频率
#endif
#ifndef SS_USE_PERIODS
#define SS_USE_PERIODS   4       // 取最近 N 个周期的中位数
#endif
#define SS_TIMEOUT_K     2.0f    // 超过 K × 当前周期没有新边沿 → 启停频率失效
#define SS_CONSIST_RATIO 1.5f    // 参与统计的周期 最大/最小 超过此比值 → 节奏不稳，不给读数
#define FREQ_STALE_MS    300     // 主频结果超过此时间未更新 → 失效
#define AMP_STALE_MS     100     // 幅值超过此时间未更新 → 不可信（包络 100Hz）

/* ---------- 削顶质量 ----------
 * 幅值：任意轴任意削顶 → ampValid=false（合成 RMS 必偏小，不看比例）
 * 主频：只看主轴；比例 >= 此值 → 主频失效
 */
#ifndef CLIP_SEVERE_RATIO
#define CLIP_SEVERE_RATIO 0.10f  // 主轴削顶 ≥10% → 主频失效
#endif

#define CT_MODE          1       // 0 = 仅姿态变化；1 = 姿态变化 或 振动中
#define CT_K             4.0f    // 接触阈值 = K × 静置姿态偏差
#define CT_MIN_MG        30.0f
#define CT_OFF_RATIO     0.6f    // 释放阈值 = 比例 × 接触阈值（滞回）
#define CT_CONFIRM_MS    100     // 接触状态翻转确认时间

/* ---------- 输出 ---------- */
#define PLOT_MS          20      // 绘图行 50Hz
#define DBG_MS           200     // 调试文字行 5Hz
#define PLOT_DEFAULT     1       // 上电默认绘图组（见 output.h），0 = 不绘图
// 三轴波形抽取比：运行时可用 'w' 命令调整。UART 115200 下 1:1 必然阻塞，
// 默认 4（250Hz）在 CDC 下安全；UART 下建议 w8 或更大。
#ifndef PLOT_WAVE_DEFAULT_DECIM
#define PLOT_WAVE_DEFAULT_DECIM 4
#endif

// 采样周期（微秒）：所有"耗时是否过大"的判据都用它做基准，改 FS_HZ 时自动跟着变
#define FS_PERIOD_US     (1000000UL / FS_HZ)

/* ============================================================
 * 参数边界检查（编译期）
 * 目的：改错一个宏时立即报错，而不是烧进去之后才发现算法失效。
 * 每条都写明"为什么"和"怎么改"，现场不用查代码。
 * ============================================================ */

/* ---------- 采样与窗口 ---------- */
#if (FS_HZ <= 0)
  #error "FS_HZ 必须大于 0（采样周期 FS_PERIOD_US 会出现除零）"
#endif
#if (FS_HZ > 1000)
  #error "MPU6050 加速度输出率固定 1kHz，FS_HZ 不能超过 1000"
#endif
#if (BLOCK_N & (BLOCK_N - 1)) || (RING_N & (RING_N - 1))
  #error "BLOCK_N 与 RING_N 必须是 2 的幂（代码用 &(N-1) 做环形索引）"
#endif
#if (HOP <= 0) || (HOP > BLOCK_N)
  #error "HOP 必须在 (0, BLOCK_N] 内（步长大于窗口时窗口永远填不满）"
#endif
#if (ENV_WIN <= 0) || (ENV_WIN > RING_N)
  #error "ENV_WIN 必须在 (0, RING_N] 内（包络窗不能大于缓冲）"
#endif
#if (ENV_HOP <= 0) || (ENV_HOP > ENV_WIN)
  #error "ENV_HOP 必须在 (0, ENV_WIN] 内（输出间隔不能大于窗口）"
#endif
#if (RING_N < BLOCK_N)
  #error "RING_N 不能小于 BLOCK_N（主频窗需要连续 BLOCK_N 个样本）"
#endif

/* ---------- 频段与奈奎斯特 ----------
 * 预处理器不能算浮点，所以检查用整数镜像宏（FREQ_*_HZ_INT），
 * 它们必须与上面的浮点宏保持一致——若改了浮点值，这两个也要同步改。
 */
#ifndef FREQ_LO_HZ_INT
#define FREQ_LO_HZ_INT   60
#endif
#ifndef FREQ_HI_HZ_INT
#define FREQ_HI_HZ_INT   220
#endif

#if (FREQ_LO_HZ_INT <= 0)
  #error "FREQ_LO_HZ 必须大于 0"
#endif
#if (FREQ_HI_HZ_INT <= FREQ_LO_HZ_INT)
  #error "FREQ_HI_HZ 必须大于 FREQ_LO_HZ"
#endif
#if ((FREQ_HI_HZ_INT * 2) >= FS_HZ)
  #error "FREQ_HI_HZ 必须低于奈奎斯特频率 FS_HZ/2（否则混叠，测出假频率）"
#endif
#if ((FS_HZ / BLOCK_N) >= FREQ_LO_HZ_INT)
  #error "BLOCK_N 太短：窗口内放不下一个 FREQ_LO_HZ 周期，主频永远算不出"
#endif

/* ---------- 启停频率 ---------- */
#if (SS_MIN_PERIODS < 1)
  #error "SS_MIN_PERIODS 至少为 1"
#endif
#if (SS_USE_PERIODS < SS_MIN_PERIODS)
  #error "SS_USE_PERIODS 不能小于 SS_MIN_PERIODS（取样数少于下限等于永不生效）"
#endif
#if (SS_EDGE_MAX < SS_USE_PERIODS)
  #error "SS_EDGE_MAX 不能小于 SS_USE_PERIODS（历史存不下要取的样本数）"
#endif
#if (SS_T_MIN_MS <= 0) || (SS_T_MAX_MS <= SS_T_MIN_MS)
  #error "SS_T_MIN_MS / SS_T_MAX_MS 区间无效"
#endif
#if ((SS_T_MIN_MS) < (ENV_HOP * 1000 / FS_HZ))
  #error "SS_T_MIN_MS 小于一个包络间隔，边沿时间分辨不出来"
#endif

/* ---------- MPU 配置 ---------- */
#if (((ACCEL_FS_SEL) & ~0x03) != 0)
  #error "ACCEL_FS_SEL 只能是 0..3（0=±2g 1=±4g 2=±8g 3=±16g）"
#endif
#if (((DLPF_CFG) & ~0x07) != 0)
  #error "DLPF_CFG 只能是 0..7"
#endif
#if (((MPU_CLKSEL) & ~0x07) != 0)
  #error "MPU_CLKSEL 只能是 0..7"
#endif
#if ((ACCEL_CLIP_RAW <= 0) || (ACCEL_CLIP_RAW > 32767))
  #error "ACCEL_CLIP_RAW 必须在 (0, 32767] 内（int16 量程）"
#endif

/* DLPF 带宽必须覆盖 FREQ_HI_HZ，否则高通后没有信号
 * 带宽对应（MPU6050 数据手册）：0=260Hz 1=184Hz 2=94Hz 3=44Hz 4=21Hz 5=10Hz 6=5Hz */
#if (DLPF_CFG == 1) && (FREQ_HI_HZ_INT > 184)
  #error "DLPF_CFG=1 的加速度带宽仅 184Hz，FREQ_HI_HZ 不能超过 184（保持 DLPF_CFG=0 才有 260Hz）"
#endif
#if (DLPF_CFG == 2) && (FREQ_HI_HZ_INT > 94)
  #error "DLPF_CFG=2 的加速度带宽仅 94Hz"
#endif
#if (DLPF_CFG >= 3)
  #error "DLPF_CFG>=3 的带宽（≤44Hz）低于测频下限，不能用于本项目"
#endif

/* ---------- 索引容量 ---------- */
#if (BLOCK_N > 32767)
  #error "BLOCK_N 超过 int16 索引容量"
#endif
#if (RING_N > 65535)
  #error "RING_N 超过 uint16_t 索引容量"
#endif

/* ---------- 其它 ---------- */
#if (CT_CONFIRM_MS < 0)
  #error "CT_CONFIRM_MS 不能为负"
#endif
#if (CAL_RETRY_MS <= 0)
  #error "CAL_RETRY_MS 必须大于 0"
#endif
#if (COMP_PERIOD_MS <= 0)
  #error "COMP_PERIOD_MS 必须大于 0"
#endif
