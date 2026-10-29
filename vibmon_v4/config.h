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
#define ACCEL_FS_SEL     1       // 0=±2g 1=±4g 2=±8g 3=±16g
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

/* ---------- 接触判据模式 ----------
 * 0 = 仅姿态变化（相对**标定姿态**）   —— 实测会全程误报，保留做对比
 * 1 = 姿态变化 或 振动中              —— **已证明会误报**，仅保留做对比测试
 * 2 = 到**接触姿态**的距离            —— v3 判据，保留做对比
 * 3 = **平放基准 + 姿态·幅值·频率三重与**  —— **v4 默认**
 *
 * 【v3 的模式 0/1/2 为什么不够 —— 空夹场景】
 *   v3 的模式 2 只判"姿态离接触姿态多远"，它无法区分：
 *
 *   ① 平放未握      姿态≈参考点      -> 判未接触   正确
 *   ② 完全握住、**没夹到物体**（空夹）
 *                    姿态大幅偏离    -> 判**接触** 误报！
 *   ③ 握住有物体     姿态大幅偏离    -> 判接触     正确
 *
 *   ② 的振动幅值可能比 ③ 还大（空夹时机械手自身刚性低、自由度多），
 *   所以"幅值大"既不能证明接触，也区分不了 ②/③。
 *   唯一能区分 ② 和 ③ 的是**振动频率**：
 *     ② 空夹没有物体共振，主频落在机械手自身的低频带
 *     ③ 有物体时主频被物体固有频率拉高
 *
 * 【v4 模式 3 的判据 - 三条同时成立才算接触】
 *   (1) 姿态：到**平放**姿态距离 > D_ON      —— 手确实握下去了
 *   (2) 幅值：ampValid && amp > A_ON          —— 确实在振（排除静止空夹）
 *   (3) 频率：freqValid && f   > F_ON         —— 频率高于空夹本底
 *   退出用各自 OFF 阈值；接触保持需要(1)成立且[(2)或(3)]仍成立。
 *
 * 【为什么改成"平放"为基准】
 *   v3 需要选手在**握住时**按 't' 记录接触姿态，赛前手按一次；
 *   v4 改用**平放姿态**做零点，设备平放静置即可采集，无需现场操作。
 */
#ifndef CT_MODE
#define CT_MODE          3
#endif
#define CT_K             4.0f    // 模式 0/1 用：接触阈值 = K × 静置姿态偏差
#define CT_MIN_MG        30.0f   // 模式 0/1 用：接触阈值下限
#define CT_OFF_RATIO     0.6f    // 模式 0/1 用：释放阈值 = 比例 × 接触阈值（滞回）
#define CT_CONFIRM_MS    100     // 接触状态翻转确认时间（四种模式共用）

/* ---------- 模式 2：到接触姿态的距离判据（v3 原判据，保留对比）----------
 * 【数值来源】c4.csv 5 轮握/松 + c5.csv 3 分钟静置，见 07-Q4 文档 §5。
 *   握住：到接触姿态 6~1309 mg（稳态多数 <50）
 *   松开：到接触姿态 535~1750 mg（多数 >1300）
 * 取进入 400 / 退出 700：
 *   - 400 覆盖握住稳态（<50）且远低于松开最小值（535）
 *   - 700 提供滞回带，避免在边界抖动
 *   - 仍有 200mg 余量到松开最小值，且静置漂移方向远离阈值，安全
 */
#define CT_TOUCH_MG      400.0f  // 进入接触：到接触姿态距离 < 此值
#define CT_TOUCH_OFF_MG  700.0f  // 退出接触：到接触姿态距离 > 此值（滞回）

/* ================= 模式 3：平放基准 + 三重与（v4 默认）=================
 *
 * 判据 = 姿态 且 幅值 且 频率，三条同时成立才置 contact=1。
 * 每条的 on/off 各自成对，形成滞回；off 一律比 on 更宽松，避免边界抖动。
 */

/* ---- (1) 姿态：到平放姿态的距离 ----
 * 【语义】f.ctAxis[] 是"相对标定零偏后的三轴低通值"，平放静置时 ≈ (0,0,0)。
 *   所以"到平放姿态的距离" = ||ctAxis - flatRef||，flatRef 是平放时采到的
 *   那组 ctAxis 值（标定已吃掉零偏，平放时通常接近 0，但仍有残留，故显式记录）。
 *
 * 【TODO 待实测】占位值沿用 v3 的 400/700，**不可信**，仅保证能编译运行。
 *   采样后按下面规则替换：
 *     D_ON  = 空夹与平放的下沿之间取中值（要能稳定抓住"手已握下"）
 *     D_OFF = D_ON × 0.7 左右（滞回）
 */
#define CT_FLAT_D_ON_MG   400.0f  /* TODO(实测) 姿态进入：到平放距离 > 此值 */
#define CT_FLAT_D_OFF_MG  280.0f  /* TODO(实测) 姿态退出：到平放距离 < 此值 */

/* ---- (2) 幅值：合成 RMS 包络 ----
 * 用 f.env（合成 RMS 包络 mg），配合 r.ampValid。
 * 目的：平放静置时幅值极低，用它快速排除"没在振"。
 *
 * 【TODO 待实测】平放噪声底 ×(2~3) 起；空夹幅值很大，本阈值**不是**用来
 *   区分空夹与真接触的（那由频率负责），只用来排除静止状态。
 */
#define CT_FLAT_A_ON_MG   30.0f   /* TODO(实测) 幅值进入：包络 > 此值 */
#define CT_FLAT_A_OFF_MG  20.0f   /* TODO(实测) 幅值退出：包络 < 此值 */

/* ---- (3) 频率：主频 ----
 * 用 r.freqHz，配合 r.freqValid。
 * **这是区分"空夹"与"真握住有物体"的唯一依据。**
 *   空夹：没有物体共振 -> 主频偏低，落在机械手自身摆动/驱动频带
 *   有物：物体固有频率被激起 -> 主频高于空夹本底
 *
 * 【TODO 待实测 -- v4 成败的关键】
 *   若实测发现物体固有频率**落在空夹频带内**，这条判据就分不开，
 *   本方案需要重新设计（例如改用幅值谱形状或双频特征）。
 *   采样时务必分别记录空夹与握有物体的 freqHz 分布。
 */
#define CT_FLAT_F_ON_HZ   120.0f  /* TODO(实测) 频率进入：主频 > 此值 */
#define CT_FLAT_F_OFF_HZ  90.0f   /* TODO(实测) 频率退出：主频 < 此值 */

/* ---- 判据降级策略 ----
 * 三重与的代价：任何一路数据失效（频率没估出来、幅值被削顶）都会让
 * contact 掉到 0，比赛中可能表现为"接触突然丢失"。
 * 定义"数据失效时怎么办"：
 *   1 = 保守：失效的那一路视为不满足 -> contact=0（不会误报，但可能漏报）
 *   0 = 宽松：失效的那一路视为满足 -> 只看其余两路
 *
 * 【2026-10-29 实板实测：默认改成 1】
 *   选 0 时，闭合动作的瞬态把幅值/频率门点着后，只要数据失效就恒被视为满足，
 *   保持规则 (幅值||频率) 永远成立 -> contact 永久锁在 1。
 *   实测证据：握紧无物 + 振源关，ct 连续 29 条全为 1（真值 0）；
 *   握紧无物 + 振源开（空夹接不到振动）同样 29/29 误报。
 *   改成 1 后两格均为 0，接触 + 振动仍稳定为 1。
 *   代价：目标物停振时 contact 会掉到 0 —— 这是"无振动即无证据"的物理上限，
 *   不是本宏引入的缺陷；若要支持"接触但物体不振"，得改成两轴独立输出。
 */
#ifndef CT_FLAT_DEGRADE_STRICT
#define CT_FLAT_DEGRADE_STRICT  1
#endif

/* ---- 空夹（握住但没物体）独立上报 ----
 * 即使判为"未接触"，只要姿态已握下（(1) 成立）却因频率不够被拦，
 * 就置 result.gripNoObject=1，让现场知道"手合上了但没夹到东西"。
 * 这个状态**不参与** contact，只是诊断/提示用。
 */
#define CT_FLAT_REPORT_NO_OBJ   1

/* ---------- 参考点采集：延时 + 平均 + 稳定性检查 ----------
 * 【为什么不能记瞬时值】
 *   实测（t_verify.csv，2026-10-18）：按 't' 的瞬间手往往还没握到位，
 *   记下的参考点偏离稳态 1593mg，而稳态只有 21mg。
 *   虽然 21mg 仍远小于阈值 400 所以判定侥幸正确，但余量被吃掉一大半，
 *   现场手抖时可能直接判错。
 *
 * 【两阶段】v3 的方案
 *   阶段1 等待 CT_REF_SETTLE_MS：让手完成动作、姿态稳定下来
 *   阶段2 平均 CT_REF_AVG_MS 内的样本：抵消手抖与传感器噪声
 * 平均而不是取最后一点，是因为最后一点同样可能落在抖动峰上。
 * 总耗时 = 300 + 500 = 800ms，现场按一下等不到 1 秒，可接受。
 *
 * 【v4 增加：稳定性检查】
 *   v3 由人手按 't' 触发，人自己知道"我刚放好"。
 *   v4 改为**上电自动采平放基准**（设备平放静置即可，免现场按键），
 *   于是出现新风险：**采集期间有人搬动设备**，采到的将是错误零点，
 *   且此后所有姿态判定都系统性偏移 —— 这是最危险的一类错误，
 *   因为它不会报错，只会让判定悄悄变错。所以必须能识别并拒绝：
 *   统计窗口内每轴的极差(max-min)，超过 CT_REF_MAX_SWING_MG 判定"在动"，
 *   作废并重试；重试 CT_REF_AUTO_RETRY 次仍失败，则接触判据整体置为
 *   不可信（contactValid=0）—— 宁可说"不知道"，也不给出错值。
 *
 * 采集期间界面会提示进度；期间若发生重标定/复位则放弃。
 */
#define CT_REF_SETTLE_MS      300     // 记录前的稳定等待
#define CT_REF_AVG_MS         500     // 平均窗口长度
#define CT_REF_MAX_SWING_MG   60.0f   /* TODO(实测) 采集窗口内单轴极差上限，超过即判"在动" */
#define CT_REF_AUTO_RETRY     3       // 自动采集失败后的重试次数
#define CT_REF_AUTO_ENABLE    1       // 1 = 上电自动采平放基准（免按键）；0 = 等同 v3，需按 't'

/* ---- 模式 3 的编译期检查 ----
 * 三重与判据最容易犯的错：on/off 写反（off 更严 → 滞回失效 → 边界抖动），
 * 或频率窗口与测频频段不重叠（永远不可能满足 → 永不接触）。
 * 预处理器算不了浮点，用整数镜像宏做检查，浮点值改了这里也要同步。
 */
#ifndef CT_FLAT_D_ON_MG_INT
#define CT_FLAT_D_ON_MG_INT     400
#endif
#ifndef CT_FLAT_D_OFF_MG_INT
#define CT_FLAT_D_OFF_MG_INT    280
#endif
#ifndef CT_FLAT_A_ON_MG_INT
#define CT_FLAT_A_ON_MG_INT     30
#endif
#ifndef CT_FLAT_A_OFF_MG_INT
#define CT_FLAT_A_OFF_MG_INT    20
#endif
#ifndef CT_FLAT_F_ON_HZ_INT
#define CT_FLAT_F_ON_HZ_INT     120
#endif
#ifndef CT_FLAT_F_OFF_HZ_INT
#define CT_FLAT_F_OFF_HZ_INT    90
#endif

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

/* ---------- 模式 3（平放三重与）参数检查 ----------
 * 只在 CT_MODE==3 时检查，其它模式下去掉这些宏也不该报错。
 */
#if (CT_MODE == 3)
  #if (CT_FLAT_D_OFF_MG_INT >= CT_FLAT_D_ON_MG_INT)
    #error "CT_FLAT_D_OFF 必须小于 CT_FLAT_D_ON（否则滞回带反了，接触会在阈值处抖动）"
  #endif
  #if (CT_FLAT_A_OFF_MG_INT >= CT_FLAT_A_ON_MG_INT)
    #error "CT_FLAT_A_OFF 必须小于 CT_FLAT_A_ON（同上）"
  #endif
  #if (CT_FLAT_F_OFF_HZ_INT >= CT_FLAT_F_ON_HZ_INT)
    #error "CT_FLAT_F_OFF 必须小于 CT_FLAT_F_ON（同上）"
  #endif
  #if (CT_FLAT_F_ON_HZ_INT > FREQ_HI_HZ_INT)
    #error "CT_FLAT_F_ON 高于测频上限 FREQ_HI_HZ，接触永远不可能成立（主频最多到 FREQ_HI_HZ）"
  #endif
  #if (CT_FLAT_F_OFF_HZ_INT < FREQ_LO_HZ_INT)
    #error "CT_FLAT_F_OFF 低于测频下限 FREQ_LO_HZ，空夹时主频被下限钉住，退出条件永远不满足"
  #endif
  #if ((CT_FLAT_D_ON_MG_INT <= 0) || (CT_FLAT_A_ON_MG_INT <= 0) || (CT_FLAT_F_ON_HZ_INT <= 0))
    #error "模式 3 的进入阈值必须为正"
  #endif
#endif
#if (CT_REF_AVG_MS <= 0)
  #error "CT_REF_AVG_MS 必须大于 0（没有样本就无法求平均）"
#endif
#if (CT_REF_AUTO_RETRY < 0)
  #error "CT_REF_AUTO_RETRY 不能为负"
#endif
