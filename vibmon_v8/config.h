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
#define UART_SWAP      1        // 1 = 交换 TX/RX
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
/* 比赛帧是否带 CL（closed，手指闭合）字段：默认 0 = 关闭。
 * 关闭时帧与历史版本**逐字节一致**（35 字节），现场平台允许改帧格式后
 * 只需把这里改成 1 重新烧录，不需要动 output.cpp：
 *   关闭：$VD,SEQ,FREQx10,AMP_MG,VIB,CT,SSx100,VALID,CHK           (35 字节)
 *   开启：$VD,SEQ,FREQx10,AMP_MG,VIB,CT,CL,SSx100,VALID,CHK        (37 字节)
 * 为什么单独一个 CL 位：CT=0 时平台看不出是"手没合"还是"合了但没振动证据"，
 * closed=1 且 CT=0 即空握/无振动证据；closed=0 即尚未握下。 */
#ifndef COMP_FRAME_CL
#define COMP_FRAME_CL 0
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

/* ---------- 陀螺仪（v5 新增） ----------
 * 用途：积分得到"闭合角度"，替代 v4 里分辨力不足的 flatDist(D)。
 * 加速度仍负责"认零位"（D 小 = 已回到张开），陀螺负责"量转了多少度"。
 */
#ifndef GYRO_FS_SEL
#define GYRO_FS_SEL      1       // 0=±250 1=±500 2=±1000 3=±2000 dps
#endif
/* 每 N 个采样 tick 读一次陀螺。14 字节 ≈ 980us @200kHz，1kHz 周期装不下，
 * 取 4 ⇒ 陀螺 250Hz，普通 tick 仍只读 6 字节加速度。 */
#ifndef GYRO_DIV
#define GYRO_DIV         4
#endif
#ifndef GYRO_DEADBAND_DPS
#define GYRO_DEADBAND_DPS   1.5f   // 小于此角速度视为静止，不积分
#endif
#ifndef GYRO_STILL_MS
#define GYRO_STILL_MS       500    // 连续静止超过此时间才认定"停住了"（回零/冻结都用它）
#endif
#ifndef GYRO_CAL_MAX_STD_DPS
#define GYRO_CAL_MAX_STD_DPS 3.0f  // 标定阶段陀螺标准差上限：超过说明在动
#endif
/* 量程与削顶线：±250/500/1000/2000 对应 FS_SEL 0/1/2/3 */
#define GYRO_RANGE_DPS      (250.0f * (float)(1 << (GYRO_FS_SEL & 0x03)))
#define GYRO_CLIP_DPS       (0.9f * GYRO_RANGE_DPS)
/* 陀螺积分步长：每 GYRO_DIV 个采样 tick 读一次，故 Δt = DIV/FS_HZ */
#define GYRO_DT_S           ((float)GYRO_DIV / (float)FS_HZ)
/* 陀螺数据连续中断超过此时间 ⇒ 角度不可信 */
#ifndef GYRO_STALE_MS
#define GYRO_STALE_MS       1000
#endif

/* ---------- 闭合角度判据（CT_MODE=4） ----------
 * 【2026-10-29 实板结论】角度只用来分"松手 / 已闭合"两档：
 *   夹空气 111°/106°、夹 4cm 方块 100° —— 只差 6~11°，
 *   靠角度分"空夹 vs 夹物"不成立（指甲朝向由近端关节主导，物体挡不住它）。
 *   所以角度只做"是否闭合"的单门限，空夹/夹物交给振动轴（A/F，实测 6.4 vs 755mg）。
 * 数值来自实测：松手 0°，闭合 100~111°，取 60° 作门限余量最大。 */
#ifndef ANGLE_CLOSED_DEG
#define ANGLE_CLOSED_DEG     60.0f  /* 高于此角 = 已闭合 */
#endif
#ifndef ANGLE_CLOSED_OFF_DEG
#define ANGLE_CLOSED_OFF_DEG 45.0f  /* 滞回退出 */
#endif
#ifndef ANGLE_OPEN_DEG
#define ANGLE_OPEN_DEG      20.0f   /* 低于此角且静止 = 回到张开位，角度清零 */
#endif
#ifndef OPEN_D_MG
#define OPEN_D_MG           400.0f /* D 低于此值（且静止满 GYRO_STILL_MS）判"已回到张开"。
                                    * H1 实测静止张开态 D=3~212mg，闭合态 >1000mg；
                                    * 取 400：对张开侧余量近 2 倍，对闭合侧仍有 2.5 倍。
                                    * 机械限位做不到时，放宽这一项能让"每次张开位置不同"
                                    * 也能被判成回到张开位，配合下面的自动重采基准。 */
#endif

/* ---------- 自动重采基准（无机械限位时的兜底，2026-10-29） ----------
 * 触发条件全部满足才重采：
 *   已静止 >= AUTO_ZERO_REST_MS（陀螺在死区、由 GYRO_STILL_MS 体系维护）
 *   且 幅值 < AUTO_ZERO_AMP_MG（没有振动，排除"夹着东西静止"）
 *   且 当前闭合角 < AUTO_ZERO_ANGLE_DEG（确实回到张开位，排除"夹着东西停顿"）
 * 重采后 D 归零、θ 归零，等于自动替操作员按了一次 't'。 */
#ifndef AUTO_ZERO_ENABLE
#define AUTO_ZERO_ENABLE      1
#endif
#ifndef AUTO_ZERO_REST_MS
#define AUTO_ZERO_REST_MS     2000
#endif
#ifndef AUTO_ZERO_AMP_MG
#define AUTO_ZERO_AMP_MG      100.0f
#endif
#ifndef AUTO_ZERO_ANGLE_DEG
#define AUTO_ZERO_ANGLE_DEG   8.0f
#endif

/* ---------- 姿态诊断（START/HONGEST，仅诊断） ----------
 * 不参与 contact。上电或按 'P' 时保持 START 姿态完成一次基线采集；
 * 运行时每 500 ms 取三轴均值，按到 START 基线的距离分类。
 */
#ifndef POSTURE_ENABLE
#define POSTURE_ENABLE          1
#endif
#ifndef POSTURE_CAL_SETTLE_MS
#define POSTURE_CAL_SETTLE_MS   300
#endif
#ifndef POSTURE_CAL_AVG_MS
#define POSTURE_CAL_AVG_MS      1500
#endif
#ifndef POSTURE_WINDOW_MS
#define POSTURE_WINDOW_MS       500
#endif
#ifndef POSTURE_STALE_MS
#define POSTURE_STALE_MS        500
#endif
#ifndef POSTURE_CAL_RANGE_MG
#define POSTURE_CAL_RANGE_MG    40.0f
#endif
#ifndef POSTURE_CAL_STILL_DPS
#define POSTURE_CAL_STILL_DPS   10.0f
#endif
#ifndef POSTURE_STATIC_RANGE_MG
#define POSTURE_STATIC_RANGE_MG 40.0f
#endif
#ifndef POSTURE_STATIC_GYRO_DPS
#define POSTURE_STATIC_GYRO_DPS 10.0f
#endif
#ifndef POSTURE_START_RADIUS_MG
#define POSTURE_START_RADIUS_MG 150.0f
#endif
#ifndef POSTURE_START_MAX_CTDEV_MG
#define POSTURE_START_MAX_CTDEV_MG 300.0f
#endif
#ifndef POSTURE_HONGEST_DIST_MG
#define POSTURE_HONGEST_DIST_MG 500.0f
#endif
#ifndef POSTURE_STABLE_WINDOWS
#define POSTURE_STABLE_WINDOWS  3
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

/* ---------- T014 分层检测（实验） ---------- */
/* 全部允许命令行 -D 覆盖：LAYERED_DETECT_ENABLE=0 时应完整回到 V6 行为 */
#ifndef LAYERED_DETECT_ENABLE
#define LAYERED_DETECT_ENABLE 1
#endif
#ifndef LAYER_WINDOW_N
#define LAYER_WINDOW_N        50
#endif
#ifndef LAYER_VIB_HI_MG
#define LAYER_VIB_HI_MG       120.0f
#endif
#ifndef LAYER_VIB_LO_MG
#define LAYER_VIB_LO_MG       80.0f
#endif
#ifndef LAYER_VIB_CONFIRM
#define LAYER_VIB_CONFIRM     3
#endif
#ifndef LAYER_FREQ_SPREAD_MAX
#define LAYER_FREQ_SPREAD_MAX 0.08f
#endif
#ifndef LAYER_ENV_SPREAD_MAX
#define LAYER_ENV_SPREAD_MAX  0.25f
#endif
#ifndef LAYER_VALID_RATIO_MIN
#define LAYER_VALID_RATIO_MIN 0.70f
#endif
/* 整数镜像：仅供编译期断言用（#if 不能比较浮点字面量） */
#ifndef LAYER_VIB_HI_MG_INT
#define LAYER_VIB_HI_MG_INT      120
#endif
#ifndef LAYER_VIB_LO_MG_INT
#define LAYER_VIB_LO_MG_INT      80
#endif
#ifndef LAYER_VALID_RATIO_MIN_INT
#define LAYER_VALID_RATIO_MIN_INT 70
#endif
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

/* 启停频率专用包络阈值（mg）。
 * ERM 断电后不会立即停转，3Hz 的 166ms 停止段仍会残留约 30~150mg 振动；
 * 如果直接复用接触/振动的 TH 阈值（本机约 28/14mg），vib 会一直保持 1，
 * 启停边沿永远不出现。这里仅给 ssHz 的包络状态机使用一组更高的阈值，
 * 不改变 contact、freqValid、gripNoObject 的判据。 */
/* v6 改 180/150 → 120/80（test\ss_eval.py 回放 17 段实板数据）：
 *   旧振子 3Hz ON 峰只有 150~212mg，180 门大半拍跨不过，读数 0% → 59%；
 *   弱固定振动在 150~210mg 徘徊时会在 180/150 两门之间来回跨，产生假边沿（单测\5.txt）；
 *   实测启停 OFF 段最低点中位数 5~14mg，80 门仍有余量。
 *   代价：旧振子 2Hz 读数覆盖 44% → 38%，单测\4.txt 假读数 0 → 2s，空握 2 → 3s。 */
#ifndef SS_HI_MG
#define SS_HI_MG          120.0f
#endif
#ifndef SS_LO_MG
#define SS_LO_MG          80.0f
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
#define CT_MODE          4     // v5：角度判据（0/1/2/3 保留可回退）
#endif
#define CT_K             4.0f    // 模式 0/1 用：接触阈值 = K × 静置姿态偏差
#define CT_MIN_MG        30.0f   // 模式 0/1 用：接触阈值下限
#define CT_OFF_RATIO     0.6f    // 模式 0/1 用：释放阈值 = 比例 × 接触阈值（滞回）
#define CT_CONFIRM_MS    100     // 接触状态翻转确认时间（四种模式共用）

/* ---------- 启停 CT 保持门控（任务 B） ----------
 * 默认关闭，保证出发固件行为不变；开宏后仅在“姿态仍闭合”时，
 * 用最近一次振动证据把 contact 延长到 CT_HOLD_MS，覆盖自动启停 OFF 段。
 * 进入条件不放宽，松手/姿态打开仍立即退出。
 */
#ifndef CT_HOLD_ENABLE
#define CT_HOLD_ENABLE   1
#endif
#ifndef CT_HOLD_MS
#define CT_HOLD_MS       1200
#endif

/* ---------- 启停节奏进入接触（T011 场外，2026-11-05） ----------
 * 2/3Hz 启停的 ON 窗只有 250/167ms，振子建不到 CT_FLAT_A_ON_MG，进不了接触。
 * 不降幅值门（手指动作瞬态 250~1179mg 会误进），改用"启停频率已锁定"作第二条进入证据：
 * 锁定需要 ≥SS_MIN_PERIODS 个规律周期，手指动作是一次性的形不成节奏
 * （实测空握/夹物出现过的读数只有 0.11/0.87Hz）。
 * 只对 ≥CT_SS_ENTRY_MIN_HZ 生效：0.5/1Hz 走原幅值路径即可进入。
 * 依赖 hold 覆盖 OFF 段，所以只能和 CT_HOLD_ENABLE 一起开。 */
#ifndef CT_SS_ENTRY_ENABLE
#define CT_SS_ENTRY_ENABLE  CT_HOLD_ENABLE
#endif
#ifndef CT_SS_ENTRY_MIN_HZ
#define CT_SS_ENTRY_MIN_HZ  1.5f
#endif

/* ================= v6 接触逻辑（T012 场外，2026-11-06） =================
 * 人类定的三条：
 *   进入：手指先收拢（角度门），且手指已停下，此后的振动证据才算数；
 *   保持：进入后不再看加速度，手指不动就一直是 1（不再有 CT_HOLD_MS 超时）；
 *   分离：只看陀螺 —— 张开（角度门退出）或进入后又转动超过 CT_RELEASE_DEG。
 * 「手指已停下」不能直接用 gyroW：实测振动时原始角速度中位数 22~93dps（data\*.csv 55 个文件），
 * 所以先对每轴做两级一阶低通（GYRO_LP_HZ），滤掉 80~180Hz 振动及其混叠，再判静止。
 * 陀螺削顶（强振动）时滤波暂停，静止判据按"已停下"处理：强振动下手指转动量无法测，
 * 不让它拦住接触；分离仍由角度门/flatDist 兜底。 */
#ifndef CT_V6_ENABLE
#define CT_V6_ENABLE      1
#endif
#ifndef GYRO_LP_HZ
#define GYRO_LP_HZ        4.0f    // 两级一阶低通截止；手指动作 <2Hz，振动 ≥80Hz
#endif
#ifndef CT_SETTLE_ENABLE
#define CT_SETTLE_ENABLE  1       // 0 = 不要求"已停下"（实板若发现振动时滤波后仍过门限，现场可关）
#endif
#ifndef CT_SETTLE_DPS
#define CT_SETTLE_DPS     10.0f   // 滤波后合成角速度低于此值 = 手指不动
#endif
#ifndef CT_SETTLE_MS
#define CT_SETTLE_MS      150     // 连续不动这么久 = 已停下
#endif
#ifndef CT_RELEASE_DEG
#define CT_RELEASE_DEG    10.0f   // 接触后滤波角速度累计转过此角度 = 分离（含继续收拢 = 物体滑脱）
#endif
#ifndef SS_CLEAR_ON_SETTLE
#define SS_CLEAR_ON_SETTLE 1      // 每次收拢后第一次停下时清启停历史，丢掉收拢/撞击瞬态留下的边沿
#endif
/* 实板 V6\4.txt/5.txt：0.5Hz 首个读数晚 8.0/15.9s，原因是起振前的离群周期（如 7.94s）
 * 要等它滑出 SS_USE_PERIODS 窗口。已接触时改为取最新的连续一致段，2 个周期即出读数。
 * 只在 contact=1 时生效：v5 回放显示无条件放宽会让空握假读数 11→27 秒。 */
#ifndef SS_TRAIL_IN_CONTACT
#define SS_TRAIL_IN_CONTACT CT_V6_ENABLE
#endif
/* 分离用"净转角"：接触后每轴低通角速度带泄漏积分（时间常数 CT_RELEASE_TAU_MS），取合成长度。
 * 实板 V6\5.txt 旧振子 2Hz：振幅正常、启停读数有效，接触却掉了 4 次 —— 推测是启停的反扭矩
 * 让手指来回摆，原来累计 |角速度| 把来回摆也算成转动。净转角下来回摆互相抵消；
 * 真松手/滑脱是单向转动，0.35s 左右即过 10°。泄漏让残余偏置最多积到 偏置×τ（0.5dps→0.25°）。 */
#ifndef CT_RELEASE_NET
#define CT_RELEASE_NET    1
#endif
#ifndef CT_RELEASE_TAU_MS
#define CT_RELEASE_TAU_MS 500
#endif

/* ---------- 启停缺口假边沿修正（T011-2 场外，2026-11-06） ----------
 * 实板 结果\T009\新\3.txt 75.8s：0.5Hz ON 段中途一次采样缺口后读数掉了 8s。
 * 原因：缺口后把 s_ssVib 清 0，下一个包络点（仍在 ON 段）被当成"起振"，插进 0.4s 假周期，
 * 一致性检查要等它滑出 SS_USE_PERIODS 窗口才恢复。改为置 1：缺口后先确认一次下降才接受起振。
 * 代价：缺口恰好落在真起振上时少记一个边沿（该周期算成 2 倍，同样被一致性检查拒掉）。
 * 实板回放：空握假读数不增加（11→9 秒），76~83s 掉线消失。
 * 试过"从最新周期往回取一致段"让 0.5Hz 读数提前 4s，但空握假读数 11→27 秒，已撤回。
 * 默认跟随 CT_HOLD_ENABLE，出发默认版不变。 */
#ifndef SS_EDGE_FIX_ENABLE
#define SS_EDGE_FIX_ENABLE  CT_HOLD_ENABLE
#endif

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
 * 【2026-10-31 实板实测修正】空夹最坏瞬态 304mg、真接触 500~1152mg，
 *   幅值门是当前**实际起把关作用**的判据；频率门只作辅助证据（实测频段重叠）。
 */
#define CT_FLAT_A_ON_MG   400.0f  /* 实测：空夹中位 6.2mg、最坏瞬态 304mg；真接触 500~1152mg。
                                   * 取 400：高于最坏瞬态 1.3 倍、低于最弱接触 1.25 倍。
                                   * 历史：80 曾被瞬态点着过；200 时最坏瞬态 304 仍能越过门限，
                                   * 只是那一刻主频无效才没误判。 */
#define CT_FLAT_A_OFF_MG  300.0f  /* 滞回退出 */

/* ---- (3) 频率：主频（辅助证据，不是唯一依据） ----
 * 用 r.freqHz，配合 r.freqValid。
 * 【实测结论】空夹假频 86~169Hz 与真接触 157.8~179Hz 重叠，本门限**不能**
 *   单独区分空夹与真接触；真正把关的是幅值门。保留它的作用有两点：
 *   一是真接触时提供第二条独立证据，二是幅值被削顶时兜底。
 *   freqValid=false（空夹占多数）时按 CT_FLAT_DEGRADE_STRICT 清零。
 *
 * 【TODO 待实测 -- v4 成败的关键】
 *   若实测发现物体固有频率**落在空夹频带内**，这条判据就分不开，
 *   本方案需要重新设计（例如改用幅值谱形状或双频特征）。
 *   采样时务必分别记录空夹与握有物体的 freqHz 分布。
 */
#define CT_FLAT_F_ON_HZ   75.0f   /* 严格大于；命题下限 80Hz，留 5Hz 测频余量 */
#define CT_FLAT_F_OFF_HZ  63.0f   /* 滞回退出；仍高于测频下限 60 */

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
#define CT_FLAT_A_ON_MG_INT     400
#endif
#ifndef CT_FLAT_A_OFF_MG_INT
#define CT_FLAT_A_OFF_MG_INT    300
#endif
#ifndef CT_FLAT_F_ON_HZ_INT
#define CT_FLAT_F_ON_HZ_INT     75
#endif
#ifndef CT_FLAT_F_OFF_HZ_INT
#define CT_FLAT_F_OFF_HZ_INT    63
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
#if (CT_HOLD_ENABLE != 0) && (CT_HOLD_ENABLE != 1)
  #error "CT_HOLD_ENABLE 只能为 0 或 1"
#endif
#if CT_HOLD_ENABLE && (CT_HOLD_MS <= CT_CONFIRM_MS)
  #error "CT_HOLD_MS 必须大于 CT_CONFIRM_MS"
#endif
#if CT_HOLD_ENABLE && (CT_HOLD_MS < 0)
  #error "CT_HOLD_MS 不能为负"
#endif
#if (CT_SS_ENTRY_ENABLE != 0) && (CT_SS_ENTRY_ENABLE != 1)
  #error "CT_SS_ENTRY_ENABLE 只能为 0 或 1"
#endif
#if CT_V6_ENABLE && (CT_MODE != 4)
  #error "CT_V6_ENABLE 依赖 CT_MODE=4 的陀螺闭合角"
#endif
#if CT_V6_ENABLE && !CT_HOLD_ENABLE
  #error "CT_V6_ENABLE 需要 CT_HOLD_ENABLE=1（启停进入路径依赖它）"
#endif
#if CT_V6_ENABLE && (CT_SETTLE_MS < 0)
  #error "CT_SETTLE_MS 不能为负"
#endif
#if (SS_EDGE_FIX_ENABLE != 0) && (SS_EDGE_FIX_ENABLE != 1)
  #error "SS_EDGE_FIX_ENABLE 只能为 0 或 1"
#endif
#if CT_SS_ENTRY_ENABLE && !CT_HOLD_ENABLE
  #error "CT_SS_ENTRY_ENABLE 依赖 CT_HOLD_ENABLE：没有 hold，OFF 段会在 CT_CONFIRM_MS 后掉 0"
#endif
#if (CAL_RETRY_MS <= 0)
  #error "CAL_RETRY_MS 必须大于 0"
#endif
#if (COMP_PERIOD_MS <= 0)
  #error "COMP_PERIOD_MS 必须大于 0"
#endif

/* ---------- 模式 3/4（平放三重与）参数检查 ----------
 * 模式 4 复用同一组 A/F 门限，必须一起做编译期检查。
 */
#if (CT_MODE == 3) || (CT_MODE == 4)
  #if (CT_FLAT_D_OFF_MG_INT >= CT_FLAT_D_ON_MG_INT)
    #error "CT_FLAT_D_OFF 必须小于 CT_FLAT_D_ON（否则滞回带反了，接触会在阈值处抖动）"
  #endif
  #if (CT_FLAT_A_OFF_MG_INT >= CT_FLAT_A_ON_MG_INT)
    #error "CT_FLAT_A_OFF 必须小于 CT_FLAT_A_ON（同上）"
  #endif
  #if (CT_FLAT_F_OFF_HZ_INT >= CT_FLAT_F_ON_HZ_INT)
    #error "CT_FLAT_F_OFF 必须小于 CT_FLAT_F_ON（同上）"
  #endif
  #if (CT_FLAT_F_ON_HZ_INT >= 80)
    #error "CT_FLAT_F_ON 高于命题频段下限 80Hz，80~门限之间的合规振源将永远判不出接触"
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

/* ---------- T014 分层检测参数检查 ---------- */
#if (LAYER_WINDOW_N < 2) || (LAYER_WINDOW_N > 512)
  #error "LAYER_WINDOW_N 必须在 2~512（窗口按此静态分配，过大会吃 RAM）"
#endif
#if (LAYER_VIB_CONFIRM < 1)
  #error "LAYER_VIB_CONFIRM 必须 >= 1（否则单帧即翻转，失去确认意义）"
#endif
#if (LAYER_VIB_HI_MG_INT <= LAYER_VIB_LO_MG_INT)
  #error "LAYER_VIB_HI 必须大于 LAYER_VIB_LO（否则滞回带反了，振动标志会在阈值处抖动）"
#endif
#if (LAYER_VALID_RATIO_MIN_INT < 0) || (LAYER_VALID_RATIO_MIN_INT > 100)
  #error "LAYER_VALID_RATIO_MIN_INT 必须在 0~100"
#endif

#if (CT_REF_AVG_MS <= 0)
  #error "CT_REF_AVG_MS 必须大于 0（没有样本就无法求平均）"
#endif
#if (CT_REF_AUTO_RETRY < 0)
  #error "CT_REF_AUTO_RETRY 不能为负"
#endif
