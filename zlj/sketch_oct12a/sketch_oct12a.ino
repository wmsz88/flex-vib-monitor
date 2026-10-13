/* =====================================================================
 *  柔性振动检测模块 · 代码二：振动检测主程序  (vibmon v1.1 · 评审修正版)
 * ---------------------------------------------------------------------
 *  目标板 : ESP32-C3（组委会扩展模块，arduino-esp32 内核 3.x）
 *  引脚依据: 电路搭建\PCB\Netlist_PCB1_2026-09-24.tel（定稿网表）
 *
 *  ┌─ v1.1 相对 v1.0 的修改（逐条对应评审意见）───────────────────────┐
 *  │ [M1] 补 #include <stdarg.h>/<stdio.h>，消除依赖 Arduino.h 隐式包含 │
 *  │ [M2] 新增 SERIAL_VERBOSE 总开关：0 = 比赛口只跑 $VD 帧，          │
 *  │      横幅/标定/帮助/调试行全部静默，防止上位机解析被杂线干扰。     │
 *  │ [M3] 调试行默认关闭（原为默认开）——现场用 'd' 随时打开。          │
 *  │ [M4] 传感器故障停机指示改为“绿灯灭 + 黄灯 2Hz 快闪”，             │
 *  │      避免评委把绿灯闪烁误判成“程序正常运行”。                     │
 *  │ [M5] 标定阶段A 的忙等循环里定期 delay(1) 让出 CPU，喂空闲任务狗。  │
 *  │ [M6] 启停频率统计窗 4000→8000ms、边沿缓存 16→32，低频下限         │
 *  │      由 0.5Hz 放宽到 0.25Hz（命题要求 0~3Hz）。                   │
 *  │ [M7] 去掉对 UART 引脚调用 pinMode（ESP32 上会与 UART 驱动抢引脚），│
 *  │      改由 UART 驱动自行配置。                                    │
 *  │ [M8] 帧格式增加 FRAME_WITH_CONTACT 开关（默认0=与 v1.0 完全一致），│
 *  │      置 1 时在帧尾追加“接触状态”位，用于满足命题“接触状态识别+串口  │
 *  │      输出”的要求——具体取哪种以组委会/现场公布的协议为准。        │
 *  │ [M9] 状态行补充打印协议开关、接触门限，便于现场排障。             │
 *  └────────────────────────────────────────────────────────────────┘
 *
 *  ★★★ 上场比赛前必须确认的 3 件事（代码里已留好“一键改”位置）★★★
 *   ① 串口协议：本程序默认发 $VD 自定义帧（见 sendFrame）。命题正文未
 *      给出帧格式，第二环节/现场命题可能另有规定 —— 若不是这种帧，
 *      只改 sendFrame() 里那一个 snprintf 即可，算法全不用动。
 *   ② 波特率：默认 115200（改 BAUD 一处）。
 *   ③ LED 极性：LED_ACTIVE_LOW。若 PCB 是 5V→电阻→LED→GPIO（灌电流），
 *      必须改成 1，否则绿/黄灯逻辑整体反相。
 *   另需确认：TX/RX 是否要互换（UART_SWAP），默认 TX=GPIO21/RX=GPIO20。
 *
 *  ┌─ 软件架构（采样与算法解耦）────────────────────────────────────┐
 *  │  esp_timer 1kHz 周期回调 ──读MPU 6字节──► 环形缓冲(512)         │
 *  │  loop()：环形缓冲 ─► 去直流 ─► 二阶高通 ─► 合成功率包络(RMS)    │
 *  │         ├─ 100Hz：双阈值滞回 ─► 启停状态机 ─► 启停频率           │
 *  │         ├─ 15.6Hz(每64点)：滞回过零+线性插值+周期中位数 ─► 主频  │
 *  │         └─ 50Hz：组 $VD 帧 ─► UART 输出（发送永不阻塞采样）      │
 *  └────────────────────────────────────────────────────────────────┘
 *
 *  赛题指标：持续振动主频 80~180Hz（±2Hz）；启停频率 0~3Hz
 *            （状态翻转确认延迟 30ms，远小于 0.5s 预算）
 *
 *  引脚：SDA=GPIO8  SCL=GPIO9  绿LED=GPIO10  黄LED=GPIO5
 *        TX=GPIO21   RX=GPIO20（4PIN ①/②）  红LED 硬件直连 5V
 *
 *  关键约束：
 *    · 严禁启用 Wi-Fi / BLE（LDO 压差 7V，射频峰值会造成热关断）
 *    · 不使用 String / 动态内存 / DMP
 *    · 不用 MPU 的 DRDY 中断（INT 脚在板上悬空），只用定时器定速采样
 *
 *  烧录：按住 BOOT → 按一下 RESET → 松开 RESET → 松开 BOOT，115200 8N1
 * ===================================================================== */

#include <Wire.h>
#include <esp_timer.h>
#include <stdarg.h>          // [M1] va_list / va_start
#include <stdio.h>           // [M1] vsnprintf / snprintf
#include <math.h>            // [M1] sqrtf / cosf / sinf / fmaxf

/* ========================= 配置区 ========================= */

#define PIN_SDA       8      // 网表 SDA ; U1.4 U3.4
#define PIN_SCL       9      // 网表 SCL ; U1.5 U3.3
#define PIN_LED_G     10     // 网表 P10 ; R7.2 U1.6（绿：心跳）
#define PIN_LED_Y     5      // 网表 P5  ; R8.1 U1.1（黄：振动中指示）

#define UART_SWAP     0      // 0 = TX→GPIO21/RX→GPIO20（与网表一致）
                             // 1 = 交换（现场与平台不通时改这里）
#if UART_SWAP
  #define PIN_UART_TX 20
  #define PIN_UART_RX 21
#else
  #define PIN_UART_TX 21
  #define PIN_UART_RX 20
#endif

#define BAUD          115200     // 现场若平台要求改波特率，只改这一处
#define I2C_CLOCK     200000UL   // 200kHz：1kHz 采样下读 6 字节约 0.42ms，余量 2.4x
                                 // 自检的压力测试不过就降到 100000UL（并考虑 FS_HZ 降 500）
#define MPU_ADDR      0x68       // AD0 接地

#define FS_HZ         1000       // 采样率 Hz。I2C 吃紧或拍频异常时可降 500 / 900
#define BLOCK_N       128        // 频率估计窗长（样本）= 128ms，必须是 2 的幂（为 FFT 留路）
#define HOP           64         // 窗滑动步长 = 64ms → 主频更新率 15.6Hz
#define RING_N        512        // 环形缓冲深度（2 的幂）= 512ms 容错

#define ENV_WIN       30         // 包络滑窗 = 30ms ≈ 180Hz 的 5 个周期
#define ENV_HOP       10         // 包络输出间隔 = 10ms → 包络序列 100Hz
#define SS_CONFIRM    3          // 状态翻转需连续 3 个包络点确认 = 30ms（远小于 0.5s 预算）
#define SS_EDGE_MAX   32         // [M6] 记录"起"边沿的最大个数（原 16）
#define SS_WIN_MS     8000       // [M6] 启停频率统计滑窗（原 4000）
                                 //      需 >=3 个"起"边沿 = 2 个完整周期
                                 //      → 可测下限 = 2 / 8s = 0.25Hz
#define SS_MIN_SPAN_MS 500       // 至少要铺开 500ms 才敢给启停频率（3Hz 时约 667ms，OK）
#define FREQ_LO_HZ    60.0f      // 有效周期判据（样本间隔）对应频率范围
#define FREQ_HI_HZ    220.0f
#define MIN_PERIODS   5          // 窗内少于此周期数就判"无持续振动"

#define HP_FC_HZ      70.0f      // 二阶高通截止：80Hz 处 -2.0dB，50Hz 工频 -6.8dB
                                 // 50Hz 工频不会被完全压掉，但过零周期的 60~220Hz
                                 // 带通判据会把 50Hz 造成的长周期整段丢弃
#define HP_Q          0.7071f
#define ZCL_K         3.6f       // 过零阈值 = 3.6 x 带内噪声 sigma
#define ZCL_MIN_MG    1.0f
#define TH_HI_K       5.0f       // 启停高阈值 = 5 x 静置包络
#define TH_LO_K       2.5f
#define TH_HI_MIN_MG  8.0f
#define TH_LO_MIN_MG  4.0f
#define CT_TAU_S      1.0f       // 接触判定的低频分量时间常数 1s
#define CT_K          4.0f       // 接触阈值 = 4 x 静置低频水平
#define CT_MIN_MG     30.0f

#define CAL_BIAS_MS   2000       // 阶段A：静置零偏标定
#define CAL_NOISE_MS  1200       // 阶段B：带内噪声 / 包络基线

#define FRAME_MS      20         // 帧率 50Hz
#define DBG_MS        200        // '#' 调试行 5Hz
#define RAW_MS        10         // 'R' 原始数据 dump 100Hz

#define FREQ_ALGO     0          // 0 = 简易法（滞回过零+插值+中位数，本版）
                                 // 1 = FFT（桩函数，会自动回退到简易法，见文末）
#define LED_ACTIVE_LOW 0         // ★LED 极性：0 = GPIO 高电平点亮（GPIO→电阻→LED→GND）
                                 //   1 = 灌电流（5V→电阻→LED→GPIO，GPIO 低电平点亮）
                                 //   必须与 PCB 实际接法一致，否则绿/黄灯逻辑整体反相

/* [M2] 文字/调试信息总开关。
   0 = 比赛口(GPIO20/21)只输出 $VD 帧，横幅/标定/帮助/调试行全部不发，
       彻底避免上位机解析被杂线干扰；
   1 = 正常打印（开发调试用）。 */
#define SERIAL_VERBOSE 1

/* [M3] 调试行上电默认状态。比赛建议 0，现场按 'd' 随时打开。 */
#define DBG_DEFAULT   0

/* [M8] 帧尾是否追加"接触状态"位。
   0 = 与 v1.0 完全一致的 6 字段帧（默认）；
   1 = 追加 ,<ct>（1=有接触 / 0=无接触），满足命题"接触状态识别并串口输出"。
   ★ 以组委会/现场公布的协议为准，确认后把这里改成对应取值。 */
#define FRAME_WITH_CONTACT 0

/* MPU6050 寄存器（依据官方 RM-MPU-6000A-00，详见 代码\MPU6050寄存器速查.md） */
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75
#define REG_VALUE_DLPF   0x00    
/* 
 * CONFIG(0x1A) 的 DLPF_CFG 字段
 * DLPF_CFG=0 → 加速度带宽 260Hz / 延迟 0ms（对 180Hz 只衰减约 0.7dB）
 *              陀螺仪输出率基准 = 8kHz（本程序不用陀螺仪）
 * DLPF_CFG=1 → 加速度带宽 184Hz / 延迟 2.0ms，陀螺仪基准 = 1kHz
 *              （压噪声更好，但 180Hz 会被压约 3dB）
 * 赛题上限是 180Hz，所以选 0 保住带宽最稳。
 * ★官方原文 Note：加速度输出率恒为 1kHz，与 DLPF_CFG 无关 ——
 *   这正是"1kHz 采样率 + 只读加速度"成立的根本原因。 
 */
#define ACCEL_LSB_PER_G  16384.0f

/* 编译期自检：BLOCK_N / RING_N 必须是 2 的幂 */
#if (BLOCK_N & (BLOCK_N - 1)) || (RING_N & (RING_N - 1))
  #error "BLOCK_N 与 RING_N 必须是 2 的幂"
#endif

/* 编译期自检：SS_EDGE_MAX 必须放得下 uint8_t 计数值 */
#if SS_EDGE_MAX > 200
  #error "SS_EDGE_MAX 过大(g_edgeCnt 是 uint8_t)"
#endif

/* ========================= 输出层 ========================= */
/* 勾选 "USB CDC On Boot" 时 Serial 是 USB-CDC，可镜像调试信息；
   否则 Serial 与 Serial1 抢同一个 UART0(GPIO20/21)，不镜像，避免驱动打架。 */
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  #define MIRROR_USB 1
#else
  #define MIRROR_USB 0
#endif

static HardwareSerial Comp(1);   // 比赛口：显式绑 GPIO20/21

/* $VD 帧专用输出：永远发（比赛口 + 可选 USB 镜像） */
static void vdRaw(const char* s, size_t n) {
  Comp.write((const uint8_t*)s, n);
#if MIRROR_USB
  Serial.write((const uint8_t*)s, n);
#endif
}

/* 文字/调试信息输出：受 SERIAL_VERBOSE 控制 [M2] */
static void vdTxtRaw(const char* s, size_t n) {
#if SERIAL_VERBOSE
  Comp.write((const uint8_t*)s, n);
  #if MIRROR_USB
  Serial.write((const uint8_t*)s, n);
  #endif
#else
  (void)s; (void)n;
#endif
}

static void vdPrintf(const char* fmt, ...) {   // 定长缓冲，不用 String、不动态分配
  char buf[240];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
  vdTxtRaw(buf, (size_t)n);
}

/* ========================= 全局状态 ========================= */

/* I2C / 传感器 */
static volatile uint32_t g_i2cErr = 0;

/* 环形缓冲：单生产者（esp_timer 任务）单消费者（loop 任务） */
static float    g_rx[RING_N], g_ry[RING_N], g_rz[RING_N];
static volatile uint16_t g_rHead = 0, g_rTail = 0;
static volatile uint32_t g_ringDrop = 0;

/* 标定与阈值 */
static float g_bias[3]   = {0, 0, 0};      // mg
static float g_sigBand[3]= {0, 0, 0};      // 带内噪声 sigma, mg
static float g_zcl = ZCL_MIN_MG;           // 过零阈值 mg
static float g_thHi = TH_HI_MIN_MG;        // 启停高阈值 mg
static float g_thLo = TH_LO_MIN_MG;        // 启停低阈值 mg
static float g_thCt = CT_MIN_MG;           // 接触阈值 mg
static float g_envRest = 0.0f;             // 静置包络 mg
static float g_ctRest  = 0.0f;             // 静置低频水平 mg
static bool  g_calib = false;              // 标定模式：正常收样本，但不跑检测
static bool  g_lastCalOk = false;

/* 高通滤波器（二阶 Butterworth，Direct Form II Transposed） */
static float g_b0, g_b1, g_b2, g_a1, g_a2;
static float g_z1[3], g_z2[3];

/* 历史（环形）与包络 */
static float    g_hist[3][BLOCK_N];
static uint16_t g_histIdx = 0;
static uint32_t g_histTotal = 0;
static uint16_t g_hopCnt = 0;

static float    g_envBuf[ENV_WIN];
static uint8_t  g_envIdx = 0;
static int      g_envFill = 0;
static float    g_envSumSq = 0.0f;
static uint8_t  g_envHopCnt = 0;
static float    g_envLevel = 0.0f;

/* 启停状态机 */
static uint8_t  g_vibState = 0;            // 0=停 1=振
static uint8_t  g_hiCnt = 0, g_loCnt = 0;
static uint32_t g_startT[SS_EDGE_MAX] = {0};
static uint8_t  g_edgeIdx = 0;
static uint8_t  g_edgeCnt = 0;
static uint32_t g_nStart = 0, g_nStop = 0, g_nFrames = 0;

/* 接触判定 */
static float g_ctLevel = 0.0f;
static uint8_t g_contact = 0;              // [M8] 1 = 判定为接触

/* 最近一次窗口算法的结果 */
static float g_freqHz = 0.0f;
static float g_ampMg  = 0.0f;
static uint8_t g_domAxis = 2;

/* 标定统计累加器 */
static double g_calBandSq[3] = {0, 0, 0};
static double g_calEnvAcc = 0;
static uint32_t g_calEnvN = 0;
static double g_calCtAcc = 0;      // 标定期直接累加 acMag 求均值 ——
static uint32_t g_calCtN = 0;      // 不用 EMA：tau=1s 而标定只有 1.2s，EMA 只收敛到 70%，阈值会算低

/* 输出开关 */
static bool g_dbgOn = DBG_DEFAULT;         // [M3] 默认关，现场用 'd' 打开
static bool g_rawOn = false;
static uint16_t g_seq = 0;

/* ========================= 小工具 ========================= */

static inline void ledWrite(int pin, int on) {
#if LED_ACTIVE_LOW
  digitalWrite(pin, on ? LOW : HIGH);
#else
  digitalWrite(pin, on ? HIGH : LOW);
#endif
}

static inline int16_t be16(const uint8_t* p) { return (int16_t)((p[0] << 8) | p[1]); }
static inline float   raw2mg(int16_t r) { return (float)r * (1000.0f / ACCEL_LSB_PER_G); }

static float medianOf(float* a, int n) {      // n 很小（<=64），插入排序足够
  for (int i = 1; i < n; i++) {
    float v = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
  return (n & 1) ? a[n / 2] : 0.5f * (a[n / 2 - 1] + a[n / 2]);
}

/* ========================= I2C 底层 ========================= */

static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  bool ok = (Wire.endTransmission() == 0);
  if (!ok) g_i2cErr = g_i2cErr + 1;      // volatile 上不用 ++（C++20 已弃用）
  return ok;
}

static bool mpuReadReg(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { g_i2cErr = g_i2cErr + 1; return false; }
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)len) != (size_t)len) { g_i2cErr = g_i2cErr + 1; return false; }
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

/* 只读 6 字节加速度：83 bit @200kHz ≈ 0.42ms，落在 1ms 周期里余量 2.4x */
static inline bool mpuReadAccel6(float* xmg, float* ymg, float* zmg) {
  uint8_t b[6];
  if (!mpuReadReg(REG_ACCEL_XOUT_H, b, 6)) return false;
  *xmg = raw2mg(be16(&b[0]));
  *ymg = raw2mg(be16(&b[2]));
  *zmg = raw2mg(be16(&b[4]));
  return true;
}

/* ========================= 环形缓冲 ========================= */

static inline bool ringPop(float* x, float* y, float* z) {
  if (g_rTail == g_rHead) return false;
  *x = g_rx[g_rTail];
  *y = g_ry[g_rTail];
  *z = g_rz[g_rTail];
  g_rTail = (uint16_t)((g_rTail + 1) & (RING_N - 1));
  return true;
}

/* ========================= 采样回调（1kHz） =========================
 * 运行在 esp_timer 任务上下文（不是中断），可以安全调用 Wire。
 * 只做：读 6 字节 → 转 mg → 入环形缓冲。不做任何算法。 */
static void IRAM_ATTR onSampleTimer(void*) {
  float x, y, z;
  if (!mpuReadAccel6(&x, &y, &z)) return;
  uint16_t n = (uint16_t)((g_rHead + 1) & (RING_N - 1));
  if (n == g_rTail) { g_ringDrop = g_ringDrop + 1; return; }  // 满：宁可丢新样，不覆盖未消费数据
  g_rx[g_rHead] = x;
  g_ry[g_rHead] = y;
  g_rz[g_rHead] = z;
  g_rHead = n;
}

/* ========================= 滤波器 ========================= */

static void hpInit(void) {
  float w0 = 2.0f * PI * HP_FC_HZ / (float)FS_HZ;
  float cw = cosf(w0), sw = sinf(w0);
  float alpha = sw / (2.0f * HP_Q);
  float a0 = 1.0f + alpha;
  g_b0 = (1.0f + cw) * 0.5f / a0;
  g_b1 = -(1.0f + cw) / a0;
  g_b2 = g_b0;
  g_a1 = -2.0f * cw / a0;
  g_a2 = (1.0f - alpha) / a0;
  for (int i = 0; i < 3; i++) { g_z1[i] = 0; g_z2[i] = 0; }
}

static inline float hpStep(int axis, float x) {
  float y = g_b0 * x + g_z1[axis];
  g_z1[axis] = g_b1 * x - g_a1 * y + g_z2[axis];
  g_z2[axis] = g_b2 * x - g_a2 * y;
  return y;
}

/* ========================= 启停检测（100Hz 推进） ========================= */

static void ssReset(void) {
  g_vibState = 0; g_hiCnt = 0; g_loCnt = 0;
  g_edgeIdx = 0; g_edgeCnt = 0;
  for (int i = 0; i < SS_EDGE_MAX; i++) g_startT[i] = 0;
  g_nStart = 0; g_nStop = 0;
}

/* 双阈值滞回 + 连续 SS_CONFIRM 次确认。
   单阈值会在噪声边界反复穿越，把 3Hz 数成几十 Hz —— 这是必须用滞回的原因。 */
static void ssStep(float E) {
  uint32_t now = millis();
  if (g_vibState == 0) {
    if (E > g_thHi) {
      if (++g_hiCnt >= SS_CONFIRM) {
        g_hiCnt = 0;
        g_vibState = 1;
        g_startT[g_edgeIdx] = now;
        g_edgeIdx = (uint8_t)((g_edgeIdx + 1) % SS_EDGE_MAX);
        if (g_edgeCnt < SS_EDGE_MAX) g_edgeCnt++;
        g_nStart++;
      }
    } else {
      g_hiCnt = 0;
    }
  } else {
    if (E < g_thLo) {
      if (++g_loCnt >= SS_CONFIRM) { g_loCnt = 0; g_vibState = 0; g_nStop++; }
    } else {
      g_loCnt = 0;
    }
  }
}

/* 启停频率 = "起"边沿在滑窗内的平均重复率。
   注意：启停频率本身无法瞬时测出——测一个周期至少要等一个周期。
   这里必须在滑窗内凑够 >=3 个"起"边沿（= 2 个完整周期）才给读数，
   否则输出 0 表示"未确立"。配合 SS_WIN_MS=8000ms，可测下限 0.25Hz，
   覆盖命题的 0~3Hz。（若只给 2 个边沿就出数，一次误触发就会报出假频率。） */
static float ssCompute(void) {
  if (g_edgeCnt < 3) return 0.0f;
  uint32_t now = millis();
  uint32_t tmin = 0, tmax = 0;
  int n = 0;
  for (uint8_t i = 0; i < g_edgeCnt; i++) {
    uint32_t t = g_startT[i];
    if (t == 0) continue;
    if ((uint32_t)(now - t) > SS_WIN_MS) continue;
    if (n == 0 || (int32_t)(t - tmin) < 0) tmin = t;
    if (n == 0 || (int32_t)(t - tmax) > 0) tmax = t;
    n++;
  }
  if (n < 3) return 0.0f;
  uint32_t span = tmax - tmin;
  if (span < SS_MIN_SPAN_MS) return 0.0f;
  return (float)(n - 1) * 1000.0f / (float)span;
}

/* ========================= 主频估计 ========================= */

static float estimateFreqSimple(const float* x, int N) {
  /* 选中的轴已在调用方做过去均值；这里做滞回过零 + 线性插值 + 周期中位数。
     插值让分辨率不受 fs/N 限制 —— 这就是简易法也能做到 ±2Hz 的原因：
     N=128 时 FFT 的 Δf=7.8Hz，而插值可到亚 Hz。 */
  const float zc = g_zcl;
  const float tMin = (float)FS_HZ / FREQ_HI_HZ;   // 最短周期（样本）
  const float tMax = (float)FS_HZ / FREQ_LO_HZ;   // 最长周期（样本）

  float tUp[BLOCK_N / 2];
  int   nUp = 0;
  int   armed = 0;                                // 必须先越过 -zc 才允许记 +zc

  for (int i = 1; i < N; i++) {
    if (!armed) {
      if (x[i] < -zc) armed = 1;
    } else if (x[i] > zc) {
      float d = x[i] - x[i - 1];
      if (d > 1e-9f && nUp < (BLOCK_N / 2)) {
        float t = (float)(i - 1) + (zc - x[i - 1]) / d;   // 亚采样点精度的过零时刻
        tUp[nUp++] = t;
      }
      armed = 0;
    }
  }
  if (nUp < 2) return 0.0f;

  float T[BLOCK_N / 2];
  int nT = 0;
  for (int k = 1; k < nUp; k++) {
    float d = tUp[k] - tUp[k - 1];
    if (d >= tMin && d <= tMax) T[nT++] = d;      // 带外周期直接丢弃（防谐波/防50Hz工频）
  }
  if (nT < MIN_PERIODS) return 0.0f;

  float tmed = medianOf(T, nT);                   // 中位数：少数被污染的周期拽不动它
  if (tmed <= 0.0f) return 0.0f;
  float f = (float)FS_HZ / tmed;
  if (f < FREQ_LO_HZ || f > FREQ_HI_HZ) return 0.0f;
  return f;
}

/* FFT 路径（本版为桩，调用方会自动回退到简易法）。
   填入时只需实现下面两步，主流程 / 缓冲 / 协议 / LED 全部不用动：
     1) static void fft_radix2(float* re, float* im, int N);  原地 radix-2，N 为 2 的幂
     2) 汉宁窗：x[i] *= 0.5f*(1.0f - cosf(2.0f*PI*i/(N-1)));
     3) 取模后找 [FREQ_LO_HZ, FREQ_HI_HZ] 内峰值 bin，再做三点抛物线峰值插值
        d = 0.5f*(a-c)/(a-2b+c)，f = (k+d)*fs/N
     4) 回到 setup 把 FREQ_ALGO 改成 1 */
#if (FREQ_ALGO == 1)
static float estimateFreqFFT(const float* x, int N, float fs) {
  (void)x; (void)N; (void)fs;
  return -1.0f;   // TODO：按上面 1)~3) 实现；返回 <0 会让调用方回退简易法
}
#endif

/* ========================= 窗口处理（每 HOP 个样本一次） ========================= */

static void runWindow(void) {
  /* 1) 把环形历史按时间顺序拉直，同时求各轴能量与均值 */
  float win[3][BLOCK_N];
  double sum[3] = {0, 0, 0};
  double sq[3]  = {0, 0, 0};
  for (int i = 0; i < BLOCK_N; i++) {
    uint16_t idx = (uint16_t)((g_histIdx + i) & (BLOCK_N - 1));
    for (int a = 0; a < 3; a++) {
      float v = g_hist[a][idx];
      win[a][i] = v;
      sum[a] += v;
      sq[a] += (double)v * v;
    }
  }
  /* 2) 选交流能量最大的轴做主频估计（柔性板贴在手指上，姿态不定，
        固定用 Z 轴会漏检 —— 按能量自动选轴才稳） */
  float var[3];
  for (int a = 0; a < 3; a++) {
    double m = sum[a] / BLOCK_N;
    var[a] = (float)(sq[a] / BLOCK_N - m * m);
  }
  int ax = 0;
  if (var[1] > var[ax]) ax = 1;
  if (var[2] > var[ax]) ax = 2;
  g_domAxis = (uint8_t)ax;

  /* 3) 去均值后过零 → 主频 */
  float m = (float)(sum[ax] / BLOCK_N);
  for (int i = 0; i < BLOCK_N; i++) win[ax][i] -= m;

#if (FREQ_ALGO == 1)
  float f = estimateFreqFFT(win[ax], BLOCK_N, (float)FS_HZ);
  if (f <= 0.0f) f = estimateFreqSimple(win[ax], BLOCK_N);
#else
  float f = estimateFreqSimple(win[ax], BLOCK_N);
#endif

  /* 4) 只在"振动中"才对外报主频；停止时给 0（协议里 00000 = 无持续振动） */
  g_freqHz = (g_vibState == 1) ? f : 0.0f;
  g_ampMg  = g_envLevel;
}

/* ========================= 采样流水线（主循环里跑） ========================= */

static void drainPipeline(void) {
  /* 把环形缓冲里的新样本全部取出：去直流 → 二阶高通 → 入历史 → 包络 */
  float x, y, z;
  while (ringPop(&x, &y, &z)) {
    /* 接触判定的低频支路：用"原始减静置偏置"的模长，重力方向改变会抬升它。
       注意不能用滤波后的值 —— 高通已经把接触带来的静态变化滤掉了。
       ★局限：这条支路靠"姿态/重力方向变化"判接触，手指平移靠近而姿态不变
         时不一定抬得起来。所以下面 g_contact 做成了"低频变化 或 已起振"的
         并集——接触与振动本身就是强相关的。 */
    float dx = x - g_bias[0], dy = y - g_bias[1], dz = z - g_bias[2];
    float acMag = sqrtf(dx * dx + dy * dy + dz * dz);
    if (g_calib) {
      g_calCtAcc += acMag;           // 标定期：直接求均值，无收敛误差
      g_calCtN++;
    } else {
      g_ctLevel += (acMag - g_ctLevel) * (1.0f / ((float)FS_HZ * CT_TAU_S));
    }

    /* 振动支路 */
    float fx = hpStep(0, dx);
    float fy = hpStep(1, dy);
    float fz = hpStep(2, dz);

    g_hist[0][g_histIdx] = fx;
    g_hist[1][g_histIdx] = fy;
    g_hist[2][g_histIdx] = fz;
    g_histIdx = (uint16_t)((g_histIdx + 1) & (BLOCK_N - 1));
    g_histTotal++;

    /* 合成功率的滑动 RMS 包络（与姿态无关，比单轴更适合做"有没有振"的判据） */
    float msg = fx * fx + fy * fy + fz * fz;
    g_envSumSq -= g_envBuf[g_envIdx];
    g_envBuf[g_envIdx] = msg;
    g_envSumSq += msg;
    g_envIdx = (uint8_t)((g_envIdx + 1) % ENV_WIN);
    if (g_envFill < ENV_WIN) g_envFill++;

    if (++g_envHopCnt >= ENV_HOP) {
      g_envHopCnt = 0;
      if (g_envFill >= ENV_WIN) {
        g_envLevel = sqrtf(g_envSumSq / (float)ENV_WIN);
        if (g_calib) {
          g_calEnvAcc += g_envLevel;
          g_calEnvN++;
        } else {
          ssStep(g_envLevel);
          g_contact = ((g_ctLevel > g_thCt) || (g_vibState == 1)) ? 1 : 0;  // [M8]
        }
      }
    }

    if (g_calib) {
      g_calBandSq[0] += (double)fx * fx;
      g_calBandSq[1] += (double)fy * fy;
      g_calBandSq[2] += (double)fz * fz;
    }

    if (++g_hopCnt >= HOP) {
      g_hopCnt = 0;
      if (g_histTotal >= BLOCK_N && !g_calib) runWindow();
    }
  }
}

/* ========================= 标定 ========================= */

static void resetPipeline(void) {
  g_rHead = 0;
  g_rTail = 0;
  g_ringDrop = 0;
  g_histIdx = 0; g_histTotal = 0; g_hopCnt = 0;
  g_envIdx = 0; g_envFill = 0; g_envSumSq = 0; g_envLevel = 0; g_envHopCnt = 0;
  g_ctLevel = 0;
  g_contact = 0;
  hpInit();
  ssReset();
  g_freqHz = 0; g_ampMg = 0;
}

static esp_timer_handle_t g_timer = nullptr;

static void stopTimer(void) {
  if (g_timer) { esp_timer_stop(g_timer); }
}

static void startTimer(void) {
  if (!g_timer) {
    esp_timer_create_args_t cfg = {};
    cfg.callback = &onSampleTimer;
    cfg.name = "vd_smp";
    /* 说明：回调跑在 esp_timer 任务上下文（非 ISR），因此可以调用 Wire。
       如担心被高优先级任务抢占导致丢样，可显式指定
       cfg.dispatch_method = ESP_TIMER_TASK; 与优先级参数。 */
    esp_timer_create(&cfg, &g_timer);
  }
  esp_timer_start_periodic(g_timer, 1000000ULL / FS_HZ);
}

/* 一边等一边把流水线跑起来：delay(1) 让出 CPU，定时器任务才能取到样本 */
static void waitDrain(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    drainPipeline();
    delay(1);
  }
  drainPipeline();
}

static bool mpuInit(void) {
  stopTimer();
  Wire.begin(PIN_SDA, PIN_SCL, (uint32_t)I2C_CLOCK);
  Wire.setTimeOut(50);

  uint8_t who = 0;
  if (!mpuReadReg(REG_WHO_AM_I, &who, 1) || who != 0x68) {
    vdPrintf("[FATAL] WHO_AM_I = 0x%02X(期望 0x68)—— 先跑 selftest 查硬件\n", who);
    return false;
  }
  mpuWrite(REG_PWR_MGMT_1, 0x80);   // 软复位
  delay(100);
  mpuWrite(REG_PWR_MGMT_1, 0x00);   // 唤醒，内部 8MHz 时钟
  delay(50);
  mpuWrite(REG_ACCEL_CONFIG, 0x00); // ±2g，16384 LSB/g（振动幅度小，要最细的分辨率）
  mpuWrite(REG_GYRO_CONFIG, 0x00);  // ±250dps（本程序不用陀螺仪，保持默认）
  mpuWrite(REG_CONFIG, REG_VALUE_DLPF);
  /* SMPLRT_DIV：Sample Rate = 陀螺仪输出率 / (1 + SMPLRT_DIV)。
     官方原文：DLPF 关闭(DLPF_CFG=0/7) 时基准 8kHz，DLPF 打开(1~6) 时基准 1kHz。
     这里 DLPF_CFG=0 → 基准 8kHz，写 0 得 8kHz 寄存器刷新率；但
     ★加速度输出率恒为 1kHz，所以我们按 1kHz 读，每次拿到的都是新样本，不重不漏、无混叠。
     （注意：MPU 内部时钟与 ESP32 定时器不是同一个振荡源，两者若有微小频差，
       会形成很慢的"重复/漏采"拍频。过零周期中位数对这类个别畸形周期不敏感，
       实测若主频抖动偏大，可把 FS_HZ 降到 500/900 或改用定长均值抗拍频。） */
  mpuWrite(REG_SMPLRT_DIV, 0x00);
  delay(50);
  vdPrintf("[OK] MPU6050 就绪(±2g / DLPF_CFG=%u,加速度带宽 260Hz、输出率 1kHz)\n", REG_VALUE_DLPF);
  return true;
}

static bool calibrateAll(void) {
  stopTimer();
  resetPipeline();

  /* ---- 阶段A：静置零偏（2s，约 2000 点，不过滤，直接求均值） ---- */
  vdPrintf("\n[CAL] 阶段A:静置零偏标定 %u ms —— 请勿触碰模块\n", (unsigned)CAL_BIAS_MS);
  double sx = 0, sy = 0, sz = 0;
  uint32_t n = 0, err0 = g_i2cErr;
  uint32_t t0 = millis();
  while (millis() - t0 < CAL_BIAS_MS) {
    float x, y, z;
    if (mpuReadAccel6(&x, &y, &z)) { sx += x; sy += y; sz += z; n++; }
    /* [M5] 定期让出 CPU：不然这里 2s 死循环会饿死空闲任务，
       在 XTAL_32K 缺失的板子上可能触发看门狗告警。 */
    if ((n & 31u) == 0u) delay(1);
  }
  if (n < 200) {
    vdPrintf("[CAL] 失败：只取到 %u 点,I2C 基本不通\n", n);
    g_lastCalOk = false;
    return false;
  }
  g_bias[0] = (float)(sx / n);
  g_bias[1] = (float)(sy / n);
  g_bias[2] = (float)(sz / n);
  vdPrintf("[CAL] bias = %.1f / %.1f / %.1f mg%u 点, I2C 错误 %u)\n",
           g_bias[0], g_bias[1], g_bias[2], n, g_i2cErr - err0);

  /* ---- 阶段B：带内噪声 / 包络 / 接触基线（1.2s，走完整流水线） ---- */
  vdPrintf("[CAL] 阶段B:噪声基线 %u ms\n", (unsigned)CAL_NOISE_MS);
  g_calBandSq[0] = g_calBandSq[1] = g_calBandSq[2] = 0;
  g_calEnvAcc = 0; g_calEnvN = 0;
  g_calCtAcc = 0;  g_calCtN = 0;
  g_calib = true;
  startTimer();
  waitDrain(CAL_NOISE_MS);
  stopTimer();
  g_calib = false;

  if (g_calEnvN < 20) {
    vdPrintf("[CAL] 失败：包络点太少（%u)\n", (unsigned)g_calEnvN);
    g_lastCalOk = false;
    return false;
  }
  double bandN = (double)g_histTotal;
  for (int a = 0; a < 3; a++) g_sigBand[a] = (float)sqrt(g_calBandSq[a] / bandN);
  g_envRest = (float)(g_calEnvAcc / g_calEnvN);
  g_ctRest  = (g_calCtN > 0) ? (float)(g_calCtAcc / g_calCtN) : CT_MIN_MG;

  /* ---- 阈值整定 ---- */
  float smax = g_sigBand[0];
  if (g_sigBand[1] > smax) smax = g_sigBand[1];
  if (g_sigBand[2] > smax) smax = g_sigBand[2];

  g_zcl  = fmaxf(ZCL_MIN_MG, ZCL_K * smax);
  g_thHi = fmaxf(TH_HI_MIN_MG, TH_HI_K * g_envRest);
  g_thLo = fmaxf(TH_LO_MIN_MG, TH_LO_K * g_envRest);
  g_thCt = fmaxf(CT_MIN_MG,   CT_K   * g_ctRest);

  vdPrintf("[CAL] 带内 sigma = %.1f / %.1f / %.1f mg\n", g_sigBand[0], g_sigBand[1], g_sigBand[2]);
  vdPrintf("[CAL] 静置包络 = %.1f mg   静置低频 = %.1f mg\n", g_envRest, g_ctRest);
  vdPrintf("[CAL] 阈值：过零 %.1f mg | 启停 hi %.1f / lo %.1f mg | 接触 %.1f mg\n",
           g_zcl, g_thHi, g_thLo, g_thCt);

  resetPipeline();     // 清掉标定期间的样本与统计
  g_ctLevel = g_ctRest;  // 接触 EMA 从静置水平起步，避免刚启动的 1s 内误报"接触"
  g_lastCalOk = true;
  return true;
}

/* ========================= 串口输出 ========================= */

static uint8_t xorChk(const char* s, int n) {
  uint8_t x = 0;
  for (int i = 0; i < n; i++) x ^= (uint8_t)s[i];
  return x;
}

/* ============================================================
 * ★★★ 比赛协议适配点：若不采用 $VD 帧，只改本函数里的 snprintf ★★★
 * 默认帧（FRAME_WITH_CONTACT=0，与 v1.0 逐字节一致）：
 *   $VD,<seq 4位>,<主频×10 5位>,<幅值mg 5位>,<振动状态1位>,<启停频率×100 3位>,<XOR校验 2位>\r\n
 *   seq=0000 表示"无持续振动"时主频字段为 00000
 * FRAME_WITH_CONTACT=1 时帧尾追加 ,<接触状态>（1=接触 / 0=未接触）
 * ============================================================ */
static void sendFrame(void) {
  char body[88];
  uint32_t f10 = (uint32_t)(g_freqHz * 10.0f + 0.5f);
  uint32_t a   = (uint32_t)(g_ampMg + 0.5f);
  float ss = ssCompute();
  uint32_t s100 = (uint32_t)(ss * 100.0f + 0.5f);

  if (f10 > 99999) f10 = 99999;
  if (a   > 99999) a   = 99999;      // 饱和用 99999 表示
  if (s100 > 999)  s100 = 999;

#if FRAME_WITH_CONTACT
  int n = snprintf(body, sizeof(body), "$VD,%04u,%05lu,%05lu,%u,%03lu,%u",
                   (unsigned)(g_seq % 10000),
                   (unsigned long)f10, (unsigned long)a,
                   (unsigned)(g_vibState ? 1 : 0), (unsigned long)s100,
                   (unsigned)g_contact);
#else
  int n = snprintf(body, sizeof(body), "$VD,%04u,%05lu,%05lu,%u,%03lu",
                   (unsigned)(g_seq % 10000),
                   (unsigned long)f10, (unsigned long)a,
                   (unsigned)(g_vibState ? 1 : 0), (unsigned long)s100);
#endif
  if (n <= 0) return;
  if (n > (int)sizeof(body) - 1) n = sizeof(body) - 1;

  char out[100];
  int m = snprintf(out, sizeof(out), "%s,%02X\r\n", body, xorChk(body, n));
  if (m > 0) vdRaw(out, (size_t)((m > (int)sizeof(out) - 1) ? sizeof(out) - 1 : m));
  g_seq++;
  g_nFrames++;
}

static void sendDebug(void) {
  vdPrintf("#F=%.1f A=%.0f CT=%u SS=%.2f st=%d ax=%u zcl=%.1f hi=%.0f lo=%.0f ct=%.0f drop=%lu i2c=%lu\n",
           g_freqHz, g_ampMg, (unsigned)g_contact, ssCompute(),
           (int)g_vibState, (unsigned)g_domAxis, g_zcl, g_thHi, g_thLo, g_thCt,
           (unsigned long)g_ringDrop, (unsigned long)g_i2cErr);
}

static void sendRaw(void) {
  vdPrintf("#RAW,%04u,%.1f,%.1f,%.1f,E=%.1f\n",
           (unsigned)(g_seq % 10000), g_hist[0][(g_histIdx + BLOCK_N - 1) & (BLOCK_N - 1)],
           g_hist[1][(g_histIdx + BLOCK_N - 1) & (BLOCK_N - 1)],
           g_hist[2][(g_histIdx + BLOCK_N - 1) & (BLOCK_N - 1)], g_envLevel);
}

static void sendStatus(void) {
  vdPrintf("\n===== vibmon v1.1 状态 =====\n");
  vdPrintf(" 采样 %d Hz  I2C %lu Hz  帧率 %d Hz  算法 %s\n",
           FS_HZ, (unsigned long)I2C_CLOCK, 1000 / FRAME_MS,
           (FREQ_ALGO == 1) ? "FFT(桩)" : "简易法(过零+插值+中位数)");
  vdPrintf(" 协议 $VD%s  波特率 %d  TX=GPIO%d RX=GPIO%d  文字输出 %s\n",
           FRAME_WITH_CONTACT ? "(7字段/含接触)" : "(6字段)",
           BAUD, PIN_UART_TX, PIN_UART_RX, SERIAL_VERBOSE ? "开" : "关(只发帧)");
  vdPrintf(" 标定 %s  bias %.1f/%.1f/%.1f mg\n",
           g_lastCalOk ? "OK" : "未完成", g_bias[0], g_bias[1], g_bias[2]);
  vdPrintf(" 阈值 过零 %.1f | 启停 %.0f/%.0f | 接触 %.0f mg\n", g_zcl, g_thHi, g_thLo, g_thCt);
  vdPrintf(" 当前 F=%.1f Hz  A=%.0f mg  状态=%s  接触=%u  主轴=%c\n",
           g_freqHz, g_ampMg, g_vibState ? "振动" : "静止", (unsigned)g_contact, 'X' + g_domAxis);
  vdPrintf(" 统计 起 %lu 次 / 停 %lu 次 / 帧 %lu / 环形溢出 %lu / I2C 错 %lu\n",
           (unsigned long)g_nStart, (unsigned long)g_nStop, (unsigned long)g_nFrames,
           (unsigned long)g_ringDrop, (unsigned long)g_i2cErr);
  vdPrintf("===========================\n\n");
}

static void handleCmd(char c) {
  switch (c) {
    case '?':
      vdPrintf("命令: ? 帮助 | c 重做标定 | z 打印阈值 | S 状态 | d 调试行开关 | R 原始dump开关 | n 清零统计\n");
      break;
    case 'c': calibrateAll(); startTimer(); break;
    case 'z':
      vdPrintf("#阈值 过零 %.1f | 启停 hi %.1f lo %.1f | 接触 %.1f mg | 静置包络 %.1f\n",
               g_zcl, g_thHi, g_thLo, g_thCt, g_envRest);
      break;
    case 'S': sendStatus(); break;
    case 'd': g_dbgOn = !g_dbgOn; vdPrintf("#调试行 %s\n", g_dbgOn ? "开" : "关"); break;
    case 'R': g_rawOn = !g_rawOn; vdPrintf("#原始dump %s\n", g_rawOn ? "开(100Hz)" : "关"); break;
    case 'n': g_nStart = g_nStop = g_nFrames = 0; g_ringDrop = 0; g_i2cErr = 0; vdPrintf("#统计已清零\n"); break;
    default: break;
  }
}

/* ========================= setup / loop ========================= */

static void printBanner(void) {
  vdPrintf("\n\n");
  vdPrintf("==================================================\n");
  vdPrintf(" 柔性振动检测模块 · 振动检测主程序 (vibmon v1.1)\n");
  vdPrintf(" 目标: ESP32-C3 / arduino-esp32 %d.%d.%d\n",
           ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  vdPrintf(" 网表: Netlist_PCB1_2026-09-24.tel\n");
  vdPrintf(" 采样 %d Hz | 窗 %d 点(%.0fms) | 步 %d 点 | I2C %lu Hz\n",
           FS_HZ, BLOCK_N, 1000.0f * BLOCK_N / FS_HZ, HOP, (unsigned long)I2C_CLOCK);
  vdPrintf(" 高通 %.0f Hz(Q=%.2f) | 包络窗 %d ms | 帧率 %d Hz @ %d bps\n",
           HP_FC_HZ, HP_Q, ENV_WIN * 1000 / FS_HZ, 1000 / FRAME_MS, BAUD);
  vdPrintf(" 串口: TX=GPIO%d RX=GPIO%d(4PIN ①/②）, UART_SWAP=%d\n",
           PIN_UART_TX, PIN_UART_RX, UART_SWAP);
  vdPrintf(" LED: 绿=GPIO%d(1Hz心跳) 黄=GPIO%d(振动/故障) 红=硬件常亮; ACTIVE_LOW=%d\n",
           PIN_LED_G, PIN_LED_Y, LED_ACTIVE_LOW);
  vdPrintf(" 约束: 已禁用 Wi-Fi/BLE(防 AMS1117 热关断）\n");
  vdPrintf("==================================================\n");
}

void setup() {
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_Y, OUTPUT);
  ledWrite(PIN_LED_G, 0);
  ledWrite(PIN_LED_Y, 0);

  /* [M7] 不再对 UART 引脚调 pinMode —— ESP32 上 UART 走 GPIO 矩阵，
     由 UART 驱动自行接管引脚；手动 pinMode 会与驱动抢配置。 */
  Comp.begin(BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
#if MIRROR_USB
  Serial.begin(BAUD);
#endif
  delay(300);
  printBanner();

  hpInit();
  if (!mpuInit()) {
    vdPrintf("[HALT] 传感器不通，停止。请先烧 selftest 定位硬件问题。\n");
    /* [M4] 故障指示：绿灯灭 + 黄灯 2Hz 快闪。
       绝不能停在这里还让绿灯闪——评委/平台会把"绿灯闪"直接读成"程序正常运行"。 */
    ledWrite(PIN_LED_G, 0);
    while (true) {
      ledWrite(PIN_LED_Y, 1); delay(250);
      ledWrite(PIN_LED_Y, 0); delay(250);
    }
  }

  if (!calibrateAll()) {
    vdPrintf("[WARN] 标定未完成，先用保守默认阈值运行；可输 'c' 重做。\n");
  }
  ssReset();
  startTimer();
  vdPrintf("[OK] 开始采样。命令 '?' 看交互。\n\n");
}

void loop() {
  /* 1) 采样流水线：把环形缓冲吃干，算法全在这里跑 */
  drainPipeline();

  /* 2) 绿 LED 心跳 1Hz（证明固件在跑）；黄 LED = 振动中指示 */
  static uint32_t tLed = 0;
  static int ledOn = 0;
  if (millis() - tLed >= 500) {
    tLed = millis();
    ledOn = !ledOn;
    ledWrite(PIN_LED_G, ledOn);
  }
  ledWrite(PIN_LED_Y, (g_vibState == 1) ? 1 : 0);

  /* 3) 串口命令 */
  while (Comp.available()) handleCmd((char)Comp.read());

  /* 4) 输出：50Hz 帧 + 5Hz 调试行 + 可选 100Hz 原始 dump
        （都从 loop 发，永远不阻塞采样 —— 采样在定时器回调里） */
  uint32_t now = millis();
  static uint32_t tFrame = 0, tDbg = 0, tRaw = 0;
  if (now - tFrame >= FRAME_MS) { tFrame = now; sendFrame(); }
  if (g_dbgOn && now - tDbg >= DBG_MS) { tDbg = now; sendDebug(); }
  if (g_rawOn && now - tRaw >= RAW_MS) { tRaw = now; sendRaw(); }
}

