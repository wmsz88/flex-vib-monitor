// 信号处理模块实现。
//
// 【数据流总览】
//   Sample.raw(LSB)
//      │  × k (1000/LSB_PER_G)  → 转成 mg
//      │  − s_cal.bias[]        → 去零偏（此时静止读数≈重力矢量）
//      ├──────────────► 姿态支路：一阶低通(0.2s) → 模长 → ctDev
//      └──────────────► 振动支路：二阶高通(70Hz) → f[]
//                            ├─► 平方和 → 滑动窗平均 → sqrt → env（每 10 样本出一次）
//                            └─► 存入 s_hist[3][128] → 每 64 样本跑一次窗 → freqHz
//
// 【为什么先去零偏再高通】顺序不能反。高通本身就能滤掉重力，但零偏里包含
// 传感器自身的固定偏移，若不先减掉，它会一直叠加在信号上，抬高包络基线。
#include "dsp.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

/* ---------- 标定结果 ---------- */
static DspCal s_cal = {};          // 当前生效的标定。上电初值全 0，即"零偏为 0、阈值走默认下限"
static bool   s_calMode = false;   // 标定模式标志。为 true 时 dspProcess 只累加统计量，
                                   // 不输出包络/主频（避免标定期间的中间结果被 state 当成真实状态）

/* ---------- 高通（二阶 Butterworth，DF2T） ---------- */
// 系数在 hpInit() 里按 HP_FC_HZ/HP_Q 算一次，运行期不再变。
static float s_b0, s_b1, s_b2, s_a1, s_a2;   // 归一化后的传递函数系数（a0 已除到 1）
static float s_z1[3], s_z2[3];                // 三轴各自的状态变量（DF2T 结构只需两个状态量）
                                              // 用数组按轴索引，避免复制三份代码

/* ---------- 主频窗 ---------- */
// 只保存"连续"样本，s_histFill 在不连续时清零。
// s_hist 是环形缓冲：s_histIdx 指向"最老"的样本，读取时从它开始绕一圈即为时间正序。
static float    s_hist[3][BLOCK_N];   // 三轴的高通后数据，各 BLOCK_N 个
static uint16_t s_histIdx = 0;        // 环形写指针（同时也是最老样本的位置）
static uint16_t s_histFill = 0;       // 已填入的有效样本数，满 BLOCK_N 才允许出主频
static uint16_t s_hopCnt = 0;         // 距上次出主频已过的样本数，到 HOP 触发一次
static uint16_t s_clipCnt = 0;        // 本窗内累计的削顶样本数，出窗时上报并清零

/* ---------- RMS 包络 ---------- */
// 用滑动窗求和（加新值减旧值）实现 O(1) 更新，而不是每次重算 30 个平方和。
static float   s_envBuf[ENV_WIN];     // 窗内各样本的"三轴平方和"，被减出的旧值来自这里
static uint8_t s_envIdx = 0;          // 环形写指针
static uint8_t s_envFill = 0;         // 已填样本数，满 ENV_WIN 才开始出包络
static float   s_envSum = 0.0f;       // 窗内平方和（滑动维护）
static uint8_t s_envHop = 0;          // 距上次出包络已过的样本数，到 ENV_HOP 输出一次
static float   s_env = 0.0f;          // 最近一次算出的包络值（mg），也是标定阶段的累加源

/* ---------- 姿态支路 ---------- */
// 每轴一个一阶低通状态。三轴各自滤波，最后取模长——不能先取模长再滤波，
// 因为模长是非线性运算，先取模长会把振动分量整流成直流偏置，污染姿态读数。
static float s_lp[3];

/* ---------- 标定累加器 ---------- */
// 标定期间持续累加，结束时一次算出所有基准值。
static double   s_calBandSq[3];       // Σf²（高通后），用于算带内噪声 sigma
static uint32_t s_calBandN;           // Σ 的样本数
static double   s_calEnv, s_calCt;    // Σenv、ΣctDev
static uint32_t s_calEnvN;            // 包络样本数（比 s_calBandN 小，因为包络 10 样本才出一个）

// 计算二阶高通（巴特沃斯）的系数。用双线性变换（预畸变）从模拟原型映射到数字域。
// 推导：模拟二阶高通的极点由截止频率 w0 与品质因数 Q 决定；
// 令 alpha = sin(w0)/(2Q)，a0 = 1+alpha，则归一化系数为
//   b0 = (1+cos w0)/2 / a0,  b1 = -(1+cos w0)/a0,  b2 = b0
//   a1 = -2cos w0 / a0,     a2 = (1-alpha)/a0
// Q=0.7071 时通带最平坦（无过冲），这也是 HP_Q 取该值的原因。
static void hpInit(void) {
  float w0 = 2.0f * PI * HP_FC_HZ / (float)FS_HZ;   // 数字角频率
  float cw = cosf(w0), sw = sinf(w0);
  float alpha = sw / (2.0f * HP_Q);
  float a0 = 1.0f + alpha;
  s_b0 = (1.0f + cw) * 0.5f / a0;
  s_b1 = -(1.0f + cw) / a0;
  s_b2 = s_b0;                                      // 高通分子对称
  s_a1 = -2.0f * cw / a0;
  s_a2 = (1.0f - alpha) / a0;
}

// 清空滤波器历史。采样断档或数值异常后必须调用：旧的 s_z1/s_z2 对应的是断档前的信号，
// 继续用会让输出在恢复瞬间产生一段虚假的暂态。
static void hpReset(void) {
  for (int i = 0; i < 3; i++) { s_z1[i] = 0; s_z2[i] = 0; }
}

// 单轴推进一步（Direct Form II Transposed，每样本 5 乘 4 加）。
// 选 DF2T 而不是 DF1：状态变量少一半，且定点/浮点下的数值精度更好。
// y = b0*x + z1 ; z1 = b1*x - a1*y + z2 ; z2 = b2*x - a2*y
static inline float hpStep(int a, float x) {
  float y = s_b0 * x + s_z1[a];
  s_z1[a] = s_b1 * x - s_a1 * y + s_z2[a];
  s_z2[a] = s_b2 * x - s_a2 * y;
  return y;
}

// 重置包络滑窗。
static void envReset(void) {
  memset(s_envBuf, 0, sizeof(s_envBuf));   // 缓冲内容必须与累计和一致
                                           // 否则残留的旧值会在下一次"减旧值"时被减掉，导致 envSum 变成负数
  s_envIdx = 0; s_envFill = 0; s_envSum = 0.0f; s_envHop = 0; s_env = 0.0f;
}

// 重置主频窗的填充进度与计数（不动 s_hist 内容本身，因为 fill 已归零，
// 未填满前不会去读那些旧数据）。
static void winReset(void) {
  s_histFill = 0; s_hopCnt = 0; s_clipCnt = 0;
}

// 上电初始化：算一次滤波器系数，再清空所有运行状态。
void dspInit(void) {
  hpInit();
  dspReset();
}

// 清空全部运行状态（不含标定结果 s_cal）。
// 运行中掉线重连、标定前后、以及检测到数值异常时都会调用。
void dspReset(void) {
  hpReset();
  envReset();
  memset(s_hist, 0, sizeof(s_hist));
  s_histIdx = 0;
  winReset();
  for (int i = 0; i < 3; i++) s_lp[i] = 0.0f;
}

// 采样出现不连续时的处理。
//   missed 较小（≤ SMP_BIG_GAP）：只放弃当前主频窗——窗内样本时间轴已经不连续，
//     用它算频率会得到错误结果；但滤波器和包络只依赖"最近若干个样本"，影响有限，
//     保留它们可以避免每次轻微丢样都产生一段暂态。
//   missed 较大：说明采集断档了一段时间，滤波器和包络的连续性都已被破坏，整体重置。
void dspOnGap(uint32_t missed) {
  winReset();
  if (missed > SMP_BIG_GAP) { hpReset(); envReset(); }
}

// 就地插入排序求中位数（n 很小，且必须就地修改传入数组——调用方已明确接受这一点）。
// 用中位数而非平均值：主频/启停周期的统计里，偶尔一个异常值会把平均值带偏，
// 中位数对异常值不敏感。
static float medianOf(float* a, int n) {
  for (int i = 1; i < n; i++) {
    float v = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
  return (n & 1) ? a[n / 2] : 0.5f * (a[n / 2 - 1] + a[n / 2]);   // 偶数个取中间两个的平均
}

/* 滞回过零 + 线性插值 + 周期中位数。x 已去均值。返回 0 = 无有效主频 */
// 【算法推演】为什么不用 FFT：
//   128 点 FFT 频率分辨率只有 1000/128 ≈ 7.8Hz，而要求的分辨精度是 1Hz 级；
//   且本信号是单频主导，过零法在信噪比足够时精度远高于同长度 FFT，计算量也小得多。
// 【为什么要"滞回"】单纯判 x 过零，噪声会让信号在零点附近反复穿越，一次振动被数成几十个周期。
//   这里要求"先低于 -zc，之后才认高于 +zc 的穿越"，形成一段迟滞带，噪声无法单独触发。
// 【为什么要"线性插值"】只在样本点上判过零，量化误差最大可达半个采样周期（500us），
//   对 180Hz（5.5ms 周期）就是近 10% 误差。用相邻两点连线与阈值的交点作为穿越时刻，
//   把时间精度提升到远小于采样周期。
// 【流程】
//   ① 扫一遍窗内数据，用滞回找出所有"上升穿越"时刻 tUp[]（用插值精确定位）；
//   ② 相邻穿越时刻相减得到各周期长度，只保留落在 [tMin, tMax] 范围内的；
//   ③ 周期数不足 MIN_PERIODS 则判无效；
//   ④ 取周期中位数 → 频率 = FS / 中位数，再做一次范围与有限性校验。
static float estimateFreq(const float* x, int N, float zc) {
  // 把频率范围换算成周期范围（单位：样本数）
  const float tMin = (float)FS_HZ / FREQ_HI_HZ;   // 220Hz → 4.5 样本
  const float tMax = (float)FS_HZ / FREQ_LO_HZ;   // 60Hz  → 16.7 样本
  float tUp[BLOCK_N / 2];                          // 上升穿越时刻表；最坏情况每 2 样本一次穿越
  int nUp = 0, armed = 0;                          // armed：是否已进入"等待上升穿越"的状态
  for (int i = 1; i < N; i++) {
    if (!armed) {
      if (x[i] < -zc) armed = 1;                   // 先跌破负阈值，武装
    } else if (x[i] > zc) {                        // 再升破正阈值，确认一次有效上升穿越
      // 在 (i-1, i) 之间线性插值求穿越点：t = (i-1) + (zc - x[i-1]) / (x[i]-x[i-1])
      float d = x[i] - x[i - 1];
      if (d > 1e-9f && nUp < BLOCK_N / 2) tUp[nUp++] = (float)(i - 1) + (zc - x[i - 1]) / d;
      armed = 0;
    }
  }
  if (nUp < 2) return 0.0f;                        // 少于两个穿越点，连一个周期都构不成

  float T[BLOCK_N / 2];                            // 有效周期长度表
  int nT = 0;
  for (int k = 1; k < nUp; k++) {
    float d = tUp[k] - tUp[k - 1];
    if (d >= tMin && d <= tMax) T[nT++] = d;       // 落在有效频段外的周期直接丢弃
  }
  if (nT < MIN_PERIODS) return 0.0f;               // 周期数太少，中位数不可信
  float tmed = medianOf(T, nT);
  if (!(tmed > 0.0f)) return 0.0f;                 // 防御除零（含 NaN 比较为 false 的情况）
  float f = (float)FS_HZ / tmed;
  if (!isfinite(f) || f < FREQ_LO_HZ || f > FREQ_HI_HZ) return 0.0f;
  return f;
}

// 跑一个完整的主频窗。
// 【流程】
//   ① 从环形缓冲按时间正序取出三轴数据，同时累加各轴的 Σx 与 Σx²；
//   ② 用方差公式 var = E[x²] − (E[x])² 比较三轴，选出方差最大的作为主轴
//      （振动主方向信噪比最好，避免在几乎不动的轴上做无意义的过零统计）；
//   ③ 只把主轴数据去均值后拷进临时数组 win[]——estimateFreq 需要连续内存，
//      而环形缓冲是绕圈的，必须展平；
//   ④ 用标定得到的过零阈值 s_cal.zcl 估计主频，并上报本窗削顶数。
// 注意 out->winReady 无条件置 true：即使 freqHz 为 0，也代表"这一窗已经跑完、
// 结果是无效"这个事实，state 模块据此刷新时效计时。
static void runWindow(DspFrame* out) {
  float win[BLOCK_N];                 // 主轴数据的连续副本（展平后的窗）
  double sum[3] = {0, 0, 0}, sq[3] = {0, 0, 0};
  for (int i = 0; i < BLOCK_N; i++) {
    uint16_t idx = (uint16_t)((s_histIdx + i) & (BLOCK_N - 1));   // 从最老样本开始绕一圈
    for (int a = 0; a < 3; a++) {
      double v = s_hist[a][idx];
      sum[a] += v; sq[a] += v * v;
    }
  }
  // 选主轴：方差最大者
  int ax = 0;
  float var[3];
  for (int a = 0; a < 3; a++) {
    double m = sum[a] / BLOCK_N;
    var[a] = (float)(sq[a] / BLOCK_N - m * m);    // 用 double 累加再转 float，减少长窗累加误差
  }
  if (var[1] > var[ax]) ax = 1;
  if (var[2] > var[ax]) ax = 2;

  // 去均值：过零法要求信号关于 0 对称，否则过零点会系统性偏移，频率随之出错
  float m = (float)(sum[ax] / BLOCK_N);
  for (int i = 0; i < BLOCK_N; i++)
    win[i] = s_hist[ax][(s_histIdx + i) & (BLOCK_N - 1)] - m;

  out->winReady = true;
  out->freqHz = estimateFreq(win, BLOCK_N, s_cal.zcl);
  out->axis = (uint8_t)ax;
  out->winClip = s_clipCnt;           // 上报本窗削顶数（该窗内累加，跨窗不累计）
  s_clipCnt = 0;
}

// 单样本主处理链——全工程调用最频繁的函数（1kHz）。
// 【流程推演】按执行顺序：
//   ① 复位本次的输出标志（调用方复用同一个 DspFrame，必须显式清）
//   ② 削顶检测：原始值接近 int16 满量程即为削顶，计入本窗计数与全局统计
//   ③ 单位换算 + 去零偏 → d[3]（mg）
//   ④ 姿态支路：一阶低通 d → s_lp，取模长得 ctDev
//      一阶低通系数 aLp = 1/(FS×τ)；τ=0.2s 时 aLp≈0.005，即每样本只吸收 0.5% 的新值，
//      时间常数远大于振动周期，因此振动被平均掉，输出跟随的是重力方向
//   ⑤ 振动支路：二阶高通 d → f[3]
//   ⑥ 数值守卫：滤波输出出现 inf/NaN 说明状态已被污染（多为断档或极端值所致），
//      整体重置后放弃本样本，避免污染扩散到后续所有样本
//   ⑦ 写入主频窗环形缓冲，更新 fill 计数
//   ⑧ 包络滑窗：加新减旧得到窗内平方和；每 ENV_WIN 个样本重算一次总和，
//      消除浮点加减的累积漂移（长期运行时这个漂移会让基线缓慢偏移）
//   ⑨ 每 ENV_HOP 个样本输出一次包络（env 与 ctDev 同时输出，两者节奏一致）
//   ⑩ 标定模式：只累加统计量后提前返回，不输出结果
//   ⑪ 每 HOP 个样本跑一次主频窗（前提是窗已填满）
void dspProcess(const Sample& s, DspFrame* out) {
  out->envReady = false;
  out->winReady = false;
  out->fx = out->fy = out->fz = 0.0f;

  // ② 削顶检测：在原始域判断，不受量程换算影响
  const float k = 1000.0f / mpuLsbPerG();   // LSB → mg 的换算系数（±2g 时为 1000/16384 ≈ 0.061）
  if (abs(s.raw.x) > ACCEL_CLIP_RAW || abs(s.raw.y) > ACCEL_CLIP_RAW || abs(s.raw.z) > ACCEL_CLIP_RAW) {
    s_clipCnt++;
    samplerAddClip();
  }
  // ③ 换算 + 去零偏
  float d[3] = {
    s.raw.x * k - s_cal.bias[0],
    s.raw.y * k - s_cal.bias[1],
    s.raw.z * k - s_cal.bias[2],
  };

  /* 姿态支路：低通保留重力方向变化，去掉振动分量 */
  const float aLp = 1.0f / ((float)FS_HZ * CT_LPF_TAU_S);   // 一阶 IIR 系数
  for (int a = 0; a < 3; a++) s_lp[a] += (d[a] - s_lp[a]) * aLp;
  out->ctDev = sqrtf(s_lp[0] * s_lp[0] + s_lp[1] * s_lp[1] + s_lp[2] * s_lp[2]);

  /* 振动支路 */
  float f[3];
  for (int a = 0; a < 3; a++) f[a] = hpStep(a, d[a]);
  // ⑥ 数值守卫
  if (!isfinite(f[0]) || !isfinite(f[1]) || !isfinite(f[2])) {
    dspReset();
    return;
  }
  out->fx = f[0]; out->fy = f[1]; out->fz = f[2];

  // ⑦ 写入主频窗
  for (int a = 0; a < 3; a++) s_hist[a][s_histIdx] = f[a];
  s_histIdx = (uint16_t)((s_histIdx + 1) & (BLOCK_N - 1));
  if (s_histFill < BLOCK_N) s_histFill++;

  // ⑧ 包络滑窗：平方和用"加新值、减旧值"的方式 O(1) 维护
  float msq = f[0] * f[0] + f[1] * f[1] + f[2] * f[2];   // 三轴平方和（正比于瞬时能量）
  s_envSum += msq - s_envBuf[s_envIdx];
  s_envBuf[s_envIdx] = msq;
  if (++s_envIdx >= ENV_WIN) {
    s_envIdx = 0;
    // 每轮重新求和，消除浮点加减的累积误差
    // 滑动求和跑几千轮后，float 的舍入误差会累积成可见的基线漂移，必须周期性重算归零
    float acc = 0.0f;
    for (int i = 0; i < ENV_WIN; i++) acc += s_envBuf[i];
    s_envSum = acc;
  }
  if (s_envFill < ENV_WIN) s_envFill++;

  // ⑨ 每 ENV_HOP 个样本输出一次包络（100Hz）
  if (++s_envHop >= ENV_HOP) {
    s_envHop = 0;
    if (s_envFill >= ENV_WIN) {                          // 窗未满时不输出，避免用不足的数据算出偏小的包络
      s_env = sqrtf(fmaxf(s_envSum, 0.0f) / (float)ENV_WIN);   // fmaxf 防御浮点误差导致的微负值
      out->envReady = true;
      out->env = s_env;
      // 标定阶段顺便累加包络与姿态偏差
      if (s_calMode) { s_calEnv += s_env; s_calCt += out->ctDev; s_calEnvN++; }
    }
  }

  // ⑩ 标定模式：只累加带内噪声，不产出结果
  if (s_calMode) {
    for (int a = 0; a < 3; a++) s_calBandSq[a] += (double)f[a] * f[a];
    s_calBandN++;
    return;
  }

  // ⑪ 主频窗：每 HOP 个样本触发一次；必须等窗填满才跑
  if (++s_hopCnt >= HOP) {
    s_hopCnt = 0;
    if (s_histFill >= BLOCK_N) runWindow(out);
  }
}

/* ---------------- 标定 ---------------- */
// 标定分两个阶段，顺序不能颠倒：
//   阶段A（calBias）：停掉采样器，主循环直接读传感器 2s，求三轴均值（=零偏/重力矢量）
//                     与标准差（=标定质量）。必须先有零偏，阶段B 的处理链才有意义。
//   阶段B：把零偏写进 s_cal.bias，启动采样器跑完整处理链 1.2s，测量"静置时的真实表现"
//         ——带内噪声 sigma、静置包络、静置姿态偏差，再据此算出各阈值。
// 【为什么阈值要现场测而不是写死】不同板子的噪声底、安装角度差异很大。用实测值乘以
// 倍数（TH_HI_K 等）得到阈值，可以让同一份固件在不同板子上都工作。
//
// 阶段A：停采样器后直接读 2s 原始数据，算均值与标准差。
// 用 double 累加：2s 内有约 2000 个样本，float 累加 16384 量级的值会有可见误差。
static bool calBias(void) {
  double sx = 0, sy = 0, sz = 0, qx = 0, qy = 0, qz = 0;   // 各轴 Σx 与 Σx²
  uint32_t n = 0;                                          // 成功样本数
  const float k = 1000.0f / mpuLsbPerG();
  uint32_t t0 = millis();
  while (millis() - t0 < CAL_BIAS_MS) {
    MpuRaw r;
    if (mpuReadAccel(&r)) {                                // 读失败直接跳过，不计入 n
      double x = r.x * k, y = r.y * k, z = r.z * k;
      sx += x; sy += y; sz += z;
      qx += x * x; qy += y * y; qz += z * z;
      n++;
    }
    // 每 32 个样本让出一次 CPU。这段是忙等循环，完全不让出会触发看门狗复位
    if ((n & 31u) == 0u) delay(1);
  }
  s_cal.nBias = n;
  if (n < 200) return false;                               // 样本太少（I2C 大面积失败），标定无意义
  double m[3] = {sx / n, sy / n, sz / n};                  // 均值 = 零偏
  double q[3] = {qx / n, qy / n, qz / n};
  for (int a = 0; a < 3; a++) {
    s_cal.bias[a] = (float)m[a];
    // 标准差 = sqrt(E[x²] − (E[x])²)。内层 fmax 防御浮点误差导致的微负值
    s_cal.rawStd[a] = (float)sqrt(fmax(q[a] - m[a] * m[a], 0.0));
  }
  // 零偏矢量模长。静置时该矢量就是重力，模长应≈1000mg，是 quality 判据之一
  s_cal.biasNormMg = sqrtf(s_cal.bias[0] * s_cal.bias[0] + s_cal.bias[1] * s_cal.bias[1] +
                           s_cal.bias[2] * s_cal.bias[2]);
  return true;
}

// 完整标定流程。返回 true 表示流程跑完（不代表质量合格，质量看 s_cal.quality）。
// 【副作用】调用前后采样器状态：进入时先停止，返回时仍处于停止状态（由调用方决定何时重启）。
// 【流程推演】
//   ① 停采样器并复位，清掉旧数据；把 ok/quality 先置 false（中途失败就不会留下"半标定"状态）
//   ② 阶段A：求零偏与标准差；失败（样本不足）直接返回 false
//   ③ 清空阶段B 累加器，dspReset() 让滤波器从零状态开始（否则残留状态会污染噪声测量）
//   ④ 打开 s_calMode，启动采样器，跑 CAL_NOISE_MS，只做累加不做判断
//   ⑤ 停采样器，关闭 s_calMode
//   ⑥ 若包络样本或带内样本不足 → 失败返回
//   ⑦ 由累加值算出：带内噪声 sigma（各轴）、静置包络、静置姿态偏差、过零阈值
//   ⑧ 判定 quality：标准差与零偏模长都在容差内才说明"标定时确实静置"
//   ⑨ 置 ok=true，再次复位采样器与 dsp，保证返回后处于干净状态
bool dspCalibrate(void) {
  samplerStop();
  samplerReset();
  s_cal.ok = false;
  s_cal.quality = false;

  if (!calBias()) return false;                    // ② 阶段A

  // ③ 清空阶段B 累加器
  for (int a = 0; a < 3; a++) s_calBandSq[a] = 0;
  s_calBandN = 0; s_calEnv = 0; s_calCt = 0; s_calEnvN = 0;
  dspReset();
  s_calMode = true;                                // ④ 进入标定模式
  samplerStart();
  uint32_t t0 = millis();
  while (millis() - t0 < CAL_NOISE_MS) {
    // 这段循环把采样器产出的样本全部喂给 dspProcess（内部只累加不输出），
    // delay(1) 让出 CPU，同时也给采样定时器回调留出运行机会
    Sample s; uint32_t gap; DspFrame fr;
    while (samplerPop(&s, &gap)) {
      if (gap) dspOnGap(gap);
      dspProcess(s, &fr);
    }
    delay(1);
  }
  samplerStop();                                   // ⑤
  s_calMode = false;

  // ⑥ 数据量校验：包络至少 20 个点（1.2s × 100Hz = 120 个，留足余量），
  //    带内样本不能为 0（否则下面要除以 0）
  if (s_calEnvN < 20 || s_calBandN == 0) return false;

  // ⑦ 算各基准值
  float smax = 0.0f;                               // 三轴中最大的噪声 sigma，用于统一过零阈值
  for (int a = 0; a < 3; a++) {
    s_cal.sigBand[a] = (float)sqrt(s_calBandSq[a] / s_calBandN);   // RMS → sigma
    smax = fmaxf(smax, s_cal.sigBand[a]);
  }
  s_cal.envRest = (float)(s_calEnv / s_calEnvN);   // 静置包络均值
  s_cal.ctRest  = (float)(s_calCt / s_calEnvN);    // 静置姿态偏差均值
  // 过零阈值取三轴最大值：任一轴的噪声都不该触发过零
  s_cal.zcl = fmaxf(ZCL_MIN_MG, ZCL_K * smax);

  // ⑧ quality 判据：标定期间标准差不能太大（说明被碰过），
  //    且零偏模长必须接近 1g（说明模块放平且静止）。两者同时满足才算"确实静置"
  float stdMax = fmaxf(s_cal.rawStd[0], fmaxf(s_cal.rawStd[1], s_cal.rawStd[2]));
  s_cal.quality = stdMax <= CAL_MAX_STD_MG && fabsf(s_cal.biasNormMg - 1000.0f) <= CAL_G_TOL_MG;
  s_cal.ok = true;

  // ⑨ 复位，返回干净状态
  samplerReset();
  dspReset();
  return true;
}

// 只读访问当前标定结果。
const DspCal& dspCal(void) { return s_cal; }
