#include "dsp.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

static DspCal s_cal = {};        // 生效参数：全流程只有提交点会写它
static DspCal s_cand = {};       // 本次候选：阶段A/阶段B 都只写这里
static bool   s_useCand = false; // 处理链用哪一份
static bool   s_calMode = false; // 阶段B 统计模式（不出窗、只累加）

static DspCalReport s_report = {};   // 本次尝试的诊断证据

/* 削顶位图：与各自的环形索引同步滚动，天然表示"最近 N 个样本"。
   主频三轴分开存，因为主轴才决定主频可信度；包络只要"任轴"即可。
   RAM: 3×16 + 4 = 52 字节。 */
static uint8_t s_clipBits[3][BLOCK_N / 8];
static uint8_t s_envClipBits[(ENV_WIN + 7) / 8];

static uint16_t popcountBits(const uint8_t* bits, uint16_t nBytes) {
  uint16_t n = 0;
  for (uint16_t i = 0; i < nBytes; i++) n += (uint16_t)__builtin_popcount((unsigned)bits[i]);
  return n;
}

// 主轴上的削顶数（真窗口计数）
static uint16_t clipInWinAxis(int a) { return popcountBits(s_clipBits[a], BLOCK_N / 8); }
// 包络窗内是否有任意轴削顶
static bool envWindowClipped(void) { return popcountBits(s_envClipBits, (ENV_WIN + 7) / 8) > 0; }

static ClipLevel clipLevelOf(uint16_t n) {
  if (n == 0) return CLIP_NONE;
  const float ratio = (float)n / (float)BLOCK_N;
  return (ratio >= CLIP_SEVERE_RATIO) ? CLIP_SEVERE : CLIP_MILD;
}

// 处理链统一入口：标定阶段用候选，平时用生效参数。
// 这样"阶段B 用候选偏置"不需要在多处替换，也不会有第二个"忘了改"的地方。
static inline const DspCal& activeCal(void) { return s_useCand ? s_cand : s_cal; }

/* 高通（二阶 Butterworth，DF2T） */
static float s_b0, s_b1, s_b2, s_a1, s_a2;
static float s_z1[3], s_z2[3];

/* 主频窗：只保存"连续"样本，s_histFill 在不连续时清零 */
static float    s_hist[3][BLOCK_N];
static uint16_t s_histIdx = 0;
static uint16_t s_histFill = 0;
static uint16_t s_hopCnt = 0;

/* RMS 包络 */
static float   s_envBuf[ENV_WIN];
static uint8_t s_envIdx = 0;
static uint8_t s_envFill = 0;
static float   s_envSum = 0.0f;
static uint8_t s_envHop = 0;
static float   s_env = 0.0f;

/* 姿态支路：每轴低通后相对零偏的偏差 */
static float s_lp[3];

/* 标定累加 */
static double   s_calBandSq[3];
static uint32_t s_calBandN;
static double   s_calEnv, s_calCt;
static uint32_t s_calEnvN;

static void hpInit(void) {
  float w0 = 2.0f * PI * HP_FC_HZ / (float)FS_HZ;
  float cw = cosf(w0), sw = sinf(w0);
  float alpha = sw / (2.0f * HP_Q);
  float a0 = 1.0f + alpha;
  s_b0 = (1.0f + cw) * 0.5f / a0;
  s_b1 = -(1.0f + cw) / a0;
  s_b2 = s_b0;
  s_a1 = -2.0f * cw / a0;
  s_a2 = (1.0f - alpha) / a0;
}

static void hpReset(void) {
  for (int i = 0; i < 3; i++) { s_z1[i] = 0; s_z2[i] = 0; }
}

static inline float hpStep(int a, float x) {
  float y = s_b0 * x + s_z1[a];
  s_z1[a] = s_b1 * x - s_a1 * y + s_z2[a];
  s_z2[a] = s_b2 * x - s_a2 * y;
  return y;
}

static void envReset(void) {
  memset(s_envBuf, 0, sizeof(s_envBuf));   // 缓冲内容必须与累计和一致
  memset(s_envClipBits, 0, sizeof(s_envClipBits));
  s_envIdx = 0; s_envFill = 0; s_envSum = 0.0f; s_envHop = 0; s_env = 0.0f;
}

static void winReset(void) {
  s_histFill = 0; s_hopCnt = 0;
  // 位图必须一起清：缺口后窗口重新攒，旧削顶记录不再属于"最近 BLOCK_N 个样本"
  memset(s_clipBits, 0, sizeof(s_clipBits));
}

void dspInit(void) {
  hpInit();
  dspReset();
}

void dspReset(void) {
  hpReset();
  envReset();
  memset(s_hist, 0, sizeof(s_hist));
  s_histIdx = 0;
  winReset();
  for (int i = 0; i < 3; i++) s_lp[i] = 0.0f;
}

void dspOnGap(const SamplerGap& gap) {
  if (gap.kind == SMP_GAP_NONE) return;
  // 任何缺口都做完整重置，理由：
  //   1. 主频估计假设等间隔采样，JITTER 同样破坏该假设；
  //   2. 包络是 ENV_WIN 样本滑动窗，不清会得到"缺口前+缺口后"的混合窗；
  //   3. 高通是 IIR，状态里记着缺口前的波形，不清则缺口后前 ~15ms 是暂态。
  // 代价：丢一个主频窗(128ms)+包络窗(30ms)。缺口本身说明采样不稳，宁可慢也要准。
  winReset();
  hpReset();
  envReset();
}

static float medianOf(float* a, int n) {
  for (int i = 1; i < n; i++) {
    float v = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
  return (n & 1) ? a[n / 2] : 0.5f * (a[n / 2 - 1] + a[n / 2]);
}

/* 滞回过零 + 线性插值 + 周期中位数。x 已去均值。返回 0 = 无有效主频 */
static float estimateFreq(const float* x, int N, float zc) {
  const float tMin = (float)FS_HZ / FREQ_HI_HZ;
  const float tMax = (float)FS_HZ / FREQ_LO_HZ;
  float tUp[BLOCK_N / 2];
  int nUp = 0, armed = 0;
  for (int i = 1; i < N; i++) {
    if (!armed) {
      if (x[i] < -zc) armed = 1;
    } else if (x[i] > zc) {
      float d = x[i] - x[i - 1];
      if (d > 1e-9f && nUp < BLOCK_N / 2) tUp[nUp++] = (float)(i - 1) + (zc - x[i - 1]) / d;
      armed = 0;
    }
  }
  if (nUp < 2) return 0.0f;

  float T[BLOCK_N / 2];
  int nT = 0;
  for (int k = 1; k < nUp; k++) {
    float d = tUp[k] - tUp[k - 1];
    if (d >= tMin && d <= tMax) T[nT++] = d;
  }
  if (nT < MIN_PERIODS) return 0.0f;
  float tmed = medianOf(T, nT);
  if (!(tmed > 0.0f)) return 0.0f;
  float f = (float)FS_HZ / tmed;
  if (!isfinite(f) || f < FREQ_LO_HZ || f > FREQ_HI_HZ) return 0.0f;
  return f;
}

static void runWindow(DspFrame* out) {
  float win[BLOCK_N];
  double sum[3] = {0, 0, 0}, sq[3] = {0, 0, 0};
  for (int i = 0; i < BLOCK_N; i++) {
    uint16_t idx = (uint16_t)((s_histIdx + i) & (BLOCK_N - 1));
    for (int a = 0; a < 3; a++) {
      double v = s_hist[a][idx];
      sum[a] += v; sq[a] += v * v;
    }
  }
  int ax = 0;
  float var[3];
  for (int a = 0; a < 3; a++) {
    double m = sum[a] / BLOCK_N;
    var[a] = (float)(sq[a] / BLOCK_N - m * m);
  }
  if (var[1] > var[ax]) ax = 1;
  if (var[2] > var[ax]) ax = 2;

  float m = (float)(sum[ax] / BLOCK_N);
  for (int i = 0; i < BLOCK_N; i++)
    win[i] = s_hist[ax][(s_histIdx + i) & (BLOCK_N - 1)] - m;

  out->winReady = true;
  out->freqHz = estimateFreq(win, BLOCK_N, activeCal().zcl);
  out->axis = (uint8_t)ax;
  // 只统计主轴：非主轴削顶不破坏主轴过零序列
  out->winClip = clipInWinAxis(ax);
  out->clipLevel = clipLevelOf(out->winClip);
}

void dspProcess(const Sample& s, DspFrame* out) {
  out->envReady = false;
  out->winReady = false;
  out->fx = out->fy = out->fz = 0.0f;
  // 质量字段每样本重新初始化：
  //   runWindow 每 HOP 个样本才跑一次，其余样本若不初始化，
  //   调用方会读到上一窗的 winClip/clipLevel。
  out->winClip = 0;
  out->clipLevel = CLIP_NONE;
  out->sampleClipped = false;
  out->envClipped = false;
  out->clipAxisMask = 0;

  const float k = 1000.0f / mpuLsbPerG();

  // 逐轴判断削顶：主轴削顶才影响主频，任意轴削顶都影响合成 RMS
  uint8_t mask = 0;
  if (abs(s.raw.x) > ACCEL_CLIP_RAW) mask |= 0x01;
  if (abs(s.raw.y) > ACCEL_CLIP_RAW) mask |= 0x02;
  if (abs(s.raw.z) > ACCEL_CLIP_RAW) mask |= 0x04;
  out->clipAxisMask = mask;
  out->sampleClipped = (mask != 0);
  if (mask) samplerAddClip();

  const DspCal& ac = activeCal();
  float d[3] = {
    s.raw.x * k - ac.bias[0],
    s.raw.y * k - ac.bias[1],
    s.raw.z * k - ac.bias[2],
  };

  /* ---- v5 陀螺：原始值 → dps → 去零偏 ---- */
  out->gyroValid = s.gyroValid;
  if (s.gyroValid) {
    const float kg = 1.0f / mpuLsbPerDps();
    const float gr[3] = {(float)s.raw.gx, (float)s.raw.gy, (float)s.raw.gz};
    bool clip = false;
    for (int a = 0; a < 3; a++) {
      out->gyroDps[a] = gr[a] * kg - ac.gyroBias[a];
      if (fabsf(out->gyroDps[a]) > GYRO_CLIP_DPS) clip = true;
    }
    out->gyroClipped = clip;
  }

  /* 姿态支路：低通保留重力方向变化，去掉振动分量 */
  const float aLp = 1.0f / ((float)FS_HZ * CT_LPF_TAU_S);
  for (int a = 0; a < 3; a++) s_lp[a] += (d[a] - s_lp[a]) * aLp;
  out->ctDev = sqrtf(s_lp[0] * s_lp[0] + s_lp[1] * s_lp[1] + s_lp[2] * s_lp[2]);
  /* 【Q4】同时输出三轴低通值，保留方向信息。
   * ctDev 是模长，"哪个轴在变、往哪个方向变"这种信息被压掉了；
   * 接触可能只表现在某一轴上，分轴看才能分析。
   *
   * 【语义】s_lp 是**相对标定零偏后的偏差量**，不是含重力的绝对值。
   *   因为 d[a] 在上面已经减掉了 ac.bias[a]（bias 就是标定姿态的重力向量），
   *   所以静置时 ctAxis ≈ (0,0,0)。
   *   这与 ctRest 的实测值一致：静置姿态偏差约 0.1mg，若含重力会是 1000mg，
   *   阈值就会变成 4000mg 而不是实测的 30~56mg。
   *   要还原绝对朝向，把 dspCal().bias[a] 加回来即可。 */
  out->ctAxis[0] = s_lp[0];
  out->ctAxis[1] = s_lp[1];
  out->ctAxis[2] = s_lp[2];

  /* 振动支路 */
  float f[3];
  for (int a = 0; a < 3; a++) f[a] = hpStep(a, d[a]);
  if (!isfinite(f[0]) || !isfinite(f[1]) || !isfinite(f[2])) {
    dspReset();
    return;
  }
  out->fx = f[0]; out->fy = f[1]; out->fz = f[2];

  // 写位图：索引必须用写入前的 s_histIdx（与 s_hist 完全对齐）
  {
    const uint16_t bit = s_histIdx;
    const uint8_t  m   = (uint8_t)(1u << (bit & 7));
    for (int a = 0; a < 3; a++) {
      if ((mask >> a) & 1) s_clipBits[a][bit >> 3] |=  m;
      else                 s_clipBits[a][bit >> 3] &= (uint8_t)~m;
    }
  }
  for (int a = 0; a < 3; a++) s_hist[a][s_histIdx] = f[a];
  s_histIdx = (uint16_t)((s_histIdx + 1) & (BLOCK_N - 1));
  if (s_histFill < BLOCK_N) s_histFill++;

  float msq = f[0] * f[0] + f[1] * f[1] + f[2] * f[2];
  s_envSum += msq - s_envBuf[s_envIdx];
  s_envBuf[s_envIdx] = msq;
  // 包络位图：任轴削顶即置位（合成 RMS 会被任一轴污染）
  {
    const uint16_t ebit = s_envIdx;
    const uint8_t  em   = (uint8_t)(1u << (ebit & 7));
    if (mask) s_envClipBits[ebit >> 3] |=  em;
    else      s_envClipBits[ebit >> 3] &= (uint8_t)~em;
  }
  if (++s_envIdx >= ENV_WIN) {
    s_envIdx = 0;
    // 每轮重新求和，消除浮点加减的累积误差
    float acc = 0.0f;
    for (int i = 0; i < ENV_WIN; i++) acc += s_envBuf[i];
    s_envSum = acc;
  }
  if (s_envFill < ENV_WIN) s_envFill++;
  out->envClipped = envWindowClipped();

  if (++s_envHop >= ENV_HOP) {
    s_envHop = 0;
    if (s_envFill >= ENV_WIN) {
      s_env = sqrtf(fmaxf(s_envSum, 0.0f) / (float)ENV_WIN);
      out->envReady = true;
      out->env = s_env;
      if (s_calMode) { s_calEnv += s_env; s_calCt += out->ctDev; s_calEnvN++; }
    }
  }

  if (s_calMode) {
    for (int a = 0; a < 3; a++) s_calBandSq[a] += (double)f[a] * f[a];
    s_calBandN++;
    return;
  }

  if (++s_hopCnt >= HOP) {
    s_hopCnt = 0;
    if (s_histFill >= BLOCK_N) runWindow(out);
  }
}

/* ---------------- 标定（事务化） ----------------
 *
 * 结构：候选 → 校验 → 提交。全流程只有一处写 s_cal。
 *
 *   calBias(&cand)      阶段A 只写候选
 *   stageAOk(cand)      样本数 / 静置 std / 重力模长
 *   runStageB(&cand)    阶段B 用候选偏置跑，写候选的噪声与包络
 *   stageBOk(cand)      数据量 / 有限性 / 削顶 / 包络 / 姿态偏差
 *   s_cal = cand        唯一提交点
 *
 * 任一失败 → 丢弃候选，s_cal 一个字节都不变。
 * 合约：无论成败，返回时采样器停止、DSP 干净、s_useCand/s_calMode 已复位。
 */

/* 阶段A：只写 cand，绝不碰 s_cal */
static bool calBias(DspCal* cand) {
  double sx = 0, sy = 0, sz = 0, qx = 0, qy = 0, qz = 0;
  /* v5：同一段静置里顺带统计陀螺零偏，不额外增加标定时间 */
  double gx = 0, gy = 0, gz = 0, qgx = 0, qgy = 0, qgz = 0;
  uint32_t n = 0;
  const float k = 1000.0f / mpuLsbPerG();
  const float kg = 1.0f / mpuLsbPerDps();
  uint32_t t0 = millis();
  while (millis() - t0 < CAL_BIAS_MS) {
    MpuRaw r;
    if (mpuReadAll(&r)) {
      double x = r.x * k, y = r.y * k, z = r.z * k;
      sx += x; sy += y; sz += z;
      qx += x * x; qy += y * y; qz += z * z;
      const double wx = r.gx * kg, wy = r.gy * kg, wz = r.gz * kg;
      gx += wx; gy += wy; gz += wz;
      qgx += wx * wx; qgy += wy * wy; qgz += wz * wz;
      n++;
    }
    if ((n & 31u) == 0u) delay(1);   // 让出 CPU，避免看门狗
  }
  if (n < (uint32_t)CAL_MIN_SAMPLES) return false;   // 样本数不足：不写候选

  double m[3] = {sx / n, sy / n, sz / n};
  double q[3] = {qx / n, qy / n, qz / n};
  for (int a = 0; a < 3; a++) {
    cand->bias[a] = (float)m[a];
    cand->rawStd[a] = (float)sqrt(fmax(q[a] - m[a] * m[a], 0.0));
  }
  cand->biasNormMg = sqrtf(cand->bias[0] * cand->bias[0] + cand->bias[1] * cand->bias[1] +
                           cand->bias[2] * cand->bias[2]);

  /* 陀螺零偏与静止标准差：零偏直接减掉，标准差用于判断"标定时是否在动" */
  const double gm[3] = {gx / n, gy / n, gz / n};
  const double gq[3] = {qgx / n, qgy / n, qgz / n};
  float gstd[3];
  for (int a = 0; a < 3; a++) {
    cand->gyroBias[a] = (float)gm[a];
    gstd[a] = (float)sqrt(fmax(gq[a] - gm[a] * gm[a], 0.0));
  }
  cand->gyroStdMax = fmaxf(gstd[0], fmaxf(gstd[1], gstd[2]));

  cand->nBias = n;                                   // 写在检查之后（子问题④）
  return true;
}

/* 阶段A 校验：不合格就不浪费时间跑阶段B，也不拿脏偏置去统计噪声 */
static bool stageAOk(const DspCal& cand) {
  float stdMax = fmaxf(cand.rawStd[0], fmaxf(cand.rawStd[1], cand.rawStd[2]));
  return stdMax <= CAL_MAX_STD_MG &&
         fabsf(cand.biasNormMg - 1000.0f) <= CAL_G_TOL_MG &&
         cand.gyroStdMax <= GYRO_CAL_MAX_STD_DPS;     // v5：标定时陀螺也不能在动
}

/* 阶段B 校验：削顶、静置包络、静置姿态偏差 */
static CalResult stageBOk(const DspCal& cand);

/* 阶段B：用候选偏置跑 CAL_NOISE_MS，统计带内噪声 / 静置包络 / 静置姿态 */
static CalResult runStageB(DspCal* cand) {
  for (int a = 0; a < 3; a++) s_calBandSq[a] = 0;
  s_calBandN = 0; s_calEnv = 0; s_calCt = 0; s_calEnvN = 0;

  s_cand = *cand;          // 处理链切到候选
  s_useCand = true;
  s_calMode = true;
  dspReset();

  // 削顶统计统一走全局累计（samplerStats().clip）的差值，
  // 不再维护标定专用计数器：s_clipCnt/s_calClip 都已删除。
  const uint32_t clipBase = samplerStats().clip;

  if (samplerStartIsError(samplerStart())) {
    s_calMode = false;
    s_useCand = false;
    return CALRES_ERR_TIMER;
  }
  uint32_t t0 = millis();
  while (millis() - t0 < CAL_NOISE_MS) {
    Sample s; SamplerGap gap; DspFrame fr;
    while (samplerPop(&s, &gap)) {
      if (gap.kind != SMP_GAP_NONE) dspOnGap(gap);
      dspProcess(s, &fr);
    }
    delay(1);
  }
  if (samplerStopIsError(samplerStop())) {
    s_calMode = false;
    s_useCand = false;
    return CALRES_ERR_STOP_TIMEOUT;
  }

  s_calMode = false;
  s_useCand = false;

  s_report.stageBClip = (uint16_t)(samplerStats().clip - clipBase);

  if (s_calEnvN < 20 || s_calBandN == 0) return CALRES_ERR_STAGE_B;

  float smax = 0.0f;
  for (int a = 0; a < 3; a++) {
    cand->sigBand[a] = (float)sqrt(s_calBandSq[a] / s_calBandN);
    smax = fmaxf(smax, cand->sigBand[a]);
  }
  cand->envRest = (float)(s_calEnv / s_calEnvN);
  cand->ctRest  = (float)(s_calCt / s_calEnvN);
  cand->zcl = fmaxf(ZCL_MIN_MG, ZCL_K * smax);

  s_report.envRest = cand->envRest;
  s_report.ctRest  = cand->ctRest;

  // 参数有限性：任何一项非有限都说明计算过程出了问题
  bool finite = isfinite(cand->envRest) && isfinite(cand->ctRest) && isfinite(cand->zcl);
  for (int a = 0; a < 3 && finite; a++)
    if (!isfinite(cand->sigBand[a])) finite = false;
  if (!finite) return CALRES_ERR_STAGE_B;

  return stageBOk(*cand);
}

/* 阶段B 校验：削顶、静置包络、静置姿态偏差 */
static CalResult stageBOk(const DspCal& cand) {
  if (s_report.stageBClip > 0) return CALRES_ERR_STAGE_B_QUALITY;
  if (cand.envRest > CAL_MAX_ENV_MG) return CALRES_ERR_STAGE_B_QUALITY;
  if (cand.ctRest > CAL_MAX_CT_MG) return CALRES_ERR_STAGE_B_QUALITY;
  return CALRES_OK;
}

bool dspCalibrate(void) {
  if (samplerStopIsError(samplerStop())) {
    // 回调未退出：不 reset、不碰缓冲，直接报错
    s_report.result = CALRES_ERR_STOP_TIMEOUT;
    return false;
  }
  samplerReset();
  dspReset();

  s_report = DspCalReport{};

  DspCal cand = {};
  CalResult err = CALRES_ERR_STAGE_A;   // 先设最坏，成功再改
  bool ok = false;

  do {
    /* ---- 阶段A ---- */
    if (!calBias(&cand)) { err = CALRES_ERR_SAMPLES; break; }
    s_report.nBias      = cand.nBias;
    s_report.rawStdMax  = fmaxf(cand.rawStd[0], fmaxf(cand.rawStd[1], cand.rawStd[2]));
    s_report.biasNormMg = cand.biasNormMg;

    if (!stageAOk(cand)) { err = CALRES_ERR_STAGE_A; break; }

    /* ---- 阶段B ---- */
    err = runStageB(&cand);
    if (err != CALRES_OK) break;

    /* ---- 唯一提交点 ---- */
    cand.quality = true;
    cand.fsSel = (uint8_t)(ACCEL_FS_SEL & 0x03);   // 记录本标定对应的量程
    s_cal = cand;
    s_cal.ok = true;
    ok = true;
  } while (0);

  /* ---- 统一清理：成功失败都走这里 ---- */
  s_calMode = false;
  s_useCand = false;
  // 若 stop 超时，回调可能仍在跑：此时不得 reset（会被拒绝并计数），
  // 直接保留缓冲现状，让下一次故障恢复流程处理。
  if (!samplerStopIsError(samplerStop())) samplerReset();
  dspReset();

  s_report.result = ok ? CALRES_OK : err;
  return ok;
}

const DspCal& dspCal(void) { return s_cal; }

CalResult dspCalLastResult(void) { return s_report.result; }
const DspCalReport& dspCalReport(void) { return s_report; }

const char* calResultStr(CalResult r) {
  switch (r) {
    case CALRES_NONE:                return "未尝试";
    case CALRES_OK:                  return "成功，已提交";
    case CALRES_ERR_SAMPLES:         return "阶段A 样本不足";
    case CALRES_ERR_STAGE_A:         return "阶段A 静置质量不合格";
    case CALRES_ERR_STAGE_B:         return "阶段B 数据不足或参数异常";
    case CALRES_ERR_STAGE_B_QUALITY: return "阶段B 被扰动（削顶/包络/姿态超限）";
    case CALRES_ERR_TIMER:           return "阶段B 采样器启动失败";
    case CALRES_ERR_STOP_TIMEOUT:    return "采样回调未退出，标定中止";
  }
  return "?";
}
