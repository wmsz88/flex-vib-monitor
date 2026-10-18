#include "state.h"
#include "config.h"
#include <math.h>

static Thresholds s_th = {TH_HI_MIN_MG, TH_LO_MIN_MG, CT_MIN_MG, CT_MIN_MG * CT_OFF_RATIO};
static Result     s_r = {};

/* 启停状态机 */
static uint8_t  s_hiCnt = 0, s_loCnt = 0;
static bool     s_haveEdge = false;
static uint32_t s_lastEdgeUs = 0;
static float    s_periodMs[SS_EDGE_MAX];
static uint8_t  s_nPeriod = 0;

/* 主频时效 */
static bool     s_haveWin = false;
static uint32_t s_lastWinUs = 0;

/* 幅值时效 */
static bool     s_haveEnv = false;
static uint32_t s_lastEnvUs = 0;

/* 接触确认 */
static uint8_t  s_ctPosture = 0;
static uint8_t  s_ctCand = 0;
static uint32_t s_ctCandUs = 0;

/* 边沿插值：保存上一个包络点，用于求"阈值真正被跨越的时刻"。
   包络每 ENV_HOP 个样本才出一个点（1kHz 下 10ms），直接用包络点时刻
   会有 ±10ms 的量化误差；对 3Hz（周期 333ms）就是 ±3%。
   注意：这 5 个状态在 stateReset 和 stateOnGap 里都必须清，
   否则缺口会拿缺口前/后的两个包络点插出一个从未发生的边沿。 */
static float    s_prevEnv       = 0.0f;
static uint32_t s_prevEnvUs     = 0;
static bool     s_havePrevEnv   = false;
static uint32_t s_pendingEdgeUs = 0;
static bool     s_havePendingEdge = false;

/* 阈值穿越时刻的线性插值：e0/e1 跨越 th，返回对应时刻 */
static uint32_t crossTimeUs(float e0, uint32_t t0, float e1, uint32_t t1, float th) {
  const float de = e1 - e0;
  if (de == 0.0f) return t1;
  float frac = (th - e0) / de;
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;
  const int32_t dt = (int32_t)(t1 - t0);
  return (uint32_t)((int32_t)t0 + (int32_t)(frac * (float)dt));
}

/* 清空边沿插值状态（stateReset / stateOnGap 共用） */
static void clearEdgeInterp(void) {
  s_prevEnv = 0.0f;
  s_prevEnvUs = 0;
  s_havePrevEnv = false;
  s_pendingEdgeUs = 0;
  s_havePendingEdge = false;
}

static float medianCopy(const float* a, int n) {
  float t[SS_EDGE_MAX];
  for (int i = 0; i < n; i++) t[i] = a[i];
  for (int i = 1; i < n; i++) {
    float v = t[i];
    int j = i - 1;
    while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
    t[j + 1] = v;
  }
  return (n & 1) ? t[n / 2] : 0.5f * (t[n / 2 - 1] + t[n / 2]);
}

static void ssClearPeriods(void) { s_nPeriod = 0; s_r.ssValid = false; s_r.ssHz = 0; s_r.ssPeriods = 0; }

static void ssEvaluate(void) {
  int n = s_nPeriod < SS_USE_PERIODS ? s_nPeriod : SS_USE_PERIODS;
  s_r.ssPeriods = (uint8_t)n;
  if (n < SS_MIN_PERIODS) { s_r.ssValid = false; s_r.ssHz = 0; return; }
  const float* p = &s_periodMs[s_nPeriod - n];
  float pmin = p[0], pmax = p[0];
  for (int i = 1; i < n; i++) { pmin = fminf(pmin, p[i]); pmax = fmaxf(pmax, p[i]); }
  if (pmax > pmin * SS_CONSIST_RATIO) { s_r.ssValid = false; s_r.ssHz = 0; return; }
  float hz = 1000.0f / medianCopy(p, n);
  s_r.ssValid = isfinite(hz);
  s_r.ssHz = s_r.ssValid ? hz : 0.0f;
}

static void ssOnStartEdge(uint32_t tUs) {
  if (s_haveEdge) {
    float pMs = (float)(uint32_t)(tUs - s_lastEdgeUs) / 1000.0f;
    if (pMs < SS_T_MIN_MS || pMs > SS_T_MAX_MS) {
      ssClearPeriods();             // 抖动或间隔过长：节奏中断，重新累计
    } else {
      if (s_nPeriod >= SS_EDGE_MAX) {
        for (int i = 1; i < SS_EDGE_MAX; i++) s_periodMs[i - 1] = s_periodMs[i];
        s_nPeriod = SS_EDGE_MAX - 1;
      }
      s_periodMs[s_nPeriod++] = pMs;
      ssEvaluate();
    }
  }
  s_haveEdge = true;
  s_lastEdgeUs = tUs;
}

static void ssStep(float env, uint32_t tUs) {
  // ① 上升穿越检测：只在"未振动 + 真的跨过 hi"时记录插值时刻。
  //    这一步与确认机制解耦：确认要等 SS_CONFIRM 个点，但边沿时刻用穿越点。
  if (s_havePrevEnv && s_r.vib == 0 &&
      s_prevEnv <= s_th.hi && env > s_th.hi) {
    s_pendingEdgeUs   = crossTimeUs(s_prevEnv, s_prevEnvUs, env, tUs, s_th.hi);
    s_havePendingEdge = true;
  }

  if (s_r.vib == 0) {
    if (env > s_th.hi) {
      if (++s_hiCnt >= SS_CONFIRM) {
        s_hiCnt = 0; s_r.vib = 1; s_r.nStart++;
        // 用插值出的穿越时刻；没有记录（如启动时已在阈值上）则回退到当前时刻
        ssOnStartEdge(s_havePendingEdge ? s_pendingEdgeUs : tUs);
        s_havePendingEdge = false;
      }
    } else {
      s_hiCnt = 0;
      s_havePendingEdge = false;   // 没升上去，作废
    }
  } else {
    if (env < s_th.lo) {
      if (++s_loCnt >= SS_CONFIRM) {
        s_loCnt = 0; s_r.vib = 0; s_r.nStop++;
        s_r.freqValid = false; s_r.freqHz = 0;
        s_r.freqClipLimited = false;   // 振动停止，降级标记无意义
      }
    } else s_loCnt = 0;
  }

  s_prevEnv     = env;
  s_prevEnvUs   = tUs;
  s_havePrevEnv = true;
}

static void ctStep(float dev, uint32_t tUs) {
  if (!s_ctPosture && dev > s_th.ctOn) s_ctPosture = 1;
  else if (s_ctPosture && dev < s_th.ctOff) s_ctPosture = 0;

  uint8_t raw = s_ctPosture;
#if CT_MODE == 1
  if (s_r.vib) raw = 1;
#endif
  if (raw == s_r.contact) { s_ctCand = raw; return; }
  if (raw != s_ctCand) { s_ctCand = raw; s_ctCandUs = tUs; return; }
  if ((uint32_t)(tUs - s_ctCandUs) >= (uint32_t)CT_CONFIRM_MS * 1000u) s_r.contact = raw;
}

void stateApplyCal(const DspCal& cal) {
  s_r.calOk = cal.ok;
  s_r.calQuality = cal.quality;
  s_r.contactValid = false;   // 重新标定不代表接触判据可信
  if (!cal.ok) return;
  s_th.hi = fmaxf(TH_HI_MIN_MG, TH_HI_K * cal.envRest);
  s_th.lo = fmaxf(TH_LO_MIN_MG, TH_LO_K * cal.envRest);
  s_th.ctOn = fmaxf(CT_MIN_MG, CT_K * cal.ctRest);
  s_th.ctOff = s_th.ctOn * CT_OFF_RATIO;

  // 运行期契约自检：滞回阈值必须有序（hi > lo、ctOn > ctOff），
  // 否则状态机会在阈值附近反复翻转，启停计数与接触状态都会乱。
  // 正常参数（TH_HI_K=5 > TH_LO_K=2.5、CT_OFF_RATIO<1）天然满足；
  // 参数被改坏时这里能把问题暴露出来。
  // 不中止运行（阈值仍按公式生效），但把标定质量标为可疑。
  if (!(s_th.hi > s_th.lo) || !(s_th.ctOn > s_th.ctOff)) {
    s_r.calQuality = false;
  }
}

void stateReset(void) {
  bool sensorOk = s_r.sensorOk, calOk = s_r.calOk, calQ = s_r.calQuality;
  uint32_t ns = s_r.nStart, np = s_r.nStop;
  s_r = Result{};
  s_r.sensorOk = sensorOk; s_r.calOk = calOk; s_r.calQuality = calQ;
  s_r.nStart = ns; s_r.nStop = np;
  s_hiCnt = s_loCnt = 0;
  s_haveEdge = false;
  s_nPeriod = 0;
  s_haveWin = false;
  s_lastWinUs = 0;        // 原有代码清了 haveWin 却没清时间戳，一并补上
  s_haveEnv = false;      // 幅值时效
  s_lastEnvUs = 0;
  s_ctPosture = 0; s_ctCand = 0; s_ctCandUs = 0;
  s_r.freqClipLimited = false;   // 削顶降级标记
  s_r.contactValid = false;      // 接触判据未经真实标签验证，一律不可信
  s_lastEdgeUs = 0;              // 时间基准也清，避免逻辑变化后踩坑
  clearEdgeInterp();             // 边沿插值状态
}

/* 缺样通知：采样不连续时由主循环调用（在 stateOnFrame 之前） */
void stateOnGap(const SamplerGap& g) {
  if (g.kind == SMP_GAP_NONE) return;

  // ① 主频：立即失效，不等 FREQ_STALE_MS 超时
  s_r.freqValid = false;
  s_r.freqHz = 0;
  s_haveWin = false;

  // ② 确认链：不跨缺口累计"连续"
  s_hiCnt = 0;
  s_loCnt = 0;
  s_ctCand = s_r.contact;      // 候选回到当前状态，重新计时
  s_ctCandUs = 0;

  // ③ 启停读数：任何缺口都立即作废。
  //    保留历史 ≠ 当前读数有效：缺口后、下一个边沿到来前，
  //    旧的 ssHz 不能被当作"现在仍然成立"的结论。
  s_r.ssValid = false;
  s_r.ssHz = 0;

  // ④ 大缺口（按时间判）：历史也清掉，因为可能漏了边沿。
  //    必须限定 MISSING：JITTER 不丢边沿，不该清历史。
  if (g.kind == SMP_GAP_MISSING && g.spanUs > (uint32_t)SMP_BIG_GAP_MS * 1000u) {
    ssClearPeriods();
    s_haveEdge = false;
  }

  // ⑤ 幅值不再反映当前时刻（数值保留，用 ampValid 表达可信度）
  s_r.ampValid = false;

  // ⑥ 主频已失效，降级标记随之清零（避免"无效但带降级"的混合态）
  s_r.freqClipLimited = false;

  // ⑦ 包络点历史与待确认边沿：缺口后绝不能再拿缺口前的包络做插值。
  //    否则"缺口前低于 hi / 缺口后高于 hi"会在缺口中间插出一个
  //    从未真实发生的"起振边沿"，直接制造错误周期。
  clearEdgeInterp();
}

void stateOnFrame(const DspFrame& f, uint32_t tUs) {
  // ① 逐样本：任意轴削顶立刻让幅值失效。
  //    幅值是合成 RMS，任一轴被削平都会让它偏小。
  if (f.sampleClipped) s_r.ampValid = false;

  if (f.envReady) {
    s_r.ampMg = f.env;
    // ② 恢复条件看"包络窗内有无削顶"，而不是"当前样本是否削顶"：
    //    包络是 ENV_WIN=30 的滑动窗，当前样本干净不代表窗里干净。
    s_r.ampValid = !f.envClipped;
    s_haveEnv = true;
    s_lastEnvUs = tUs;
    s_r.ctDev = f.ctDev;
    ssStep(f.env, tUs);
    ctStep(f.ctDev, tUs);
  }
  if (f.winReady) {
    s_haveWin = true;
    s_lastWinUs = tUs;
    s_r.axis = f.axis;

    // ③ 削顶分级：只看主轴（winClip 已按主轴统计）
    const bool severe = (f.clipLevel == CLIP_SEVERE);
    const bool freqOk = (s_r.vib == 1) && f.freqHz > 0.0f;
    s_r.freqValid = freqOk && !severe;
    s_r.freqHz = s_r.freqValid ? f.freqHz : 0.0f;
    // ④ 降级标记与有效性解耦：只要主轴有削顶就标记，
    //    避免出现"主频已失效但还带降级标记"或反过来的混合态
    s_r.freqClipLimited = (f.clipLevel != CLIP_NONE);
  }
}

void stateTick(uint32_t nowUs) {
  if (s_r.freqValid && (!s_haveWin || (uint32_t)(nowUs - s_lastWinUs) > (uint32_t)FREQ_STALE_MS * 1000u)) {
    s_r.freqValid = false; s_r.freqHz = 0;
    s_r.freqClipLimited = false;      // 主频失效后不再保留降级标记
  }
  if (s_r.ampValid && (!s_haveEnv || (uint32_t)(nowUs - s_lastEnvUs) > (uint32_t)AMP_STALE_MS * 1000u)) {
    s_r.ampValid = false;
  }
  if (s_haveEdge) {
    float sinceMs = (float)(uint32_t)(nowUs - s_lastEdgeUs) / 1000.0f;
    float limit = s_r.ssValid ? SS_TIMEOUT_K * 1000.0f / s_r.ssHz : (float)SS_T_MAX_MS;
    if (sinceMs > limit && sinceMs > (float)SS_T_MIN_MS) {
      ssClearPeriods();
      if (sinceMs > (float)SS_T_MAX_MS) s_haveEdge = false;
    }
  }
}

void stateSetSensorOk(bool ok) {
  if (s_r.sensorOk == ok) return;
  s_r.sensorOk = ok;
  if (!ok) stateReset();
}

void stateClearCounters(void) { s_r.nStart = 0; s_r.nStop = 0; }

const Result&     stateResult(void) { return s_r; }
const Thresholds& stateThresholds(void) { return s_th; }

/* ---------------- 测试钩子 ---------------- */

#ifdef STATE_TEST_HOOKS
uint8_t stateTestPeriods(float* out, uint8_t maxN) {
  if (!out || maxN == 0) return 0;
  const uint8_t n = (s_nPeriod < maxN) ? s_nPeriod : maxN;
  // 返回最近 n 个（与 ssEvaluate 的取法一致：末尾 n 个）
  const uint8_t start = (uint8_t)(s_nPeriod - n);
  for (uint8_t i = 0; i < n; i++) out[i] = s_periodMs[start + i];
  return n;
}

void stateTestInjectEnv(float env, uint32_t tUs) {
  // 只驱动启停支路。接触支路需要 ctDev（姿态偏差），
  // 与包络不是同一个量，不能拿 env 冒充，所以这里刻意不调 ctStep。
  ssStep(env, tUs);
}

bool stateTestLastEdgeUs(uint32_t* out) {
  if (!out || !s_haveEdge) return false;
  *out = s_lastEdgeUs;
  return true;
}
#endif
