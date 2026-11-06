#include "state.h"
#include "config.h"
#include <math.h>

static Thresholds s_th = {TH_HI_MIN_MG, TH_LO_MIN_MG, CT_MIN_MG, CT_MIN_MG * CT_OFF_RATIO};
static Result     s_r = {};

/* 启停状态机 */
static uint8_t  s_hiCnt = 0, s_loCnt = 0;
/* 启停频率专用包络状态。它与 s_r.vib 分开：
 * s_r.vib 仍按低阈值判断真实振动，供 contact/freq 使用；
 * s_ssVib 使用更低的灵敏度/更高阈值，用来识别 ERM 惯性残余后的
 * 幅值下降沿，只驱动 ssHz。 */
static uint8_t  s_ssVib = 0;
static uint8_t  s_ssHiCnt = 0, s_ssLoCnt = 0;
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

/* 【模式2】接触姿态参考点。
 * 判据 = 当前姿态到"握住时的姿态"的距离。为什么不能用标定姿态作参考，
 * 见 config.h 的 CT_MODE 注释（标定姿态随手放，握住时旋转约100°→必然误报）。 */
static float s_ctRef[3]      = {0.0f, 0.0f, 0.0f};
static bool  s_ctRefSet      = false;
static float s_lastCtAxis[3] = {0.0f, 0.0f, 0.0f};   // 最近一次三轴低通值
/* 【模式3】平放基准：三个分量 + 是否已记录。
 * 与模式2的 s_ctRef 并存（两者语义不同，不能共用一个数组）。 */
static float s_flatRef[3]    = {0.0f, 0.0f, 0.0f};
static bool  s_flatRefSet    = false;
/* 【模式3】采集窗口内的逐轴最小/最大值，用于结算时算极差。
 * 为什么不直接用"平均值偏差"：平均会掩盖抖动，
 * 必须看窗口内实际摆动了多少，才知道"采的时候到底动了没有"。 */
static float s_refMin[3]     = {0.0f, 0.0f, 0.0f};
static float s_refMax[3]     = {0.0f, 0.0f, 0.0f};
static bool  s_refHaveSample = false;
/* 【模式3】自动采集已失败次数，以及本次采集是否由自动流程启动。
 * 区分来源很重要：只有自动启动的才允许自动重试；
 * 人手按 't' 触发的采集若失败应直接报告，不要偷偷重采，
 * 否则人会以为"按一下就成了"，实际基准来自另一时刻。
 */
static uint8_t s_flatAutoFail = 0;
static bool    s_flatAutoArm  = false;
/* 【模式3】三道门的保持状态。写成独立变量而不是复用 s_ctPosture，
 * 是因为 s_ctPosture 是模式0/1/2 的"姿态门"，语义不同。
 * 合成规则见 combineContact 的注释。 */
static uint8_t s_ctPostCmd  = 0;   // (1) 姿态门
static uint8_t s_ampHeld    = 0;   // (2) 幅值门
static uint8_t s_freqHeld   = 0;   // (3) 频率门

#if CT_HOLD_ENABLE
/* 任务 B：保留最近一次振动证据的时间，使自动启停 OFF 段仍能维持 contact。
 * 姿态门仍是安全边界；姿态一打开就清掉证据，绝不允许 hold 续命。 */
static uint32_t s_ctEvidenceUs = 0;
static bool     s_ctHaveEvidence = false;
static bool     s_ctHoldOk = false;
static uint32_t s_ctMsSinceEvidence = 0xFFFFFFFFu;

static void ctHoldClear(void) {
  s_ctEvidenceUs = 0;
  s_ctHaveEvidence = false;
  s_ctHoldOk = false;
  s_ctMsSinceEvidence = 0xFFFFFFFFu;
}
#endif

/* ---------- v5：闭合角度（陀螺积分） ----------
 * 分工：flatDist 负责"认零位"（绝对、不漂），三轴积分负责"量转了多少度"（分辨率高）。
 * 只积分、不融合姿态——姿态融合（互补滤波/卡尔曼）留给后续，先把角度量准。
 */
static float    s_gyroBias[3]   = {0.0f, 0.0f, 0.0f};  // 来自标定
static float    s_angleDeg[3]   = {0.0f, 0.0f, 0.0f};  // 三轴各自积分角度
static uint32_t s_stillUs       = 0;                   // 连续静止时长（ZUPT 用）
static uint32_t s_lastGyroUs    = 0;                   // 最近一次收到陀螺数据的时刻
static uint8_t  s_angOpenHeld   = 0;                   // 角度滞回：已离开松手位
static uint8_t  s_angAirHeld    = 0;                   // 角度滞回：已闭合到底（空夹）
static bool     s_gyroClipLatched = false;              // 超量程后角度不可信，直到张开归零
static bool     s_sinceClose    = false;               // 上次采基准以来是否闭合过（自动重采用）
/* 【为什么上面这些模式3变量不放在 #if CT_MODE == 3 里】
 *   它们的使用点分布在 stateStartFlatRef / flatRefCommit / stateTickFlatRef /
 *   stateClearFlatRef / stateApplyCal / stateReset 等函数中，这些函数本身是
 *   无条件编译的（state.h 里对它们都有声明）。曾把声明单独包进 #if 过，
 *   结果 CT_MODE=0/1/2 编译时全部报 not declared in this scope（35 个错误）。
 *   代价只是非模式3时多出十来个从不被读写的静态变量（约 40 字节 RAM），
 *   换来的是任何一种模式都能编译通过 —— 这个取舍是明确的。 */

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
  // ① 启停频率支路：用独立的 SS_HI/SS_LO 看包络调制。
  //    它不改变 s_r.vib，也不把残余振动冒充成 contact。
  if (s_havePrevEnv && s_ssVib == 0 &&
      s_prevEnv <= SS_HI_MG && env > SS_HI_MG) {
    s_pendingEdgeUs   = crossTimeUs(s_prevEnv, s_prevEnvUs, env, tUs, SS_HI_MG);
    s_havePendingEdge = true;
  }

  if (s_ssVib == 0) {
    if (env > SS_HI_MG) {
      if (++s_ssHiCnt >= SS_CONFIRM) {
        s_ssHiCnt = 0; s_ssVib = 1;
        // 用插值出的穿越时刻；没有记录（如启动时已在阈值上）则回退到当前时刻
        ssOnStartEdge(s_havePendingEdge ? s_pendingEdgeUs : tUs);
        s_havePendingEdge = false;
      }
    } else {
      s_ssHiCnt = 0;
      s_havePendingEdge = false;   // 没升上去，作废
    }
  } else {
    if (env < SS_LO_MG) {
      if (++s_ssLoCnt >= SS_CONFIRM) {
        s_ssLoCnt = 0; s_ssVib = 0;
      }
    } else s_ssLoCnt = 0;
  }

  // ② 原振动状态仍按 TH_HI/TH_LO 维护，供 contact/freqValid 使用。
  if (s_r.vib == 0) {
    if (env > s_th.hi) {
      if (++s_hiCnt >= SS_CONFIRM) {
        s_hiCnt = 0; s_r.vib = 1; s_r.nStart++;
      }
    } else {
      s_hiCnt = 0;
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

/**
  * @brief  接触判据状态机（四种模式共用确认逻辑，只有 raw 的求法不同）
  *
  * @param dev   相对标定姿态的姿态偏差 mg（模式 0/1 用）
  * @param tUs   当前时刻
  *
  * 【模式 2/3 的 raw 在这里算不出来】
  *   两者都需要三轴向量与参考点的距离，所以由 stateOnFrame 预先算好，
  *   写入 s_ctDist / s_flatDist，再由本函数读取。
  *   这样四种模式仍共用同一套滞回 + 确认逻辑 —— 确认延时 CT_CONFIRM_MS
  *   在四种模式下行为完全一致，现场调参经验可以跨模式复用。
  */
static float s_ctDist = -1.0f;   // 【模式2】到接触姿态的距离；-1 = 无参考点

static float s_flatDist = -1.0f; // 【模式3】到平放姿态的距离；-1 = 无参考点

/* 【模式3】滞回门模板：输入当前量值 x 与保持状态 held，返回下一状态。
 *   held=0（尚未成立）→ 必须超过 onTh 才置 1
 *   held=1（已成立）  → 必须跌破 offTh 才清 0
 * 三道门共用一个函数，是为了保证三条门的滞回写法不可能不一致
 * （on/off 写反是本类判据最常见的错误，统一到一处即可杜绝）。 */
static uint8_t gateStep(uint8_t held, float x, float onTh, float offTh) {
  if (!held) return (x > onTh) ? 1 : 0;
  return (x < offTh) ? 0 : 1;
}

/* 【模式3】把三道门合成 contact。
 *
 * 【为什么不是简单的"三路与"】
 *   激励通常是间歇的（人摇几下会停一下）。若逐样本要求三路全真，
 *   幅值门会在两次激励之间掉下去，contact 随之在 CT_CONFIRM_MS 后归零
 *   -> 一次"握持"被切成一串碎片。单点读数不受影响，但接触状态的
 *   连续性被破坏，上位机做持续时间统计时会严重低估。
 *
 * 【实际采用的规则】
 *   进入：三路**同时**成立；CT_SS_ENTRY_ENABLE=1 时，姿态 + 启停节奏锁定（≥CT_SS_ENTRY_MIN_HZ）也可进入。
 *   保持：姿态仍成立，且"幅值或频率"任意一路仍成立。
 *   退出：其余情况。
 *   姿态是必要条件（手必须还握着）；振动类证据维持一条即可。
 *   于是空夹（频率门不成立）无论幅值多大都进不了 contact，
 *   而握住有物体时即使幅值短暂跌落，只要主频还在就不会掉出来。
 *
 * 【数据失效与判据不成立的区分】
 *   ampKnown/freqKnown 表示"这个量此刻是否可信"。
 *   不可信 ≠ 低于阈值（后者是判据不成立）。二者混同会让
 *   "主频没估出来"被当成"频率不够"，从而错误地把接触判掉。
 *   失效时怎么办由 CT_FLAT_DEGRADE_STRICT 决定：
 *     1 = 视为不满足（保守，不误报，但可能漏报）
 *     0 = 视为满足（宽松，不断线，但可能误报）
 */
#if CT_MODE == 3 || CT_MODE == 4
static uint8_t combineContact(uint8_t postureOk, uint8_t ampOk, uint8_t freqOk,
                              uint8_t ampKnown, uint8_t freqKnown, uint8_t prev,
                              uint32_t nowUs) {
#if CT_FLAT_DEGRADE_STRICT
  const uint8_t ampEff  = ampOk;
  const uint8_t freqEff = freqOk;
#else
  const uint8_t ampEff  = ampKnown  ? ampOk  : 1u;
  const uint8_t freqEff = freqKnown ? freqOk : 1u;
#endif
#if CT_SS_ENTRY_ENABLE
  /* T011：启停节奏已锁定（≥SS_MIN_PERIODS 个规律周期）且够快，作为进入证据。
   * ssStep 在 ctStep 之前执行，这里读到的是本帧最新值。 */
  const uint8_t ssEff = (s_r.ssValid && s_r.ssHz >= CT_SS_ENTRY_MIN_HZ) ? 1u : 0u;
#else
  const uint8_t ssEff = 0u;
#endif
#if CT_HOLD_ENABLE
  const uint8_t evidence = (ampEff || freqEff || ssEff) ? 1u : 0u;
  if (!postureOk) {
    ctHoldClear();
  } else if (evidence) {
    s_ctEvidenceUs = nowUs;
    s_ctHaveEvidence = true;
  }
  s_ctMsSinceEvidence = s_ctHaveEvidence
                            ? (uint32_t)((nowUs - s_ctEvidenceUs) / 1000u)
                            : 0xFFFFFFFFu;
  s_ctHoldOk = (postureOk && s_ctHaveEvidence &&
                s_ctMsSinceEvidence <= (uint32_t)CT_HOLD_MS) ? 1u : 0u;
#endif
  if (!prev) return (postureOk && ((ampEff && freqEff) || ssEff)) ? 1u : 0u;
#if CT_HOLD_ENABLE
  return (postureOk && (ampEff || freqEff || s_ctHoldOk)) ? 1u : 0u;
#else
  (void)nowUs;
  return (postureOk && (ampEff || freqEff)) ? 1u : 0u;
#endif
}
#endif

#if CT_HOLD_ENABLE
bool stateContactHoldOk(void) { return s_ctHoldOk; }
uint32_t stateContactMsSinceEvidence(void) { return s_ctMsSinceEvidence; }
#endif

/* ---------- v5：闭合角度积分 ----------
 * 用 flatDist 判"回到张开"（绝对、不漂）来定零位，用陀螺积分量"转了多少"（分辨率高）。
 * 静止（合成角速度 < 死区）时不积分，天然完成零速修正，避免零偏长时间累积。
 */
static void angleStep(const DspFrame& f, uint32_t tUs) {
  if (f.gyroValid) {
    s_lastGyroUs = tUs;
    s_r.gyroSeen = true;

    if (f.gyroClipped) {
      /* 超量程：漏掉的角度无法补回。锁存到下次在张开位归零，
       * 避免下一帧恢复非削顶后 angleValid 又立刻变成 true。 */
      s_gyroClipLatched = true;
      s_r.angleValid = false;
      return;
    }

    const float w = sqrtf(f.gyroDps[0] * f.gyroDps[0] + f.gyroDps[1] * f.gyroDps[1] +
                          f.gyroDps[2] * f.gyroDps[2]);
    s_r.gyroW = w;          // 诊断：当前合成角速度（判断是否静止）
    const uint32_t dtUs = (uint32_t)(GYRO_DT_S * 1e6f + 0.5f);

    const bool still = (w < GYRO_DEADBAND_DPS);
    if (still) {
      if (s_stillUs < 0xFFFFFFFFu - dtUs) s_stillUs += dtUs;   // 饱和保护
    } else {
      s_stillUs = 0;
      for (int a = 0; a < 3; a++) s_angleDeg[a] += f.gyroDps[a] * GYRO_DT_S;
    }

    /* "停在张开位"= 姿态接近张开 **且** 角速度在死区内 **且** 已静止够久。
     * 三个条件都必要：
     *   - 只看 D：转动起步、D 还没超门槛的那几度会被反复清零（实测少积 18%）；
     *   - 不看 GYRO_STILL_MS：闭合途中路过张开角度附近的一次瞬时停顿也会被清零。
     * 所以要求"静止满 GYRO_STILL_MS"才认定是停回了张开位。 */
    const bool parked = (s_flatRefSet && s_flatDist >= 0.0f &&
                         s_flatDist < OPEN_D_MG && still &&
                         s_stillUs >= (uint32_t)GYRO_STILL_MS * 1000u);

    /* 停在张开位 ⇒ 角度归零。这是整套方案的"绝对参考"来源。 */
    if (parked) {
      s_angleDeg[0] = s_angleDeg[1] = s_angleDeg[2] = 0.0f;
      s_gyroClipLatched = false;
    }

    float m = fabsf(s_angleDeg[0]);
    if (fabsf(s_angleDeg[1]) > m) m = fabsf(s_angleDeg[1]);
    if (fabsf(s_angleDeg[2]) > m) m = fabsf(s_angleDeg[2]);

    /* 积分超过 360° 视为异常（不可能真转这么多），复位并标无效 */
    if (m > 360.0f) {
      s_angleDeg[0] = s_angleDeg[1] = s_angleDeg[2] = 0.0f;
      m = 0.0f;
      s_gyroClipLatched = true;
      s_r.angleValid = false;
    } else if (!s_gyroClipLatched) {
      s_r.angleValid = true;
    } else {
      s_r.angleValid = false;
    }
    s_r.closeAngleDeg = m;
  }

  /* 长期收不到陀螺数据 ⇒ 角度不可信（区别于"没有陀螺"的 gyroSeen=false） */
  if (s_r.angleValid && s_r.gyroSeen &&
      (uint32_t)(tUs - s_lastGyroUs) > (uint32_t)GYRO_STALE_MS * 1000u) {
    s_r.angleValid = false;
    /* 【P1 修正】陀螺失效等价于"证据中断"：确认链必须从现在重新累计，
     * 否则恢复后的第一帧可能沿用中断前的时间基准，把确认延时算短。 */
    s_ctCand   = s_r.contact;
    s_ctCandUs = 0;
  }
}

static bool postureInput(float* x, float* onTh, float* offTh) {
#if CT_MODE == 4
  if (s_r.angleValid && s_flatRefSet) {
    *x = s_r.closeAngleDeg; *onTh = ANGLE_CLOSED_DEG; *offTh = ANGLE_CLOSED_OFF_DEG;
    return true;
  }
  /* 振动/超量程会让角度失效；若已有平放基准，用 flatDist 兜底姿态门，
   * 避免"机械上仍夹住、但角度变 ?"时把 contact 直接判 0。 */
  if (s_flatRefSet && s_flatDist >= 0.0f) {
    *x = s_flatDist; *onTh = CT_FLAT_D_ON_MG; *offTh = CT_FLAT_D_OFF_MG;
    return true;
  }
  *x = s_r.closeAngleDeg; *onTh = ANGLE_CLOSED_DEG; *offTh = ANGLE_CLOSED_OFF_DEG;
  return false;
#else
  *x = s_flatDist; *onTh = CT_FLAT_D_ON_MG; *offTh = CT_FLAT_D_OFF_MG;
  return s_flatRefSet;
#endif
}
static void ctStep(float dev, uint32_t tUs) {


#if CT_MODE == 3 || CT_MODE == 4
  /* 用花括号包成独立作用域：模式 3 与下面模式 0/1/2 都有局部变量 raw，
   * 不加作用域会冲突（实测报 conflicting declaration 'uint8_t raw'）。 */
  {
  /* ---------- 模式 3：平放基准 + 三重与 ----------
   * 无平放基准时无法判定：保持 contact=0、contactValid=false。
   * 这与模式 2 无参考点的处理一致 —— "不知道"必须与"未接触"可区分，
   * 否则上位机会把"还没采到基准"当成"确实没接触"。 */
  float px, pon, poff;
  if (!postureInput(&px, &pon, &poff)) {
    s_ctPosture      = 0;
    s_ctPostCmd      = 0;
    s_ampHeld        = 0;
    s_freqHeld       = 0;
#if CT_HOLD_ENABLE
    ctHoldClear();
#endif
    s_ctCand         = 0;
    s_ctCandUs       = 0;
    s_r.contact      = 0;
    s_r.closed       = false;
    s_r.condPosture  = 0;
    s_r.condAmp      = 0;
    s_r.condFreq     = 0;
    s_r.gripNoObject = false;
    /* 【P1 修正】判据输入不可用时，结论本身也不可信。
     * 原来这里不写 contactValid，它只在 stateReset 里被赋成 flatSet，
     * 于是模式 4 下"角度无效但平放基准有效"会留下 contactValid=1 —— 语义错了。 */
    s_r.contactValid = false;
    return;
  }

  /* ---- (1) 姿态门：到平放姿态的距离越大，说明手压得越深 ---- */
  s_r.flatDist = s_flatDist;
  s_ctPostCmd  = gateStep(s_ctPostCmd, px, pon, poff);
  s_r.condPosture = s_ctPostCmd;

  /* ---- (2) 幅值门：只用来排除"静止" ----
   * ampValid=false 时 ampMg 可能是上一次的值或 0，不可作为判据输入。
   *
   * 【2026-10-29 实测修正】原来"无效时保留上一状态"会让门永久锁死：
   *   闭合动作的瞬态把 s_ampHeld 抬到 1 之后，若 ampValid 一直为 false，
   *   gateStep 再也不执行，s_ampHeld 就停在 1；而 combineContact 的
   *   保持规则 (幅值||频率) 因此恒成立，contact 永远掉不下来。
   * 实板现象：空夹 30s 内 ct 恒为 1，几分钟后才自己松开。
   * 正确语义是"数据失效 = 这条证据当前不成立"，所以必须显式清零，
   * 否则 CT_FLAT_DEGRADE_STRICT=1 也救不回来（它读的正是这个保持位）。 */
  if (s_r.ampValid) {
    s_ampHeld = gateStep(s_ampHeld, s_r.ampMg, CT_FLAT_A_ON_MG, CT_FLAT_A_OFF_MG);
  } else {
    s_ampHeld = 0;
  }
  s_r.condAmp = (s_r.ampValid && s_r.ampMg > CT_FLAT_A_ON_MG) ? 1 : 0;

  /* ---- (3) 频率门：辅助证据（实测不能单独区分空夹）----
   * 同上：freqValid=false（空夹测不出主频）时必须把保持位清零，
   * 否则瞬态估出的一次主频会让频率门永久为真。 */
  if (s_r.freqValid) {
    s_freqHeld = gateStep(s_freqHeld, s_r.freqHz, CT_FLAT_F_ON_HZ, CT_FLAT_F_OFF_HZ);
  } else {
    s_freqHeld = 0;
  }
  s_r.condFreq = (s_r.freqValid && s_r.freqHz > CT_FLAT_F_ON_HZ) ? 1 : 0;

  const uint8_t raw = combineContact(s_ctPostCmd, s_ampHeld, s_freqHeld,
                                     s_r.ampValid  ? 1u : 0u,
                                     s_r.freqValid ? 1u : 0u,
                                     s_r.contact, tUs);
  /* 三道门的输入此刻都可用（postureInput 已过），所以"结论可信"。
   * 注意语义：contactValid=1 只表示"这个 0/1 结论是按有效输入算出来的"，
   * 不表示 contact=1。上位机不能用 contactValid 当"检测到接触"。 */
  s_r.contactValid = true;

  /* 空夹判定（诊断用，不参与 contact）：
   *   姿态已握下 + **幅值门真的过了** + 频率门没通过
   *   -> 手合上了、确实有振动，却没有物体共振特征，最可能是没夹到东西。
   *
   * 【2026-10-29 评审修正】原来用的是 s_r.ampValid，它只表示"幅值可信"，
   * 不表示"幅值够大"。于是"闭合后一点振动都没有（5mg）"也被标成空夹 ——
   * 把"不知道有没有物体"说成了"确定没有物体"。
   * 现在改用 s_ampHeld（幅值门已经过，>CT_FLAT_A_ON_MG）：
   *   没有振动证据 ⇒ gripNoObject=0（既不说有、也不说没有），
   *   上位机应结合 closed / closedAngle 自行判断"已闭合但无振动证据"。 */
#if CT_FLAT_REPORT_NO_OBJ
  s_r.gripNoObject = (s_ctPostCmd && s_ampHeld && !s_freqHeld) ? true : false;
#else
  s_r.gripNoObject = false;
#endif
  /* 显式给出"已闭合"，让上位机不必再从角度自己推断 */
  s_r.closed = (s_ctPostCmd != 0);
  if (s_ctPostCmd) s_sinceClose = true;   // 记下"本轮到过闭合位"，供自动重采判据用

  /* 回到共用确认链（与模式 0/1/2 完全一致，保证确认延时不因模式而异） */
  if (raw == s_r.contact) { s_ctCand = raw; return; }
  if (raw != s_ctCand) { s_ctCand = raw; s_ctCandUs = tUs; return; }
  if ((uint32_t)(tUs - s_ctCandUs) >= (uint32_t)CT_CONFIRM_MS * 1000u) s_r.contact = raw;
  return;
  }
#endif

#if CT_MODE == 2
  /* 模式 2：以"接触姿态"为参考点。无参考点时无法判定，
   * 保持 contact=0 且 contactValid=false（不假装知道）。 */
  if (!s_ctRefSet) {
    s_ctPosture = 0;
    s_ctCand    = 0;
    s_ctCandUs  = 0;
    s_r.contact = 0;
    return;
  }
  if (!s_ctPosture && s_ctDist < CT_TOUCH_MG)      s_ctPosture = 1;
  else if (s_ctPosture && s_ctDist > CT_TOUCH_OFF_MG) s_ctPosture = 0;
  uint8_t raw = s_ctPosture;
#else
  if (!s_ctPosture && dev > s_th.ctOn) s_ctPosture = 1;
  else if (s_ctPosture && dev < s_th.ctOff) s_ctPosture = 0;

  uint8_t raw = s_ctPosture;
#if CT_MODE == 1
  if (s_r.vib) raw = 1;
#endif
#endif

  if (raw == s_r.contact) { s_ctCand = raw; return; }
  if (raw != s_ctCand) { s_ctCand = raw; s_ctCandUs = tUs; return; }
  if ((uint32_t)(tUs - s_ctCandUs) >= (uint32_t)CT_CONFIRM_MS * 1000u) s_r.contact = raw;
}

/* ---- 模式 2：接触姿态参考点 ---- */

/* ---- 模式 2：接触姿态参考点（两阶段采集，不阻塞主循环）----
 *
 * 为什么需要延时+平均，而不是记瞬时值：
 *   实测按 't' 的瞬间手往往还没握到位，记下的参考点偏离稳态 1593mg，
 *   而稳态只有 21mg。判定侥幸正确但余量被吃掉一大半。
 *
 * 三阶段状态机（全部在 stateOnFrame / stateTickTouchRef 里推进）：
 *   IDLE      未采集
 *   SETTLE    等待 CT_REF_SETTLE_MS，让手完成动作
 *   AVERAGING 累加 CT_REF_AVG_MS 内的样本（1kHz 下约 500 个）
 *   （结算在 stateTickTouchRef 里做，因为它需要判断时间到了）
 */
enum CtRefPhase : uint8_t { CTREF_IDLE = 0, CTREF_SETTLE, CTREF_AVG };
static uint8_t  s_refPhase   = CTREF_IDLE;
static uint32_t s_refStartUs = 0;
static uint32_t s_refAvgN    = 0;
static double   s_refSum[3]  = {0, 0, 0};

bool stateStartTouchRef(void) {
  if (s_refPhase != CTREF_IDLE) return false;   // 已在采集中，忽略重复按键
  s_refPhase   = CTREF_SETTLE;
  s_refStartUs = 0;                             // 首帧时用样本时间戳初始化
  s_refAvgN    = 0;
  s_refSum[0] = s_refSum[1] = s_refSum[2] = 0.0;
  return true;
}

bool stateTouchRefCapturing(void) { return s_refPhase != CTREF_IDLE; }

/* 由主循环每圈调用：结算采集（用样本时间戳，不用 millis） */
void stateTickTouchRef(uint32_t nowUs) {
#if CT_MODE != 2
  /* 模式 3/4 使用 stateTickFlatRef；两套采集共用 s_refPhase，
   * 不能同时推进，否则平放基准会被接触姿态采集提前消费。 */
  (void)nowUs;
  return;
#else
  if (s_refPhase == CTREF_IDLE) return;
  if (s_refStartUs == 0) { s_refStartUs = nowUs; return; }

  const uint32_t elapsedMs = (nowUs - s_refStartUs) / 1000u;

  if (s_refPhase == CTREF_SETTLE) {
    if (elapsedMs >= (uint32_t)CT_REF_SETTLE_MS) {
      s_refPhase = CTREF_AVG;
      s_refAvgN  = 0;
      s_refSum[0] = s_refSum[1] = s_refSum[2] = 0.0;
    }
    return;
  }

  /* AVERAGING 阶段：样本累加在 stateOnFrame 里做，这里只判断时间到 */
  if (elapsedMs >= (uint32_t)(CT_REF_SETTLE_MS + CT_REF_AVG_MS)) {
    if (s_refAvgN > 0) {
      for (int a = 0; a < 3; a++) s_ctRef[a] = (float)(s_refSum[a] / (double)s_refAvgN);
      s_ctRefSet = true;
      s_r.ctRefSet = true;
      /* 判据已按真实标签数据验证（c2~c5.csv，准确率 99.13%），可以置为可信。
       * 注意仅限模式 2 —— 模式 0/1 仍保持 false。 */
      s_r.contactValid = true;
      /* 记录参考点后重新开始确认，避免用旧状态立刻翻转 */
      s_ctPosture = 0; s_ctCand = 0; s_ctCandUs = 0;
      s_r.contact = 0;
      s_r.ctTouchDist = 0.0f;
    }
    s_refPhase = CTREF_IDLE;
  }
#endif
}

void stateClearTouchRef(void) {
  s_ctRefSet = false;
  s_r.ctRefSet = false;
  s_r.contactValid = false;
  s_r.contact = 0;
  s_r.ctTouchDist = -1.0f;
  s_ctPosture = 0;
  s_ctCand = 0;
  s_ctCandUs = 0;
  /* 同时取消进行中的采集，避免"清掉后又被旧采集覆盖" */
  s_refPhase = CTREF_IDLE;
  s_refAvgN = 0;
}

bool stateTouchRefSet(void) { return s_ctRefSet; }

/* ============================================================
 * 模式 3：平放基准采集（带稳定性检查与自动重试）
 *
 * 【为什么需要这一整套，而不是像模式 2 那样记瞬时值】
 *   模式 2 由人手按 't' 触发，人自己知道"我刚握好"。
 *   模式 3 改成上电自动采平放基准（设备平放静置即可，免现场按键），
 *   于是多出一个风险：**采集期间有人搬动设备**。
 *   此时采到的是错误零点，此后所有姿态判定的距离都系统性偏移 ——
 *   这是最危险的一类错误：不报错，只让判定悄悄变错。
 *   因此必须能识别"采的时候在动"，并且识别出来后**拒绝**这份数据，
 *   而不是凑合使用。
 *
 * 【复用模式 2 的两阶段时序】
 *   SETTLE(300ms) 等设备放稳 -> AVG(500ms) 累加 -> 结算
 *   状态机变量复用 s_refPhase/s_refStartUs/s_refAvgN，
 *   但极差统计用小/最大独立记录（见 s_refMin/s_refMax）：
 *   平均值会掩盖抖动，必须看窗口内实际摆动了多少。
 *
 * 【失败与重试】
 *   极差超限 -> 作废 -> 若是自动流程启动的，则重试，最多 CT_REF_AUTO_RETRY 次
 *   重试用尽 -> flatRefSet 保持 false，contactValid 保持 false，
 *               contact 恒为 0。此时必须由人重新平放设备并手动重采
 *               （'t' 命令或 stateStartFlatRef()）。
 *   绝不在没有可信基准的情况下输出接触结论。
 */

bool stateStartFlatRef(void) {
  if (s_refPhase != CTREF_IDLE) return false;   // 已在采集中，忽略重复触发
  s_refPhase      = CTREF_SETTLE;
  s_refStartUs    = 0;                          // 首帧时用样本时间戳初始化
  s_refAvgN       = 0;
  s_refSum[0] = s_refSum[1] = s_refSum[2] = 0.0;
  s_refHaveSample = false;
  s_flatAutoArm   = false;                      // 手动触发：不自动重试
  return true;
}

bool stateFlatRefCapturing(void) { return s_refPhase != CTREF_IDLE; }

/* 结算：把累加的平均值落地，并做稳定性判定。
 * 返回 true 表示本次结算产出了一份可用基准。 */
static bool flatRefCommit(void) {
  if (s_refAvgN == 0 || !s_refHaveSample) return false;

  /* 极差 = 窗口内单轴 max - min，取三轴最大值作为"这次有多动"。 */
  float swing = 0.0f;
  for (int a = 0; a < 3; a++) {
    const float r = s_refMax[a] - s_refMin[a];
    if (r > swing) swing = r;
  }
  s_r.flatRefSwing = swing;

  if (swing > CT_REF_MAX_SWING_MG) {
    /* 采集中设备在动，这份基准不可信 —— 丢弃，不要"用大部分样本凑一个"。 */
    return false;
  }

  for (int a = 0; a < 3; a++) s_flatRef[a] = (float)(s_refSum[a] / (double)s_refAvgN);
  s_flatRefSet  = true;
  s_r.flatRefSet = true;
  /* 基准可信后接触判据才可信。注意：模式 3 的 contactValid 由
   * 平放基准决定，与模式 2 的接触姿态参考点无关，两者不会同时生效。 */
  s_r.contactValid = true;
  /* 拿到基准后重新开始确认，避免用旧状态立刻翻转 */
  s_ctPostCmd = 0; s_ampHeld = 0; s_freqHeld = 0;
#if CT_HOLD_ENABLE
  ctHoldClear();
#endif
  s_ctPosture = 0; s_ctCand = 0; s_ctCandUs = 0;
  s_r.contact = 0;
  s_r.flatDist = 0.0f;
  s_flatAutoFail = 0;
  /* 【2026-10-29】刚采到一份基准，静止计时必须归零。
   * 否则采集过程本身累积的静置时间（>= AUTO_ZERO_REST_MS）会让
   * 自动重采在下一圈立刻再次触发，把刚采到的基准清掉，形成"采了又清"的抖动。 */
  s_stillUs = 0;
  return true;
}

/* 由主循环每圈调用：自动启动 + 阶段推进 + 结算 + 重试。
 * 与 stateTickTouchRef 结构相同，多出"自动启动"和"失败重试"两步。 */
void stateTickFlatRef(uint32_t nowUs) {
#if CT_MODE != 3 && CT_MODE != 4
  (void)nowUs;
  return;
#else
#if AUTO_ZERO_ENABLE
  /* 【无机械限位时的兜底】静止足够久、没有振动、且角度已经回到张开位 ⇒ 自动重采基准。
   * 三个条件缺一不可：
   *   - 只看"静止"：夹着物体停住不动 2s 也会触发，会把闭合位当成张开位；
   *   - 只看"角度小"：还没回到张开位（例如刚松开一半）也会触发，零点偏；
   *   - 只看"幅值低"：张开但还在动的时候也会触发，基准采到运动中的姿态。
   * 触发后清基准，下一圈由下面的 CT_REF_AUTO_ENABLE 块自动重采。 */
  if (s_sinceClose && s_flatRefSet && s_refPhase == CTREF_IDLE &&
      s_stillUs >= (uint32_t)AUTO_ZERO_REST_MS * 1000u &&
      s_r.ampValid && s_r.ampMg < AUTO_ZERO_AMP_MG &&
      fabsf(s_r.closeAngleDeg) < AUTO_ZERO_ANGLE_DEG) {
    stateClearFlatRef();
    s_stillUs = 0;              // 防止连续触发
    s_sinceClose = false;       // 一次闭合只重采一次，避免静止时反复清基准
    s_angleDeg[0] = s_angleDeg[1] = s_angleDeg[2] = 0.0f;
  }
#endif

#if CT_REF_AUTO_ENABLE
  /* 自动启动：上电后若还没有基准、也不在采集中、且重试未用尽，就启动。 */
  if (!s_flatRefSet && s_refPhase == CTREF_IDLE &&
      s_flatAutoFail < (uint8_t)CT_REF_AUTO_RETRY) {
    if (stateStartFlatRef()) {
      s_flatAutoArm = true;   // 标记来源，允许自动重试
    }
  }
#endif

  if (s_refPhase == CTREF_IDLE) return;
  if (s_refStartUs == 0) { s_refStartUs = nowUs; return; }

  const uint32_t elapsedMs = (nowUs - s_refStartUs) / 1000u;

  if (s_refPhase == CTREF_SETTLE) {
    if (elapsedMs >= (uint32_t)CT_REF_SETTLE_MS) {
      s_refPhase      = CTREF_AVG;
      s_refAvgN       = 0;
      s_refSum[0] = s_refSum[1] = s_refSum[2] = 0.0;
      s_refHaveSample = false;
    }
    return;
  }

  /* AVERAGING 阶段：逐样本累加在 stateOnFrame 里做，这里只判断时间到 */
  if (elapsedMs >= (uint32_t)(CT_REF_SETTLE_MS + CT_REF_AVG_MS)) {
    const bool ok = flatRefCommit();
    s_refPhase = CTREF_IDLE;
    if (!ok && s_flatAutoArm) {
      s_flatAutoFail++;   // 自动流程：计数后由下一圈重新启动
    }
    s_flatAutoArm = false;
  }
#endif
}

void stateClearFlatRef(void) {
  s_flatRefSet   = false;
  s_r.flatRefSet = false;
  s_r.contactValid = false;
  s_r.contact    = 0;
  s_r.flatDist   = -1.0f;
  s_r.flatRefSwing = 0.0f;
  s_ctPostCmd = 0; s_ampHeld = 0; s_freqHeld = 0;
#if CT_HOLD_ENABLE
  ctHoldClear();
#endif
  s_ctPosture = 0; s_ctCand = 0; s_ctCandUs = 0;
  /* 同时取消进行中的采集，避免"清掉后又被旧采集覆盖" */
  s_refPhase    = CTREF_IDLE;
  s_refAvgN     = 0;
  s_refHaveSample = false;
  s_flatAutoFail = 0;
  s_flatAutoArm  = false;
}

bool stateFlatRefSet(void) { return s_flatRefSet; }

void stateApplyCal(const DspCal& cal) {
  s_r.calOk = cal.ok;
  s_r.calQuality = cal.quality;
  /* 重新标定会改变零偏，而参考点是在**旧零偏**下记录的三轴低通值，
   * 两者不在同一坐标系里，继续用会得到错误的距离。
   * 所以清掉参考点，要求重新记录一次（contactValid 同步回 false）。
   *
   * 【模式3 的平放基准同样必须清】它也是"三轴低通值"，
   * 零偏一变就不再对应新坐标系里的平放姿态。
   * 清掉后 stateTickFlatRef 会在下一圈自动重采，但前提是那一刻
   * 设备确实平放着 —— 所以重标定时应让设备保持平放，
   * 否则自动重采会拿到一个"不在平放"的零点。
   */
  s_ctRefSet       = false;
  s_flatRefSet     = false;
  s_r.ctRefSet     = false;
  s_r.flatRefSet   = false;
  s_r.contactValid = false;
  s_r.ctTouchDist  = -1.0f;
  s_r.flatDist     = -1.0f;
  s_ctDist         = -1.0f;
  s_flatDist       = -1.0f;
  s_ctPosture = 0; s_ctCand = 0; s_ctCandUs = 0;
  s_ctPostCmd = 0; s_ampHeld = 0; s_freqHeld = 0;
#if CT_HOLD_ENABLE
  ctHoldClear();
#endif
  s_r.contact = 0;
  s_r.gripNoObject = false;

  /* v5：换了一套陀螺零偏，之前积分的角度不再有意义，一并清零 */
  for (int a = 0; a < 3; a++) {
    s_gyroBias[a] = cal.gyroBias[a];
    s_angleDeg[a] = 0.0f;
  }
  s_stillUs = 0;
  s_angOpenHeld = 0; s_angAirHeld = 0;
  s_sinceClose = false;
  s_gyroClipLatched = false;
  s_r.closeAngleDeg = 0.0f;
  s_r.angleValid = false;
  /* 标定会改变零偏，进行中的参考点采集作废（采到的值在新坐标系里没意义） */
  s_refPhase = CTREF_IDLE;
  s_refAvgN = 0;
  s_refHaveSample = false;
  s_flatAutoFail = 0;
  s_flatAutoArm  = false;
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
  /* 【模式2】接触姿态参考点**跨 stateReset 保留**：
   *   它是选手手动记录的现场依据，不属于"一次测量"的状态。
   *   若在 startRun() 时被清掉，每次故障恢复后都要重新按键，现场很别扭。
   *   换机械手/换工位时才需要显式 stateClearTouchRef()。 */
  bool refSet = s_ctRefSet;
  float ref0 = s_ctRef[0], ref1 = s_ctRef[1], ref2 = s_ctRef[2];
  /* 【模式3】平放基准同理需要跨 reset 保留，理由与上面相同：
   *   它是"设备放在同一工位上"这个物理事实，不是一次测量结果。
   *   若每次故障恢复都重采，等于在不确定的时刻重新定义零点，
   *   反而更容易出错。 */
#if CT_MODE == 3 || CT_MODE == 4
  const bool  flatSet = s_flatRefSet;
  const float f0 = s_flatRef[0], f1 = s_flatRef[1], f2 = s_flatRef[2];
  const float flatSwingKeep = s_r.flatRefSwing;
  const uint8_t flatRetryKeep = s_flatAutoFail;
#endif
  float ax0 = s_lastCtAxis[0], ax1 = s_lastCtAxis[1], ax2 = s_lastCtAxis[2];

  s_r = Result{};
  s_r.sensorOk = sensorOk; s_r.calOk = calOk; s_r.calQuality = calQ;
  s_r.nStart = ns; s_r.nStop = np;
  s_hiCnt = s_loCnt = 0;
  s_ssVib = 0; s_ssHiCnt = s_ssLoCnt = 0;
  s_haveEdge = false;
  s_nPeriod = 0;
  s_haveWin = false;
  s_lastWinUs = 0;        // 原有代码清了 haveWin 却没清时间戳，一并补上
  s_haveEnv = false;      // 幅值时效
  s_lastEnvUs = 0;
  s_ctPosture = 0; s_ctCand = 0; s_ctCandUs = 0;
  s_r.freqClipLimited = false;   // 削顶降级标记
  s_lastEdgeUs = 0;              // 时间基准也清，避免逻辑变化后踩坑
  clearEdgeInterp();             // 边沿插值状态

  /* v5：角度从当前姿态重新起算（零偏保留，它是传感器事实，不是一次测量） */
  s_angleDeg[0] = s_angleDeg[1] = s_angleDeg[2] = 0.0f;
  s_stillUs = 0;
  s_lastGyroUs = 0;
  s_angOpenHeld = 0; s_angAirHeld = 0;
  s_gyroClipLatched = false;
  s_sinceClose = false;
  s_r.closeAngleDeg = 0.0f;
  s_r.angleValid = false;

  /* 恢复参考点 */
  s_ctRefSet = refSet;
  s_ctRef[0] = ref0; s_ctRef[1] = ref1; s_ctRef[2] = ref2;
  s_lastCtAxis[0] = ax0; s_lastCtAxis[1] = ax1; s_lastCtAxis[2] = ax2;
  s_ctDist = -1.0f;
#if CT_MODE == 3 || CT_MODE == 4
  s_flatRefSet  = flatSet;
  s_flatRef[0] = f0; s_flatRef[1] = f1; s_flatRef[2] = f2;
  s_r.flatRefSet   = flatSet;
  s_r.flatRefSwing = flatSwingKeep;
  s_r.flatRefRetry = flatRetryKeep;
  s_ctPostCmd = 0; s_ampHeld = 0; s_freqHeld = 0;
#if CT_HOLD_ENABLE
  ctHoldClear();
#endif
  s_flatDist = -1.0f;
  /* 模式 3：有平放基准才可信；还没采到基准就仍是"不知道"。
   * 与模式 2 同一原则，不因 reset 而把"不知道"当成"未接触"。 */
  s_r.contactValid = flatSet;
  s_r.ctRefSet = false;
#elif CT_MODE == 2
  /* 模式 2：判据已由真实标签数据验证，有参考点即可信；无参考点不可信 */
  s_r.contactValid = refSet;
  s_r.ctRefSet = refSet;
#else
  /* 模式 0/1：实测会误报，一律不可信 */
  s_r.contactValid = false;
  s_r.ctRefSet = false;
#endif
  s_r.ctTouchDist = -1.0f;
  /* 不清参考点采集状态：stateReset 由 startRun 调用，
   * 若此时正在采集，贸然取消会让选手白按一次。采集本身不依赖测量状态。 */
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
#if SS_EDGE_FIX_ENABLE
  s_ssVib = 1;   // 缺口后先要确认一次下降，防止 ON 段中途插进假起振边沿
#else
  s_ssVib = 0;
#endif
  s_ssHiCnt = 0;
  s_ssLoCnt = 0;
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

  /* ② 逐样本保存三轴低通值：
   *    - 模式 2 需要它算"到接触姿态的距离"
   *    - stateSetTouchRef() 需要它取"当前姿态"作为参考点
   *    必须逐样本更新（不能只在 envReady 时更新），否则按键记录时
   *    拿到的可能是 30ms 前的旧值，参考点会有偏差。 */
  s_lastCtAxis[0] = f.ctAxis[0];
  s_lastCtAxis[1] = f.ctAxis[1];
  s_lastCtAxis[2] = f.ctAxis[2];

  /* 参考点采集：AVERAGING 阶段逐样本累加（1kHz，不用停采样器）。
   * 模式 2 与模式 3 共用这一段：两者都只是"在某个窗口内对三轴低通值
   * 求平均"，区别只在窗口的起止时刻由谁触发、以及结算时怎么校验。
   * 模式 3 额外在这里维护逐轴 min/max（见下方 #if CT_MODE == 3）。 */
  if (s_refPhase == CTREF_AVG) {
    s_refSum[0] += f.ctAxis[0];
    s_refSum[1] += f.ctAxis[1];
    s_refSum[2] += f.ctAxis[2];
    s_refAvgN++;
#if CT_MODE == 3 || CT_MODE == 4
    /* 逐轴极值统计：首样本直接初始化，后续逐个比较。
     * 这是"采集期间是否在动"的唯一证据来源，不能省。 */
    if (!s_refHaveSample) {
      s_refMin[0] = s_refMax[0] = f.ctAxis[0];
      s_refMin[1] = s_refMax[1] = f.ctAxis[1];
      s_refMin[2] = s_refMax[2] = f.ctAxis[2];
      s_refHaveSample = true;
    } else {
      for (int a = 0; a < 3; a++) {
        if (f.ctAxis[a] < s_refMin[a]) s_refMin[a] = f.ctAxis[a];
        if (f.ctAxis[a] > s_refMax[a]) s_refMax[a] = f.ctAxis[a];
      }
    }
#endif
  }

#if CT_MODE == 2
  /* 到接触姿态的距离。无参考点时置 -1（表示"不知道"，不是 0）。 */
  if (s_ctRefSet) {
    const float dx = f.ctAxis[0] - s_ctRef[0];
    const float dy = f.ctAxis[1] - s_ctRef[1];
    const float dz = f.ctAxis[2] - s_ctRef[2];
    s_ctDist = sqrtf(dx * dx + dy * dy + dz * dz);
    s_r.ctTouchDist = s_ctDist;
  } else {
    s_ctDist = -1.0f;
    s_r.ctTouchDist = -1.0f;
  }
#elif CT_MODE == 3 || CT_MODE == 4
  /* 到**平放**姿态的距离。与模式 2 同一套算法，只换了参考点。
   * 注意不能复用 s_ctDist：两者可能在切换模式时并存，混用会让
   * "切换模式后第一帧"拿旧模式的距离参与新判据。 */
  if (s_flatRefSet) {
    const float dx = f.ctAxis[0] - s_flatRef[0];
    const float dy = f.ctAxis[1] - s_flatRef[1];
    const float dz = f.ctAxis[2] - s_flatRef[2];
    s_flatDist = sqrtf(dx * dx + dy * dy + dz * dz);
    s_r.flatDist = s_flatDist;
  } else {
    s_flatDist = -1.0f;
    s_r.flatDist = -1.0f;
  }
#endif

  /* v5：闭合角度必须**逐样本**更新。
   * 注意不能放进下面的 if (f.envReady)：那个分支每 ENV_HOP(10) 个样本才进来一次，
   * 而陀螺数据本身已经是每 GYRO_DIV(4) 个样本才有一个，两者叠加会把积分次数再砍到 1/10。 */
  angleStep(f, tUs);

  if (f.envReady) {
    s_r.ampMg = f.env;
    // ③ 恢复条件看"包络窗内有无削顶"，而不是"当前样本是否削顶"：
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
