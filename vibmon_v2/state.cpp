// 状态判断模块实现。
//
// 【四个并行的判定逻辑，各自独立维护状态】
//   ① ssStep()       启停状态机：双阈值 + 连续确认 → vib；翻转时记录"起"边沿
//   ② ssOnStartEdge() 启停频率：把相邻"起"边沿的间隔当作周期，攒够后取中位数
//   ③ ctStep()       接触判定：姿态偏差（模式1 下也算振动）→ 滞回 + 时间确认 → contact
//   ④ stateTick()    时效检查：主频太久没更新则失效、启停太久没新边沿则失效
// 这些逻辑都靠"当前时间"做判断，所以每个函数都要传入 tUs（微秒）。
#include "state.h"
#include "config.h"
#include <math.h>

// 当前阈值。初值取 config.h 的下限值，这样即使标定失败也有可用的保守阈值，
// 不会出现"阈值为 0 导致任何噪声都算振动"的情况。
static Thresholds s_th = {TH_HI_MIN_MG, TH_LO_MIN_MG, CT_MIN_MG, CT_MIN_MG * CT_OFF_RATIO};
// 对外结果。所有字段由本文件维护，外部只读。
static Result     s_r = {};

/* ---------- 启停状态机 ---------- */
static uint8_t  s_hiCnt = 0, s_loCnt = 0;   // 高/低阈值的连续确认计数（达到 SS_CONFIRM 才翻转）
static bool     s_haveEdge = false;         // 是否已记录过"起"边沿（首个边沿只作基准，不产生周期）
static uint32_t s_lastEdgeUs = 0;           // 上一个"起"边沿的时刻
static float    s_periodMs[SS_EDGE_MAX];    // 历史周期（ms），只追加不覆盖，满时整体前移
static uint8_t  s_nPeriod = 0;              // s_periodMs 中的有效个数

/* ---------- 主频时效 ---------- */
static bool     s_haveWin = false;          // 是否收到过任何主频窗结果
static uint32_t s_lastWinUs = 0;            // 最近一次主频窗结果的时刻，用于判断是否过期

/* ---------- 接触确认 ---------- */
static uint8_t  s_ctPosture = 0;            // 姿态支路自己的判定结果（滞回后），未经时间确认
static uint8_t  s_ctCand = 0;               // 正在确认中的候选状态
static uint32_t s_ctCandUs = 0;             // 候选状态首次出现的时刻，用于计确认时长

// 求中位数（拷贝一份再排序，不修改原数组——原数组要留着继续累积）。
// 与 dsp.cpp 的 medianOf 同名但用途不同：这里必须保留原数组，所以多一次拷贝。
static float medianCopy(const float* a, int n) {
  float t[SS_EDGE_MAX];
  for (int i = 0; i < n; i++) t[i] = a[i];
  for (int i = 1; i < n; i++) {          // 插入排序（n ≤ 4，开销可忽略）
    float v = t[i];
    int j = i - 1;
    while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
    t[j + 1] = v;
  }
  return (n & 1) ? t[n / 2] : 0.5f * (t[n / 2 - 1] + t[n / 2]);
}

// 清空周期统计并作废启停频率。三种情况会调用：节奏中断、判定不稳、超时。
static void ssClearPeriods(void) { s_nPeriod = 0; s_r.ssValid = false; s_r.ssHz = 0; s_r.ssPeriods = 0; }

// 用最近若干个周期评估启停频率。
// 【为什么只取最近 SS_USE_PERIODS 个】启停频率是"用户当前的操作节奏"，
// 早期周期不代表现在的节奏，取太久远的历史会让读数反应迟钝。
// 【为什么要一致性检查】若最近几个周期长短悬殊（max/min > SS_CONSIST_RATIO），
// 说明节奏本身不稳定，此时给出"平均频率"是误导，不如明确报告"未确立"。
// 【为什么用中位数】用户手抖一次会产生一个异常周期，中位数能把它排除掉。
static void ssEvaluate(void) {
  // 实际使用的周期数：受"已收集数"和"上限"双重约束
  int n = s_nPeriod < SS_USE_PERIODS ? s_nPeriod : SS_USE_PERIODS;
  s_r.ssPeriods = (uint8_t)n;
  if (n < SS_MIN_PERIODS) { s_r.ssValid = false; s_r.ssHz = 0; return; }   // 周期数不够
  const float* p = &s_periodMs[s_nPeriod - n];   // 指向最近 n 个周期（数组尾部）
  float pmin = p[0], pmax = p[0];
  for (int i = 1; i < n; i++) { pmin = fminf(pmin, p[i]); pmax = fmaxf(pmax, p[i]); }
  if (pmax > pmin * SS_CONSIST_RATIO) { s_r.ssValid = false; s_r.ssHz = 0; return; }   // 节奏不稳
  float hz = 1000.0f / medianCopy(p, n);         // 周期(ms) → 频率(Hz)
  s_r.ssValid = isfinite(hz);                    // 防御除零/异常
  s_r.ssHz = s_r.ssValid ? hz : 0.0f;
}

// 记录一次"起"边沿，并更新启停频率。
// 【流程】
//   ① 若之前已有边沿，算出与上一个边沿的间隔 pMs；
//   ② 间隔落在 [SS_T_MIN_MS, SS_T_MAX_MS] 之外 → 视为节奏中断（抖动或停太久），
//      清空历史周期重新开始统计；注意此时仍会更新 s_lastEdgeUs（见函数末尾），
//      也就是把这次边沿当作新的计时基准，而不是继续拿很久以前的那个算周期；
//   ③ 间隔正常 → 追加进数组；数组满时整体前移一位（丢弃最老的周期），保持固定内存；
//   ④ 重新评估启停频率。
// 首次调用（s_haveEdge=false）不产生周期，只建立基准。
static void ssOnStartEdge(uint32_t tUs) {
  if (s_haveEdge) {
    float pMs = (float)(uint32_t)(tUs - s_lastEdgeUs) / 1000.0f;   // 无符号差值，跨回绕安全
    if (pMs < SS_T_MIN_MS || pMs > SS_T_MAX_MS) {
      ssClearPeriods();             // 抖动或间隔过长：节奏中断，重新累计
    } else {
      if (s_nPeriod >= SS_EDGE_MAX) {
        // 数组已满：整体左移一位，腾出末尾位置。SS_EDGE_MAX 只有 16，
        // 这种 O(n) 搬移每次边沿最多发生一次，开销可忽略
        for (int i = 1; i < SS_EDGE_MAX; i++) s_periodMs[i - 1] = s_periodMs[i];
        s_nPeriod = SS_EDGE_MAX - 1;
      }
      s_periodMs[s_nPeriod++] = pMs;
      ssEvaluate();
    }
  }
  // 无论上面走哪条分支，都要把基准推进到本次边沿
  s_haveEdge = true;
  s_lastEdgeUs = tUs;
}

// 启停状态机单步。输入是包络值（100Hz 调用一次）。
// 【双阈值滞回 + 连续确认】
//   停 → 振：包络连续 SS_CONFIRM 次高于 hi 才翻转（过滤单点毛刺）
//   振 → 停：包络连续 SS_CONFIRM 次低于 lo 才翻转
// 中间地带（lo ≤ env ≤ hi）不满足任何一侧条件，计数被清零，状态保持不变。
// 【翻转时的动作】转"振"时记一次"起"边沿并累加 nStart；
// 转"停"时立即作废主频（振动已停止，旧的主频读数不再代表当前状态）并累加 nStop。
static void ssStep(float env, uint32_t tUs) {
  if (s_r.vib == 0) {
    if (env > s_th.hi) {
      if (++s_hiCnt >= SS_CONFIRM) {
        s_hiCnt = 0; s_r.vib = 1; s_r.nStart++;
        ssOnStartEdge(tUs);
      }
    } else s_hiCnt = 0;          // 一旦跌回阈值内，连续计数立即清零
  } else {
    if (env < s_th.lo) {
      if (++s_loCnt >= SS_CONFIRM) {
        s_loCnt = 0; s_r.vib = 0; s_r.nStop++;
        s_r.freqValid = false; s_r.freqHz = 0;
      }
    } else s_loCnt = 0;
  }
}

// 接触判定单步。输入是姿态偏差（100Hz 调用一次）。
// 【流程】
//   ① 姿态支路自身先做滞回判定：dev > ctOn 置 1，dev < ctOff 清 0，
//      中间地带保持原值 → 得到 s_ctPosture；
//   ② 按 CT_MODE 决定"原始判定"：模式 1 下，只要在振动就直接算接触
//      （手按上去通常既改变姿态也带轻微振动，两者取或更灵敏）；
//   ③ 时间确认：只有候选状态持续 CT_CONFIRM_MS 才真正写入 s_r.contact。
//      确认期比启停长（100ms vs 30ms），因为姿态支路本身响应慢（时间常数 0.2s），
//      快速确认没有意义；但也不能太长，否则手感迟钝。
// 【确认逻辑的细节】若原始判定回到与当前 contact 一致，说明是抖动，直接撤销候选；
// 若候选状态变了，重新计时；只有候选稳定超过阈值时长才提交。
static void ctStep(float dev, uint32_t tUs) {
  if (!s_ctPosture && dev > s_th.ctOn) s_ctPosture = 1;
  else if (s_ctPosture && dev < s_th.ctOff) s_ctPosture = 0;

  uint8_t raw = s_ctPosture;
#if CT_MODE == 1
  if (s_r.vib) raw = 1;          // 模式 1：振动中一律视为接触
#endif
  if (raw == s_r.contact) { s_ctCand = raw; return; }        // 与当前状态相同，无需确认
  if (raw != s_ctCand) { s_ctCand = raw; s_ctCandUs = tUs; return; }   // 新候选，重新计时
  if ((uint32_t)(tUs - s_ctCandUs) >= (uint32_t)CT_CONFIRM_MS * 1000u) s_r.contact = raw;   // 确认通过
}

// 由标定结果计算全部阈值。
// 【核心思想】阈值不是写死的绝对值，而是"实测静置值 × 倍数"：
//   启停高阈值 = max(下限, TH_HI_K × 静置包络)
//   启停低阈值 = max(下限, TH_LO_K × 静置包络)
//   接触阈值   = max(下限, CT_K   × 静置姿态偏差)
// 这样同一份固件在不同噪声水平的板子上都能工作：噪声大的板子阈值自动抬高。
// 下限（TH_*_MIN_MG / CT_MIN_MG）用于兜底——若标定时噪声极小，纯倍数算出的阈值
// 会低到被噪声触发，必须有个绝对下限压住。
// 【注意】标定失败（cal.ok=false）时直接返回，保留上一次/默认阈值，
// 避免用一份无效的标定结果把阈值改坏。
void stateApplyCal(const DspCal& cal) {
  s_r.calOk = cal.ok;
  s_r.calQuality = cal.quality;
  if (!cal.ok) return;
  s_th.hi = fmaxf(TH_HI_MIN_MG, TH_HI_K * cal.envRest);
  s_th.lo = fmaxf(TH_LO_MIN_MG, TH_LO_K * cal.envRest);
  s_th.ctOn = fmaxf(CT_MIN_MG, CT_K * cal.ctRest);
  s_th.ctOff = s_th.ctOn * CT_OFF_RATIO;    // 释放阈值恒为触发阈值的固定比例，保证滞回
}

// 清空结果与所有判定状态，但保留三样东西：
//   · 传感器/标定标志（sensorOk/calOk/calQuality）——它们描述的是系统配置，不是本次测量
//   · 累计次数（nStart/nStop）——现场核对操作次数用，不应被状态复位抹掉
// 其余字段（振动、主频、启停、接触等）全部归零，所有内部计数器与边沿基准一并重置。
void stateReset(void) {
  bool sensorOk = s_r.sensorOk, calOk = s_r.calOk, calQ = s_r.calQuality;
  uint32_t ns = s_r.nStart, np = s_r.nStop;
  s_r = Result{};                                  // 整体清零
  s_r.sensorOk = sensorOk; s_r.calOk = calOk; s_r.calQuality = calQ;
  s_r.nStart = ns; s_r.nStop = np;
  s_hiCnt = s_loCnt = 0;
  s_haveEdge = false;                              // 下一个边沿重新作基准
  s_nPeriod = 0;
  s_haveWin = false;
  s_ctPosture = 0; s_ctCand = 0;
}

// 每样本喂入。dsp 的每个 DspFrame 都会调一次，但只有在对应 Ready 标志为真时才动作：
//   envReady（100Hz）：刷新幅值与姿态偏差，驱动启停与接触两个状态机
//   winReady（15.6Hz）：刷新主频。注意 freqValid 要求 vib==1——
//     主频窗在静置时也可能算出某个频率（纯噪声的过零），只有确实在振动时才认它
void stateOnFrame(const DspFrame& f, uint32_t tUs) {
  if (f.envReady) {
    s_r.ampMg = f.env;
    s_r.ctDev = f.ctDev;
    ssStep(f.env, tUs);
    ctStep(f.ctDev, tUs);
  }
  if (f.winReady) {
    s_haveWin = true;
    s_lastWinUs = tUs;
    s_r.axis = f.axis;
    s_r.freqValid = (s_r.vib == 1) && f.freqHz > 0.0f;
    s_r.freqHz = s_r.freqValid ? f.freqHz : 0.0f;
  }
}

// 时效检查。由主循环按自己的节奏调用（不受样本节拍约束）。
// 【为什么需要它】状态机只在收到数据时才更新。如果振动停止、或用户不再操作，
// 就再也不会有新数据来把旧读数清掉，界面会一直显示一个早已过期的值。
// 这个函数就是"没人来更新时主动作废"的兜底。
//   ① 主频过期：超过 FREQ_STALE_MS 没有新窗结果 → 主频失效
//   ② 启停超时：距离上一个"起"边沿太久 → 清空周期、作废启停频率
//      超时门限是自适应的：已知频率时取 SS_TIMEOUT_K 倍当前周期（启停快则快速失效），
//      未知频率时取 SS_T_MAX_MS（给足等待时间）
//   ③ 若连 SS_T_MAX_MS 都超过了，说明这一轮操作彻底结束，连边沿基准也一并丢弃
void stateTick(uint32_t nowUs) {
  if (s_r.freqValid && (!s_haveWin || (uint32_t)(nowUs - s_lastWinUs) > (uint32_t)FREQ_STALE_MS * 1000u)) {
    s_r.freqValid = false; s_r.freqHz = 0;
  }
  if (s_haveEdge) {
    float sinceMs = (float)(uint32_t)(nowUs - s_lastEdgeUs) / 1000.0f;
    // 自适应超时：已有有效启停频率时用它的 K 倍，否则用上限
    float limit = s_r.ssValid ? SS_TIMEOUT_K * 1000.0f / s_r.ssHz : (float)SS_T_MAX_MS;
    // 两个条件都要满足：既超过自适应门限，也不能短于最小周期（避免刚记录边沿就立刻作废）
    if (sinceMs > limit && sinceMs > (float)SS_T_MIN_MS) {
      ssClearPeriods();
      if (sinceMs > (float)SS_T_MAX_MS) s_haveEdge = false;   // 太久没动，基准也丢弃
    }
  }
}

// 设置传感器在线状态。掉线时清空全部结果——数据源已经不可信，
// 保留旧读数比显示"无数据"更危险（会被误读成当前状态）。
// 注意：只在状态真正变化时才动作，避免每次调用都做一次无谓的复位。
void stateSetSensorOk(bool ok) {
  if (s_r.sensorOk == ok) return;
  s_r.sensorOk = ok;
  if (!ok) stateReset();
}

void stateClearCounters(void) { s_r.nStart = 0; s_r.nStop = 0; }

const Result&     stateResult(void) { return s_r; }
const Thresholds& stateThresholds(void) { return s_th; }
