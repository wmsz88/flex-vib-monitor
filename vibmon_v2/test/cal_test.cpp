// 标定事务化测试（Q1 / T01~T04）：链接真实 dsp.cpp + state.cpp。
//
// 编译（旧代码，验证"红"）：
//   g++ -std=c++17 -Wall -I test -I . test/cal_test.cpp dsp.cpp state.cpp -o test/cal_test.exe
// 编译（新代码，验证"绿"，多一个宏启用报告断言）：
//   g++ -std=c++17 -Wall -DCAL_TEST_HAS_REPORT -I test -I . test/cal_test.cpp dsp.cpp state.cpp -o test/cal_test.exe
//
// 设计要点：
//   * 阶段A 由 mpuReadAccel 直接轮询，阶段B 由 samplerPop 喂样本，两个数据源**独立**
//     （g_gravA / g_gravB），才能构造"阶段A 结束后模块被转动"。
//   * 量程从 mpuLsbPerG() 推导，不写死 16.384，改 ACCEL_FS_SEL 不会静默错一倍。
//   * 逐字段精确比较，不用浮点近似：同一份配置复制不应有任何位变化。
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "Arduino.h"
#include "dsp.h"
#include "state.h"
#include "config.h"

/* 新参数在实现落地前先用兜底值，保证旧代码也能编译出"红" */
#ifndef CAL_MIN_SAMPLES
#define CAL_MIN_SAMPLES 200
#endif
#ifndef CAL_MAX_ENV_MG
#define CAL_MAX_ENV_MG 50.0f
#endif
#ifndef CAL_MAX_CT_MG
#define CAL_MAX_CT_MG 50.0f
#endif

/* ================= 虚拟时间 ================= */
static uint32_t g_us = 0;
uint32_t millis(void) { return g_us / 1000u; }
uint32_t micros(void) { return g_us; }

/* ================= 量程（与固件同源） ================= */
static float g_lsbPerG = 16384.0f;
float mpuLsbPerG(void) { return g_lsbPerG; }
static inline int16_t mg2raw(float mg) { return (int16_t)lrintf(mg * g_lsbPerG / 1000.0f); }

/* ================= 阶段A 数据源 ================= */
static float g_gravA[3] = {0, 0, 1000};
static float g_noiseA   = 0.0f;      // 阶段A 读数抖动（模拟标定时被碰）
static bool  g_mpuFailA = false;     // 阶段A 读失败
static int   g_stageBRuns = 0;       // 阶段B 被启动的次数（断言用）

bool mpuReadAccel(MpuRaw* o) {
  if (g_mpuFailA) return false;
  float n = g_noiseA ? ((rand() % 2001) - 1000) / 1000.0f * g_noiseA : 0.0f;
  o->x = mg2raw(g_gravA[0] + n);
  o->y = mg2raw(g_gravA[1] + n);
  o->z = mg2raw(g_gravA[2] + n);
  return true;
}
uint32_t mpuErrCount(void) { return 0; }
void     mpuClearErr(void) {}

/* ================= 阶段B 数据源 ================= */
static float    g_gravB[3] = {0, 0, 1000};
static bool     g_feedEnable = true;   // false = 阶段B 断流
static bool     g_clipB      = false;  // 阶段B 制造削顶
static bool     g_stageBOn   = false;  // samplerStart/Stop 之间
static int      g_feedLeft   = 0;      // 本轮还能吐多少样本
static uint32_t g_feedSeq    = 0;

#define FEED_BURST 16

// 第 2 步：接口从 void 改为枚举/bool，桩跟着改返回值（不动测试逻辑）
SamplerStartResult samplerStart(void) {
  g_stageBOn = true; g_stageBRuns++; g_feedLeft = FEED_BURST;
  return SMP_START_OK;
}
SamplerStopResult samplerStop(void) {
  g_stageBOn = false; g_feedLeft = 0;
  return SMP_STOP_OK;
}
bool samplerReset(void) { g_feedSeq = 0; return true; }
uint32_t samplerInFlight(void) { return 0; }
bool samplerStartIsError(SamplerStartResult r) {
  return r == SMP_START_ERR_CREATE || r == SMP_START_ERR_START;
}
bool samplerStopIsError(SamplerStopResult r) { return r == SMP_STOP_TIMEOUT; }
const char* samplerStartStr(SamplerStartResult) { return ""; }
const char* samplerStopStr(SamplerStopResult)   { return ""; }

bool samplerPop(Sample* s, SamplerGap* gap) {
  if (!g_stageBOn || !g_feedEnable || g_feedLeft <= 0) return false;
  g_feedLeft--;
  g_us += 1000000u / FS_HZ;

  float n = ((rand() % 2001) - 1000) / 1000.0f * 3.0f;   // ±3mg 底噪
  s->seq = g_feedSeq++;
  s->tUs = g_us;
  s->raw.x = mg2raw(g_gravB[0] + n);
  s->raw.y = mg2raw(g_gravB[1] + n);
  // 削顶：|raw| 必须严格大于 ACCEL_CLIP_RAW，且不能溢出 int16
  s->raw.z = g_clipB ? (int16_t)32767 : mg2raw(g_gravB[2] + n);
  gap->kind = SMP_GAP_NONE;
  gap->missed = 0;
  gap->spanUs = 1000000u / FS_HZ;
  gap->devUs = 0;
  return true;
}

/* 阶段B 的循环体每轮 pop 到空后调 delay(1)，在这里补下一批 */
void delay(uint32_t ms) {
  g_us += ms * 1000u;
  if (g_stageBOn) g_feedLeft = FEED_BURST;
}

static SamplerStats g_stats = {};
/* 第 5 步：dsp/state 不直接用，但保持桩完整，避免链接期才发现 */
static TimingStats g_stubTiming = {};
const TimingStats& samplerTiming(void) { return g_stubTiming; }
void samplerTimingReset(void) { g_stubTiming = TimingStats(); }
void samplerNoteLoopUs(uint32_t) {}
void samplerNoteOutUs(uint32_t) {}
void samplerAddClip(void) { g_stats.clip++; }
const SamplerStats& samplerStats(void) { return g_stats; }
void samplerClearStats(void) { g_stats = SamplerStats(); }

/* ================= 断言 ================= */
static int g_fail = 0;
static int g_total = 0;

#define CHECK(cond, ...) do {                                   \
  g_total++;                                                    \
  if (!(cond)) { g_fail++; printf("[FAIL] "); }                 \
  else          { printf("[PASS] "); }                          \
  printf(__VA_ARGS__); printf("\n");                            \
} while (0)

#define NOTE(...) do { printf("       "); printf(__VA_ARGS__); printf("\n"); } while (0)

/* ================= 比较辅助 ================= */
static bool calEqual(const DspCal& a, const DspCal& b) {
  if (a.ok != b.ok || a.quality != b.quality) return false;
  if (a.nBias != b.nBias) return false;
  if (a.biasNormMg != b.biasNormMg) return false;
  if (a.envRest != b.envRest) return false;
  if (a.ctRest  != b.ctRest)  return false;
  if (a.zcl     != b.zcl)     return false;
  for (int i = 0; i < 3; i++) {
    if (a.bias[i]    != b.bias[i])    return false;
    if (a.rawStd[i]  != b.rawStd[i])  return false;
    if (a.sigBand[i] != b.sigBand[i]) return false;
  }
  return true;
}

static bool thEqual(const Thresholds& a, const Thresholds& b) {
  return a.hi == b.hi && a.lo == b.lo && a.ctOn == b.ctOn && a.ctOff == b.ctOff;
}

struct Snap { DspCal cal; Thresholds th; };

static void snap(Snap* s) { s->cal = dspCal(); s->th = stateThresholds(); }

/* 打印"哪几个字段变了"，红的时候直接看出污染点。
   用 %.6g：既能看出 999.51 与 1000 的差别，也不会把 1000.0 打成 1000.000000 */
static void diffCal(const DspCal& before, const DspCal& after) {
  if (before.nBias != after.nBias)
    NOTE("  nBias      %lu -> %lu   <<< 被改", (unsigned long)before.nBias, (unsigned long)after.nBias);
  if (before.ok != after.ok)
    NOTE("  ok         %d -> %d       <<< 被改", before.ok, after.ok);
  if (before.quality != after.quality)
    NOTE("  quality    %d -> %d       <<< 被改", before.quality, after.quality);
  if (before.biasNormMg != after.biasNormMg)
    NOTE("  |g|        %.6g -> %.6g   <<< 被改", before.biasNormMg, after.biasNormMg);
  for (int i = 0; i < 3; i++) {
    if (before.bias[i] != after.bias[i])
      NOTE("  bias[%d]    %.6g -> %.6g <<< 被改", i, before.bias[i], after.bias[i]);
    if (before.rawStd[i] != after.rawStd[i])
      NOTE("  rawStd[%d]  %.6g -> %.6g <<< 被改", i, before.rawStd[i], after.rawStd[i]);
    if (before.sigBand[i] != after.sigBand[i])
      NOTE("  sigBand[%d] %.6g -> %.6g <<< 被改", i, before.sigBand[i], after.sigBand[i]);
  }
  if (before.envRest != after.envRest) NOTE("  envRest    %.6g -> %.6g <<< 被改", before.envRest, after.envRest);
  if (before.ctRest  != after.ctRest)  NOTE("  ctRest     %.6g -> %.6g <<< 被改", before.ctRest,  after.ctRest);
  if (before.zcl     != after.zcl)     NOTE("  zcl        %.6g -> %.6g <<< 被改", before.zcl,     after.zcl);
}

static void diffTh(const Thresholds& before, const Thresholds& after) {
  if (before.hi    != after.hi)    NOTE("  阈值 hi    %.2f -> %.2f <<< 被改", before.hi,    after.hi);
  if (before.lo    != after.lo)    NOTE("  阈值 lo    %.2f -> %.2f <<< 被改", before.lo,    after.lo);
  if (before.ctOn  != after.ctOn)  NOTE("  阈值 ctOn  %.2f -> %.2f <<< 被改", before.ctOn,  after.ctOn);
  if (before.ctOff != after.ctOff) NOTE("  阈值 ctOff %.2f -> %.2f <<< 被改", before.ctOff, after.ctOff);
}

/* 模拟 vibmon_v2.ino 的调用约定：只有成功才应用阈值 */
static bool runCal(void) {
  bool ok = dspCalibrate();
  if (ok) stateApplyCal(dspCal());
  return ok;
}

static void resetScenario(void) {
  g_noiseA = 0.0f;
  g_mpuFailA = false;
  g_feedEnable = true;
  g_clipB = false;
  g_gravA[0] = 0; g_gravA[1] = 0; g_gravA[2] = 1000;
  g_gravB[0] = 0; g_gravB[1] = 0; g_gravB[2] = 1000;
  g_stageBRuns = 0;
  g_stageBOn = false;
  g_feedLeft = 0;
  samplerClearStats();
}

#ifdef CAL_TEST_HAS_REPORT
static const char* resName(CalResult r) { return calResultStr(r); }
#endif

/* ================= 用例 ================= */
int main(void) {
  g_lsbPerG = 16384.0f / (float)(1 << (ACCEL_FS_SEL & 0x03));
  srand(20261017);

  printf("=== cal_test：Q1 标定事务化 ===\n");
  printf("ACCEL_FS_SEL=%d  lsb/g=%.0f  CAL_MAX_STD_MG=%.0f  CAL_G_TOL_MG=%.0f\n",
         ACCEL_FS_SEL, g_lsbPerG, CAL_MAX_STD_MG, CAL_G_TOL_MG);
#ifdef CAL_TEST_HAS_REPORT
  printf("模式：新代码（含报告断言）\n");
#else
  printf("模式：旧代码（报告断言已跳过）\n");
#endif
  printf("\n");

  dspInit();
  stateReset();
  stateSetSensorOk(true);

  const Thresholds kDefaultTh = {TH_HI_MIN_MG, TH_LO_MIN_MG, CT_MIN_MG, CT_MIN_MG * CT_OFF_RATIO};

  /* ---------- T03c：首次上电，标定失败 ---------- */
  printf("--- T03c 首次上电（无旧标定）阶段A 失败 ---\n");
  resetScenario();
  g_mpuFailA = true;
  {
    bool ok = runCal();
    const DspCal& c = dspCal();
    const Thresholds& t = stateThresholds();
    CHECK(!ok, "标定返回 false");
    CHECK(!c.ok, "cal.ok == false（未标定，不冒充已标定）");
    CHECK(c.nBias == 0 && c.bias[0] == 0 && c.bias[1] == 0 && c.bias[2] == 0,
          "s_cal 全零（nBias=%lu bias=%.1f/%.1f/%.1f）",
          (unsigned long)c.nBias, c.bias[0], c.bias[1], c.bias[2]);
    CHECK(!stateResult().calOk, "Result.calOk == false");
    CHECK(thEqual(t, kDefaultTh), "阈值 == 编译期默认 %.1f/%.1f/%.1f/%.1f",
          t.hi, t.lo, t.ctOn, t.ctOff);
  }
  printf("\n");

  /* ---------- T04：干净标定成功，参数自洽 ---------- */
  printf("--- T04 干净标定成功，偏置/噪声/阈值同源 ---\n");
  resetScenario();
  {
    bool ok = runCal();
    const DspCal& c = dspCal();
    const Thresholds& t = stateThresholds();
    CHECK(ok, "标定返回 true");
    CHECK(c.ok, "cal.ok == true");
    CHECK(c.quality, "cal.quality == true（静置合格）");
    CHECK(fabsf(c.biasNormMg - 1000.0f) < 50.0f, "|g| = %.1f mg ≈ 1000", c.biasNormMg);
    CHECK(c.nBias >= (uint32_t)CAL_MIN_SAMPLES, "阶段A 样本数 %lu >= %d",
          (unsigned long)c.nBias, CAL_MIN_SAMPLES);
    CHECK(t.hi == fmaxf(TH_HI_MIN_MG, TH_HI_K * c.envRest),
          "hi == max(%.1f, %.1f×%.2f) = %.2f",
          TH_HI_MIN_MG, TH_HI_K, c.envRest, t.hi);
    CHECK(t.lo == fmaxf(TH_LO_MIN_MG, TH_LO_K * c.envRest), "lo == max(%.1f, %.1f×%.2f) = %.2f",
          TH_LO_MIN_MG, TH_LO_K, c.envRest, t.lo);
    CHECK(t.ctOn == fmaxf(CT_MIN_MG, CT_K * c.ctRest), "ctOn == max(%.1f, %.1f×%.2f) = %.2f",
          CT_MIN_MG, CT_K, c.ctRest, t.ctOn);
    CHECK(stateResult().calOk, "Result.calOk == true");
#ifdef CAL_TEST_HAS_REPORT
    CHECK(dspCalLastResult() == CALRES_OK, "report.result == CALRES_OK（实际 %s）",
          resName(dspCalLastResult()));
#endif
  }
  printf("\n");

  /* 以下用例都以 T04 的成功标定为"上次有效配置" */
  Snap base;
  snap(&base);
  printf("基准标定：bias=%.2f/%.2f/%.2f  |g|=%.1f  env=%.2f  ct=%.2f  zcl=%.2f\n",
         base.cal.bias[0], base.cal.bias[1], base.cal.bias[2], base.cal.biasNormMg,
         base.cal.envRest, base.cal.ctRest, base.cal.zcl);
  printf("基准阈值：hi=%.2f lo=%.2f ctOn=%.2f ctOff=%.2f\n\n",
         base.th.hi, base.th.lo, base.th.ctOn, base.th.ctOff);

  /* ---------- T01：阶段A 失败，完整保留 ---------- */
  printf("--- T01 阶段A 失败（读不到样本）---\n");
  resetScenario();
  g_mpuFailA = true;
  {
    bool ok = runCal();
    const DspCal& c = dspCal();
    const Thresholds& t = stateThresholds();
    CHECK(!ok, "标定返回 false");
    CHECK(calEqual(c, base.cal), "s_cal 逐字段完全不变（含 nBias）");
    if (!calEqual(c, base.cal)) diffCal(base.cal, c);
    CHECK(thEqual(t, base.th), "四个阈值完全不变");
    if (!thEqual(t, base.th)) diffTh(base.th, t);
    CHECK(c.ok, "cal.ok 仍为 true（上次有效标定仍可信）");
    CHECK(g_stageBRuns == 0, "阶段B 未被启动（提前拒绝，省 1.2s）");
#ifdef CAL_TEST_HAS_REPORT
    CHECK(dspCalLastResult() == CALRES_ERR_SAMPLES, "report.result == CALRES_ERR_SAMPLES（实际 %s）",
          resName(dspCalLastResult()));
#endif
  }
  printf("\n");

  /* ---------- T02：阶段A 成功但不同，阶段B 断流 ---------- */
  printf("--- T02 阶段A 合格（bias={0,30,999.5}，|g|≈1000）但阶段B 断流 ---\n");
  NOTE("模长必须仍 ≈1000mg，否则会被阶段A 的模长检查提前拒绝，走不到阶段B");
  resetScenario();
  g_gravA[1] = 30.0f; g_gravA[2] = 999.5f;
  g_feedEnable = false;
  {
    bool ok = runCal();
    const DspCal& c = dspCal();
    const Thresholds& t = stateThresholds();
    CHECK(!ok, "标定返回 false");
    CHECK(c.bias[1] == base.cal.bias[1] && c.bias[2] == base.cal.bias[2],
          "偏置未被替换（bias=%.2f/%.2f/%.2f，应保持 %.2f/%.2f/%.2f）",
          c.bias[0], c.bias[1], c.bias[2],
          base.cal.bias[0], base.cal.bias[1], base.cal.bias[2]);
    CHECK(c.biasNormMg == base.cal.biasNormMg, "|g| 未被替换（%.1f）", c.biasNormMg);
    CHECK(calEqual(c, base.cal), "s_cal 逐字段完全不变");
    if (!calEqual(c, base.cal)) diffCal(base.cal, c);
    CHECK(thEqual(t, base.th), "四个阈值完全不变（不得出现新偏置配旧阈值）");
    if (!thEqual(t, base.th)) diffTh(base.th, t);
    CHECK(c.ok, "cal.ok 仍为 true");
    CHECK(g_stageBRuns == 1, "阶段B 确实跑过（%d 次）", g_stageBRuns);
#ifdef CAL_TEST_HAS_REPORT
    CHECK(dspCalLastResult() == CALRES_ERR_STAGE_B, "report.result == CALRES_ERR_STAGE_B（实际 %s）",
          resName(dspCalLastResult()));
#endif
  }
  printf("\n");

  /* ---------- T03a：阶段A 被碰（std 超限） ---------- */
  printf("--- T03a 阶段A 抖动（noise=60 → std≈35mg > %.0f）---\n", CAL_MAX_STD_MG);
  resetScenario();
  g_noiseA = 60.0f;
  {
    bool ok = runCal();
    const DspCal& c = dspCal();
    /* 旧代码：quality=false 但仍 ok=true 并提交 → 这条是红的，正是子问题② */
    CHECK(!ok, "标定返回 false（质量不合格必须拒绝提交）");
    CHECK(calEqual(c, base.cal), "s_cal 逐字段完全不变");
    if (!calEqual(c, base.cal)) diffCal(base.cal, c);
    CHECK(thEqual(stateThresholds(), base.th), "四个阈值完全不变");
    CHECK(c.ok, "cal.ok 仍为 true");
    CHECK(g_stageBRuns == 0, "阶段B 未被启动");
#ifdef CAL_TEST_HAS_REPORT
    CHECK(dspCalLastResult() == CALRES_ERR_STAGE_A, "report.result == CALRES_ERR_STAGE_A（实际 %s）",
          resName(dspCalLastResult()));
#endif
  }
  printf("\n");

  /* ---------- T03b：阶段B 削顶 ---------- */
  printf("--- T03b 阶段B 削顶 ---\n");
  resetScenario();
  g_clipB = true;
  {
    bool ok = runCal();
    const DspCal& c = dspCal();
    CHECK(!ok, "标定返回 false");
    CHECK(calEqual(c, base.cal), "s_cal 逐字段完全不变");
    if (!calEqual(c, base.cal)) diffCal(base.cal, c);
    CHECK(thEqual(stateThresholds(), base.th), "四个阈值完全不变");
#ifdef CAL_TEST_HAS_REPORT
    CHECK(dspCalLastResult() == CALRES_ERR_STAGE_B_QUALITY,
          "report.result == CALRES_ERR_STAGE_B_QUALITY（实际 %s）", resName(dspCalLastResult()));
    CHECK(dspCalReport().stageBClip > 0, "report.stageBClip = %u > 0",
          (unsigned)dspCalReport().stageBClip);
#endif
  }
  printf("\n");

  /* ---------- T03d：阶段A 后模块被转动 → 拒绝 ---------- */
  printf("--- T03d 标定中转动（A 重力 {0,0,1000}，B 重力 {500,866,0}）---\n");
  NOTE("候选偏置去不掉新姿态 → 残留约 1409mg → 姿态支路 ctRest 被抬高，必须拒绝");
  resetScenario();
  g_gravB[0] = 500.0f; g_gravB[1] = 866.0f; g_gravB[2] = 0.0f;
  {
    bool ok = runCal();
    const DspCal& c = dspCal();
    CHECK(!ok, "标定返回 false（拒绝候选）");
    CHECK(calEqual(c, base.cal), "s_cal 逐字段完全不变");
    if (!calEqual(c, base.cal)) diffCal(base.cal, c);
    CHECK(thEqual(stateThresholds(), base.th), "四个阈值完全不变");
    CHECK(c.ok, "cal.ok 仍为 true");
#ifdef CAL_TEST_HAS_REPORT
    CHECK(dspCalLastResult() == CALRES_ERR_STAGE_B_QUALITY,
          "report.result == CALRES_ERR_STAGE_B_QUALITY（实际 %s）", resName(dspCalLastResult()));
    CHECK(dspCalReport().ctRest > CAL_MAX_CT_MG,
          "report.ctRest = %.1f > %.1f", dspCalReport().ctRest, CAL_MAX_CT_MG);
#endif
  }
  printf("\n");

  /* ---------- 收尾：所有失败都没污染基准 ---------- */
  printf("--- 收尾：连续 5 次失败后基准仍未变 ---\n");
  {
    const DspCal& c = dspCal();
    CHECK(calEqual(c, base.cal), "s_cal 仍等于 T04 的结果");
    if (!calEqual(c, base.cal)) diffCal(base.cal, c);
    CHECK(thEqual(stateThresholds(), base.th), "阈值仍等于 T04 的结果");
    CHECK(c.ok, "cal.ok 仍为 true");
  }
  printf("\n");

  /* ---------- T03e：阶段B 用的是候选偏置，不是旧偏置 ---------- */
  printf("--- T03e 阶段B 用候选偏置（A/B 同为新姿态 {0,500,866}）---\n");
  NOTE("旧偏置 {0,0,1000} 若仍生效，残留约 1000mg，ctRest 会被抬高而拒绝");
  NOTE("正确用候选偏置时残留≈0，应当接受并提交新配置");
  resetScenario();
  g_gravA[0] = 0.0f; g_gravA[1] = 500.0f; g_gravA[2] = 866.0f;
  g_gravB[0] = 0.0f; g_gravB[1] = 500.0f; g_gravB[2] = 866.0f;
  {
    bool ok = runCal();
    const DspCal& c = dspCal();
    CHECK(ok, "标定返回 true（姿态一致，无残留）");
    CHECK(c.ok, "cal.ok == true");
    CHECK(fabsf(c.bias[1] - 500.0f) < 30.0f && fabsf(c.bias[2] - 866.0f) < 30.0f,
          "新偏置已提交：bias=%.1f/%.1f/%.1f（应≈0/500/866）",
          c.bias[0], c.bias[1], c.bias[2]);
    CHECK(c.ctRest < CAL_MAX_CT_MG,
          "ctRest = %.2f < %.1f（证明阶段B 用的是候选偏置，残留≈0）",
          c.ctRest, CAL_MAX_CT_MG);
#ifdef CAL_TEST_HAS_REPORT
    CHECK(dspCalLastResult() == CALRES_OK, "report.result == CALRES_OK（实际 %s）",
          resName(dspCalLastResult()));
#endif
  }

  printf("\n%s（%d/%d 项通过，%d 项失败）\n",
         g_fail ? "存在失败" : "全部通过", g_total - g_fail, g_total, g_fail);
  return g_fail ? 1 : 0;
}
