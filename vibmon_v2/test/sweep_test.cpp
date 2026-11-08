// 主频与启停验证（T07、T08）：链接真实 dsp.cpp + state.cpp。
//
// 编译：
//   g++ -std=c++17 -Wall -DSTATE_TEST_HOOKS -I test -I . test/sweep_test.cpp dsp.cpp state.cpp
//       -o test/sweep_test.exe
//
// 设计要点：
//   * 量程从 mpuLsbPerG() 推导，不写死
//   * T07j 用 stateTestInjectEnv 直接注入包络，隔离"边沿插值"的效果
//   * 边界测试用 61/219 与 59/221，不用精确 60/220（浮点边界不稳）
//   * 谐波跑 5 个相位，统计正确率而不是单次通过
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Arduino.h"
#include "dsp.h"
#include "state.h"
#include "config.h"

/* ================= 虚拟时间 ================= */
static uint32_t g_us = 0;
uint32_t millis(void) { return g_us / 1000u; }
uint32_t micros(void) { return g_us; }
void delay(uint32_t ms) { g_us += ms * 1000u; }
void delayMicroseconds(uint32_t us) { g_us += us; }

/* ================= 量程 ================= */
static float g_lsbPerG = 16384.0f;
float mpuLsbPerG(void) { return g_lsbPerG; }
static inline int16_t mg2raw(float mg) { return (int16_t)lrintf(mg * g_lsbPerG / 1000.0f); }

/* ================= MPU / 采样器桩 ================= */
bool mpuReadAccel(MpuRaw*) { return false; }
uint32_t mpuErrCount(void) { return 0; }
void mpuClearErr(void) {}
void samplerAddClip(void) {}
SamplerStartResult samplerStart(void) { return SMP_START_OK; }
SamplerStopResult  samplerStop(void)  { return SMP_STOP_OK; }
bool samplerReset(void) { return true; }
uint32_t samplerInFlight(void) { return 0; }
bool samplerStartIsError(SamplerStartResult r) {
  return r == SMP_START_ERR_CREATE || r == SMP_START_ERR_START;
}
bool samplerStopIsError(SamplerStopResult r) { return r == SMP_STOP_TIMEOUT; }
const char* samplerStartStr(SamplerStartResult) { return ""; }
const char* samplerStopStr(SamplerStopResult)   { return ""; }
static SamplerStats g_stats = {};
const SamplerStats& samplerStats(void) { return g_stats; }
void samplerClearStats(void) { g_stats = SamplerStats(); }
bool samplerSensorLost(void) { return false; }
bool samplerPop(Sample*, SamplerGap*) { return false; }
static TimingStats g_timing = {};
const TimingStats& samplerTiming(void) { return g_timing; }
void samplerTimingReset(void) { g_timing = TimingStats(); }
void samplerNoteLoopUs(uint32_t) {}
void samplerNoteOutUs(uint32_t) {}

/* ================= 断言 ================= */
static int g_fail = 0, g_total = 0;
#define CHECK(cond, ...) do {                                   \
  g_total++;                                                    \
  if (!(cond)) { g_fail++; printf("[FAIL] "); }                 \
  else          { printf("[PASS] "); }                          \
  printf(__VA_ARGS__); printf("\n");                            \
} while (0)
#define NOTE(...) do { printf("       "); printf(__VA_ARGS__); printf("\n"); } while (0)

/* ================= 信号源 ================= */
static uint32_t g_seq = 0;

static void initCal(void) {
  DspCal& cal = const_cast<DspCal&>(dspCal());
  cal.bias[2] = 1000.0f; cal.envRest = 2.0f; cal.ctRest = 1.0f; cal.zcl = 8.0f;
  cal.ok = cal.quality = true;
  stateApplyCal(cal);
}

static void resetAll(void) {
  dspInit();
  stateReset();
  stateSetSensorOk(true);
  g_us = 0; g_seq = 0;
  samplerClearStats();
  initCal();
  stateReset();
  stateSetSensorOk(true);
}

/*
 * 喂样本。
 *   freqHz  振动频率
 *   ampMg   振幅（落在 axis 轴）
 *   ssHz    启停频率（0 = 持续振动）
 *   duty    占空比
 *   noise   噪声 mg
 *   harm2/3 二/三次谐波比例
 *   phase   谐波相位（弧度）
 */
static void feed(uint32_t ms, float freqHz, float ampMg, int axis,
                 float ssHz = 0.0f, float duty = 0.5f,
                 float noise = 1.0f, float harm2 = 0.0f, float harm3 = 0.0f,
                 float phase = 0.0f) {
  const uint32_t n = ms * FS_HZ / 1000;
  for (uint32_t i = 0; i < n; i++) {
    g_us += 1000000u / FS_HZ;
    const float t = g_us / 1e6f;

    float sig = 0.0f;
    if (ampMg > 0.0f) {
      bool on = (ssHz <= 0.0f) || (fmodf(t * ssHz, 1.0f) < duty);
      if (on) {
        sig = ampMg * sinf(2 * PI * freqHz * t);
        if (harm2 > 0.0f) sig += harm2 * ampMg * sinf(2 * PI * 2 * freqHz * t + phase);
        if (harm3 > 0.0f) sig += harm3 * ampMg * sinf(2 * PI * 3 * freqHz * t + phase);
      }
    }
    const float nz = ((rand() % 2001) - 1000) / 1000.0f * noise;

    Sample s;
    s.seq = g_seq++;
    s.tUs = g_us;
    s.raw.x = (axis == 0) ? mg2raw(sig + nz) : mg2raw(nz);
    s.raw.y = (axis == 1) ? mg2raw(sig + nz) : mg2raw(nz);
    s.raw.z = (axis == 2) ? mg2raw(1000.0f + sig + nz) : mg2raw(1000.0f + nz);

    DspFrame f;
    dspProcess(s, &f);
    stateOnFrame(f, s.tUs);
    stateTick(g_us);
  }
}

/* 两轴同幅（T08k） */
static void feed2Axis(uint32_t ms, float freqHz, float ampMg) {
  const uint32_t n = ms * FS_HZ / 1000;
  for (uint32_t i = 0; i < n; i++) {
    g_us += 1000000u / FS_HZ;
    const float t = g_us / 1e6f;
    const float sig = ampMg * sinf(2 * PI * freqHz * t);
    const float nz = ((rand() % 2001) - 1000) / 1000.0f * 1.0f;

    Sample s;
    s.seq = g_seq++;
    s.tUs = g_us;
    s.raw.x = mg2raw(sig + nz);
    s.raw.y = mg2raw(sig + nz);
    s.raw.z = mg2raw(1000.0f + nz);

    DspFrame f;
    dspProcess(s, &f);
    stateOnFrame(f, s.tUs);
    stateTick(g_us);
  }
}

int main(void) {
  g_lsbPerG = 16384.0f / (float)(1 << (ACCEL_FS_SEL & 0x03));
  srand(20261018);

  printf("=== sweep_test：T07 启停 / T08 主频 ===\n");
  printf("FS_HZ=%d  BLOCK_N=%d  ENV_HOP=%d  ENV_WIN=%d\n", FS_HZ, BLOCK_N, ENV_HOP, ENV_WIN);
  printf("频段 %0.0f~%0.0f Hz  SS_T_MAX_MS=%d  SS_USE_PERIODS=%d  SS_CONSIST_RATIO=%.1f\n\n",
         FREQ_LO_HZ, FREQ_HI_HZ, SS_T_MAX_MS, SS_USE_PERIODS, SS_CONSIST_RATIO);

  /* ================= T07 启停频率 ================= */
  printf("--- T07a~f 启停频率与占空比 ---\n");
  {
    struct { float ss, duty; const char* tag; } cases[] = {
      {0.5f, 0.5f, "T07a 0.5Hz/50%"}, {1.0f, 0.5f, "T07b 1.0Hz/50%"},
      {2.0f, 0.5f, "T07c 2.0Hz/50%"}, {3.0f, 0.5f, "T07d 3.0Hz/50%"},
      {1.0f, 0.2f, "T07e 1.0Hz/20%"}, {1.0f, 0.8f, "T07f 1.0Hz/80%"},
    };
    for (unsigned k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
      resetAll();
      /* 跑足够长：至少 6 个启停周期 */
      const uint32_t ms = (uint32_t)(6.0f / cases[k].ss * 1000.0f) + 600;
      feed(ms, 150.0f, 300.0f, 2, cases[k].ss, cases[k].duty);

      const Result& r = stateResult();
      const float err = fabsf(r.ssHz - cases[k].ss) / cases[k].ss;
      float per[SS_EDGE_MAX] = {0};
      const uint8_t np = stateTestPeriods(per, SS_EDGE_MAX);

      NOTE("%-16s → %.3f Hz (%u 周期, 误差 %.2f%%)",
           cases[k].tag, r.ssHz, (unsigned)r.ssPeriods, err * 100.0f);
      if (np >= 2) {
        printf("       ");
        for (uint8_t i = 0; i < np; i++) printf("%.1f ", per[i]);
        printf("ms\n");
      }
      CHECK(r.ssValid && err < 0.02f, "%s 误差 < 2%%", cases[k].tag);
    }
  }
  printf("\n");

  printf("--- T07g/h 低频边界 ---\n");
  {
    struct { float ss; float tol; const char* tag; } cases[] = {
      {0.25f, 0.02f, "T07g 0.25Hz"}, {0.1f, 0.05f, "T07h 0.1Hz"},
    };
    for (unsigned k = 0; k < 2; k++) {
      resetAll();
      const uint32_t ms = (uint32_t)(5.0f / cases[k].ss * 1000.0f) + 800;
      feed(ms, 150.0f, 300.0f, 2, cases[k].ss, 0.5f);
      const Result& r = stateResult();
      const float err = fabsf(r.ssHz - cases[k].ss) / cases[k].ss;
      NOTE("%-14s → %.4f Hz (%u 周期, 误差 %.2f%%)",
           cases[k].tag, r.ssHz, (unsigned)r.ssPeriods, err * 100.0f);
      CHECK(r.ssValid && err < cases[k].tol, "%s 误差 < %.0f%%", cases[k].tag, cases[k].tol * 100.0f);
    }
  }
  printf("\n");

  printf("--- T07i 频率切换收敛（%d 个新周期内）---\n", SS_USE_PERIODS);
  {
    resetAll();
    feed(6000, 150.0f, 300.0f, 2, 0.5f, 0.5f);
    CHECK(stateResult().ssValid, "切换前 0.5Hz 有效（%.3f Hz）", stateResult().ssHz);

    /* 切到 1Hz，跑 SS_USE_PERIODS 个新周期 */
    const uint32_t msAfter = (uint32_t)(SS_USE_PERIODS / 1.0f * 1000.0f) + 600;
    feed(msAfter, 150.0f, 300.0f, 2, 1.0f, 0.5f);
    const Result& r = stateResult();
    NOTE("切换后 %u 个周期：%.3f Hz（期望 1.000）", (unsigned)r.ssPeriods, r.ssHz);
    CHECK(r.ssValid && fabsf(r.ssHz - 1.0f) < 0.05f,
          "切换后 %d 个新周期内收敛到 1Hz", SS_USE_PERIODS);
  }
  printf("\n");

  printf("--- T07j 边沿插值：快/慢上升沿应给出相近周期 ---\n");
  NOTE("用 stateTestInjectEnv 直接注入包络，隔离插值效果（不走 DSP 链路）");
  {

    /* 场景 A：快上升沿 —— 1 个包络点内从 0 跳到 400 */
    resetAll(); initCal(); stateReset(); stateSetSensorOk(true);
    float pFast[SS_EDGE_MAX] = {0}; uint8_t nFast = 0;
    {
      const uint32_t step = ENV_HOP * (1000000u / FS_HZ);    // 10ms
      uint32_t t = 0;
      const uint32_t periodUs = 1000000u;                   // 1Hz → 1000ms
      for (int cyc = 0; cyc < 5; cyc++) {
        /* 低电平段 500ms */
        for (int i = 0; i < 50; i++) { stateTestInjectEnv(0.5f, t); t += step; }
        /* 起振：一个点直接跳到 400 */
        for (int i = 0; i < 50; i++) { stateTestInjectEnv(400.0f, t); t += step; }
      }
      (void)periodUs;
      nFast = stateTestPeriods(pFast, SS_EDGE_MAX);
    }

    /* 场景 B：慢上升沿 —— 4 个包络点线性从 0 爬到 400 */
    resetAll(); initCal(); stateReset(); stateSetSensorOk(true);
    float pSlow[SS_EDGE_MAX] = {0}; uint8_t nSlow = 0;
    {
      const uint32_t step = ENV_HOP * (1000000u / FS_HZ);
      uint32_t t = 0;
      for (int cyc = 0; cyc < 5; cyc++) {
        for (int i = 0; i < 50; i++) { stateTestInjectEnv(0.5f, t); t += step; }
        /* 4 个点爬到 400（每点 100） */
        stateTestInjectEnv(100.0f, t); t += step;
        stateTestInjectEnv(200.0f, t); t += step;
        stateTestInjectEnv(300.0f, t); t += step;
        stateTestInjectEnv(400.0f, t); t += step;
        for (int i = 0; i < 46; i++) { stateTestInjectEnv(400.0f, t); t += step; }
      }
      nSlow = stateTestPeriods(pSlow, SS_EDGE_MAX);
    }

    if (nFast >= 1 && nSlow >= 1) {
      const float fFast = pFast[nFast - 1];
      const float fSlow = pSlow[nSlow - 1];
      NOTE("快沿周期 %.1f ms（%u 个）  慢沿周期 %.1f ms（%u 个）",
           fFast, (unsigned)nFast, fSlow, (unsigned)nSlow);
      CHECK(fabsf(fFast - fSlow) < 20.0f,
            "快/慢沿周期差 %.1f ms < 20ms（插值把形状差异基本消除）",
            fabsf(fFast - fSlow));
    } else {
      CHECK(false, "两个场景都应产出至少 1 个周期（fast=%u slow=%u）",
            (unsigned)nFast, (unsigned)nSlow);
    }
  }
  printf("\n");

  printf("--- T07k/l 无周期时不得报有效 0Hz ---\n");
  {
    resetAll();
    feed(3000, 150.0f, 300.0f, 2, 0.0f);       // 持续振动，不启停
    CHECK(!stateResult().ssValid && stateResult().ssHz == 0.0f,
          "T07k 持续振动：ssValid=false（%.3f Hz）", stateResult().ssHz);

    resetAll();
    feed(3000, 0.0f, 0.0f, 2);                  // 完全静止
    CHECK(!stateResult().ssValid && stateResult().ssHz == 0.0f,
          "T07l 完全静止：ssValid=false（%.3f Hz）", stateResult().ssHz);
  }
  printf("\n");

  /* ================= T08 主频 ================= */
  printf("--- T08a~e 标称频点 ---\n");
  {
    const float fList[] = {80.0f, 100.0f, 130.0f, 160.0f, 180.0f};
    const char* tags[] = {"T08a", "T08b", "T08c", "T08d", "T08e"};
    for (int k = 0; k < 5; k++) {
      resetAll();
      feed(800, fList[k], 300.0f, 2);
      const Result& r = stateResult();
      NOTE("%s %5.0f Hz → %.2f Hz（误差 %+.2f Hz，轴 %c）",
           tags[k], fList[k], r.freqHz, r.freqHz - fList[k], 'X' + r.axis);
      CHECK(r.freqValid && fabsf(r.freqHz - fList[k]) < 1.0f, "%s 误差 < 1Hz", tags[k]);
    }
  }
  printf("\n");

  printf("--- T08f 噪声 ---\n");
  {
    resetAll();
    feed(800, 130.0f, 300.0f, 2, 0.0f, 0.5f, 5.0f);
    const Result& r = stateResult();
    NOTE("130Hz + 5mg 噪声 → %.2f Hz", r.freqHz);
    CHECK(r.freqValid && fabsf(r.freqHz - 130.0f) < 2.0f, "T08f 误差 < 2Hz");
  }
  printf("\n");

  printf("--- T08g/h 谐波（5 个相位，统计正确率）---\n");
  {
    const float phases[] = {0.0f, (float)PI / 4, (float)PI / 2, 3 * (float)PI / 4, (float)PI};
    for (int harm = 2; harm <= 3; harm++) {
      int ok = 0, doubled = 0, invalid = 0;
      for (int p = 0; p < 5; p++) {
        resetAll();
        feed(800, 130.0f, 300.0f, 2, 0.0f, 0.5f, 1.0f,
             harm == 2 ? 0.20f : 0.0f, harm == 3 ? 0.20f : 0.0f, phases[p]);
        const Result& r = stateResult();
        if (!r.freqValid) { invalid++; continue; }
        if (fabsf(r.freqHz - 130.0f) < 3.0f) ok++;
        else if (fabsf(r.freqHz - 260.0f) < 6.0f || fabsf(r.freqHz - 390.0f) < 9.0f) doubled++;
      }
      NOTE("%d 次谐波 20%%：正确 %d/5，倍频误判 %d/5，无有效值 %d/5",
           harm, ok, doubled, invalid);
      CHECK(ok >= 3, "T08%s %d 次谐波：至少 3/5 相位算对基频", harm == 2 ? "g" : "h", harm);
      CHECK(doubled == 0, "T08%s %d 次谐波：无倍频误判", harm == 2 ? "g" : "h", harm);
    }
  }
  printf("\n");

  printf("--- T08i~k 多轴 ---\n");
  {
    for (int ax = 0; ax < 2; ax++) {
      resetAll();
      feed(800, 100.0f, 300.0f, ax);
      const Result& r = stateResult();
      NOTE("主轴 %c → 选用 %c，频率 %.2f Hz", 'X' + ax, 'X' + r.axis, r.freqHz);
      CHECK(r.freqValid && r.axis == (uint8_t)ax, "T08%s 轴选择正确",
            ax == 0 ? "i" : "j");
    }
    resetAll();
    feed2Axis(800, 100.0f, 300.0f);
    const Result& r = stateResult();
    NOTE("两轴等幅 → 轴 %c，频率 %.2f Hz", 'X' + r.axis, r.freqHz);
    CHECK(r.freqValid, "T08k 两轴等幅不崩溃且能出值");
  }
  printf("\n");

  printf("--- T08l/m 频段边界（61/219 内，59/221 外）---\n");
  NOTE("已知边界特性：tMin/tMax 是硬截断，界外的真实周期会被滤掉一部分，");
  NOTE("剩余周期若落回界内仍可能给出读数。所以 221Hz 的期望是「明显偏离」而不是「必须无效」。");
  {
    const float inside[] = {61.0f, 219.0f};
    for (int k = 0; k < 2; k++) {
      resetAll();
      feed(900, inside[k], 300.0f, 2);
      const Result& r = stateResult();
      NOTE("边界内 %.0f Hz → %.2f Hz（valid=%d）", inside[k], r.freqHz, r.freqValid);
      CHECK(r.freqValid && fabsf(r.freqHz - inside[k]) < 2.0f,
            "T08l %.0f Hz 应能出值", inside[k]);
    }

    /* 59Hz：周期 16.95ms > tMax(16.67ms)，全部被滤掉 → 必无效 */
    {
      resetAll();
      feed(900, 59.0f, 300.0f, 2);
      const Result& r = stateResult();
      NOTE("边界外 59 Hz → freqValid=%d freqHz=%.2f", r.freqValid, r.freqHz);
      CHECK(!r.freqValid, "T08m 59 Hz 应判无效（周期全部超 tMax）");
    }

    /* 221Hz：周期 4.52ms < tMin(4.545ms)，但插值抖动会让部分周期落回界内，
       给出一个被截断到 ~218 的读数。这是已知的边界泄漏：
       对外承诺 80~180Hz，220 以上的行为不作保证，但**不能伪装成准确值**。 */
    {
      resetAll();
      feed(900, 221.0f, 300.0f, 2);
      const Result& r = stateResult();
      NOTE("边界外 221 Hz → freqValid=%d freqHz=%.2f（与 221 偏差 %.1f Hz）",
           r.freqValid, r.freqHz, fabsf(r.freqHz - 221.0f));
      CHECK(!r.freqValid || fabsf(r.freqHz - 221.0f) > 1.5f,
            "T08m 221 Hz 要么无效，要么读数明显偏离（不得伪装成准确值）");
    }
  }
  printf("\n");

  printf("--- T08n/o 弱信号幅值扫描 ---\n");
  {
    const float fList[] = {80.0f, 130.0f, 180.0f};
    const float aList[] = {20.0f, 50.0f};
    for (int fi = 0; fi < 3; fi++) {
      for (int ai = 0; ai < 2; ai++) {
        resetAll();
        feed(900, fList[fi], aList[ai], 2);
        const Result& r = stateResult();
        NOTE("%3.0f Hz / %2.0f mg → valid=%d  %.2f Hz",
             fList[fi], aList[ai], r.freqValid, r.freqHz);
        CHECK(r.freqValid && fabsf(r.freqHz - fList[fi]) < 3.0f,
              "T08n %.0fHz/%.0fmg 能出值", fList[fi], aList[ai]);
      }
    }
  }
  printf("\n");

  printf("\n%s（%d/%d 项通过，%d 项失败）\n",
         g_fail ? "存在失败" : "全部通过", g_total - g_fail, g_total, g_fail);
  return g_fail ? 1 : 0;
}
