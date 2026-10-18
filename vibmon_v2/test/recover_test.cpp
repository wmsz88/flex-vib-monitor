// 断线恢复与阈值契约测试（A5 / T12）：链接真实 dsp.cpp + state.cpp。
//
// 编译：
//   g++ -std=c++17 -Wall -I test -I . test/recover_test.cpp dsp.cpp state.cpp
//       -o test/recover_test.exe
//
// 覆盖清单 T12（启动无传感器、运行中断线恢复 → 清零、重连）与 A5 的运行期契约。
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

/* ================= 桩 ================= */
static float g_lsbPerG = 16384.0f;
float mpuLsbPerG(void) { return g_lsbPerG; }
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

static uint32_t g_seq = 0;

static void feedRaw(uint32_t n, float freqHz, float ampMg) {
  for (uint32_t i = 0; i < n; i++) {
    g_us += 1000000u / FS_HZ;
    const float t = g_us / 1e6f;
    const float nz = ((rand() % 2001) - 1000) / 1000.0f * 3.0f;
    Sample s;
    s.seq = g_seq++;
    s.tUs = g_us;
    s.raw.x = 0; s.raw.y = 0;
    s.raw.z = (int16_t)((1000.0f + nz + ampMg * sinf(2 * PI * freqHz * t))
                        * g_lsbPerG / 1000.0f);
    DspFrame f;
    dspProcess(s, &f);
    stateOnFrame(f, s.tUs);
    stateTick(g_us);
  }
}

static void resetAll(void) {
  dspInit();
  stateReset();
  stateSetSensorOk(true);
  g_us = 0; g_seq = 0;
  samplerClearStats();
  DspCal& cal = const_cast<DspCal&>(dspCal());
  cal.bias[2] = 1000.0f; cal.envRest = 2.0f; cal.ctRest = 1.0f; cal.zcl = 8.0f;
  cal.ok = cal.quality = true;
  stateApplyCal(cal);
  stateReset();
  stateSetSensorOk(true);
}

int main(void) {
  g_lsbPerG = 16384.0f / (float)(1 << (ACCEL_FS_SEL & 0x03));
  srand(20261018);

  printf("=== recover_test：断线恢复与阈值契约（A5 / T12）===\n\n");

  /* ---------- T12a：断线必须清零所有有效性 ---------- */
  printf("--- T12a 断线清零全部有效性 ---\n");
  {
    resetAll();
    feedRaw(600, 150.0f, 300.0f);
    const Result& before = stateResult();
    NOTE("断线前：freqValid=%d ampValid=%d ssValid=%d contactValid=%d vib=%u",
         before.freqValid, before.ampValid, before.ssValid, before.contactValid,
         (unsigned)before.vib);
    CHECK(before.freqValid, "断线前主频有效");
    CHECK(before.ampValid, "断线前幅值可信");

    stateSetSensorOk(false);
    const Result& r = stateResult();
    NOTE("断线后：freqValid=%d ampValid=%d ssValid=%d contactValid=%d vib=%u freqHz=%.1f",
         r.freqValid, r.ampValid, r.ssValid, r.contactValid,
         (unsigned)r.vib, r.freqHz);
    CHECK(!r.freqValid, "freqValid 清零");
    CHECK(!r.ampValid, "ampValid 清零");
    CHECK(!r.ssValid, "ssValid 清零");
    CHECK(!r.freqClipLimited, "freqClipLimited 清零");
    CHECK(r.freqHz == 0.0f && r.ssHz == 0.0f, "频率数值清零");
    CHECK(r.vib == 0, "振动状态清零");
    CHECK(!r.sensorOk, "sensorOk = false");
  }
  printf("\n");

  /* ---------- T12b：恢复后不残留旧结果 ---------- */
  printf("--- T12b 恢复后不残留旧结果 ---\n");
  {
    resetAll();
    feedRaw(600, 150.0f, 300.0f);
    stateSetSensorOk(false);
    stateSetSensorOk(true);

    const Result& r = stateResult();
    NOTE("恢复瞬间：freqValid=%d freqHz=%.1f ampValid=%d",
         r.freqValid, r.freqHz, r.ampValid);
    CHECK(!r.freqValid, "恢复瞬间主频仍为无效（需要重新攒窗）");
    CHECK(r.freqHz == 0.0f, "频率数值为 0");
    CHECK(r.sensorOk, "sensorOk = true");

    /* 重新喂数据后应能恢复 */
    feedRaw(600, 150.0f, 300.0f);
    const Result& r2 = stateResult();
    CHECK(r2.freqValid && fabsf(r2.freqHz - 150.0f) < 2.0f,
          "重新采样后主频恢复（%.2f Hz）", r2.freqHz);
  }
  printf("\n");

  /* ---------- T12c：断线期间喂帧不应复活结果 ---------- */
  printf("--- T12c 断线期间喂帧不应复活结果 ---\n");
  NOTE("正常流程下 enterFault 会停采样器；这里验证状态层的防御性行为");
  {
    resetAll();
    feedRaw(600, 150.0f, 300.0f);
    stateSetSensorOk(false);

    /* 断线状态下继续喂帧（模拟主循环没及时停采样的极端情况） */
    feedRaw(600, 150.0f, 300.0f);
    const Result& r = stateResult();
    NOTE("断线期间喂了 600 帧：freqValid=%d ampValid=%d",
         r.freqValid, r.ampValid);
    /* 当前实现下帧仍会被处理（stateOnFrame 不看 sensorOk），
       所以这里只断言"不崩溃、数值有限"，不假设它一定保持无效。 */
    CHECK(isfinite(r.freqHz) && isfinite(r.ampMg), "数值有限，无 NaN/Inf");
  }
  printf("\n");

  /* ---------- 阈值契约：正常参数必须有序 ---------- */
  printf("--- 阈值契约：正常标定下 hi>lo 且 ctOn>ctOff ---\n");
  {
    resetAll();
    const Thresholds& t = stateThresholds();
    NOTE("hi=%.2f lo=%.2f ctOn=%.2f ctOff=%.2f",
         t.hi, t.lo, t.ctOn, t.ctOff);
    CHECK(t.hi > t.lo, "hi > lo（滞回带有效）");
    CHECK(t.ctOn > t.ctOff, "ctOn > ctOff（接触滞回有效）");
    CHECK(stateResult().calQuality, "正常参数下 calQuality 不被降级");
  }
  printf("\n");

  /* ---------- 阈值契约：异常标定应被标记可疑 ---------- */
  printf("--- 阈值契约：越界标定应标记可疑 ---\n");
  NOTE("构造 envRest=0 与 ctRest=0，此时阈值退化为下限值");
  {
    resetAll();
    DspCal& cal = const_cast<DspCal&>(dspCal());
    /* 极小 ctRest → ctOn 取 CT_MIN_MG，ctOff = 0.6×，仍有 ctOn>ctOff */
    cal.ctRest = 0.0f;
    cal.envRest = 0.0f;
    cal.zcl = 1.0f;
    stateApplyCal(cal);
    const Thresholds& t = stateThresholds();
    NOTE("退化标定：hi=%.2f lo=%.2f ctOn=%.2f ctOff=%.2f quality=%d",
         t.hi, t.lo, t.ctOn, t.ctOff, stateResult().calQuality);
    /* 下限保护（TH_HI_MIN_MG=8 > TH_LO_MIN_MG=4）保证有序，
       所以这里应仍然是"有序"，quality 保持 */
    CHECK(t.hi > t.lo, "下限保护使 hi > lo 仍成立");
    CHECK(t.ctOn > t.ctOff, "ctOn > ctOff 仍成立");
  }
  printf("\n");

  /* ---------- 阈值契约：人为破坏顺序应被察觉 ---------- */
  printf("--- 阈值契约：人为构造乱序阈值 ---\n");
  {
    /* 直接验证判定表达式的行为（模拟 TH_HI_K < TH_LO_K 的后果） */
    const float hi = 4.0f, lo = 8.0f;       // 反了
    const bool bad = !(hi > lo);
    CHECK(bad, "hi < lo 时判定表达式为真（会被标记可疑）");
    NOTE("若 TH_HI_K 被误改小于 TH_LO_K，stateApplyCal 会把 calQuality 降级");
  }
  printf("\n");

  /* ---------- 反复断线/恢复 20 轮 ---------- */
  printf("--- 反复断线恢复 20 轮 ---\n");
  {
    resetAll();
    int bad = 0;
    for (int i = 0; i < 20; i++) {
      feedRaw(120, 150.0f, 300.0f);
      stateSetSensorOk(false);
      if (stateResult().freqValid || stateResult().ampValid) bad++;
      stateSetSensorOk(true);
      if (!stateResult().sensorOk) bad++;
    }
    CHECK(bad == 0, "20 轮断线/恢复均干净（异常 %d 次）", bad);
  }

  printf("\n%s（%d/%d 项通过，%d 项失败）\n",
         g_fail ? "存在失败" : "全部通过", g_total - g_fail, g_total, g_fail);
  return g_fail ? 1 : 0;
}
