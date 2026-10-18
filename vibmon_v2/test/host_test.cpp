// 主机测试：用合成信号驱动 dsp + state（真实源码），验证主频、启停频率、缺样与超时。
// 编译：g++ -std=c++17 -I test -I . test/host_test.cpp dsp.cpp state.cpp -o test/host_test
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <initializer_list>
#include "Arduino.h"
#include "dsp.h"
#include "state.h"
#include "config.h"

static uint32_t g_us = 0;
uint32_t millis(void) { return g_us / 1000; }
uint32_t micros(void) { return g_us; }
void delay(uint32_t) {}
void delayMicroseconds(uint32_t us) { g_us += us; }
float mpuLsbPerG(void) { return 16384.0f; }
bool mpuReadAccel(MpuRaw*) { return false; }
void samplerAddClip(void) {}
// 第 4 步：dsp.cpp 的标定路径改用 samplerStats().clip 差值，这里补桩
static SamplerStats g_stubStats = {};
/* 第 5 步：dsp/state 不直接用，但保持桩完整，避免链接期才发现 */
static TimingStats g_stubTiming = {};
const TimingStats& samplerTiming(void) { return g_stubTiming; }
void samplerTimingReset(void) { g_stubTiming = TimingStats(); }
void samplerNoteLoopUs(uint32_t) {}
void samplerNoteOutUs(uint32_t) {}
const SamplerStats& samplerStats(void) { return g_stubStats; }
void samplerClearStats(void) { g_stubStats = SamplerStats(); }
// 第 2 步：接口从 void 改为枚举/bool，桩跟着改返回值（不动测试逻辑）
SamplerStartResult samplerStart(void) { return SMP_START_OK; }
SamplerStopResult  samplerStop(void)  { return SMP_STOP_OK; }
bool samplerReset(void) { return true; }
bool samplerPop(Sample*, SamplerGap*) { return false; }
uint32_t samplerInFlight(void) { return 0; }
bool samplerStartIsError(SamplerStartResult r) {
  return r == SMP_START_ERR_CREATE || r == SMP_START_ERR_START;
}
bool samplerStopIsError(SamplerStopResult r) { return r == SMP_STOP_TIMEOUT; }

static int g_fail = 0;
#define CHECK(cond, ...) do { printf("%s ", (cond) ? "[PASS]" : "[FAIL]"); printf(__VA_ARGS__); printf("\n"); if (!(cond)) g_fail++; } while (0)

static uint32_t g_seq = 0;
static float noise(void) { return ((rand() % 2001) - 1000) / 1000.0f * 3.0f; }   // ±3mg

// 喂 ms 毫秒信号：fHz 正弦，振幅 ampMg 落在 Z 轴；onoffHz>0 时 50% 占空比启停
static void feed(uint32_t ms, float fHz, float ampMg, float onoffHz, uint32_t dropEvery = 0) {
  for (uint32_t i = 0; i < ms * FS_HZ / 1000; i++) {
    g_us += 1000000 / FS_HZ;
    uint32_t seq = g_seq++;
    if (dropEvery && (seq % dropEvery) == 0) {
      SamplerGap gap;
      gap.kind = SMP_GAP_MISSING;
      gap.missed = 1;
      gap.spanUs = 2u * 1000000u / FS_HZ;
      gap.devUs = 0;
      dspOnGap(gap);
      continue;
    }
    float t = g_us / 1e6f;
    bool on = ampMg > 0 && (onoffHz <= 0 || fmodf(t * onoffHz, 1.0f) < 0.5f);
    float z = 1000.0f + noise() + (on ? ampMg * sinf(2 * PI * fHz * t) : 0);
    Sample s;
    s.seq = seq; s.tUs = g_us;
    s.raw.x = (int16_t)(noise() * 16.384f);
    s.raw.y = (int16_t)(noise() * 16.384f);
    s.raw.z = (int16_t)(z * 16.384f);
    DspFrame f;
    dspProcess(s, &f);
    stateOnFrame(f, s.tUs);
    stateTick(g_us);
  }
}

int main() {
  dspInit();
  // 手工构造标定结果：静置 1g 在 Z 轴
  DspCal& cal = const_cast<DspCal&>(dspCal());
  cal.bias[2] = 1000.0f; cal.envRest = 2.0f; cal.ctRest = 1.0f; cal.zcl = 8.0f; cal.ok = cal.quality = true;
  stateApplyCal(cal);
  stateSetSensorOk(true);
  stateReset();

  feed(1000, 0, 0, 0);
  CHECK(stateResult().vib == 0 && !stateResult().freqValid, "静置：无振动、主频无效");

  for (float fz : {80.0f, 123.4f, 180.0f}) {
    feed(1000, fz, 300, 0);
    const Result& r = stateResult();
    CHECK(r.vib == 1 && r.freqValid && fabsf(r.freqHz - fz) < 1.0f, "持续 %.1fHz → %.2fHz", fz, r.freqHz);
  }
  feed(500, 150, 20, 0);
  CHECK(stateResult().freqValid && fabsf(stateResult().freqHz - 150) < 2.0f,
        "弱信号 150Hz/20mg → %.2fHz", stateResult().freqHz);

  feed(1000, 150, 300, 0, 50);   // 每 50 个样本缺 1 个：窗口永远攒不满 128
  CHECK(!stateResult().freqValid, "频繁缺样：主频判为无效（不按残缺窗口计算）");
  feed(1000, 150, 300, 0);
  CHECK(stateResult().freqValid, "缺样结束后恢复：%.2fHz", stateResult().freqHz);

  feed(500, 0, 0, 0);
  CHECK(stateResult().vib == 0 && stateResult().freqHz == 0, "停止后主频清零");

  for (float ss : {0.5f, 1.2f, 3.0f}) {
    stateReset();
    feed((uint32_t)(4000 / ss) + 500, 150, 300, ss);
    const Result& r = stateResult();
    CHECK(r.ssValid && fabsf(r.ssHz - ss) < 0.05f * ss + 0.02f, "启停 %.2fHz → %.3fHz (%u 周期)",
          ss, r.ssHz, r.ssPeriods);
  }
  feed(1500, 0, 0, 0);
  CHECK(!stateResult().ssValid && stateResult().ssHz == 0, "启停停止 > 2 周期：启停频率失效，不报旧值");

  stateReset();
  feed(1200, 150, 300, 1.0f);
  CHECK(!stateResult().ssValid, "只有 1 个周期：启停频率未确立");

  feed(2000, 150, 300, 1.0f);
  stateSetSensorOk(false);
  const Result& r = stateResult();
  CHECK(!r.freqValid && !r.ssValid && r.vib == 0 && r.contact == 0, "断线：全部结果清零");

  printf("\n%s (%d 项失败)\n", g_fail ? "存在失败" : "全部通过", g_fail);
  return g_fail ? 1 : 0;
}
