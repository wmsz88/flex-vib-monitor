// T010 改动 A 主机测试：黄灯三态（AUTO/ON/OFF）+ 绿灯心跳 + FAULT/CAL 不回归
//
// 编译（在 vibmon_v6 目录下；output.cpp 依赖 dsp.cpp 的 dspCal 系列符号，必须一起链）：
//   g++ -std=c++17 -Wall -I test -I . test/led_test.cpp output.cpp state.cpp dsp.cpp -o test/led_test.exe
//
// 只测 LED 驱动逻辑：挂 g_ledWriteHook 抓引脚电平，喂 'y'/'?' 走真实命令分发。

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "Arduino.h"
#include "output.h"
#include "state.h"
#include "config.h"
#include "sampler.h"
#include "posture.h"

/* ---------------- 虚拟时间 ---------------- */
static uint32_t g_us = 0;
uint32_t millis(void) { return g_us / 1000u; }
uint32_t micros(void) { return g_us; }
void delay(uint32_t ms) { g_us += ms * 1000u; }
void delayMicroseconds(uint32_t us) { g_us += us; }

SerialStub Serial;
SerialStub Serial1;

/* MPU / 采样器桩 */
float mpuLsbPerG(void) { return 8192.0f; }
float mpuLsbPerDps(void) { return 65.5f; }
bool mpuReadAccel(MpuRaw*) { return false; }
bool mpuReadAll(MpuRaw*) { return false; }
uint32_t mpuErrCount(void) { return 0; }
void mpuClearErr(void) {}
void mpuBusBegin(void) {}
MpuInitResult mpuInit(uint8_t*) { return MPU_OK; }
const char* mpuResultStr(MpuInitResult) { return ""; }

void samplerAddClip(void) {}
SamplerStartResult samplerStart(void) { return SMP_START_OK; }
SamplerStopResult samplerStop(void) { return SMP_STOP_OK; }
bool samplerReset(void) { return true; }
uint32_t samplerInFlight(void) { return 0; }
bool samplerStartIsError(SamplerStartResult) { return false; }
bool samplerStopIsError(SamplerStopResult) { return false; }
const char* samplerStartStr(SamplerStartResult) { return ""; }
const char* samplerStopStr(SamplerStopResult) { return ""; }
void samplerClearStats(void) {}
const SamplerStats& samplerStats(void) { static SamplerStats s = {}; return s; }
const TimingStats& samplerTiming(void) { static TimingStats t = {}; return t; }
void samplerTimingReset(void) {}
bool samplerPop(Sample*, SamplerGap*) { return false; }

/* 姿态桩 */
void postureInit(void) {}
void postureResetRuntime(void) {}
void postureInvalidateCal(void) {}
bool postureStartCal(void) { return true; }
bool postureCalCapturing(void) { return false; }
void postureOnFrame(const DspFrame&, uint32_t) {}
void postureOnGap(const SamplerGap&) {}
void postureTick(uint32_t) {}
const PostureResult& postureResult(void) { static PostureResult p = {}; return p; }
const char* postureClassName(uint8_t) { return "?"; }

/* ---------------- 引脚电平钩子 ---------------- */
static int g_pin[64];
static void ledHook(int pin, int val) { if (pin >= 0 && pin < 64) g_pin[pin] = val; }

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do {                                          \
  if (cond) { g_pass++; }                                              \
  else      { g_fail++; printf("[FAIL] "); printf(__VA_ARGS__); printf("\n"); } \
} while (0)

static void ledRun(uint32_t nowMs, uint8_t contact, uint8_t vib) {
  Result r = {};
  r.contact = contact;
  r.vib = vib;
  g_pin[PIN_LED_G] = -1;
  g_pin[PIN_LED_Y] = -1;
  ledUpdate(nowMs, r);
}

static void feedC(const char* s) {
  Serial.clearTx();
  for (const char* p = s; *p; p++) Serial.push(*p);
  outPollCmd();
}

int main(void) {
  ledWriteHook() = ledHook;
  printf("PIN_LED_G=%d PIN_LED_Y=%d\n", PIN_LED_G, PIN_LED_Y);

  const uint32_t ts[] = {0, 250, 499, 500, 749, 1000, 1249, 1500, 1750};
  const unsigned NT = sizeof(ts) / sizeof(ts[0]);

  /* L1 绿灯 1Hz 心跳不变（(nowMs/500)&1） */
  for (unsigned i = 0; i < NT; i++) {
    ledRun(ts[i], 0, 0);
    CHECK(g_pin[PIN_LED_G] == (int)((ts[i] / 500) & 1),
          "L1 绿心跳 t=%u 期望 %u 实得 %d",
          (unsigned)ts[i], (unsigned)((ts[i] / 500) & 1), g_pin[PIN_LED_G]);
  }

  /* L2 默认 AUTO：黄 = contact && (t/250)&1 —— 没接触时任何相位都灭 */
  for (unsigned i = 0; i < NT; i++) {
    ledRun(ts[i], 0, 1);
    CHECK(g_pin[PIN_LED_Y] == 0, "L2 AUTO ct=0 黄应灭 t=%u 实得 %d",
          (unsigned)ts[i], g_pin[PIN_LED_Y]);
  }
  for (unsigned i = 0; i < NT; i++) {
    ledRun(ts[i], 1, 0);
    CHECK(g_pin[PIN_LED_Y] == (int)((ts[i] / 250) & 1),
          "L2 AUTO ct=1 黄应 2Hz t=%u 期望 %u 实得 %d",
          (unsigned)ts[i], (unsigned)((ts[i] / 250) & 1), g_pin[PIN_LED_Y]);
  }

  /* L3 'y' 三态循环 AUTO -> ON -> OFF -> AUTO，每次都回显 */
  feedC("y");
  CHECK(strstr(Serial.tx(), "[LEDY] ON") != NULL, "L3 第一次 y 应回显 ON，实得 %s", Serial.tx());
  ledRun(250, 0, 0);
  CHECK(g_pin[PIN_LED_Y] == 1, "L3 ON：无接触也应常亮");
  ledRun(500, 1, 1);
  CHECK(g_pin[PIN_LED_Y] == 1, "L3 ON：相位 500 也应常亮");

  feedC("y");
  CHECK(strstr(Serial.tx(), "[LEDY] OFF") != NULL, "L3 第二次 y 应回显 OFF，实得 %s", Serial.tx());
  for (unsigned i = 0; i < NT; i++) {
    ledRun(ts[i], 1, 1);
    CHECK(g_pin[PIN_LED_Y] == 0, "L3 OFF：有接触也应常灭 t=%u", (unsigned)ts[i]);
  }

  feedC("y");
  CHECK(strstr(Serial.tx(), "[LEDY] AUTO") != NULL, "L3 第三次 y 应回显 AUTO，实得 %s", Serial.tx());
  ledRun(250, 1, 0);
  CHECK(g_pin[PIN_LED_Y] == 1, "L3 回到 AUTO：t=250 接触中应为亮");
  ledRun(500, 1, 0);
  CHECK(g_pin[PIN_LED_Y] == 0, "L3 回到 AUTO：t=500 接触中应为灭");

  /* L4 FAULT 4Hz 与 y 无关：绿灭、黄 (t/125)&1 */
  ledSet(LED_FAULT);
  for (unsigned i = 0; i < NT; i++) {
    ledRun(ts[i], 1, 1);
    CHECK(g_pin[PIN_LED_G] == 0, "L4 FAULT 绿应灭 t=%u", (unsigned)ts[i]);
    CHECK(g_pin[PIN_LED_Y] == (int)((ts[i] / 125) & 1),
          "L4 FAULT 黄应 4Hz t=%u 期望 %u 实得 %d",
          (unsigned)ts[i], (unsigned)((ts[i] / 125) & 1), g_pin[PIN_LED_Y]);
  }
  ledSet(LED_RUN);

  /* L5 CAL：ledSet 点亮两灯，ledUpdate 不覆盖 */
  ledSet(LED_CAL);
  CHECK(g_pin[PIN_LED_G] == 1 && g_pin[PIN_LED_Y] == 1, "L5 CAL 应两灯都亮");
  /* 这里故意不清 g_pin：若 ledUpdate 在 CAL 下动了引脚，值就会变，能直接看出来 */
  { Result rc = {}; rc.contact = 1; rc.vib = 1; ledUpdate(750, rc); }
  CHECK(g_pin[PIN_LED_G] == 1 && g_pin[PIN_LED_Y] == 1, "L5 CAL 下 ledUpdate 不应改灯");
  ledSet(LED_RUN);

  /* L6 '?' 帮助里应有 y */
  feedC("?");
  CHECK(strstr(Serial.tx(), "y 黄灯") != NULL, "L6 帮助应含 'y 黄灯'");

  printf("led_test: %d pass / %d fail\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
