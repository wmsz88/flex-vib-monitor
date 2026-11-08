// 比赛帧（0x55 二进制帧）主机测试
//
// 现场口径：9600 8N1
//   0x55 | 振动整数 | 振动小数 | 启停 | 接触 | 停止位(0x0D)
//
// 编译（在 vibmon_v6 目录下）：
//   g++ -std=c++17 -Wall -I test -I . test/comp_frame_test.cpp output.cpp state.cpp dsp.cpp -o test/comp_frame_test.exe
//
// 只测纯编码函数 compBuildFrame()：构造已知 Result 值，
// 验证帧头、字段顺序、0.1Hz 拆分、四舍五入、缓冲区保护。

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "Arduino.h"
#include "output.h"
#include "state.h"
#include "config.h"
#include "sampler.h"
#include "posture.h"

/* ---------------- Arduino / 外设桩 ---------------- */
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

/* 状态函数使用 state.cpp 真实实现（链接时提供） */

/* ---------------- 辅助函数 ---------------- */
static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("FAIL: %s\n", msg); } \
  } while (0)

#if COMP_FRAME_TRAILER_EN
  #define FRAME_LEN 6
#else
  #define FRAME_LEN 5
#endif

static uint8_t byteAt(const char* buf, int i) { return (uint8_t)buf[i]; }

/* 典型接触：165.2Hz、振动中、已接触 */
static Result makeContact(void) {
  Result r = {};
  r.sensorOk = true;
  r.calOk = true;
  r.vib = 1;
  r.freqValid = true;
  r.freqHz = 165.2f;
  r.contact = 1;
  r.contactValid = true;
  return r;
}

static void testAllZero(void) {
  Result r = {};
  char buf[16];
  int n = compBuildFrame(r, 0, buf, sizeof(buf));
  CHECK(n == FRAME_LEN, "all-zero frame length");
  CHECK(byteAt(buf, 0) == COMP_FRAME_HEADER, "all-zero starts with 0x55");
  CHECK(byteAt(buf, 1) == 0 && byteAt(buf, 2) == 0, "invalid freq -> 0,0");
  CHECK(byteAt(buf, 3) == 0 && byteAt(buf, 4) == 0, "vib/contact -> 0,0");
#if COMP_FRAME_TRAILER_EN
  CHECK(byteAt(buf, 5) == COMP_FRAME_TRAILER, "trailer byte");
#endif
}

static void testContact(void) {
  Result r = makeContact();
  char buf[16];
  int n = compBuildFrame(r, 137, buf, sizeof(buf));
  CHECK(n == FRAME_LEN, "contact frame length");
  CHECK(byteAt(buf, 0) == 0x55, "header is 0x55");
  CHECK(byteAt(buf, 1) == 165, "165.2Hz integer part");
  CHECK(byteAt(buf, 2) == 2, "165.2Hz decimal part");
  CHECK(byteAt(buf, 3) == 1, "vib=1");
  CHECK(byteAt(buf, 4) == 1, "contact=1");
#if COMP_FRAME_TRAILER_EN
  CHECK(byteAt(buf, 5) == COMP_FRAME_TRAILER, "trailer is 0x0D");
#endif
}

static void testRounding(void) {
  Result r = makeContact();
  char buf[16];

  r.freqHz = 165.24f;
  compBuildFrame(r, 0, buf, sizeof(buf));
  CHECK(byteAt(buf, 1) == 165 && byteAt(buf, 2) == 2, "165.24 -> 165,2");

  r.freqHz = 165.26f;
  compBuildFrame(r, 0, buf, sizeof(buf));
  CHECK(byteAt(buf, 1) == 165 && byteAt(buf, 2) == 3, "165.26 -> 165,3");

  r.freqHz = 99.99f;
  compBuildFrame(r, 0, buf, sizeof(buf));
  CHECK(byteAt(buf, 1) == 100 && byteAt(buf, 2) == 0, "99.99 -> 100,0");
}

static void testInvalidFreq(void) {
  Result r = makeContact();
  r.freqValid = false;
  r.freqHz = 165.2f;
  char buf[16];
  compBuildFrame(r, 0, buf, sizeof(buf));
  CHECK(byteAt(buf, 1) == 0 && byteAt(buf, 2) == 0,
        "freqValid=false sends 0,0 even if freqHz is set");
}

static void testClamp(void) {
  Result r = makeContact();
  r.freqHz = 300.0f;
  char buf[16];
  compBuildFrame(r, 0, buf, sizeof(buf));
  CHECK(byteAt(buf, 1) == 255, "integer part clamps to 255");
}

static void testFlags(void) {
  Result r = makeContact();
  char buf[16];

  r.vib = 0;
  r.contact = 0;
  compBuildFrame(r, 0, buf, sizeof(buf));
  CHECK(byteAt(buf, 3) == 0 && byteAt(buf, 4) == 0, "vib/contact both 0");

  r.vib = 1;
  r.contact = 0;
  compBuildFrame(r, 0, buf, sizeof(buf));
  CHECK(byteAt(buf, 3) == 1 && byteAt(buf, 4) == 0, "vib only");
}

static void testBufferProtection(void) {
  Result r = makeContact();
  char exact[FRAME_LEN];
  char ok[FRAME_LEN + 1];
  CHECK(compBuildFrame(r, 0, exact, sizeof(exact)) == -1,
        "buffer == frame len returns -1");
  CHECK(compBuildFrame(r, 0, ok, sizeof(ok)) == FRAME_LEN,
        "buffer == frame len+1 returns frame len");
  CHECK(compBuildFrame(r, 0, nullptr, 64) == -1, "null buffer returns -1");
  CHECK(compBuildFrame(r, 0, ok, 0) == -1, "zero-sized buffer returns -1");
}

static void testSeqIgnored(void) {
  Result r = makeContact();
  char a[16], b[16];
  int n1 = compBuildFrame(r, 0, a, sizeof(a));
  int n2 = compBuildFrame(r, 65535, b, sizeof(b));
  CHECK(n1 == n2 && memcmp(a, b, (size_t)n1) == 0,
        "seq is not part of the field frame");
}

int main(void) {
  printf("binary frame: %d bytes (trailer=%d)\n", FRAME_LEN, COMP_FRAME_TRAILER_EN);
  testAllZero();
  testContact();
  testRounding();
  testInvalidFreq();
  testClamp();
  testFlags();
  testBufferProtection();
  testSeqIgnored();
  printf("comp_frame_test: %d pass / %d fail\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
