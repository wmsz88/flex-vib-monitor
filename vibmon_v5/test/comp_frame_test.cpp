// v5 串口比赛帧协议主机测试
//
// 编译：
//   g++ -std=c++17 -Wall -I test -I .. test/comp_frame_test.cpp ../output.cpp ../state.cpp -o test/comp_frame_test.exe
//
// 只测纯编码函数 compBuildFrame()：构造已知 Result 值，
// 验证帧格式、字段宽度、校验和、缓冲区保护。

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

static uint8_t xorChk(const char* s, int n) {
  uint8_t x = 0;
  for (int i = 0; i < n; i++) x ^= (uint8_t)s[i];
  return x;
}

/* 构造一个"典型接触"Result */
static Result makeContact(void) {
  Result r = {};
  r.sensorOk = true;
  r.calOk = true;
  r.calQuality = true;
  r.vib = 1;
  r.freqValid = true;
  r.freqHz = 165.2f;
  r.ampValid = true;
  r.ampMg = 965.7f;
  r.ssValid = true;
  r.ssHz = 1.20f;
  r.ssPeriods = 4;
  r.contact = 1;
  r.contactValid = true;
  r.closed = true;
  return r;
}

/* ---------------- 测试用例 ---------------- */
static void testAllZero(void) {
  Result r = {};   // 全零：所有字段无效
  char buf[64];
  int n = compBuildFrame(r, 0, buf, sizeof(buf));
  printf("[allzero] len=%d frame=[%s]\n", n, n > 0 ? buf : "?");
  CHECK(n == 35, "all-zero frame is 35 bytes");
  if (n == 35) {
    CHECK(memcmp(buf, "$VD,0000,00000,00000,0,0,000,0", 30) == 0,
          "all-zero body matches expected format");
    CHECK(buf[33] == '\r' && buf[34] == '\n', "frame ends with CRLF");
    char body[64];
    memcpy(body, buf, 30);
    uint8_t expect = xorChk(body, 30);
    char chkHex[4];
    snprintf(chkHex, sizeof(chkHex), "%02X", expect);
    CHECK(buf[31] == chkHex[0] && buf[32] == chkHex[1], "checksum matches");
  }
}

static void testContact(void) {
  Result r = makeContact();
  char buf[64];
  int n = compBuildFrame(r, 137, buf, sizeof(buf));
  printf("[contact] len=%d frame=[%s]\n", n, n > 0 ? buf : "?");
  CHECK(n == 35, "contact frame is 35 bytes");
  if (n == 35) {
    CHECK(memcmp(buf, "$VD,0137,01652,00966,1,1,120,F", 30) == 0,
          "contact body matches expected format");
  }
}

static void testSeqWrap(void) {
  Result r = makeContact();
  char buf[64];
  compBuildFrame(r, 9998, buf, sizeof(buf));
  CHECK(memcmp(buf + 4, "9998", 4) == 0, "seq=9998 correct");
  compBuildFrame(r, 9999, buf, sizeof(buf));
  CHECK(memcmp(buf + 4, "9999", 4) == 0, "seq=9999 correct");
  compBuildFrame(r, 10000, buf, sizeof(buf));
  CHECK(memcmp(buf + 4, "0000", 4) == 0, "seq=10000 wraps to 0000");
}

static void testInvalidFields(void) {
  Result r = {};
  r.ampValid = true; r.ampMg = 500.0f;
  char buf[64];
  compBuildFrame(r, 42, buf, sizeof(buf));
  CHECK(memcmp(buf + 4, "0042", 4) == 0, "seq=42 padded");
  CHECK(memcmp(buf + 9, "00000", 5) == 0, "invalid freq -> 00000");
  CHECK(memcmp(buf + 15, "00500", 5) == 0, "valid amp -> 00500");
  CHECK(memcmp(buf + 25, "000", 3) == 0, "invalid ss -> 000");
  CHECK(buf[29] == '1', "valid bitmap is '1' (amp only)");
}

static void testSaturation(void) {
  Result r = makeContact();
  r.freqHz = 9999.9f;   // *10 = 99999 exactly at limit
  r.ampMg = 123456.0f;
  r.ssHz = 9.99f;   // *100 = 999 exactly at limit
  char buf[64];
  compBuildFrame(r, 0, buf, sizeof(buf));
  CHECK(memcmp(buf + 9, "99999", 5) == 0, "freq clamped to 99999");
  CHECK(memcmp(buf + 15, "99999", 5) == 0, "amp clamped to 99999");
  CHECK(memcmp(buf + 25, "999", 3) == 0, "ss clamped to 999");
}

static void testBufferProtection(void) {
  Result r = makeContact();
  char tiny[8];
  CHECK(compBuildFrame(r, 0, tiny, sizeof(tiny)) == -1, "buffer too small returns -1");
  char exact[35];
  CHECK(compBuildFrame(r, 0, exact, sizeof(exact)) == -1, "buffer 35 returns -1");
  char ok[36];
  CHECK(compBuildFrame(r, 0, ok, sizeof(ok)) == 35, "buffer 36 returns 35");
  CHECK(compBuildFrame(r, 0, nullptr, 64) == -1, "null buffer returns -1");
}

static void testChecksumKnown(void) {
  Result r = {};
  r.ampValid = true; r.ampMg = 12.0f;
  r.contactValid = true; r.contact = 0;
  char buf[64];
  compBuildFrame(r, 0, buf, sizeof(buf));
  uint8_t expect = xorChk(buf, 30);
  char chkHex[4];
  snprintf(chkHex, sizeof(chkHex), "%02X", expect);
  CHECK(buf[31] == chkHex[0] && buf[32] == chkHex[1], "checksum is consistent with xorChk");
}

int main(void) {
  testAllZero();
  testContact();
  testSeqWrap();
  testInvalidFields();
  testSaturation();
  testBufferProtection();
  testChecksumKnown();
  printf("comp_frame_test: %d pass / %d fail\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}