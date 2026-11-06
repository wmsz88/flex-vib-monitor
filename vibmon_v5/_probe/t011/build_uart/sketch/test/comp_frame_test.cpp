#line 1 "D:\\大学\\课程\\工科赛\\代码\\vibmon_v5\\test\\comp_frame_test.cpp"
// v5 串口比赛帧协议主机测试
//
// 编译（在 vibmon_v5 目录下；output.cpp 依赖 dsp.cpp 的 dspCal 系列符号，必须一起链）：
//   g++ -std=c++17 -Wall -I test -I . test/comp_frame_test.cpp output.cpp state.cpp dsp.cpp -o test/comp_frame_test.exe
//   g++ -std=c++17 -Wall -DCOMP_FRAME_CL=1 -I test -I . test/comp_frame_test.cpp output.cpp state.cpp dsp.cpp -o test/comp_frame_test_cl1.exe
// 两条都要跑：默认构建断言 35 字节帧（与历史逐字节一致），-DCOMP_FRAME_CL=1 断言 37 字节帧含 CL。
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

/* 帧长随 COMP_FRAME_CL 变化：0 = 35 字节（无 CL）／1 = 37 字节（CT 后插 CL）。
 * 本测试直接 include config.h，所以两种模式用同一个可执行文件是做不到的：
 * 要用 -DCOMP_FRAME_CL=1 重新编译，同一份断言按下面的常量自动切换。 */
#if COMP_FRAME_CL
  #define BODY_LEN   32
  #define FRAME_LEN  37
  #define OUT_MIN    38
  #define BODY_ZERO  "$VD,0000,00000,00000,0,0,0,000,0"
  #define BODY_CTACT "$VD,0137,01652,00966,1,1,1,120,F"
#else
  #define BODY_LEN   30
  #define FRAME_LEN  35
  #define OUT_MIN    36
  #define BODY_ZERO  "$VD,0000,00000,00000,0,0,000,0"
  #define BODY_CTACT "$VD,0137,01652,00966,1,1,120,F"
#endif

/* 正文内的字段偏移：seq/freq/amp 两种模式相同；ss 与 valid 随 CL 后移 2 字节 */
#define OFF_SEQ   4
#define OFF_FREQ  9
#define OFF_AMP   15
#define OFF_CT    23
#define OFF_CL    25                  /* 仅 COMP_FRAME_CL=1 时存在 */
#define OFF_SS    (BODY_LEN - 5)
#define OFF_VALID (BODY_LEN - 1)

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
  CHECK(n == FRAME_LEN, "all-zero frame length matches mode");
  if (n == FRAME_LEN) {
    CHECK(memcmp(buf, BODY_ZERO, BODY_LEN) == 0,
          "all-zero body matches expected format");
    CHECK(buf[FRAME_LEN - 2] == '\r' && buf[FRAME_LEN - 1] == '\n',
          "frame ends with CRLF");
    char body[64];
    memcpy(body, buf, BODY_LEN);
    uint8_t expect = xorChk(body, BODY_LEN);
    char chkHex[4];
    snprintf(chkHex, sizeof(chkHex), "%02X", expect);
    CHECK(buf[BODY_LEN + 1] == chkHex[0] && buf[BODY_LEN + 2] == chkHex[1],
          "checksum matches");
  }
}

static void testContact(void) {
  Result r = makeContact();
  char buf[64];
  int n = compBuildFrame(r, 137, buf, sizeof(buf));
  CHECK(n == FRAME_LEN, "contact frame length matches mode");
  if (n == FRAME_LEN) {
    CHECK(memcmp(buf, BODY_CTACT, BODY_LEN) == 0,
          "contact body matches expected format");
  }
#if COMP_FRAME_CL
  /* CL 位专项：只翻转 closed，看 CL 位与校验是否跟着变 */
  Result r0 = makeContact();
  r0.closed = false;
  char buf0[64];
  compBuildFrame(r0, 137, buf0, sizeof(buf0));
  CHECK(buf[OFF_CL] == '1' && buf0[OFF_CL] == '0', "CL=1 reflects closed=true / false");
  CHECK(memcmp(buf0, "$VD,0137,01652,00966,1,1,0,120,F", 32) == 0,
        "closed=false body is $VD,...,CT=1,CL=0,...");
  CHECK(buf[BODY_LEN + 1] != buf0[BODY_LEN + 1] ||
        buf[BODY_LEN + 2] != buf0[BODY_LEN + 2],
        "checksum covers CL field (differs when CL flips)");
#endif
}

static void testSeqWrap(void) {
  Result r = makeContact();
  char buf[64];
  compBuildFrame(r, 9998, buf, sizeof(buf));
  CHECK(memcmp(buf + OFF_SEQ, "9998", 4) == 0, "seq=9998 correct");
  compBuildFrame(r, 9999, buf, sizeof(buf));
  CHECK(memcmp(buf + OFF_SEQ, "9999", 4) == 0, "seq=9999 correct");
  compBuildFrame(r, 10000, buf, sizeof(buf));
  CHECK(memcmp(buf + OFF_SEQ, "0000", 4) == 0, "seq=10000 wraps to 0000");
}

static void testInvalidFields(void) {
  Result r = {};
  r.ampValid = true; r.ampMg = 500.0f;
  char buf[64];
  compBuildFrame(r, 42, buf, sizeof(buf));
  CHECK(memcmp(buf + OFF_SEQ, "0042", 4) == 0, "seq=42 padded");
  CHECK(memcmp(buf + OFF_FREQ, "00000", 5) == 0, "invalid freq -> 00000");
  CHECK(memcmp(buf + OFF_AMP, "00500", 5) == 0, "valid amp -> 00500");
  CHECK(memcmp(buf + OFF_SS, "000", 3) == 0, "invalid ss -> 000");
  CHECK(buf[OFF_VALID] == '1', "valid bitmap is '1' (amp only)");
}

static void testSaturation(void) {
  Result r = makeContact();
  r.freqHz = 9999.9f;   // *10 = 99999 exactly at limit
  r.ampMg = 123456.0f;
  r.ssHz = 9.99f;   // *100 = 999 exactly at limit
  char buf[64];
  compBuildFrame(r, 0, buf, sizeof(buf));
  CHECK(memcmp(buf + OFF_FREQ, "99999", 5) == 0, "freq clamped to 99999");
  CHECK(memcmp(buf + OFF_AMP, "99999", 5) == 0, "amp clamped to 99999");
  CHECK(memcmp(buf + OFF_SS, "999", 3) == 0, "ss clamped to 999");
}

static void testBufferProtection(void) {
  Result r = makeContact();
  char tiny[8];
  CHECK(compBuildFrame(r, 0, tiny, sizeof(tiny)) == -1, "buffer too small returns -1");
  char exact[FRAME_LEN];
  CHECK(compBuildFrame(r, 0, exact, sizeof(exact)) == -1, "buffer == frame len returns -1");
  char ok[OUT_MIN];
  CHECK(compBuildFrame(r, 0, ok, sizeof(ok)) == FRAME_LEN, "buffer == frame len+1 returns frame len");
  CHECK(compBuildFrame(r, 0, nullptr, 64) == -1, "null buffer returns -1");
}

static void testChecksumKnown(void) {
  Result r = {};
  r.ampValid = true; r.ampMg = 12.0f;
  r.contactValid = true; r.contact = 0;
  char buf[64];
  compBuildFrame(r, 0, buf, sizeof(buf));
  uint8_t expect = xorChk(buf, BODY_LEN);
  char chkHex[4];
  snprintf(chkHex, sizeof(chkHex), "%02X", expect);
  CHECK(buf[BODY_LEN + 1] == chkHex[0] && buf[BODY_LEN + 2] == chkHex[1],
        "checksum is consistent with xorChk");
}

int main(void) {
  printf("mode: COMP_FRAME_CL=%d, frame=%d bytes, body=%d bytes, min buffer=%d\n",
         COMP_FRAME_CL, FRAME_LEN, BODY_LEN, OUT_MIN);
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