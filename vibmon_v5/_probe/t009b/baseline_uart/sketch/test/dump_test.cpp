#line 1 "D:\\大学\\课程\\工科赛\\代码\\vibmon_v5\\test\\dump_test.cpp"
// v5 'D' 鍛戒护锛圖UMP 鍏ㄩ噺鐘舵€?CSV锛変富鏈烘祴璇?//
// 缂栬瘧锛堝湪 vibmon_v5 鐩綍涓嬶級锛?//   g++ -std=c++17 -Wall -I test -I . test/dump_test.cpp output.cpp state.cpp dsp.cpp -o test/dump_test.exe
//
// 鍙祴绾牸寮忓寲鍑芥暟 outBuildDump() 涓庤〃澶村父閲?OUT_DUMP_HEADER锛?//   - 琛ㄥご鍒楁暟涓庢暟鎹鍒楁暟涓€鑷达紙閫楀彿鏁扮浉鍚岋級鈥斺€旈槻姝互鍚庡姞鍒楀彧鏀逛簡鏁版嵁娌℃敼琛ㄥご
//   - 闈欐€侊紙鏃犲熀鍑嗐€佹棤鎸姩锛変笌妯℃嫙鎺ヨЕ鎬佷袱渚嬬殑瀹屾暣鏂囨湰
//   - 缂撳啿鍖轰繚鎶?
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

/* ---------------- Arduino / 澶栬妗?---------------- */
static uint32_t g_us = 0;
uint32_t millis(void) { return g_us / 1000u; }
uint32_t micros(void) { return g_us; }
void delay(uint32_t ms) { g_us += ms * 1000u; }
void delayMicroseconds(uint32_t us) { g_us += us; }

SerialStub Serial;
SerialStub Serial1;

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

/* ---------------- 杈呭姪 ---------------- */
static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("FAIL: %s\n", msg); } \
  } while (0)

static int countChar(const char* s, char c) {
  int n = 0;
  for (; *s; s++) if (*s == c) n++;
  return n;
}

/* ---------------- 鐢ㄤ緥 ---------------- */
/* 琛ㄥご鍒楁暟 == 鏁版嵁琛屽垪鏁?*/
static void testColumnsMatch(void) {
  Result r = {};
  char buf[192];
  int n = outBuildDump(r, buf, (int)sizeof(buf));
  CHECK(n > 0, "dump line builds");
  if (n > 0) {
    /* 鍓嶇紑("#DUMPH" / "#DUMP")涓嶈锛屽彧姣斿悗闈㈢殑瀛楁閫楀彿鏁?*/
    const int hCommas = countChar(OUT_DUMP_HEADER + 6, ',');
    const int dCommas = countChar(buf + 5, ',');
    CHECK(hCommas == dCommas, "header and data have same field count");
    CHECK(hCommas == 23, "23 separators -> 24 fields");
  }
  /* 鐜板満蹇呴渶鐨勫垪閮藉湪琛ㄥご閲?*/
  CHECK(strstr(OUT_DUMP_HEADER, ",closed,") != nullptr, "header has closed");
  CHECK(strstr(OUT_DUMP_HEADER, ",flatDist,") != nullptr, "header has flatDist");
  CHECK(strstr(OUT_DUMP_HEADER, ",closeAngleDeg,") != nullptr, "header has closeAngleDeg");
  CHECK(strstr(OUT_DUMP_HEADER, ",condPosture,") != nullptr, "header has condPosture");
  CHECK(strstr(OUT_DUMP_HEADER, ",condAmp,") != nullptr, "header has condAmp");
  CHECK(strstr(OUT_DUMP_HEADER, ",condFreq,") != nullptr, "header has condFreq");
  CHECK(strstr(OUT_DUMP_HEADER, ",ct,") != nullptr, "header has ct");
  CHECK(strstr(OUT_DUMP_HEADER, ",ctValid,") != nullptr, "header has ctValid");
}

/* 闈欐€侊細鏈噰鍩哄噯銆佹棤鎸姩銆佽搴︽棤鏁?*/
static void testStatic(void) {
  g_us = 0;
  Result r = {};
  char buf[192];
  int n = outBuildDump(r, buf, (int)sizeof(buf));
  CHECK(n == (int)strlen(buf), "return value equals length");
  CHECK(strcmp(buf, "#DUMP,0,0,-1.0,0,0.0,0,0,0,0,0,0,0,0.0,0,0.0,0,0.00,0,0,0.0,0,0,0") == 0,
        "static line matches expected text");
}

/* 妯℃嫙鎺ヨЕ锛氫笁璺垽鎹叏 1 + 鍏ㄩ儴閲忔湁鏁?*/
static void testContact(void) {
  g_us = 12345 * 1000u;   // t_ms = 12345
  Result r = {};
  r.sensorOk = true;
  r.calOk = true;
  r.vib = 1;
  r.freqValid = true;  r.freqHz = 165.2f;
  r.ampValid  = true;  r.ampMg  = 965.7f;
  r.ssValid   = true;  r.ssHz   = 1.20f;
  r.contact = 1; r.contactValid = true; r.closed = true;
  r.flatDist = 486.4f; r.flatRefSet = true;
  r.closeAngleDeg = 72.5f; r.angleValid = true;
  r.condPosture = 1; r.condAmp = 1; r.condFreq = 1;
  r.gyroW = 12.3f; r.axis = 2;
  char buf[192];
  int n = outBuildDump(r, buf, (int)sizeof(buf));
  CHECK(n > 0, "contact line builds");
  CHECK(strcmp(buf, "#DUMP,12345,1,486.4,1,72.5,1,1,1,1,1,1,1,165.2,1,965.7,1,1.20,1,0,12.3,2,0,1") == 0,
        "contact line matches expected text");
}

/* 绌哄す锛氭彙涓嬶紙condPosture=1锛変絾骞呭€?棰戠巼娌¤捣鏉?*/
static void testGripNoObject(void) {
  g_us = 0;
  Result r = {};
  r.closed = true; r.gripNoObject = true;
  r.flatDist = 520.0f; r.flatRefSet = true;
  r.condPosture = 1; r.condAmp = 0; r.condFreq = 0;
  r.contactValid = true;   // 鍩哄噯鏈夋晥锛屼絾 contact 鍒や笉鍑烘潵
  char buf[192];
  outBuildDump(r, buf, (int)sizeof(buf));
  CHECK(strcmp(buf, "#DUMP,0,1,520.0,1,0.0,0,1,0,0,0,1,0,0.0,0,0.0,0,0.00,0,1,0.0,0,0,0") == 0,
        "grip-no-object line shows condPosture=1, condAmp=0, condFreq=0");
}

static void testBufferProtection(void) {
  Result r = {};
  char tiny[32];
  CHECK(outBuildDump(r, tiny, (int)sizeof(tiny)) == -1, "buffer too small returns -1");
  CHECK(outBuildDump(r, nullptr, 192) == -1, "null buffer returns -1");
  char ok[192];
  CHECK(outBuildDump(r, ok, (int)sizeof(ok)) > 0, "normal buffer builds");
}

int main(void) {
  testColumnsMatch();
  testStatic();
  testContact();
  testGripNoObject();
  testBufferProtection();
  printf("dump_test: %d pass / %d fail\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}