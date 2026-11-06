#include <stdio.h>
#include <string.h>

#include "Arduino.h"
#include "config.h"
#include "output.h"
#include "state.h"
#include "sampler.h"
#include "posture.h"

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

static int countComma(const char* text) {
  int count = 0;
  for (; *text; ++text) {
    if (*text == ',') ++count;
  }
  return count;
}

int main(void) {
  Result result = {};
  char line[256] = {};
  const int n = outBuildDump(result, line, sizeof(line));

  const int headerCommas = countComma(OUT_DUMP_HEADER + 6);
  const int dataCommas = countComma(line + 5);
  const int checks = 5;
  int passed = 0;

  passed += strstr(OUT_DUMP_HEADER, ",holdOk,msSinceEvidence") != nullptr;
  passed += headerCommas == dataCommas;
  passed += headerCommas == 25;
  passed += n > 0;
  passed += strstr(line, "#DUMP,") == line;

  printf(
      "dump_hold_probe: %d/%d pass, header_commas=%d data_commas=%d\n",
      passed, checks, headerCommas, dataCommas);
  return passed == checks ? 0 : 1;
}
