// 三轴姿态诊断：仅用于 START/HONGEST 辅助判断，不参与正式 contact。
#pragma once
#include <stdint.h>
#include "dsp.h"
#include "sampler.h"

enum PostureClass : uint8_t {
  POSTURE_UNKNOWN = 0,
  POSTURE_START = 1,
  POSTURE_HONGEST = 2,
};

struct PostureResult {
  bool calValid;
  bool calActive;
  bool calFailed;
  bool valid;
  uint8_t cls;
  uint8_t stableCls;
  bool stable;
  uint8_t stableCount;
  float distMg;
  float x, y, z;
  float startX, startY, startZ;
  uint32_t blockN;
  uint32_t calN;
};

void postureInit(void);
void postureResetRuntime(void);
void postureInvalidateCal(void);
bool postureStartCal(void);
bool postureCalCapturing(void);
void postureOnFrame(const DspFrame& f, uint32_t tUs);
void postureOnGap(const SamplerGap& gap);
void postureTick(uint32_t nowUs);
const PostureResult& postureResult(void);
const char* postureClassName(uint8_t cls);
