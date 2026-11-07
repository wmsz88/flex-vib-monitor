#pragma once

#include <stdint.h>

struct LayerInput {
  uint32_t tUs;
  float envMg;
  float freqHz;
  float ssHz;
  uint8_t closed;
  uint8_t settled;
  uint8_t freqValid;
  uint8_t ssValid;
};

struct RealtimeLayerResult {
  uint8_t vib;
  uint8_t contact;
};

struct UniformLayerResult {
  float freqMedian;
  float freqSpread;
  float envMedian;
  float envSpread;
  float ssMedian;
  uint8_t validRatio;
  uint8_t stable;
};

void layerInit(void);
void layerPush(const LayerInput* in);
RealtimeLayerResult layerRealtime(void);
UniformLayerResult layerUniform(void);
