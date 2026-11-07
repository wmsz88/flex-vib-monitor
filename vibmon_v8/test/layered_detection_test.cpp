#include "layered_detection.h"

#include "config.h"

#include <cmath>
#include <cstdio>

int main() {
  int pass = 0, fail = 0;
  auto check = [&](bool ok, const char* name) {
    ++(ok ? pass : fail);
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
  };

  layerInit();

  // Realtime vibration: three confirmed envelope points are required.
  LayerInput f = {};
  f.closed = 1;
  f.settled = 1;
  f.tUs = 0;
  f.envMg = 0;
  layerPush(&f);
  check(!layerRealtime().vib, "realtime starts off");

  for (int i = 1; i <= 3; ++i) {
    f.tUs = i * 10000;
    f.envMg = 200.0f;
    layerPush(&f);
  }
  check(layerRealtime().vib, "realtime vibration confirms");
  check(layerRealtime().contact, "realtime contact confirms");

  f.tUs = 40000;
  f.envMg = 60.0f;
  for (int i = 0; i < 3; ++i) layerPush(&f);
  check(!layerRealtime().vib, "realtime vibration clears");

  // Stable carrier: fixed frequency and envelope in a full window.
  layerInit();
  f = LayerInput{};
  f.closed = 1;
  f.settled = 1;
  f.freqValid = 1;
  f.ssValid = 1;
  for (int i = 0; i < LAYER_WINDOW_N; ++i) {
    f.tUs = i * 20000;
    f.envMg = 800.0f + (i % 2) * 10.0f;
    f.freqHz = 160.0f + (i % 2) * 0.2f;
    f.ssHz = 1.0f;
    layerPush(&f);
  }
  const UniformLayerResult u = layerUniform();
  check(u.stable, "uniform accepts stable carrier");
  check(std::abs(u.freqMedian - 160.1f) < 0.11f, "uniform frequency median");
  check(std::abs(u.envMedian - 805.0f) < 1.0f, "uniform envelope median");

  // Drifting signal must be rejected.
  layerInit();
  f = LayerInput{};
  f.closed = 1;
  f.settled = 1;
  f.freqValid = 1;
  f.ssValid = 1;
  for (int i = 0; i < LAYER_WINDOW_N; ++i) {
    f.tUs = i * 20000;
    f.envMg = 300.0f + i * 20.0f;
    f.freqHz = 120.0f + i * 6.0f;
    f.ssHz = 1.0f;
    layerPush(&f);
  }
  check(!layerUniform().stable, "uniform rejects drifting signal");

  std::printf("layered_detection_test: %d pass / %d fail\n", pass, fail);
  return fail ? 1 : 0;
}
