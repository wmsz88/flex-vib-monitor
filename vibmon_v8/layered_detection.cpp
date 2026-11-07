#include "layered_detection.h"

#include "config.h"

#include <math.h>
#include <string.h>

static RealtimeLayerResult s_realtime;
static UniformLayerResult s_uniform;

static float s_freq[LAYER_WINDOW_N];
static float s_env[LAYER_WINDOW_N];
static float s_ss[LAYER_WINDOW_N];
static uint8_t s_freqValid[LAYER_WINDOW_N];
static uint8_t s_ssValid[LAYER_WINDOW_N];
static uint16_t s_head;
static uint16_t s_count;

static uint8_t s_hiCnt;
static uint8_t s_loCnt;
static uint8_t s_realtimeContact;

static float medianOf(const float* src, uint8_t n) {
  float tmp[LAYER_WINDOW_N];
  memcpy(tmp, src, (size_t)n * sizeof(float));
  for (uint8_t i = 1; i < n; ++i) {
    float v = tmp[i];
    int8_t j = i - 1;
    while (j >= 0 && tmp[j] > v) {
      tmp[j + 1] = tmp[j];
      --j;
    }
    tmp[j + 1] = v;
  }
  return (n & 1u) ? tmp[n / 2u] : 0.5f * (tmp[n / 2u - 1u] + tmp[n / 2u]);
}

static float percentileOf(const float* src, uint8_t n, uint8_t percent) {
  float tmp[LAYER_WINDOW_N];
  memcpy(tmp, src, (size_t)n * sizeof(float));
  for (uint8_t i = 1; i < n; ++i) {
    float v = tmp[i];
    int8_t j = i - 1;
    while (j >= 0 && tmp[j] > v) {
      tmp[j + 1] = tmp[j];
      --j;
    }
    tmp[j + 1] = v;
  }
  uint8_t idx = (uint8_t)(((uint16_t)n * percent + 50u) / 100u);
  if (idx >= n) idx = n - 1u;
  return tmp[idx];
}

void layerInit(void) {
  s_realtime = RealtimeLayerResult{};
  s_uniform = UniformLayerResult{};
  memset(s_freq, 0, sizeof(s_freq));
  memset(s_env, 0, sizeof(s_env));
  memset(s_ss, 0, sizeof(s_ss));
  memset(s_freqValid, 0, sizeof(s_freqValid));
  memset(s_ssValid, 0, sizeof(s_ssValid));
  s_head = 0;
  s_count = 0;
  s_hiCnt = 0;
  s_loCnt = 0;
  s_realtimeContact = 0;
}

void layerPush(const LayerInput* in) {
  if (!in) return;

  // Realtime vibration uses the same hysteresis/confirmation idea as V6 SS.
  if (s_realtime.vib) {
    if (in->envMg < LAYER_VIB_LO_MG) {
      s_hiCnt = 0;
      if (++s_loCnt >= LAYER_VIB_CONFIRM) {
        s_realtime.vib = 0;
        s_loCnt = 0;
      }
    } else {
      s_loCnt = 0;
    }
  } else {
    if (in->envMg > LAYER_VIB_HI_MG) {
      s_loCnt = 0;
      if (++s_hiCnt >= LAYER_VIB_CONFIRM) {
        s_realtime.vib = 1;
        s_hiCnt = 0;
      }
    } else {
      s_hiCnt = 0;
    }
  }

  // Contact follows V6 semantics: closed, settled, and vibration evidence.
  if (!in->closed) {
    s_realtimeContact = 0;
  } else if (!s_realtimeContact) {
    const uint8_t evidence = (s_realtime.vib || in->ssValid || in->freqValid);
    s_realtimeContact = (in->settled && evidence) ? 1u : 0u;
  }
  s_realtime.contact = s_realtimeContact;

  // Fixed-size sliding window; no heap allocation.
  s_freq[s_head] = in->freqHz;
  s_env[s_head] = in->envMg;
  s_ss[s_head] = in->ssHz;
  s_freqValid[s_head] = in->freqValid;
  s_ssValid[s_head] = in->ssValid;
  s_head = (uint16_t)((s_head + 1u) % LAYER_WINDOW_N);
  if (s_count < LAYER_WINDOW_N) ++s_count;

  float freq[LAYER_WINDOW_N];
  float env[LAYER_WINDOW_N];
  float ss[LAYER_WINDOW_N];
  uint8_t nFreq = 0;
  uint8_t nEnv = 0;
  uint8_t nSs = 0;
  for (uint16_t i = 0; i < s_count; ++i) {
    const uint16_t idx = (uint16_t)((s_head + LAYER_WINDOW_N - s_count + i) %
                                    LAYER_WINDOW_N);
    if (s_freqValid[idx] && isfinite(s_freq[idx]) && s_freq[idx] > 0.0f) {
      freq[nFreq++] = s_freq[idx];
    }
    if (isfinite(s_env[idx]) && s_env[idx] >= 0.0f) {
      env[nEnv++] = s_env[idx];
    }
    if (s_ssValid[idx] && isfinite(s_ss[idx]) && s_ss[idx] > 0.0f) {
      ss[nSs++] = s_ss[idx];
    }
  }

  s_uniform = UniformLayerResult{};
  if (nFreq > 0) {
    s_uniform.freqMedian = medianOf(freq, nFreq);
    const float hi = percentileOf(freq, nFreq, 90);
    const float lo = percentileOf(freq, nFreq, 10);
    s_uniform.freqSpread =
        s_uniform.freqMedian > 0.0f ? (hi - lo) / s_uniform.freqMedian : 1.0f;
  }
  if (nEnv > 0) {
    s_uniform.envMedian = medianOf(env, nEnv);
    const float hi = percentileOf(env, nEnv, 90);
    const float lo = percentileOf(env, nEnv, 10);
    s_uniform.envSpread =
        s_uniform.envMedian > 0.0f ? (hi - lo) / s_uniform.envMedian : 1.0f;
  }
  if (nSs > 0) s_uniform.ssMedian = medianOf(ss, nSs);

  const uint16_t validCount = nFreq < nSs ? nFreq : nSs;
  const uint16_t validRatio = s_count ? (validCount * 100u) / s_count : 0u;
  s_uniform.validRatio = (uint8_t)validRatio;
  s_uniform.stable = (nFreq > 0 && s_uniform.freqSpread <= LAYER_FREQ_SPREAD_MAX &&
                      s_uniform.envSpread <= LAYER_ENV_SPREAD_MAX &&
                      validRatio >= (uint8_t)(LAYER_VALID_RATIO_MIN * 100.0f)) ? 1u : 0u;
}

RealtimeLayerResult layerRealtime(void) { return s_realtime; }
UniformLayerResult layerUniform(void) { return s_uniform; }
