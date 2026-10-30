#include "posture.h"

#include <math.h>

#include "config.h"
#include "output.h"

namespace {

enum CalPhase : uint8_t {
  CAL_IDLE = 0,
  CAL_SETTLE,
  CAL_AVG,
};

static PostureResult s_p = {};
static CalPhase s_calPhase = CAL_IDLE;
static uint32_t s_calStartUs = 0;
static uint32_t s_calAvgStartUs = 0;
static float s_calSum[3];
static float s_calMin[3];
static float s_calMax[3];
static uint32_t s_calN;
static float s_calMaxGyro;

static uint32_t s_blockStartUs = 0;
static float s_blockSum[3];
static float s_blockMin[3];
static float s_blockMax[3];
static uint32_t s_blockN;
static bool s_blockHave;
static uint32_t s_lastFrameUs = 0;
static uint8_t s_candidateCls = POSTURE_UNKNOWN;
static uint8_t s_candidateCount = 0;

static float norm3(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}

static void resetCalAccumulator(void) {
  for (int i = 0; i < 3; i++) {
    s_calSum[i] = 0.0f;
    s_calMin[i] = 0.0f;
    s_calMax[i] = 0.0f;
  }
  s_calN = 0;
  s_calMaxGyro = 0.0f;
  s_p.calN = 0;
}

static void resetBlock(void) {
  s_blockN = 0;
  s_blockHave = false;
  s_blockStartUs = 0;
  for (int i = 0; i < 3; i++) {
    s_blockSum[i] = 0.0f;
    s_blockMin[i] = 0.0f;
    s_blockMax[i] = 0.0f;
  }
}

static void finishCalibration(bool ok, const char* why) {
  s_calPhase = CAL_IDLE;
  s_p.calActive = false;
  s_p.calFailed = !ok;
  if (ok) {
    s_p.startX = s_calSum[0] / (float)s_calN;
    s_p.startY = s_calSum[1] / (float)s_calN;
    s_p.startZ = s_calSum[2] / (float)s_calN;
    s_p.calValid = true;
    s_p.calN = s_calN;
    s_p.stableCls = POSTURE_UNKNOWN;
    s_p.stable = false;
    s_p.stableCount = 0;
    s_candidateCls = POSTURE_UNKNOWN;
    s_candidateCount = 0;
    outPrintf("[POST] START 基线有效：%.1f/%.1f/%.1f mg，样本 %lu\n",
              (double)s_p.startX, (double)s_p.startY, (double)s_p.startZ,
              (unsigned long)s_calN);
  } else {
    s_p.calN = s_calN;
    outPrintf("[POST] START 基线失败：%s\n", why);
  }
  resetBlock();
}

}  // namespace

void postureInit(void) {
  s_p = PostureResult{};
  s_calPhase = CAL_IDLE;
  s_calStartUs = 0;
  s_calAvgStartUs = 0;
  s_calN = 0;
  s_calMaxGyro = 0.0f;
  s_lastFrameUs = 0;
  resetCalAccumulator();
  resetBlock();
  s_candidateCls = POSTURE_UNKNOWN;
  s_candidateCount = 0;
}

void postureResetRuntime(void) {
  resetBlock();
  s_p.valid = false;
  s_p.cls = POSTURE_UNKNOWN;
  s_p.stableCls = POSTURE_UNKNOWN;
  s_p.stable = false;
  s_p.stableCount = 0;
  s_p.distMg = -1.0f;
  s_candidateCls = POSTURE_UNKNOWN;
  s_candidateCount = 0;
}

void postureInvalidateCal(void) {
  s_calPhase = CAL_IDLE;
  s_p.calActive = false;
  s_p.calValid = false;
  s_p.calFailed = false;
  s_p.calN = 0;
  s_p.startX = s_p.startY = s_p.startZ = 0.0f;
  s_p.stableCls = POSTURE_UNKNOWN;
  s_p.stable = false;
  s_p.stableCount = 0;
  s_candidateCls = POSTURE_UNKNOWN;
  s_candidateCount = 0;
  resetCalAccumulator();
  resetBlock();
}

bool postureStartCal(void) {
  if (s_calPhase != CAL_IDLE) return false;
  s_calPhase = CAL_SETTLE;
  s_calStartUs = 0;
  s_calAvgStartUs = 0;
  s_p.calActive = true;
  s_p.calFailed = false;
  resetCalAccumulator();
  resetBlock();
  return true;
}

bool postureCalCapturing(void) {
  return s_calPhase != CAL_IDLE;
}

void postureOnGap(const SamplerGap& gap) {
  (void)gap;
  resetBlock();
  s_p.valid = false;
  s_p.cls = POSTURE_UNKNOWN;
  s_p.stable = false;
  s_p.stableCount = 0;
  s_p.distMg = -1.0f;
  s_candidateCls = POSTURE_UNKNOWN;
  s_candidateCount = 0;
  if (s_calPhase != CAL_IDLE) {
    finishCalibration(false, "采样中断");
  }
}

void postureOnFrame(const DspFrame& f, uint32_t tUs) {
  s_lastFrameUs = tUs;
  const float gyroW = f.gyroValid
      ? sqrtf(
            f.gyroDps[0] * f.gyroDps[0] +
            f.gyroDps[1] * f.gyroDps[1] +
            f.gyroDps[2] * f.gyroDps[2])
      : 0.0f;

  if (s_calPhase == CAL_SETTLE) {
    if (s_calStartUs == 0) s_calStartUs = tUs;
    if (tUs - s_calStartUs >= (uint32_t)POSTURE_CAL_SETTLE_MS * 1000u) {
      s_calPhase = CAL_AVG;
      s_calAvgStartUs = tUs;
      resetCalAccumulator();
    }
    return;
  }

  if (s_calPhase == CAL_AVG) {
    for (int i = 0; i < 3; i++) {
      const float v = f.ctAxis[i];
      if (s_calN == 0) {
        s_calMin[i] = s_calMax[i] = v;
      } else {
        if (v < s_calMin[i]) s_calMin[i] = v;
        if (v > s_calMax[i]) s_calMax[i] = v;
      }
      s_calSum[i] += v;
    }
    s_calN++;
    s_p.calN = s_calN;
    if (gyroW > s_calMaxGyro) s_calMaxGyro = gyroW;
    if (tUs - s_calAvgStartUs >= (uint32_t)POSTURE_CAL_AVG_MS * 1000u) {
      float maxRange = 0.0f;
      for (int i = 0; i < 3; i++) {
        const float r = s_calMax[i] - s_calMin[i];
        if (r > maxRange) maxRange = r;
      }
      if (s_calN < 100) {
        finishCalibration(false, "样本不足");
      } else if (maxRange > POSTURE_CAL_RANGE_MG) {
        finishCalibration(false, "标定期间在动");
      } else if (s_calMaxGyro > POSTURE_CAL_STILL_DPS) {
        finishCalibration(false, "标定期间角速度过大");
      } else {
        finishCalibration(true, "");
      }
    }
    return;
  }

  // 运行时 500 ms 分块均值；不用大数组，避免占用 RAM。
  if (!s_blockHave || s_blockStartUs == 0) {
    s_blockStartUs = tUs;
    s_blockHave = true;
    s_blockN = 0;
    for (int i = 0; i < 3; i++) {
      s_blockSum[i] = 0.0f;
      s_blockMin[i] = f.ctAxis[i];
      s_blockMax[i] = f.ctAxis[i];
    }
  }
  for (int i = 0; i < 3; i++) {
    const float v = f.ctAxis[i];
    s_blockSum[i] += v;
    if (v < s_blockMin[i]) s_blockMin[i] = v;
    if (v > s_blockMax[i]) s_blockMax[i] = v;
  }
  s_blockN++;
  if (tUs - s_blockStartUs < (uint32_t)POSTURE_WINDOW_MS * 1000u) return;

  float maxRange = 0.0f;
  for (int i = 0; i < 3; i++) {
    const float r = s_blockMax[i] - s_blockMin[i];
    if (r > maxRange) maxRange = r;
  }
  s_p.x = s_blockSum[0] / (float)s_blockN;
  s_p.y = s_blockSum[1] / (float)s_blockN;
  s_p.z = s_blockSum[2] / (float)s_blockN;
  s_p.blockN = s_blockN;
  s_p.valid = s_p.calValid && s_blockN >= 100 &&
              maxRange <= POSTURE_STATIC_RANGE_MG &&
              gyroW <= POSTURE_STATIC_GYRO_DPS;
  if (s_p.valid) {
    s_p.distMg = norm3(s_p.x - s_p.startX, s_p.y - s_p.startY, s_p.z - s_p.startZ);
    if (s_p.distMg <= POSTURE_START_RADIUS_MG) {
      s_p.cls = POSTURE_START;
    } else if (s_p.distMg >= POSTURE_HONGEST_DIST_MG) {
      s_p.cls = POSTURE_HONGEST;
    } else {
      s_p.cls = POSTURE_UNKNOWN;
    }

    // 连续多个窗口同类别才提交 stableCls；过渡段的 raw cls 仍保留。
    if (s_p.cls == POSTURE_UNKNOWN) {
      // UNKNOWN 只表示过渡/数据不足，不提交为稳定类别。
      s_p.stable = false;
      s_p.stableCount = 0;
      s_candidateCls = POSTURE_UNKNOWN;
      s_candidateCount = 0;
    } else if (s_p.cls == s_p.stableCls) {
      s_p.stable = true;
      s_p.stableCount = POSTURE_STABLE_WINDOWS;
      s_candidateCls = s_p.cls;
      s_candidateCount = 0;
    } else {
      if (s_candidateCls == s_p.cls) {
        if (s_candidateCount < 255) s_candidateCount++;
      } else {
        s_candidateCls = s_p.cls;
        s_candidateCount = 1;
      }
      s_p.stable = false;
      s_p.stableCount = s_candidateCount;
      if (s_candidateCount >= POSTURE_STABLE_WINDOWS) {
        s_p.stableCls = s_p.cls;
        s_p.stable = true;
        s_p.stableCount = s_candidateCount;
        s_candidateCount = 0;
      }
    }
  } else {
    s_p.distMg = -1.0f;
    s_p.cls = POSTURE_UNKNOWN;
    s_p.stable = false;
    s_p.stableCount = 0;
    s_candidateCls = POSTURE_UNKNOWN;
    s_candidateCount = 0;
  }
  resetBlock();
}

void postureTick(uint32_t nowUs) {
  if (s_calPhase == CAL_SETTLE && s_calStartUs != 0 &&
      nowUs - s_calStartUs > (uint32_t)(POSTURE_CAL_SETTLE_MS + POSTURE_CAL_AVG_MS + 1000u) * 1000u) {
    finishCalibration(false, "标定超时");
  } else if (s_calPhase == CAL_AVG &&
             nowUs - s_calAvgStartUs > (uint32_t)(POSTURE_CAL_AVG_MS + 1000u) * 1000u) {
    finishCalibration(false, "标定超时");
  }
  if (s_lastFrameUs != 0 && nowUs - s_lastFrameUs > POSTURE_STALE_MS * 1000u) {
    s_p.valid = false;
    s_p.cls = POSTURE_UNKNOWN;
    s_p.stable = false;
    s_p.stableCount = 0;
    s_candidateCls = POSTURE_UNKNOWN;
    s_candidateCount = 0;
  }
}

const PostureResult& postureResult(void) {
  return s_p;
}

const char* postureClassName(uint8_t cls) {
  switch (cls) {
    case POSTURE_START: return "START";
    case POSTURE_HONGEST: return "HONGEST";
    default: return "UNKNOWN";
  }
}
