// v5 角度积分主机测试（G1/G3/G5/G6/G7）
//
// 编译：
//   g++ -std=c++17 -Wall -I test -I .. test/gyro_test.cpp ../dsp.cpp ../state.cpp -o test/gyro_test.exe
//
// 只测纯算法：喂合成加速度（决定 flatDist）与合成角速度（决定积分角度），
// 不涉及 I2C / 定时器 / 串口。实板行为另见 陀螺仪改造方案.md §12/§14/§15。
#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "dsp.h"
#include "state.h"
#include "config.h"

/* ---------------- 虚拟时间 ---------------- */
static uint32_t g_us = 0;
uint32_t millis(void) { return g_us / 1000u; }
uint32_t micros(void) { return g_us; }
void delay(uint32_t ms) { g_us += ms * 1000u; }
void delayMicroseconds(uint32_t us) { g_us += us; }

/* ---------------- MPU / 采样器桩 ---------------- */
static float g_lsbPerG = 8192.0f;      // 与 ACCEL_FS_SEL=1 一致
static float g_lsbPerDps = 65.5f;      // 与 GYRO_FS_SEL=1 一致
float mpuLsbPerG(void) { return g_lsbPerG; }
float mpuLsbPerDps(void) { return g_lsbPerDps; }
bool mpuReadAccel(MpuRaw*) { return false; }
bool mpuReadAll(MpuRaw*) { return false; }
uint32_t mpuErrCount(void) { return 0; }
void mpuClearErr(void) {}
void samplerAddClip(void) {}
SamplerStartResult samplerStart(void) { return SMP_START_OK; }
SamplerStopResult samplerStop(void) { return SMP_STOP_OK; }
bool samplerReset(void) { return true; }
uint32_t samplerInFlight(void) { return 0; }
bool samplerStartIsError(SamplerStartResult) { return false; }
bool samplerStopIsError(SamplerStopResult) { return false; }
const char* samplerStartStr(SamplerStartResult) { return ""; }
const char* samplerStopStr(SamplerStopResult) { return ""; }
static SamplerStats g_stats = {};
const SamplerStats& samplerStats(void) { return g_stats; }
void samplerClearStats(void) { g_stats = SamplerStats(); }
bool samplerSensorLost(void) { return false; }
bool samplerPop(Sample*, SamplerGap*) { return false; }
static TimingStats g_timing = {};
const TimingStats& samplerTiming(void) { return g_timing; }
void samplerTimingReset(void) { g_timing = TimingStats(); }
void samplerNoteLoopUs(uint32_t) {}
void samplerNoteOutUs(uint32_t) {}

/* ---------------- 断言 ---------------- */
static int g_fail = 0, g_total = 0;
#define CHECK(cond, ...) do {                                   \
  g_total++;                                                    \
  if (!(cond)) { g_fail++; printf("[FAIL] "); }                  \
  else          { printf("[PASS] "); }                           \
  printf(__VA_ARGS__); printf("\n");                             \
} while (0)

static inline int16_t mg2raw(float mg) { return (int16_t)lrintf(mg * g_lsbPerG / 1000.0f); }

/* ---------------- 初始化 ---------------- */
static void initAll(void) {
  DspCal& cal = const_cast<DspCal&>(dspCal());
  cal.bias[0] = cal.bias[1] = 0.0f;
  cal.bias[2] = 1000.0f;
  cal.gyroBias[0] = cal.gyroBias[1] = cal.gyroBias[2] = 0.0f;
  cal.envRest = 2.0f;
  cal.ctRest = 20.0f;
  cal.zcl = 8.0f;
  cal.ok = cal.quality = true;
  stateApplyCal(cal);
}

static void resetAll(void) {
  dspInit();
  stateReset();
  stateSetSensorOk(true);
  g_us = 0;
  samplerClearStats();
  initAll();
  stateReset();
  stateSetSensorOk(true);
}

/* ---------------- 喂样本 ---------------- */
// devMg: 姿态偏离（决定 flatDist）；dps: 三轴角速度（作用在 X）；gyroOk: 是否带陀螺
static void feed(float devMg, float dps, uint32_t n, bool gyroOk = true) {
  for (uint32_t i = 0; i < n; i++) {
    g_us += 1000000u / FS_HZ;
    const float tilt = asinf(fminf(fmaxf(devMg, 0.0f) / 1000.0f, 1.0f));
    Sample s;
    s.seq = i;
    s.tUs = g_us;
    s.raw.x = mg2raw(1000.0f * sinf(tilt));
    s.raw.y = mg2raw(0.0f);
    s.raw.z = mg2raw(1000.0f * cosf(tilt));
    s.gyroValid = gyroOk && ((i % (uint32_t)GYRO_DIV) == 0);
    s.raw.gx = s.gyroValid ? (int16_t)lrintf(dps * g_lsbPerDps) : 0;
    s.raw.gy = s.raw.gz = 0;
    DspFrame f;
    dspProcess(s, &f);
    stateOnFrame(f, s.tUs);
  }
}

static void captureFlatRef(void) {
  stateStartFlatRef();
  for (int i = 0; i < 2500; i++) {
    feed(0.0f, 0.0f, 1);
    stateTickFlatRef(g_us);
  }
}

int main(void) {
  printf("=== v5 gyro_test (CT_MODE=%d GYRO_DIV=%d FS=%d DT=%.4f) ===\n\n",
         CT_MODE, GYRO_DIV, FS_HZ, (double)GYRO_DT_S);

  /* G1：100 dps × 1s ⇒ 100° */
  resetAll();
  captureFlatRef();
  feed(700.0f, 0.0f, 200);            // 先离开张开位（D>OPEN_D_MG）
  feed(700.0f, 100.0f, 1000);         // 1s @100dps
  {
    const float a = stateResult().closeAngleDeg;
    CHECK(fabsf(a - 100.0f) <= 2.0f, "G1 100dps x 1s -> %.1f deg (expect 100±2)", (double)a);
  }

  /* G3：静止 60s、残余零偏 0.5dps ⇒ 角度基本不涨 */
  {
    const float before = stateResult().closeAngleDeg;
    feed(700.0f, 0.5f, 60000);
    const float after = stateResult().closeAngleDeg;
    CHECK(fabsf(after - before) <= 3.0f, "G3 still 60s @0.5dps -> drift %.1f deg (expect <=3)",
          (double)(after - before));
  }

  /* G5：单轴超量程 ⇒ angleValid=false */
  resetAll();
  captureFlatRef();
  feed(700.0f, 0.0f, 200);
  feed(700.0f, GYRO_CLIP_DPS + 50.0f, 200);
  CHECK(stateResult().angleValid == false, "G5 over-range -> angleValid=false");

  /* G6：连续无陀螺超过 GYRO_STALE_MS ⇒ angleValid=false */
  resetAll();
  captureFlatRef();
  feed(700.0f, 0.0f, 200);
  feed(700.0f, 100.0f, 500);          // 先积一点角度
  CHECK(stateResult().angleValid == true, "G6 前置：有陀螺时 angleValid=true");
  feed(700.0f, 0.0f, (uint32_t)(GYRO_STALE_MS * FS_HZ / 1000) + 300, false);
  CHECK(stateResult().angleValid == false, "G6 陀螺中断 -> angleValid=false");

  /* G7：张开→闭合→张开 循环 10 次，每次回到张开后角度归零 */
  {
    int bad = 0;
    for (int k = 0; k < 10; k++) {
      feed(700.0f, 150.0f, 600);      // 闭合 0.6s
      feed(700.0f, 0.0f, 300);        // 停一下（角度保持）
      feed(0.0f, 0.0f, 1500);         // 回张开并静止 1.5s ⇒ parked ⇒ 清零
      if (stateResult().closeAngleDeg > 3.0f) bad++;
    }
    CHECK(bad == 0, "G7 10 次开合循环，回张开后角度未归零的次数 = %d", bad);
  }

  printf("\n%s（%d/%d 项通过）\n", g_fail == 0 ? "全部通过" : "存在失败",
         g_total - g_fail, g_total);
  return g_fail == 0 ? 0 : 1;
}
