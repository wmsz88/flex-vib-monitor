// 接触判据测试（Q4 / T10a~h）：链接真实 dsp.cpp + state.cpp。
//
// 编译（三种模式都必须通过）：
//   g++ -std=c++17 -Wall -I test -I . test/ct_test.cpp dsp.cpp state.cpp -o test/ct_test.exe
//   g++ -std=c++17 -Wall -DCT_MODE=0 -I test -I . test/ct_test.cpp dsp.cpp state.cpp -o test/ct_test_0.exe
//   g++ -std=c++17 -Wall -DCT_MODE=1 -I test -I . test/ct_test.cpp dsp.cpp state.cpp -o test/ct_test_1.exe
//
// 默认 CT_MODE=2（到接触姿态的距离），这是实测可用的判据；
// 0/1 是已证明会误报的旧判据，保留做对比与回归。
//
// 设计要点：
//   * T10a/b/c 验证阈值与滞回逻辑（判据"能否工作"的下限）
//   * T10d/e 用同一份源码测不同 CT_MODE：d 断言模式 2 不被振动误触发，
//            e 断言模式 1 会被触发（记录旧行为，证明改默认值的必要性）
//   * T10g 是关键断言：模式 2 **未记录参考点**时 contactValid 必须为 false
//           （区分"未接触"与"不知道"）；记录之后才允许为 true。
//   * T10i/j 是模式 2 专有：参考点记录/清除、以及距离判据的滞回。
//   * T10h 验证 ctAxis[3] 的方向信息保留正确。
//
// 注意：本测试**不链接 output.cpp**（它依赖 Serial/Serial1）。
//       CSV 输出格式与 't' 命令由实板验证，主机侧不覆盖。
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Arduino.h"
#include "dsp.h"
#include "state.h"
#include "config.h"

/* ================= 虚拟时间 ================= */
static uint32_t g_us = 0;
uint32_t millis(void) { return g_us / 1000u; }
uint32_t micros(void) { return g_us; }
void delay(uint32_t ms) { g_us += ms * 1000u; }
void delayMicroseconds(uint32_t us) { g_us += us; }

/* ================= 量程 ================= */
static float g_lsbPerG = 16384.0f;
float mpuLsbPerG(void) { return g_lsbPerG; }
static inline int16_t mg2raw(float mg) { return (int16_t)lrintf(mg * g_lsbPerG / 1000.0f); }

/* ================= MPU / 采样器桩 ================= */
bool mpuReadAccel(MpuRaw*) { return false; }
uint32_t mpuErrCount(void) { return 0; }
void mpuClearErr(void) {}
void samplerAddClip(void) {}
SamplerStartResult samplerStart(void) { return SMP_START_OK; }
SamplerStopResult  samplerStop(void)  { return SMP_STOP_OK; }
bool samplerReset(void) { return true; }
uint32_t samplerInFlight(void) { return 0; }
bool samplerStartIsError(SamplerStartResult r) {
  return r == SMP_START_ERR_CREATE || r == SMP_START_ERR_START;
}
bool samplerStopIsError(SamplerStopResult r) { return r == SMP_STOP_TIMEOUT; }
const char* samplerStartStr(SamplerStartResult) { return ""; }
const char* samplerStopStr(SamplerStopResult)   { return ""; }
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

/* ================= 断言 ================= */
static int g_fail = 0, g_total = 0;
#define CHECK(cond, ...) do {                                   \
  g_total++;                                                    \
  if (!(cond)) { g_fail++; printf("[FAIL] "); }                 \
  else          { printf("[PASS] "); }                          \
  printf(__VA_ARGS__); printf("\n");                            \
} while (0)
#define NOTE(...) do { printf("       "); printf(__VA_ARGS__); printf("\n"); } while (0)

/* ================= 初始化 ================= */

/* 用一份明确的标定参数，让阈值为已知值：
 *   ctOn  = max(CT_MIN_MG=30, CT_K=4 × ctRest)
 *   ctOff = ctOn × CT_OFF_RATIO=0.6
 * 取 ctRest=20 → ctOn=80, ctOff=48。这样测试可以硬编码这两个数，
 * 不依赖 CT_K/CT_MIN_MG 的当前取值。 */
static const float kCtRest  = 20.0f;
static float g_ctOn  = 0.0f;
static float g_ctOff = 0.0f;

static void initCal(void) {
  DspCal& cal = const_cast<DspCal&>(dspCal());
  cal.bias[0] = cal.bias[1] = 0.0f;
  cal.bias[2] = 1000.0f;
  cal.envRest = 2.0f;
  cal.ctRest  = kCtRest;
  cal.zcl     = 8.0f;
  cal.ok = cal.quality = true;
  stateApplyCal(cal);
  g_ctOn  = stateThresholds().ctOn;
  g_ctOff = stateThresholds().ctOff;
}

static void resetAll(void) {
  dspInit();
  stateReset();
  stateSetSensorOk(true);
  g_us = 0;
  samplerClearStats();
  initCal();
  stateReset();
  stateSetSensorOk(true);
  // stateReset 后阈值不变（它只清结果），重新读一次保证一致
  g_ctOn  = stateThresholds().ctOn;
  g_ctOff = stateThresholds().ctOff;
}

/* 喂样本：直接给定姿态偏差（通过重力方向）与振动幅值。
 *
 * 姿态偏差通过"让重力方向倾斜"来产生：把重力分量从 Z 轴挪一部分到 X 轴。
 *   devMg=0   → 纯 Z 轴 1000mg
 *   devMg=D   → X 轴分量使 sqrt(x²+z²)=1000，模长不变但方向偏了
 * 这正是加速度计能观测到的"姿态变化"，也是 ctDev 的物理含义。
 */
static uint32_t g_seq = 0;

static void step(float devMg, float vibAmpMg, float freqHz = 140.0f) {
  g_us += 1000000u / FS_HZ;
  const float t = g_us / 1e6f;

  /* 姿态：重力从 Z 轴向外偏，偏多少由 devMg 决定。
   * 用 asin 保证合成的重力模长恒为 1000mg（倾斜不产生额外"重量"）。 */
  float tilt = asinf(fminf(devMg / 1000.0f, 1.0f));
  float gx = 1000.0f * sinf(tilt);
  float gz = 1000.0f * cosf(tilt);

  /* 振动：叠加在 X 轴上（高通支路会滤掉姿态的直流分量） */
  float vib = (vibAmpMg > 0.0f) ? vibAmpMg * sinf(2 * PI * freqHz * t) : 0.0f;

  Sample s;
  s.seq = g_seq++;
  s.tUs = g_us;
  s.raw.x = mg2raw(gx + vib);
  s.raw.y = mg2raw(0.0f);
  s.raw.z = mg2raw(gz);

  DspFrame f;
  dspProcess(s, &f);
  stateOnFrame(f, s.tUs);
  stateTick(g_us);
}

/* 保持当前姿态与振动 ms 毫秒 */
static void hold(uint32_t ms, float devMg, float vibAmpMg = 0.0f, float freqHz = 140.0f) {
  const uint32_t n = ms * FS_HZ / 1000;
  for (uint32_t i = 0; i < n; i++) step(devMg, vibAmpMg, freqHz);
}

/* 从 devA 线性爬升到 devB，历时 ms 毫秒 */
static void ramp(uint32_t ms, float devA, float devB, float vibAmpMg = 0.0f) {
  const uint32_t n = ms * FS_HZ / 1000;
  for (uint32_t i = 0; i < n; i++) {
    float k = (n > 1) ? (float)i / (float)(n - 1) : 1.0f;
    step(devA + (devB - devA) * k, vibAmpMg);
  }
}

/* ================= 测试 ================= */

/* ================= 判据方向适配 =================
 *
 * 三种模式的"接触"物理含义**相反**，测试不能写死一个方向：
 *
 *   模式 0/1：接触 = 姿态偏差 > 阈值        （远离标定姿态 = 接触）
 *   模式 2  ：接触 = 到参考点距离 < 阈值    （靠近接触姿态 = 接触）
 *
 * 所以用两个辅助函数统一表达"接近接触状态"与"回到未接触状态"，
 * 让 T10a/b/c/f 在各模式下都能表达同一件事。
 */

/* 【模式2】驱动一次完整的参考点采集。
 * 固件里这是三阶段状态机（SETTLE → AVG → 结算），由主循环每圈调用
 * stateTickTouchRef 推进。测试里用同一套时序模拟：
 *   step() 推进采样时间（并保证 stateOnFrame 被调用以累加样本），
 *   每次推进后调 stateTickTouchRef，直到采集结束。
 * 这样测的是**真实采集路径**，而不是绕过它直接塞一个值。 */
static void captureTouchRef(void) {
#if CT_MODE == 2
  if (!stateStartTouchRef()) return;
  /* 采集总时长 = SETTLE + AVG，多给 20% 余量确保结束 */
  const uint32_t total = (uint32_t)((CT_REF_SETTLE_MS + CT_REF_AVG_MS) * 1.2);
  for (uint32_t i = 0; i < total; i++) {
    hold(1, 0.0f);                     // 1ms 采样，姿态不变
    stateTickTouchRef(g_us);
    if (!stateTouchRefCapturing()) break;
  }
#else
  (void)0;
#endif
}

#if CT_MODE == 2
  /* 模式 2：参考点记为"静置姿态"，则"接触"= 回到静置姿态 */
  static float nearTouch()  { return 0.0f; }        // 距参考点 0 → 接触
  static float awayTouch()  { return 2000.0f; }     // 远离 → 未接触
  static void  armRef()     { hold(1200, 0.0f); captureTouchRef(); }
#else
  /* 模式 0/1：无参考点概念。这里显式引用一次 captureTouchRef 的地址，
   * 避免它成为 unused-function 而触发 -Wunused-function 警告
   * （编译要求是"零警告"，所以不能靠 __attribute__ 压掉）。 */
  static float nearTouch()  { return g_ctOn * 2.0f; }  // 远超阈值 → 接触
  static float awayTouch()  { return 0.0f; }           // 静置 → 未接触
  static void  armRef()     { void (*volatile keep)(void) = captureTouchRef; (void)keep; }
#endif

int main(void) {
  printf("=== 接触判据测试（CT_MODE=%d）===\n\n", (int)CT_MODE);

  /* ---------------- T10a：阶跃，确认后翻转 ---------------- */
  {
    resetAll();
    NOTE("ctOn=%.1f ctOff=%.1f mg（由 ctRest=%.1f 推出）", g_ctOn, g_ctOff, kCtRest);
    armRef();
    hold(600, awayTouch());                 // 未接触
    CHECK(stateResult().contact == 0, "T10a 未接触时 contact=0");

    hold(600, nearTouch());                 // 阶跃到接触态
    /* 确认时间 CT_CONFIRM_MS=100ms，已给 600ms，必然翻转过 */
    CHECK(stateResult().contact == 1, "T10a 阶跃到接触态后 contact=1");
  }

  /* ---------------- T10b：缓慢过渡（不依赖变化率）---------------- */
  {
    resetAll();
    armRef();
    hold(600, awayTouch());
    CHECK(stateResult().contact == 0, "T10b 过渡前 contact=0");
    ramp(2000, awayTouch(), nearTouch());   // 2 秒缓慢过渡
    CHECK(stateResult().contact == 1, "T10b 缓慢过渡到接触态后 contact=1（判据不依赖变化率）");
  }

  /* ---------------- T10c：滞回 ----------------
   * 模式 0/1：在 ctOn/ctOff 之间应保持接触
   * 模式 2  ：在 CT_TOUCH_MG/CT_TOUCH_OFF_MG 之间应保持接触
   * 两者都是"进入阈值"与"退出阈值"之间的带，语义一致。 */
  {
    resetAll();
    armRef();
    hold(600, nearTouch());
    CHECK(stateResult().contact == 1, "T10c 进入接触态后 contact=1");

#if CT_MODE == 2
    const float mid   = (CT_TOUCH_MG + CT_TOUCH_OFF_MG) * 0.5f;
    const float beyond = CT_TOUCH_OFF_MG * 1.5f;
#else
    const float mid   = (g_ctOn + g_ctOff) * 0.5f;
    const float beyond = g_ctOff * 0.5f;
#endif
    /* 落在滞回带内：应保持 1（不释放） */
    hold(700, mid);
    CHECK(stateResult().contact == 1, "T10c 滞回带内(%.1f)仍 contact=1", mid);

    /* 超出退出阈值：应释放 */
    hold(700, beyond);
    CHECK(stateResult().contact == 0, "T10c 超出退出阈值(%.1f)后 contact=0", beyond);
  }

  /* ---------------- T10d/e：仅振动不得触发接触（模式相关） ---------------- */
#if CT_MODE == 1
  {
    resetAll();
    hold(300, 0.0f);
    hold(1500, 0.0f, 500.0f, 140.0f);
    CHECK(stateResult().vib == 1, "T10e 振动已被检出（vib=1）");
    CHECK(stateResult().contact == 1, "T10e CT_MODE=1：振动被当成接触（旧行为，会误报）");
  }
#else
  {
    resetAll();
    armRef();
    hold(600, awayTouch());
    CHECK(stateResult().contact == 0, "T10d 振动前 contact=0");
    /* 强振动但姿态不变：不得因此判为接触 */
    hold(1500, awayTouch(), 500.0f, 140.0f);
    CHECK(stateResult().vib == 1, "T10d 振动已被检出（vib=1，测试前提成立）");
    CHECK(stateResult().contact == 0, "T10d 仅振动不触发接触");
  }
#endif

  /* ---------------- T10f：缺口打断接触确认链 ---------------- */
  {
    resetAll();
    armRef();
    hold(600, awayTouch());
    CHECK(stateResult().contact == 0, "T10f 缺口前 contact=0");

    /* 过渡到接触态，但只持续不到确认时间，中途插入缺口 */
    hold(50, nearTouch());
    SamplerGap gap;
    gap.kind = SMP_GAP_MISSING;
    gap.missed = 1;
    gap.spanUs = 2000;
    dspOnGap(gap);
    stateOnGap(gap);
    /* 缺口打断了"连续确认"，但之后又持续了 800ms > 100ms，
     * 所以最终仍应确认接触。这里验证的是"中断后能重新累计"，
     * 而不是"永远确认不了"。 */
    hold(800, nearTouch());
    CHECK(stateResult().contact == 1, "T10f 缺口打断后重新累计确认，contact=1");
  }

  /* ---------------- T10g：不得假装可信（关键断言） ---------------- */
#if CT_MODE == 2
  {
    resetAll();
    CHECK(stateResult().contactValid == false, "T10g 初始 contactValid=false");
    CHECK(stateResult().ctRefSet == false, "T10g 初始 ctRefSet=false");
    CHECK(stateResult().ctTouchDist < 0.0f, "T10g 无参考点时 ctTouchDist=-1（表示不知道）");

    /* 未记录参考点：无论姿态怎么变，都必须 contact=0 且不可信 */
    hold(1200, 0.0f);
    hold(500, 2000.0f);
    CHECK(stateResult().contact == 0, "T10g ★ 未记录参考点时 contact 恒为 0");
    CHECK(stateResult().contactValid == false, "T10g ★ 未记录参考点时 contactValid=false");

    /* 记录参考点后才可信 */
    hold(1200, 2000.0f);                // 停在"接触"姿态
    captureTouchRef();
    CHECK(stateResult().ctRefSet == true, "T10g 记录后 ctRefSet=true");
    CHECK(stateResult().contactValid == true, "T10g ★ 记录后 contactValid=true（判据已实测验证）");

    /* 重新标定必须清掉参考点：旧参考点是在旧零偏下记录的三轴值，
     * 换了零偏就不在同一坐标系，继续用会算错距离。 */
    initCal();
    CHECK(stateResult().ctRefSet == false, "T10g ★ 重新标定后参考点被清除");
    CHECK(stateResult().contactValid == false, "T10g ★ 重新标定后 contactValid 回到 false");

    /* 缺口不应把参考点弄丢 */
    captureTouchRef();
    SamplerGap gap;
    gap.kind = SMP_GAP_MISSING; gap.missed = 1; gap.spanUs = 2000;
    dspOnGap(gap); stateOnGap(gap);
    CHECK(stateResult().ctRefSet == true, "T10g 缺口后参考点仍保留");
    CHECK(stateResult().contactValid == true, "T10g 缺口后 contactValid 仍为 true");
  }

  /* ---------------- T10i：参考点记录与清除 ---------------- */
  {
    resetAll();
    hold(1200, 0.0f);
    captureTouchRef();
    CHECK(stateTouchRefSet() == true, "T10i 记录后 stateTouchRefSet()=true");
    hold(300, 0.0f);
    CHECK(stateResult().ctTouchDist < 50.0f, "T10i 停在参考点姿态时距离 <50mg");

    stateClearTouchRef();
    CHECK(stateTouchRefSet() == false, "T10i 清除后 stateTouchRefSet()=false");
    CHECK(stateResult().contactValid == false, "T10i 清除后 contactValid=false");
    CHECK(stateResult().contact == 0, "T10i 清除后 contact=0");
  }

  /* ---------------- T10j：距离判据的滞回 ---------------- */
  {
    resetAll();
    hold(1200, 0.0f);
    captureTouchRef();                 // 参考点 = 静置姿态
    hold(300, 0.0f);
    CHECK(stateResult().contact == 1, "T10j 记录参考点后在参考点附近 → contact=1");

    /* 远离参考点 → 释放 */
    hold(600, 2000.0f);
    CHECK(stateResult().contact == 0, "T10j 远离参考点（2000mg）时 contact=0");

    /* 回到参考点附近 → 重新判定接触 */
    hold(600, 0.0f);
    CHECK(stateResult().contact == 1, "T10j ★ 回到参考点附近时 contact=1（这就是\"接触\"）");

    /* 落在滞回带内（CT_TOUCH_MG~CT_TOUCH_OFF_MG）：应**保持**接触 */
    hold(800, (CT_TOUCH_MG + CT_TOUCH_OFF_MG) * 0.5f);
    CHECK(stateResult().contact == 1, "T10j 滞回带内保持 contact=1");

    /* 超出退出阈值 → 释放 */
    hold(800, CT_TOUCH_OFF_MG * 1.5f);
    CHECK(stateResult().contact == 0, "T10j 超出退出阈值后 contact=0");
  }
#else
  {
    resetAll();
    CHECK(stateResult().contactValid == false, "T10g 初始 contactValid=false");

    hold(500, g_ctOn * 2.0f);
    CHECK(stateResult().contact == 1, "T10g 已确认接触（contact=1）");
    CHECK(stateResult().contactValid == false, "T10g ★ 模式 0/1 已证明会误报，contactValid 恒为 false");

    initCal();
    CHECK(stateResult().contactValid == false, "T10g ★ 标定后 contactValid 仍为 false");

    SamplerGap gap;
    gap.kind = SMP_GAP_MISSING; gap.missed = 1; gap.spanUs = 2000;
    dspOnGap(gap); stateOnGap(gap);
    CHECK(stateResult().contactValid == false, "T10g ★ 缺口后 contactValid 仍为 false");
  }
#endif

  /* ---------------- T10h：ctAxis 保留方向信息 ---------------- */
  {
    resetAll();

    /* 【语义】ctAxis 是**相对标定零偏后的偏差量**，静置时接近 (0,0,0)。
     * 不是含重力的绝对值 —— 因为 dsp.cpp 里先减 bias 再低通，而 bias 本身就是
     * 标定姿态下的重力向量。证据：静置 ctRest 实测约 0.1mg；若含重力会是 1000mg，
     * 阈值就会是 4000mg 而不是实测的 30~56mg。 */
    hold(1200, 0.0f);                       // 低通收敛（tau=0.2s，给足时间）
    {
      DspFrame f;
      Sample s;
      s.seq = g_seq++; s.tUs = g_us;
      s.raw.x = mg2raw(0.0f);
      s.raw.y = mg2raw(0.0f);
      s.raw.z = mg2raw(1000.0f);            // 与 bias[2]=1000 相同 → 偏差应为 0
      dspProcess(s, &f);
      NOTE("静置 ctAxis=(%.2f, %.2f, %.2f) ctDev=%.2f", f.ctAxis[0], f.ctAxis[1], f.ctAxis[2], f.ctDev);
      CHECK(fabsf(f.ctAxis[0]) < 5.0f && fabsf(f.ctAxis[1]) < 5.0f && fabsf(f.ctAxis[2]) < 5.0f,
            "T10h 静置时三轴都接近 0（ctAxis 是相对零偏的偏差量）");
      CHECK(f.ctDev < 5.0f, "T10h 静置时 ctDev 接近 0");
    }

    /* 倾斜 200mg：X 轴应出现约 200mg 分量。
     * 这是方向信息的核心价值：ctDev 只说"偏了 200mg"，
     * ctAxis 能说出"是往 X 方向偏的"。 */
    {
      DspFrame f;
      Sample s;
      float tilt = asinf(200.0f / 1000.0f);
      s.seq = g_seq++; s.tUs = g_us;
      s.raw.x = mg2raw(1000.0f * sinf(tilt));
      s.raw.y = mg2raw(0.0f);
      s.raw.z = mg2raw(1000.0f * cosf(tilt));
      dspProcess(s, &f);
      NOTE("单次倾斜后 ctAxis=(%.2f, %.2f, %.2f)（低通未收敛，仅验证符号与方向）",
           f.ctAxis[0], f.ctAxis[1], f.ctAxis[2]);
      CHECK(f.ctAxis[0] > 0.0f,
            "T10h 往 +X 倾斜时 X 轴分量为正（方向符号正确）");
      CHECK(fabsf(f.ctAxis[1]) < 0.01f,
            "T10h Y 轴未倾斜，分量保持 0（方向未被混淆）");
    }

    /* 让低通收敛到倾斜姿态，验证模长关系 */
    hold(1200, 200.0f);
    {
      DspFrame f;
      Sample s;
      float tilt = asinf(200.0f / 1000.0f);
      s.seq = g_seq++; s.tUs = g_us;
      s.raw.x = mg2raw(1000.0f * sinf(tilt));
      s.raw.y = mg2raw(0.0f);
      s.raw.z = mg2raw(1000.0f * cosf(tilt));
      dspProcess(s, &f);
      float norm = sqrtf(f.ctAxis[0]*f.ctAxis[0] + f.ctAxis[1]*f.ctAxis[1] + f.ctAxis[2]*f.ctAxis[2]);
      NOTE("倾斜收敛后 ctAxis=(%.1f, %.1f, %.1f) ctDev=%.1f 模长=%.1f",
           f.ctAxis[0], f.ctAxis[1], f.ctAxis[2], f.ctDev, norm);
      CHECK(f.ctAxis[0] > 100.0f,
            "T10h 倾斜收敛后 X 轴有明显分量");
      CHECK(fabsf(f.ctDev - norm) < 1.0f,
            "T10h ctDev 等于三轴分量的模长（与定义一致）");
    }
  }

  /* ---------------- 汇总 ---------------- */
  printf("\n");
  if (g_fail == 0) printf("全部通过（%d/%d 项通过，0 项失败）\n", g_total, g_total);
  else             printf("%d/%d 项失败\n", g_fail, g_total);
  return g_fail == 0 ? 0 : 1;
}
