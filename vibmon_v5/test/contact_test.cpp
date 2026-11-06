// v5 接触判据主机测试（CT_MODE=4）
//
// 直接用合成 DspFrame 驱动 state.cpp，不经过 dsp.cpp 的真实滤波/测频链路。
// 覆盖平放基准采集、姿态/幅值/频率三道门、滞回、确认延时、
// 数据失效降级和退出规则。
//
// 编译：
//   g++ -std=c++17 -Wall -I test -I . test/contact_test.cpp state.cpp -o test/contact_test.exe

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "state.h"
#include "config.h"
#include "sampler.h"

static uint32_t g_us = 0;
static int g_pass = 0, g_fail = 0;

#define CHECK(cond, ...) do {                                   \
  if (cond) { g_pass++; printf("[PASS] "); }                    \
  else      { g_fail++; printf("[FAIL] "); }                    \
  printf(__VA_ARGS__); printf("\n");                            \
} while (0)

// 合成一帧：amp 直接作为包络/幅值输入，freq 直接作为主频输入，
// gxDps 用来积分闭合角度，axisX 用来制造“离平放姿态的距离”。
static void feed(float axisX, float amp, float freq, float gxDps,
                 bool gyroValid = true, bool clipped = false) {
  DspFrame f = {};
  f.gyroValid = gyroValid;
  f.gyroDps[0] = gxDps;
  f.gyroClipped = false;
  f.envReady = true;
  f.env = amp;
  f.ctDev = fabsf(axisX);
  f.ctAxis[0] = axisX;
  f.ctAxis[1] = 0.0f;
  f.ctAxis[2] = 0.0f;
  f.winReady = true;
  f.freqHz = freq;
  f.axis = 0;
  f.clipLevel = CLIP_NONE;
  f.sampleClipped = clipped;
  f.envClipped = clipped;
  g_us += 1000u;
  stateOnFrame(f, g_us);
}

static void resetScenario(void) {
  stateSetSensorOk(true);
  stateReset();
}

// 稳定采一份平放基准（900 个 1ms 帧，覆盖 300ms 稳定 + 500ms 平均）。
static void captureStableFlatRef(void) {
  stateClearFlatRef();
  stateSetSensorOk(true);
  stateStartFlatRef();
  stateTickFlatRef(g_us);
  for (int i = 0; i < 900; i++) {
    feed(0.0f, 0.0f, 0.0f, 0.0f);
    stateTickFlatRef(g_us);
  }
}

// 采集期间摆动 200mg，应被稳定性检查拒绝。
static void captureMovingFlatRef(void) {
  stateClearFlatRef();
  stateSetSensorOk(true);
  stateStartFlatRef();
  stateTickFlatRef(g_us);
  for (int i = 0; i < 900; i++) {
    const float a = (i % 2) ? 100.0f : -100.0f;
    feed(a, 0.0f, 0.0f, 0.0f);
    stateTickFlatRef(g_us);
  }
}

static void rotateUntilClosed(float dps) {
  int guard = 0;
  while (stateResult().closeAngleDeg < ANGLE_CLOSED_DEG && guard++ < 5000) {
    feed(500.0f, 0.0f, 0.0f, dps);
  }
}

static void hold(float axisX, float amp, float freq, int ms) {
  for (int i = 0; i < ms; i++) feed(axisX, amp, freq, 0.0f);
}

int main(void) {
  printf("=== v5 contact_test (CT_MODE=%d) ===\n\n", CT_MODE);

  /* C1：没有平放基准时不能输出接触结论。
   * FINDING-01 已修：CT_MODE=4 的 postureInput 同时检查 angleValid
   * 和 flatRefSet。 */
  stateClearFlatRef();
  resetScenario();
  feed(500.0f, 1000.0f, 165.0f, 0.0f);
  CHECK(!stateFlatRefSet(),              "C1 未采基准：flatRefSet=0");
  CHECK(!stateResult().contactValid,     "C1 未采基准：contactValid=0（FINDING-01 已修）");
  CHECK(stateResult().contact == 0,      "C1 未采基准：contact=0");

  /* C2：稳定采集平放基准成功 */
  captureStableFlatRef();
  CHECK(stateFlatRefSet(),               "C2 稳定采集：flatRefSet=1");
  CHECK(stateResult().contactValid,      "C2 稳定采集：contactValid=1");

  /* C3：采集期间在动，基准必须被拒绝 */
  captureMovingFlatRef();
  CHECK(!stateFlatRefSet(),              "C3 采集在动：flatRefSet=0");
  CHECK(!stateResult().contactValid,     "C3 采集在动：contactValid=0（FINDING-01 已修）");

  /* 后续用一份稳定基准 */
  captureStableFlatRef();
  resetScenario();

  /* C4：角度没到闭合门，即便 A/F 都过也不能接触 */
  hold(500.0f, 1000.0f, 165.0f, 300);
  CHECK(stateResult().closed == 0,       "C4 角度不足：closed=0");
  CHECK(stateResult().contact == 0,      "C4 角度不足：contact=0");
  CHECK(stateResult().contactValid,      "C4 角度不足：contactValid=1（结论可信，值为 0）");

  /* C5：姿态 + 幅值 + 频率三路全过 */
  resetScenario();
  rotateUntilClosed(100.0f);
  hold(500.0f, 1000.0f, 165.0f, 200);
  CHECK(stateResult().closed == 1,       "C5 三路全过：closed=1");
  CHECK(stateResult().contact == 1,      "C5 三路全过：contact=1");

  /* C6：角度退回 45° 以下，姿态门失效，contact 应退出 */
  {
    int guard = 0;
    while (stateResult().closeAngleDeg >= ANGLE_CLOSED_OFF_DEG && guard++ < 5000) {
      feed(500.0f, 1000.0f, 165.0f, -100.0f);
    }
    hold(500.0f, 1000.0f, 165.0f, 200);
    CHECK(stateResult().closed == 0,     "C6 角度滞回退出：closed=0");
    CHECK(stateResult().contact == 0,    "C6 角度滞回退出：contact=0");
  }

  /* C7：幅值门严格大于 400mg */
  resetScenario();
  rotateUntilClosed(100.0f);
  hold(500.0f, 399.9f, 165.0f, 300);
  CHECK(stateResult().condAmp == 0,      "C7 幅值边界：A=399.9 不成立");
  CHECK(stateResult().contact == 0,      "C7 幅值边界：contact=0");
  hold(500.0f, 400.1f, 165.0f, 200);
  CHECK(stateResult().condAmp == 1,      "C7 幅值边界：A=400.1 成立");
  CHECK(stateResult().contact == 1,      "C7 幅值边界：contact=1");

  /* C8：频率门严格大于 75Hz，命题下限 80Hz 单独回归 */
  resetScenario();
  rotateUntilClosed(100.0f);
  hold(500.0f, 1000.0f, 75.0f, 300);
  CHECK(stateResult().condFreq == 0,     "C8 频率边界：F=75.0 不成立");
  CHECK(stateResult().contact == 0,      "C8 频率边界：contact=0");
  hold(500.0f, 1000.0f, 75.1f, 200);
  CHECK(stateResult().condFreq == 1,     "C8 频率边界：F=75.1 成立");
  CHECK(stateResult().contact == 1,      "C8 频率边界：contact=1");

  resetScenario();
  rotateUntilClosed(100.0f);
  hold(500.0f, 1000.0f, 80.0f, 200);
  CHECK(stateResult().condFreq == 1,     "C8b 80Hz 下限：condFreq=1");
  CHECK(stateResult().contact == 1,      "C8b 80Hz 下限：contact=1");

  /* C9：contact 已成立后，A 掉到 300mg 以下但 F 仍在，应保持 */
  resetScenario();
  rotateUntilClosed(100.0f);
  hold(500.0f, 1000.0f, 165.0f, 200);
  CHECK(stateResult().contact == 1,      "C9 保持规则：先建立 contact=1");
  hold(500.0f, 200.0f, 165.0f, 300);
  CHECK(stateResult().condAmp == 0,      "C9 保持规则：A=200 时幅值门已掉");
  CHECK(stateResult().contact == 1,      "C9 保持规则：F 仍在 -> contact 保持 1");

  /* C10：频率滞回退出；两路都掉后才退出 */
  hold(500.0f, 200.0f, 63.0f, 300);
  CHECK(stateResult().contact == 1,      "C10 频率滞回：F=63.0 仍保持");
  hold(500.0f, 200.0f, 62.9f, 300);
#if CT_HOLD_ENABLE
  CHECK(stateResult().contact == 1,      "C10 宏开：两路掉后进入 CT hold");
#else
  CHECK(stateResult().contact == 0,      "C10 频率滞回：F=62.9 且 A 已掉 -> contact=0");
#endif

  /* C11：数据失效等价于该路不成立；只在两路都失效后退出 */
  resetScenario();
  rotateUntilClosed(100.0f);
  hold(500.0f, 1000.0f, 165.0f, 200);
  CHECK(stateResult().contact == 1,      "C11 失效降级：先建立 contact=1");
  for (int i = 0; i < 200; i++) feed(500.0f, 1000.0f, 165.0f, 0.0f, true, true);
  CHECK(!stateResult().ampValid,         "C11 失效降级：削顶后 ampValid=0");
  CHECK(stateResult().contact == 1,      "C11 失效降级：F 仍在 -> contact 保持 1");
  for (int i = 0; i < 200; i++) feed(500.0f, 1000.0f, 0.0f, 0.0f, true, true);
  CHECK(!stateResult().freqValid,        "C11 失效降级：F 也失效");
#if CT_HOLD_ENABLE
  CHECK(stateResult().contact == 1,      "C11 宏开：两路失效后进入 CT hold");
#else
  CHECK(stateResult().contact == 0,      "C11 失效降级：两路都失效 -> contact=0");
#endif

  /* C12：角度失效时用 flatDist 兜底姿态门 */
  resetScenario();
  rotateUntilClosed(100.0f);
  hold(500.0f, 1000.0f, 165.0f, 200);
  CHECK(stateResult().contact == 1,      "C12 角度失效兜底：先建立 contact=1");
  for (int i = 0; i < GYRO_STALE_MS + 100; i++) {
    feed(500.0f, 1000.0f, 165.0f, 0.0f, false);
  }
  CHECK(!stateResult().angleValid,       "C12 角度失效兜底：angleValid=0");
  CHECK(stateResult().contactValid,      "C12 角度失效兜底：flatDist 高 -> contactValid=1");
  CHECK(stateResult().closed,            "C12 角度失效兜底：flatDist 高 -> closed=1");
  CHECK(stateResult().contact == 1,      "C12 角度失效兜底：flatDist 高 -> contact=1");
  for (int i = 0; i < 300; i++) {
    feed(0.0f, 1000.0f, 165.0f, 0.0f, false);
  }
  CHECK(stateResult().contactValid,      "C12 角度失效兜底：flatDist 低 -> contactValid=1");
  CHECK(stateResult().closed == 0,       "C12 角度失效兜底：flatDist 低 -> closed=0");
  CHECK(stateResult().contact == 0,      "C12 角度失效兜底：flatDist 低 -> contact=0");

  /* C13：空夹诊断：姿态过、幅值过、频率没过 */
  resetScenario();
  rotateUntilClosed(100.0f);
  hold(500.0f, 1000.0f, 70.0f, 300);
  CHECK(stateResult().gripNoObject,      "C13 空夹：gripNoObject=1");
  CHECK(stateResult().contact == 0,      "C13 空夹：contact=0");

  /* C14：确认延时：三路刚同时成立时不得立即翻转，满 100ms 后才翻转 */
  resetScenario();
  {
    int guard = 0;
    while (stateResult().closeAngleDeg < ANGLE_CLOSED_DEG && guard++ < 5000) {
      feed(500.0f, 1000.0f, 165.0f, 100.0f);
    }
    const uint32_t tCross = g_us;
    while ((uint32_t)(g_us - tCross) < 99000u) {
      feed(500.0f, 1000.0f, 165.0f, 0.0f);
    }
    CHECK(stateResult().contact == 0,    "C14 确认延时：<100ms 不翻转");
    while ((uint32_t)(g_us - tCross) < 101000u) {
      feed(500.0f, 1000.0f, 165.0f, 0.0f);
    }
    CHECK(stateResult().contact == 1,    "C14 确认延时：>=100ms 翻转");
  }

#if CT_HOLD_ENABLE
  /* H1：证据消失但姿态仍闭合，CT 保持到 CT_HOLD_MS，再经确认延时退出。 */
  resetScenario();
  rotateUntilClosed(100.0f);
  hold(500.0f, 1000.0f, 165.0f, 200);
  CHECK(stateResult().contact == 1,    "H1 hold 前置：contact=1");
  hold(500.0f, 0.0f, 0.0f, CT_HOLD_MS - 200);
  CHECK(stateResult().contact == 1,    "H1 hold 未到期：contact 保持 1");
  hold(500.0f, 0.0f, 0.0f, CT_CONFIRM_MS + 300);
  CHECK(stateResult().contact == 0,    "H1 hold 到期+确认：contact=0");

  /* H2：保持期内证据刷新，lastEvidence 顺延；超过最后一次证据 + hold 才退出。 */
  resetScenario();
  rotateUntilClosed(100.0f);
  hold(500.0f, 1000.0f, 165.0f, 200);
  hold(500.0f, 0.0f, 0.0f, 600);
  hold(500.0f, 1000.0f, 165.0f, 100);
  CHECK(stateResult().contact == 1,    "H2 证据刷新：contact=1");
  hold(500.0f, 0.0f, 0.0f, 600);
  CHECK(stateResult().contact == 1,    "H2 刷新后未到期：contact 保持 1");
  hold(500.0f, 0.0f, 0.0f, 900);
  CHECK(stateResult().contact == 0,    "H2 最后一次证据过期：contact=0");

  /* H3：保持期内姿态打开，必须立即走原有退出，hold 不得续命。 */
  resetScenario();
  rotateUntilClosed(100.0f);
  hold(500.0f, 1000.0f, 165.0f, 200);
  hold(500.0f, 0.0f, 0.0f, 600);
  CHECK(stateResult().contact == 1,    "H3 松手前置：hold 仍在");
  {
    int guard = 0;
    while (stateResult().closeAngleDeg >= ANGLE_CLOSED_OFF_DEG && guard++ < 5000) {
      feed(0.0f, 0.0f, 0.0f, -100.0f);
    }
  }
  const uint32_t tOpen = g_us;
  while (stateResult().contact != 0 && (uint32_t)(g_us - tOpen) < 300000u) {
    feed(0.0f, 0.0f, 0.0f, 0.0f);
  }
  CHECK(stateResult().closed == 0,     "H3 姿态打开：closed=0");
  CHECK(stateResult().contact == 0,    "H3 姿态打开：contact=0");
  CHECK((uint32_t)(g_us - tOpen) <= 200000u,
        "H3 姿态打开：contact 在 0.2s 内退出");
#endif

#if CT_SS_ENTRY_ENABLE
  /* T011：启停节奏进入。ON 段幅值 300mg（低于 400 进入门）+ 120Hz，模拟 2/3Hz 档建不起幅值。 */
  auto startStop = [](float axisX, int onMs, int offMs, int cycles) {
    for (int c = 0; c < cycles; c++) {
      hold(axisX, 300.0f, 120.0f, onMs);
      hold(axisX, 5.0f, 0.0f, offMs);
    }
  };

  /* S1：闭合 + 3Hz：第 1 个周期无节奏不进入；锁定后进入，OFF 段保持。 */
  resetScenario();
  rotateUntilClosed(100.0f);
  startStop(500.0f, 167, 166, 1);
  CHECK(stateResult().contact == 0,    "S1 3Hz 首周期：节奏未锁定 contact=0");
  startStop(500.0f, 167, 166, 5);
  CHECK(stateResult().ssValid && stateResult().ssHz > 2.8f && stateResult().ssHz < 3.2f,
        "S1 3Hz：ssHz 锁定在 3Hz（%.2f）", stateResult().ssHz);
  CHECK(stateResult().contact == 1,    "S1 3Hz 锁定后：contact=1（OFF 段末尾仍为 1）");

  /* S4：接 S1，节奏仍在但手张开：姿态门必须立即退出。 */
  {
    int guard = 0;
    while (stateResult().closeAngleDeg >= ANGLE_CLOSED_OFF_DEG && guard++ < 5000) {
      feed(0.0f, 300.0f, 120.0f, -100.0f);
    }
    const uint32_t tOpen = g_us;
    while (stateResult().contact != 0 && (uint32_t)(g_us - tOpen) < 300000u) {
      feed(0.0f, 300.0f, 120.0f, 0.0f);
    }
    CHECK(stateResult().contact == 0,  "S4 节奏中松手：contact=0");
    CHECK((uint32_t)(g_us - tOpen) <= 200000u, "S4 节奏中松手：0.2s 内退出");
  }

  /* S2：未闭合 + 3Hz 节奏：不能进入。 */
  resetScenario();
  startStop(500.0f, 167, 166, 6);
  CHECK(stateResult().ssValid,         "S2 未闭合：节奏已锁定");
  CHECK(stateResult().contact == 0,    "S2 未闭合：contact=0");

  /* S3：闭合 + 1Hz 且幅值不够：低于 CT_SS_ENTRY_MIN_HZ，不走节奏路径。 */
  resetScenario();
  rotateUntilClosed(100.0f);
  startStop(500.0f, 500, 500, 5);
  CHECK(stateResult().ssValid && stateResult().ssHz < CT_SS_ENTRY_MIN_HZ,
        "S3 1Hz：ssHz=%.2f 低于进入门", stateResult().ssHz);
  CHECK(stateResult().contact == 0,    "S3 1Hz 幅值不够：contact=0");
#endif

#if SS_EDGE_FIX_ENABLE
  /* E2：0.5Hz 读数已锁定后，ON 段中途采样缺口（实板 3.txt 75.8s 复现）。
   * 修正前缺口后被当成新起振，插进 0.4s 假周期，读数掉 8s；修正后只失效到下一个真起振。 */
  resetScenario();
  for (int c = 0; c < 4; c++) { hold(0.0f, 500.0f, 120.0f, 1000); hold(0.0f, 5.0f, 0.0f, 1000); }
  hold(0.0f, 500.0f, 120.0f, 100);
  CHECK(stateResult().ssValid,         "E2 前置：0.5Hz 读数已锁定");
  hold(0.0f, 500.0f, 120.0f, 300);
  { SamplerGap g = {}; g.kind = SMP_GAP_MISSING; g.spanUs = 5000; stateOnGap(g); }
  hold(0.0f, 500.0f, 120.0f, 500);
  hold(0.0f, 5.0f, 0.0f, 1000);
  hold(0.0f, 500.0f, 120.0f, 100);   // 缺口后的第一个真起振
  CHECK(stateResult().ssValid && stateResult().ssHz > 0.45f && stateResult().ssHz < 0.55f,
        "E2 ON 段缺口后：下一个起振即恢复读数（%.2f）", stateResult().ssHz);
#endif

  printf("\n%s（%d/%d 项通过）\n", g_fail == 0 ? "全部通过" : "存在失败",
         g_pass, g_pass + g_fail);
  return g_fail == 0 ? 0 : 1;
}
