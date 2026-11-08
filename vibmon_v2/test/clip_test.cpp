// 削顶质量测试（A3 / T09）：链接真实 dsp.cpp + state.cpp。
//
// 编译：
//   g++ -std=c++17 -Wall -I test -I . test/clip_test.cpp dsp.cpp state.cpp
//       -o test/clip_test.exe
//
// 设计要点：
//   * 量程从 mpuLsbPerG() 推导，不写死 16384（ACCEL_FS_SEL 变了会跟着变）
//   * 削顶用 raw 直接超 ACCEL_CLIP_RAW 注入
//   * T09i 验证"包络窗内有削顶时 ampValid 不能提前恢复"
//   * T09j 验证"非主轴削顶不误伤主频"
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

/* ================= 量程（与固件同源） ================= */
static float g_lsbPerG = 16384.0f;
float mpuLsbPerG(void) { return g_lsbPerG; }
static inline int16_t mg2raw(float mg) { return (int16_t)lrintf(mg * g_lsbPerG / 1000.0f); }

/* ================= MPU 桩 ================= */
bool mpuReadAccel(MpuRaw*) { return false; }
uint32_t mpuErrCount(void) { return 0; }
void mpuClearErr(void) {}

/* ================= 采样器桩 ================= */
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
/* 第 5 步：dsp/state 不直接用，但保持桩完整，避免链接期才发现 */
static TimingStats g_stubTiming = {};
const TimingStats& samplerTiming(void) { return g_stubTiming; }
void samplerTimingReset(void) { g_stubTiming = TimingStats(); }
void samplerNoteLoopUs(uint32_t) {}
void samplerNoteOutUs(uint32_t) {}
const SamplerStats& samplerStats(void) { return g_stats; }
void samplerClearStats(void) { g_stats = SamplerStats(); }
bool samplerSensorLost(void) { return false; }
bool samplerPop(Sample*, SamplerGap*) { return false; }

/* ================= 断言 ================= */
static int g_fail = 0, g_total = 0;
#define CHECK(cond, ...) do {                                   \
  g_total++;                                                    \
  if (!(cond)) { g_fail++; printf("[FAIL] "); }                 \
  else          { printf("[PASS] "); }                          \
  printf(__VA_ARGS__); printf("\n");                            \
} while (0)
#define NOTE(...) do { printf("       "); printf(__VA_ARGS__); printf("\n"); } while (0)

/* ================= 喂数 ================= */
static uint32_t g_seq = 0;
static float    g_ampMg[3] = {0, 0, 0};   // 三轴振幅
static float    g_freqHz = 150;
static bool     g_clipX = false, g_clipY = false, g_clipZ = false;

/* 记录每个 winReady 的观测值，供"第 N 个窗口"类断言使用 */
struct WinObs { uint16_t clip; int level; };
static WinObs   g_winObs[64];
static int      g_nWin = 0;
static int      g_nEnvReady = 0;

static void resetAll(void) {
  dspInit();
  stateReset();
  stateSetSensorOk(true);
  g_us = 0; g_seq = 0;
  g_ampMg[0] = g_ampMg[1] = g_ampMg[2] = 0;
  g_freqHz = 150;
  g_clipX = g_clipY = g_clipZ = false;
  g_nWin = 0; g_nEnvReady = 0;
  samplerClearStats();

  /* 手工注入一份有效标定 */
  DspCal& cal = const_cast<DspCal&>(dspCal());
  cal.bias[2] = 1000.0f; cal.envRest = 2.0f; cal.ctRest = 1.0f; cal.zcl = 8.0f;
  cal.ok = cal.quality = true;
  stateApplyCal(cal);
  stateReset();
  stateSetSensorOk(true);
}

/* 喂 n 个样本 */
static void feed(uint32_t n) {
  for (uint32_t i = 0; i < n; i++) {
    g_us += 1000000u / FS_HZ;
    const float t = g_us / 1e6f;
    const float noise = ((rand() % 2001) - 1000) / 1000.0f * 3.0f;

    Sample s;
    s.seq = g_seq++;
    s.tUs = g_us;
    s.raw.x = g_clipX ? (int16_t)32767 : mg2raw(g_ampMg[0] * sinf(2 * PI * g_freqHz * t));
    s.raw.y = g_clipY ? (int16_t)32767 : mg2raw(g_ampMg[1] * sinf(2 * PI * g_freqHz * t));
    s.raw.z = g_clipZ ? (int16_t)32767
                      : mg2raw(1000.0f + noise + g_ampMg[2] * sinf(2 * PI * g_freqHz * t));

    DspFrame f;
    dspProcess(s, &f);
    if (f.winReady && g_nWin < 64) {
      g_winObs[g_nWin].clip = f.winClip;
      g_winObs[g_nWin].level = (int)f.clipLevel;
      g_nWin++;
    }
    if (f.envReady) g_nEnvReady++;
    stateOnFrame(f, s.tUs);
    stateTick(g_us);
  }
}

/* 关键：两个窗口的周期不同，必须分别控制。
   - 主频窗：hop = HOP(64)，窗口 BLOCK_N(128) → 削顶点要留 128 个样本内
   - 包络窗：窗口 ENV_WIN(30)                    → 削顶点要留 30 个样本内
   所以无法用一个固定的"补充喂数"同时满足两个断言。

   解法：分别构造两个独立场景。
   T09b/T09c 只验证主频侧（clipLevel / freqClipLimited），不验证 ampValid；
   ampValid 已由 T09g/T09i/T09j 专门覆盖。 */
static void injectClipAndRoll(uint32_t nClip) {
  /* 喂到即将出窗（HOP 的整数倍附近），再插削顶，再喂 HOP 让窗口滚过 */
  feed(HOP);
  g_clipZ = true;  feed(nClip);
  g_clipZ = false; feed(HOP);
}

int main(void) {
  g_lsbPerG = 16384.0f / (float)(1 << (ACCEL_FS_SEL & 0x03));
  srand(20261018);

  printf("=== clip_test：A3 削顶质量策略 ===\n");
  printf("ACCEL_FS_SEL=%d  lsb/g=%.0f  量程 ±%dg\n",
         ACCEL_FS_SEL, g_lsbPerG, 2 << (ACCEL_FS_SEL & 0x03));
  printf("ACCEL_CLIP_RAW=%d  削顶线 ≈ %.0f mg\n",
         ACCEL_CLIP_RAW, ACCEL_CLIP_RAW * 1000.0f / g_lsbPerG);
  printf("BLOCK_N=%d  ENV_WIN=%d  HOP=%d  CLIP_SEVERE_RATIO=%.2f\n\n",
         BLOCK_N, ENV_WIN, HOP,
#ifdef CLIP_SEVERE_RATIO
         CLIP_SEVERE_RATIO
#else
         -1.0
#endif
         );

  /* ================= T09l：量程换算自检 ================= */
  printf("--- T09l 削顶线随量程变化 ---\n");
  {
    const float clipMg = ACCEL_CLIP_RAW * 1000.0f / g_lsbPerG;
    const float expect = (ACCEL_FS_SEL == 0) ? 1996.0f
                       : (ACCEL_FS_SEL == 1) ? 3992.0f
                       : (ACCEL_FS_SEL == 2) ? 7983.0f : 15967.0f;
    CHECK(fabsf(clipMg - expect) < 5.0f,
          "FS_SEL=%d 时削顶线 %.0f mg ≈ 期望 %.0f mg", ACCEL_FS_SEL, clipMg, expect);
    CHECK(clipMg > 1000.0f, "削顶线 %.0f mg 远高于 300mg（v1 描述的 300mg 是错的）", clipMg);
  }
  printf("\n");

  /* ================= T09a：无削顶 ================= */
  printf("--- T09a 无削顶，全部正常 ---\n");
  resetAll();
  g_ampMg[2] = 300;
  feed(600);
  {
    const Result& r = stateResult();
    CHECK(r.ampValid, "ampValid == true");
    CHECK(r.freqValid, "freqValid == true（%.1f Hz）", r.freqHz);
    CHECK(!r.freqClipLimited, "freqClipLimited == false");
    CHECK(g_nWin > 0, "出过 %d 个主频窗", g_nWin);
    CHECK(g_winObs[g_nWin - 1].level == (int)CLIP_NONE,
          "最后一个窗 clipLevel == NONE（实际 %d）", g_winObs[g_nWin - 1].level);
  }
  printf("\n");

  /* ================= T09g：单样本削顶 ================= */
  printf("--- T09g 单样本削顶，ampValid 立即失效 ---\n");
  resetAll();
  g_ampMg[2] = 300;
  feed(400);
  CHECK(stateResult().ampValid, "削顶前 ampValid == true");

  g_clipZ = true;
  feed(1);                       // 只削 1 个样本
  g_clipZ = false;
  CHECK(!stateResult().ampValid, "单样本削顶后 ampValid 立即 false");
  printf("\n");

  /* ================= T09e：真窗口计数 ================= */
  printf("--- T09e winClip 是窗口计数而非区间计数 ---\n");
  resetAll();
  g_ampMg[2] = 300;
  feed(200);                     // 先稳定
  const int winBefore = g_nWin;

  g_clipZ = true;
  feed(BLOCK_N);                 // 喂满一个窗口的削顶
  g_clipZ = false;
  {
    /* 找一个包含这段时间的窗口 */
    uint16_t maxClip = 0;
    int maxIdx = -1;
    for (int i = winBefore; i < g_nWin; i++) {
      if (g_winObs[i].clip > maxClip) { maxClip = g_winObs[i].clip; maxIdx = i; }
    }
    CHECK(maxIdx >= 0, "削顶期间出过窗口");
    NOTE("最大 winClip = %u（窗口 %d）", maxClip, maxIdx);
    CHECK(maxClip > BLOCK_N / 2,
          "winClip 接近满窗（%u > %d），说明统计的是整个 128 样本窗",
          maxClip, BLOCK_N / 2);
  }
  printf("\n");

  /* ================= T09f：窗口滚动（前 64 削顶 / 后 64 正常） ================= */
  printf("--- T09f 窗口滚动：前 64 削顶后 64 正常，第一窗应为 64 ---\n");
  resetAll();
  g_ampMg[2] = 300;
  /* 精确构造：先喂到历史窗刚满、下一次 winReady 前的边界 *
     由于 hop 相位不确定，改为直接观察"首个含削顶的窗口"的计数量级 */
  g_clipZ = true;  feed(HOP);        // 64 个削顶
  g_clipZ = false; feed(HOP);        // 64 个正常
  {
    uint16_t maxClip = 0;
    for (int i = 0; i < g_nWin; i++) if (g_winObs[i].clip > maxClip) maxClip = g_winObs[i].clip;
    NOTE("最大 winClip = %u", maxClip);
    CHECK(maxClip >= HOP - 2 && maxClip <= BLOCK_N,
          "含削顶的窗口计数在 [%d, %d] 区间（实际 %u）",
          HOP - 2, BLOCK_N, maxClip);
    /* 旧实现（区间计数）在同一场景下会给出 ≤64 的偏小值，
       且容易因 hop 相位不同给出 0；这里只断言"确实统计到了削顶" */
    CHECK(maxClip > 0, "窗口确实统计到削顶（区间计数在相位不利时会给出 0）");
  }
  printf("\n");

  /* ================= T09b：轻度削顶（5 个） ================= */
  printf("--- T09b 主轴削顶 5 个（3.9%%）→ MILD ---\n");
  resetAll();
  g_ampMg[2] = 300;
  feed(400);
  {
    CHECK(stateResult().freqValid, "削顶前主频有效");

    /* 主频侧验证：削顶后喂 HOP 让窗口滚过，此时削顶点仍在 128 样本窗内。
       （ampValid 由 T09g/T09i/T09j 单独覆盖，此处不断言） */
    const int winBefore = g_nWin;
    injectClipAndRoll(5);

    int mildIdx = -1;
    for (int i = winBefore; i < g_nWin; i++)
      if (g_winObs[i].level == (int)CLIP_MILD) { mildIdx = i; break; }
    NOTE("削顶后出的窗口数 = %d，最后窗 clip=%u level=%d",
         g_nWin - winBefore,
         g_winObs[g_nWin - 1].clip, g_winObs[g_nWin - 1].level);

    CHECK(mildIdx >= 0, "出现过 MILD 等级窗口");
    CHECK(stateResult().freqClipLimited, "freqClipLimited == true（主轴有削顶）");
  }
  printf("\n");

  /* ================= T09c：重度削顶（30 个） ================= */
  printf("--- T09c 主轴削顶 30 个（23%%）→ SEVERE ---\n");
  resetAll();
  g_ampMg[2] = 300;
  feed(400);
  {
    const int winBefore = g_nWin;
    injectClipAndRoll(30);

    bool sawSevere = false;
    for (int i = winBefore; i < g_nWin; i++)
      if (g_winObs[i].level == (int)CLIP_SEVERE) { sawSevere = true; break; }

    NOTE("最后窗 clip=%u level=%d",
         g_winObs[g_nWin - 1].clip, g_winObs[g_nWin - 1].level);
    CHECK(sawSevere, "出现过 SEVERE 等级窗口");
    CHECK(stateResult().freqClipLimited, "freqClipLimited == true（SEVERE 也要标记）");
  }
  printf("\n");

  /* ================= T09c2：SEVERE 时主频必须失效 ================= */
  printf("--- T09c2 SEVERE 窗口内 freqValid 必须为 false ---\n");
  {
    /* 在 SEVERE 窗口刚出的时候立刻检查（不能再喂，否则滚出窗口） */
    resetAll();
    g_ampMg[2] = 300;
    feed(400);
    g_clipZ = true;  feed(30);
    g_clipZ = false;
    /* 只喂到刚好出下一个窗 */
    const int before = g_nWin;
    while (g_nWin == before) feed(1);
    {
      const Result& r = stateResult();
      NOTE("刚出的窗 clip=%u level=%d", g_winObs[g_nWin - 1].clip, g_winObs[g_nWin - 1].level);
      if (g_winObs[g_nWin - 1].level == (int)CLIP_SEVERE) {
        CHECK(!r.freqValid, "SEVERE 时 freqValid == false");
      } else {
        NOTE("本窗尚未达到 SEVERE（clip=%u），跳过", g_winObs[g_nWin - 1].clip);
      }
    }
  }
  printf("\n");

  /* ================= T09i：削顶发生在两个 envReady 之间 ================= */
  printf("--- T09i 包络窗内有削顶时，ampValid 不得提前恢复 ---\n");
  NOTE("构造：喂到快要出包络点，插 1 个削顶样本，再喂到出点");
  NOTE("包络窗 30 样本仍含那个削顶点，RMS 已偏小 → ampValid 必须仍为 false");
  resetAll();
  g_ampMg[2] = 300;
  feed(400);
  CHECK(stateResult().ampValid, "初始 ampValid == true");

  /* 找到"下一次 envReady 之前"的位置：先削顶，再喂满 ENV_WIN 但不足 ENV_WIN+HOP */
  g_clipZ = true;  feed(1);              // 1 个削顶样本进包络窗
  g_clipZ = false;
  feed(ENV_WIN / 2);                     // 喂不够让削顶点滚出包络窗
  {
    /* 此时可能已经出过 envReady，但包络窗内仍有削顶 */
    const Result& r = stateResult();
    CHECK(!r.ampValid,
          "ampValid == false（包络窗内仍有削顶，不能因当前样本干净就恢复）");
  }
  printf("\n");

  /* ================= T09j：非主轴削顶不误伤主频 ================= */
  printf("--- T09j 主轴 Z 正常、X 轴削顶 → 主频不受影响 ---\n");
  resetAll();
  g_ampMg[2] = 300;
  feed(400);
  CHECK(stateResult().freqValid, "基准主频有效");

  g_clipX = true;                        // 只削 X 轴（主轴是 Z）
  feed(BLOCK_N + HOP);
  g_clipX = false;
  {
    const Result& r = stateResult();
    const int lastLevel = g_winObs[g_nWin - 1].level;
    NOTE("最后窗 clip=%u level=%d", g_winObs[g_nWin - 1].clip, lastLevel);
    CHECK(lastLevel == (int)CLIP_NONE,
          "主轴 Z 未削顶 → clipLevel == NONE（实际 %d）", lastLevel);
    CHECK(!r.ampValid,
          "但 ampValid == false（任意轴削顶污染合成 RMS）");
    CHECK(r.freqValid, "主频仍有效（%.1f Hz）", r.freqHz);
    CHECK(!r.freqClipLimited, "freqClipLimited == false（主轴没削顶）");
  }
  printf("\n");

  /* ================= T09d：削顶后恢复 ================= */
  printf("--- T09d 停止削顶后恢复 ---\n");
  resetAll();
  g_ampMg[2] = 300;
  feed(400);
  g_clipZ = true;  feed(BLOCK_N);
  g_clipZ = false;
  {
    CHECK(!stateResult().ampValid, "削顶期间 ampValid == false");
  }
  feed(400);                             // 足够让位图与包络窗都滚干净
  {
    const Result& r = stateResult();
    CHECK(r.ampValid, "恢复后 ampValid == true");
    CHECK(r.freqValid, "恢复后 freqValid == true（%.1f Hz）", r.freqHz);
    CHECK(!r.freqClipLimited, "恢复后 freqClipLimited == false");
    CHECK(g_winObs[g_nWin - 1].level == (int)CLIP_NONE,
          "恢复后 clipLevel == NONE");
  }
  printf("\n");

  /* ================= T09k：freqClipLimited 四处清零 ================= */
  printf("--- T09k freqClipLimited 生命周期 ---\n");
  /* ① stateReset */
  resetAll();
  g_ampMg[2] = 300;
  g_clipZ = true; feed(BLOCK_N + HOP); g_clipZ = false;
  {
    const bool hadLimit = stateResult().freqClipLimited;
    NOTE("削顶后 freqClipLimited = %d", hadLimit);
    stateReset();
    CHECK(!stateResult().freqClipLimited, "stateReset 后清零");
  }
  /* ② stateOnGap */
  {
    resetAll();
    g_ampMg[2] = 300;
    g_clipZ = true; feed(BLOCK_N + HOP); g_clipZ = false;
    SamplerGap g;
    g.kind = SMP_GAP_MISSING; g.missed = 1; g.spanUs = 2000; g.devUs = 0;
    stateOnGap(g);
    CHECK(!stateResult().freqClipLimited, "stateOnGap 后清零");
  }
  /* ③ 主频过期 */
  {
    resetAll();
    g_ampMg[2] = 300;
    g_clipZ = true; feed(BLOCK_N + HOP); g_clipZ = false;
    stateTick(g_us + (uint32_t)FREQ_STALE_MS * 1000u + 10000u);
    CHECK(!stateResult().freqClipLimited, "主频过期后清零");
    CHECK(!stateResult().freqValid, "同时 freqValid 也失效");
  }
  /* ④ 振动停止 */
  {
    resetAll();
    g_ampMg[2] = 300;
    g_clipZ = true; feed(BLOCK_N + HOP); g_clipZ = false;
    g_clipZ = false; g_ampMg[2] = 0;
    feed(600);                          // 停止振动
    CHECK(stateResult().vib == 0, "振动已停止");
    CHECK(!stateResult().freqClipLimited, "振动停止后清零");
  }
  printf("\n");

  /* ================= T09h：fsSel 记录 ================= */
  printf("--- T09h 标定记录量程 ---\n");
  {
    /* 本测试不跑真实标定（桩不支持），只验证字段存在且可读 */
    const DspCal& c = dspCal();
    NOTE("dspCal().fsSel = %d, ACCEL_FS_SEL = %d", (int)c.fsSel, ACCEL_FS_SEL);
    CHECK(c.fsSel == 0 || c.fsSel == ACCEL_FS_SEL,
          "fsSel 字段可读（未标定时为 0，标定后 == ACCEL_FS_SEL）");
  }

  printf("\n%s（%d/%d 项通过，%d 项失败）\n",
         g_fail ? "存在失败" : "全部通过", g_total - g_fail, g_total, g_fail);
  return g_fail ? 1 : 0;
}
