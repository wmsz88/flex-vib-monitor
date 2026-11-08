// 缺样通知测试（Q3 / T05、T06）：链接真实 dsp.cpp + state.cpp。
//
// 编译：
//   g++ -std=c++17 -Wall -I test -I . test/gap_test.cpp dsp.cpp state.cpp
//       -o test/gap_test.exe
//
// 设计要点：
//   * 用可控的 SamplerGap 注入缺口，验证状态层的反应
//   * feed() 逐样本驱动，能在任意时刻插入缺口
//   * T05g 用"缺口前后振幅突变"验证包络窗是否被污染
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Arduino.h"
#include "dsp.h"
#include "state.h"
#include "config.h"

/* 新接口在实现落地前用兜底声明，保证旧代码也能编译出"红" */
#if !defined(SMP_BIG_GAP_MS) && !defined(SMP_GAP_KIND_DEFINED)
// 旧代码：用 uint32_t 缺口
#define GAP_IS_LEGACY 1
#endif

/* ================= 虚拟时间 ================= */
static uint32_t g_us = 0;
uint32_t millis(void) { return g_us / 1000u; }
uint32_t micros(void) { return g_us; }
void delay(uint32_t ms) { g_us += ms * 1000u; }
void delayMicroseconds(uint32_t us) { g_us += us; }

/* ================= MPU 桩 ================= */
float mpuLsbPerG(void) { return 16384.0f; }
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
static float    g_ampMg = 0;      // 当前振动振幅（Z 轴）
static float    g_freqHz = 150;

static void resetAll(void) {
  dspInit();
  stateReset();
  stateSetSensorOk(true);
  g_us = 0;
  g_seq = 0;
  g_ampMg = 0;
  g_freqHz = 150;
  /* 手工注入一份有效标定（与 host_test 一致） */
  DspCal& cal = const_cast<DspCal&>(dspCal());
  cal.bias[2] = 1000.0f; cal.envRest = 2.0f; cal.ctRest = 1.0f; cal.zcl = 8.0f;
  cal.ok = cal.quality = true;
  stateApplyCal(cal);
  stateReset();
  stateSetSensorOk(true);
}

/* 喂 n 个样本，可指定启停（onoffHz>0 时 50% 占空比） */
static void feed(uint32_t n, float onoffHz) {
  for (uint32_t i = 0; i < n; i++) {
    g_us += 1000000u / FS_HZ;
    float t = g_us / 1e6f;
    bool on = g_ampMg > 0 && (onoffHz <= 0 || fmodf(t * onoffHz, 1.0f) < 0.5f);
    float noise = ((rand() % 2001) - 1000) / 1000.0f * 3.0f;
    float z = 1000.0f + noise + (on ? g_ampMg * sinf(2 * PI * g_freqHz * t) : 0);

    Sample s;
    s.seq = g_seq++;
    s.tUs = g_us;
    s.raw.x = 0;
    s.raw.y = 0;
    s.raw.z = (int16_t)(z * 16.384f);

    DspFrame f;
    dspProcess(s, &f);
    stateOnFrame(f, s.tUs);
    stateTick(g_us);
  }
}

/* 注入缺口：驱动 dspOnGap + stateOnGap，然后跳过 n 个节拍的采样 */
static void injectGap(SamplerGapKind kind, uint32_t missed, uint32_t spanUs) {
  SamplerGap g;
  g.kind = kind;
  g.missed = (kind == SMP_GAP_MISSING) ? missed : 0;
  g.spanUs = spanUs;
  g.devUs = (kind == SMP_GAP_JITTER) ? (int32_t)spanUs - (1000000 / FS_HZ) : 0;
  dspOnGap(g);
  stateOnGap(g);
  g_seq += (kind == SMP_GAP_MISSING) ? missed : 0;
  g_us += spanUs;
}

int main(void) {
  srand(20261018);
  printf("=== gap_test：Q3 缺样通知状态层 ===\n");
  printf("FS_HZ=%d  ENV_WIN=%d  BLOCK_N=%d  SMP_BIG_GAP_MS=%d  AMP_STALE_MS=%d\n\n",
         FS_HZ, ENV_WIN, BLOCK_N,
#ifdef SMP_BIG_GAP_MS
         SMP_BIG_GAP_MS,
#else
         -1,
#endif
#ifdef AMP_STALE_MS
         AMP_STALE_MS
#else
         -1
#endif
         );

  /* ================= T05a：单次缺样 → 主频立即失效 ================= */
  printf("--- T05a 单次缺样，主频应立即失效 ---\n");
  resetAll();
  g_ampMg = 300;
  feed(600, 0);                       // 攒满多个窗口
  CHECK(stateResult().freqValid, "缺样前主频有效（%.1f Hz）", stateResult().freqHz);

  injectGap(SMP_GAP_MISSING, 1, 2000);
  CHECK(!stateResult().freqValid, "缺样后主频立即失效（不等 300ms）");
  CHECK(stateResult().freqHz == 0.0f, "freqHz 清零（实际 %.1f）", stateResult().freqHz);
  printf("\n");

  /* ================= T05b：恢复 ================= */
  printf("--- T05b 缺口后恢复 ---\n");
  feed(400, 0);
  CHECK(stateResult().freqValid, "恢复满窗后主频有效（%.1f Hz）", stateResult().freqHz);
  CHECK(fabsf(stateResult().freqHz - g_freqHz) < 2.0f, "频率准确（%.2f vs %.1f）",
        stateResult().freqHz, g_freqHz);
  printf("\n");

  /* ================= T05c：大缺口 → 周期历史清空 ================= */
  printf("--- T05c 大缺口（30ms），启停历史应清空 ---\n");
  resetAll();
  g_ampMg = 300;
  feed(2500, 1.0f);                   // 1Hz 启停，攒够周期
  CHECK(stateResult().ssValid, "缺样前启停有效（%.2f Hz, %u 周期）",
        stateResult().ssHz, (unsigned)stateResult().ssPeriods);

  injectGap(SMP_GAP_MISSING, 30, 30000);
  CHECK(!stateResult().freqValid, "主频失效");
  CHECK(!stateResult().ssValid, "ssValid 立即失效");
  CHECK(stateResult().ssPeriods == 0, "周期历史清空（实际 %u）",
        (unsigned)stateResult().ssPeriods);
  printf("\n");

  /* ================= T05d：JITTER → 周期历史保留 ================= */
  printf("--- T05d 时间抖动，周期历史应保留 ---\n");
  resetAll();
  g_ampMg = 300;
  feed(2500, 1.0f);
  CHECK(stateResult().ssValid, "缺样前启停有效（%u 周期）", (unsigned)stateResult().ssPeriods);
  uint8_t periodsBefore = stateResult().ssPeriods;

  injectGap(SMP_GAP_JITTER, 0, 1600);
  CHECK(!stateResult().freqValid, "主频失效");
  CHECK(!stateResult().ampValid, "ampValid 失效");
  CHECK(!stateResult().ssValid, "ssValid 立即失效");
  CHECK(stateResult().ssPeriods == periodsBefore,
        "周期历史保留（%u → %u）", (unsigned)periodsBefore, (unsigned)stateResult().ssPeriods);
  printf("\n");

  /* ================= T05e：幅值"未知"而非 0 ================= */
  printf("--- T05e 缺口后 ampValid=false 但数值不清零 ---\n");
  resetAll();
  g_ampMg = 300;
  feed(400, 0);
  float ampBefore = stateResult().ampMg;
  CHECK(ampBefore > 100.0f, "缺样前幅值 %.1f mg", ampBefore);

  injectGap(SMP_GAP_MISSING, 1, 2000);
  CHECK(!stateResult().ampValid, "ampValid == false");
  CHECK(stateResult().ampMg == ampBefore,
        "ampMg 数值未清零（%.1f，区分未知与 0）", stateResult().ampMg);
  printf("\n");

  /* ================= T05f：幅值超时 ================= */
  printf("--- T05f 只停喂包络 > AMP_STALE_MS，幅值应失效 ---\n");
  resetAll();
  g_ampMg = 300;
  feed(400, 0);
  CHECK(stateResult().ampValid, "刚喂完幅值有效");

  stateTick(g_us + (uint32_t)AMP_STALE_MS * 1000u + 5000u);
  CHECK(!stateResult().ampValid, "超时后 ampValid == false");
  printf("\n");

  /* ================= T06a：缺口打断"连续确认" ================= */
  printf("--- T06a 缺口打断启停确认链（关键用例）---\n");
  NOTE("构造：让 hiCnt 累计到 SS_CONFIRM-1，插缺口，再喂 1 个点。");
  NOTE("若缺口不清 hiCnt，会跨缺口拼成 SS_CONFIRM → vib 误翻转为 1。");
  resetAll();
  g_ampMg = 0;
  feed(300, 0);                       // 静置，vib=0，hiCnt=0
  CHECK(stateResult().vib == 0, "静置时 vib == 0");

  /* 关键时序（实测，见下方调试输出）：
       ENV_WIN=30, ENV_HOP=10, SS_CONFIRM=3
       第 1 个包络点在第 30 个样本附近，之后每 10 个一个点
       hiCnt: 第1点→1, 第2点→2, 第3点→3 才翻转
     为避免依赖"包络点落在第几个样本"这种脆弱假设，
     这里先找到"刚好还没翻转"的位置，再插缺口。 */
  g_ampMg = 300;
  uint32_t fed = 0;
  while (stateResult().vib == 0 && fed < ENV_WIN + ENV_HOP * (SS_CONFIRM + 2)) {
    feed(1, 0);
    fed++;
  }
  /* 此刻 vib 刚变 1。回退无法撤销，所以改为：用一个"差一个点"的独立场景。 */
  NOTE("（校准）首次翻转发生在第 %u 个样本", fed);
  printf("\n");

  /* 正式构造：直接用一个全新的、只喂到"差一个点"的状态。
     由于 stateReset 不清 dsp 的 envHop 累加器，样本数不能硬编码，
     这里用"翻转前最后一次 vib==0"的样本数作为分界。 */
  resetAll();
  g_ampMg = 300;
  uint32_t lastZero = 0;
  for (uint32_t i = 0; i < ENV_WIN + ENV_HOP * (SS_CONFIRM + 2); i++) {
    feed(1, 0);
    if (stateResult().vib == 0) lastZero = i + 1;
    else break;
  }
  NOTE("翻转前最后一个 vib==0 的样本数 = %u（即此时已有 %d 个确认点）",
       lastZero, SS_CONFIRM - 1);

  resetAll();
  g_ampMg = 300;
  feed(lastZero, 0);
  CHECK(stateResult().vib == 0, "确认前 vib 仍为 0（幅值 %.1f > hi %.1f）",
        stateResult().ampMg, stateThresholds().hi);

  injectGap(SMP_GAP_MISSING, 1, 2000);   // ★ 若不清 hiCnt，下一步会拼成 3
  /* 缺口后 dspOnGap 会 envReset()，必须重新攒满 ENV_WIN 才出新包络点。
     只喂 ENV_HOP 根本产不出点，测不到"拼连续"这个缺陷——这里必须喂够。 */
  feed(ENV_WIN + ENV_HOP, 0);
  CHECK(stateResult().vib == 0,
        "缺口打断后，新点不足以确认（vib=%u；若为 1 说明跨缺口拼了连续）",
        (unsigned)stateResult().vib);

  /* 对照组：不插缺口时，再多喂 1 个点应该翻转 */
  printf("      对照组：不插缺口，再喂 1 个点应翻转 ---\n");
  resetAll();
  g_ampMg = 300;
  feed(lastZero + ENV_HOP, 0);
  CHECK(stateResult().vib == 1,
        "无缺口时第 %d 个点足以确认（vib=%u）", SS_CONFIRM, (unsigned)stateResult().vib);
  printf("\n");

  /* ================= T06b：边沿附近大缺口 ================= */
  printf("--- T06b 边沿附近大缺口 → 周期清空 ---\n");
  resetAll();
  g_ampMg = 300;
  feed(2500, 1.0f);
  CHECK(stateResult().ssValid, "缺口前启停有效");

  injectGap(SMP_GAP_MISSING, 30, 30000);
  feed(200, 1.0f);
  CHECK(!stateResult().ssValid || stateResult().ssPeriods < SS_MIN_PERIODS,
        "大缺口后周期未立即恢复（ssValid=%d, %u 周期）",
        stateResult().ssValid, (unsigned)stateResult().ssPeriods);
  printf("\n");

  /* ================= T06c / T06e：小缺口后 ssValid 立即 false，但历史保留 ================= */
  printf("--- T06c/T06e 小缺口：ssValid 立即 false，周期历史保留 ---\n");
  resetAll();
  g_ampMg = 300;
  feed(2500, 1.0f);
  uint8_t pBefore = stateResult().ssPeriods;
  CHECK(stateResult().ssValid && pBefore >= SS_MIN_PERIODS,
        "缺口前有效（%u 周期）", (unsigned)pBefore);

  injectGap(SMP_GAP_MISSING, 5, 6000);
  CHECK(!stateResult().ssValid, "小缺口后 ssValid 立即为 false");
  CHECK(stateResult().ssPeriods == pBefore,
        "周期历史保留（%u → %u）", (unsigned)pBefore, (unsigned)stateResult().ssPeriods);
  printf("\n");

  /* ================= T06d：大 span 的 JITTER 不清历史 ================= */
  printf("--- T06d 大 span 的 JITTER，周期历史仍保留 ---\n");
  resetAll();
  g_ampMg = 300;
  feed(2500, 1.0f);
  uint8_t pJ = stateResult().ssPeriods;
  CHECK(pJ >= SS_MIN_PERIODS, "缺口前 %u 周期", (unsigned)pJ);

  injectGap(SMP_GAP_JITTER, 0, 30000);   // spanUs 很大，但 kind=JITTER
  CHECK(!stateResult().ssValid, "ssValid 失效");
  CHECK(stateResult().ssPeriods == pJ,
        "周期历史保留（%u → %u，因 kind!=MISSING）",
        (unsigned)pJ, (unsigned)stateResult().ssPeriods);
  printf("\n");

  /* ================= T05g：缺口后首个包络点应是干净窗 ================= */
  printf("--- T05g 缺口后首个包络点不得是混合窗 ---\n");
  resetAll();
  g_ampMg = 300;
  feed(400, 0);
  float ampVib = stateResult().ampMg;
  CHECK(ampVib > 100.0f, "缺口前振动幅值 %.1f mg", ampVib);

  /* 缺口后立刻停止振动 */
  injectGap(SMP_GAP_MISSING, 1, 2000);
  g_ampMg = 0;
  feed(ENV_WIN + ENV_HOP, 0);          // 足够攒满一个干净包络窗

  float ampAfter = stateResult().ampMg;
  CHECK(ampAfter < 20.0f,
        "缺口后首个包络点接近 0（实际 %.1f mg；若为混合窗会显著偏高）", ampAfter);
  printf("\n");

  /* ================= T06f：缺口不得插出虚假边沿（第 6 步新增） ================= */
  printf("--- T06f 缺口前后包络跨阈值，边沿时刻必须落在缺口之后 ---\n");
  NOTE("构造：缺口前包络 0.5（低于 hi），缺口插在 t=T0，缺口后第一点就是 400（高于 hi）。");
  NOTE("若 stateOnGap 没清包络点历史，ssStep 会拿这两点插值，");
  NOTE("把边沿时刻插到缺口中间（甚至缺口之前）——那是从未真实发生的边沿。");
  {
    resetAll();
    const uint32_t step = ENV_HOP * (1000000u / FS_HZ);   // 10ms
    uint32_t t = 0;

    /* 基准段：60 个低电平包络点（600ms） */
    for (int i = 0; i < 60; i++) { stateTestInjectEnv(0.5f, t); t += step; }
    CHECK(stateResult().vib == 0, "缺口前 vib == 0");
    const uint32_t tGap = t;                      // 缺口发生在这一刻

    SamplerGap g;
    g.kind = SMP_GAP_MISSING; g.missed = 3; g.spanUs = 3000; g.devUs = 0;
    stateOnGap(g);

    /* 缺口后：直接给高电平，让它在缺口后立即起振 */
    const uint32_t tFirstHigh = t;
    for (int i = 0; i < 10; i++) { stateTestInjectEnv(400.0f, t); t += step; }

    uint32_t edgeUs = 0;
    const bool hasEdge = stateTestLastEdgeUs(&edgeUs);
    CHECK(hasEdge, "确实记录到一次起振边沿");
    if (hasEdge) {
      NOTE("缺口时刻 t=%lu us，缺口后首个高电平 t=%lu us，记录到的边沿 t=%lu us",
           (unsigned long)tGap, (unsigned long)tFirstHigh, (unsigned long)edgeUs);
      /* 关键断言：边沿不得早于缺口后第一个高电平点。
         若没清历史，插值会用 (0.5 @ tGap-step) 和 (400 @ tFirstHigh)，
         算出的穿越时刻会落在两者之间——紧挨着 tGap，属于缺口内部。 */
      CHECK(edgeUs >= tFirstHigh,
            "边沿时刻 %lu >= 缺口后首个高电平 %lu（没有被插到缺口中间）",
            (unsigned long)edgeUs, (unsigned long)tFirstHigh);
    }
  }
  printf("\n");

  /* ================= 收尾：断线清零 ================= */
  printf("--- 收尾：断线应清零所有有效性 ---\n");
  resetAll();
  g_ampMg = 300;
  feed(600, 0);
  CHECK(stateResult().freqValid, "断线前主频有效");

  stateSetSensorOk(false);
  CHECK(!stateResult().freqValid, "断线后主频失效");
  CHECK(!stateResult().ampValid, "断线后幅值失效");
  CHECK(!stateResult().ssValid, "断线后启停失效");
  CHECK(stateResult().vib == 0, "断线后 vib 清零");

  printf("\n%s（%d/%d 项通过，%d 项失败）\n",
         g_fail ? "存在失败" : "全部通过", g_total - g_fail, g_total, g_fail);
  return g_fail ? 1 : 0;
}
