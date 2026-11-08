// 时序统计测试（A4 / T15）：链接真实 sampler.cpp。
//
// 编译：
//   g++ -std=c++17 -Wall -DSMP_TEST_HOOKS -DSMP_STOP_TIMEOUT_MS=5
//       -I test -I . test/timing_test.cpp test/esp_timer_stub.cpp sampler.cpp
//       -o test/timing_test.exe
//
// 设计要点：
//   * 直接驱动 tickBegin/tickEnd 灌入可控的读耗时
//   * 验证 EMA 的算术平均阶段精确、之后平滑且不回绕
//   * 验证缺口不污染 span 统计（T15g）
//   * 验证失败读取计入（T15h）
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Arduino.h"
#include "sampler.h"
#include "config.h"
#include "esp_timer.h"

/* 新接口在实现落地前用兜底声明 */
#ifndef FS_PERIOD_US
#define FS_PERIOD_US (1000000UL / FS_HZ)
#endif

/* ================= 虚拟时间 ================= */
static uint32_t g_us = 0;
uint32_t millis(void) { return g_us / 1000u; }
uint32_t micros(void) { return g_us; }
void delay(uint32_t ms) { g_us += ms * 1000u; }
void delayMicroseconds(uint32_t us) { g_us += us; }

/* ================= MPU 桩 ================= */
static bool g_readOk = true;
static uint32_t g_readCostUs = 400;      // 每次读的耗时
bool mpuReadAccel(MpuRaw* o) {
  g_us += g_readCostUs;
  o->x = 0; o->y = 0; o->z = 16384;
  return g_readOk;
}
float mpuLsbPerG(void) { return 16384.0f; }
uint32_t mpuErrCount(void) { return 0; }
void mpuClearErr(void) {}
/* samplerAddClip 由真实 sampler.cpp 提供，不要再定义 */

/* ================= 断言 ================= */
static int g_fail = 0, g_total = 0;
#define CHECK(cond, ...) do {                                   \
  g_total++;                                                    \
  if (!(cond)) { g_fail++; printf("[FAIL] "); }                 \
  else          { printf("[PASS] "); }                          \
  printf(__VA_ARGS__); printf("\n");                            \
} while (0)
#define NOTE(...) do { printf("       "); printf(__VA_ARGS__); printf("\n"); } while (0)

static void fresh(void) {
  probe_reset();
  g_us = 0;
  g_readOk = true;
  g_readCostUs = 400;
  (void)samplerStop();
  (void)samplerReset();
  samplerClearStats();
  samplerTimingReset();
}

/* 模拟一次完整回调：tickBegin -> 读 -> tickEnd
   注意 t0Us 必须 >0：实现里用 0 表示"不统计本次耗时"（生产路径不需要这个豁免，
   但测试钩子要能表达"跳过"。这里一律传真实起始时刻）。 */
static bool oneTick(uint32_t /*unused*/) {
  uint32_t epoch, seq;
  if (!samplerTestTickBegin(&epoch, &seq)) return false;
  const uint32_t t0 = micros();          // 与生产路径一致：进入读之前取时刻
  if (t0 == 0) g_us += 1;                // 极早期避免 t0==0（会被当成"不统计"）
  const uint32_t t0b = micros();
  MpuRaw r;
  const bool ok = mpuReadAccel(&r);
  return samplerTestTickEnd(ok, epoch, seq, r, t0b);
}

int main(void) {
  printf("=== timing_test：A4 时序统计 ===\n");
  printf("FS_HZ=%d  FS_PERIOD_US=%lu  RING_N=%d\n\n",
         FS_HZ, (unsigned long)FS_PERIOD_US, RING_N);

  /* ================= T15i：初始值 ================= */
  printf("--- T15i 初始状态与清零 ---\n");
  fresh();
  {
    const TimingStats& t = samplerTiming();
    CHECK(t.readN == 0 && t.readUsMax == 0, "初始 readN/readUsMax 为 0");
    CHECK(t.spanUsMin == 0xFFFFFFFFu, "初始 spanUsMin == 0xFFFFFFFF（实际 0x%08lX）",
          (unsigned long)t.spanUsMin);
    CHECK(t.ringHighWater == 0, "初始 ringHighWater == 0");
  }
  printf("\n");

  /* ================= T15a：readUsMax ================= */
  printf("--- T15a readUsMax 抓到最大值 ---\n");
  fresh();
  {
    samplerStart();
    /* 灌入递增的读耗时：100,200,...,1000 */
    for (int i = 1; i <= 10; i++) {
      g_readCostUs = (uint32_t)(i * 100);
      oneTick(0);
      g_us += FS_PERIOD_US;      // 补齐一个采样周期
    }
    const TimingStats& t = samplerTiming();
    NOTE("readN=%lu readUsMax=%lu", (unsigned long)t.readN, (unsigned long)t.readUsMax);
    CHECK(t.readN == 10, "readN == 10（实际 %lu）", (unsigned long)t.readN);
    CHECK(t.readUsMax >= 900 && t.readUsMax <= 1100,
          "readUsMax ≈ 1000（实际 %lu）", (unsigned long)t.readUsMax);
  }
  printf("\n");

  /* ================= T15b：64 个样本的算术平均精确 ================= */
  printf("--- T15b 前 64 个样本用算术平均，avg 精确 ---\n");
  fresh();
  {
    samplerStart();
    g_readCostUs = 500;
    for (int i = 0; i < 64; i++) {
      oneTick(0);
      g_us += FS_PERIOD_US;
    }
    const TimingStats& t = samplerTiming();
    NOTE("readN=%lu readUsAvg=%lu", (unsigned long)t.readN, (unsigned long)t.readUsAvg);
    CHECK(t.readN == 64, "readN == 64");
    CHECK(t.readUsAvg == 500, "readUsAvg 精确等于 500（实际 %lu）", (unsigned long)t.readUsAvg);
  }
  printf("\n");

  /* ================= T15c：EMA 阶段平滑 ================= */
  printf("--- T15c 第 65 个样本起转 EMA，突变被平滑 ---\n");
  fresh();
  {
    samplerStart();
    g_readCostUs = 500;
    for (int i = 0; i < 64; i++) { oneTick(0); g_us += FS_PERIOD_US; }
    const uint32_t avgBefore = samplerTiming().readUsAvg;

    /* 灌一个 1500us 的突刺 */
    g_readCostUs = 1500;
    oneTick(0);
    g_us += FS_PERIOD_US;
    const uint32_t avgAfter = samplerTiming().readUsAvg;

    NOTE("avg: %lu -> %lu（突刺 1500）", (unsigned long)avgBefore, (unsigned long)avgAfter);
    CHECK(avgAfter > avgBefore, "avg 上升");
    CHECK(avgAfter < 600, "avg 仍远小于突刺 1500（EMA 生效，实际 %lu）",
          (unsigned long)avgAfter);
    CHECK(samplerTiming().readUsMax >= 1400, "但 readUsMax 记录了突刺");
  }
  printf("\n");

  /* ================= T15d：EMA 无符号回绕 ================= */
  printf("--- T15d 从 1000 降到 100，avg 单调下降不回绕 ---\n");
  fresh();
  {
    samplerStart();
    g_readCostUs = 1000;
    for (int i = 0; i < 128; i++) { oneTick(0); g_us += FS_PERIOD_US; }
    const uint32_t high = samplerTiming().readUsAvg;
    NOTE("高位稳定值 avg = %lu", (unsigned long)high);
    CHECK(high > 900 && high < 1100, "稳定在 ~1000");

    g_readCostUs = 100;
    uint32_t prev = high;
    bool monotonic = true;
    /* EMA 时间常数是 64 个样本，从 1000 收敛到 100 需要 ~4 倍常数，
       所以灌 256 个才能接近目标。这里只断言"单调下降 + 明显下降 + 不回绕"，
       不要求精确收敛到 100——精确收敛是 EMA 的固有慢速特性，不是缺陷。 */
    for (int i = 0; i < 256; i++) {
      oneTick(0); g_us += FS_PERIOD_US;
      const uint32_t cur = samplerTiming().readUsAvg;
      if (cur > prev) monotonic = false;     // 回绕会导致突然变大
      prev = cur;
    }
    NOTE("256 次下降后 avg = %lu", (unsigned long)prev);
    CHECK(monotonic, "avg 单调下降（无符号回绕会让它突然变大）");
    CHECK(prev < high, "最终值 %lu < 初始 %lu", (unsigned long)prev, (unsigned long)high);
    CHECK(prev < 200, "已明显接近 100（实际 %lu < 200）", (unsigned long)prev);
  }
  printf("\n");

  /* ================= T15e：水位 ================= */
  printf("--- T15e ringHighWater 反映入队占用 ---\n");
  fresh();
  {
    samplerStart();
    /* 不消费，灌 20 个 */
    for (int i = 0; i < 20; i++) { oneTick(0); g_us += FS_PERIOD_US; }
    const TimingStats& t = samplerTiming();
    NOTE("ringHighWater = %u", (unsigned)t.ringHighWater);
    CHECK(t.ringHighWater >= 19 && t.ringHighWater <= 20,
          "ringHighWater ≈ 20（实际 %u）", (unsigned)t.ringHighWater);
  }
  printf("\n");

  /* ================= T15f：水位不因消费而降低 ================= */
  printf("--- T15f 消费后水位保持峰值 ---\n");
  {
    const uint16_t peak = samplerTiming().ringHighWater;
    Sample s; SamplerGap g;
    int n = 0;
    while (samplerPop(&s, &g)) n++;
    NOTE("消费 %d 个后，ringHighWater = %u（峰值 %u）",
         n, (unsigned)samplerTiming().ringHighWater, (unsigned)peak);
    CHECK(samplerTiming().ringHighWater == peak, "水位保持峰值不降低");
  }
  printf("\n");

  /* ================= T15g：缺口不污染 span 统计 ================= */
  printf("--- T15g 缺口不污染 spanUsMax ---\n");
  fresh();
  {
    samplerStart();
    Sample s; SamplerGap g;

    /* 先灌 3 个正常样本 */
    for (int i = 0; i < 3; i++) { oneTick(0); g_us += FS_PERIOD_US; }
    while (samplerPop(&s, &g)) {}

    /* 灌 1 个样本，但序号跳 10（模拟缺样 9 个） */
    uint32_t epoch, seq;
    samplerTestTickBegin(&epoch, &seq);
    MpuRaw r; g_readOk = true; mpuReadAccel(&r);
    samplerTestTickEnd(true, epoch, seq + 10, r, 0);
    g_us += 10 * FS_PERIOD_US;

    while (samplerPop(&s, &g)) {
      if (g.kind != SMP_GAP_NONE) {
        NOTE("检测到缺口 kind=%d spanUs=%lu missed=%lu",
             (int)g.kind, (unsigned long)g.spanUs, (unsigned long)g.missed);
      }
    }

    /* 再灌 5 个正常 */
    for (int i = 0; i < 5; i++) { oneTick(0); g_us += FS_PERIOD_US; }
    while (samplerPop(&s, &g)) {}

    const TimingStats& t = samplerTiming();
    NOTE("spanUsMax=%lu  gapSpanUsMax=%lu  spanN=%lu  gapSpanN=%lu",
         (unsigned long)t.spanUsMax, (unsigned long)t.gapSpanUsMax,
         (unsigned long)t.spanN, (unsigned long)t.gapSpanN);
    CHECK(t.spanUsMax < 2000,
          "spanUsMax 未被 10ms 缺口污染（实际 %lu < 2000）", (unsigned long)t.spanUsMax);
    CHECK(t.gapSpanN > 0, "缺口间隔被单独统计（gapSpanN=%lu）", (unsigned long)t.gapSpanN);
    CHECK(t.gapSpanUsMax > 5000,
          "gapSpanUsMax 反映了缺口时长（实际 %lu）", (unsigned long)t.gapSpanUsMax);
  }
  printf("\n");

  /* ================= T15h：失败读取计入 ================= */
  printf("--- T15h 读失败也计入耗时 ---\n");
  fresh();
  {
    samplerStart();
    g_readCostUs = 400;
    for (int i = 0; i < 10; i++) { oneTick(0); g_us += FS_PERIOD_US; }
    const uint32_t failMaxBefore = samplerTiming().readFailUsMax;

    /* 一次 5000us 的失败读取 */
    g_readOk = false;
    g_readCostUs = 5000;
    oneTick(0);
    g_us += FS_PERIOD_US;
    g_readOk = true;
    g_readCostUs = 400;

    const TimingStats& t = samplerTiming();
    NOTE("readN=%lu readFailN=%lu readUsMax=%lu readFailUsMax=%lu",
         (unsigned long)t.readN, (unsigned long)t.readFailN,
         (unsigned long)t.readUsMax, (unsigned long)t.readFailUsMax);
    CHECK(t.readFailN == 1, "readFailN == 1");
    CHECK(t.readFailUsMax >= 4500,
          "readFailUsMax ≈ 5000（实际 %lu）", (unsigned long)t.readFailUsMax);
    CHECK(t.readUsMax >= 4500,
          "readUsMax 也包含失败那次（实际 %lu）", (unsigned long)t.readUsMax);
    CHECK(t.readFailUsMax > failMaxBefore, "失败耗时确实更新了");
  }
  printf("\n");

  /* ================= T15j：抽取比与输出计数 ================= */
  printf("--- T15j 波形抽取计数（由 output 模块维护，这里只验证统计口径）---\n");
  NOTE("抽取逻辑在 output.cpp，主机测试不链接 output（依赖 Serial），");
  NOTE("改由实板 T15j 验证。此处在 timing_test 中跳过。");
  printf("\n");

  /* ================= 收尾 ================= */
  printf("--- 收尾：停止态下应无新统计 ---\n");
  {
    (void)samplerStop();
    const uint32_t readN = samplerTiming().readN;
    oneTick(0);      // 停止后 tickBegin 会被拒
    CHECK(samplerTiming().readN == readN, "停止后 tickBegin 被拒，统计不增长");
  }

  printf("\n%s（%d/%d 项通过，%d 项失败）\n",
         g_fail ? "存在失败" : "全部通过", g_total - g_fail, g_total, g_fail);
  return g_fail ? 1 : 0;
}
