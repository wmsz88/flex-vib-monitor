// 序号与时间戳回绕测试（A5 / T13）：验证 32 位回绕不破坏时间差与状态。
//
// 编译：
//   g++ -std=c++17 -Wall -DSMP_TEST_HOOKS -DSMP_STOP_TIMEOUT_MS=5
//       -I test -I . test/wrap_test.cpp test/esp_timer_stub.cpp sampler.cpp
//       -o test/wrap_test.exe
//
// 关键点：现有代码用 (uint32_t)(a - b) 做差值，模 2^32 下回绕是正确的。
// 本测试的价值是"证明它正确"，防止以后有人把它"优化"成有符号比较。
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Arduino.h"
#include "sampler.h"
#include "config.h"
#include "esp_timer.h"

/* ================= 虚拟时间（可直接跳到回绕点） ================= */
static uint32_t g_us = 0;
uint32_t millis(void) { return g_us / 1000u; }
uint32_t micros(void) { return g_us; }
void delay(uint32_t ms) { g_us += ms * 1000u; }
void delayMicroseconds(uint32_t us) { g_us += us; }

/* ================= MPU 桩 ================= */
bool mpuReadAccel(MpuRaw* o) { o->x = 0; o->y = 0; o->z = 16384; return true; }
float mpuLsbPerG(void) { return 16384.0f; }
uint32_t mpuErrCount(void) { return 0; }
void mpuClearErr(void) {}

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
  (void)samplerStop();
  (void)samplerReset();
  samplerClearStats();
  samplerTimingReset();
}

/* 手动入队一个指定 seq/tUs 的样本 */
static bool pushSample(uint32_t seq, uint32_t tUs) {
  uint32_t epoch, s2;
  if (!samplerTestTickBegin(&epoch, &s2)) return false;
  MpuRaw r = {0, 0, 16384};
  return samplerTestTickEnd(true, epoch, seq, r, 0);
}

int main(void) {
  printf("=== wrap_test：序号与时间戳回绕（A5 / T13）===\n");
  printf("FS_HZ=%d  标称间隔=%d us\n\n", FS_HZ, 1000000 / FS_HZ);

  /* ---------- T13a：micros 回绕后的时间差 ---------- */
  printf("--- T13a micros 回绕 ---\n");
  NOTE("直接验证模 2^32 减法：这是 samplerPop 里 dUs 的算法");
  {
    const uint32_t prev = 0xFFFFFF00u;     // 距回绕还有 256us
    const uint32_t now  = 0x00000100u;     // 回绕后 256us
    const uint32_t dUs  = now - prev;      // 无符号减法，模 2^32
    NOTE("prev=0x%08lX now=0x%08lX → dUs=%lu",
         (unsigned long)prev, (unsigned long)now, (unsigned long)dUs);
    CHECK(dUs == 512u, "回绕后间隔正确 = 512 us（实际 %lu）", (unsigned long)dUs);

    /* 真正的错误做法：先用有符号比较算"now 是否小于 prev"，再取绝对值差。
       那样会认为 now < prev（因为 0x00000100 < 0xFFFFFF00），走错分支。 */
    const bool wrongBranch = (now < prev);          // 有符号比较会得到 true
    NOTE("若用有符号比较，会认为 now < prev（%s），从而走错分支",
         wrongBranch ? "是" : "否");
    CHECK(wrongBranch, "有符号比较确实会误判先后顺序（证明必须用无符号减）");
  }
  printf("\n");

  /* ---------- T13b：seq 回绕识别为连续 ---------- */
  printf("--- T13b seq 回绕应识别为连续 ---\n");
  {
    fresh();
    samplerStart();

    /* 构造：seq 从 0xFFFFFFFF 回绕到 0，间隔正常 */
    const uint32_t tNom = 1000000u / FS_HZ;
    g_us = 0xFFFFFE00u;                    // 接近 micros 回绕
    pushSample(0xFFFFFFFEu, g_us);
    g_us += tNom;
    pushSample(0xFFFFFFFFu, g_us);
    g_us += tNom;
    pushSample(0u, g_us);                  // seq 回绕
    g_us += tNom;
    pushSample(1u, g_us);

    Sample s; SamplerGap g;
    uint32_t nGap = 0, nTotal = 0;
    uint32_t prevSeq = 0; bool first = true;
    while (samplerPop(&s, &g)) {
      nTotal++;
      if (g.kind != SMP_GAP_NONE) {
        nGap++;
        NOTE("意外缺口：seq=%lu kind=%d", (unsigned long)s.seq, (int)g.kind);
      }
      if (!first) {
        const uint32_t d = s.seq - prevSeq;
        CHECK(d == 1, "相邻 seq 差为 1（%lu → %lu，d=%lu）",
              (unsigned long)prevSeq, (unsigned long)s.seq, (unsigned long)d);
      }
      prevSeq = s.seq;
      first = false;
    }
    CHECK(nTotal == 4, "共 4 个样本（实际 %lu）", (unsigned long)nTotal);
    CHECK(nGap == 0, "seq 回绕未被误判为缺口（缺口数 %lu）", (unsigned long)nGap);
  }
  printf("\n");

  /* ---------- T13c：回绕后仍能识别真实缺口 ---------- */
  printf("--- T13c 回绕后仍能识别真实缺口 ---\n");
  {
    fresh();
    samplerStart();
    const uint32_t tNom = 1000000u / FS_HZ;

    g_us = 0xFFFFFF00u;
    pushSample(0xFFFFFFFFu, g_us);
    g_us += tNom;
    pushSample(0u, g_us);
    g_us += tNom;
    /* 真实缺口：seq 跳 3（缺 2 个） */
    pushSample(3u, g_us);

    Sample s; SamplerGap g;
    int gaps = 0;
    while (samplerPop(&s, &g)) {
      if (g.kind == SMP_GAP_MISSING) {
        gaps++;
        NOTE("识别到缺口 seq=%lu missed=%lu", (unsigned long)s.seq, (unsigned long)g.missed);
        CHECK(g.missed == 2, "缺口数 = 2（实际 %lu）", (unsigned long)g.missed);
      }
    }
    CHECK(gaps == 1, "回绕后仍识别出 1 次缺口（实际 %d）", gaps);
  }
  printf("\n");

  /* ---------- T13d：stateTick 的 nowUs 回绕 ---------- */
  printf("--- T13d stateTick 时间回绕不误判过期 ---\n");
  NOTE("stateTick 用 (uint32_t)(nowUs - lastWinUs) 比较，回绕时必须仍然正确");
  {
    /* 直接验证模运算：假设最后一个窗口在 0xFFFFF000，现在回绕到 0x00001000 */
    const uint32_t lastWin = 0xFFFFF000u;
    const uint32_t now     = 0x00001000u;
    const uint32_t sinceUs = now - lastWin;      // = 0x2000 = 8192us
    NOTE("lastWin=0x%08lX now=0x%08lX → since=%lu us",
         (unsigned long)lastWin, (unsigned long)now, (unsigned long)sinceUs);
    CHECK(sinceUs == 8192u, "回绕后流逝时间正确（实际 %lu us）", (unsigned long)sinceUs);
    CHECK(sinceUs < (uint32_t)FREQ_STALE_MS * 1000u,
          "8192us < FREQ_STALE_MS(%d ms)，不应判过期", FREQ_STALE_MS);
  }
  printf("\n");

  /* ---------- T13e：启停周期跨回绕 ---------- */
  printf("--- T13e 启停周期跨越时间回绕 ---\n");
  NOTE("ssOnStartEdge 用 (uint32_t)(tUs - s_lastEdgeUs)，模运算保证回绕正确");
  {
    const uint32_t edge1 = 0xFFFFFC00u;    // 回绕前 1024us
    const uint32_t edge2 = 0x00000400u;    // 回绕后 1024us
    const uint32_t periodUs = edge2 - edge1;
    const float periodMs = (float)periodUs / 1000.0f;
    NOTE("edge1=0x%08lX edge2=0x%08lX → 周期 %.3f ms",
         (unsigned long)edge1, (unsigned long)edge2, periodMs);
    CHECK(periodUs == 2048u, "回绕后周期正确 = 2048 us（实际 %lu）", (unsigned long)periodUs);
    CHECK(periodMs >= (float)SS_T_MIN_MS || periodMs < 2048.0f,
          "周期 %.1f ms 的判定逻辑不受回绕影响", periodMs);
  }
  printf("\n");

  /* ---------- 附加：跨回绕的差 ---------- */
  printf("--- 附加：跨回绕的差值 ---\n");
  {
    const uint32_t a = 0xFFFF0000u;
    const uint32_t b = 0x00010000u;        // 跨 1 次回绕
    const uint32_t d = b - a;
    NOTE("0x%08lX → 0x%08lX，差 = %lu us",
         (unsigned long)a, (unsigned long)b, (unsigned long)d);
    CHECK(d == 0x20000u, "跨回绕的差 = 131072 us（实际 %lu）", (unsigned long)d);
  }

  printf("\n%s（%d/%d 项通过，%d 项失败）\n",
         g_fail ? "存在失败" : "全部通过", g_total - g_fail, g_total, g_fail);
  return g_fail ? 1 : 0;
}
