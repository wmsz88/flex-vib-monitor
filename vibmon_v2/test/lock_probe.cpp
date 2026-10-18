// 锁配对运行时检查：在主机测试里给 SMP_LOCK/SMP_UNLOCK 加计数，
// 直接验证每次加锁都有对应的解锁（包括异常返回路径）。
//
// 编译：
//   g++ -std=c++17 -Wall -DSMP_TEST_HOOKS -DSMP_STOP_TIMEOUT_MS=5
//       -DSMP_LOCK_PROBE -I test -I . test/lock_probe.cpp test/esp_timer_stub.cpp
//       sampler.cpp -o test/lock_probe.exe
//
// 原理：sampler.cpp 在主机测试下 SMP_LOCK 是空宏。这里改成"计数+断言深度"，
// 任何一处漏解锁都会让深度不为 0，从而被抓到。
#include <stdio.h>
#include <stdlib.h>
#include "Arduino.h"
#include "sampler.h"
#include "config.h"
#include "esp_timer.h"

/* 覆盖 sampler.cpp 里的空宏 —— 通过 SMP_LOCK_PROBE 让 sampler.cpp 引用外部计数 */
extern int g_lockDepth;
extern int g_lockMax;
extern int g_lockUnbalanced;

/* ================= 虚拟时间 ================= */
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

static int g_fail = 0, g_total = 0;
#define CHECK(cond, ...) do {                                   \
  g_total++;                                                    \
  if (!(cond)) { g_fail++; printf("[FAIL] "); }                 \
  else          { printf("[PASS] "); }                          \
  printf(__VA_ARGS__); printf("\n");                            \
} while (0)

static void fresh(void) {
  probe_reset();
  g_us = 0;
  (void)samplerStop();
  (void)samplerReset();
  samplerClearStats();
  g_lockDepth = 0;
  g_lockMax = 0;
  g_lockUnbalanced = 0;
}

int main(void) {
  printf("=== lock_probe：锁配对运行时检查 ===\n\n");

  uint32_t epoch, seq;

  printf("--- 路径1：正常 start / fire / stop / reset ---\n");
  fresh();
  {
    samplerStart();
    probe_fire();
    samplerStop();
    samplerReset();
    CHECK(g_lockDepth == 0, "锁深度归零（实际 %d）", g_lockDepth);
    CHECK(g_lockUnbalanced == 0, "无不配对（实际 %d）", g_lockUnbalanced);
    CHECK(g_lockMax > 0, "确实进过临界区（最大深度 %d）", g_lockMax);
  }

  printf("\n--- 路径2：start 已运行（提前返回分支）---\n");
  fresh();
  {
    samplerStart();
    samplerStart();          // ALREADY_RUNNING 分支
    samplerStop();
    samplerReset();
    CHECK(g_lockDepth == 0, "锁深度归零（实际 %d）", g_lockDepth);
    CHECK(g_lockUnbalanced == 0, "无不配对（实际 %d）", g_lockUnbalanced);
  }

  printf("\n--- 路径3：stop 未启动（提前返回分支）---\n");
  fresh();
  {
    samplerStop();           // ALREADY_STOPPED 分支
    CHECK(g_lockDepth == 0, "锁深度归零（实际 %d）", g_lockDepth);
    CHECK(g_lockUnbalanced == 0, "无不配对（实际 %d）", g_lockUnbalanced);
  }

  printf("\n--- 路径4：reset 被拒（提前返回分支）---\n");
  fresh();
  {
    samplerStart();
    samplerTestTickBegin(&epoch, &seq);   // inFlight=1
    samplerStop();                        // 超时
    samplerReset();                       // 被拒分支
    CHECK(g_lockDepth == 0, "锁深度归零（实际 %d）", g_lockDepth);
    CHECK(g_lockUnbalanced == 0, "无不配对（实际 %d）", g_lockUnbalanced);
    samplerTestTickEnd(true, epoch, seq, MpuRaw{0,0,16384}, 0);
  }

  printf("\n--- 路径5：tickBegin 被拒（accept=false）---\n");
  fresh();
  {
    samplerTestTickBegin(&epoch, &seq);   // 未启动 → accept=false → 拒绝分支
    CHECK(g_lockDepth == 0, "锁深度归零（实际 %d）", g_lockDepth);
    CHECK(g_lockUnbalanced == 0, "无不配对（实际 %d）", g_lockUnbalanced);
  }

  printf("\n--- 路径6：tickEnd 跨代丢弃（提前返回分支）---\n");
  fresh();
  {
    samplerStart();
    uint32_t e1, s1;
    samplerTestTickBegin(&e1, &s1);
    samplerStop();
    samplerStart();                        // 换代
    samplerTestTickEnd(true, e1, s1, MpuRaw{0,0,16384}, 0);   // 跨代 → 丢弃分支
    CHECK(g_lockDepth == 0, "锁深度归零（实际 %d）", g_lockDepth);
    CHECK(g_lockUnbalanced == 0, "无不配对（实际 %d）", g_lockUnbalanced);
    samplerStop();
    samplerReset();
  }

  printf("\n--- 路径7：pop 空缓冲（提前返回分支）---\n");
  fresh();
  {
    Sample s; SamplerGap gap;
    samplerPop(&s, &gap);                  // 空 → 提前返回
    CHECK(g_lockDepth == 0, "锁深度归零（实际 %d）", g_lockDepth);
    CHECK(g_lockUnbalanced == 0, "无不配对（实际 %d）", g_lockUnbalanced);
  }

  printf("\n%s（%d/%d 项通过，%d 项失败）\n",
         g_fail ? "存在失败" : "全部通过", g_total - g_fail, g_total, g_fail);
  return g_fail ? 1 : 0;
}
