// 采样器可靠性测试（A2 / Q2）：T14 定时器失败、T04 停止握手。
//
// 编译：
//   g++ -std=c++17 -Wall -DSMP_TEST_HOOKS -DSMP_STOP_TIMEOUT_MS=5
//       -I test -I . test/sampler_test.cpp test/esp_timer_stub.cpp sampler.cpp
//       -o test/sampler_test.exe
//
// 设计要点：
//   * 用 SMP_TEST_HOOKS 暴露的 tickBegin/tickEnd 手动驱动回调，
//     才能在单线程里造出"回调正在执行（inFlight=1）"的状态。
//   * delay/delayMicroseconds 必须推进虚拟时间，否则 samplerStop() 死循环。
//   * 断言分成"错误态"和"幂等态"两类，对应 v3 的 IsError() 语义。
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Arduino.h"
#include "sampler.h"
#include "config.h"
#include "esp_timer.h"

/* 接口在实现落地前先用兜底声明，保证旧代码也能编译出"红" */
#ifndef SMP_STOP_TIMEOUT_MS
#define SMP_STOP_TIMEOUT_MS 15
#endif
#ifndef SMP_TEST_HOOKS
#error "sampler_test 需要 -DSMP_TEST_HOOKS"
#endif

/* ================= 虚拟时间 ================= */
static uint32_t g_us = 0;
uint32_t millis(void) { return g_us / 1000u; }
uint32_t micros(void) { return g_us; }
void delay(uint32_t ms) { g_us += ms * 1000u; }
void delayMicroseconds(uint32_t us) { g_us += us; }

/* ================= MPU 桩 ================= */
static int  g_readCalls = 0;
static bool g_readFail  = false;
static float g_ax = 0, g_ay = 0, g_az = 1000;

bool mpuReadAccel(MpuRaw* o) {
  g_readCalls++;
  if (g_readFail) return false;
  o->x = (int16_t)(g_ax * 16.384f);
  o->y = (int16_t)(g_ay * 16.384f);
  o->z = (int16_t)(g_az * 16.384f);
  return true;
}
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

/* 每次用例前重置桩与采样器 */
static void fresh(void) {
  probe_reset();
  g_us = 0;
  g_readCalls = 0;
  g_readFail = false;
  g_ax = 0; g_ay = 0; g_az = 1000;
  /* 确保处于停止态 */
  (void)samplerStop();
  (void)samplerReset();
  samplerClearStats();
}

int main(void) {
  printf("=== sampler_test：A2 + Q2 采样器可靠性 ===\n");
  printf("SMP_STOP_TIMEOUT_MS=%d  FS_HZ=%d  RING_N=%d\n\n",
         SMP_STOP_TIMEOUT_MS, FS_HZ, RING_N);

  uint32_t epoch, seq;
  Sample s; SamplerGap gap;

  /* ================= T14：定时器失败 ================= */
  printf("--- T14a 创建失败 ---\n");
  fresh();
  probe_failCreate = true;
  {
    SamplerStartResult r = samplerStart();
    CHECK(r == SMP_START_ERR_CREATE, "返回 ERR_CREATE（实际 %s）", samplerStartStr(r));
    CHECK(samplerStartIsError(r), "IsError() 为 true");
    CHECK(!samplerPop(&s, &gap), "pop 恒 false（无假运行）");
    CHECK(!probe_anyRunning(), "定时器确实没在跑");
  }
  printf("\n");

  printf("--- T14b 启动失败 ---\n");
  fresh();
  probe_failStart = true;
  {
    SamplerStartResult r = samplerStart();
    CHECK(r == SMP_START_ERR_START, "返回 ERR_START（实际 %s）", samplerStartStr(r));
    CHECK(samplerStartIsError(r), "IsError() 为 true");
    CHECK(!samplerPop(&s, &gap), "pop 恒 false（accept 未被误开）");

    probe_failStart = false;
    SamplerStartResult r2 = samplerStart();
    CHECK(r2 == SMP_START_OK, "清除注入后可正常启动（实际 %s）", samplerStartStr(r2));
  }
  printf("\n");

  printf("--- T14c 重复启动不算错误 ---\n");
  fresh();
  {
    SamplerStartResult r1 = samplerStart();
    SamplerStartResult r2 = samplerStart();
    CHECK(r1 == SMP_START_OK, "首次 OK");
    CHECK(r2 == SMP_START_ALREADY_RUNNING, "第二次 ALREADY_RUNNING（实际 %s）",
          samplerStartStr(r2));
    CHECK(!samplerStartIsError(r2), "IsError() 为 false（不能当故障）");
  }
  printf("\n");

  printf("--- T14d stop 返回值异常被记录 ---\n");
  fresh();
  {
    samplerStart();
    probe_failStop = true;
    samplerStop();
    const SamplerStats& st = samplerStats();
    CHECK(st.timerErr > 0, "timerErr = %lu > 0", (unsigned long)st.timerErr);
    CHECK(st.timerErrCode != 0, "timerErrCode = 0x%02X 非 0", (unsigned)st.timerErrCode);
    probe_failStop = false;
    (void)samplerStop();
    (void)samplerReset();
  }
  printf("\n");

  /* ================= T04：停止握手 ================= */
  printf("--- T04a 回调未退出 → 超时 ---\n");
  fresh();
  {
    samplerStart();
    bool began = samplerTestTickBegin(&epoch, &seq);
    CHECK(began, "tickBegin 成功（inFlight=1）");
    CHECK(samplerInFlight() == 1, "inFlight == 1");

    SamplerStopResult r = samplerStop();     // 不调 tickEnd，模拟回调卡住
    CHECK(r == SMP_STOP_TIMEOUT, "返回 TIMEOUT（实际 %s）", samplerStopStr(r));
    CHECK(samplerStopIsError(r), "IsError() 为 true");
    CHECK(samplerStats().stopTimeout > 0, "stopTimeout = %lu > 0",
          (unsigned long)samplerStats().stopTimeout);
    CHECK(samplerInFlight() == 1, "inFlight 仍为 1（没有假清零）");

    /* 收尾：让回调退出 */
    samplerTestTickEnd(true, epoch, seq, MpuRaw{0, 0, 16384}, 0);
    CHECK(samplerInFlight() == 0, "回调退出后 inFlight == 0");
  }
  printf("\n");

  printf("--- T04b 正常停止 ---\n");
  fresh();
  {
    samplerStart();
    probe_fire();
    SamplerStopResult r = samplerStop();
    CHECK(r == SMP_STOP_OK, "返回 OK（实际 %s）", samplerStopStr(r));
    CHECK(!samplerStopIsError(r), "IsError() 为 false");
    CHECK(samplerInFlight() == 0, "inFlight == 0");
    CHECK(samplerReset(), "reset 返回 true（可安全清空）");
  }
  printf("\n");

  printf("--- T04c 超时后 reset 被拒绝 ---\n");
  fresh();
  {
    samplerStart();
    samplerTestTickBegin(&epoch, &seq);
    samplerStop();                            // 超时
    uint32_t violBefore = samplerStats().resetViolation;

    CHECK(!samplerReset(), "reset 返回 false（明确告知没清成）");
    CHECK(samplerStats().resetViolation > violBefore,
          "resetViolation 递增到 %lu", (unsigned long)samplerStats().resetViolation);

    samplerTestTickEnd(true, epoch, seq, MpuRaw{0, 0, 16384}, 0);
    CHECK(samplerReset(), "回调退出后 reset 才返回 true");
  }
  printf("\n");

  printf("--- T04d 跨代陈旧回调被丢弃 ---\n");
  fresh();
  {
    samplerStart();
    uint32_t e1, s1;
    CHECK(samplerTestTickBegin(&e1, &s1), "第一代回调进入");
    samplerStop();                            // 超时，e1 未完成
    /* 重启：epoch 递增 */
    samplerStart();
    uint32_t e2 = samplerTestEpoch();
    CHECK(e2 != e1, "重启后 epoch 变化（%lu → %lu）", (unsigned long)e1, (unsigned long)e2);

    /* 旧回调现在才退出，带着第一代的 epoch */
    bool accepted = samplerTestTickEnd(true, e1, s1, MpuRaw{1, 2, 3}, 0);
    CHECK(!accepted, "陈旧回调被拒绝入队");
    CHECK(!samplerPop(&s, &gap), "缓冲里没有陈旧样本");
  }
  printf("\n");

  printf("--- T04e 反复 stop/start 50 次 ---\n");
  fresh();
  {
    int bad = 0;
    for (int i = 0; i < 50; i++) {
      if (samplerStart() != SMP_START_OK) { bad++; break; }
      probe_fire();
      if (samplerStop() != SMP_STOP_OK) { bad++; break; }
      if (!samplerReset()) { bad++; break; }
    }
    CHECK(bad == 0, "50 轮全部干净完成");
    CHECK(samplerStats().resetViolation == 0,
          "resetViolation == 0（实际 %lu）", (unsigned long)samplerStats().resetViolation);
    CHECK(samplerStats().stopTimeout == 0,
          "stopTimeout == 0（实际 %lu）", (unsigned long)samplerStats().stopTimeout);
  }
  printf("\n");

  printf("--- T04f 运行中 stop 返回 OK ---\n");
  fresh();
  {
    samplerStart();
    SamplerStopResult r = samplerStop();
    CHECK(r == SMP_STOP_OK, "返回 OK 而非 ALREADY_STOPPED（实际 %s）", samplerStopStr(r));
  }
  printf("\n");

  printf("--- T04g 未启动就 stop ---\n");
  fresh();
  {
    SamplerStopResult r = samplerStop();
    CHECK(r == SMP_STOP_ALREADY_STOPPED, "返回 ALREADY_STOPPED（实际 %s）", samplerStopStr(r));
    CHECK(!samplerStopIsError(r), "IsError() 为 false（幂等，不是故障）");
  }
  printf("\n");

  printf("--- T04h 停止后 tickBegin 必须被拒（防 inFlight 下溢）---\n");
  fresh();
  {
    samplerStart();
    samplerStop();
    uint32_t before = samplerInFlight();
    bool began = samplerTestTickBegin(&epoch, &seq);
    CHECK(!began, "tickBegin 返回 false");
    CHECK(samplerInFlight() == before, "inFlight 未被改动（仍为 %lu）",
          (unsigned long)samplerInFlight());
    CHECK(samplerInFlight() == 0, "inFlight == 0，无下溢");
  }
  printf("\n");

  /* ================= 缓冲与统计 ================= */
  printf("--- 附加：样本确实入队且 gap 正确 ---\n");
  fresh();
  {
    samplerStart();
    uint32_t e, q;
    for (int i = 0; i < 5; i++) {
      samplerTestTickBegin(&e, &q);
      samplerTestTickEnd(true, e, q, MpuRaw{(int16_t)i, 0, 16384}, 0);
    }
    int n = 0;
    while (samplerPop(&s, &gap)) n++;
    CHECK(n == 5, "pop 出 %d 个样本（应为 5）", n);
    samplerStop();
    samplerReset();
  }

  printf("\n%s（%d/%d 项通过，%d 项失败）\n",
         g_fail ? "存在失败" : "全部通过", g_total - g_fail, g_total, g_fail);
  return g_fail ? 1 : 0;
}
