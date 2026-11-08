// vibmon v2.0 主程序：只负责模块编排与运行状态（运行 / 故障恢复 / 标定）。
// 模块说明见同目录 README.md。
#include "config.h"
#include "mpu.h"
#include "sampler.h"
#include "dsp.h"
#include "state.h"
#include "output.h"
#include "motor_test.h"     // 【临时调试】默认关闭（MOTOR_TEST_ENABLE=0）

static bool     s_fault = false;
static uint32_t s_lastRecoverMs = 0;
static bool     s_calNeedsRetry = false;   // 从未标定成功过 → 允许自动重试
static uint32_t s_lastCalTryMs  = 0;

/* 主循环计时：标定/命令等重活期间置位，避免污染 loopUsMax */
static bool     s_ignoreLoopStats = false;
static bool     s_loopStatsResetPending = false;

static void enterFault(const char* why) {
  SamplerStopResult st = samplerStop();
  s_fault = true;
  s_lastRecoverMs = millis();
  stateSetSensorOk(false);
  ledSet(LED_FAULT);
  outPrintf("[FAULT] %s，结果已清零，每 %d ms 尝试重连\n", why, RECOVER_PERIOD_MS);
  if (samplerStopIsError(st)) {
    // 回调仍在跑：本轮不做总线重配，等下一轮
    outPrintf("[FAULT] %s，跳过本轮总线重配\n", samplerStopStr(st));
  }
}

static bool bringUpSensor(void) {
  mpuBusBegin();
  uint8_t bad = 0;
  MpuInitResult r = mpuInit(&bad);
  if (r != MPU_OK) {
    outPrintf("[MPU] 初始化失败：%s (寄存器 0x%02X)\n", mpuResultStr(r), bad);
    return false;
  }
  outPrintf("[MPU] 就绪，配置寄存器回读一致\n");
  return true;
}

static bool startRun(void) {
  if (!samplerReset()) {
    enterFault("采样器状态未清，拒绝启动");
    return false;
  }
  dspReset();
  stateReset();
  stateSetSensorOk(true);

  SamplerStartResult r = samplerStart();
  if (samplerStartIsError(r)) {
    stateSetSensorOk(false);
    enterFault(samplerStartStr(r));
    return false;
  }
  ledSet(LED_RUN);
  s_fault = false;
  return true;
}

static void runCalibration(void) {
  s_ignoreLoopStats = true;          // 标定 3.2s 不能污染 loopUsMax
  ledSet(LED_CAL);
  outPrintf("[CAL] 开始标定 %d ms，请保持模块静置\n", CAL_BIAS_MS + CAL_NOISE_MS);
  bool ok = dspCalibrate();
  if (ok) stateApplyCal(dspCal());   // 只有成功才应用阈值，失败时阈值原封不动
  outPrintCal();
  if (!ok) {
    outPrintf("[CAL] 未提交：%s；沿用%s\n",
              calResultStr(dspCalLastResult()),
              dspCal().ok ? "上次有效标定" : "编译期默认参数（未标定）");
  }
  s_ignoreLoopStats = false;
  s_loopStatsResetPending = true;    // 下一轮重置计时基准，避免把标定耗时算进去
}

void setup() {
  outBegin();
  delay(300);
  outPrintBanner();
#if MOTOR_TEST_ENABLE
  motorTestBegin();       // 【临时调试】同板驱动马达
#endif
  dspInit();

  if (!bringUpSensor()) { enterFault("传感器初始化失败"); return; }
  runCalibration();
  // 从未标定成功：允许带标记运行，并定时重试（已有旧标定时绝不重试）
  if (!dspCal().ok) { s_calNeedsRetry = true; s_lastCalTryMs = millis(); }
  startRun();
}

void loop() {
  /* ---- 主循环计时：测的是"上一轮 loop() 从头到尾"的耗时 ---- */
  static uint32_t s_lastLoopUs = 0;
  static bool     s_firstLoop  = true;
  const uint32_t loopStartUs = (uint32_t)micros();
  if (s_loopStatsResetPending) {
    // 上一轮干了重活（标定/命令），重置基准，别把它的耗时算进本轮
    s_lastLoopUs = loopStartUs;
    s_firstLoop  = false;
    s_loopStatsResetPending = false;
  } else if (!s_firstLoop) {
    if (!s_ignoreLoopStats) samplerNoteLoopUs(loopStartUs - s_lastLoopUs);
    s_lastLoopUs = loopStartUs;
  } else {
    s_lastLoopUs = loopStartUs;
    s_firstLoop = false;
  }

  uint32_t nowMs = millis();
  bool didHeavy = false;      // 本轮是否做了会影响计时的重活

  if (s_fault) {
    if (nowMs - s_lastRecoverMs >= RECOVER_PERIOD_MS) {
      s_lastRecoverMs = nowMs;
      // 回调没退出前不能碰总线：Wire 不是线程安全的
      if (samplerInFlight() != 0) {
        outPrintf("[FAULT] 采样回调仍未退出，继续等待\n");
      } else if (bringUpSensor()) {
        // 首次上电就失败时还没有标定，恢复后补做；运行中掉线则沿用原标定
        if (!dspCal().ok) { didHeavy = true; runCalibration(); }
        startRun();
        outPrintf("[OK] 传感器已恢复\n");
      }
    }
  } else {
    Sample s;
    SamplerGap gap;
    DspFrame f;
    while (samplerPop(&s, &gap)) {
      if (gap.kind != SMP_GAP_NONE) {
        dspOnGap(gap);
        stateOnGap(gap);        // 必须在 stateOnFrame 之前：先作废旧读数
      }
      dspProcess(s, &f);
      stateOnFrame(f, s.tUs);
      const uint32_t tOut = (uint32_t)micros();
      outOnSample(f);
      samplerNoteOutUs((uint32_t)micros() - tOut);
    }
    stateTick((uint32_t)micros());
    stateTickTouchRef((uint32_t)micros());   // 参考点采集结算（非阻塞）
    if (samplerSensorLost()) enterFault("传感器无响应");
  }

  // 无有效标定时定时重试；一旦成功就永久停止。
  // 不打扰已有有效标定的情况：否则一次失败会每 5s 打断测量、每次静默 3.2s。
  if (!s_fault && s_calNeedsRetry && !dspCal().ok &&
      nowMs - s_lastCalTryMs >= (uint32_t)CAL_RETRY_MS) {
    s_lastCalTryMs = nowMs;
    outPrintf("[CAL] 无有效标定，自动重试（请保持静置）\n");
    runCalibration();
    if (dspCal().ok) s_calNeedsRetry = false;
    startRun();
    didHeavy = true;
  }

  if (outPollCmd() == CMD_RECAL && !s_fault) {
    runCalibration();
    if (dspCal().ok) s_calNeedsRetry = false;
    startRun();
    didHeavy = true;
  }

  ledUpdate(nowMs, stateResult());
  outPeriodic(nowMs);
  compPeriodic(nowMs);      // COMP_FRAME_ENABLED=0 时编译为空
#if MOTOR_TEST_ENABLE
  motorTestTick(nowMs);     // 【临时调试】推进马达扫频
#endif

  // 本轮回合做了重活：下一轮重置基准，避免把重活耗时计入 loopUs
  if (didHeavy) s_loopStatsResetPending = true;
}
