// vibmon v2.0 主程序：只负责模块编排与运行状态（运行 / 故障恢复 / 标定）。
// 模块说明见同目录 README.md。
//
// 【本文件为什么这么薄】所有实际工作都在各模块里（mpu 读硬件、sampler 定速采集、
// dsp 算数值、state 下判断、output 打印）。这里只做三件事：
//   ① 上电初始化顺序编排（setup）
//   ② 每圈把样本从采样器搬到 dsp → state → output（loop 的正常分支）
//   ③ 管理两个"模式"：正常运行的互斥状态 与 故障恢复
// 保持这个文件薄，是为了让"改了行为"和"改了算法"能一眼分开。
#include "config.h"
#include "mpu.h"
#include "sampler.h"
#include "dsp.h"
#include "state.h"
#include "output.h"

/* ---------- 运行状态 ---------- */
static bool     s_fault = false;          // 是否处于故障态。true 时 loop 不走正常处理链，只尝试重连
static uint32_t s_lastRecoverMs = 0;      // 上次尝试重连的时刻，用于按 RECOVER_PERIOD_MS 限流

// 进入故障态。四件事必须一起做，缺一样都会留下不一致的状态：
//   ① 停采样器——传感器已不可信，继续采只会往缓冲里灌垃圾数据
//   ② 置故障标志——让 loop 切到重连分支
//   ③ 记录重连计时起点，并告知 state 传感器已掉线（state 会因此清空全部结果）
//   ④ 切 LED 到故障模式并打印原因
// 【为什么要清结果】故障时保留旧读数比显示"无数据"更危险——用户会把它当成当前状态。
static void enterFault(const char* why) {
  samplerStop();
  s_fault = true;
  s_lastRecoverMs = millis();
  stateSetSensorOk(false);
  ledSet(LED_FAULT);
  outPrintf("[FAULT] %s，结果已清零，每 %d ms 尝试重连\n", why, RECOVER_PERIOD_MS);
}

// 上电/重连时的传感器初始化。
// 【为什么每次重连都重新初始化总线】传感器掉线常常伴随 I2C 总线被拉死
// （器件半死状态把 SDA 持续拉低），单纯重读寄存器是读不出来的，必须重建总线。
// 返回 true 仅表示初始化成功，不涉及采样器状态（由调用方决定）。
static bool bringUpSensor(void) {
  mpuBusBegin();
  uint8_t bad = 0;
  MpuInitResult r = mpuInit(&bad);
  if (r != MPU_OK) {
    // bad 给出出错的寄存器地址，配合 mpuResultStr 的中文说明可直接定位问题
    outPrintf("[MPU] 初始化失败：%s (寄存器 0x%02X)\n", mpuResultStr(r), bad);
    return false;
  }
  outPrintf("[MPU] 就绪，配置寄存器回读一致\n");
  return true;
}

// 启动一次正常运行。顺序有讲究，不能随意调换：
//   ① 复位采样器/dsp/state —— 清掉上一轮（可能是故障前）的残留数据与滤波器状态，
//      否则恢复后最初几个样本会混入旧状态，产生一段虚假读数
//   ② 告知 state 传感器已在线（必须在 stateReset 之后：stateSetSensorOk 在
//      状态变化为 false 时会触发一次 stateReset，顺序反了会把复位结果覆盖掉）
//   ③ 切 LED，清故障标志
//   ④ 最后才启动采样器——此时下游已全部就绪，不会有样本被漏处理
static void startRun(void) {
  samplerReset();
  dspReset();
  stateReset();
  stateSetSensorOk(true);
  ledSet(LED_RUN);
  s_fault = false;
  samplerStart();
}

// 执行一次标定。
// 【流程】dspCalibrate 内部会自行停止并复位采样器，返回时采样器处于停止状态；
// 因此这里只需：切 LED → 提示用户静置 → 标定 → 把结果应用到 state 的阈值 → 打印结果。
// 【标定失败怎么办】沿用上一次（或默认）阈值继续运行，不阻断流程。
// 提示用户可用 'c' 命令重做，而不是让设备停在那里等。
static void runCalibration(void) {
  ledSet(LED_CAL);
  outPrintf("[CAL] 开始标定 %d ms，请保持模块静置\n", CAL_BIAS_MS + CAL_NOISE_MS);
  bool ok = dspCalibrate();
  stateApplyCal(dspCal());          // 无论成败都调用：stateApplyCal 内部会在 !ok 时保留原阈值
  outPrintCal(dspCal());
  if (!ok) outPrintf("[CAL] 未完成，沿用默认/上次阈值；可发送 'c' 重做\n");
}

// 上电初始化。
// 【顺序推演】
//   ① outBegin  —— 最先初始化串口，后面所有步骤的报错才有地方输出
//   ② delay(300)—— 等 USB-CDC 枚举完成。不等的话最初的几行输出会丢失
//   ③ 横幅 + dspInit —— 打印配置便于核对固件，并算好滤波器系数
//   ④ 传感器初始化 —— 失败直接进故障态并 return（此时采样器还没启动，
//      进入故障态是安全的；若标定后才发现问题，状态会更复杂）
//   ⑤ 标定 —— 必须在启动采样器之前做：标定自己会控制采样器的启停，
//      若此时采样器已在运行，两边会争抢 I2C
//   ⑥ 启动运行
void setup() {
  outBegin();
  delay(300);
  outPrintBanner();
  dspInit();

  if (!bringUpSensor()) { enterFault("传感器初始化失败"); return; }
  runCalibration();
  startRun();
}

// 主循环。分两条互斥路径：
//
// 【故障路径】按 RECOVER_PERIOD_MS 限流重试：
//   到点 → 重新初始化传感器 → 成功则（首次上电就失败时还没有标定，恢复后补做；
//   运行中掉线则沿用原标定）→ 启动运行。失败则什么都不做，等下一个周期。
//
// 【正常路径】把采样器缓冲里的样本全部搬完：
//   samplerPop 取出样本 → 有缺样先通知 dsp（让它作废主频窗）→ dsp 算数值
//   → state 更新判断 → output 输出波形（仅波形组）
//   搬完后：stateTick 做一次超时检查（用 micros 而不是样本时刻，
//   因为"没有样本"正是需要检查超时的情形）；再检查传感器是否掉线。
//
// 【为什么样本要 while 全部搬完】采样器 1kHz 生产、主循环节奏不定，
// 若每圈只处理一个样本，积压会迅速撑满环形缓冲导致丢样。
void loop() {
  uint32_t nowMs = millis();

  if (s_fault) {
    if (nowMs - s_lastRecoverMs >= RECOVER_PERIOD_MS) {
      s_lastRecoverMs = nowMs;
      if (bringUpSensor()) {
        // 首次上电就失败时还没有标定，恢复后补做；运行中掉线则沿用原标定
        if (!dspCal().ok) runCalibration();
        startRun();
        outPrintf("[OK] 传感器已恢复\n");
      }
    }
  } else {
    Sample s;
    uint32_t gap;
    DspFrame f;
    while (samplerPop(&s, &gap)) {
      if (gap) dspOnGap(gap);       // 时间轴不连续：作废当前主频窗
      dspProcess(s, &f);
      stateOnFrame(f, s.tUs);
      outOnSample(f);               // 仅波形组下真正输出
    }
    stateTick((uint32_t)micros());  // 时效检查（即使没有新样本也要跑）
    if (samplerSensorLost()) enterFault("传感器无响应");
  }

  // 串口命令：重标定需要主循环统一编排，所以放在这里处理。
  // 故障态下忽略该命令——传感器都读不到，标定没有意义。
  if (outPollCmd() == CMD_RECAL && !s_fault) {
    runCalibration();
    startRun();                     // 标定会停掉采样器，必须重新启动
  }

  ledUpdate(nowMs, stateResult());
  outPeriodic(nowMs);
}
