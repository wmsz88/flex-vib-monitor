#line 1 "D:\\大学\\课程\\工科赛\\代码\\vibmon_v5\\sampler.h"
// 采样模块：esp_timer 定速采集、样本序号与时间戳、环形缓冲、丢样与断线统计。
//
// 【并发模型】
//   生产者：esp_timer 任务上下文里的 onTick（经 tickBegin/tickEnd 两半）
//   消费者：loop() 里的 samplerPop
//   共享状态（head/tail/seq/inFlight/epoch/统计）统一由 s_mux 保护。
//   s_prevSeq/s_prevUs/s_havePrev 只被 samplerPop（消费者）与 samplerReset
//   （要求已停止）访问，因此不进锁 —— 改动这两个前提时必须重新评估。
//
// 【启动/停止合约】
//   samplerStart() 返回四态，其中 ALREADY_RUNNING 不是错误。
//   samplerStop() 返回三态，其中 ALREADY_STOPPED 不是错误；TIMEOUT 是错误，
//   表示回调未在 SMP_STOP_TIMEOUT_MS 内退出，此时调用方**不得** reset 或重配总线。
//   samplerReset() 返回 bool，false 表示运行中或回调未退，缓冲区未被清空。
#pragma once
#include <stdint.h>
#include "mpu.h"

struct Sample {
  uint32_t seq;      // 采样节拍序号（读失败的节拍也计数，用于发现缺样）
  uint32_t tUs;      // 读取完成时刻 micros()
  MpuRaw   raw;
  bool     gyroValid = false;  // v5：本样本是否带有效陀螺读数（每 GYRO_DIV 个 tick 一次）
};

struct SamplerStats {
  // 第 1 步
  uint32_t readFail;       // I2C 读失败的节拍数
  uint32_t ringDrop;       // 缓冲满被丢弃的样本数
  uint32_t gaps;           // 不连续总次数（= missing 次数 + jitter 次数）
  uint32_t missed;         // 累计缺失样本数
  uint32_t clip;           // 削顶样本数（由信号处理模块累加）
  // 第 2 步
  uint32_t resetViolation; // 运行中调 samplerReset 被拒的次数
  uint32_t stopTimeout;    // 停止握手超时次数
  uint32_t timerErr;       // esp_timer_* 返回异常的累计次数
  uint8_t  timerErrCode;   // 最近一次 esp_err_t 的低 8 位（排障用）
  // 第 3 步
  uint32_t jitter;         // 时间抖动次数（序号连续但间隔异常）
};

// 缺口类型：两者对下游的意义不同，不能混为一谈
enum SamplerGapKind : uint8_t {
  SMP_GAP_NONE = 0,      // 连续
  SMP_GAP_MISSING,       // 序号断开：真的丢了样本，中间数据永久缺失
  SMP_GAP_JITTER,        // 序号连续但间隔偏离：样本都在，只是时间轴被拉伸/压缩
};

// 一次不连续的完整描述
struct SamplerGap {
  SamplerGapKind kind;
  uint32_t       missed;   // MISSING: 缺失样本数；JITTER: 0
  uint32_t       spanUs;   // 距上一个样本的真实时间跨度
  int32_t        devUs;    // JITTER: 间隔偏差（微秒）；MISSING: 0
};

enum SamplerStartResult : uint8_t {
  SMP_START_OK = 0,
  SMP_START_ALREADY_RUNNING,   // 非错误：本来就在跑
  SMP_START_ERR_CREATE,        // 错误：esp_timer_create 失败
  SMP_START_ERR_START,         // 错误：esp_timer_start_periodic 失败
};

enum SamplerStopResult : uint8_t {
  SMP_STOP_OK = 0,
  SMP_STOP_ALREADY_STOPPED,    // 非错误：本来就没跑
  SMP_STOP_TIMEOUT,            // 错误：回调未在超时内退出
};

// 时序统计：用于定位"采样是否稳"的瓶颈，本身只做整数算术，不新增时间调用。
// 注意 ringHighWater 的语义是"每次成功入队时刻观测到的占用最大值"，
// 不是严格的"任意时刻最高水位"（消费者可能在两次入队之间取走样本）。
struct TimingStats {
  // 读耗时（微秒）—— 成功与失败都计入
  uint32_t readN;
  uint32_t readUsAvg;
  uint32_t readUsMax;       // 含失败（超时）的那次
  uint32_t readFailN;
  uint32_t readFailUsMax;   // 只统计失败的那些

  // 样本间隔（微秒）—— 只统计连续样本，缺口不进来
  uint32_t spanN;
  uint32_t spanUsAvg;
  uint32_t spanUsMax;
  uint32_t spanUsMin;       // 初始 0xFFFFFFFF

  // 缺口间隔（微秒）—— 单独统计，避免污染 span
  uint32_t gapSpanN;
  uint32_t gapSpanUsAvg;
  uint32_t gapSpanUsMax;

  // 主循环（微秒）—— 调用方负责排除标定/命令输出
  uint32_t loopN;
  uint32_t loopUsAvg;
  uint32_t loopUsMax;

  // 输出耗时（微秒）
  uint32_t outN;
  uint32_t outUsAvg;
  uint32_t outUsMax;

  // 缓冲水位：入队时刻占用的最大值（见上方语义说明）
  uint16_t ringHighWater;
};

SamplerStartResult samplerStart(void);
SamplerStopResult  samplerStop(void);
bool               samplerReset(void);   // false = 运行中/回调未退，未清空

// 取一个样本；*gap 描述它与上一个样本之间的不连续（kind == SMP_GAP_NONE 表示连续）
bool samplerPop(Sample* s, SamplerGap* gap);
bool samplerSensorLost(void);            // 连续读失败或长时间无新样本
const SamplerStats& samplerStats(void);
void samplerAddClip(void);
void samplerClearStats(void);

uint32_t samplerInFlight(void);          // 正在执行的回调数（故障恢复前查询）
const TimingStats& samplerTiming(void);
void samplerTimingReset(void);           // 'N' 命令：清零统计并开始新一轮测量
void samplerNoteLoopUs(uint32_t us);     // 由主循环调用（调用方负责排除重活）
void samplerNoteOutUs(uint32_t us);      // 由主循环包住 outOnSample 调用
bool     samplerStartIsError(SamplerStartResult r);
bool     samplerStopIsError(SamplerStopResult r);
const char* samplerStartStr(SamplerStartResult r);
const char* samplerStopStr(SamplerStopResult r);

/* ---- 测试钩子：正式固件不定义 SMP_TEST_HOOKS，以下不参与编译 ---- */
#ifdef SMP_TEST_HOOKS
// 返回 false 表示本次回调被拒绝（未递增 inFlight），调用方**不得**再调 tickEnd
bool     samplerTestTickBegin(uint32_t* epoch, uint32_t* seq);
// t0Us：回调开始时刻（onTick 里取的 micros），用于统计读耗时。
// 传 0 表示"不统计本次耗时"（测试用）。
// 返回值：样本是否真的入队（跨代/缓冲满时为 false）
bool     samplerTestTickEnd(bool ok, uint32_t epoch, uint32_t seq, const MpuRaw& r, uint32_t t0Us,
                            bool gyroValid = false);
uint32_t samplerTestEpoch(void);
#endif
