#include "sampler.h"
#include "config.h"
#include <Arduino.h>
#include <esp_timer.h>

/* ---------------- 临界区 ----------------
 * ESP32 上用 portENTER_CRITICAL（关中断，只放纯内存操作）；
 * 主机测试没有 FreeRTOS，退化为空宏（单线程无并发）。
 * 锁内禁止：I2C、millis()/micros()、日志。
 */
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
  static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
  #define SMP_LOCK()   portENTER_CRITICAL(&s_mux)
  #define SMP_UNLOCK() portEXIT_CRITICAL(&s_mux)
#elif defined(SMP_LOCK_PROBE)
  // 主机侧锁配对检查：用计数验证每次加锁都有对应解锁
  extern int g_lockDepth;
  extern int g_lockMax;
  extern int g_lockUnbalanced;
  #define SMP_LOCK()   do { g_lockDepth++; if (g_lockDepth > g_lockMax) g_lockMax = g_lockDepth; \
                            if (g_lockDepth > 1) g_lockUnbalanced++; } while (0)
  #define SMP_UNLOCK() do { g_lockDepth--; if (g_lockDepth < 0) g_lockUnbalanced++; } while (0)
#else
  #define SMP_LOCK()   ((void)0)
  #define SMP_UNLOCK() ((void)0)
#endif

// 共享状态：全部在 s_mux 保护下访问，因此**不需要 volatile**
// （volatile 只防优化，不提供原子性/内存序；加了反而触发 C++20 的 -Wvolatile 弃用警告）
static Sample            s_ring[RING_N];
static uint16_t          s_head = 0, s_tail = 0;
static uint32_t          s_tickSeq = 0;
static uint32_t          s_failRun = 0;     // 连续失败节拍数
static uint32_t          s_lastOkMs = 0;
static uint32_t          s_readFail = 0, s_ringDrop = 0;

static SamplerStats      s_stats = {};
static bool              s_havePrev = false;   // 仅消费者写
static uint32_t          s_prevSeq = 0, s_prevUs = 0;   // 仅消费者写
static esp_timer_handle_t s_timer = nullptr;
static bool              s_running = false;

/* 回调同步三件套（同样由 s_mux 保护） */
static bool     s_accept   = false;  // 是否允许入队
static uint32_t s_inFlight = 0;      // 正在执行的回调数（0 或 1）
static uint32_t s_epoch    = 0;      // 每次 start 递增，用于丢弃跨代回调

/* ---------------- 时序统计 ----------------
 * 只做整数算术，不新增时间函数调用（时间戳由调用方传入或复用已有的）。
 * EMA 在前 64 个样本用算术平均，避免初值 0 导致的长时间爬升。
 */
static TimingStats s_timing = {};

struct EmaState { uint32_t avg, sum; uint16_t n; };

static inline void emaStep(EmaState* e, uint32_t v) {
  if (e->n < 64) {
    e->sum += v;
    e->n++;
    e->avg = e->sum / e->n;          // 算术平均，第 64 个时精确
  } else {
    const int32_t d = (int32_t)v - (int32_t)e->avg;
    e->avg = (uint32_t)((int32_t)e->avg + (d >> 6));   // 有符号差值，避免回绕
  }
}

static EmaState s_emaRead  = {};
static EmaState s_emaSpan  = {};
static EmaState s_emaGap   = {};
static EmaState s_emaLoop  = {};
static EmaState s_emaOut   = {};

static void timingResetEma(void) {
  s_emaRead = EmaState{}; s_emaSpan = EmaState{};
  s_emaGap  = EmaState{}; s_emaLoop = EmaState{};
  s_emaOut  = EmaState{};
}

/* ---------------- 回调：拆成 begin / end 两半 ----------------
 * 拆开的目的是让"进入/退出"能被测试单独驱动，否则单线程主机测试
 * 永远造不出"回调正在执行（inFlight=1）"的状态。
 *
 * 关键不变量：tickBegin 返回 true 才允许调 tickEnd。
 * 若 tickBegin 返回 false（被 accept 拒绝），inFlight 未被递增，
 * 此时调 tickEnd 会让 inFlight 下溢 —— 所以 onTick 必须直接 return。
 */

static bool tickBegin(uint32_t* epoch, uint32_t* seq) {
  SMP_LOCK();
  if (!s_accept) { SMP_UNLOCK(); return false; }   // 未递增 inFlight
  *epoch = s_epoch;
  *seq   = s_tickSeq++;
  s_inFlight++;
  SMP_UNLOCK();
  return true;
}

static bool tickEnd(bool ok, uint32_t epoch, uint32_t seq, const MpuRaw& r, uint32_t t0Us) {
  // 时间戳在锁外取：临界区里只做纯内存操作
  const uint32_t nowMs = millis();
  const uint32_t nowUs = micros();
  const uint32_t readUs = (t0Us != 0) ? (nowUs - t0Us) : 0;

  SMP_LOCK();
  s_inFlight--;                              // tickBegin 成功过，必然成对

  // 读耗时：成功与失败都计入（失败那次的耗时恰恰最需要知道）
  if (t0Us != 0) {
    s_timing.readN++;
    emaStep(&s_emaRead, readUs);
    s_timing.readUsAvg = s_emaRead.avg;
    if (readUs > s_timing.readUsMax) s_timing.readUsMax = readUs;
    if (!ok) {
      s_timing.readFailN++;
      if (readUs > s_timing.readFailUsMax) s_timing.readFailUsMax = readUs;
    }
  }

  // 先判是否当前代：陈旧回调不应影响当前代的断线判断
  const bool current = (epoch == s_epoch) && s_accept;

  if (!ok) {
    s_readFail++;
    if (current) s_failRun++;
    SMP_UNLOCK();
    return false;
  }

  if (current) {
    s_failRun = 0;
    s_lastOkMs = nowMs;
  }

  // 跨代回调一律不入队：它属于上一轮的节拍，塞进缓冲会污染序号与时间戳
  if (!current) { SMP_UNLOCK(); return false; }

  const uint16_t n = (uint16_t)((s_head + 1) & (RING_N - 1));
  if (n == s_tail) { s_ringDrop++; SMP_UNLOCK(); return false; }   // 满：丢新样本

  Sample& d = s_ring[s_head];                // ≤14 字节结构拷贝，纯内存
  d.seq = seq;
  d.tUs = nowUs;
  d.raw = r;
  s_head = n;

  // 缓冲水位：记录"入队时刻"的占用峰值
  {
    const uint16_t used = (uint16_t)((s_head - s_tail) & (RING_N - 1));
    if (used > s_timing.ringHighWater) s_timing.ringHighWater = used;
  }
  SMP_UNLOCK();
  return true;
}

static void onTick(void*) {
  uint32_t epoch, seq;
  if (!tickBegin(&epoch, &seq)) return;      // 被拒绝：直接返回，不碰 tickEnd

  const uint32_t t0 = micros();              // 唯一新增的时间调用，用于统计读耗时
  MpuRaw r;
  const bool ok = mpuReadAccel(&r);          // 阻塞 I2C 在锁外
  tickEnd(ok, epoch, seq, r, t0);
}

/* ---------------- 启动 / 停止 ---------------- */

SamplerStartResult samplerStart(void) {
  SMP_LOCK();
  if (s_running) { SMP_UNLOCK(); return SMP_START_ALREADY_RUNNING; }
  const uint32_t nextEpoch = s_epoch + 1;
  s_accept = false;                          // 先关，失败时不会留下半启动
  SMP_UNLOCK();

  // ① 创建（失败不留半初始化句柄）
  if (!s_timer) {
    esp_timer_create_args_t cfg = {};
    cfg.callback        = &onTick;
    cfg.dispatch_method = ESP_TIMER_TASK;    // 任务上下文，可调用 Wire
    cfg.name            = "vd_smp";
    esp_timer_handle_t h = nullptr;
    if (esp_timer_create(&cfg, &h) != ESP_OK || h == nullptr) {
      s_timer = nullptr;
      return SMP_START_ERR_CREATE;
    }
    s_timer = h;
  }

  // ② 启动
  if (esp_timer_start_periodic(s_timer, 1000000ULL / FS_HZ) != ESP_OK) {
    SMP_LOCK(); s_accept = false; SMP_UNLOCK();   // 失败路径确保关掉
    return SMP_START_ERR_START;
  }

  // ③ 只有成功后才设运行标志、换代、开 accept
  const uint32_t nowMs = millis();
  SMP_LOCK();
  s_epoch    = nextEpoch;                    // 在 start 递增（不在 stop）
  s_accept   = true;
  s_running  = true;
  s_lastOkMs = nowMs;
  s_failRun  = 0;
  SMP_UNLOCK();

  return SMP_START_OK;
}

SamplerStopResult samplerStop(void) {
  // ① 取快照并禁止新回调
  SMP_LOCK();
  const bool wasRunning = s_running;
  const bool hasTimer   = (s_timer != nullptr);
  const uint32_t inFlightNow = s_inFlight;
  s_accept = false;
  SMP_UNLOCK();

  if (!wasRunning && inFlightNow == 0) return SMP_STOP_ALREADY_STOPPED;

  // ② 停定时器，返回值检查
  esp_err_t err = ESP_OK;
  if (hasTimer) {
    err = (esp_err_t)esp_timer_stop(s_timer);
  } else if (wasRunning) {
    err = ESP_ERR_INVALID_STATE;             // 标记说在跑但句柄没了 → 内部不一致
  }

  SMP_LOCK();
  s_running = false;
  // ESP_ERR_INVALID_STATE 且原本就没跑 = 正常；其余非 OK 都算异常
  if (err != ESP_OK && !(err == ESP_ERR_INVALID_STATE && !wasRunning)) {
    s_stats.timerErr++;
    s_stats.timerErrCode = (uint8_t)(err & 0xFF);
  }
  SMP_UNLOCK();

  // ③ 等回调退出：delay(1) 让出 CPU，不忙等
  const uint32_t t0 = millis();
  while (true) {
    SMP_LOCK();
    const uint32_t n = s_inFlight;
    SMP_UNLOCK();
    if (n == 0) return SMP_STOP_OK;
    if ((uint32_t)(millis() - t0) > (uint32_t)SMP_STOP_TIMEOUT_MS) {
      SMP_LOCK(); s_stats.stopTimeout++; SMP_UNLOCK();
      return SMP_STOP_TIMEOUT;               // 不继续做任何事
    }
    delay(1);                                // 让出 CPU，回调才有机会跑完
  }
}

bool samplerReset(void) {
  SMP_LOCK();
  if (s_running || s_inFlight != 0) {
    s_stats.resetViolation++;
    SMP_UNLOCK();
    return false;                            // 明确告知调用方"没清成"
  }
  s_head = 0; s_tail = 0;
  s_tickSeq = 0;
  s_failRun = 0;
  s_havePrev = false;
  SMP_UNLOCK();
  return true;
}

/* ---------------- 消费端 ---------------- */

bool samplerPop(Sample* s, SamplerGap* gap) {
  SMP_LOCK();
  if (s_tail == s_head) { SMP_UNLOCK(); return false; }
  *s = s_ring[s_tail];                       // ≤14 字节结构拷贝
  s_tail = (uint16_t)((s_tail + 1) & (RING_N - 1));
  SMP_UNLOCK();

  // gap 计算在锁外：只读 s_prevSeq/s_prevUs/s_havePrev，只有消费者写，无竞争
  gap->kind = SMP_GAP_NONE;
  gap->missed = 0;
  gap->devUs = 0;
  gap->spanUs = 0;

  if (s_havePrev) {
    const uint32_t dSeq = s->seq - s_prevSeq;
    const int32_t  dUs  = (int32_t)(s->tUs - s_prevUs);
    const int32_t  tNom = 1000000 / FS_HZ;
    gap->spanUs = (uint32_t)dUs;

    if (dSeq != 1) {
      // 序号断开：真的丢了样本
      gap->kind = SMP_GAP_MISSING;
      gap->missed = dSeq - 1;
    } else if (dUs < tNom - SMP_JITTER_TOL_US || dUs > tNom + SMP_JITTER_TOL_US) {
      // 序号连续但间隔偏离：样本都在，时间轴被拉伸/压缩
      gap->kind = SMP_GAP_JITTER;
      gap->devUs = dUs - tNom;
    }
  }
  s_havePrev = true;
  s_prevSeq = s->seq;
  s_prevUs = s->tUs;

  // 间隔统计分流：连续样本进 span，缺口进 gapSpan（否则 10ms 缺口会污染 spanUsMax）
  if (gap->spanUs != 0) {
    SMP_LOCK();
    if (gap->kind == SMP_GAP_NONE) {
      s_timing.spanN++;
      emaStep(&s_emaSpan, gap->spanUs);
      s_timing.spanUsAvg = s_emaSpan.avg;
      if (gap->spanUs > s_timing.spanUsMax) s_timing.spanUsMax = gap->spanUs;
      if (gap->spanUs < s_timing.spanUsMin) s_timing.spanUsMin = gap->spanUs;
    } else {
      s_timing.gapSpanN++;
      emaStep(&s_emaGap, gap->spanUs);
      s_timing.gapSpanUsAvg = s_emaGap.avg;
      if (gap->spanUs > s_timing.gapSpanUsMax) s_timing.gapSpanUsMax = gap->spanUs;
    }
    SMP_UNLOCK();
  }

  if (gap->kind != SMP_GAP_NONE) {
    SMP_LOCK();
    s_stats.gaps++;
    if (gap->kind == SMP_GAP_MISSING) s_stats.missed += gap->missed;
    else                              s_stats.jitter++;
    SMP_UNLOCK();
  }
  return true;
}

/* ---------------- 时序统计 ---------------- */

const TimingStats& samplerTiming(void) { return s_timing; }

void samplerTimingReset(void) {
  SMP_LOCK();
  s_timing = TimingStats{};
  s_timing.spanUsMin = 0xFFFFFFFFu;
  timingResetEma();
  SMP_UNLOCK();
}

void samplerNoteLoopUs(uint32_t us) {
  SMP_LOCK();
  s_timing.loopN++;
  emaStep(&s_emaLoop, us);
  s_timing.loopUsAvg = s_emaLoop.avg;
  if (us > s_timing.loopUsMax) s_timing.loopUsMax = us;
  SMP_UNLOCK();
}

void samplerNoteOutUs(uint32_t us) {
  SMP_LOCK();
  s_timing.outN++;
  emaStep(&s_emaOut, us);
  s_timing.outUsAvg = s_emaOut.avg;
  if (us > s_timing.outUsMax) s_timing.outUsMax = us;
  SMP_UNLOCK();
}

/* ---------------- 状态查询 ---------------- */

bool samplerSensorLost(void) {
  SMP_LOCK();
  const bool running = s_running;
  const uint32_t failRun = s_failRun;
  const uint32_t lastOkMs = s_lastOkMs;
  SMP_UNLOCK();

  if (!running) return false;
  if (failRun >= SENSOR_LOST_N) return true;
  return (uint32_t)(millis() - lastOkMs) > SENSOR_STALE_MS;
}

uint32_t samplerInFlight(void) {
  SMP_LOCK();
  const uint32_t n = s_inFlight;
  SMP_UNLOCK();
  return n;
}

const SamplerStats& samplerStats(void) {
  SMP_LOCK();
  s_stats.readFail = s_readFail;
  s_stats.ringDrop = s_ringDrop;
  SMP_UNLOCK();
  return s_stats;
}

void samplerAddClip(void) {
  SMP_LOCK(); s_stats.clip++; SMP_UNLOCK();
}

void samplerClearStats(void) {
  SMP_LOCK();
  s_readFail = 0; s_ringDrop = 0;
  s_stats = SamplerStats{};
  SMP_UNLOCK();
}
/* ---------------- 枚举转字符串 ---------------- */

bool samplerStartIsError(SamplerStartResult r) {
  return r == SMP_START_ERR_CREATE || r == SMP_START_ERR_START;
}

bool samplerStopIsError(SamplerStopResult r) {
  return r == SMP_STOP_TIMEOUT;
}

const char* samplerStartStr(SamplerStartResult r) {
  switch (r) {
    case SMP_START_OK:              return "已启动";
    case SMP_START_ALREADY_RUNNING: return "已在运行";
    case SMP_START_ERR_CREATE:      return "定时器创建失败";
    case SMP_START_ERR_START:       return "定时器启动失败";
  }
  return "?";
}

const char* samplerStopStr(SamplerStopResult r) {
  switch (r) {
    case SMP_STOP_OK:              return "已停止";
    case SMP_STOP_ALREADY_STOPPED: return "本来就已停止";
    case SMP_STOP_TIMEOUT:         return "回调未退出";
  }
  return "?";
}

/* ---------------- 测试钩子 ---------------- */

#ifdef SMP_TEST_HOOKS
bool samplerTestTickBegin(uint32_t* epoch, uint32_t* seq) {
  return tickBegin(epoch, seq);
}

bool samplerTestTickEnd(bool ok, uint32_t epoch, uint32_t seq, const MpuRaw& r, uint32_t t0Us) {
  return tickEnd(ok, epoch, seq, r, t0Us);
}

uint32_t samplerTestEpoch(void) {
  SMP_LOCK();
  const uint32_t e = s_epoch;
  SMP_UNLOCK();
  return e;
}
#endif
