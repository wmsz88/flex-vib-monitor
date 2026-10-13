// 采样模块实现：生产者—消费者环形缓冲。
//
// 【并发模型】这是全工程唯一的跨上下文（并发）代码，改动前务必理解：
//   生产者 = esp_timer 回调 onTick()，运行在 ESP_TIMER_TASK 任务上下文，1kHz 写入
//   消费者 = 主循环 samplerPop()，运行在 loopTask 上下文，按自己节奏读出
//   共享区 = s_ring[] 与 head/tail 索引
// 之所以不需要加锁：这是标准的单生产者单消费者环形缓冲——生产者只写 s_head，
// 消费者只写 s_tail，双方只"读"对方的索引。ESP32 上对齐的 32/16 位读写是原子的，
// 因此不存在读到半个变量的情况。前提是两边各只写自己那个索引，不要越界去改对方的。
#include "sampler.h"
#include "config.h"
#include <Arduino.h>
#include <esp_timer.h>

/* ---------- 环形缓冲（生产者与消费者共享） ---------- */
static Sample            s_ring[RING_N];        // 样本池。深度 RING_N，用位与取模（N 必须是 2 的幂）
// s_head = 写指针（生产者私有写），指向下一个待写入的位置；
// s_tail = 读指针（消费者私有写），指向下一个待读出的位置。
// 空的条件是 head == tail；满的条件是 (head+1)&mask == tail——刻意牺牲 1 个槽位
// 来区分"空"与"满"，省掉一个计数字段。两者合并声明，与原代码保持一致。
static volatile uint16_t s_head = 0, s_tail = 0;

/* ---------- 生产者侧统计 ---------- */
static volatile uint32_t s_tickSeq = 0;         // 节拍序号。每个节拍 +1，读失败也 +1（下游据此发现缺样）
static volatile uint32_t s_failRun = 0;         // 连续失败节拍数
                                                // 一旦成功立即归零，因此它衡量的是"当前是否持续读不到"
static volatile uint32_t s_lastOkMs = 0;        // 最近一次成功读取的 millis()
                                                // 用于兜底判据：如果连回调都没跑，failRun 也不会增长
static volatile uint32_t s_readFail = 0, s_ringDrop = 0;   // 累计读失败 / 累计缓冲溢出丢样

/* ---------- 消费者侧状态（仅主循环访问，无需 volatile） ---------- */
static SamplerStats      s_stats = {};          // 对外暴露的统计快照（把生产者计数器拷进来）
static bool              s_havePrev = false;    // 是否已有"上一个样本"可比对（首个样本不判连续性）
static uint32_t          s_prevSeq = 0, s_prevUs = 0;   // 上一个样本的序号与时刻，用于算 gap
static esp_timer_handle_t s_timer = nullptr;    // 定时器句柄。只创建一次，反复 start/stop 复用
static bool              s_running = false;     // 定时器是否在跑（samplerStop/Start 的幂等依据）

// 定时器回调：每个采样节拍执行一次。设计约束是"必须短"——它在共享任务里跑，
// 拖长了会影响同任务下的其他定时器，也会让采样节拍抖动。
static void onTick(void*) {
  // 先取号再自增：无论后面读取成功还是失败，这一拍都已经"用掉"了序号。
  // 这正是缺样可被发现的原因——失败那一拍不会写入 ring，但序号已经前进了。
  uint32_t seq = s_tickSeq;
  s_tickSeq = seq + 1;

  MpuRaw r;
  if (!mpuReadAccel(&r)) {
    s_readFail = s_readFail + 1;
    s_failRun = s_failRun + 1;
    return;   // 失败就不写入缓冲，留给消费者一个 seq 跳变作为"这里缺样"的证据
  }
  s_failRun = 0;                 // 读到就说明链路恢复了
  s_lastOkMs = millis();

  // 预计算写入后的新 head，用它与 tail 比较判断是否已满
  uint16_t n = (uint16_t)((s_head + 1) & (RING_N - 1));
  // 满：丢新样本，序号断开会被消费端识别
  // 注意这里是"丢新保旧"策略——保住的旧样本时间戳更连续，对主频估计更有价值
  if (n == s_tail) { s_ringDrop = s_ringDrop + 1; return; }
  Sample& d = s_ring[s_head];
  d.seq = seq;
  d.tUs = (uint32_t)micros();    // 取"读完之后"的时刻，比取读之前更贴近数据真实产生时间
  d.raw = r;
  s_head = n;                    // 最后一步才发布 head：保证消费者看到 head 前进时数据已经写好
}

// 启动定时器。可重复调用（已在跑则直接返回），因此上层不必关心当前状态。
void samplerStart(void) {
  if (!s_timer) {
    // 首次调用时创建定时器。参数用 {} 零初始化，避免结构体里有垃圾值。
    esp_timer_create_args_t cfg = {};
    cfg.callback = &onTick;
    cfg.dispatch_method = ESP_TIMER_TASK;   // 任务上下文，可调用 Wire
                                            // 不能用 ESP_TIMER_ISR：中断里不允许调用 I2C 的 Wire API
    cfg.name = "vd_smp";
    esp_timer_create(&cfg, &s_timer);
  }
  if (s_running) return;
  // 重置健康度基准：否则"停机期间"会被算成"很久没有成功样本"，一启动就误报断线
  s_lastOkMs = millis();
  s_failRun = 0;
  esp_timer_start_periodic(s_timer, 1000000ULL / FS_HZ);   // 周期单位是微秒
  s_running = true;
}

// 停止定时器。必须真正停下来之后主循环才能独占 I2C（标定阶段要自己直接读传感器）。
void samplerStop(void) {
  if (s_timer && s_running) esp_timer_stop(s_timer);
  s_running = false;
  // esp_timer_stop 只保证"不再触发新的回调"，不保证"正在执行的回调已经结束"。
  // 这里等 2ms 让可能正在途中的回调跑完，避免标定时主循环与回调同时操作 Wire。
  delay(2);
}

// 清空缓冲与序号。必须在停止状态下调用，否则生产者会同时写入，清理结果不确定。
void samplerReset(void) {
  s_head = 0; s_tail = 0;        // 一起归零即表示"空"
  s_tickSeq = 0;
  s_failRun = 0;
  s_havePrev = false;            // 忘记上一个样本，否则复位后第一个样本会算出巨大的 gap
}

// 取出一个样本，并判断它与上一个样本之间是否连续。
// 连续性有两条判据，任一不满足即认为缺样：
//   ① 序号不连续 → 中间有读取失败或缓冲溢出，缺样数 = 序号差 - 1
//   ② 序号连续但时间间隔偏离标称超过容差 → 回调被推迟/挤压，样本不再等间隔
// 判据②不能省：定时器回调被高优先级任务挤开时，序号是连续的，但采样时刻已经乱了。
// 而主频估计依赖"等间隔"这个前提，用乱掉的时间轴算频率必然出错。
bool samplerPop(Sample* s, uint32_t* gap) {
  if (s_tail == s_head) return false;              // 空
  *s = s_ring[s_tail];
  s_tail = (uint16_t)((s_tail + 1) & (RING_N - 1));   // 先取数据再推进 tail

  uint32_t g = 0;
  if (s_havePrev) {
    uint32_t dSeq = s->seq - s_prevSeq;            // 无符号减法，跨回绕也正确
    int32_t  dUs  = (int32_t)(s->tUs - s_prevUs);  // 有符号：micros() 回绕（约 71 分钟）时仍能得到正确差值
    const int32_t tNom = 1000000 / FS_HZ;          // 标称间隔 = 1000us
    if (dSeq != 1) {
      g = dSeq - 1;                                // 判据①
    } else if (dUs < tNom - SMP_JITTER_TOL_US || dUs > tNom + SMP_JITTER_TOL_US) {
      g = 1;   // 序号连续但回调被推迟/挤压：样本不再等间隔，按 1 个缺样处理
    }
    if (g) { s_stats.gaps++; s_stats.missed += g; }
  }
  s_havePrev = true;
  s_prevSeq = s->seq;
  s_prevUs = s->tUs;
  *gap = g;
  return true;
}

// 判定传感器是否掉线。两条判据取"或"，覆盖两类不同的故障：
//   ① 连续失败节拍数超限 → 能读到总线但一直失败（例如器件被拔、供电掉了）
//   ② 太久没有成功样本   → 连回调都没跑起来或根本没走到成功分支（例如定时器异常、总线卡死）
// 只用①会漏掉②（失败计数没增长），只用②则反应太慢，所以两者都要。
bool samplerSensorLost(void) {
  if (!s_running) return false;                    // 没在采样就谈不上"掉线"
  if (s_failRun >= SENSOR_LOST_N) return true;
  return (uint32_t)(millis() - s_lastOkMs) > SENSOR_STALE_MS;
}

// 返回统计快照。这里把生产者侧的 volatile 计数器拷进结构体，让调用方拿到一致的一份数据。
// 注意 s_stats 自身由消费者维护（gaps/missed/clip），生产者只写 readFail/ringDrop 两个独立变量。
const SamplerStats& samplerStats(void) {
  s_stats.readFail = s_readFail;
  s_stats.ringDrop = s_ringDrop;
  return s_stats;
}

// 由信号处理模块在发现削顶样本时调用（累加器放在消费者侧，无并发问题）。
void samplerAddClip(void) { s_stats.clip++; }

// 清零全部统计（响应 'n' 命令）。
void samplerClearStats(void) {
  s_readFail = 0; s_ringDrop = 0;
  s_stats = SamplerStats{};
}
