#include "output.h"
#include "config.h"
#include "mpu.h"
#include "sampler.h"
#include "motor_test.h"     // 【临时调试】MOTOR_TEST_ENABLE=0 时无实际内容
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

// 勾选 "USB CDC On Boot" 时 Serial 是 USB 口；否则 Serial 是 UART0，绑到 GPIO20/21
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  #define OUT_USB_CDC 1
#else
  #define OUT_USB_CDC 0
#endif
#define OUT Serial

static uint8_t s_plot = PLOT_DEFAULT;
static bool    s_dbg = false;
static LedMode s_led = LED_RUN;

/* 三轴波形抽取比：运行时可通过 'w' 命令调整（0 = 关闭输出） */
static uint8_t s_waveDecim    = PLOT_WAVE_DEFAULT_DECIM;
static uint8_t s_waveDecimCnt = 0;
static uint32_t s_waveDropped = 0;

static const char* const kPlotName[PLOT_COUNT] = {"关", "结果", "启停包络", "接触", "三轴波形"};

/* ---- Q4 打标模式 ----
 * s_label    当前人工标签 0~6（k0~k6）
 * s_labelMode 是否处于打标模式（独占串口，只输出 CSV）
 * s_csvSeq   CSV 行序号，用于离线检测丢行
 */
static bool     s_labelMode = false;
static uint8_t  s_label     = 0;
static uint32_t s_csvSeq    = 0;
static uint16_t s_csvDecimCnt = 0;

/* CSV 抽取比 = 1kHz / 100Hz = 10，即每 10 个样本输出一行。
 * 不用 1kHz 是因为数据量大 100 倍，而接触判据关心的是几十~几百 ms 尺度的变化。 */
#define CSV_DECIM      (FS_HZ / (FS_HZ / ENV_HOP))

void outBegin(void) {
#if OUT_USB_CDC
  OUT.begin(BAUD);
  // 非阻塞：调试数据写不进去就丢，绝不拖慢核心测量。
  // USBCDC::write 默认有 250ms 超时，主机不读时会卡住主循环 250ms（丢掉 250 个样本）。
  OUT.setTxTimeoutMs(0);
  compBegin();                       // 比赛口：UART1，独立引脚
#else
  OUT.begin(BAUD, SERIAL_8N1, PIN_DEBUG_RX, PIN_DEBUG_TX);
  // 非 CDC 模式下 Serial 占 UART0。若比赛口引脚与调试口相同，两者会打架，
  // 此时比赛口让位（编译器会把这个判断消掉，零运行时开销）。
  if (PIN_COMP_TX != PIN_DEBUG_TX || PIN_COMP_RX != PIN_DEBUG_RX) {
    compBegin();
  }
#endif
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_Y, OUTPUT);
}

/* ---------------- 比赛口 ---------------- */

static bool     s_compReady = false;
#if COMP_FRAME_ENABLED
static uint16_t s_compSeq   = 0;
#endif

void compBegin(void) {
  Serial1.begin(COMP_BAUD, SERIAL_8N1, PIN_COMP_RX, PIN_COMP_TX);
  s_compReady = true;
  // 比赛口不做任何自检输出：协议未定时保持完全静默。
  // 状态报告走调试口，见 outPrintBanner。
}

bool compReady(void) { return s_compReady; }

/* 数值编码：把物理量映射成定宽字段。
 * 约定：无效量一律发 0，由 VALID 位图区分"未知"与"测得 0"。 */
static unsigned encFreqX10(const Result& r) {
  if (!r.freqValid) return 0;
  float v = r.freqHz * 10.0f;
  if (!isfinite(v) || v < 0.0f) return 0;
  if (v > 99999.0f) v = 99999.0f;
  return (unsigned)(v + 0.5f);
}

static unsigned encAmpMg(const Result& r) {
  if (!r.ampValid) return 0;
  float v = r.ampMg;
  if (!isfinite(v) || v < 0.0f) return 0;
  if (v > 99999.0f) v = 99999.0f;
  return (unsigned)(v + 0.5f);
}

static unsigned encSsX100(const Result& r) {
  if (!r.ssValid) return 0;
  float v = r.ssHz * 100.0f;
  if (!isfinite(v) || v < 0.0f) return 0;
  if (v > 999.0f) v = 999.0f;
  return (unsigned)(v + 0.5f);
}

/* VALID 位图：bit0=ampValid bit1=freqValid bit2=ssValid bit3=contactValid */
static char encValid(const Result& r) {
  uint8_t v = 0;
  if (r.ampValid)     v |= 0x01;
  if (r.freqValid)    v |= 0x02;
  if (r.ssValid)      v |= 0x04;
  if (r.contactValid) v |= 0x08;
  return "0123456789ABCDEF"[v & 0x0F];
}

static uint8_t xorChk(const char* s, int n) {
  uint8_t x = 0;
  for (int i = 0; i < n; i++) x ^= (uint8_t)s[i];
  return x;
}

/* 帧格式（35 字节）：
 *   $VD,SEQ,FREQx10,AMP_MG,VIB,CT,SSx100,VALID,CHK\r\n
 *   正文 30 字节 + ',' + 2 位校验 + "\r\n" = 35
 * 返回写入的字节数（不含 \0），失败返回 -1。 */
int compBuildFrame(const Result& r, uint16_t seq, char* out, int outSize) {
  if (!out || outSize < 36) return -1;

  char body[32];
  const int n = snprintf(body, sizeof(body), "$VD,%04u,%05u,%05u,%u,%u,%03u,%c",
                         (unsigned)(seq % 10000u),
                         encFreqX10(r), encAmpMg(r),
                         (unsigned)(r.vib ? 1 : 0),
                         (unsigned)(r.contact ? 1 : 0),
                         encSsX100(r),
                         encValid(r));
  if (n <= 0 || n >= (int)sizeof(body)) return -1;

  const uint8_t chk = xorChk(body, n);
  const int m = snprintf(out, (size_t)outSize, "%s,%02X\r\n", body, chk);
  if (m <= 0 || m >= outSize) return -1;
  return m;
}

void compPeriodic(uint32_t nowMs) {
#if COMP_FRAME_ENABLED
  if (!s_compReady) return;
  static uint32_t lastMs = 0;
  if ((uint32_t)(nowMs - lastMs) < (uint32_t)COMP_PERIOD_MS) return;
  lastMs = nowMs;

  char buf[40];
  const int n = compBuildFrame(stateResult(), s_compSeq, buf, (int)sizeof(buf));
  if (n <= 0) return;
  Serial1.write((const uint8_t*)buf, (size_t)n);
  s_compSeq++;
#else
  (void)nowMs;   // 协议口径确定前：整个函数编译为空，零开销、零输出
#endif
}

void outPrintf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
  OUT.write((const uint8_t*)buf, (size_t)n);
}

static float fin(float v) { return isfinite(v) ? v : 0.0f; }

void outPrintBanner(void) {
  outPrintf("\n===== vibmon v2.0 (ESP32-C3 + MPU6050) =====\n");
  outPrintf("采样 %d Hz | 窗 %d/%d | 高通 %.0f Hz | 包络 %d ms | %d bps %s\n",
            FS_HZ, BLOCK_N, HOP, HP_FC_HZ, ENV_WIN * 1000 / FS_HZ, BAUD,
            OUT_USB_CDC ? "USB-CDC" : "UART 调试口");
  // 比赛口状态只走调试口打印，绝不写到比赛口（比赛口默认完全静默）
  outPrintf("[COMP] %s TX=%d RX=%d baud=%d，帧输出 %s\n",
            s_compReady ? "UART1 ready" : "未启用",
            PIN_COMP_TX, PIN_COMP_RX, COMP_BAUD,
            COMP_FRAME_ENABLED ? "开启" : "关闭（协议未定）");
  outPrintf("命令 '?' 查看帮助\n");
}

void outPrintCal(void) {
  const DspCalReport& rep = dspCalReport();
  const DspCal& c = dspCal();
  const Thresholds& t = stateThresholds();

  // 第一段：本次尝试（失败时也打印候选指标，否则现场只知道"失败了"）
  outPrintf("[CAL] 本次：%s\n", calResultStr(rep.result));
  if (rep.result != CALRES_OK && rep.result != CALRES_NONE) {
    outPrintf("[CAL]   nBias=%lu  stdMax=%.1f mg  |g|=%.0f mg  env=%.2f  ct=%.2f  clip=%u\n",
              (unsigned long)rep.nBias, rep.rawStdMax, rep.biasNormMg,
              rep.envRest, rep.ctRest, (unsigned)rep.stageBClip);
  }

  // 第二段：当前生效配置
  outPrintf("[CAL] 生效：%s | %s | 样本 %lu\n",
            c.ok ? "有效标定" : "未标定（编译期默认参数）",
            c.quality ? "质量合格" : "质量可疑(标定时可能被碰或未静置)",
            (unsigned long)c.nBias);
  outPrintf("[CAL] bias %.1f/%.1f/%.1f mg  |g|=%.0f mg  std %.1f/%.1f/%.1f mg\n",
            c.bias[0], c.bias[1], c.bias[2], c.biasNormMg, c.rawStd[0], c.rawStd[1], c.rawStd[2]);
  outPrintf("[CAL] 带内 sigma %.2f/%.2f/%.2f mg  静置包络 %.2f  静置姿态 %.2f mg\n",
            c.sigBand[0], c.sigBand[1], c.sigBand[2], c.envRest, c.ctRest);
  outPrintf("[CAL] 阈值 过零 %.1f | 启停 %.1f/%.1f | 接触 %.1f/%.1f mg\n",
            c.zcl, t.hi, t.lo, t.ctOn, t.ctOff);
  outPrintf("[CAL] 量程 ±%dg（%.0f LSB/g），削顶线 %d LSB ≈ %.0f mg\n",
            2 << (ACCEL_FS_SEL & 0x03), mpuLsbPerG(), ACCEL_CLIP_RAW,
            ACCEL_CLIP_RAW * 1000.0f / mpuLsbPerG());
}

void outPrintStatus(void) {
  const Result& r = stateResult();
  const SamplerStats& s = samplerStats();
  outPrintf("\n--- 状态 ---\n");
  outPrintf("传感器 %s  标定 %s/%s  绘图组 %u(%s)  调试行 %s\n",
            r.sensorOk ? "在线" : "故障",
            r.calOk ? "有效" : "未标定",
            r.calQuality ? "合格" : "可疑",
            (unsigned)s_plot, kPlotName[s_plot], s_dbg ? "开" : "关");
  outPrintf("主频 %.1f Hz(%s%s)  幅值 %.1f mg(%s)  振动 %u  启停 %.2f Hz(%s, %u 周期)\n",
            fin(r.freqHz), r.freqValid ? "有效" : "无效",
            r.freqClipLimited ? ",削顶降级" : "",
            fin(r.ampMg), r.ampValid ? "可信" : "不可信",
            (unsigned)r.vib,
            fin(r.ssHz), r.ssValid ? "有效" : "未确立", (unsigned)r.ssPeriods);
  /* 接触单独一行：必须显式区分"未接触"和"不知道"（contactValid=false 时判据不可信） */
#if CT_MODE == 2
  if (stateTouchRefCapturing()) {
    outPrintf("接触 采集中… 请保持握住不动（尚未生效）\n");
  } else if (r.ctRefSet) {
    outPrintf("接触 %u(%s)  到接触姿态 %.1f mg (进入%.0f/退出%.0f)  姿态偏差 %.1f mg  主轴 %c\n",
              (unsigned)r.contact, r.contactValid ? "可信" : "不可信",
              fin(r.ctTouchDist), CT_TOUCH_MG, CT_TOUCH_OFF_MG, fin(r.ctDev), 'X' + r.axis);
  } else {
    outPrintf("接触 %u(不可信)  未记录接触姿态参考点，按 't' 记录（握住测试件时按）\n",
              (unsigned)r.contact);
  }
#else
  outPrintf("接触 %u(%s)  姿态偏差 %.1f mg  主轴 %c\n",
            (unsigned)r.contact, r.contactValid ? "可信" : "不可信",
            fin(r.ctDev), 'X' + r.axis);
#endif
  outPrintf("起 %lu / 停 %lu | 读失败 %lu  缓冲溢出 %lu  不连续 %lu(缺 %lu, 抖动 %lu)  削顶 %lu  I2C错 %lu\n",
            (unsigned long)r.nStart, (unsigned long)r.nStop, (unsigned long)s.readFail,
            (unsigned long)s.ringDrop, (unsigned long)s.gaps, (unsigned long)s.missed,
            (unsigned long)s.jitter, (unsigned long)s.clip, (unsigned long)mpuErrCount());
  outPrintf("采样器 | 回调中 %lu  停止超时 %lu  非法重置 %lu  定时器错 %lu(0x%02X)\n",
            (unsigned long)samplerInFlight(), (unsigned long)s.stopTimeout,
            (unsigned long)s.resetViolation, (unsigned long)s.timerErr,
            (unsigned)s.timerErrCode);
  outPrintCal();
}

/* ---------- Q4 打标模式 ---------- */

/* 一帧 CSV。字段见 output.h 注释。
 * gap 取自 samplerStats().gaps 的变化：本行相对上一行是否有新增缺口。
 * 用静态变量记录上次数值，差值 >0 即表示这一行之前发生过缺口。 */
static void csvLine(const DspFrame& f) {
  const Result& r = stateResult();
  const SamplerStats& s = samplerStats();
  static uint32_t s_lastGaps = 0;
  uint8_t gapFlag = (s.gaps != s_lastGaps) ? 1 : 0;
  s_lastGaps = s.gaps;

  outPrintf("%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u,%.2f\n",
            (unsigned long)(++s_csvSeq),
            (unsigned long)millis(),
            fin(f.ctAxis[0]), fin(f.ctAxis[1]), fin(f.ctAxis[2]),
            fin(r.ctDev), fin(r.ampMg),
            (unsigned)r.vib, (unsigned)r.contact, (unsigned)r.contactValid,
            (unsigned)gapFlag, (unsigned)s_label,
            fin(r.ctTouchDist));      /* 第 13 列：到接触姿态的距离（-1 = 无参考点） */
}

/* 标签切换事件行。人工打标有反应延迟，离线分析靠它精确定位切换时刻。 */
static void csvEvent(uint8_t label) {
  outPrintf("EV,t_ms=%lu,label=%u\n", (unsigned long)millis(), (unsigned)label);
}

/* 【打标模式要能恢复原设置】
 * 进入时把绘图组/调试行/波形抽取全部关掉（独占串口），
 * 但**退出时必须恢复**，否则打标一次之后就永久用不了波形组：
 *   s_waveDecim 被设成 0，而 outOnSample 里 "s_waveDecim == 0" 直接 return，
 *   之后即使发 '4' 选了波形组也永远没有输出。
 * 实测踩过这个坑（2026-10-18）：打完标后发 '4' 无任何输出，
 * 必须发 w1 手工恢复。
 * 所以这里把三个设置存起来，退出时还原。 */
static uint8_t s_savedPlot      = PLOT_OFF;
static bool    s_savedDbg       = false;
static uint8_t s_savedWaveDecim = 0;

void outEnterLabelMode(void) {
  if (!s_labelMode) {
    s_labelMode = true;
    s_csvSeq    = 0;
    /* 只在首次进入时保存（重复按 k0~k6 不应覆盖成"已是打标态"的值） */
    s_savedPlot      = s_plot;
    s_savedDbg       = s_dbg;
    s_savedWaveDecim = s_waveDecim;
    outPrintf("#打标模式：仅输出 CSV（seq,t_ms,ctX,ctY,ctZ,ctDev,ampMg,vib,contact,cvalid,gap,label）\n"
              "#k0 空闲 | k1 接触中 | k2 保持+振动暂停 | k3 释放 | k4 无接触转动 | k5 周期振动 | k6 接触中+振动 | K 退出\n");
  }
  s_plot       = PLOT_OFF;   // 关绘图
  s_dbg        = false;      // 关调试行
  s_waveDecim  = 0;          // 关三轴波形
}

void outExitLabelMode(void) {
  if (!s_labelMode) return;
  s_labelMode = false;
  /* 恢复进入打标前的设置，避免"打标一次后波形组永久失效" */
  s_plot      = s_savedPlot;
  s_dbg       = s_savedDbg;
  s_waveDecim = s_savedWaveDecim;
  outPrintf("#退出打标模式，共 %lu 行；已恢复绘图组 %u(%s)、波形抽取 w%u\n",
            (unsigned long)s_csvSeq,
            (unsigned)s_plot, kPlotName[s_plot], (unsigned)s_waveDecim);
}

bool outInLabelMode(void) { return s_labelMode; }

void outOnSample(const DspFrame& f) {
  if (s_labelMode) {
    /* 打标模式：每 CSV_DECIM 个样本输出一行。
     * 放在这里（逐样本回调）而不是 outPeriodic，是为了让时间戳严格对齐
     * 实际采样序列 —— 用 millis() 定时会因为主循环抖动产生不均匀间隔。 */
    if (++s_csvDecimCnt >= CSV_DECIM) { s_csvDecimCnt = 0; csvLine(f); }
    return;                                  // 独占：不再走波形输出
  }
  if (s_plot != PLOT_WAVE) return;
  if (s_waveDecim == 0) { s_waveDropped++; return; }        // 0 = 关闭输出
  if (++s_waveDecimCnt < s_waveDecim) { s_waveDropped++; return; }
  s_waveDecimCnt = 0;
  outPrintf("X:%.1f,Y:%.1f,Z:%.1f\n", f.fx, f.fy, f.fz);
}

void outSetWaveDecim(uint8_t decim) {
  s_waveDecim = decim;
  s_waveDecimCnt = 0;
}

uint32_t outWaveDropped(void) { return s_waveDropped; }

static void plotLine(void) {
  const Result& r = stateResult();
  const Thresholds& t = stateThresholds();
  switch (s_plot) {
    case PLOT_RESULT:
      outPrintf("Freq_Hz:%.1f,Amp_mg:%.1f,SS_Hz_x10:%.1f,Vib_x50:%d,Contact_x100:%d\n",
                fin(r.freqHz), fin(r.ampMg), fin(r.ssHz) * 10.0f, r.vib * 50, r.contact * 100);
      break;
    case PLOT_SS:
      outPrintf("Env_mg:%.2f,ThHi:%.2f,ThLo:%.2f,Vib:%.1f\n",
                fin(r.ampMg), t.hi, t.lo, r.vib * t.hi * 2.0f);
      break;
    case PLOT_CONTACT:
      outPrintf("CtDev_mg:%.2f,CtOn:%.2f,CtOff:%.2f,Contact:%.1f\n",
                fin(r.ctDev), t.ctOn, t.ctOff, r.contact * t.ctOn * 1.5f);
      break;
    default: break;
  }
}

static void debugLine(void) {
  const Result& r = stateResult();
  const SamplerStats& s = samplerStats();
  outPrintf("#F=%.1f%s A=%.1f%s vib=%u SS=%.2f%s ct=%u%s dev=%.1f ax=%u sens=%u gap=%lu jit=%lu drop=%lu fail=%lu clip=%lu%s\n",
            fin(r.freqHz), r.freqValid ? "" : "(x)",
            fin(r.ampMg), r.ampValid ? "" : "(x)", (unsigned)r.vib,
            fin(r.ssHz), r.ssValid ? "" : "(x)",
            (unsigned)r.contact, r.contactValid ? "" : "(x)",
            fin(r.ctDev), (unsigned)r.axis,
            (unsigned)r.sensorOk, (unsigned long)s.gaps, (unsigned long)s.jitter,
            (unsigned long)s.ringDrop, (unsigned long)s.readFail, (unsigned long)s.clip,
            r.freqClipLimited ? " [clip]" : "");
}

void outPeriodic(uint32_t nowMs) {
  static uint32_t tPlot = 0, tDbg = 0;
  if (s_labelMode) return;   // 打标模式独占串口：只有 CSV，不混入绘图行/调试行
  if (s_plot != PLOT_OFF && s_plot != PLOT_WAVE && nowMs - tPlot >= PLOT_MS) { tPlot = nowMs; plotLine(); }
  if (s_dbg && nowMs - tDbg >= DBG_MS) { tDbg = nowMs; debugLine(); }
}

static void ledWrite(int pin, bool on) {
#if LED_ACTIVE_LOW
  digitalWrite(pin, on ? LOW : HIGH);
#else
  digitalWrite(pin, on ? HIGH : LOW);
#endif
}

void ledSet(LedMode m) {
  s_led = m;
  if (m == LED_CAL) { ledWrite(PIN_LED_G, true); ledWrite(PIN_LED_Y, true); }
}

void ledUpdate(uint32_t nowMs, const Result& r) {
  switch (s_led) {
    case LED_RUN:   // 绿 1Hz 心跳，黄 = 振动中
      ledWrite(PIN_LED_G, (nowMs / 500) & 1);
      ledWrite(PIN_LED_Y, r.vib == 1);
      break;
    case LED_FAULT: // 绿灭，黄 4Hz 快闪：避免把故障误读成"正常运行"
      ledWrite(PIN_LED_G, false);
      ledWrite(PIN_LED_Y, (nowMs / 125) & 1);
      break;
    case LED_CAL:
      break;
  }
}

void outPrintTiming(void) {
  const TimingStats& t = samplerTiming();
  const SamplerStats& s = samplerStats();
  const uint32_t period = 1000000UL / FS_HZ;

  outPrintf("\n=== 时序统计（采样 %d Hz，周期 %lu us）===\n", FS_HZ, (unsigned long)period);

  outPrintf("读耗时    平均 %lu us (%lu%%)  最大 %lu us  (共 %lu 次，失败 %lu 次，失败最大 %lu us)\n",
            (unsigned long)t.readUsAvg,
            (unsigned long)(period ? (t.readUsAvg * 100UL / period) : 0),
            (unsigned long)t.readUsMax, (unsigned long)t.readN,
            (unsigned long)t.readFailN, (unsigned long)t.readFailUsMax);

  outPrintf("样本间隔  平均 %lu us  最小 %lu  最大 %lu  (连续 %lu 个)\n",
            (unsigned long)t.spanUsAvg,
            (unsigned long)(t.spanN ? t.spanUsMin : 0),
            (unsigned long)t.spanUsMax, (unsigned long)t.spanN);

  outPrintf("缺口间隔  平均 %lu us  最大 %lu  (%lu 次)\n",
            (unsigned long)t.gapSpanUsAvg, (unsigned long)t.gapSpanUsMax,
            (unsigned long)t.gapSpanN);

  outPrintf("主循环    平均 %lu us  最大 %lu us  (%lu 轮，已排除标定/命令)\n",
            (unsigned long)t.loopUsAvg, (unsigned long)t.loopUsMax, (unsigned long)t.loopN);

  outPrintf("输出耗时  平均 %lu us  最大 %lu us  (%lu 次)\n",
            (unsigned long)t.outUsAvg, (unsigned long)t.outUsMax, (unsigned long)t.outN);

  outPrintf("缓冲水位  最高 %u / %d (%u%%)\n", (unsigned)t.ringHighWater, RING_N,
            (unsigned)((unsigned long)t.ringHighWater * 100UL / RING_N));

  outPrintf("统计      缺样 %lu 次(缺 %lu 点)  抖动 %lu  溢出 %lu  读失败 %lu\n",
            (unsigned long)s.gaps, (unsigned long)s.missed, (unsigned long)s.jitter,
            (unsigned long)s.ringDrop, (unsigned long)s.readFail);

  outPrintf("采样器    回调中 %lu  停止超时 %lu  非法重置 %lu  定时器错 %lu(0x%02X)\n",
            (unsigned long)samplerInFlight(), (unsigned long)s.stopTimeout,
            (unsigned long)s.resetViolation, (unsigned long)s.timerErr,
            (unsigned)s.timerErrCode);

  outPrintf("波形      抽取 w%u  已丢弃 %lu 个样本\n",
            (unsigned)s_waveDecim, (unsigned long)s_waveDropped);

  /* 判据提示：把相对阈值直接算给现场看 */
  const uint32_t half = period / 2;
  if (t.readUsAvg > half)
    outPrintf("[!] 读耗时均值超过半周期，I2C 偏慢（考虑降 I2C_CLOCK 或上 FIFO）\n");
  if (t.readUsMax > period)
    outPrintf("[!] 单次读耗时超过一个采样周期，回调必然挤压\n");
  if (t.outUsMax > period)
    outPrintf("[!] 输出耗时超过一个采样周期，考虑加大抽取比（w8/w16）\n");
  if (t.ringHighWater > RING_N / 8)
    outPrintf("[!] 缓冲水位超过 1/8，消费开始吃力\n");
  if (s.stopTimeout > 0)
    outPrintf("[!] 出现停止超时，Wire 互斥量有竞争（考虑专用采样任务）\n");
}

static void setPlot(uint8_t g) {
  s_plot = g % PLOT_COUNT;
  if (s_plot != PLOT_OFF) s_dbg = false;   // 文字行会打乱绘图器
#if !OUT_USB_CDC
  // UART 115200 下 1kHz × 28 字节 ≈ 28KB/s，是带宽的 2.4 倍，必然阻塞。
  // 抽取比由 'w' 命令控制；进入波形组时若抽取不足则提示（不再强制禁止，
  // 因为 w8/w16 在 UART 下是可用的）。
  if (s_plot == PLOT_WAVE && s_waveDecim != 0 && s_waveDecim < 8) {
    outPrintf("#UART 下建议 w8 或更大抽取（当前 w%u，28KB/s 超 11.5KB/s 带宽）\n",
              (unsigned)s_waveDecim);
  }
#endif
}

CmdAction outPollCmd(void) {
  CmdAction act = CMD_NONE;
  while (OUT.available()) {
    char c = (char)OUT.read();
    switch (c) {
      case '?':
        setPlot(PLOT_OFF);
        /* 【必须拆成多条】outPrintf 的缓冲区是 256 字节，超长会被**静默截断**。
         * 中文一个字 3 字节，整段帮助 330 字节 → 之前第 3 行起全丢了。
         * 所以每条控制在一行以内，不依赖缓冲区大小。 */
        outPrintf("命令: 0~4 选绘图组(0关 1结果 2启停 3接触 4波形) | p 轮换\n");
        outPrintf("      d 调试行 | S 状态 | c 重标定(须静置) | n 清零统计 | ? 帮助\n");
        outPrintf("      t 记录接触姿态(握住时按) | t0 清除参考点\n");
        outPrintf("      k0~k6 打标模式(仅CSV) | K 退出打标\n");
        break;
      case '0': case '1': case '2': case '3': case '4':
        setPlot((uint8_t)(c - '0'));
        break;
      case 'p': setPlot((uint8_t)(s_plot + 1)); break;
      case 'd':
        s_dbg = !s_dbg;
        if (s_dbg) s_plot = PLOT_OFF;
        break;
      case 'S': setPlot(PLOT_OFF); outPrintStatus(); break;
      case 'T': setPlot(PLOT_OFF); outPrintTiming(); break;
      case 'c': act = CMD_RECAL; break;
      case 'w': {
        /* w<N>：设置三轴波形抽取比。N=0 关闭输出。
           只接受 0/1/2/4/8/16，其它值忽略（避免误触发）。 */
        uint8_t v = 0;
        bool have = false;
        while (OUT.available()) {
          char d = (char)OUT.peek();
          if (d < '0' || d > '9') break;
          OUT.read();
          v = (uint8_t)(v * 10 + (d - '0'));
          have = true;
        }
        if (!have) {
          outPrintf("#当前波形抽取 w%u（0=关）\n", (unsigned)s_waveDecim);
        } else if (v == 0 || v == 1 || v == 2 || v == 4 || v == 8 || v == 16) {
          outSetWaveDecim(v);
          outPrintf("#波形抽取设为 w%u（%s）\n", (unsigned)v,
                    v == 0 ? "关闭" : v == 1 ? "1:1 不抽取" : "已抽取");
        } else {
          outPrintf("#仅支持 w0/w1/w2/w4/w8/w16\n");
        }
        break;
      }
      case 'n':
        samplerClearStats(); mpuClearErr(); stateClearCounters();
        samplerTimingReset();
        outPrintf("#统计已清零（含时序），开始新一轮测量\n");
        break;

      /* ---- 接触姿态参考点（CT_MODE=2）----
       * t  = 记录当前姿态为"接触姿态"（选手在**握住**测试件时按一下）
       * t0 = 清除参考点（换机械手/换工位时用）
       * 为什么必须现场手动记录：接触姿态取决于机械手型号、装夹位置、工位姿态，
       * 换一套就完全失效，写死固定值不可靠。 */
      case 't': {
        bool clear = false;
        if (OUT.available()) {
          char d = (char)OUT.peek();
          if (d == '0') { OUT.read(); clear = true; }
        }
        if (clear) {
          stateClearTouchRef();
          outPrintf("#已清除接触姿态参考点（接触判定回到不可信）\n");
        } else if (stateStartTouchRef()) {
          /* 两阶段采集：先稳定 CT_REF_SETTLE_MS，再平均 CT_REF_AVG_MS。
           * 为什么要延时：实测按下的瞬间手往往还没握到位，
           * 记瞬时值会偏 1593mg（稳态只有 21mg）。 */
          outPrintf("#开始采集接触姿态：等待 %dms 稳定 + 平均 %dms，请保持握住不动\n",
                    CT_REF_SETTLE_MS, CT_REF_AVG_MS);
        } else {
          outPrintf("#正在采集中，请稍等\n");
        }
        break;
      }

      /* ---- Q4 打标模式 ----
       * k0~k6 直接写目标标签（不是"循环切换"）：
       *   采到一半忘了当前在哪一档就会整段标错，显式数字杜绝这个问题。
       * 每次切换额外输出一行 EV 事件，离线分析靠它定位切换时刻。 */
      case 'k': {
        if (!OUT.available()) break;
        char d = (char)OUT.peek();
        if (d >= '0' && d <= '6') {
          OUT.read();
          uint8_t newLabel = (uint8_t)(d - '0');
          outEnterLabelMode();
          s_label = newLabel;
          csvEvent(newLabel);
        }
        break;
      }
      case 'K':
        outExitLabelMode();
        break;

      /* ---- 【临时调试】同板马达驱动 ----
       * 仅 MOTOR_TEST_ENABLE=1 时生效。命令字符与正式功能完全不重叠：
       *   m0~m9 占空比、H 100%、P 55%、q/r 微调、W 扫频
       * 用 'm' 前缀是为了避开已有的 0~9（绘图组选择）。 */
#if MOTOR_TEST_ENABLE
      case 'm': {
        /* m<digit>：占空比 = 数字 x 10% */
        if (OUT.available()) {
          char d = (char)OUT.peek();
          if (d >= '0' && d <= '9') {
            OUT.read();
            motorTestSetDutyDigit(d);
            break;
          }
        }
        outPrintf("#[motor] 当前 duty %u%%（用法 m0~m9 / H / P / q / r / W）\n",
                  (unsigned)motorTestDuty());
        break;
      }
      case 'H': case 'P': case 'q': case 'r': case 'W':
        motorTestHandleCmd(c);
        break;
#endif

      default: break;
    }
  }
  return act;
}
