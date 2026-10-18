#include "output.h"
#include "config.h"
#include "mpu.h"
#include "sampler.h"
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
  outPrintf("主频 %.1f Hz(%s%s)  幅值 %.1f mg(%s)  振动 %u  启停 %.2f Hz(%s, %u 周期)  接触 %u  主轴 %c\n",
            fin(r.freqHz), r.freqValid ? "有效" : "无效",
            r.freqClipLimited ? ",削顶降级" : "",
            fin(r.ampMg), r.ampValid ? "可信" : "不可信",
            (unsigned)r.vib,
            fin(r.ssHz), r.ssValid ? "有效" : "未确立", (unsigned)r.ssPeriods, (unsigned)r.contact,
            'X' + r.axis);
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

void outOnSample(const DspFrame& f) {
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
  outPrintf("#F=%.1f%s A=%.1f%s vib=%u SS=%.2f%s ct=%u dev=%.1f ax=%u sens=%u gap=%lu jit=%lu drop=%lu fail=%lu clip=%lu%s\n",
            fin(r.freqHz), r.freqValid ? "" : "(x)",
            fin(r.ampMg), r.ampValid ? "" : "(x)", (unsigned)r.vib,
            fin(r.ssHz), r.ssValid ? "" : "(x)", (unsigned)r.contact, fin(r.ctDev), (unsigned)r.axis,
            (unsigned)r.sensorOk, (unsigned long)s.gaps, (unsigned long)s.jitter,
            (unsigned long)s.ringDrop, (unsigned long)s.readFail, (unsigned long)s.clip,
            r.freqClipLimited ? " [clip]" : "");
}

void outPeriodic(uint32_t nowMs) {
  static uint32_t tPlot = 0, tDbg = 0;
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
        outPrintf("命令: 0~4 选绘图组(0关 1结果 2启停包络 3接触 4三轴波形) | p 轮换绘图组\n"
                  "      d 调试文字行 | S 状态 | c 重新标定(须静置) | n 清零统计 | ? 帮助\n");
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
      default: break;
    }
  }
  return act;
}
