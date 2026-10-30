#include "output.h"
#include "config.h"
#include "mpu.h"
#include "sampler.h"
#include "posture.h"
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
  outPrintf("\n===== vibmon v5.0 (ESP32-C3 + MPU6050, 6轴) =====\n");
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
  /* v5：陀螺零偏与静止标准差。零偏是后面角度积分的基准，必须能看见。 */
  outPrintf("[GYRO] 零偏 %.2f/%.2f/%.2f dps  标准差最大 %.2f dps  量程 ±%.0f dps  采样 %d Hz\n",
            c.gyroBias[0], c.gyroBias[1], c.gyroBias[2], c.gyroStdMax,
            GYRO_RANGE_DPS, FS_HZ / GYRO_DIV);
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
#if CT_MODE == 4
  /* ---- 模式 4：角度判"闭合"+ 振动判"接触" ----
   * 角度只分两档（松手 / 已闭合），空夹与夹物靠幅值/频率区分（实测 6.4 vs 755mg）。 */
  outPrintf("接触 %u(%s)%s  闭合角 %.0f° (闭合>=%.0f/退出%.0f) [%s]\n",
            (unsigned)r.contact, r.contactValid ? "可信" : "不可信",
            r.gripNoObject ? " 空夹?" : "",
            fin(r.closeAngleDeg), ANGLE_CLOSED_DEG, ANGLE_CLOSED_OFF_DEG,
            r.angleValid ? "V" : "-");
  outPrintf("      幅值 %.1f/%.0f mg [%c]  频率 %.1f/%.0f Hz [%c]  三路与=%u\n",
            fin(r.ampMg), CT_FLAT_A_ON_MG, r.condAmp ? '*' : '-',
            fin(r.freqHz), CT_FLAT_F_ON_HZ, r.condFreq ? '*' : '-',
            (unsigned)(r.condPosture && r.condAmp && r.condFreq));
  if (!r.gyroSeen) {
    outPrintf("      （从未收到陀螺数据：检查 GYRO_DIV / mpuReadAll / 接线）\n");
  } else if (!r.angleValid) {
    outPrintf("      （陀螺数据失效或超量程，角度不可信）\n");
  }
  /* 【P2】角度靠"回到张开位"清零，而认零位用的是平放基准 D。
   * 基准没采到时 D 恒为 -1，清零条件永远不成立 ⇒ 角度会一直累积、
   * 无法回到 0。这时必须先按 t 采基准，否则闭合角读数没有意义。 */
  if (!r.flatRefSet) {
    outPrintf("      （平放基准未采到：角度无法归零，闭合角不可靠，请先按 't'）\n");
  }
#elif CT_MODE == 3
  /* ---- 模式 3：平放基准 + 三重与 ----
   * 这一行是 v4 最重要的调试界面：三条判据各自是否过门、当前量值、
   * 以及"空夹"提示，全部并列显示。
   * 为什么必须并列：三重与出错时最怕"只知道结果是 0，不知道卡在哪一路"，
   * 分开列出来才能一眼定位是姿态没压下去、还是频率没上到阈值。 */
  if (stateFlatRefCapturing()) {
    outPrintf("接触 采集中… 请让设备保持平放不动（基准尚未生效）\n");
  } else if (!r.flatRefSet) {
    outPrintf("接触 %u(不可信)  未采到平放基准", (unsigned)r.contact);
    if (r.flatRefRetry >= (uint8_t)CT_REF_AUTO_RETRY && r.flatRefRetry > 0) {
      outPrintf("（自动采集 %u 次均判定为在动，极差 %.0f>%.0f mg；请放稳后按 't' 重采）\n",
                (unsigned)r.flatRefRetry, fin(r.flatRefSwing), CT_REF_MAX_SWING_MG);
    } else {
      outPrintf("（等待自动采集，或按 't' 手动采集）\n");
    }
  } else {
    /* 第二行：三路判据明细。condXxx 是"原始比较结果"（不含滞回），
     * 用方块直观表示过门与否，现场调阈值时看这一行最快。 */
    outPrintf("接触 %u(%s)%s  平放到 %.0f mg (进入%.0f/退出%.0f) [%c]\n",
              (unsigned)r.contact, r.contactValid ? "可信" : "不可信",
              r.gripNoObject ? " 空夹?" : "",
              fin(r.flatDist), CT_FLAT_D_ON_MG, CT_FLAT_D_OFF_MG,
              r.condPosture ? 'V' : '-');
    outPrintf("      幅值 %.1f/%.0f mg [%c]  频率 %.1f/%.0f Hz [%c]  三路与=%u  基准极差 %.0f mg\n",
              fin(r.ampMg), CT_FLAT_A_ON_MG, r.condAmp ? 'V' : '-',
              fin(r.freqHz), CT_FLAT_F_ON_HZ, r.condFreq ? 'V' : '-',
              (unsigned)(r.condPosture && r.condAmp && r.condFreq),
              fin(r.flatRefSwing));
  }
#elif CT_MODE == 2
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
#if POSTURE_ENABLE
  {
    const PostureResult& p = postureResult();
    if (p.calActive) {
      outPrintf("姿态诊断 | 正在采集 START 基线（n=%lu）\n",
                (unsigned long)p.calN);
    } else if (!p.calValid) {
      outPrintf("姿态诊断 | 未标定 START 基线，请保持 START 后按 'P'\n");
    } else if (p.calFailed) {
      outPrintf("姿态诊断 | 本次 P 失败，沿用旧 START 基线；请回 START 后重按 'P'\n");
    } else {
      outPrintf("姿态诊断 | raw=%s stable=%s d=%.0fmg x/y/z=%.0f/%.0f/%.0f 窗口=%s scnt=%u n=%lu 基线=%.0f/%.0f/%.0f\n",
                postureClassName(p.cls), postureClassName(p.stableCls),
                (double)p.distMg,
                (double)p.x, (double)p.y, (double)p.z,
                p.valid ? "有效" : "无效",
                (unsigned)p.stableCount,
                (unsigned long)p.blockN,
                (double)p.startX, (double)p.startY, (double)p.startZ);
    }
  }
#endif
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

  /* v5 追加 3 列：closed / closeAngleDeg / gyroW
   * 打标 CSV 是离线分析的主要证据，缺了角度和角速度就无法验证"轨迹停顿"与"稳态"。 */
  outPrintf("%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u,%.2f,%u,%.1f,%.2f\n",
            (unsigned long)(++s_csvSeq),
            (unsigned long)millis(),
            fin(f.ctAxis[0]), fin(f.ctAxis[1]), fin(f.ctAxis[2]),
            fin(r.ctDev), fin(r.ampMg),
            (unsigned)r.vib, (unsigned)r.contact, (unsigned)r.contactValid,
            (unsigned)gapFlag, (unsigned)s_label,
            fin(r.ctTouchDist),       /* 第 13 列：到接触姿态的距离（-1 = 无参考点） */
            (unsigned)(r.closed ? 1u : 0u),
            fin(r.closeAngleDeg),
            fin(r.gyroW));
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

/* ---- 扫频采样行（'w' 命令，专为「振动源占空比 -> 振动范围」标定设计）----
 *
 * 【为什么必须单独做一条，而不复用 PLOT_RESULT 那行 CSV】
 *   plotLine() 里的 PLOT_RESULT 用 fin(v) 处理数值，而 fin() 的定义是
 *   `isfinite(v) ? v : 0.0f` —— 它只防止 NaN/INF 打印成乱码，
 *   **完全没有携带 r.freqValid / r.ampValid 这些有效性标记**。
 *   后果：马达空转、信号太弱、主频超出测频带时，freqHz 内部是不成立的值，
 *   CSV 里照样打印一个看起来正常的数字。扫频时把这些数字抄进表格，
 *   得到的就是一条完全错误的"占空比-频率"曲线，而且事后无法分辨哪一行是假的。
 *
 *   所以这条扫频行改用一个硬规则：
 *     - 量无效（freqValid/ampValid 为假，或数值不是有限值）就打印 '?'
 *     - 有效才打印数字
 *   '?' 在表格里一眼可见，抄表时不会混进假数据。
 *   这也意味着本函数**不能**复用 fin()，要自己判有效性。
 *
 * 【这一行打印什么、为什么】
 *   F     = 主频（Hz）        —— 扫频要记的第一个量，对应振动源的转速
 *   A     = 振动幅值（mg）     —— 扫频要记的第二个量，反映振动强度
 *   D     = 到平放姿态的距离（mg）—— 姿态通道的原始量，反映"有没有被握住"
 *   tr    = 基准极差（mg）      —— 平放基准采集质量；越小说明基准越干净
 *   vib   = 振动标志（0/1）     —— 固件的启停判定（包络超过 hi 阈值）
 *   ct    = 接触标志（0/1）     —— 固件的 Q4 判定结果，标定时可顺手核对
 *   clip  = 是否削顶（0/1）     —— 1 表示某个轴超出 ±4g 量程，
 *                                此时 A/F 都不可信（波形被削平后频谱会造假峰）
 *   ok    = 整条可用（0/1）     —— 上面任一量无效就是 0
 *
 * 【为什么不带换行外的任何前缀文字】
 *   扫频时敲一次 w 出一次，粘进表格/Excel 直接可用；
 *   带中文说明会破坏"一行一条"的可解析性。
 */
static void sweepLine(void) {
  const Result& r = stateResult();

  /* 逐项判断有效性：数值本身是有限值 && 固件给出的有效标记为真。
   * 两个条件都要，因为 freqValid 为真时理论上有值，但防御性检查一次成本极低。 */
  const bool fOk = r.freqValid && isfinite(r.freqHz);
  const bool aOk = r.ampValid  && isfinite(r.ampMg);
  const bool dOk = isfinite(r.flatDist) && (r.flatDist >= 0.0f);   // -1 = 尚未采到平放基准
  const bool sOk = isfinite(r.flatRefSwing) && r.flatRefSet;

  /* 整条是否可用：F 和 A 是扫频的核心量，缺任一就算不可用。
   * D 只用于辅助判断姿态，缺了不算整条废掉（平地测振动时本来就不需要 D）。 */
  const bool allOk = fOk && aOk && !r.freqClipLimited;

  /* 数值格式化的小工具：无效就写 '?'，有效才写数字。
   * 用两个并列分支而不是 printf 的 %*.*f，是为了让 '?' 与数字共用同一列宽，
   * 表格纵向对齐，一眼能看出哪些行是废的。 */
  char fBuf[12], aBuf[12], dBuf[12], sBuf[12];
  if (fOk) snprintf(fBuf, sizeof(fBuf), "%7.1f", r.freqHz); else snprintf(fBuf, sizeof(fBuf), "%7s", "?");
  if (aOk) snprintf(aBuf, sizeof(aBuf), "%8.1f", r.ampMg);  else snprintf(aBuf, sizeof(aBuf), "%8s", "?");
  if (dOk) snprintf(dBuf, sizeof(dBuf), "%6.0f", r.flatDist); else snprintf(dBuf, sizeof(dBuf), "%6s", "?");
  if (sOk) snprintf(sBuf, sizeof(sBuf), "%5.0f", r.flatRefSwing); else snprintf(sBuf, sizeof(sBuf), "%5s", "?");

  /* v5：闭合角度（度）。角度不可信时写 '?'，与 F/A 的语义一致。 */
  char angBuf[10];
  if (r.angleValid && isfinite(r.closeAngleDeg)) snprintf(angBuf, sizeof(angBuf), "%5.0f", r.closeAngleDeg);
  else                                          snprintf(angBuf, sizeof(angBuf), "%5s", "?");

  outPrintf("#SWEEP t=%lus F=%sHz A=%smg D=%smg ang=%sdeg tr=%smg vib=%u ct=%u grip=%u cl=%u w=%.1f cliplim=%u ax=%u ok=%u\n",
            (unsigned long)(millis() / 1000u),
            fBuf, aBuf, dBuf, angBuf, sBuf,
            (unsigned)r.vib,
            (unsigned)r.contact,
            (unsigned)(r.gripNoObject ? 1u : 0u),   /* 空夹：握下但未证明接触 */
            (unsigned)(r.closed ? 1u : 0u),         /* 已闭合（姿态门过） */
            (double)r.gyroW,                         /* 合成角速度 dps */
            (unsigned)(r.freqClipLimited ? 1u : 0u),/* 主频因削顶降级 */
            (unsigned)r.axis,                        /* 主频取用的轴 */
            (unsigned)(allOk ? 1u : 0u));
}

#if POSTURE_ENABLE
static void postureLine(void) {
  const PostureResult& p = postureResult();
  outPrintf("#POST cal=%u act=%u valid=%u cls=%u d=%.1f x=%.1f y=%.1f z=%.1f "
            "bx=%.1f by=%.1f bz=%.1f clsd=%u stable=%u scnt=%u n=%lu\n",
            (unsigned)(p.calValid ? 1u : 0u),
            (unsigned)(p.calActive ? 1u : 0u),
            (unsigned)(p.valid ? 1u : 0u),
            (unsigned)p.cls,
            (double)p.distMg,
            (double)p.x, (double)p.y, (double)p.z,
            (double)p.startX, (double)p.startY, (double)p.startZ,
            (unsigned)p.stableCls,
            (unsigned)(p.stable ? 1u : 0u),
            (unsigned)p.stableCount,
            (unsigned long)p.blockN);
}
#endif

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
#if CT_MODE == 3 || CT_MODE == 4
        outPrintf("      t 记录平放基准(平放时按) | t0 清除基准\n");
#else
        outPrintf("      t 记录接触姿态(握住时按) | t0 清除参考点\n");
#endif
        outPrintf("      P 重采 START 姿态基线 | S 状态\n");
        outPrintf("      Q 单行姿态诊断帧（便于连续记录）\n");
        outPrintf("      k0~k6 打标模式(仅CSV) | K 退出打标\n");
        outPrintf("      w 扫频采样：打一行带有效标记的标定数据(无效量显示?)\n");
        break;
      case '0': case '1': case '2': case '3': case '4':
        setPlot((uint8_t)(c - '0'));
        break;
      case 'p': setPlot((uint8_t)(s_plot + 1)); break;
      /* 'w' = 扫频采样：打印一行带有效性标记的标定数据（见 sweepLine 注释）。
       * 为什么单独一条命令：plotLine 的 CSV 丢掉了有效性标记，
       * 无效数据会被 fin() 变成 0.0，扫频时无法分辨真假。 */
      case 'w': sweepLine(); break;
      case 'c':
        setPlot(PLOT_OFF);
        act = CMD_RECAL;
        break;
      case 'd':
        s_dbg = !s_dbg;
        if (s_dbg) s_plot = PLOT_OFF;
        break;
      case 'S': setPlot(PLOT_OFF); outPrintStatus(); break;
      case 'T': setPlot(PLOT_OFF); outPrintTiming(); break;
      case 'Q':
#if POSTURE_ENABLE
        postureLine();
#else
        outPrintf("#POST disabled\n");
#endif
        break;
      case 'P':
#if POSTURE_ENABLE
        if (!stateResult().angleValid) {
          outPrintf("#P 拒绝：闭合角尚无效，请等待角度有效并回到 START\n");
        } else if (stateResult().closeAngleDeg >= AUTO_ZERO_ANGLE_DEG) {
          outPrintf("#P 拒绝：当前闭合角 %.0f°，请回到 START（<%.0f°）再按 P\n",
                    (double)stateResult().closeAngleDeg, AUTO_ZERO_ANGLE_DEG);
        } else if (stateResult().ctDev > POSTURE_START_MAX_CTDEV_MG) {
          outPrintf("#P 拒绝：当前三轴姿态偏差 %.0f mg，超过 START 门限 %.0f mg\n",
                    (double)stateResult().ctDev, POSTURE_START_MAX_CTDEV_MG);
        } else if (postureStartCal()) {
          outPrintf("#开始采集 START 姿态：保持 2 s 不动\n");
        } else {
          outPrintf("#正在采集 START 姿态，请稍等\n");
        }
#else
        outPrintf("#姿态诊断未启用\n");
#endif
        break;
      /* ---- 现场标定入口（按判据模式分发）----
       * 两种模式都需要"现场采一次姿态基准"，但采的东西不同：
       *   CT_MODE=2：采"接触姿态"——选手**握住**测试件时按（有物体可抓的前提下才成立）
       *   CT_MODE=3：采"平放姿态"——选手把设备**平放在桌面**时按（不需要抓着任何东西）
       * 为什么 v4 要换基准：模式 2 要求先抓住物体才能记录参考点，
       * 而"完全握住但没夹到物体"这种情况根本没有可抓的东西，基准就记不下来。
       * 平放姿态永远存在，所以 v4 拿它当零点。
       * t  = 记录基准（模式 2 握住 / 模式 3 平放）
       * t0 = 清除基准（换机械手/换工位/换装夹姿态时用）
       * 为什么必须现场采而不能写死：姿态基准取决于机械手型号、装夹位置、工位安装角度，
       * 换一套就完全失效。 */
      case 't': {
        bool clear = false;
        if (OUT.available()) {
          char d = (char)OUT.peek();
          if (d == '0') { OUT.read(); clear = true; }
        }
#if CT_MODE == 3 || CT_MODE == 4
        if (clear) {
          stateClearFlatRef();
          outPrintf("#已清除平放基准（接触判定回到不可信，将自动重采）\n");
        } else if (stateStartFlatRef()) {
          outPrintf("#开始采集平放基准：等待 %dms 稳定 + 平均 %dms，请保持平放不动\n",
                    CT_REF_SETTLE_MS, CT_REF_AVG_MS);
        } else {
          outPrintf("#正在采集中，请稍等\n");
        }
#else
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
#endif
        break;
      }
      case 'x':
        samplerClearStats(); mpuClearErr(); stateClearCounters();
        samplerTimingReset();
        outPrintf("#统计已清零（含时序），开始新一轮测量\n");
        break;

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

      default: break;
    }
  }
  return act;
}
