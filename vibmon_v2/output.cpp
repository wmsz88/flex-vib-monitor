// 输出模块实现：串口三种格式、LED 指示、命令解析。
//
// 【输出带宽是硬约束】这是全工程最容易拖垮采样的一环：
// 主循环和采样定时器共享 CPU，串口打印是同步阻塞的。三轴波形组每样本一行
// （约 25 字节 × 1kHz = 25KB/s），远超 115200bps（≈11.5KB/s），会把主循环拖到
// 环形缓冲溢出。因此波形组在 UART 模式下被强制禁用（见 setPlot）。
// 排障口诀：看到 SamplerStats.ringDrop 增长，先降输出频率。
#include "output.h"
#include "config.h"
#include "mpu.h"
#include "sampler.h"
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

// 勾选 "USB CDC On Boot" 时 Serial 是 USB 口；否则 Serial 是 UART0，绑到 GPIO20/21
// 两种模式的区别：
//   USB-CDC：波特率只是形式，实际带宽高得多，且能承受波形组；但拔插会重枚举
//   UART  ：固定 115200bps，波形组带宽不够
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  #define OUT_USB_CDC 1
#else
  #define OUT_USB_CDC 0
#endif
#define OUT Serial    // 统一入口。想换串口只改这一处

/* ---------- 输出状态 ---------- */
static uint8_t s_plot = PLOT_DEFAULT;   // 当前绘图组（0 = 不绘图）
static bool    s_dbg = false;           // 调试文字行开关。与绘图组互斥（两者混在同一串口会打乱绘图器）
static LedMode s_led = LED_RUN;         // 当前 LED 模式，由 ledSet 切换

// 绘图组编号 → 中文名，仅用于状态打印。数组长度由 PLOT_COUNT 约束，
// 新增绘图组时编译器会因初始化项不足而告警，避免漏改。
static const char* const kPlotName[PLOT_COUNT] = {"关", "结果", "启停包络", "接触", "三轴波形"};

// 初始化串口与 LED 引脚。上电调用一次。
void outBegin(void) {
#if OUT_USB_CDC
  OUT.begin(BAUD);
#else
  // UART 模式需显式指定引脚（ESP32-C3 的默认串口脚与 PCB 走线不一致）
  OUT.begin(BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
#endif
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_Y, OUTPUT);
}

// 格式化打印（printf 风格，最长 255 字符）。
// 【为什么不用 Serial.printf】需要控制截断行为：vsnprintf 返回"本应写入的长度"，
// 超出缓冲时会被截断，这里把长度夹回缓冲区内再 write，避免越界读。
// 用 write(buf,n) 而不是 print(buf)：前者按长度写出，能正确处理缓冲区里的 \0 之前的全部内容。
void outPrintf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;                                        // 格式化失败
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;         // 夹回，防止 write 越界
  OUT.write((const uint8_t*)buf, (size_t)n);
}

// 浮点安全化：串口绘图器无法处理 inf/nan，会让整条曲线断掉或显示异常。
// 所有要打印的 float 都过一遍这个函数。
static float fin(float v) { return isfinite(v) ? v : 0.0f; }

// 上电横幅。打印关键配置，作用是在现场快速确认"板子里烧的是哪份固件、参数对不对"，
// 不必去翻源码。这些值与 config.h 一一对应。
void outPrintBanner(void) {
  outPrintf("\n===== vibmon v2.0 (ESP32-C3 + MPU6050) =====\n");
  outPrintf("采样 %d Hz | 窗 %d/%d | 高通 %.0f Hz | 包络 %d ms | %d bps %s\n",
            FS_HZ, BLOCK_N, HOP, HP_FC_HZ, ENV_WIN * 1000 / FS_HZ, BAUD,
            OUT_USB_CDC ? "USB-CDC" : "UART GPIO21/20");
  outPrintf("命令 '?' 查看帮助\n");
}

// 打印完整标定结果。这些数字是判断"读数为什么不对"的第一手依据：
//   ok/quality     标定是否可用、是否在静置状态下完成
//   bias / |g|     零偏与模长（应≈1000mg），异常说明模块没放平
//   std            标定期间的标准差，偏大说明标定时被碰过
//   带内 sigma     噪声底，决定了过零阈值
//   静置包络/姿态  启停与接触阈值的计算基准
//   阈值行         最终生效的阈值，可与上面的基准值对照验证倍数关系
void outPrintCal(const DspCal& c) {
  const Thresholds& t = stateThresholds();
  outPrintf("[CAL] %s  质量 %s  样本 %lu\n", c.ok ? "完成" : "失败",
            c.quality ? "合格" : "可疑(标定时可能被碰或未静置)", (unsigned long)c.nBias);
  outPrintf("[CAL] bias %.1f/%.1f/%.1f mg  |g|=%.0f mg  std %.1f/%.1f/%.1f mg\n",
            c.bias[0], c.bias[1], c.bias[2], c.biasNormMg, c.rawStd[0], c.rawStd[1], c.rawStd[2]);
  outPrintf("[CAL] 带内 sigma %.2f/%.2f/%.2f mg  静置包络 %.2f  静置姿态 %.2f mg\n",
            c.sigBand[0], c.sigBand[1], c.sigBand[2], c.envRest, c.ctRest);
  outPrintf("[CAL] 阈值 过零 %.1f | 启停 %.1f/%.1f | 接触 %.1f/%.1f mg\n",
            c.zcl, t.hi, t.lo, t.ctOn, t.ctOff);
}

// 打印完整运行状态（响应 'S' 命令）。分四块：
//   ① 系统配置：传感器/标定/当前绘图组/调试行开关
//   ② 测量结果：主频、幅值、振动、启停、接触、主轴
//   ③ 采集质量：起停次数、读失败、缓冲溢出、不连续、削顶、I2C 错误
//   ④ 标定详情（复用 outPrintCal）
// 排障顺序建议：先看③判断采集是否可靠，再看①的标定是否合格，最后才怀疑②的算法结果。
void outPrintStatus(void) {
  const Result& r = stateResult();
  const SamplerStats& s = samplerStats();
  outPrintf("\n--- 状态 ---\n");
  outPrintf("传感器 %s  标定 %s/%s  绘图组 %u(%s)  调试行 %s\n",
            r.sensorOk ? "在线" : "故障", r.calOk ? "完成" : "未完成", r.calQuality ? "合格" : "可疑",
            (unsigned)s_plot, kPlotName[s_plot], s_dbg ? "开" : "关");
  // 'X' + axis 把 0/1/2 映射成 X/Y/Z
  outPrintf("主频 %.1f Hz(%s)  幅值 %.1f mg  振动 %u  启停 %.2f Hz(%s, %u 周期)  接触 %u  主轴 %c\n",
            fin(r.freqHz), r.freqValid ? "有效" : "无效", fin(r.ampMg), (unsigned)r.vib,
            fin(r.ssHz), r.ssValid ? "有效" : "未确立", (unsigned)r.ssPeriods, (unsigned)r.contact,
            'X' + r.axis);
  outPrintf("起 %lu / 停 %lu | 读失败 %lu  缓冲溢出 %lu  不连续 %lu(缺 %lu)  削顶 %lu  I2C错 %lu\n",
            (unsigned long)r.nStart, (unsigned long)r.nStop, (unsigned long)s.readFail,
            (unsigned long)s.ringDrop, (unsigned long)s.gaps, (unsigned long)s.missed,
            (unsigned long)s.clip, (unsigned long)mpuErrCount());
  outPrintCal(dspCal());
}

// 波形组专用：每个样本输出一行三轴高通后的值。
// 只在 PLOT_WAVE 下生效；此函数由主循环对每个样本调用，因此这里的判断必须便宜。
void outOnSample(const DspFrame& f) {
  if (s_plot != PLOT_WAVE) return;
  outPrintf("X:%.1f,Y:%.1f,Z:%.1f\n", f.fx, f.fy, f.fz);
}

// 生成一行绘图数据。三种绘图组各自输出一组 label:value，
// 但共用同一个时间轴，便于在绘图器里对照观察。
// 【两个技巧】
//   ① 所有量都换算到相近的数量级再输出：串口绘图器是单纵轴，若主频(百)和振动标志(0/1)
//      直接混在一起，小信号会被压成一条平线。所以振动 ×50、接触 ×100、启停频率 ×10，
//      让各曲线都能看清（图例上标注了 _x50 之类的倍数，读图时要还原）
//   ② 布尔量用"乘法映射成参考线附近的数值"而不是 0/1：例如 PLOT_SS 里振动映射到
//      ThHi×2，这样振动标志线就贴在阈值线附近，一眼能看出翻转点与阈值的关系
static void plotLine(void) {
  const Result& r = stateResult();
  const Thresholds& t = stateThresholds();
  switch (s_plot) {
    case PLOT_RESULT:
      outPrintf("Freq_Hz:%.1f,Amp_mg:%.1f,SS_Hz_x10:%.1f,Vib_x50:%d,Contact_x100:%d\n",
                fin(r.freqHz), fin(r.ampMg), fin(r.ssHz) * 10.0f, r.vib * 50, r.contact * 100);
      break;
    case PLOT_SS:
      // 包络与双阈值同图：直接看出滞回带的宽度和翻转时机
      outPrintf("Env_mg:%.2f,ThHi:%.2f,ThLo:%.2f,Vib:%.1f\n",
                fin(r.ampMg), t.hi, t.lo, r.vib * t.hi * 2.0f);
      break;
    case PLOT_CONTACT:
      // 姿态偏差与接触双阈值同图
      outPrintf("CtDev_mg:%.2f,CtOn:%.2f,CtOff:%.2f,Contact:%.1f\n",
                fin(r.ctDev), t.ctOn, t.ctOff, r.contact * t.ctOn * 1.5f);
      break;
    default: break;
  }
}

// 生成一行紧凑调试文本，给人看（不是给绘图器看）。
// 每个量后面带 (x) 表示该项当前无效；末尾是采集质量计数。
// 字段含义：#F 主频 A 幅值 vib 振动 SS 启停 ct 接触 dev 姿态偏差 ax 主轴
//           sens 传感器在线 gap 不连续 drop 缓冲溢出 fail 读失败 clip 削顶
static void debugLine(void) {
  const Result& r = stateResult();
  const SamplerStats& s = samplerStats();
  outPrintf("#F=%.1f%s A=%.1f vib=%u SS=%.2f%s ct=%u dev=%.1f ax=%u sens=%u gap=%lu drop=%lu fail=%lu clip=%lu\n",
            fin(r.freqHz), r.freqValid ? "" : "(x)", fin(r.ampMg), (unsigned)r.vib,
            fin(r.ssHz), r.ssValid ? "" : "(x)", (unsigned)r.contact, fin(r.ctDev), (unsigned)r.axis,
            (unsigned)r.sensorOk, (unsigned long)s.gaps, (unsigned long)s.ringDrop,
            (unsigned long)s.readFail, (unsigned long)s.clip);
}

// 周期性输出。由主循环每圈调用，内部用静态变量分别计时。
// 【为什么用函数内 static】这两个时间戳只服务于本函数，放在这里可以避免
// 污染文件级作用域；缺点是首次调用时初值为 0，若 nowMs 恰好很小可能立即触发一次。
// 【波形组例外】PLOT_WAVE 的输出来自 outOnSample（每样本），不能在这里再打，否则会重复。
void outPeriodic(uint32_t nowMs) {
  static uint32_t tPlot = 0, tDbg = 0;
  if (s_plot != PLOT_OFF && s_plot != PLOT_WAVE && nowMs - tPlot >= PLOT_MS) { tPlot = nowMs; plotLine(); }
  if (s_dbg && nowMs - tDbg >= DBG_MS) { tDbg = nowMs; debugLine(); }
}

// 写 LED 引脚。LED_ACTIVE_LOW 决定"点亮"对应高电平还是低电平
// （灌电流接法下 GPIO 拉低才导通，见 config.h）。
static void ledWrite(int pin, bool on) {
#if LED_ACTIVE_LOW
  digitalWrite(pin, on ? LOW : HIGH);
#else
  digitalWrite(pin, on ? HIGH : LOW);
#endif
}

// 切换 LED 模式。LED_CAL 下立即把两个灯都点亮，作为"正在标定、别碰"的提示；
// 其余模式的具体闪烁由 ledUpdate 每圈刷新。
void ledSet(LedMode m) {
  s_led = m;
  if (m == LED_CAL) { ledWrite(PIN_LED_G, true); ledWrite(PIN_LED_Y, true); }
}

// 按当前模式刷新 LED。由主循环每圈调用，用 nowMs 做时间基准。
//   运行：绿 1Hz 心跳（亮 500ms 灭 500ms），黄 = 振动中
//   故障：绿灭，黄 4Hz 快闪——刻意与"振动中"区分开，避免把故障误读成正常运行
//   标定：两灯常亮（已在 ledSet 中点亮），此处不做任何事
void ledUpdate(uint32_t nowMs, const Result& r) {
  switch (s_led) {
    case LED_RUN:   // 绿 1Hz 心跳，黄 = 振动中
      ledWrite(PIN_LED_G, (nowMs / 500) & 1);   // 每 500ms 翻转一次 → 周期 1s
      ledWrite(PIN_LED_Y, r.vib == 1);
      break;
    case LED_FAULT: // 绿灭，黄 4Hz 快闪：避免把故障误读成"正常运行"
      ledWrite(PIN_LED_G, false);
      ledWrite(PIN_LED_Y, (nowMs / 125) & 1);   // 每 125ms 翻转 → 周期 250ms = 4Hz
      break;
    case LED_CAL:
      break;
  }
}

// 切换绘图组。
// 【取模】g % PLOT_COUNT 使 'p' 键轮换时自动回绕到 0，不必在命令处理里判边界。
// 【互斥】绘图行与调试行不能同时开：两者交替输出会让串口绘图器无法解析。
// 【UART 下禁用波形组】1kHz × ~25 字节远超 115200bps，会阻塞主循环导致缓冲溢出，
// 所以这里直接拒绝并提示改用 USB CDC，而不是让用户拿到一份充满丢样的数据。
static void setPlot(uint8_t g) {
  s_plot = g % PLOT_COUNT;
  if (s_plot != PLOT_OFF) s_dbg = false;   // 文字行会打乱绘图器
#if !OUT_USB_CDC
  // 1kHz × ~25 字节远超 115200bps，会阻塞主循环导致缓冲溢出
  if (s_plot == PLOT_WAVE) {
    s_plot = PLOT_OFF;
    outPrintf("#三轴波形需 USB CDC（工具→USB CDC On Boot→Enabled），UART 模式下不可用\n");
  }
#endif
}

// 收取并处理串口命令。返回需要主循环执行的动作（重标定），其余命令就地处理完。
// 【为什么返回动作而不是直接执行】重标定会停止采样器、影响其他模块，
// 由主循环统一编排更安全，也避免 output 反向依赖业务模块。
// 【为什么用 while 循环】一次把缓冲里的字符全部读完，防止积压导致后续命令延迟。
// 命令表：0~4 选绘图组 | p 轮换 | d 调试行 | S 状态 | c 重标定 | n 清零统计 | ? 帮助
CmdAction outPollCmd(void) {
  CmdAction act = CMD_NONE;
  while (OUT.available()) {
    char c = (char)OUT.read();
    switch (c) {
      case '?':
        setPlot(PLOT_OFF);        // 帮助文字必须独占串口，先关绘图
        outPrintf("命令: 0~4 选绘图组(0关 1结果 2启停包络 3接触 4三轴波形) | p 轮换绘图组\n"
                  "      d 调试文字行 | S 状态 | c 重新标定(须静置) | n 清零统计 | ? 帮助\n");
        break;
      case '0': case '1': case '2': case '3': case '4':
        // 用字符差值直接得到组号，避免写 5 个几乎相同的分支
        setPlot((uint8_t)(c - '0'));
        break;
      case 'p': setPlot((uint8_t)(s_plot + 1)); break;   // 轮换，靠 setPlot 里的取模回绕
      case 'd':
        s_dbg = !s_dbg;
        if (s_dbg) s_plot = PLOT_OFF;   // 反向互斥：开调试行则关绘图
        break;
      case 'S': setPlot(PLOT_OFF); outPrintStatus(); break;
      case 'c': act = CMD_RECAL; break;   // 交给主循环执行
      case 'n':
        // 三个模块的统计各自清零（计数口径不同，不能在一个地方统一清）
        samplerClearStats(); mpuClearErr(); stateClearCounters();
        outPrintf("#统计已清零\n");
        break;
      default: break;   // 未知字符静默忽略（换行、回车等都会走到这里）
    }
  }
  return act;
}
