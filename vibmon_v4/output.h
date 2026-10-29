// 输出模块：串口绘图行、调试文字、LED、调试命令解析。
// 当前协议未定，只输出 Arduino 串口绘图器格式（label:value）。比赛帧以后加在这里。
#pragma once
#include <stdint.h>
#include "dsp.h"
#include "state.h"

enum PlotGroup : uint8_t {
  PLOT_OFF = 0,
  PLOT_RESULT,       // 1 主频 / 幅值 / 启停频率 / 振动 / 接触
  PLOT_SS,           // 2 包络与启停阈值
  PLOT_CONTACT,      // 3 姿态偏差与接触阈值
  PLOT_WAVE,         // 4 高通后三轴波形（每个样本一行，需高波特率或 USB CDC）
  PLOT_COUNT
};

enum LedMode : uint8_t { LED_RUN, LED_CAL, LED_FAULT };

enum CmdAction : uint8_t { CMD_NONE, CMD_RECAL };

void outBegin(void);
// ---- 比赛口（Serial1 / UART1）----
// 与调试口完全解耦：不受 ARDUINO_USB_CDC_ON_BOOT 影响。
// 比赛口默认静默：只发正式帧，不发任何自检/横幅/调试文字。
void compBegin(void);                         // 由 outBegin 调用
bool compReady(void);
// 组装一帧（纯函数，便于主机测试）。
// 返回写入的字节数；<0 表示失败。out 里不含结尾的 \0。
int  compBuildFrame(const Result& r, uint16_t seq, char* out, int outSize);
void compPeriodic(uint32_t nowMs);            // 调用点；COMP_FRAME_ENABLED=0 时编译为空
void outPrintf(const char* fmt, ...);         // 文字信息（启动、标定、命令回显）
void outPrintBanner(void);
void outPrintCal(void);                       // 本次尝试 + 当前生效配置两段
void outPrintStatus(void);
void outPrintTiming(void);                    // 时序统计（'T' 命令）
void outOnSample(const DspFrame& f);          // 波形组逐样本输出（受抽取比控制）
void outSetWaveDecim(uint8_t decim);          // 0=关，1=不抽取，N=N抽1
uint32_t outWaveDropped(void);
void outPeriodic(uint32_t nowMs);             // 绘图行 / 调试行定时输出
void ledSet(LedMode m);
void ledUpdate(uint32_t nowMs, const Result& r);
CmdAction outPollCmd(void);

/* ---- Q4 打标模式（T10 接触判据数据采集）----
 * 目的：为"接触判据"采集带人工标签的数据。
 *
 * 为什么必须独占串口：打标期间若同时输出绘图行（Freq_Hz:...）、调试行（#F=...）
 * 或三轴波形（X:...），CSV 里会混入这些行，离线解析要额外处理且容易误判。
 * 所以进入打标模式时强制关掉它们，只输出一条 CSV。
 *
 * 输出速率：每 10ms 一行（跟随包络输出，100Hz）。1kHz 逐样本输出数据量会大 100 倍。
 * 命令：k0~k6 设标签（自动进入打标模式）| K 退出
 */
void outEnterLabelMode(void);
void outExitLabelMode(void);
bool outInLabelMode(void);
