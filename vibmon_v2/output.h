// 输出模块：串口绘图行、调试文字、LED、调试命令解析。
// 当前协议未定，只输出 Arduino 串口绘图器格式（label:value）。比赛帧以后加在这里。
//
// 【本模块的职责】
// 把人能看的东西都集中在这里：串口打印（三种格式）、LED 指示、以及从串口收命令。
// 它是唯一允许做"格式化输出"的模块，其他模块只提供数据。
// 【三种串口输出格式】互斥使用，不能同时开：
//   ① 绘图行（plotLine）   周期性单行 label:value，给 Arduino 串口绘图器画曲线
//   ② 调试行（debugLine）  周期性单行紧凑文本，给人快速扫一眼
//   ③ 事件文字（outPrintf）非周期性，仅在启动/标定/报错/命令回显时输出
//   ④ 波形组（outOnSample）每样本一行，仅在 USB-CDC 下可用（见 output.cpp 的 setPlot）
#pragma once
#include <stdint.h>
#include "dsp.h"
#include "state.h"

// 绘图组：决定绘图行打印哪些量。用数字键 0~4 切换。
enum PlotGroup : uint8_t {
  PLOT_OFF = 0,      // 关闭绘图
  PLOT_RESULT,       // 1 主频 / 幅值 / 启停频率 / 振动 / 接触
  PLOT_SS,           // 2 包络与启停阈值
  PLOT_CONTACT,      // 3 姿态偏差与接触阈值
  PLOT_WAVE,         // 4 高通后三轴波形（每个样本一行，需高波特率或 USB CDC）
  PLOT_COUNT         // 组数，用于取模轮换（p 键）
};

// LED 工作模式。决定 ledUpdate 如何驱动两个灯。
enum LedMode : uint8_t { LED_RUN, LED_CAL, LED_FAULT };

// 串口命令解析结果。返回给主循环执行（避免 output 直接反向调用业务模块）。
enum CmdAction : uint8_t { CMD_NONE, CMD_RECAL };

void outBegin(void);
void outPrintf(const char* fmt, ...);         // 文字信息（启动、标定、命令回显）
void outPrintBanner(void);                    // 上电横幅：打印关键配置，便于核对固件版本
void outPrintCal(const DspCal& cal);          // 打印完整标定结果（排障主依据）
void outPrintStatus(void);                    // 打印完整运行状态（响应 'S' 命令）
void outOnSample(const DspFrame& f);          // 波形组逐样本输出
void outPeriodic(uint32_t nowMs);             // 绘图行 / 调试行定时输出
void ledSet(LedMode m);                       // 切换 LED 模式
void ledUpdate(uint32_t nowMs, const Result& r);   // 按模式刷新 LED（每圈调用）
CmdAction outPollCmd(void);                   // 收取串口命令，返回需要主循环执行的动作
