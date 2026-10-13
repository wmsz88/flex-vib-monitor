// 状态判断模块：振动启停、启停频率、接触状态、结果有效性与故障清零。
//
// 【本模块在全工程中的位置】
// dsp 只负责"算出数字"，本模块负责"下判断"：现在算不算在振动？多久振动一次？
// 手有没有按上去？这些判断都带滞回、去抖和超时逻辑，因此单独成模块。
//
// 【三个独立的输出维度】彼此正交，不要混为一谈：
//   ① vib    振动状态（0/1）—— 由包络与双阈值判定，回答"现在振不振"
//   ② freqHz 持续振动主频 —— 由 dsp 的主频窗给出，回答"振多快"（需 vib=1 才有效）
//   ③ ssHz   启停频率     —— 由"起"边沿的周期统计得出，回答"多久振一次"
//   ④ contact 接触状态（0/1）—— 由姿态偏差（或振动）判定，回答"手有没有按上去"
#pragma once
#include <stdint.h>
#include "dsp.h"

// 运行期阈值。由 stateApplyCal() 从标定结果算出，不直接取自 config.h 的宏
// （宏只提供倍数与下限，实际数值 = 倍数 × 实测静置值，见 stateApplyCal）。
struct Thresholds {
  float hi, lo;        // 启停双阈值 mg
                       // 上升穿越 hi 才算"起"，下降穿越 lo 才算"停"。hi>lo 形成滞回带，
                       // 防止包络在阈值附近抖动时状态反复翻转
  float ctOn, ctOff;   // 接触滞回阈值 mg
                       // 同理：ctOn 触发接触，ctOff 释放接触
};

// 对外输出的完整结果。output 模块只读这个结构体，不做任何加工。
struct Result {
  bool    sensorOk;    // 传感器在线
  bool    calOk;       // 标定完成
  bool    calQuality;  // 标定时确实静置
  uint8_t vib;         // 振动状态 0=停 1=振
  bool    freqValid;   // 主频是否有效（vib=1 且最近有新的主频窗结果）
  float   freqHz;      // 持续振动主频；无效时为 0
  float   ampMg;       // 合成 RMS 包络
  bool    ssValid;     // 启停频率是否已确立
  float   ssHz;        // 启停频率；未确立/超时为 0
  uint8_t ssPeriods;   // 当前参与统计的周期数
  uint8_t contact;     // 接触状态 0/1（与振动状态是两个独立字段）
  float   ctDev;       // 姿态偏差 mg
  uint8_t axis;        // 主频估计所用轴
  uint32_t nStart, nStop;   // 累计"起"/"停"次数，用于现场核对操作次数
};

void stateApplyCal(const DspCal& cal);         // 由标定结果算出全部阈值
void stateReset(void);                         // 清结果与计数（不清阈值）
void stateOnFrame(const DspFrame& f, uint32_t tUs);   // 每样本喂入，内部只在 Ready 时动作
void stateTick(uint32_t nowUs);                // 超时检查：主频过期、启停频率超时
void stateSetSensorOk(bool ok);                // 断线时清掉所有有效结果
void stateClearCounters(void);                 // 清 nStart/nStop（响应 'n' 命令）
const Result&     stateResult(void);
const Thresholds& stateThresholds(void);
