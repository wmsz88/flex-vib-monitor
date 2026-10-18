// 状态判断模块：振动启停、启停频率、接触状态、结果有效性与故障清零。
#pragma once
#include <stdint.h>
#include "dsp.h"

struct Thresholds {
  float hi, lo;        // 启停双阈值 mg
  float ctOn, ctOff;   // 接触滞回阈值 mg
};

struct Result {
  bool    sensorOk;    // 传感器在线
  bool    calOk;       // 标定完成
  bool    calQuality;  // 标定时确实静置
  uint8_t vib;         // 振动状态 0=停 1=振
  bool    freqValid;
  bool    freqClipLimited;  // 主频因削顶被降级（有效但需打折）；与 freqValid 解耦
  float   freqHz;      // 持续振动主频；无效时为 0
  float   ampMg;       // 合成 RMS 包络（数值保留，可信度看 ampValid）
  bool    ampValid;    // 幅值是否反映当前时刻；缺口后或超时为 false
  bool    ssValid;
  float   ssHz;        // 启停频率；未确立/超时为 0
  uint8_t ssPeriods;   // 当前参与统计的周期数
  uint8_t contact;     // 接触状态 0/1（状态量，非瞬时测量值）
  bool    contactValid;// 接触字段是否可信。判据未经真实标签数据验证前恒为 false，
                       // 上位机据此区分"未接触"与"不知道"。
  float   ctDev;       // 姿态偏差 mg
  uint8_t axis;        // 主频估计所用轴
  uint32_t nStart, nStop;
};

void stateApplyCal(const DspCal& cal);
void stateReset(void);                         // 清结果与计数（不清阈值）
void stateOnFrame(const DspFrame& f, uint32_t tUs);
void stateOnGap(const SamplerGap& g);          // 缺样通知：主频失效/打断确认链/作废读数
void stateTick(uint32_t nowUs);                // 超时检查：主频过期、启停频率超时、幅值过期
void stateSetSensorOk(bool ok);                // 断线时清掉所有有效结果
void stateClearCounters(void);
const Result&     stateResult(void);
const Thresholds& stateThresholds(void);

/* ---- 测试钩子：正式固件不定义 STATE_TEST_HOOKS，以下不参与编译 ---- */
#ifdef STATE_TEST_HOOKS
// 取最近的启停周期快照（毫秒），返回实际个数。用于验证周期一致性。
uint8_t stateTestPeriods(float* out, uint8_t maxN);
// 直接注入一个包络点，绕过 DSP 链路。
// T07j 用它隔离"边沿插值"的效果：走完整 DSP 时，快/慢上升沿的包络形状本身不同，
// 测到的是"形状差异 + 插值误差"的混合结果，无法单独证明插值有效。
void stateTestInjectEnv(float env, uint32_t tUs);
// 最近一次记录的"起"边沿时刻（微秒）。用于验证缺口不会插出虚假边沿。
bool     stateTestLastEdgeUs(uint32_t* out);
#endif
