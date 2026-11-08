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
  bool    contactValid;// 接触字段是否可信。
                       //   CT_MODE=2 且已记录接触姿态 → true
                       //   其余情况（含未记录参考点）→ false
                       // 上位机据此区分"未接触"与"不知道"。
  float   ctDev;       // 姿态偏差 mg（相对标定零偏）
  float   ctTouchDist; // 【模式2】到接触姿态的距离 mg；未记录参考点时为 -1
  bool    ctRefSet;    // 【模式2】是否已记录接触姿态参考点
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

/* ---- 模式 2：接触姿态参考点 ----
 * 为什么要记录而不是写死：接触姿态取决于机械手型号、装夹位置、工位姿态，
 * 换一套就完全失效。比赛是手动操作，所以由选手在握住测试件时按键记录。
 *
 * 【两阶段采集，不阻塞主循环】
 *   按 't' 后先等 CT_REF_SETTLE_MS 让手到位，再平均 CT_REF_AVG_MS 的样本。
 *   分两次调用实现：
 *     stateStartTouchRef()  —— 启动采集（返回是否接受）
 *     stateTickTouchRef()   —— 由主循环每圈调用，负责结算完成
 *   采样累加在 stateOnFrame 内部完成（逐样本，1kHz）。
 *
 *   为什么不像标定那样阻塞等待：主循环阻塞期间采样回调仍在跑，
 *   会与 I2C 总线竞争（标定路径为此专门做了 samplerStop）。
 *   这里保持采样器运行、纯累加，不需要停总线。
 */
bool  stateStartTouchRef(void);      // 启动参考点采集；返回是否接受
bool  stateTouchRefCapturing(void);  // 是否正在采集
void  stateTickTouchRef(uint32_t nowUs);  // 主循环每圈调用，结算完成
void  stateClearTouchRef(void);      // 清除参考点（此后 contactValid 回到 false）
bool  stateTouchRefSet(void);        // 是否已记录

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
