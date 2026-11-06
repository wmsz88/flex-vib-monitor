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
  bool    freqValid;   // 【模式3】三重与的第(3)路是否可用
  bool    freqClipLimited;  // 主频因削顶被降级（有效但需打折）；与 freqValid 解耦
  float   freqHz;      // 持续振动主频；无效时为 0。【模式3】用它跟 CT_FLAT_F_ON_HZ 比
  float   ampMg;       // 合成 RMS 包络（数值保留，可信度看 ampValid）
  bool    ampValid;    // 幅值是否反映当前时刻；缺口后或超时为 false。【模式3】第(2)路
  bool    ssValid;
  float   ssHz;        // 启停频率；未确立/超时为 0
  uint8_t ssPeriods;   // 当前参与统计的周期数
  uint8_t contact;     // 接触状态 0/1（状态量，非瞬时测量值）
  bool    contactValid;// 接触字段是否可信。
                       //   CT_MODE=2 且已记录接触姿态 → true
  //   CT_MODE=3/4 且已记录平放基准 → true
  //     （CT_MODE=4 角度有效时用闭合角；角度失效时用 flatDist 兜底）
                       //   其余情况（含未记录参考点、采集被判"在动"）→ false
                       // 上位机据此区分"未接触"与"不知道"。
  float   ctDev;       // 姿态偏差 mg（相对标定零偏）
  float   ctTouchDist; // 【模式2】到接触姿态的距离 mg；未记录参考点时为 -1
  bool    ctRefSet;    // 【模式2】是否已记录接触姿态参考点

  /* ---------- 模式 3：平放三重与（v4 新增）----------
   * 为什么要把三路的中间量都输出：
   *   三重与判据一旦出现"该接触却没接触"或"不该接触却接触"，
   *   现场必须能一眼看出是**哪一路**没满足。只输出最终的 contact
   *   等于把三条判据的全部信息压缩成一位，无法调试。
   *   比赛前的阈值采样也依赖这三路同时打印（见 README「阈值采样」）。
   */

  // 【模式3】(1) 姿态：当前到**平放**姿态的距离 mg。
  //   未记录平放基准时为 -1，与模式2的 ctTouchDist 同一约定。
  float   flatDist;
  // 【模式3】平放基准是否已记录。与 ctRefSet（模式2的接触姿态）并列，
  //   两者语义不同，不能混用，故各自一个标志。
  bool    flatRefSet;
  // 【模式3】采集平放基准时窗口内的最大单轴极差 mg。
  //   超过 CT_REF_MAX_SWING_MG 说明采集时设备在动，基准不可信。
  //   这个值保留下来而不是丢弃，现场可直接看出"当时动了多厉害"。
  float   flatRefSwing;
  // 【模式3】采集失败的自动重试已用次数 / 上限。
  uint8_t flatRefRetry;
  // 【模式3】三路判据的即时结果 0/1，**不含滞回与确认**，是原始比较结果。
  //   调试时看它就知道"现在到底哪一路卡住了"。
  uint8_t condPosture;  // (1) 姿态：flatDist > CT_FLAT_D_ON_MG
  uint8_t condAmp;      // (2) 幅值：ampValid && ampMg > CT_FLAT_A_ON_MG
  uint8_t condFreq;     // (3) 频率：freqValid && freqHz > CT_FLAT_F_ON_HZ
  // 【模式3】空夹：姿态已握下（condPosture）但有振动没能证明接触。
  //   即"机械手合上了，却大概率没夹到物体"。不参与 contact，
  //   仅供现场判断是"没夹到"还是"根本没动"——这两者的处置完全不同：
  //   没夹到 → 调整物体位置；没动 → 检查激励是否给了。
  bool    gripNoObject;
  uint8_t axis;        // 主频估计所用轴

  /* ---------- v5：闭合角度（陀螺积分） ---------- */
  // 从"张开零位"起累加角速度，取三轴积分绝对值最大者。
  // 与 flatDist 分工：flatDist 认零位（绝对、不漂），本值量"转了多少"（分辨率高）。
  float   closeAngleDeg;  // 无效时为 0，可信度看 angleValid
  bool    angleValid;     // 有陀螺数据且未超时、未削顶
  bool    gyroSeen;       // 本次上电以来是否收到过陀螺数据
  bool    closed;         // 已闭合（姿态门过）。与 contact 独立：
                          //   closed=1 且 contact=0 表示"闭合了但没有振动证据"
  float   gyroW;          // 当前合成角速度 dps（诊断用，判断是否静止）

  /* ---------- v6：接触三条（CT_V6_ENABLE） ---------- */
  float   gyroLpW;        // 低通后合成角速度 dps：去掉振动后的"手指转动速度"
  bool    settled;        // 手指已停下（gyroLpW < CT_SETTLE_DPS 持续 CT_SETTLE_MS，或陀螺削顶）
  float   ctMovedDeg;     // 接触后累计转过的角度；> CT_RELEASE_DEG 判分离

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

/* ---- 模式 3：平放基准（v4 新增）----
 * 【与模式 2 的接触姿态不是同一个东西】
 *   模式 2 记录的参考点是"握住测试件时的姿态"，判据是"离它多远"。
 *   模式 3 记录的参考点是"平放静置时的姿态"，判据是"离它多远"。
 *   前者需要选手在握持状态下按键，后者设备平放即可，可自动采集。
 *
 * 【自动采集的时序】由主循环每圈调用 stateTickFlatRef(nowUs) 推进：
 *   上电后 CT_REF_AUTO_ENABLE=1 且尚未采集时自动启动 →
 *   等待 CT_REF_SETTLE_MS → 累加 CT_REF_AVG_MS →
 *   结算：极差超限则作废并重试（上限 CT_REF_AUTO_RETRY 次），
 *   成功则置 flatRefSet=true、contactValid 变为 true。
 *
 * 【失败后的行为】
 *   重试用尽仍不稳定 → flatRefSet 保持 false，contactValid 保持 false，
 *   contact 恒为 0（而不是沿用旧值）。此时必须由人重新平放设备并手动
 *   调用 stateStartFlatRef() 重采，或用 't' 命令触发。
 *   绝不"凑合用一个不可信的基准继续判"。
 */
bool  stateStartFlatRef(void);       // 手动启动/重采平放基准；返回是否接受
bool  stateFlatRefCapturing(void);   // 是否正在采集平放基准
void  stateTickFlatRef(uint32_t nowUs);  // 主循环每圈调用：自动启动 + 结算 + 重试
void  stateClearFlatRef(void);       // 清除平放基准（此后 contactValid 回到 false）
bool  stateFlatRefSet(void);         // 是否已记录

/* ---- 测试钩子：正式固件不定义 STATE_TEST_HOOKS，以下不参与编译 ---- */
#ifdef STATE_TEST_HOOKS
// 取最近的启停周期快照（毫秒），返回实际个数。用于验证周期一致性。
uint8_t stateTestPeriods(float* out, uint8_t maxN);
// 直接注入一个包络点，绕过 DSP 链路。
// T07j 用它隔离"边沿插值"的效果：走完整 DSP 时，快/慢上升沿的包络形状本身不同，
// 测到的是"形状差异 + 插值误差"的混合结果，无法单独证明插值有效。
void stateTestInjectEnv(float env, uint32_t tUs);
void stateTestSetContact(uint8_t c);   // 回放用：按日志的 CT 列设置接触
// 最近一次记录的"起"边沿时刻（微秒）。用于验证缺口不会插出虚假边沿。
bool     stateTestLastEdgeUs(uint32_t* out);
#endif
