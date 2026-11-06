#line 1 "D:\\大学\\课程\\工科赛\\代码\\vibmon_v5\\posture.h"
// 三轴姿态诊断：仅用于 START/HONGEST 辅助判断，不参与正式 contact。
#pragma once
#include <stdint.h>
#include "dsp.h"
#include "sampler.h"

enum PostureClass : uint8_t {
  POSTURE_UNKNOWN = 0,
  POSTURE_START = 1,     // 回到张开/基准姿态
  POSTURE_HONGEST = 2,   // 已闭合到底的机械姿态
};

/* START/HONGEST 静态姿态诊断结果。
 * 本模块只输出诊断，不参与正式 contact：
 *   - cls 是当前 500ms 窗口的即时分类；
 *   - stableCls 是连续多个窗口确认后的稳定分类；
 *   - calValid 只说明 START 基线有效，不代表当前类别有效。 */
struct PostureResult {
  bool calValid;        // START 基线是否有效
  bool calActive;       // 是否正在采 START 基线
  bool calFailed;       // 最近一次 P 基线是否失败
  bool valid;           // 当前窗口是否满足静态质量门
  uint8_t cls;            // 当前窗口即时分类
  uint8_t stableCls;    // 防抖后的稳定分类
  bool stable;              // stableCls 是否刚被确认
  uint8_t stableCount;  // 连续同类别窗口计数
  float distMg;              // 当前三轴均值到 START 基线的距离
  float x, y, z;             // 当前窗口三轴均值
  float startX, startY, startZ; // START 基线
  uint32_t blockN;      // 当前窗口样本数
  uint32_t calN;           // 基线窗口样本数
};

/* 全量复位：上电或重新开始一次姿态诊断时调用。 */
void postureInit(void);

/* 只清运行窗口，不清 START 基线；startRun() 使用。 */
void postureResetRuntime(void);

/* 主动作废 START 基线。
 * 重标定改变加速度零偏后必须调用，否则旧基线与新 ctAxis 不在同一坐标系。 */
void postureInvalidateCal(void);

/* 按 P 后启动 START 基线采集；已在采集中时返回 false。 */
bool postureStartCal(void);
bool postureCalCapturing(void);

/* 逐样本输入。内部同时推进基线采集状态机和 500ms 运行分类窗口。 */
void postureOnFrame(const DspFrame& f, uint32_t tUs);

/* 采样缺口处理：当前窗口立即作废；若正在采基线则本次基线失败。 */
void postureOnGap(const SamplerGap& gap);

/* 超时检查：标定超时、长时间无新样本时使当前分类无效。 */
void postureTick(uint32_t nowUs);
const PostureResult& postureResult(void);
const char* postureClassName(uint8_t cls);
