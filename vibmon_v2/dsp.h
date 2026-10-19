// 信号处理模块：标定、去零偏、高通滤波、RMS 包络、姿态（低频）支路、主频估计。
#pragma once
#include <stdint.h>
#include "sampler.h"

struct DspCal {
  float bias[3];       // 零偏 mg
  float rawStd[3];     // 阶段A 原始数据标准差 mg（标定质量）
  float biasNormMg;    // 零偏模长，静置时应接近 1000mg
  float sigBand[3];    // 高通后带内噪声 sigma mg
  float envRest;       // 静置包络 mg
  float ctRest;        // 静置姿态偏差 mg
  float zcl;           // 过零阈值 mg
  uint32_t nBias;      // 阶段A 样本数
  uint8_t  fsSel;      // 标定时的 ACCEL_FS_SEL（诊断用，不做运行期一致性检查）
  bool  ok;            // 【语义】当前生效配置是否来自一次成功标定
                       //   true  = 有一份可信配置在跑（可能是上次成功的）
                       //   false = 从未成功过，阈值仍是编译期默认值
                       // 它不表示"本次流程是否跑完"，后者看 DspCalReport
  bool  quality;       // 当前生效配置来自静置合格的标定
};

// 本次标定尝试的结果。与 DspCal 是两个维度：
//   DspCal       回答"现在用的是哪套参数、可不可信"
//   DspCalReport 回答"刚才那次尝试怎么样、为什么失败"
enum CalResult : uint8_t {
  CALRES_NONE = 0,             // 本次上电还没跑过标定
  CALRES_OK,                   // 本次成功，已提交
  CALRES_ERR_SAMPLES,          // 阶段A 样本不足
  CALRES_ERR_STAGE_A,          // 阶段A 静置质量不合格（std 或模长超限）
  CALRES_ERR_STAGE_B,          // 阶段B 数据不足 / 参数非有限
  CALRES_ERR_STAGE_B_QUALITY,  // 阶段B 削顶 / 静置包络或姿态偏差超限
  CALRES_ERR_TIMER,            // 阶段B 采样器启动失败
  CALRES_ERR_STOP_TIMEOUT,     // 阶段B 结束时采样回调未退出
};

// 本次尝试的诊断证据：成功失败都填，供串口排障
struct DspCalReport {
  CalResult result;
  uint32_t  nBias;        // 阶段A 样本数（失败时也能看）
  float     rawStdMax;    // 阶段A 最大标准差
  float     biasNormMg;   // 阶段A 零偏模长
  float     envRest;      // 阶段B 静置包络（没走到阶段B 时为 0）
  float     ctRest;       // 阶段B 静置姿态偏差
  uint16_t  stageBClip;   // 阶段B 削顶样本数（取自 samplerStats().clip 差值）
};

// 本窗削顶等级：由主轴上的削顶比例决定
//   幅值与主频的判据故意不同——
//   幅值是合成 RMS，任意轴削顶都让它偏小；
//   主频只看主轴，非主轴削顶不破坏过零序列。
enum ClipLevel : uint8_t {
  CLIP_NONE = 0,    // 主轴无削顶
  CLIP_MILD,        // 主轴削顶 < CLIP_SEVERE_RATIO：幅值失效，主频保留
  CLIP_SEVERE,      // 主轴削顶 >= CLIP_SEVERE_RATIO：幅值和主频都失效
};

// 每个样本处理后的输出；各 *Ready 标志表示本样本是否产生了对应的新结果
struct DspFrame {
  float fx, fy, fz;    // 高通后三轴 mg
  bool  envReady;
  float env;           // 合成 RMS 包络 mg
  float ctDev;         // 姿态偏差 mg（每轴低通后相对零偏的模长）
  float ctAxis[3];     // 【Q4】三轴低通值 mg，**相对标定零偏后的偏差量**
                       // 为什么要它：ctDev 是模长，方向信息已丢失。
                       // 接触可能只表现在某一轴的变化上，模长看不出来。
                       // 语义：静置时 ≈ (0,0,0)；要还原绝对朝向，加回 dspCal().bias[a]。
  bool  winReady;
  float freqHz;        // 0 = 本窗无有效主频
  uint8_t axis;        // 本窗选用的主轴 0/1/2
  // ---- 削顶质量字段（每样本重新初始化，见 dspProcess 开头）----
  uint16_t  winClip;       // 主轴上的削顶数（真窗口计数，覆盖最近 BLOCK_N 个样本）
  ClipLevel clipLevel;     // 主轴削顶等级
  bool      sampleClipped; // 本样本任意轴是否削顶（逐样本，用于立即失效）
  bool      envClipped;    // 最近 ENV_WIN 样本内任意轴是否削顶（用于判断能否恢复）
  uint8_t   clipAxisMask;  // 本样本哪些轴削顶：bit0=X, bit1=Y, bit2=Z
};

void dspInit(void);
void dspReset(void);                      // 清空滤波器、包络、历史窗（不清标定）
void dspOnGap(const SamplerGap& gap);    // 任何缺口：窗口/高通/包络全部重置
void dspProcess(const Sample& s, DspFrame* out);

// 事务化标定：先算到候选，全部校验通过才一次性提交；任一失败 s_cal 一个字节都不变。
// 合约：无论成功失败，返回时采样器处于停止状态、DSP 状态已清空、
//       候选与标定模式标志均已复位。
bool dspCalibrate(void);

const DspCal&        dspCal(void);         // 当前生效参数（只读）
CalResult            dspCalLastResult(void);
const DspCalReport&  dspCalReport(void);
const char*          calResultStr(CalResult r);
