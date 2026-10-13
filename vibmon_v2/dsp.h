// 信号处理模块：标定、去零偏、高通滤波、RMS 包络、姿态（低频）支路、主频估计。
//
// 【本模块在全工程中的位置】
//   sampler 送来"原始 LSB" → dsp 算出"物理量" → state 判断"是什么状态" → output 打印
// dsp 是唯一做数值计算的地方：把 raw 换成 mg、滤掉重力、算出幅值与主频。
// 它不做任何"是/否"判断（除了结果有效性），判定逻辑全在 state，便于单独测试。
//
// 【两条并行支路】每个样本进来后同时走两条路，互不干扰：
//   振动支路：去零偏 → 二阶高通(70Hz) → ①RMS包络(30样本) ②存入主频窗(128样本)
//             → 输出 env（幅值）与 freqHz（主频）
//   姿态支路：去零偏 → 一阶低通(0.2s) → 模长 → 输出 ctDev（模块朝向偏离静置的程度）
//   之所以要分两路：振动（>60Hz）和姿态（≈0Hz）在频域上差了三个数量级，
//   用同一套滤波参数无法同时照顾好两者，分开处理各取所需。
#pragma once
#include <stdint.h>
#include "sampler.h"

// 标定结果。由 dspCalibrate() 一次算出，之后作为所有阈值的基准长期使用。
// 现场排障时这些数字是判断"标定是否可信"的依据（outPrintCal 会打印全部字段）。
struct DspCal {
  float bias[3];       // 零偏 mg
                       // 三轴零偏矢量。静置时它其实就等于重力矢量，模长应接近 1000mg
  float rawStd[3];     // 阶段A 原始数据标准差 mg（标定质量）
                       // 标定期间若被碰过，这个值会明显变大，用于判定 quality
  float biasNormMg;    // 零偏模长，静置时应接近 1000mg
  float sigBand[3];    // 高通后带内噪声 sigma mg
                       // 高通滤掉了重力，剩下的全是噪声+振动。静置标定时它=纯噪声底，
                       // 用于自适应设定过零阈值（ZCL_K × sigma）
  float envRest;       // 静置包络 mg
                       // 静置时 RMS 包络的实测值，作为启停阈值的基准（TH_HI_K × envRest）
  float ctRest;        // 静置姿态偏差 mg
                       // 静置时 ctDev 的实测值，作为接触阈值的基准（CT_K × ctRest）
  float zcl;           // 过零阈值 mg
                       // 主频估计用的滞回阈值，抗噪核心参数
  uint32_t nBias;      // 阶段A 样本数
                       // 用于判断标定是否真的采够了数据（<200 视为失败）
  bool  ok;            // 标定流程完成
                       // true 表示上面各字段已填充且可用；false 时下游沿用默认阈值
  bool  quality;       // 标定期间确实静置（标准差、模长都合格）
                       // ok 与 quality 是两件事：ok=流程跑完了，quality=数据可信。
                       // 流程可能跑完但期间被碰过，此时 ok=true 而 quality=false
};

// 每个样本处理后的输出；各 *Ready 标志表示本样本是否产生了对应的新结果
// （不是每个样本都会产生包络和主频：包络每 10 个样本一次，主频每 64 个样本一次）
struct DspFrame {
  float fx, fy, fz;    // 高通后三轴 mg
                       // 已去掉重力和零偏，只剩振动分量。仅供示波器组观察
  bool  envReady;      // true 表示本样本刷新了包络，env/ctDev 有效
  float env;           // 合成 RMS 包络 mg
                       // 三轴平方和的均方根，代表"振动有多强"，是启停判据的输入
  float ctDev;         // 姿态偏差 mg（每轴低通后相对零偏的模长）
                       // 静置时接近 0（或某个稳定小值），模块被转动/按压时增大
  bool  winReady;      // true 表示本样本跑完了一个主频窗，freqHz/axis/winClip 有效
  float freqHz;        // 0 = 本窗无有效主频
  uint8_t axis;        // 本窗选用的主轴 0/1/2
                       // 取方差最大的轴：振动主方向上的信噪比最好
  uint16_t winClip;    // 本窗削顶样本数
                       // 非 0 说明该窗内波形被削平，主频读数需要打折看待
};

void dspInit(void);                       // 上电调用一次：算滤波器系数 + 清状态
void dspReset(void);                      // 清空滤波器、包络、历史窗（不清标定）
void dspOnGap(uint32_t missed);           // 采样不连续：窗口重新攒满前不出主频
void dspProcess(const Sample& s, DspFrame* out);   // 每样本主处理链
bool dspCalibrate(void);                  // 会停止并重置采样器，返回时采样器处于停止状态
const DspCal& dspCal(void);               // 读取当前标定结果（只读）
