// 采样模块：esp_timer 定速采集、样本序号与时间戳、环形缓冲、丢样与断线统计。
//
// 【为什么用 esp_timer 而不是 loop() 里读】
// loop() 会被串口打印、标定、状态判断拖慢，节奏不可控。esp_timer 由硬件定时器
// 驱动，能给出稳定的 1kHz 节拍；主循环只负责"消费"缓冲里的样本，两边通过环形
// 缓冲解耦。代价是回调里不能做耗时操作（见 I2C_TIMEOUT_MS）。
#pragma once
#include <stdint.h>
#include "mpu.h"

// 一个采样样本。三个字段共同保证下游能判断"数据是否连续可信"。
struct Sample {
  uint32_t seq;      // 采样节拍序号（读失败的节拍也计数，用于发现缺样）
                     // 关键：序号在"读失败"时也递增，因此下游看到 seq 跳变就能知道
                     // 中间丢了几拍；如果只在成功时递增，丢样就完全不可见了
  uint32_t tUs;      // 读取完成时刻 micros()
                     // 用实际时刻而不是 seq×标称周期：定时器回调会被推迟，
                     // 用标称值算周期会引入系统性误差
  MpuRaw   raw;
};

// 采样质量统计。全部是累计值，直到 'n' 命令清零。
// 这些数字是现场排障的主要依据：主频读数不准时，先看这里判断是"信号问题"还是"采集问题"。
struct SamplerStats {
  uint32_t readFail;     // I2C 读失败的节拍数
  uint32_t ringDrop;     // 缓冲满被丢弃的样本数
                         // 非 0 说明主循环太慢（通常是串口打印拖的），需要降低输出频率
  uint32_t gaps;         // 消费端发现的不连续次数（缺样或时间间隔异常）
  uint32_t missed;       // 消费端累计缺失的样本数
  uint32_t clip;         // 削顶样本数（由信号处理模块累加）
                         // 非 0 说明振动幅度超出 ACCEL_FS_SEL 所选量程，主频可能失真
};

void samplerStart(void);
void samplerStop(void);
void samplerReset(void);                 // 清空缓冲与序号，须在停止状态下调用
// 取一个样本；*gap 返回它与上一个样本之间缺了几个（0 = 连续）
bool samplerPop(Sample* s, uint32_t* gap);
bool samplerSensorLost(void);            // 连续读失败或长时间无新样本
const SamplerStats& samplerStats(void);
void samplerAddClip(void);
void samplerClearStats(void);
