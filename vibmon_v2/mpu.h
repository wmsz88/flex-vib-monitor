// MPU 驱动模块：初始化、寄存器回读、读取加速度、通信错误统计。
#pragma once
#include <stdint.h>

enum MpuInitResult : uint8_t {
  MPU_OK = 0,
  MPU_ERR_NO_ACK,      // WHO_AM_I 读不到
  MPU_ERR_WHOAMI,      // 读到了但不是 0x68
  MPU_ERR_WRITE,       // 配置寄存器写失败
  MPU_ERR_VERIFY,      // 回读值与写入值不符
};

struct MpuRaw { int16_t x, y, z; };

void        mpuBusBegin(void);
// 完整初始化：探测 → 软复位 → 唤醒 → 写配置 → 逐个回读校验。失败时 *badReg 给出出错寄存器。
MpuInitResult mpuInit(uint8_t* badReg);
bool        mpuReadAccel(MpuRaw* out);     // 6 字节突发读，失败计入错误数
float       mpuLsbPerG(void);
uint32_t    mpuErrCount(void);
void        mpuClearErr(void);
const char* mpuResultStr(MpuInitResult r);
