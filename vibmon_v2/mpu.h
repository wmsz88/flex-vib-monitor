// MPU 驱动模块：初始化、寄存器回读、读取加速度、通信错误统计。
//
// 【本模块的职责边界】
// 只跟 I2C 和寄存器打交道：把 MPU6050 配成"1kHz 输出、±2g、DLPF 260Hz"的样子，
// 并按需读出三轴原始值。不做任何单位换算以外的加工——mg 换算、滤波、状态判断
// 都在 dsp 与 state 模块。这样换传感器时只需重写本模块。
#pragma once
#include <stdint.h>

// 初始化结果码。返回值直接用于串口报错，每种失败对应一个具体的排查方向。
enum MpuInitResult : uint8_t {
  MPU_OK = 0,          // 初始化成功，全部配置寄存器回读一致
  MPU_ERR_NO_ACK,      // WHO_AM_I 读不到
                       // → 接线/上拉/供电问题，或 AD0 电平与 MPU_ADDR 不符
  MPU_ERR_WHOAMI,      // 读到了但不是 0x68
                       // → 通信正常但器件不是 MPU6050（可能是 MPU6500/9250 等兼容芯片）
  MPU_ERR_WRITE,       // 配置寄存器写失败
                       // → 写事务没有 ACK，通常是总线被拉死或器件进入异常状态
  MPU_ERR_VERIFY,      // 回读值与写入值不符
                       // → 写成功了但没生效，多见于电源不稳或休眠未真正解除
};

// 三轴原始读数。int16 与传感器寄存器位宽一致，尚未换算成物理量。
struct MpuRaw { int16_t x, y, z; };

void        mpuBusBegin(void);
// 完整初始化：探测 → 软复位 → 唤醒 → 写配置 → 逐个回读校验。失败时 *badReg 给出出错寄存器。
MpuInitResult mpuInit(uint8_t* badReg);
bool        mpuReadAccel(MpuRaw* out);     // 6 字节突发读，失败计入错误数
float       mpuLsbPerG(void);              // 当前量程下每 1g 对应的原始 LSB 数（用于换算 mg）
uint32_t    mpuErrCount(void);             // 累计 I2C 通信错误次数（含初始化与运行期）
void        mpuClearErr(void);             // 清零错误计数（响应 'n' 命令）
const char* mpuResultStr(MpuInitResult r); // 结果码转中文，供串口打印
