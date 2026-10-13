// MPU6050 驱动实现。
// 工作流程（自上而下）：
//   mpuBusBegin()  重建 I2C 总线（可重复调用，用于故障后重连）
//   mpuInit()      探测 → 软复位 → 唤醒 → 写配置 → 回读校验，任一步失败立即返回错误码
//   mpuReadAccel() 运行期唯一被高频调用的函数，由采样定时器回调驱动（1kHz）
#include "mpu.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>

/* ---------- 寄存器地址 ---------- */
// 这些是 MPU6050 数据手册里的固定地址，不属于可调参数，因此留在本文件而不放 config.h。
#define REG_SMPLRT_DIV   0x19   // 采样率分频：输出率 = 1kHz / (1 + 该值)
#define REG_CONFIG       0x1A   // 配置：低 3 位是 DLPF_CFG（数字低通带宽）
#define REG_GYRO_CONFIG  0x1B   // 陀螺量程。本模块不用陀螺，写 0 即可
#define REG_ACCEL_CONFIG 0x1C   // 加速度配置：bit3~4 是 AFS_SEL（量程档位）
#define REG_ACCEL_XOUT_H 0x3B   // 加速度数据起始寄存器（X 高字节），后 5 字节依次为 X/Y/Z
#define REG_PWR_MGMT_1   0x6B   // 电源管理 1：bit7=DEVICE_RESET，bit0~2=CLKSEL
#define REG_WHO_AM_I     0x75   // 器件 ID，固定应为 0x68

// 累计 I2C 错误数。volatile：会在定时器回调（另一个任务上下文）里被修改，
// 主循环读取时不能被编译器优化成寄存器缓存。
static volatile uint32_t s_err = 0;

// 错误计数 +1。写成函数是为了让所有出错路径口径一致，也便于将来加日志。
static void errInc(void) { s_err = s_err + 1; }

// 写单个寄存器。返回 false 表示 I2C 事务未被器件 ACK。
// 注意：不重试。1kHz 采样节拍下重试会拖长阻塞时间，初始化阶段则由上层决定是否重来。
static bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  if (Wire.endTransmission() != 0) { errInc(); return false; }
  return true;
}

// 连续读 len 个寄存器到 buf。
// endTransmission(false) 发出的是"重复起始"而非停止位，这是 MPU6050 规定的
// 寄存器地址→数据 的连续读时序，中间不能插停止位。
// requestFrom 返回值即实际读到的字节数，少于请求值说明总线出错。
static bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { errInc(); return false; }
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)len) != (size_t)len) { errInc(); return false; }
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

// 初始化 I2C 总线。先 end() 再 begin() 是为了在传感器掉线后能彻底重置总线状态
// （例如器件把 SDA 拉死导致总线卡住时，重新初始化往往能恢复）。
void mpuBusBegin(void) {
  Wire.end();
  Wire.begin(PIN_SDA, PIN_SCL, (uint32_t)I2C_CLOCK);
  Wire.setTimeOut(I2C_TIMEOUT_MS);   // 超时兜底：避免总线异常时回调被永久阻塞
}

// 完整初始化。流程推演：
//   ① 读 WHO_AM_I 确认器件在线且型号正确；
//   ② 写 PWR_MGMT_1 的 bit7 触发软复位，等 100ms 让芯片回到已知状态；
//   ③ 按 cfg 表顺序写配置（第一项是唤醒，必须最先生效）；
//   ④ 逐项回读比对，确认每一项都真正写进去了。
// 之所以要"回读校验"：I2C 写操作被 ACK 只代表器件收到了，不代表寄存器接受了该值
// （例如器件还在复位过程中，写入会被忽略）。比赛现场靠这一步把隐性配置错误变成显式报错。
MpuInitResult mpuInit(uint8_t* badReg) {
  uint8_t dummy;
  if (!badReg) badReg = &dummy;   // 允许调用方不关心出错寄存器
  *badReg = 0;

  // ① 探测：先确认通信与型号
  uint8_t who = 0;
  if (!readRegs(REG_WHO_AM_I, &who, 1)) { *badReg = REG_WHO_AM_I; return MPU_ERR_NO_ACK; }
  if (who != 0x68) { *badReg = REG_WHO_AM_I; return MPU_ERR_WHOAMI; }

  // ② 软复位：bit7=1 让所有寄存器恢复默认值，消除上一次运行残留的配置
  if (!writeReg(REG_PWR_MGMT_1, 0x80)) { *badReg = REG_PWR_MGMT_1; return MPU_ERR_WRITE; }
  delay(100);

  // ③ 配置表。数组第 0 列是寄存器地址，第 1 列是要写入的值。
  // 写入顺序即生效顺序；PWR_MGMT_1 放第一个以先唤醒芯片
  // （复位后芯片处于 SLEEP，不先唤醒的话后面的配置写入不会生效）。
  static const uint8_t cfg[][2] = {
    {REG_PWR_MGMT_1,   (uint8_t)(MPU_CLKSEL & 0x07)},              // 选时钟源，同时清掉 SLEEP 位
    {REG_ACCEL_CONFIG, (uint8_t)((ACCEL_FS_SEL & 0x03) << 3)},     // 量程档位在 bit3~4
    {REG_GYRO_CONFIG,  0x00},                                      // 陀螺用不到，置最省电档
    {REG_CONFIG,       (uint8_t)(DLPF_CFG & 0x07)},                // 数字低通带宽
    {REG_SMPLRT_DIV,   0x00},                                      // 分频 0 → 输出率 1kHz
  };
  for (unsigned i = 0; i < sizeof(cfg) / sizeof(cfg[0]); i++) {
    if (!writeReg(cfg[i][0], cfg[i][1])) { *badReg = cfg[i][0]; return MPU_ERR_WRITE; }
    if (i == 0) delay(50);   // 唤醒需要时间，等它退出 SLEEP 再写其余寄存器
  }
  delay(50);

  // ④ 回读校验：写进去什么就必须读回什么，否则报 VERIFY 并指出是哪个寄存器
  for (unsigned i = 0; i < sizeof(cfg) / sizeof(cfg[0]); i++) {
    uint8_t v = 0xFF;
    if (!readRegs(cfg[i][0], &v, 1)) { *badReg = cfg[i][0]; return MPU_ERR_NO_ACK; }
    if (v != cfg[i][1]) { *badReg = cfg[i][0]; return MPU_ERR_VERIFY; }
  }
  return MPU_OK;
}

// 读一次三轴加速度。一次突发读 6 字节，保证 X/Y/Z 来自同一个采样时刻
// （分三次读会跨越采样点，三轴数据不同步，合成包络会引入虚假抖动）。
bool mpuReadAccel(MpuRaw* out) {
  uint8_t b[6];
  if (!readRegs(REG_ACCEL_XOUT_H, b, 6)) return false;
  // 寄存器是大端：高字节在前，拼成 int16（有符号，静止时 Z 轴约为 +16384）
  out->x = (int16_t)((b[0] << 8) | b[1]);
  out->y = (int16_t)((b[2] << 8) | b[3]);
  out->z = (int16_t)((b[4] << 8) | b[5]);
  return true;
}

// 当前量程下 1g 对应多少 LSB。
// MPU6050 基准灵敏度 16384 LSB/g（±2g 档），量程每放大一倍，灵敏度减半。
// 用移位 (1 << sel) 实现 1/2/4/8 的除法。
float mpuLsbPerG(void) { return 16384.0f / (float)(1 << (ACCEL_FS_SEL & 0x03)); }

uint32_t mpuErrCount(void) { return s_err; }
void     mpuClearErr(void) { s_err = 0; }

// 结果码 → 中文。放在这里而不是 output 模块，是为了让新增错误码时不会漏改打印。
const char* mpuResultStr(MpuInitResult r) {
  switch (r) {
    case MPU_OK:         return "OK";
    case MPU_ERR_NO_ACK: return "无应答";
    case MPU_ERR_WHOAMI: return "WHO_AM_I 不符";
    case MPU_ERR_WRITE:  return "寄存器写失败";
    case MPU_ERR_VERIFY: return "寄存器回读不符";
  }
  return "?";
}
