#include "mpu.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H  0x43
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

static volatile uint32_t s_err = 0;

static void errInc(void) { s_err = s_err + 1; }

static bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  if (Wire.endTransmission() != 0) { errInc(); return false; }
  return true;
}

static bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { errInc(); return false; }
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)len) != (size_t)len) { errInc(); return false; }
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

void mpuBusBegin(void) {
  Wire.end();
  Wire.begin(PIN_SDA, PIN_SCL, (uint32_t)I2C_CLOCK);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
}

MpuInitResult mpuInit(uint8_t* badReg) {
  uint8_t dummy;
  if (!badReg) badReg = &dummy;
  *badReg = 0;

  uint8_t who = 0;
  if (!readRegs(REG_WHO_AM_I, &who, 1)) { *badReg = REG_WHO_AM_I; return MPU_ERR_NO_ACK; }
  if (who != 0x68) { *badReg = REG_WHO_AM_I; return MPU_ERR_WHOAMI; }

  if (!writeReg(REG_PWR_MGMT_1, 0x80)) { *badReg = REG_PWR_MGMT_1; return MPU_ERR_WRITE; }
  delay(100);

  // 写入顺序即生效顺序；PWR_MGMT_1 放第一个以先唤醒芯片
  static const uint8_t cfg[][2] = {
    {REG_PWR_MGMT_1,   (uint8_t)(MPU_CLKSEL & 0x07)},
    {REG_ACCEL_CONFIG, (uint8_t)((ACCEL_FS_SEL & 0x03) << 3)},
    {REG_GYRO_CONFIG,  (uint8_t)((GYRO_FS_SEL & 0x03) << 3)},
    {REG_CONFIG,       (uint8_t)(DLPF_CFG & 0x07)},
    {REG_SMPLRT_DIV,   0x00},
  };
  for (unsigned i = 0; i < sizeof(cfg) / sizeof(cfg[0]); i++) {
    if (!writeReg(cfg[i][0], cfg[i][1])) { *badReg = cfg[i][0]; return MPU_ERR_WRITE; }
    if (i == 0) delay(50);
  }
  delay(50);

  for (unsigned i = 0; i < sizeof(cfg) / sizeof(cfg[0]); i++) {
    uint8_t v = 0xFF;
    if (!readRegs(cfg[i][0], &v, 1)) { *badReg = cfg[i][0]; return MPU_ERR_NO_ACK; }
    if (v != cfg[i][1]) { *badReg = cfg[i][0]; return MPU_ERR_VERIFY; }
  }
  return MPU_OK;
}

bool mpuReadAccel(MpuRaw* out) {
  uint8_t b[6];
  if (!readRegs(REG_ACCEL_XOUT_H, b, 6)) return false;
  out->x = (int16_t)((b[0] << 8) | b[1]);
  out->y = (int16_t)((b[2] << 8) | b[3]);
  out->z = (int16_t)((b[4] << 8) | b[5]);
  out->gx = out->gy = out->gz = 0;
  return true;
}

/* 一次读完 6 轴：0x3B 起 14 字节 = 加速度 6 + 温度 2 + 陀螺 6。
 * 温度那 2 字节跳过不解析（本设计不用），但必须一起读，
 * 因为陀螺寄存器紧接在温度之后，分两次读会多一次 START/STOP 开销。 */
bool mpuReadAll(MpuRaw* out) {
  uint8_t b[14];
  if (!readRegs(REG_ACCEL_XOUT_H, b, 14)) return false;
  out->x  = (int16_t)((b[0] << 8) | b[1]);
  out->y  = (int16_t)((b[2] << 8) | b[3]);
  out->z  = (int16_t)((b[4] << 8) | b[5]);
  out->gx = (int16_t)((b[8] << 8) | b[9]);
  out->gy = (int16_t)((b[10] << 8) | b[11]);
  out->gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

float mpuLsbPerG(void) { return 16384.0f / (float)(1 << (ACCEL_FS_SEL & 0x03)); }
float mpuLsbPerDps(void) { return 131.0f / (float)(1 << (GYRO_FS_SEL & 0x03)); }

uint32_t mpuErrCount(void) { return s_err; }
void     mpuClearErr(void) { s_err = 0; }

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
