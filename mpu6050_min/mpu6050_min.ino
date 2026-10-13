/* =====================================================================
 *  mpu6050_min —— MPU6050 / I2C 最小建链验证（单文件，约 100 行）
 * ---------------------------------------------------------------------
 *  用途：当你只想快速确认「MPU6050 通不通」时用，比 selftest 轻。
 *        五步阶梯，每步直接给 PASS / FAIL，一眼知道卡在哪。
 *  板子：ESP32-C3        引脚：SDA=GPIO8  SCL=GPIO9（网表 U1.4 / U1.5）
 *  地址：0x68（AD0 接地）  速率：200kHz
 *
 *  烧录：按住 BOOT → 按 RESET → 松 RESET → 松 BOOT
 *        串口监视器 115200 / 8N1
 *        若完全无输出：勾选 "USB CDC On Boot" 重新编译，或改用 Serial1
 *        （本模块 Type-C 直连芯片 USB，不勾 CDC 时 Serial 走 GPIO20/21 比赛口）
 * ===================================================================== */

#include <Wire.h>

/* ---------------- 配置 ---------------- */
#define PIN_SDA        8
#define PIN_SCL        9
#define MPU_ADDR       0x68
#define I2C_CLOCK      200000UL   // 压力测试不过就降到 100000UL

/* ---------------- 寄存器（依据 RM-MPU-6000A-00） ---------------- */
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define ACCEL_LSB_PER_G  16384.0f   // ±2g 档

/* ---------------- 底层读写 ---------------- */
static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;          // repeated start
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)len) != (size_t)len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

static inline int16_t be16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }
static inline float   raw2mg(int16_t r) { return (float)r * (1000.0f / ACCEL_LSB_PER_G); }

/* ================= setup：五步阶梯 ================= */
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("==================================================");
  Serial.println(" MPU6050 最小建链验证   SDA=GPIO8  SCL=GPIO9  0x68");
  Serial.println("==================================================");

  /* ---- ① 空闲电平：必须在 Wire.begin 之前读，否则引脚已被 I2C 占用 ---- */
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  delay(10);
  int sda = digitalRead(PIN_SDA), scl = digitalRead(PIN_SCL);
  Serial.printf("[1] 空闲电平  SDA=%s  SCL=%s  -> %s\n",
                sda ? "HIGH" : "LOW", scl ? "HIGH" : "LOW",
                (sda && scl) ? "PASS（总线电气正常）"
                             : "FAIL（断线/虚焊/没上拉/模块没供电）");

  /* ---- ② 地址扫描 ---- */
  Wire.begin(PIN_SDA, PIN_SCL, (uint32_t)I2C_CLOCK);
  Wire.setTimeOut(50);
  int cnt = 0;
  for (uint8_t a = 0x03; a <= 0x77; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("      命中 0x%02X\n", a);
      cnt++;
    }
  }
  if (cnt == 0) {
    Serial.println("[2] 扫描：0 个器件 -> FAIL（查 SDA/SCL 焊接、模块 5V/GND、上拉）");
  } else {
    Serial.printf("[2] 扫描：%d 个器件 -> %s\n", cnt,
                  (cnt == 1) ? "PASS（恰好 0x68）" : "WARN（不是唯一的 0x68，核对 AD0）");
  }

  /* ---- ③ 身份确认 ---- */
  uint8_t id = 0;
  if (!mpuRead(REG_WHO_AM_I, &id, 1)) {
    Serial.println("[3] WHO_AM_I 读失败 -> FAIL（I2C 无应答，先修第 1、2 步）");
    return;                                  // 后面几步没意义，不跑
  }
  Serial.printf("[3] WHO_AM_I = 0x%02X -> %s\n", id,
                (id == 0x68) ? "PASS" : "FAIL（期望 0x68；0xFF=总线不通，其他值=疑似克隆片）");

  /* ---- ④ 唤醒（PWR_MGMT_1 的 SLEEP 位必须清零） ---- */
  if (!mpuWrite(REG_PWR_MGMT_1, 0x00)) {
    Serial.println("[4] 写 PWR_MGMT_1 失败 -> FAIL（写不通）");
    return;
  }
  delay(50);
  uint8_t rb = 0xFF;
  mpuRead(REG_PWR_MGMT_1, &rb, 1);
  Serial.printf("[4] PWR_MGMT_1 回读 = 0x%02X -> %s\n", rb,
                (rb == 0x00) ? "PASS（已退出睡眠）" : "FAIL（写没生效，芯片仍在睡眠）");

  /* ---- ⑤ 配置：量程 / 滤波 / 采样率 ---- */
  bool ok = true;
  ok &= mpuWrite(REG_ACCEL_CONFIG, 0x00);   // ±2g，16384 LSB/g
  ok &= mpuWrite(REG_CONFIG,       0x00);   // DLPF_CFG=0 → 加速度带宽 260Hz
  ok &= mpuWrite(REG_SMPLRT_DIV,   0x00);   // 采样分频 0 → 1kHz
  delay(100);
  Serial.printf("[5] 配置 ±2g / DLPF=260Hz / 1kHz -> %s\n",
                ok ? "PASS" : "FAIL（有寄存器没写进去）");

  Serial.println("--------------------------------------------------");
  Serial.println(" 进入连续读数：静置时 az≈+1000mg（Z 轴朝上）");
  Serial.println("--------------------------------------------------");
}

/* ================= loop：连续读加速度 ================= */
void loop() {
  uint8_t b[6];
  if (!mpuRead(REG_ACCEL_XOUT_H, b, 6)) {
    Serial.println("读加速度失败（I2C 断了？）");
    delay(500);
    return;
  }
  float ax = raw2mg(be16(&b[0]));
  float ay = raw2mg(be16(&b[2]));
  float az = raw2mg(be16(&b[4]));
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  Serial.printf("ax=%8.1f  ay=%8.1f  az=%8.1f  mg   |a|=%7.1f mg\n", ax, ay, az, mag);
  delay(200);
}
