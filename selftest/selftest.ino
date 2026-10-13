/* =====================================================================
 *  柔性振动检测模块 · 代码一：硬件自检程序  (selftest)
 * ---------------------------------------------------------------------
 *  目标板 : ESP32-C3（组委会扩展模块，arduino-esp32 内核 3.x）
 *  引脚依据: 电路搭建\PCB\Netlist_PCB1_2026-09-24.tel（定稿网表）
 *
 *  引脚映射（来自网表，勿随意改）：
 *      I2C SDA = GPIO8    I2C SCL = GPIO9    （U1.4 / U1.5）
 *      绿 LED  = GPIO10                       （U1.6 经 R7）
 *      黄 LED  = GPIO5                        （U1.1 经 R8）
 *      UART TX = GPIO21   UART RX = GPIO20    （U1.8 / U1.7 → 4PIN ①/②）
 *      红 LED  = 硬件直连 5V 常亮，软件不可控
 *
 *  用途：
 *    柔性板焊好后上电，"一眼看出哪儿是好的、哪儿要返工"。
 *    也可以在成品 ESP32-C3 开发板 + 独立 MPU6050 模块上先跑（不必等制板）。
 *
 *  13 项检查（逐项打印 PASS / FAIL / WARN）：
 *    01 复位原因         02 电源与红 LED       03 三灯（绿/黄）
 *    04 串口自环与映射   05 I2C 空闲电平       06 ★5V 上拉风险（需万用表）
 *    07 I2C 地址扫描     08 WHO_AM_I           09 寄存器回环读写
 *    10 六轴 + 温度      11 静置零偏标定       12 ★I2C 压力测试
 *    13 线序 / 开关提示
 *
 *  烧录：按住 BOOT → 按一下 RESET → 松开 RESET → 松开 BOOT
 *        串口监视器 115200 / 8N1
 * ===================================================================== */

#include <Wire.h>
#include <esp_timer.h>
#include <esp_system.h>

/* ========================= 配置区 ========================= */

#define PIN_SDA       8      // 网表 SDA ; U1.4 U3.4
#define PIN_SCL       9      // 网表 SCL ; U1.5 U3.3
#define PIN_LED_G     10     // 网表 P10 ; R7.2 U1.6（绿）
#define PIN_LED_Y     5      // 网表 P5  ; R8.1 U1.1（黄）

#define UART_SWAP     0      // 0 = TX→GPIO21 / RX→GPIO20（与网表一致）
                             // 1 = 交换（现场发现与测试平台不通时改这里）
#if UART_SWAP
  #define PIN_UART_TX 20
  #define PIN_UART_RX 21
#else
  #define PIN_UART_TX 21
  #define PIN_UART_RX 20
#endif

#define BAUD          115200
#define I2C_CLOCK     200000UL   // 200kHz：1kHz 采样下 6 字节读约 0.42ms，余量 2.4x
                                 // 压力测试不过就降到 100000UL
#define MPU_ADDR      0x68
#define LED_ACTIVE_LOW 0         // 指示灯 VF 偏高不亮时的灌电流预案：改 1

/* MPU6050 寄存器（依据官方 RM-MPU-6000A-00，详见 代码\MPU6050寄存器速查.md） */
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define ACCEL_LSB_PER_G  16384.0f   // ±2g 档
#define GYRO_LSB_PER_DPS 131.0f     // ±250dps 档

#define STRESS_N         1000       // I2C 压力测试次数
#define CALIB_MS         2000       // 静置标定持续时长

/* ========================= 输出层 ========================= */
/* 编译时若勾选 "USB CDC On Boot"，Serial 就是 USB-CDC 口，可同时镜像调试信息；
   否则 Serial 与 Serial1 抢同一个 UART0(GPIO20/21)，不镜像，避免驱动打架。 */
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  #define MIRROR_USB 1
#else
  #define MIRROR_USB 0
#endif

static HardwareSerial Comp(1);   // 比赛 / 调试口：显式绑 GPIO20/21

static void vdRaw(const char* s, size_t n) {
  Comp.write((const uint8_t*)s, n);
#if MIRROR_USB
  Serial.write((const uint8_t*)s, n);
#endif
}

static void vdPrintf(const char* fmt, ...) {   // 定长缓冲，不用 String / 不动态分配
  char buf[240];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
  vdRaw(buf, (size_t)n);
}

/* ========================= 报告层 ========================= */
static int g_pass = 0, g_fail = 0, g_warn = 0;

static const char* TAG_PASS = "  [PASS] ";
static const char* TAG_FAIL = "  [FAIL] ";
static const char* TAG_WARN = "  [WARN] ";

static void sep(void) { vdPrintf("--------------------------------------------------\n"); }

static void step(int no, const char* name) {
  vdPrintf("\n[%02d] %s\n", no, name);
}

static void report(int level, const char* msg, const char* hint) {
  // level: 0=FAIL 1=WARN 2=PASS
  if (level == 2)      { g_pass++; vdPrintf("%s%s\n", TAG_PASS, msg); }
  else if (level == 1) { g_warn++; vdPrintf("%s%s\n", TAG_WARN, msg); }
  else                 { g_fail++; vdPrintf("%s%s\n", TAG_FAIL, msg); }
  if (hint && hint[0]) vdPrintf("         -> %s\n", hint);
}

/* ========================= LED ========================= */
static inline void ledWrite(int pin, int on) {
#if LED_ACTIVE_LOW
  digitalWrite(pin, on ? LOW : HIGH);
#else
  digitalWrite(pin, on ? HIGH : LOW);
#endif
}

static void ledInit(void) {
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_Y, OUTPUT);
  ledWrite(PIN_LED_G, 0);
  ledWrite(PIN_LED_Y, 0);
}

/* ========================= I2C 底层 ========================= */
static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuRead(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;      // repeated start
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)len) != (size_t)len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

static inline int16_t be16(const uint8_t* p) { return (int16_t)((p[0] << 8) | p[1]); }
static inline float   raw2mg(int16_t r)  { return (float)r * (1000.0f / ACCEL_LSB_PER_G); }
static inline float   raw2dps(int16_t r) { return (float)r / GYRO_LSB_PER_DPS; }
static inline float   raw2c(int16_t r)   { return (float)r / 340.0f + 36.53f; }

/* ========================= 检查实现 ========================= */

/* 01 复位原因 */
static void checkResetReason(void) {
  step(1, "复位原因 / 启动状态");
  esp_reset_reason_t r = esp_reset_reason();
  const char* name = "UNKNOWN";
  int level = 2;
  const char* hint = "";
  switch (r) {
    case ESP_RST_POWERON:  name = "POWERON (正常上电)";  break;
    case ESP_RST_EXT:      name = "EXT (外部复位脚)";    break;
    case ESP_RST_SW:       name = "SW (软件复位)";       break;
    case ESP_RST_PANIC:    name = "PANIC (程序异常)";    level = 0;
                           hint = "看串口回溯；多半是空指针/看门狗，重新烧录再试"; break;
    case ESP_RST_INT_WDT:  name = "INT_WDT (中断看门狗)"; level = 0;
                           hint = "中断里干了太久的事；检查采样回调是否被 I2C 卡住"; break;
    case ESP_RST_TASK_WDT: name = "TASK_WDT (任务看门狗)"; level = 1;
                           hint = "某任务长时间不让出 CPU"; break;
    case ESP_RST_WDT:      name = "WDT (看门狗)";         level = 1; break;
    case ESP_RST_DEEPSLEEP:name = "DEEPSLEEP 唤醒";       break;
    case ESP_RST_BROWNOUT: name = "BROWNOUT (电压跌落)";  level = 0;
                           hint = "电不够！查 5V 轨带载、AMS1117 压差/散热、SW1.3 接法"; break;
    case ESP_RST_SDIO:     name = "SDIO 复位";            break;
    default: break;
  }
  vdPrintf("         复位原因 = %s\n", name);
  report(level, "复位原因已读取", hint);
}

/* 02 电源与红 LED（红 LED 由硬件保证，只能目视） */
static void checkPowerRail(void) {
  step(2, "电源轨 / 红 LED（电源指示）");
  vdPrintf("         红 LED 是 5V 轨直驱（+5V -> R3 -> LED3 -> GND），软件无法读取。\n");
  vdPrintf("         请目视确认：红 LED 常亮 = 12V 输入、开关、AMS1117、R3 这一路都通。\n");
  report(1, "红 LED 需人工目视确认（软件不可测）",
            "不亮 -> 查 12V 输入 / SW1 导通 / AMS1117 输出 5V / LED3 极性 / R3 焊点");
}

/* 03 三灯 */
static void checkLeds(void) {
  step(3, "三灯自检（绿 GPIO10 / 黄 GPIO5）");
  for (int i = 0; i < 3; i++) {
    ledWrite(PIN_LED_G, 1); ledWrite(PIN_LED_Y, 1);
    delay(150);
    ledWrite(PIN_LED_G, 0); ledWrite(PIN_LED_Y, 0);
    delay(150);
    vdPrintf("         第 %d 次闪烁\n", i + 1);
  }
  report(1, "绿 / 黄 LED 需人工目视确认（各闪 3 次）",
            "不亮 -> 焊点/极性；绿灯 VF 偏高时把代码里 LED_ACTIVE_LOW 改 1 试灌电流接法");
}

/* 04 串口 */
static void checkUart(void) {
  step(4, "串口（TTL，115200/8N1）");
  vdPrintf("         比赛口 = UART1 映射到 TX=GPIO%d / RX=GPIO%d\n", PIN_UART_TX, PIN_UART_RX);
  vdPrintf("         4PIN 焊盘线序（命题图3，更新版已确认）：①TX ②RX ③GND ④VIN\n");
  vdPrintf("         若终端看到的全是乱码 -> 波特率不对；完全无输出 -> 线序/焊接。\n");
  vdPrintf("         请在监视器里输入任意字符，2 秒内会回显：");
  uint32_t t0 = millis();
  bool got = false;
  char echo = 0;
  while (millis() - t0 < 2000) {
    if (Comp.available()) { echo = (char)Comp.read(); got = true; break; }
  }
  vdPrintf("\n");
  if (got) vdPrintf("         收到字符 '%c' (0x%02X)\n", echo, (uint8_t)echo);
  report(got ? 2 : 1,
         got ? "串口双向通（发送 + 接收均正常）" : "串口只发未收（可能无输入，或 RX 未接）",
         got ? "" : "RX 不通 -> 查 4PIN ② / 杜邦线 / 是否与平台的 TX 交叉");
}

/* 05 I2C 空闲电平（必须在 Wire.begin 之前做） */
static int g_i2cIdleSda = -1, g_i2cIdleScl = -1;
static void checkI2cIdleLevel(void) {
  step(5, "I2C 空闲电平（GPIO8/GPIO9 输入态读电平）");
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  delay(10);
  g_i2cIdleSda = digitalRead(PIN_SDA);
  g_i2cIdleScl = digitalRead(PIN_SCL);
  vdPrintf("         SDA(GPIO%d) 空闲 = %s\n", PIN_SDA, g_i2cIdleSda ? "HIGH" : "LOW");
  vdPrintf("         SCL(GPIO%d) 空闲 = %s\n", PIN_SCL, g_i2cIdleScl ? "HIGH" : "LOW");
  bool ok = (g_i2cIdleSda == HIGH) && (g_i2cIdleScl == HIGH);
  report(ok ? 2 : 0,
         ok ? "两根 I2C 线都被上拉为高（总线电气基本正常）"
            : "I2C 空闲电平不是高！",
         ok ? "" : "LOW -> 断线/虚焊最可能；也可能被短路到 GND，或 MPU 模块没供电");
}

/* 06 ★5V 上拉风险 —— 软件测不了，必须万用表 */
static void checkPullupVoltageRisk(void) {
  step(6, "★ 上拉电平风险核查（需万用表，软件做不到）");
  vdPrintf("         命题表2 只写了 MPU6050 模块『内置 5.1K 上拉』，\n");
  vdPrintf("         但没说上拉到哪一档：模块 VCC(5V) 还是板载 LDO 的 3.3V。\n");
  vdPrintf("         ESP32-C3 的 IO 绝对最大只有 3.6V —— 若被上拉到 5V 会伤 IO。\n");
  vdPrintf("         软件无法区分两者：5V 经 ESD 钳位后数字上仍判为 HIGH，\n");
  vdPrintf("         且 GPIO8/9 不属于 ADC1(GPIO0~4)，读不到电压值。\n");
  vdPrintf("         >>> 请断电前先量：SDA/SCL 对 GND 的空闲电压\n");
  vdPrintf("             <= 3.3V 正常 ； 约 5V 立刻断电\n");
  vdPrintf("         整改动作：把 MPU 模块 VCC 从 +5V 改接 ESP32 模块的 3V3 脚。\n");
  report(1, "上拉电平为【人工必检项】，请在下一步前实测确认",
            "判据：空闲电压 <= 3.3V");
}

/* 07 I2C 扫描 */
static int g_scanCount = 0;
static uint8_t g_scanFound[8];
static void checkI2cScan(void) {
  step(7, "I2C 地址扫描（0x03 ~ 0x77）");
  g_scanCount = 0;
  for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      vdPrintf("         命中 0x%02X\n", addr);
      if (g_scanCount < 8) g_scanFound[g_scanCount] = addr;
      g_scanCount++;
    }
  }
  if (g_scanCount == 0) {
    report(0, "总线上没有扫到任何器件",
              "查 SDA/SCL 焊接、MPU 模块 5V/GND、上拉是否存在、总线是否被拉死");
  } else if (g_scanCount == 1 && g_scanFound[0] == MPU_ADDR) {
    report(2, "恰好扫到 0x68（期望值，AD0 接地）", "");
  } else {
    report(1, "扫到的地址与期望 0x68 不完全一致，请核对",
              "0x69 -> AD0 悬空/接高；其他地址 -> 总线上挂了别的器件");
  }
}

/* 08 WHO_AM_I */
static uint8_t g_whoami = 0;
static bool checkWhoAmI(void) {
  step(8, "WHO_AM_I (0x75)");
  bool ok = mpuRead(REG_WHO_AM_I, &g_whoami, 1);
  if (!ok) {
    report(0, "读 WHO_AM_I 失败（I2C 无应答）", "见第 7 项排查；也可把 I2C_CLOCK 降到 100000");
    return false;
  }
  vdPrintf("         读回 0x%02X（期望 0x68）\n", g_whoami);
  if (g_whoami == 0x68) {
    report(2, "WHO_AM_I = 0x68，芯片身份正确", "");
    return true;
  }
  const char* hint = "读到 0xFF -> 总线不通；0x70/0x98/0x72 -> 疑似克隆/替换片（MPU6500/9250）";
  report(0, "WHO_AM_I 不是 0x68", hint);
  return false;
}

/* 09 寄存器回环读写 */
static void checkRegLoopback(void) {
  step(9, "寄存器回环读写（PWR_MGMT_1 = 0x00）");
  if (!mpuWrite(REG_PWR_MGMT_1, 0x00)) {
    report(0, "写 PWR_MGMT_1 失败", "I2C 写不通；查上拉与总线速率");
    return;
  }
  delay(50);
  uint8_t rb = 0xFF;
  if (!mpuRead(REG_PWR_MGMT_1, &rb, 1)) {
    report(0, "回读 PWR_MGMT_1 失败", "写成功读失败 -> 总线不稳定");
    return;
  }
  vdPrintf("         回读 0x%02X（期望 0x00，即已退出睡眠）\n", rb);
  if (rb == 0x00) report(2, "寄存器读写一致，芯片已唤醒", "");
  else            report(0, "回读值与写入值不一致",
                            "检查 I2C_CLOCK；若一直读到 0x40 说明写没生效（片子在睡眠）");
}

/* 10 六轴 + 温度 */
static void checkSixAxis(void) {
  step(10, "六轴 + 温度（±2g / ±250dps）");
  if (!mpuWrite(REG_ACCEL_CONFIG, 0x00))  vdPrintf("         警告：ACCEL_CONFIG 写入失败\n");
  if (!mpuWrite(REG_GYRO_CONFIG, 0x00))   vdPrintf("         警告：GYRO_CONFIG 写入失败\n");
  if (!mpuWrite(REG_CONFIG, 0x00))        vdPrintf("         警告：CONFIG(DLPF=260Hz) 写入失败\n");
  if (!mpuWrite(REG_SMPLRT_DIV, 0x00))    vdPrintf("         警告：SMPLRT_DIV 写入失败\n");
  delay(100);

  uint8_t b[14];
  if (!mpuRead(REG_ACCEL_XOUT_H, b, 14)) {
    report(0, "突发读 14 字节失败", "I2C 读不稳定；见第 12 项压力测试");
    return;
  }
  float ax = raw2mg(be16(&b[0])), ay = raw2mg(be16(&b[2])), az = raw2mg(be16(&b[4]));
  float tc = raw2c(be16(&b[6]));
  float gx = raw2dps(be16(&b[8])), gy = raw2dps(be16(&b[10])), gz = raw2dps(be16(&b[12]));

  vdPrintf("         ACCEL  X=%8.1f mg  Y=%8.1f mg  Z=%8.1f mg\n", ax, ay, az);
  vdPrintf("         GYRO   X=%8.2f dps Y=%8.2f dps Z=%8.2f dps\n", gx, gy, gz);
  vdPrintf("         TEMP   %8.2f C\n", tc);

  float mag = sqrtf(ax * ax + ay * ay + az * az);
  vdPrintf("         |a| = %.1f mg（静止时应≈1000mg）\n", mag);
  if (fabsf(mag - 1000.0f) < 250.0f) {
    report(2, "加速度模长接近 1g，量程与数据通路正常", "");
  } else {
    report(0, "加速度模长明显偏离 1g",
              "查 ACCEL_CONFIG 是否真写入、模块是否焊歪、是否坏了");
  }
  if (tc < 0.0f || tc > 60.0f) {
    report(1, "温度读数异常", "仅提示；温度只用于漂移监测");
  } else {
    report(2, "温度读数合理", "");
  }
}

/* 11 静置零偏标定 */
static float g_bias[3] = {0, 0, 0};
static float g_noise[3] = {0, 0, 0};
static void calibrateStaticBias(void) {
  step(11, "静置零偏标定（请保持模块静止 / 不要碰）");
  uint8_t b[6];
  double sx = 0, sy = 0, sz = 0, sx2 = 0, sy2 = 0, sz2 = 0;
  uint32_t n = 0, err = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < CALIB_MS) {
    if (!mpuRead(REG_ACCEL_XOUT_H, b, 6)) { err++; continue; }
    float x = raw2mg(be16(&b[0])), y = raw2mg(be16(&b[2])), z = raw2mg(be16(&b[4]));
    sx += x; sy += y; sz += z;
    sx2 += (double)x * x; sy2 += (double)y * y; sz2 += (double)z * z;
    n++;
  }
  if (n < 100) {
    report(0, "标定采样点太少，I2C 基本读不到数", "先解决前几项失败");
    return;
  }
  double mx = sx / n, my = sy / n, mz = sz / n;
  g_bias[0] = (float)mx; g_bias[1] = (float)my; g_bias[2] = (float)mz;
  g_noise[0] = sqrtf((float)fmax(0.0, sx2 / n - mx * mx));
  g_noise[1] = sqrtf((float)fmax(0.0, sy2 / n - my * my));
  g_noise[2] = sqrtf((float)fmax(0.0, sz2 / n - mz * mz));

  vdPrintf("         采样 %u 点，I2C 错误 %u 次\n", n, err);
  vdPrintf("         零偏 bias   X=%8.1f  Y=%8.1f  Z=%8.1f  mg\n", g_bias[0], g_bias[1], g_bias[2]);
  vdPrintf("         噪声 sigma  X=%8.1f  Y=%8.1f  Z=%8.1f  mg\n", g_noise[0], g_noise[1], g_noise[2]);
  vdPrintf("         建议的启停阈值（供主程序参考）：\n");
  vdPrintf("            Th_hi = %.0f mg   Th_lo = %.0f mg\n",
           5.0f * g_noise[2], 2.5f * g_noise[2]);
  float s = fmaxf(g_noise[0], fmaxf(g_noise[1], g_noise[2]));
  if (s < 20.0f)      report(2, "噪声地板很低，信号质量好", "");
  else if (s < 60.0f) report(1, "噪声偏大", "查结构是否松动、是否被工频/桌面振动耦合");
  else                report(0, "噪声过大，阈值会失真", "先排查机械固定与线缆，再考虑增大窗口");
}

/* 12 ★I2C 压力测试（决定 I2C_CLOCK 取 200k 还是 100k） */
static void checkI2cStress(void) {
  step(12, "★ I2C 压力测试（决定采样可行性）");
  vdPrintf("         连续 %d 次突发读 6 字节加速度（1kHz 采样时周期 1000us）...\n", STRESS_N);
  uint8_t b[6];
  uint32_t err = 0;
  uint32_t t0 = micros();
  for (int i = 0; i < STRESS_N; i++) {
    if (!mpuRead(REG_ACCEL_XOUT_H, b, 6)) err++;
  }
  uint32_t dt = micros() - t0;
  float avg = (float)dt / (float)STRESS_N;
  vdPrintf("         总耗时 %u us，平均单次 %.1f us，错误 %u 次\n", dt, avg, err);
  if (err == 0 && avg < 500.0f) {
    report(2, "余量充足，1kHz 采样（每周期 1000us）可稳定运行", "");
  } else if (err == 0 && avg < 900.0f) {
    report(1, "能用但余量偏小",
              "建议把 I2C_CLOCK 降到 100000 并把采样率降到 500Hz");
  } else {
    report(0, "压力测试不通过",
              "把 I2C_CLOCK 改成 100000 重试；仍不行就查杜邦线过长/虚焊/上拉缺失");
  }
}

/* 13 线序 / 开关提示 */
static void checkWiringAndSwitch(void) {
  step(13, "线序 / 开关 / 断电确认");
  vdPrintf("         4PIN 焊盘（命题图3 已确认）: ①TX(GPIO%d)  ②RX(GPIO%d)  ③GND  ④VIN(12V)\n",
           PIN_UART_TX, PIN_UART_RX);
  vdPrintf("         更新版赛题已删除旧正文矛盾表述，线序与本项目网表一致，UART_SWAP=%d。\n", UART_SWAP);
  vdPrintf("         与平台接通三步：平台 RX <- 本板 ①TX ；平台 TX -> 本板 ②RX ；③接地。\n");
  vdPrintf("         若不通又确认接线无误：把代码里的 UART_SWAP 改成 1 重新烧录。\n");
  vdPrintf("\n");
  vdPrintf("         断开确认：拨动 SW1 到关断档，等 10 秒，量 5V 轨应为 0V、红 LED 应灭。\n");
  report(1, "线序与断电为【人工必检项】",
            "开关断开后若仍有电 -> 存在反向馈电（USB/信号线），违反『断开零静态功耗』要求");
}

/* ========================= setup / loop ========================= */

static void printBanner(void) {
  vdPrintf("\n\n");
  vdPrintf("==================================================\n");
  vdPrintf(" 柔性振动检测模块 · 硬件自检  (selftest v1.0)\n");
  vdPrintf(" 目标: ESP32-C3 / arduino-esp32 %d.%d.%d\n",
           ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  vdPrintf(" 网表: Netlist_PCB1_2026-09-24.tel\n");
  vdPrintf(" 时钟: CPU %u MHz / Flash %u MHz\n", (unsigned)(ESP.getCpuFreqMHz()), (unsigned)(ESP.getFlashChipSpeed() / 1000000));
  vdPrintf(" 串口: %lu 8N1    I2C: %lu Hz   MPU 地址: 0x%02X\n",
           (unsigned long)BAUD, (unsigned long)I2C_CLOCK, MPU_ADDR);
  vdPrintf("==================================================\n");
}

static void printSummary(void) {
  sep();
  vdPrintf(" 自检汇总：PASS %d 项 / FAIL %d 项 / WARN %d 项\n", g_pass, g_fail, g_warn);
  if (g_fail == 0) vdPrintf(" >> 硬件通路基本正常，可以进入 vibmon 主程序验证算法。\n");
  else             vdPrintf(" >> 有硬故障，先按上面 [FAIL] 的提示返工，不要急着调算法。\n");
  sep();
  vdPrintf(" 已进入待机：绿 LED 1Hz 心跳（表示固件在跑）。\n");
  vdPrintf(" 命令: '?' 帮助   's' 重新自检   'c' 重做零偏标定\n\n");
}

void setup() {
  pinMode(PIN_UART_TX, OUTPUT);
  pinMode(PIN_UART_RX, INPUT);
  Comp.begin(BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
#if MIRROR_USB
  Serial.begin(BAUD);
#endif
  delay(300);

  ledInit();
  printBanner();

  checkResetReason();
  checkPowerRail();
  checkLeds();
  checkUart();
  checkI2cIdleLevel();
  checkPullupVoltageRisk();

  Wire.begin(PIN_SDA, PIN_SCL, (uint32_t)I2C_CLOCK);
  Wire.setTimeOut(50);

  checkI2cScan();
  bool whoOk = checkWhoAmI();
  if (whoOk) {
    checkRegLoopback();
    checkSixAxis();
    calibrateStaticBias();
    checkI2cStress();
  } else {
    vdPrintf("\n 后续检查（09~12）已跳过：芯片身份未确认，先修好 I2C 通信。\n");
  }
  checkWiringAndSwitch();
  printSummary();
}

void loop() {
  /* 绿 LED 1Hz 心跳：证明复位后固件在正常循环 */
  static uint32_t tLed = 0;
  static int ledOn = 0;
  if (millis() - tLed >= 500) {
    tLed = millis();
    ledOn = !ledOn;
    ledWrite(PIN_LED_G, ledOn);
  }

  /* 命令交互 */
  if (Comp.available()) {
    char c = (char)Comp.read();
    switch (c) {
      case '?':
        vdPrintf("命令: ?=帮助  s=重新自检  c=重做零偏标定  m=再读一次六轴\n");
        break;
      case 's':
        g_pass = g_fail = g_warn = 0;
        printBanner();
        checkResetReason(); checkLeds();
        checkI2cScan(); checkWhoAmI(); checkRegLoopback();
        checkSixAxis(); checkI2cStress(); printSummary();
        break;
      case 'c':
        calibrateStaticBias();
        break;
      case 'm':
        checkSixAxis();
        break;
      default:
        vdPrintf("收到 '%c' (0x%02X)，本程序只做自检；输 '?' 看命令\n", c, (uint8_t)c);
        break;
    }
  }
}
