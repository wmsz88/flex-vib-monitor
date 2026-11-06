#include <Arduino.h>
#line 1 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
/**
  ******************************************************************************
  * @file    vibsource_c3_key.ino
  * @brief   双路振动源 + 4 物理按键（ESP32-C3 SuperMini）  T009
  *
  * 基于 vibsource_esp32_dual 增量改造：串口命令与状态输出沿用，新增 4 个按键。
  *
  * 【通道（按实况）】A = GPIO3 = 新低速振子；B = GPIO4 = 原振子
  * 【按键】
  *   KEY1(GPIO7)  短按 = 选源三态循环 A -> B -> OFF -> A；长按>=1s = 手动/自动 切换
  *   KEY2(GPIO10) 手动 = 跳当前振子默认占空比；自动 = 切周期档 0.5->1->2->3Hz
  *   KEY3(GPIO0)  当前振子占空比 +5%
  *   KEY4(GPIO1)  当前振子占空比 -5%
  * 【串口命令】沿用：a/b 选源, 0~9/h/p/+/- 占空比, T0~T3/T 自动通断, s/x 扫频,
  *             z 两路停, q 状态, ? 帮助
  * 【状态输出】按键操作与串口命令共用同一状态机；printStatus 增加 [SRC]/[MODE] 行。
  ******************************************************************************
  */

#include "config.h"

/* ---------- 状态 ---------- */
enum Src  { SRC_A = 0, SRC_B = 1, SRC_OFF = 2 };
enum Mode { MODE_MANUAL = 0, MODE_AUTO = 1 };

struct Motor {
  uint8_t pin;
  uint8_t duty;      /* 手动模式记忆值 */
};

static Motor g_m[2] = {
  { MOTOR_PIN_A, DUTY_START },
  { MOTOR_PIN_B, DUTY_START },
};
static uint8_t g_out[2] = { 0, 0 };        /* 实际输出占空比 */

static Src  g_src  = SRC_OFF;
static Mode g_mode = MODE_MANUAL;

static bool     g_sweep   = false;
static uint32_t g_sweepMs = 0;

static bool     g_auto     = false;        /* 自动通断运行中 */
static uint8_t  g_cycleIdx = 1;            /* 默认 1Hz 档 */
static bool     g_cycleOn  = true;
static uint32_t g_cycleMs  = 0;
static uint32_t g_cycleCnt = 0;

static uint32_t g_beatMs = 0;
static bool     g_ledOn  = false;

/* ---------- PWM / LED ---------- */

#line 54 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void pwmWrite(uint8_t ch, uint8_t dutyPct);
#line 61 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void ledWrite(bool on);
#line 72 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void setOut(uint8_t ch, uint8_t d);
#line 85 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void outAllZero();
#line 93 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static const char * srcName(Src s);
#line 102 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void printOne(uint8_t ch);
#line 110 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void printStatus();
#line 126 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void printHelp();
#line 142 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void applyManual();
#line 153 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void autoStart(uint8_t idx);
#line 166 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void autoStop();
#line 172 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void setMode(Mode m);
#line 185 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void selectSource(Src s);
#line 197 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void autoTick();
#line 224 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void sweepStep();
#line 236 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void dutyOfCurrent(uint8_t d);
#line 243 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void handleCmd(char c);
#line 311 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static uint8_t scanShortPress();
#line 341 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void handleShortPress(uint8_t k);
#line 369 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
void setup();
#line 399 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
void loop();
#line 54 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\vibsource_c3_key.ino"
static void pwmWrite(uint8_t ch, uint8_t dutyPct)
{
  if (dutyPct > 100) dutyPct = 100;
  uint32_t cnt = ((uint32_t)dutyPct * MOTOR_PWM_MAX + 50) / 100;
  ledcWrite(g_m[ch].pin, cnt);
}

static void ledWrite(bool on)
{
#if LED_ACTIVE_LOW
  digitalWrite(LED_PIN, on ? LOW : HIGH);
#else
  digitalWrite(LED_PIN, on ? HIGH : LOW);
#endif
  g_ledOn = on;
}

/* 写实际输出（带踢一脚；不碰手动记忆值） */
static void setOut(uint8_t ch, uint8_t d)
{
  if (d > DUTY_MAX) d = DUTY_MAX;
#if KICK_ENABLE
  if (g_out[ch] == 0 && d > 0) {
    pwmWrite(ch, KICK_DUTY);
    delay(KICK_MS);
  }
#endif
  g_out[ch] = d;
  pwmWrite(ch, d);
}

static void outAllZero()
{
  setOut(0, 0);
  setOut(1, 0);
}

/* ---------- 状态输出 ---------- */

static const char* srcName(Src s)
{
  switch (s) {
    case SRC_A: return "A(新 GPIO3)";
    case SRC_B: return "B(旧 GPIO4)";
    default:    return "OFF";
  }
}

static void printOne(uint8_t ch)
{
  Serial.printf("[CH %c] GPIO%u  %3u%%  (out %3u%%)%s\n",
                (ch == 0 ? 'A' : 'B'), g_m[ch].pin, (unsigned)g_m[ch].duty,
                (unsigned)g_out[ch],
                (g_src == (Src)ch ? "  <- 当前" : ""));
}

static void printStatus()
{
  printOne(0);
  printOne(1);
  Serial.printf("[SRC ] %s\n", srcName(g_src));
  if (g_mode == MODE_AUTO) {
    Serial.printf("[MODE] 自动 %u.%uHz%s\n",
                  (unsigned)(CYCLE_TABLE[g_cycleIdx][0] / 10),
                  (unsigned)(CYCLE_TABLE[g_cycleIdx][0] % 10),
                  g_auto ? "" : " (空闲:源为OFF)");
  } else {
    Serial.println("[MODE] 手动");
  }
  if (g_sweep) Serial.println("  [扫频中]");
}

static void printHelp()
{
  Serial.println();
  Serial.println("===== vibsource C3-KEY (ESP32-C3 SuperMini + 2x ERM + 4 按键) =====");
  Serial.printf("PWM A: GPIO%d(新)  PWM B: GPIO%d(旧)  %d Hz / %d bit\n",
                MOTOR_PIN_A, MOTOR_PIN_B, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  Serial.printf("KEY1 GPIO%d 短按=选源循环 / 长按>=%dms=手动<->自动\n", KEY1_PIN, KEY_LONGPRESS_MS);
  Serial.printf("KEY2 GPIO%d 手动=跳默认 / 自动=切周期档\n", KEY2_PIN);
  Serial.printf("KEY3 GPIO%d 占空比+%d%%    KEY4 GPIO%d 占空比-%d%%\n",
                KEY3_PIN, DUTY_STEP, KEY4_PIN, DUTY_STEP);
  Serial.println("串口: a/b 选源 | 0~9/h/p/+/- 占空比 | T0~T3/T 自动通断 | s/x 扫频 | z 两路停 | q 状态 | ? 帮助");
  Serial.println();
}

/* ---------- 输出应用（手动模式） ---------- */

static void applyManual()
{
  if (g_mode != MODE_MANUAL) return;
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t want = (g_src == (Src)i) ? g_m[i].duty : 0;
    if (want != g_out[i]) setOut(i, want);
  }
}

/* ---------- 自动通断 ---------- */

static void autoStart(uint8_t idx)
{
  if (idx >= CYCLE_N) idx = 0;
  g_cycleIdx = idx;
  if (g_src == SRC_OFF) { g_auto = false; return; }
  outAllZero();
  g_auto = true; g_cycleOn = true; g_cycleMs = millis(); g_cycleCnt = 0;
  setOut((uint8_t)g_src, CYCLE_ON_DUTY);
  Serial.printf("[AUTO] %s  %.1f Hz (ON %u / OFF %u ms)\n",
                srcName(g_src), CYCLE_TABLE[idx][0] / 10.0f,
                CYCLE_TABLE[idx][1], CYCLE_TABLE[idx][2]);
}

static void autoStop()
{
  g_auto = false;
  if (g_src != SRC_OFF) setOut((uint8_t)g_src, 0);
}

static void setMode(Mode m)
{
  if (m == MODE_AUTO) {
    g_sweep = false;
    g_mode = MODE_AUTO;
    autoStart(g_cycleIdx);
  } else {
    autoStop();
    g_mode = MODE_MANUAL;
    applyManual();
  }
}

static void selectSource(Src s)
{
  g_sweep = false;
  g_src = s;
  if (g_mode == MODE_AUTO) {
    autoStop();
    if (s != SRC_OFF) autoStart(g_cycleIdx);
  } else {
    applyManual();
  }
}

static void autoTick()
{
  if (!g_auto || g_src == SRC_OFF) return;
  const uint32_t now = millis();
  const uint16_t onMs  = CYCLE_TABLE[g_cycleIdx][1];
  const uint16_t offMs = CYCLE_TABLE[g_cycleIdx][2];
  if (g_cycleOn) {
    if ((uint32_t)(now - g_cycleMs) >= onMs) {
      setOut((uint8_t)g_src, 0);
      g_cycleOn = false; g_cycleMs = now;
    }
  } else {
    if ((uint32_t)(now - g_cycleMs) >= offMs) {
#if KICK_ENABLE
      pwmWrite((uint8_t)g_src, KICK_DUTY);
      delay(CYCLE_KICK_MS);
#endif
      /* 自动重启已经手动踢过一脚，不能再经 setOut() 触发 200ms 启动踢 */
      g_out[g_src] = CYCLE_ON_DUTY;
      pwmWrite((uint8_t)g_src, CYCLE_ON_DUTY);
      g_cycleOn = true; g_cycleMs = now; g_cycleCnt++;
    }
  }
}

/* ---------- 扫频 ---------- */

static void sweepStep()
{
  if (g_src == SRC_OFF || g_mode != MODE_MANUAL) { g_sweep = false; return; }
  uint8_t d = g_m[g_src].duty;
  if (d > DUTY_STEP) { g_m[g_src].duty = (uint8_t)(d - DUTY_STEP); }
  else { g_m[g_src].duty = DUTY_MIN; g_sweep = false;
         Serial.println("[SWEEP] 完成，停在最低档"); }
  applyManual();
}

/* ---------- 串口命令 ---------- */

static void dutyOfCurrent(uint8_t d)
{
  if (g_src == SRC_OFF) { Serial.println("[!] 当前为 OFF，先按 KEY1/串口 a|b 选源"); return; }
  g_m[g_src].duty = (d > DUTY_MAX) ? DUTY_MAX : d;
  applyManual();
}

static void handleCmd(char c)
{
  if (c >= '0' && c <= '9') { g_sweep = false; dutyOfCurrent((uint8_t)((c - '0') * 10)); printStatus(); return; }

  switch (c) {
    case 'a': selectSource(SRC_A); printStatus(); break;
    case 'b': selectSource(SRC_B); printStatus(); break;
    case 'z': autoStop(); selectSource(SRC_OFF); printStatus(); break;

    case 'h': g_sweep = false; dutyOfCurrent(100); printStatus(); break;
    case 'p': g_sweep = false;
              if (g_src == SRC_OFF) { Serial.println("[!] 当前为 OFF"); break; }
              dutyOfCurrent(g_src == SRC_A ? DUTY_DEFAULT_A : DUTY_DEFAULT_B);
              printStatus(); break;
    case '+': g_sweep = false;
              if (g_src != SRC_OFF) { uint8_t d = g_m[g_src].duty + DUTY_STEP;
                                      dutyOfCurrent(d > DUTY_MAX ? DUTY_MAX : d); }
              printStatus(); break;
    case '-': g_sweep = false;
              if (g_src != SRC_OFF) { uint8_t d = (g_m[g_src].duty > DUTY_STEP) ? (uint8_t)(g_m[g_src].duty - DUTY_STEP) : DUTY_MIN;
                                      dutyOfCurrent(d); }
              printStatus(); break;

    case 's': if (g_src != SRC_OFF && g_mode == MODE_MANUAL) {
                g_sweep = true; g_sweepMs = millis(); g_m[g_src].duty = SWEEP_FROM; applyManual();
                Serial.println("[SWEEP] 开始 100%->0%，每档 3s"); }
              printStatus(); break;
    case 'x': g_sweep = false; Serial.println("[SWEEP] 停止"); printStatus(); break;

    case 'T':
      if (Serial.available()) {
        char d = (char)Serial.peek();
        if (d >= '0' && d <= '3') {
          Serial.read();
          g_sweep = false;
          g_mode = MODE_AUTO;
          autoStart((uint8_t)(d - '0'));
          printStatus();
          break;
        }
      }
      setMode(MODE_MANUAL); printStatus(); break;

    case 'q': printStatus(); break;
    case '?': printHelp(); break;
    default: break;
  }
}

/* ---------- 按键（非阻塞消抖 + 长按） ---------- */

struct Key {
  uint8_t  pin;
  bool     stable;        /* true=按下 */
  bool     lastRaw;
  uint32_t rawMs;
  uint32_t downMs;
  bool     longFired;
};

static Key g_key[4] = {
  { KEY1_PIN, false, false, 0, 0, false },
  { KEY2_PIN, false, false, 0, 0, false },
  { KEY3_PIN, false, false, 0, 0, false },
  { KEY4_PIN, false, false, 0, 0, false },
};

/* 扫描：返回 1..4 表示该键"短按"（长按已消费则本按不再回短按）；0 = 无 */
static uint8_t scanShortPress()
{
  uint8_t ev = 0;
  uint32_t now = millis();

  for (uint8_t i = 0; i < 4; i++) {
    bool raw = (digitalRead(g_key[i].pin) == LOW);
    if (raw != g_key[i].lastRaw) { g_key[i].lastRaw = raw; g_key[i].rawMs = now; }

    if ((uint32_t)(now - g_key[i].rawMs) >= KEY_DEBOUNCE_MS && raw != g_key[i].stable) {
      g_key[i].stable = raw;
      if (raw) {                                 /* 刚按下 */
        g_key[i].downMs = now;
        g_key[i].longFired = false;
      } else {                                   /* 刚松开 */
        if (!g_key[i].longFired && ev == 0) ev = (uint8_t)(i + 1);
      }
    }

    /* KEY1 长按：按下持续 >= 阈值 且尚未触发 */
    if (i == 0 && g_key[i].stable && !g_key[i].longFired &&
        (uint32_t)(now - g_key[i].downMs) >= KEY_LONGPRESS_MS) {
      g_key[i].longFired = true;
      setMode(g_mode == MODE_MANUAL ? MODE_AUTO : MODE_MANUAL);
      printStatus();
    }
  }
  return ev;
}

static void handleShortPress(uint8_t k)
{
  if (k == 1) {                                   /* 选源三态循环 */
    Src nxt = (g_src == SRC_A) ? SRC_B : (g_src == SRC_B) ? SRC_OFF : SRC_A;
    selectSource(nxt);
  } else if (k == 2) {                            /* 跳默认 / 切周期档 */
    if (g_mode == MODE_MANUAL) {
      if (g_src != SRC_OFF) {
        g_m[g_src].duty = (g_src == SRC_A) ? DUTY_DEFAULT_A : DUTY_DEFAULT_B;
        applyManual();
      }
    } else {
      autoStart((uint8_t)((g_cycleIdx + 1) % CYCLE_N));
    }
  } else if (k == 3 || k == 4) {                  /* 占空比 ±STEP（改手动记忆值） */
    if (g_src != SRC_OFF) {
      int d = (int)g_m[g_src].duty + (k == 3 ? DUTY_STEP : -DUTY_STEP);
      if (d < DUTY_MIN) d = DUTY_MIN;
      if (d > DUTY_MAX) d = DUTY_MAX;
      g_m[g_src].duty = (uint8_t)d;
      if (g_mode == MODE_MANUAL) applyManual();
    }
  }
  printStatus();
}

/* ---------- setup / loop ---------- */

void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(300);

  if (!ledcAttach(MOTOR_PIN_A, MOTOR_PWM_HZ, MOTOR_PWM_BITS))
    Serial.printf("[错误] LEDC 初始化失败（GPIO%d）\n", MOTOR_PIN_A);
  if (!ledcAttach(MOTOR_PIN_B, MOTOR_PWM_HZ, MOTOR_PWM_BITS))
    Serial.printf("[错误] LEDC 初始化失败（GPIO%d）\n", MOTOR_PIN_B);
  ledcWrite(MOTOR_PIN_A, 0);
  ledcWrite(MOTOR_PIN_B, 0);

  pinMode(LED_PIN, OUTPUT);
  ledWrite(false);

  pinMode(KEY1_PIN, INPUT_PULLUP);
  pinMode(KEY2_PIN, INPUT_PULLUP);
  pinMode(KEY3_PIN, INPUT_PULLUP);
  pinMode(KEY4_PIN, INPUT_PULLUP);

  g_m[0].duty = DUTY_START; g_m[1].duty = DUTY_START;
  g_out[0] = 0; g_out[1] = 0;
  g_src = SRC_OFF; g_mode = MODE_MANUAL;

  Serial.println();
  Serial.println("vibsource C3-KEY 已启动（A=GPIO3 新 / B=GPIO4 旧；4 按键就绪）");
  printHelp();
  printStatus();
}

void loop()
{
  while (Serial.available()) handleCmd((char)Serial.read());

  uint8_t k = scanShortPress();
  if (k) handleShortPress(k);

  if (g_sweep && g_mode == MODE_MANUAL && (millis() - g_sweepMs >= SWEEP_DWELL_MS)) {
    g_sweepMs = millis();
    sweepStep();
    printStatus();
  }

  autoTick();

  const bool running = (g_out[0] > 0) || (g_out[1] > 0) || (g_auto && g_cycleOn);
  if (running) { if (!g_ledOn) ledWrite(true); }
  else if (millis() - g_beatMs >= 500) { g_beatMs = millis(); ledWrite(!g_ledOn); }

  delay(LOOP_TICK_MS);
}

