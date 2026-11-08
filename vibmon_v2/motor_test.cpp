/**
  * @file    motor_test.cpp
  * @brief   【临时调试用】同板驱动振动马达（见 motor_test.h 说明）
  *
  * MOTOR_TEST_ENABLE=0 时整个文件编译为空，不占引脚、零开销。
  */

#include "motor_test.h"

#if MOTOR_TEST_ENABLE

static uint8_t  s_duty    = 0;
static bool     s_sweep   = false;
static uint32_t s_lastMs  = 0;

void motorTestBegin(void) {
  /* ESP32 core 3.x：ledcAttach 用**引脚号**（旧的 ledcSetup/ledcAttachPin 已移除） */
  if (!ledcAttach(MOTOR_TEST_PIN, MOTOR_TEST_PWM_HZ, MOTOR_TEST_PWM_BITS)) {
    Serial.printf("#[motor] LEDC 初始化失败（GPIO%d）\n", MOTOR_TEST_PIN);
    return;
  }
  ledcWrite(MOTOR_TEST_PIN, 0);
  s_duty = 0;
  Serial.printf("#[motor] 就绪：GPIO%d / %d Hz / %d bit；命令 m0~m9 M P + - S\n",
                MOTOR_TEST_PIN, MOTOR_TEST_PWM_HZ, MOTOR_TEST_PWM_BITS);
}

uint8_t motorTestDuty(void) { return s_duty; }
bool    motorTestSweepActive(void) { return s_sweep; }

void motorTestSetDutyDigit(char digit) {
  if (digit < '0' || digit > '9') return;
  motorTestSetDuty((uint8_t)((digit - '0') * 10));
}

void motorTestSetDuty(uint8_t pct) {
  if (pct > 100) pct = 100;
  const uint8_t was = s_duty;
  s_duty = pct;

  /* 从静止启动：先满压踢一脚，再落到目标值。
   * 为什么需要：启动门槛约 74%，低于此值马达静止；
   * 但转起来后维持转动所需电压更低（迟滞），踢一下就能在低占空比维持。 */
  if (was == 0 && pct > 0) {
    ledcWrite(MOTOR_TEST_PIN, MOTOR_TEST_PWM_MAX);
    delay(MOTOR_TEST_KICK_MS);
  }
  const uint32_t cnt = ((uint32_t)pct * MOTOR_TEST_PWM_MAX + 50) / 100;
  ledcWrite(MOTOR_TEST_PIN, cnt);
  Serial.printf("#[motor] duty %u%%\n", (unsigned)pct);
}

/* 扫频一步：降一档；到底停住（保持在最低档，不归零） */
static void sweepStep(void) {
  if (s_duty > MOTOR_TEST_SWEEP_LO + MOTOR_TEST_SWEEP_STEP) {
    motorTestSetDuty((uint8_t)(s_duty - MOTOR_TEST_SWEEP_STEP));
  } else {
    motorTestSetDuty((uint8_t)MOTOR_TEST_SWEEP_LO);
    s_sweep = false;
    Serial.println("#[motor] 扫频完成");
  }
}

void motorTestTick(uint32_t nowMs) {
  if (!s_sweep) return;
  if ((uint32_t)(nowMs - s_lastMs) < (uint32_t)MOTOR_TEST_SWEEP_MS) return;
  s_lastMs = nowMs;
  sweepStep();
}

/**
  * @brief  处理马达相关的单字符命令
  *
  * 【为什么 'm' 不在这里处理数字】
  *   'm0'~'m9' 需要读下一个字符（peek），而 peek 只有调用方（output.cpp
  *   的 outPollCmd）能安全做 —— 命令解析集中在一处，避免两个模块
  *   各自碰串口缓冲。所以这里只处理**单字符**命令，
  *   带参数的 'm<digit>' 由调用方消费 'm' 后调 motorTestSetDutyDigit()。
  *
  * 【字符选择】必须与 outPollCmd 已有 case 不重叠，否则 duplicate case。
  */
bool motorTestHandleCmd(char c) {
  switch (c) {
    case 'H': motorTestSetDuty(100); return true;
    case 'P': motorTestSetDuty(55);  return true;
    case 'q': motorTestSetDuty((uint8_t)min((int)s_duty + 5, 100)); return true;
    case 'r': motorTestSetDuty((uint8_t)max((int)s_duty - 5,   0)); return true;
    case 'W':
      s_sweep  = true;
      s_lastMs = millis();
      motorTestSetDuty(MOTOR_TEST_SWEEP_HI);
      Serial.printf("#[motor] 扫频 %d%% -> %d%%，每档 %d ms\n",
                    MOTOR_TEST_SWEEP_HI, MOTOR_TEST_SWEEP_LO, MOTOR_TEST_SWEEP_MS);
      return true;
    default: return false;
  }
}

#endif /* MOTOR_TEST_ENABLE */
