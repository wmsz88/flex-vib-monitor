/**
  ******************************************************************************
  * @file    motor_test.h
  * @brief   【临时调试用】在同一块板上驱动振动马达
  *
  * 【为什么需要】
  *   正常情况下马达由独立的 vibsource 板驱动（COM7），本板只负责测量（COM8）。
  *   但在"只有一块板"的调试场景下，需要同一块板既驱动马达又测 MPU6050，
  *   用于验证"马达频率 -> 测量读数"的对应关系。
  *
  * 【默认关闭】
  *   MOTOR_TEST_ENABLE 默认 0 —— 不编译、不占引脚、零开销。
  *   需要时改成 1 重新编译，测完再改回 0（或直接覆盖回正式固件）。
  *
  * 【引脚】GPIO3
  *   本板已用：GPIO8/9(I2C) GPIO10(绿灯) GPIO5(黄灯) GPIO20/21(串口)
  *   GPIO3 空闲，且不是 strapping/USB/Flash 脚，可安全用作 LEDC 输出。
  *
  * 【串口命令】（仅 MOTOR_TEST_ENABLE=1 时生效）
  *   m0~m9   占空比 = 数字 x 10%（m0 = 停）
  *   H       100%
  *   P       55%（80Hz 工作点）
  *   q / r   占空比 +5% / -5%
  *   W       自动降档扫频 100%->55%，每档 3 秒（测占空比-频率曲线用）
  *
  * 【为什么不用 S / + / -】
  *   这几个字符已被正式功能占用（S=状态汇总、+/- 未定义但 S 冲突会编译报错）。
  *   命令字符必须与 outPollCmd 里已有的 case 完全不重叠，
  *   否则 `duplicate case value` 编译失败。
  *   已占用：? 0 1 2 3 4 p d S T c w n t k K m M
  *
  * 【重要：占空比 != 振动频率】
  *   这是 ERM 偏心转子马达，振动频率 = 转速 / 60。
  *   PWM 载波固定 20kHz（避开人耳啸叫），只负责调"平均电压 -> 转速"。
  ******************************************************************************
  */

#ifndef __MOTOR_TEST_H
#define __MOTOR_TEST_H

#include "config.h"

/* 【默认关闭】正式固件不带马达驱动。调试"一板方案"时改成 1。 */
#ifndef MOTOR_TEST_ENABLE
#define MOTOR_TEST_ENABLE 0
#endif

#if MOTOR_TEST_ENABLE

#include <Arduino.h>

/* 马达 PWM 脚。与 vibsource 保持一致（GPIO3）。 */
#define MOTOR_TEST_PIN      3

/* LEDC 载波：20kHz 超出声范围；10bit 对应 0~1023 */
#define MOTOR_TEST_PWM_HZ   20000
#define MOTOR_TEST_PWM_BITS 10
#define MOTOR_TEST_PWM_MAX  ((1 << MOTOR_TEST_PWM_BITS) - 1)

/* 启动门槛约 74%，低于此值转不起来。所以从静止启动时先"踢一脚"。 */
#define MOTOR_TEST_KICK_DUTY 100
#define MOTOR_TEST_KICK_MS   200

/* 扫频参数 */
#define MOTOR_TEST_SWEEP_HI 100
#define MOTOR_TEST_SWEEP_LO 55
#define MOTOR_TEST_SWEEP_STEP 5
#define MOTOR_TEST_SWEEP_MS 3000

void     motorTestBegin(void);
void     motorTestSetDuty(uint8_t pct);
void     motorTestSetDutyDigit(char digit);  // '0'~'9' → 0%~90%
uint8_t  motorTestDuty(void);
bool     motorTestSweepActive(void);
void     motorTestTick(uint32_t nowMs);      // 主循环每圈调用（推进扫频）
bool     motorTestHandleCmd(char c);         // 返回 true 表示已消费该字符

#endif /* MOTOR_TEST_ENABLE */
#endif /* __MOTOR_TEST_H */
