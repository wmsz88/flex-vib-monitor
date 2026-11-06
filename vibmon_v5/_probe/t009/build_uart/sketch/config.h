#line 1 "C:\\Users\\Lenovo\\Desktop\\STM32project\\vibsource_c3_key\\config.h"
/**
  ******************************************************************************
  * @file    config.h
  * @brief   【双路振动源 + 4 按键 · 唯一参数总表】(T009)
  *
  * 在 vibsource_esp32_dual 基础上新增 4 个物理按键；串口命令行与状态输出沿用。
  *
  * 【通道标签按实况（T006 修正档案）】
  *   A = GPIO3 = 新低速振子（实测 71~120Hz）
  *   B = GPIO4 = 原振子    （实测 101~175.7Hz）
  *
  * 【ESP32-C3 引脚禁区】GPIO11~17=Flash、GPIO18/19=USB、GPIO2/8/9=strapping；
  * 本工程已用 GPIO3/4（PWM）、GPIO8（LED）；剩余安全脚 GPIO0/1/7/10 给按键。
  ******************************************************************************
  */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <Arduino.h>

/* ---------- 两路马达 PWM 输出 ---------- */
#define MOTOR_PIN_A         3        /* 通道 A：新低速振子 */
#define MOTOR_PIN_B         4        /* 通道 B：原振子 */

#define MOTOR_PWM_HZ        20000
#define MOTOR_PWM_BITS      10
#define MOTOR_PWM_MAX       ((1 << MOTOR_PWM_BITS) - 1)

/* ---------- 板载 LED（低电平点亮） ---------- */
#define LED_PIN             8
#define LED_ACTIVE_LOW      1

/* ---------- 4 个按键（INPUT_PULLUP，按下读 LOW，另一端接 GND） ---------- */
#define KEY1_PIN            7        /* 短按=选源三态循环；长按>=1s=手动/自动 */
#define KEY2_PIN            10       /* 手动=跳默认；自动=切周期档 */
#define KEY3_PIN            0        /* 当前振子占空比 +5% */
#define KEY4_PIN            1        /* 当前振子占空比 -5% */
#define KEY_DEBOUNCE_MS     15       /* 消抖时间 */
#define KEY_LONGPRESS_MS    1000     /* KEY1 长按阈值 */

/* ---------- 占空比范围（对外统一 0~100） ---------- */
#define DUTY_MIN            0
#define DUTY_MAX            100
#define DUTY_STEP           5
#define DUTY_START          0

/* ---------- 各振子「一键默认」占空比（KEY2 手动模式 / 串口 'p'） ----------
 * 占位值，人类实板以「vibmon 稳定检出 ct=1」为准后改这里重烧。 */
#define DUTY_DEFAULT_A      70       /* 新振子 GPIO3 */
#define DUTY_DEFAULT_B      100      /* 原振子 GPIO4 */

/* 保留旧串口 'p' 语义的参考工作点（55% ≈ 80Hz），供说明用 */
#define PRESET_DUTY         55

/* ---------- 马达标称参数（仅打印参考） ---------- */
#define MOTOR_RPM_RATED     9000

/* ---------- 「踢一脚」启动（0 -> 非0 时给满压脉冲） ---------- */
#define KICK_ENABLE         1
#define KICK_DUTY           100
#define KICK_MS             200

/* ---------- 串口 ---------- */
#define SERIAL_BAUD         115200

/* ---------- 自动通断（0~3Hz，作用于当前选中通道） ---------- */
#define CYCLE_ON_DUTY       85
#define CYCLE_KICK_MS       80

#define CYCLE_N             4
static const uint16_t CYCLE_TABLE[CYCLE_N][3] = {
  {  5, 1000, 1000 },   /* 0.5 Hz */
  { 10,  500,  500 },   /* 1.0 Hz */
  { 20,  250,  250 },   /* 2.0 Hz */
  { 30,  167,  166 },   /* 3.0 Hz */
};

/* ---------- 主循环节拍 ---------- */
#define LOOP_TICK_MS        5

/* ---------- 扫频 ---------- */
#define SWEEP_FROM          100
#define SWEEP_DWELL_MS      3000

#endif /* __CONFIG_H */
