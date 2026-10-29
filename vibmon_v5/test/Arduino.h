// 主机测试用的最小 Arduino 桩（v5）
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979f
#endif

uint32_t millis(void);
uint32_t micros(void);
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
