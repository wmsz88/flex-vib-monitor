// 主机测试用的最小 Arduino 桩
//
// 【作用】dsp.cpp 与 state.cpp 里 #include <Arduino.h>，但算法本身只用到了
// 时间函数、PI 和少量数学函数，不依赖 ESP32 硬件。这个头文件在 PC 上提供
// 同名的最小实现（具体定义在 host_test.cpp 里），使这两个核心模块可以脱离
// 硬件在 PC 上编译运行，从而对算法做回归测试。
// 【为什么不桩掉 Wire/esp_timer】dsp 与 state 不直接调用它们（只通过 mpu.h /
// sampler.h 的接口间接使用），因此测试里把那些接口写成空实现即可。
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#ifndef PI
// ESP32 的 Arduino 核心在 math.h 之外还定义了 PI；PC 上未必有，这里补上。
// 用 f 后缀保证与工程中其他浮点常量类型一致（避免 double/float 混算告警）。
#define PI 3.14159265358979f
#endif
// 以下三个由 host_test.cpp 实现：时间用可控的全局变量模拟，便于加速仿真
uint32_t millis(void);
uint32_t micros(void);
void delay(uint32_t ms);
