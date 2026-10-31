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

#define HIGH 1
#define LOW  0
#define OUTPUT 1
#define INPUT  0
#define SERIAL_8N1 0x800001c

class SerialStub {
public:
  void begin(unsigned long) {}
  void begin(unsigned long, uint8_t, int, int) {}
  void begin(unsigned long, int, int, int) {}
  int available(void) { return 0; }
  int peek(void) { return -1; }
  int read(void) { return -1; }
  void write(const uint8_t*, size_t) {}
  void setTimeout(unsigned long) {}
  void setTxTimeoutMs(unsigned long) {}
};
extern SerialStub Serial;
extern SerialStub Serial1;

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}