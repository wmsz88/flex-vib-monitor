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

/* Arduino 常量。主机测试不真正驱动 GPIO，只让 output.cpp 能编译。 */
#define HIGH 1
#define LOW  0
#define OUTPUT 1
#define INPUT  0
#define SERIAL_8N1 0x800001c

/* Serial/Serial1 空桩。
 * comp_frame_test 只验证 compBuildFrame() 纯编码逻辑，
 * 不应依赖任何真实串口行为。 */
class SerialStub {
public:
  void begin(unsigned long) {}
  void begin(unsigned long, uint8_t, int, int) {}
  void begin(unsigned long, int, int, int) {}
  int available(void) { return (int)(m_rxTail - m_rxHead); }
  int peek(void) { return available() ? m_rx[m_rxHead & 63u] : -1; }
  int read(void) { return available() ? m_rx[m_rxHead++ & 63u] : -1; }
  void write(const uint8_t* b, size_t n) {
    for (size_t i = 0; i < n && m_txN + 1 < sizeof(m_tx); i++) m_tx[m_txN++] = (char)b[i];
    m_tx[m_txN] = '\0';
  }
  void setTimeout(unsigned long) {}
  void setTxTimeoutMs(unsigned long) {}

  /* ---- 测试用：喂命令 / 读回显。默认空，不影响只测纯编码的用例。 ---- */
  void push(char c) { if (available() < 64) m_rx[m_rxTail++ & 63u] = (uint8_t)c; }
  void clearTx(void) { m_txN = 0; m_tx[0] = '\0'; }
  const char* tx(void) const { return m_tx; }

private:
  uint8_t m_rx[64] = {};
  unsigned m_rxHead = 0, m_rxTail = 0;
  char     m_tx[512] = {};
  size_t   m_txN = 0;
};
extern SerialStub Serial;
extern SerialStub Serial1;

/* LED 驱动桩。默认空实现；led_test 用 ledWriteHook() = fn 挂钩子抓引脚电平。 */
inline void (*&ledWriteHook(void))(int, int) {
  static void (*hook)(int, int) = nullptr;
  return hook;
}
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int val) { if (ledWriteHook()) ledWriteHook()(pin, val); }
