/* uart_loopback.ino — GPIO21 发送 / GPIO20 接收 的裸 UART 测试
 *
 * 用途：单独验证 ESP32-C3 的 UART 引脚（TX=GPIO21, RX=GPIO20）能否收发。
 *   不依赖 vibmon 固件、不依赖 MPU6050。
 *
 * 行为：
 *   - TX(GPIO21) 每 500ms 发一行 "PING n"。
 *   - RX(GPIO20) 收到任何字节，都原样报到 USB 串口（COM9, 115200）。
 *   - USB 串口收到的字符会转发到 TX(GPIO21)。
 *
 * 编译/烧录（想从 USB 看结果必须用 CDC 构建）：
 *   fqbn = esp32:esp32:esp32c3:CDCOnBoot=cdc
 *
 * 自测：把 GPIO21 和 GPIO20 短接，COM9 上会看到
 *   [RX n] ... 'P' / 'I' / 'N' / 'G' ... 这样逐字符回显 PING，
 *   说明 21/20 两脚收发都正常 —— 那就只差 CH340 的接线了。
 */

static uint32_t g_txSeq   = 0;
static uint32_t g_rxBytes = 0;
static uint32_t g_lastTx  = 0;

void setup() {
  Serial.begin(115200);                                    // USB CDC
  Serial1.begin(115200, SERIAL_8N1, /*rx*/ 20, /*tx*/ 21); // 21 发、20 收
  delay(300);
  Serial.println();
  Serial.println("===== UART loopback test =====");
  Serial.println("TX = GPIO21   RX = GPIO20   115200 8N1");
  Serial.println("TX 每 500ms 发一行 PING；RX 收到什么都按 [RX] 上报");
  Serial.println("把 GPIO21 和 GPIO20 短接，应看到 PING 的逐字节回显");
}

void loop() {
  // 1) 周期发送
  const uint32_t now = millis();
  if (now - g_lastTx >= 500) {
    g_lastTx = now;
    Serial1.printf("PING %lu\r\n", (unsigned long)g_txSeq++);
  }

  // 2) 收 GPIO20 的字节，原样报到 USB
  while (Serial1.available() > 0) {
    const int c = Serial1.read();
    g_rxBytes++;
    Serial.print("[RX ");
    Serial.print(g_rxBytes);
    Serial.print("] ");
    Serial.print(c);
    Serial.print(" 0x");
    if (c < 16) Serial.print('0');
    Serial.print(c, HEX);
    if (c >= 32 && c < 127) {
      Serial.print(" '");
      Serial.print((char)c);
      Serial.print('\'');
    }
    Serial.println();
  }

  // 3) USB 收到什么就转发到 TX
  while (Serial.available() > 0) {
    Serial1.write((uint8_t)Serial.read());
  }
}
