#include <Arduino.h>

#define E220_RX  44
#define E220_TX  43
#define E220_M0  40
#define E220_M1  39
#define E220_AUX 38

// JP版の標準的な6バイト設定 (ADDH, ADDL, NETID, REG0, REG1, REG2)
const uint8_t config_cmd_6byte[] = {
  0xC0, 0x00, 0x06, 
  0x00, 0x00, // Address: 0
  0x00,       // NetID: 0 (JP版は00固定)
  0x62,       // REG0: 9600bps, Air 2.4kbps
  0x00,       // REG1: 13dBm
  0x18        // REG2: 24ch (920.6MHz)
};

void waitAUX() {
  delay(200);
  while (digitalRead(E220_AUX) == LOW) delay(1);
  delay(200);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, E220_RX, E220_TX);
  pinMode(E220_M0, OUTPUT);
  pinMode(E220_M1, OUTPUT);
  pinMode(E220_AUX, INPUT);

  Serial.println("\n--- Step 1: Read Module Version ---");
  digitalWrite(E220_M0, HIGH);
  digitalWrite(E220_M1, HIGH);
  delay(1000); // モード切替を確実に待つ

  // バージョン読み出しコマンド
  uint8_t ver_cmd[] = {0xC3, 0xC3, 0xC3};
  Serial1.write(ver_cmd, 3);
  waitAUX();
  Serial.print("Module Info: ");
  while (Serial1.available()) Serial.printf("%02X ", Serial1.read());
  Serial.println();

  Serial.println("\n--- Step 2: Try 6-byte Write ---");
  Serial1.write(config_cmd_6byte, sizeof(config_cmd_6byte));
  waitAUX();
  
  // 結果確認
  uint8_t read_cmd[] = {0xC1, 0x00, 0x09};
  Serial1.write(read_cmd, 3);
  waitAUX();
  Serial.print("Final Config Result: ");
  while (Serial1.available()) Serial.printf("%02X ", Serial1.read());
  Serial.println("\n--- Finished ---");

  digitalWrite(E220_M0, LOW);
  digitalWrite(E220_M1, LOW);
}

void loop() {}