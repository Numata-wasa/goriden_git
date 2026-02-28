#include <Arduino.h>

// ピン定義 (ESP32-S3)
#define E220_RX 39
#define E220_TX 38
#define E220_M0 12
#define E220_M1 11
#define E220_AUX 40

void waitAUX() {
  while (digitalRead(E220_AUX) == LOW) {
    delay(1);
  }
  delay(10);
}

void setMode(int mode) {
  waitAUX();
  if (mode == 3) { // 設定・読み出しモード
    digitalWrite(E220_M0, HIGH);
    digitalWrite(E220_M1, HIGH);
  } else {        // 通常モード
    digitalWrite(E220_M0, LOW);
    digitalWrite(E220_M1, LOW);
  }
  delay(10);
  waitAUX();
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, E220_RX, E220_TX);

  pinMode(E220_M0, OUTPUT);
  pinMode(E220_M1, OUTPUT);
  pinMode(E220_AUX, INPUT);

  delay(1000);
  Serial.println("\n--- E220 Settings Reading Start ---");

  // 1. 設定モードへ
  setMode(3);

  // 2. 読み出しコマンド送信 (C1 + 開始アドレス00 + 長さ09)
  uint8_t read_cmd[] = {0xC1, 0x00, 0x09};
  Serial1.write(read_cmd, sizeof(read_cmd));

  // 3. レスポンスの受信と表示
  waitAUX();
  Serial.print("Current Config: ");
  
  unsigned long startTime = millis();
  while (millis() - startTime < 1000) { // 1秒間タイムアウト待ち
    if (Serial1.available()) {
      Serial.printf("%02X ", Serial1.read());
    }
  }
  
  Serial.println("\n--- Reading Finished ---");

  // 4. 通常モードに戻す
  setMode(0);
}

void loop() {
  // 読み出し専用のためloop内は空
}