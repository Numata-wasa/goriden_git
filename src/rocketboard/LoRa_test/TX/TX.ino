#include <Arduino.h>
// 受信（TX)
// Mbedコアでは Serial1 (GP0, GP1) を使用します
HardwareSerial& LoRaSerial = Serial1;

void setup() {
  // PCとの通信用 (シリアルモニタ)
  Serial.begin(115200);
  while (!Serial);

  // LoRaモジュール (E220) との通信用
  // ボーレートは送信側と合わせる (デフォルト9600)
  LoRaSerial.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("E220 Receiver (Pico Mbed Core - Serial1)");
}

void loop() {
  // LoRaモジュールからデータが届いているか確認
  if (LoRaSerial.available() > 0) {
    // データを受信し、文字列として読み込む (改行まで)
    String message = LoRaSerial.readStringUntil('\n');
    message.trim(); // 前後の空白や改行コードを削除

    if (message.length() > 0) {
      // シリアルモニタに表示
      Serial.print("Received: ");
      Serial.println(message);
      
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}