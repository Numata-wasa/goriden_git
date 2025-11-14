#include <Arduino.h>
// 送信（RX)
// Mbedコアでは Serial1 (GP0, GP1) を使用します
HardwareSerial& LoRaSerial = Serial1;

int counter = 0;

void setup() {
  // PCとの通信用 (シリアルモニタ)
  Serial.begin(115200);
  while (!Serial);

  // LoRaモジュール (E220) との通信用
  // E220のデフォルトボーレート 9600bps
  LoRaSerial.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("E220 Transmitter (Pico Mbed Core - Serial1)");
}

void loop() {
  // 送信するメッセージを作成
  String message = "Hello LoRa! Count: " + String(counter) + "\n";
  
  // LoRaモジュールにメッセージを送信
  LoRaSerial.print(message);
  
  Serial.print("Sent: ");
  Serial.print(message); 

  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);

  counter++;
  delay(5000); // 5秒待機
}