// ---------------------------------------------------
//     E220 送信側スケッチ (ESP32 - M0/M1 Control)
// ---------------------------------------------------
#define SENDER true // true=送信側

// --- ★ E220に接続するピンを指定 ★ ---
#define LORA_RX_PIN 43  // ESP32のRXピン (E220のTXDに接続)
#define LORA_TX_PIN 44  // ESP32のTXピン (E220のRXDに接続)
#define LORA_M0_PIN 40  // E220のM0ピンに接続
#define LORA_M1_PIN 39  // E220のM1ピンに接続
// ---------------------------------------------------

#include <Arduino.h>

HardwareSerial& LoRaSerial = Serial0; // ハードウェアシリアル2を使用
int counter = 0;

void setup() {
  // PCとの通信用
  Serial.begin(115200);
  while (!Serial);

  // M0/M1ピンを出力に設定
  pinMode(LORA_M0_PIN, OUTPUT);
  pinMode(LORA_M1_PIN, OUTPUT);
  
  // ★ 自動でモード0（通常モード）に設定 ★
  digitalWrite(LORA_M0_PIN, LOW);
  digitalWrite(LORA_M1_PIN, LOW);
  delay(100); // モード変更のための待機

  // LoRaモジュールとの通信を 9600bps で開始
  // (スケッチBで 9600bps に設定済みと仮定)
  LoRaSerial.begin(115200, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    
  Serial.println("E220 Transmitter (ESP32 - Mode 0)");
}

void loop() {
  // 送信側の処理
  String message = "Hello from ESP32! Count: " + String(counter) + "\n";
  
  // LoRaモジュール経由でメッセージを送信
  LoRaSerial.print(message);
  
  // PCのシリアルモニタに送信内容を表示
  Serial.print("Sent: ");
  Serial.print(message); 


  counter++;
  delay(5000); // 5秒待機
}