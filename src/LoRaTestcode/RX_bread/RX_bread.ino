#include <Arduino.h>
// RX
// --- ★ E220に接続するピンを指定 ★ ---
#define LORA_RX_PIN 7  // ESP32のRXピン (E220のTXDに接続)
#define LORA_TX_PIN 6  // ESP32のTXピン (E220のRXDに接続)
#define LORA_M0_PIN 40  // E220のM0ピンに接続
#define LORA_M1_PIN 39  // E220のM1ピンに接続

// ESP32のUARTポートを選択 (UART 0 は PCシリアル(Serial) で使用)
// UART 1 (Serial1) または UART 2 (Serial2) を使用します
HardwareSerial& LoRaSerial = Serial1; // UART 1 を LoRa用に使用

void setup() {
  // PCとの通信用 (シリアルモニタ)
  Serial.begin(115200);
  while (!Serial); // シリアルモニタが開くまで待機

  // E220モジュールを通常モード (M0=LOW, M1=LOW) に設定
  pinMode(LORA_M0_PIN, OUTPUT);
  pinMode(LORA_M1_PIN, OUTPUT);
  digitalWrite(LORA_M0_PIN, LOW);
  digitalWrite(LORA_M1_PIN, LOW);

  // LoRaモジュール (E220) との通信用
  // LoRaSerial.begin(ボーレート, プロトコル, RXピン, TXピン);
  // ボーレートは送信側と合わせる (115200)
  LoRaSerial.begin(115200, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  
  
  Serial.println("E220 Receiver (ESP32 - Serial1)");
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
      

    }
  }
}