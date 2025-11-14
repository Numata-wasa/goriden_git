#include <Arduino.h>
// TX
// --- ★ E220に接続するピンを指定 ★ ---
#define LORA_RX_PIN 7  // ESP32のRXピン (E220のTXDに接続)
#define LORA_TX_PIN 6  // ESP32のTXピン (E220のRXDに接続)
#define LORA_M0_PIN 40  // E220のM0ピンに接続
#define LORA_M1_PIN 39  // E220のM1ピンに接続

// ESP32のUARTポートを選択 (UART 0 は PCシリアル(Serial) で使用)
// UART 1 (Serial1) または UART 2 (Serial2) を使用します
HardwareSerial& LoRaSerial = Serial1; // UART 1 を LoRa用に使用

int counter = 0;

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
  // (注意: 元のコードに基づき115200bpsに設定していますが、E220側も
  //  115200bpsに設定されている必要があります。デフォルトは9600bpsです)
  LoRaSerial.begin(115200, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  
  Serial.println("E220 Transmitter (ESP32 - Serial1)");
}

void loop() {
  // 送信するメッセージを作成
  String message = "Hello LoRa! Count: " + String(counter) + "\n";
  
  // LoRaモジュールにメッセージを送信
  LoRaSerial.print(message);
  
  // シリアルモニタに送信内容を表示
  Serial.print("Sent: ");
  Serial.print(message); 


  counter++;
  delay(1000); // 5秒待機
}