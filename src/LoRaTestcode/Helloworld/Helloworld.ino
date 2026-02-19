#include <Arduino.h>

// ピン定義 (前回のPCB仕様)
#define E220_RX  44
#define E220_TX  43
#define E220_M0  40
#define E220_M1  39
#define E220_AUX 38

unsigned long lastSendTime = 0;
const unsigned long interval = 1000; // 1000ms

// AUXピンの状態を待つ関数
void waitAUX() {
  while (digitalRead(E220_AUX) == LOW) {
    delay(1);
  }
  delay(2); // 送信開始前の微小な余裕
}

void setup() {
  Serial.begin(115200);
  
  // 設定済みの 9600bps で開始
  Serial1.begin(9600, SERIAL_8N1, E220_RX, E220_TX);

  pinMode(E220_M0, OUTPUT);
  pinMode(E220_M1, OUTPUT);
  pinMode(E220_AUX, INPUT);

  // 通常運用モード (M0=0, M1=0)
  digitalWrite(E220_M0, LOW);
  digitalWrite(E220_M1, LOW);

  Serial.println("E220 LoRa Sender Started.");
}

void loop() {
  unsigned long currentTime = millis();

  // 1000msごとに送信
  if (currentTime - lastSendTime >= interval) {
    lastSendTime = currentTime;

    waitAUX(); // モジュールがビジーでないか確認
    Serial1.println("HelloWorld"); // LoRaで送信
    
    Serial.println("Sent >> HelloWorld");
  }

  // 相手からデータを受信した場合の処理
  if (Serial1.available()) {
    String rx = Serial1.readStringUntil('\n');
    Serial.print("Received << ");
    Serial.println(rx);
  }
}