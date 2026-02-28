#include <Arduino.h>

// E220 LoRa モジュール受信機
// E220 ピン定義
#define E220_RX 39
#define E220_TX 38
#define E220_M0 12
#define E220_M1 11
#define E220_AUX 40

#define E220_BAUD 9600

// E220 動作モード定義
#define E220_MODE_NORMAL 0      // M0=0, M1=0: Normal mode (TX/RX)
#define E220_MODE_WAKEUP 1      // M0=1, M1=0: Wake-up mode
#define E220_MODE_POWERDOWN 2   // M0=0, M1=1: Power down mode
#define E220_MODE_CONFIG 3      // M0=1, M1=1: Configuration mode

struct LoRaMessage {
  uint8_t rssi;      // 信号強度
  uint8_t snr;       // SN比
  uint8_t len;       // データ長
  uint8_t data[240]; // ペイロードデータ
};

volatile LoRaMessage receivedMsg;
volatile bool dataReady = false;

void setup() {
  // デバッグ用シリアル初期化
  Serial.begin(115200);
  delay(1000);
  Serial.println("E220 LoRa Receiver starting...");

  // E220 制御ピン初期化
  pinMode(E220_M0, OUTPUT);
  pinMode(E220_M1, OUTPUT);
  pinMode(E220_AUX, INPUT);
  
  // E220をノーマルモード（受信可能）に設定
  setE220Mode(E220_MODE_NORMAL);
  
  // E220用シリアル初期化
  Serial1.begin(E220_BAUD, SERIAL_8N1, E220_RX, E220_TX);
  
  delay(500);
  Serial.println("E220 initialized in Normal mode");
}

void loop() {
  // E220からデータ受信処理
  if (Serial1.available()) {
    parseLoRaData();
  }

  // 受信完了時の処理
  if (dataReady) {
    dataReady = false;
    displayReceivedData();
  }

  delay(10);
}

// E220からのデータを解析
void parseLoRaData() {
  static uint8_t buffer[250];
  static int bufferIndex = 0;

  while (Serial1.available()) {
    uint8_t byte = Serial1.read();
    buffer[bufferIndex++] = byte;

    // E220は最初のバイトがヘッダ(通常0xC1など)
    // フレーム形式: [Header][RSSI][SNR][Len][Data...]
    if (bufferIndex >= 4) {
      uint8_t len = buffer[2]; // データ長
      if (bufferIndex >= (4 + len)) {
        // フレーム完成
        receivedMsg.rssi = buffer[0];
        receivedMsg.snr = buffer[1];
        receivedMsg.len = len;
        memcpy(receivedMsg.data, &buffer[3], len);
        
        bufferIndex = 0;
        dataReady = true;
        break;
      }
    }

    // バッファオーバーフロー対策
    if (bufferIndex >= sizeof(buffer)) {
      bufferIndex = 0;
    }
  }
}

// 受信データを表示
void displayReceivedData() {
  Serial.print(">>> LoRa Received: RSSI=");
  Serial.print((int8_t)receivedMsg.rssi);
  Serial.print(" dBm, SNR=");
  Serial.print((int8_t)receivedMsg.snr);
  Serial.print(" dB, Len=");
  Serial.print(receivedMsg.len);
  Serial.print(" Bytes: ");
  
  // データを16進数で表示
  for (int i = 0; i < receivedMsg.len; i++) {
    if (receivedMsg.data[i] < 0x10) Serial.print("0");
    Serial.print(receivedMsg.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// E220 モード設定関数
void setE220Mode(uint8_t mode) {
  switch (mode) {
    case E220_MODE_NORMAL:      // M0=0, M1=0
      digitalWrite(E220_M0, LOW);
      digitalWrite(E220_M1, LOW);
      Serial.println("E220: Normal mode (TX/RX)");
      break;
    case E220_MODE_WAKEUP:      // M0=1, M1=0
      digitalWrite(E220_M0, HIGH);
      digitalWrite(E220_M1, LOW);
      Serial.println("E220: Wake-up mode");
      break;
    case E220_MODE_POWERDOWN:   // M0=0, M1=1
      digitalWrite(E220_M0, LOW);
      digitalWrite(E220_M1, HIGH);
      Serial.println("E220: Power down mode");
      break;
    case E220_MODE_CONFIG:      // M0=1, M1=1
      digitalWrite(E220_M0, HIGH);
      digitalWrite(E220_M1, HIGH);
      Serial.println("E220: Configuration mode");
      break;
  }
  delay(100); // モード切り替え安定化
}