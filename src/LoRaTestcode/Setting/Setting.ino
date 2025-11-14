#include <Arduino.h>

// --- ★ E220に接続するピンを指定 ★ ---
#define LORA_RX_PIN 16   // ESP32のRXピン (E220のTXDに接続)
#define LORA_TX_PIN 17   // ESP32のTXピン (E220のRXDに接続)
#define LORA_M0_PIN 40  // E220のM0ピンに接続
#define LORA_M1_PIN 39  // E220のM1ピンに接続

// UART 1 (Serial1) を LoRa用に使用
HardwareSerial& LoRaSerial = Serial1;

// E220 設定コマンド (7バイト)
// C0 (保存) + アドレス(00 00) + REG0 + REG1 + REG2 + REG3
byte configCommand[] = {
  0xC0,       // C0: 設定を保存して適用
  0x00,       // ADDH: アドレス上位
  0x00,       // ADDL: アドレス下位
  
  // REG0: UART設定
  // 0x62 = 011 00 010
  // 011 = 115200bps
  // 00  = 8N1 (パリティなし)
  // 010 = Wake-up time 250ms
  0x62,
  
  // REG1: Air Data Rate設定
  // 0xEA = 111 01010
  // 111 = 62.5kbps (最速)
  // 01010 = (デフォルト値)
  0xEA,
  
  // REG2: チャンネル・電力設定
  // 0x07 = 000 00 111
  // 000 = (デフォルト値)
  // 00  = 10dBm (デフォルト電力)
  // 111 = CH 23 (873MHz) (デフォルトチャンネル)
  // ※注意: このチャンネルは日本の920MHz帯ではありません
  0x07,
  
  // REG3: オプション設定
  // 0x20 = 001 00 000
  // 0x20 = (デフォルト値: Pull-up有効)
  0x20
};

// 設定読み取りコマンド
byte readCommand[] = { 0xC1, 0xC1, 0xC1 };

void setup() {
  // PCとの通信用 (シリアルモニタ)
  Serial.begin(115200);
  while (!Serial);
  delay(1000);
  Serial.println("--- E220 設定変更スケッチ ---");

  // E220モジュールを「設定モード」に設定
  pinMode(LORA_M0_PIN, OUTPUT);
  pinMode(LORA_M1_PIN, OUTPUT);
  digitalWrite(LORA_M0_PIN, HIGH);
  digitalWrite(LORA_M1_PIN, HIGH);

  Serial.println("M0=HIGH, M1=HIGH に設定 (設定モード)");
  delay(500); // モジュールがモードを認識するまで待機

  // --- 1. 設定モードで通信 (9600bps固定) ---
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  Serial.println("LoRaシリアルを 9600bps (設定モード) で開始");

  // 設定コマンドを送信
  Serial.print("設定コマンド [ ");
  for (int i = 0; i < sizeof(configCommand); i++) {
    Serial.printf("0x%02X ", configCommand[i]);
  }
  Serial.println("] を送信します...");
  
  LoRaSerial.write(configCommand, sizeof(configCommand));
  
  delay(500); // モジュールが応答するまで待機

  // モジュールからの応答（書き込んだ設定内容）を読み取る
  if (LoRaSerial.available()) {
    Serial.println("モジュールから応答を受信しました (書き込まれた設定):");
    byte response[sizeof(configCommand)];
    LoRaSerial.readBytes(response, sizeof(configCommand));
    
    Serial.print("[ ");
    for (int i = 0; i < sizeof(response); i++) {
      Serial.printf("0x%02X ", response[i]);
    }
    Serial.println("]");
  } else {
    Serial.println("モジュールから応答がありません。");
  }

  // --- 2. 通常モードに戻す ---
  Serial.println("\nM0=LOW, M1=LOW に設定 (通常モード)");
  digitalWrite(LORA_M0_PIN, LOW);
  digitalWrite(LORA_M1_PIN, LOW);

  LoRaSerial.end(); // 9600bps を終了
  delay(500); // モード変更待ち

  // --- 3. 新しい設定 (115200bps) で通信を再開 ---
  LoRaSerial.begin(115200, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  Serial.println("LoRaシリアルを 115200bps (通常モード) で再開");
  Serial.println("\n--- 設定完了 ---");
  Serial.println("このまま5秒ごとに現在設定を読み取ります。");
}

void loop() {
  // 5秒ごとに、モジュールから現在の設定を読み取ってみる
  
  Serial.println("\n現在の設定を読み取ります (C1 C1 C1)...");
  LoRaSerial.write(readCommand, sizeof(readCommand));
  
  delay(500); // 応答待ち

  if (LoRaSerial.available()) {
    Serial.println("モジュールから設定データを読み取りました:");
    // 応答は C1 + 5バイトのパラメータ
    byte response[6]; 
    LoRaSerial.readBytes(response, 6);
    
    Serial.print("[ ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("0x%02X ", response[i]);
    }
    Serial.println("]");
    
    Serial.printf("  REG0 (UART): 0x%02X\n", response[3]);
    Serial.printf("  REG1 (Air Rate): 0x%02X (0xEAなら 62.5kbps)\n", response[4]);
    Serial.printf("  REG2 (CH/Power): 0x%02X\n", response[5]);
    
  } else {
    Serial.println("モジュールから応答がありません (通常モード 115200bps)");
  }
  
  delay(5000);
}