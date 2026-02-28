#include <Arduino.h>

// ===================================================
// E220 LoRa モジュール送信機 (TX)
// ===================================================

// 関数の前方宣言
void setE220Mode(uint8_t mode);
void waitAUX();
void readE220Settings();
void writeE220Settings();
void sendLoRaData(uint8_t* data, uint8_t length);

// E220 ピン定義 (TX基板)
#define E220_RX 39
#define E220_TX 40
#define E220_M0 43
#define E220_M1 44
#define E220_AUX 38

// STAT LED
#define STAT_LED 42

// E220 設定
// ※ コンフィグモード(M0=1,M1=1)では常に9600bps固定
// ※ ノーマルモードではモジュールのSPED設定に従う
#define E220_CONFIG_BAUD 9600
#define E220_NORMAL_BAUD 9600

// E220 動作モード定義
#define E220_MODE_NORMAL 0      // M0=0, M1=0
#define E220_MODE_WAKEUP 1      // M0=1, M1=0
#define E220_MODE_POWERDOWN 2   // M0=0, M1=1
#define E220_MODE_CONFIG 3      // M0=1, M1=1

// 送信間隔（ミリ秒）
#define SEND_INTERVAL 3000

unsigned long lastSendTime = 0;
uint8_t messageCounter = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n========================================");
  Serial.println("  E220 LoRa Transmitter (TX)");
  Serial.println("========================================");

  // ピン初期化
  pinMode(STAT_LED, OUTPUT);
  digitalWrite(STAT_LED, LOW);
  pinMode(E220_M0, OUTPUT);
  pinMode(E220_M1, OUTPUT);
  pinMode(E220_AUX, INPUT);

  // ピン情報表示
  Serial.println("[PIN] RX=" + String(E220_RX) + " TX=" + String(E220_TX) +
                 " M0=" + String(E220_M0) + " M1=" + String(E220_M1) +
                 " AUX=" + String(E220_AUX));

  // ============================================
  // STEP 1: コンフィグモードで設定読み込み
  // コンフィグモードは常に9600bps
  // ============================================
  Serial.println("\n[STEP 1] Config mode (9600 bps)...");
  setE220Mode(E220_MODE_CONFIG);
  Serial1.begin(E220_CONFIG_BAUD, SERIAL_8N1, E220_RX, E220_TX);
  delay(500);
  waitAUX();

  Serial.println("\n[STEP 1a] Reading current settings...");
  readE220Settings();
  delay(200);

  Serial.println("\n[STEP 1b] Writing unified settings...");
  writeE220Settings();
  delay(200);

  Serial.println("\n[STEP 1c] Verifying settings...");
  readE220Settings();
  delay(200);

  // ============================================
  // STEP 2: ノーマルモードに切り替え
  // ============================================
  Serial.println("\n[STEP 2] Normal mode (" + String(E220_NORMAL_BAUD) + " bps)...");
  
  // ボーレート変更が必要な場合
  if (E220_NORMAL_BAUD != E220_CONFIG_BAUD) {
    Serial1.end();
    Serial1.begin(E220_NORMAL_BAUD, SERIAL_8N1, E220_RX, E220_TX);
    delay(100);
  }
  
  // バッファクリア（コンフィグモードの残留データ除去）
  while (Serial1.available()) Serial1.read();
  delay(50);

  setE220Mode(E220_MODE_NORMAL);
  waitAUX();

  // ノーマルモード切替後もバッファクリア
  delay(100);
  while (Serial1.available()) Serial1.read();

  Serial.println("\n========================================");
  Serial.println("  Ready to transmit! Interval: " + String(SEND_INTERVAL) + " ms");
  Serial.println("========================================\n");
  
  lastSendTime = millis();
}

void loop() {
  if (millis() - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = millis();
    
    // 識別しやすいテストデータ
    // マーカー: AA 55 + カウンタ + "TEST"
    uint8_t testData[16];
    int len = 0;
    
    testData[len++] = 0xAA;  // マーカー1
    testData[len++] = 0x55;  // マーカー2
    testData[len++] = messageCounter++;
    
    // テキスト "TEST"
    const char* msg = "TEST";
    for (int i = 0; msg[i] != '\0'; i++) {
      testData[len++] = (uint8_t)msg[i];
    }
    
    sendLoRaData(testData, len);
  }

  delay(100);
}

// ============================================
// AUXピンがHIGHになるまで待機
// ============================================
void waitAUX() {
  unsigned long start = millis();
  while (digitalRead(E220_AUX) == LOW && millis() - start < 2000) {
    delay(10);
  }
  Serial.print("  AUX: ");
  Serial.print(digitalRead(E220_AUX) ? "HIGH" : "LOW");
  Serial.print(" (waited ");
  Serial.print(millis() - start);
  Serial.println(" ms)");
}

// ============================================
// E220 モード設定
// ============================================
void setE220Mode(uint8_t mode) {
  switch (mode) {
    case E220_MODE_NORMAL:
      digitalWrite(E220_M0, LOW);
      digitalWrite(E220_M1, LOW);
      Serial.println("  Mode -> Normal (M0=0, M1=0)");
      break;
    case E220_MODE_WAKEUP:
      digitalWrite(E220_M0, HIGH);
      digitalWrite(E220_M1, LOW);
      Serial.println("  Mode -> Wake-up (M0=1, M1=0)");
      break;
    case E220_MODE_POWERDOWN:
      digitalWrite(E220_M0, LOW);
      digitalWrite(E220_M1, HIGH);
      Serial.println("  Mode -> Power down (M0=0, M1=1)");
      break;
    case E220_MODE_CONFIG:
      digitalWrite(E220_M0, HIGH);
      digitalWrite(E220_M1, HIGH);
      Serial.println("  Mode -> Config (M0=1, M1=1)");
      break;
  }
  delay(200);
}

// ============================================
// E220 設定読み込み
// コマンド: C1 ADDR LEN (3バイト)
// 応答:    C1 ADDR LEN DATA[0..LEN-1]
// ============================================
void readE220Settings() {
  // バッファクリア
  while (Serial1.available()) Serial1.read();
  delay(50);

  // レジスタ0x00から8バイト読み込み (CRYPT含む)
  uint8_t cmd[3] = {0xC1, 0x00, 0x08};

  Serial.print("  CMD: ");
  for (int i = 0; i < 3; i++) {
    Serial1.write(cmd[i]);
    if (cmd[i] < 0x10) Serial.print("0");
    Serial.print(cmd[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // 応答受信: C1 00 08 + 8バイトデータ = 11バイト
  uint8_t resp[16];
  int cnt = 0;
  unsigned long timeout = millis() + 2000;
  while (millis() < timeout && cnt < 11) {
    if (Serial1.available()) {
      resp[cnt++] = Serial1.read();
    }
  }

  Serial.print("  Response (");
  Serial.print(cnt);
  Serial.print(" bytes): ");
  for (int i = 0; i < cnt; i++) {
    if (resp[i] < 0x10) Serial.print("0");
    Serial.print(resp[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // 応答解析
  if (cnt >= 11 && resp[0] == 0xC1) {
    // ヘッダ: resp[0]=C1, resp[1]=ADDR_H, resp[2]=LEN
    // データ: resp[3..10] = REG0~REG7
    uint8_t addh   = resp[3];
    uint8_t addl   = resp[4];
    uint8_t sped   = resp[5];
    uint8_t opt    = resp[6];
    uint8_t chan   = resp[7];
    uint8_t opt2   = resp[8];
    uint8_t cryptH = resp[9];
    uint8_t cryptL = resp[10];

    Serial.println("  --- Register Values ---");
    Serial.print("    ADDH=0x"); Serial.print(addh, HEX);
    Serial.print(" ADDL=0x"); Serial.println(addl, HEX);
    Serial.print("    SPED=0x"); Serial.println(sped, HEX);

    // SPED: [7:6]=Parity, [5:3]=Air Rate, [2:0]=UART Baud
    //  ※ ビットバンガー診断で確認済み
    Serial.print("      Parity: ");
    switch ((sped >> 6) & 0x03) {
      case 0: Serial.print("8N1"); break;
      case 1: Serial.print("8O1"); break;
      case 2: Serial.print("8E1"); break;
      case 3: Serial.print("8N1"); break;
    }
    Serial.print(", Uart: ");
    switch (sped & 0x07) {
      case 0: Serial.print("1200"); break;
      case 1: Serial.print("2400"); break;
      case 2: Serial.print("4800"); break;
      case 3: Serial.print("9600"); break;
      case 4: Serial.print("19200"); break;
      case 5: Serial.print("38400"); break;
      case 6: Serial.print("57600"); break;
      case 7: Serial.print("115200"); break;
    }
    Serial.print(" bps, Air: ");
    switch ((sped >> 3) & 0x07) {
      case 0: case 1: Serial.print("2.4k"); break;
      case 2: Serial.print("4.8k"); break;
      case 3: Serial.print("9.6k"); break;
      case 4: Serial.print("19.2k"); break;
      case 5: Serial.print("38.4k"); break;
      case 6: Serial.print("62.5k"); break;
      case 7: Serial.print("Reserved"); break;
    }
    Serial.println(" bps");

    Serial.print("    OPTION=0x"); Serial.println(opt, HEX);
    Serial.print("    CHAN=0x"); Serial.print(chan, HEX);
    Serial.print(" (Channel "); Serial.print(chan); Serial.println(")");
    Serial.print("    OPTION2=0x"); Serial.println(opt2, HEX);
    Serial.print("    CRYPT=0x");
    if (cryptH < 0x10) Serial.print("0"); Serial.print(cryptH, HEX);
    if (cryptL < 0x10) Serial.print("0"); Serial.println(cryptL, HEX);
  } else {
    Serial.println("  WARNING: Unexpected response!");
    Serial.println("  (E220 may not be connected or config mode not entered)");
  }
}

// ============================================
// E220 設定書き込み（統一設定）
// コマンド: C0 ADDR LEN DATA[0..LEN-1]
// 応答:    C1 ADDR LEN DATA[0..LEN-1]
// ============================================
void writeE220Settings() {
  // バッファクリア
  while (Serial1.available()) Serial1.read();
  delay(50);

  // 統一設定値
  // ADDH=0x00, ADDL=0x00
  // SPED=0x03: [7:6]=00(8N1), [5:3]=000(Air:2.4k), [2:0]=011(UART:9600)
  //  ※ ビットバンガー診断で[2:0]=UART baudを確認済み
  // OPTION=0x00
  // CHAN=0x00 (Channel 0)
  // OPTION2=0x00
  // CRYPT=0x0000 (暗号化なし)
  uint8_t cmd[11] = {
    0xC0, 0x00, 0x08,   // Write command: addr=0x00, len=8
    0x00, 0x00,         // ADDH, ADDL
    0x03,               // SPED (8N1, Air:2.4k, UART:9600bps)
    0x00,               // OPTION
    0x00,               // CHAN (Channel 0)
    0x00,               // OPTION2
    0x00, 0x00          // CRYPT_H, CRYPT_L
  };

  Serial.print("  CMD: ");
  for (int i = 0; i < 11; i++) {
    Serial1.write(cmd[i]);
    if (cmd[i] < 0x10) Serial.print("0");
    Serial.print(cmd[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // 応答受信: C1 00 08 + 8バイトデータ = 11バイト
  uint8_t resp[16];
  int cnt = 0;
  unsigned long timeout = millis() + 2000;
  while (millis() < timeout && cnt < 11) {
    if (Serial1.available()) {
      resp[cnt++] = Serial1.read();
    }
  }

  Serial.print("  Response (");
  Serial.print(cnt);
  Serial.print(" bytes): ");
  for (int i = 0; i < cnt; i++) {
    if (resp[i] < 0x10) Serial.print("0");
    Serial.print(resp[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  if (cnt >= 11 && resp[0] == 0xC1) {
    Serial.println("  Settings written successfully!");
  } else {
    Serial.println("  WARNING: Write may have failed!");
  }
}

// ============================================
// E220 データ送信
// ============================================
void sendLoRaData(uint8_t* data, uint8_t length) {
  // LED点灯
  digitalWrite(STAT_LED, HIGH);

  // AUX HIGH待ち（送信可能状態）
  unsigned long w = millis();
  while (digitalRead(E220_AUX) == LOW && millis() - w < 1000) {
    delay(1);
  }

  // 送信
  Serial1.write(data, length);

  Serial.print("[TX #");
  Serial.print(messageCounter);
  Serial.print("] ");
  Serial.print(length);
  Serial.print(" bytes: ");
  for (int i = 0; i < length; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" AUX=");
  Serial.println(digitalRead(E220_AUX) ? "HIGH" : "LOW");

  delay(200);
  digitalWrite(STAT_LED, LOW);
}