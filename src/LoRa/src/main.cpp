#include <Arduino.h>

// ===================================================
// E220 LoRa モジュール受信機 (RX)
// CRYPT不一致を修正 + REG0からbaud自動検出
// ===================================================

// 関数の前方宣言
void setE220Mode(uint8_t mode);
void waitAUX();
long readE220AndDetectBaud();
bool writeAllSettings(uint8_t cryptH, uint8_t cryptL);
void readE220Settings();

// E220 ピン定義 (RX基板)
#define E220_RX 39
#define E220_TX 38
#define E220_M0 12
#define E220_M1 11
#define E220_AUX 40

// STAT LED
#define STAT_LED 42

// E220 設定
#define E220_CONFIG_BAUD 9600
// ノーマルモードのボーレートはREG0から自動検出
long detectedBaud = 9600;

// E220 動作モード定義
#define E220_MODE_NORMAL 0
#define E220_MODE_WAKEUP 1
#define E220_MODE_POWERDOWN 2
#define E220_MODE_CONFIG 3

// LED制御
unsigned long ledOnTime = 0;
bool ledOn = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n========================================");
  Serial.println("  E220 LoRa Receiver (RX) v5");
  Serial.println("  CRYPT fix + auto baud detect");
  Serial.println("========================================");

  pinMode(STAT_LED, OUTPUT);
  digitalWrite(STAT_LED, LOW);
  pinMode(E220_M0, OUTPUT);
  pinMode(E220_M1, OUTPUT);
  pinMode(E220_AUX, INPUT);

  Serial.println("[PIN] RX=" + String(E220_RX) + " TX=" + String(E220_TX) +
                 " M0=" + String(E220_M0) + " M1=" + String(E220_M1) +
                 " AUX=" + String(E220_AUX));

  // ============================================
  // STEP 1: コンフィグモード (常に9600bps)
  // ============================================
  Serial.println("\n[STEP 1] Config mode (9600 bps)...");
  setE220Mode(E220_MODE_CONFIG);
  Serial1.begin(E220_CONFIG_BAUD, SERIAL_8N1, E220_RX, E220_TX);
  delay(500);
  waitAUX();

  // 設定読み込み + baud検出（書き込みは行わない）
  Serial.println("\n[STEP 1a] Reading settings & detecting baud...");
  detectedBaud = readE220AndDetectBaud();
  delay(200);
  // 注: 0x07-0x08のバイトはJP版では読み取り専用の可能性あり
  // 書き込み(C0 00 09, C0 07 02)は全て拒否されるため省略

  // ============================================
  // STEP 2: ノーマルモードへ切り替え
  // Serial1を検出ボーレートに変更
  // ============================================
  Serial.println("\n[STEP 2] Normal mode (" + String(detectedBaud) + " bps)...");

  Serial1.end();
  delay(50);
  Serial1.begin(detectedBaud, SERIAL_8N1, E220_RX, E220_TX);
  delay(100);

  while (Serial1.available()) Serial1.read();

  setE220Mode(E220_MODE_NORMAL);
  waitAUX();
  delay(100);
  while (Serial1.available()) Serial1.read();

  Serial.print("  AUX=");
  Serial.println(digitalRead(E220_AUX) ? "HIGH" : "LOW");

  Serial.println("\n========================================");
  Serial.println("  Ready to receive! Waiting for data...");
  Serial.println("  (Expecting: AA 55 [cnt] T E S T)");
  Serial.println("  Baud=" + String(detectedBaud) + " bps");
  Serial.println("========================================\n");
}

void loop() {
  if (digitalRead(E220_AUX) == LOW || Serial1.available()) {
    // 低速時は長めに待つ (1200bps: 1byte≈8.3ms)
    unsigned long waitMs = (detectedBaud <= 2400) ? 120 : 50;
    unsigned long gapMs  = (detectedBaud <= 2400) ? 80  : 30;
    delay(waitMs);

    uint8_t buf[64];
    int len = 0;
    unsigned long lastByte = millis();
    while (millis() - lastByte < gapMs && len < 64) {
      if (Serial1.available()) {
        buf[len++] = Serial1.read();
        lastByte = millis();
      }
    }

    if (len > 0) {
      digitalWrite(STAT_LED, HIGH);
      ledOnTime = millis();
      ledOn = true;

      Serial.print("[RX] ");
      Serial.print(len);
      Serial.print(" bytes: ");
      for (int i = 0; i < len; i++) {
        if (buf[i] < 0x10) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.print(" | ");
      for (int i = 0; i < len; i++) {
        Serial.print((buf[i] >= 0x20 && buf[i] <= 0x7E) ? (char)buf[i] : '.');
      }
      Serial.println();

      if (len >= 2 && buf[0] == 0xAA && buf[1] == 0x55) {
        Serial.print("  >>> VALID PACKET! Counter=");
        if (len >= 3) Serial.print(buf[2]);
        Serial.println(" <<<");
      }
    }
  }

  if (ledOn && millis() - ledOnTime >= 500) {
    digitalWrite(STAT_LED, LOW);
    ledOn = false;
  }
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
      digitalWrite(E220_M0, LOW);  digitalWrite(E220_M1, LOW);
      Serial.println("  Mode -> Normal (M0=0, M1=0)");
      break;
    case E220_MODE_WAKEUP:
      digitalWrite(E220_M0, HIGH); digitalWrite(E220_M1, LOW);
      Serial.println("  Mode -> Wake-up (M0=1, M1=0)");
      break;
    case E220_MODE_POWERDOWN:
      digitalWrite(E220_M0, LOW);  digitalWrite(E220_M1, HIGH);
      Serial.println("  Mode -> Power down (M0=0, M1=1)");
      break;
    case E220_MODE_CONFIG:
      digitalWrite(E220_M0, HIGH); digitalWrite(E220_M1, HIGH);
      Serial.println("  Mode -> Config (M0=1, M1=1)");
      break;
  }
  delay(200);
}

// ============================================
// E220 設定読み込み＋ボーレート自動検出
// REG0[7:5] の UART baud 設定を解析して返す
// コンフィグモード(9600bps)で呼ぶこと
// ============================================
long readE220AndDetectBaud() {
  while (Serial1.available()) Serial1.read();
  delay(50);

  uint8_t cmd[3] = {0xC1, 0x00, 0x09};
  Serial.print("  CMD: ");
  for (int i = 0; i < 3; i++) {
    if (cmd[i] < 0x10) Serial.print("0");
    Serial.print(cmd[i], HEX); Serial.print(" ");
  }
  Serial1.write(cmd, 3);
  Serial1.flush();
  Serial.println();

  uint8_t resp[16];
  int cnt = 0;
  unsigned long timeout = millis() + 2000;
  while (millis() < timeout && cnt < 12) {
    if (Serial1.available()) {
      resp[cnt++] = Serial1.read();
    }
  }

  Serial.print("  Response (");
  Serial.print(cnt);
  Serial.print(" bytes): ");
  for (int i = 0; i < cnt; i++) {
    if (resp[i] < 0x10) Serial.print("0");
    Serial.print(resp[i], HEX); Serial.print(" ");
  }
  Serial.println();

  long baud = 9600; // フォールバック

  if (cnt >= 12 && resp[0] == 0xC1) {
    uint8_t addh   = resp[3];
    uint8_t addl   = resp[4];
    uint8_t netid  = resp[5];
    uint8_t reg0   = resp[6];
    uint8_t reg1   = resp[7];
    uint8_t reg2   = resp[8];
    uint8_t reg3   = resp[9];
    uint8_t cryptH = resp[10];
    uint8_t cryptL = resp[11];

    Serial.println("  --- Register Values (JP) ---");
    Serial.print("    ADDH=0x"); Serial.print(addh, HEX);
    Serial.print(" ADDL=0x"); Serial.println(addl, HEX);
    Serial.print("    NETID=0x"); Serial.println(netid, HEX);
    Serial.print("    REG0=0x"); Serial.print(reg0, HEX);

    // REG0[7:5] → UART baud rate
    uint8_t uartBits = (reg0 >> 5) & 0x07;
    switch (uartBits) {
      case 0: baud = 1200;   break;
      case 1: baud = 2400;   break;
      case 2: baud = 4800;   break;
      case 3: baud = 9600;   break;
      case 4: baud = 19200;  break;
      case 5: baud = 38400;  break;
      case 6: baud = 57600;  break;
      case 7: baud = 115200; break;
    }
    Serial.print(" -> UART "); Serial.print(baud);
    Serial.println(" bps");

    Serial.print("    REG1=0x"); Serial.println(reg1, HEX);
    Serial.print("    REG2=0x"); Serial.print(reg2, HEX);
    Serial.print(" (Ch "); Serial.print(reg2); Serial.println(")");
    Serial.print("    REG3=0x"); Serial.println(reg3, HEX);
    Serial.print("    CRYPT=0x");
    if (cryptH < 0x10) Serial.print("0"); Serial.print(cryptH, HEX);
    if (cryptL < 0x10) Serial.print("0"); Serial.println(cryptL, HEX);

    if (cryptH != 0 || cryptL != 0) {
      Serial.println("    *** WARNING: CRYPT is non-zero! May cause RX/TX mismatch ***");
    }

    Serial.println("  => Detected normal-mode baud: " + String(baud));
  } else {
    Serial.println("  WARNING: Read failed! Using fallback 9600 bps");
  }

  return baud;
}

// ============================================
// E220 設定読み込み（確認用）
// ============================================
void readE220Settings() {
  while (Serial1.available()) Serial1.read();
  delay(50);

  uint8_t cmd[3] = {0xC1, 0x00, 0x09};
  Serial.print("  CMD: ");
  for (int i = 0; i < 3; i++) {
    if (cmd[i] < 0x10) Serial.print("0");
    Serial.print(cmd[i], HEX); Serial.print(" ");
  }
  Serial1.write(cmd, 3);
  Serial1.flush();
  Serial.println();

  uint8_t resp[16];
  int cnt = 0;
  unsigned long timeout = millis() + 2000;
  while (millis() < timeout && cnt < 12) {
    if (Serial1.available()) resp[cnt++] = Serial1.read();
  }

  Serial.print("  Response (");
  Serial.print(cnt);
  Serial.print(" bytes): ");
  for (int i = 0; i < cnt; i++) {
    if (resp[i] < 0x10) Serial.print("0");
    Serial.print(resp[i], HEX); Serial.print(" ");
  }
  Serial.println();

  if (cnt >= 12 && resp[0] == 0xC1) {
    Serial.print("    ADDH=0x"); Serial.print(resp[3], HEX);
    Serial.print(" ADDL=0x"); Serial.println(resp[4], HEX);
    Serial.print("    NETID=0x"); Serial.println(resp[5], HEX);
    Serial.print("    REG0=0x"); Serial.println(resp[6], HEX);
    Serial.print("    REG1=0x"); Serial.println(resp[7], HEX);
    Serial.print("    REG2=0x"); Serial.println(resp[8], HEX);
    Serial.print("    REG3=0x"); Serial.println(resp[9], HEX);
    Serial.print("    CRYPT=0x");
    if (resp[10] < 0x10) Serial.print("0"); Serial.print(resp[10], HEX);
    if (resp[11] < 0x10) Serial.print("0"); Serial.println(resp[11], HEX);
  }
}

// ============================================
// 全レジスタ書き込み (addr 0x00 から 9バイト)
// 6バイト書き込み(C0 00 06)は成功実績あり
// 9バイト(C0 00 09)でCRYPT含め一括書き込み
// コマンド: C0 00 09 ADDH ADDL NETID REG0 REG1 REG2 REG3 CRYPT_H CRYPT_L
// 応答:    C1 00 09 + 9バイト (成功時)
// ============================================
bool writeAllSettings(uint8_t cryptH, uint8_t cryptL) {
  while (Serial1.available()) Serial1.read();
  delay(50);

  // 現在の設定値を維持しつつ CRYPT だけ変更
  uint8_t cmd[12] = {
    0xC0, 0x00, 0x09,   // Write: addr=0x00, len=9
    0x00, 0x00,         // ADDH=0, ADDL=0
    0x03,               // NETID=0x03 (工場値維持)
    0x62,               // REG0: UART 9600 / Air 2.4k
    0x00,               // REG1: 13dBm
    0x18,               // REG2: Ch24 (920.6MHz)
    0x00,               // REG3: デフォルト
    cryptH, cryptL      // CRYPT
  };

  Serial.print("  CMD: ");
  for (int i = 0; i < 12; i++) {
    if (cmd[i] < 0x10) Serial.print("0");
    Serial.print(cmd[i], HEX); Serial.print(" ");
  }
  Serial.println();

  Serial1.write(cmd, 12);
  Serial1.flush();

  // 応答: C1 00 09 + 9バイト = 12バイト
  uint8_t resp[16];
  int cnt = 0;
  unsigned long timeout = millis() + 2000;
  while (millis() < timeout && cnt < 12) {
    if (Serial1.available()) resp[cnt++] = Serial1.read();
  }

  Serial.print("  Response (");
  Serial.print(cnt);
  Serial.print(" bytes): ");
  for (int i = 0; i < cnt; i++) {
    if (resp[i] < 0x10) Serial.print("0");
    Serial.print(resp[i], HEX); Serial.print(" ");
  }
  Serial.println();

  if (cnt >= 12 && resp[0] == 0xC1 && resp[1] == 0x00 && resp[2] == 0x09) {
    Serial.println("  All settings write SUCCESS!");
    return true;
  } else {
    Serial.println("  Write FAILED!");
    if (cnt >= 3 && resp[0] == 0xFF && resp[1] == 0xFF && resp[2] == 0xFF) {
      Serial.println("  (Module rejected the command)");
    }
    return false;
  }
}
