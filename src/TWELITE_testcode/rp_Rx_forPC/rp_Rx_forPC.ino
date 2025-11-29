/*
 * Raspberry Pi Pico Ground Station Firmware
 * * 機能:
 * 1. LoRaモジュール(UART)からテレメトリデータを受信
 * 2. 電波強度(RSSI)を取得 (またはモジュールが付与したものを読み取る)
 * 3. PC(USB Serial)へ転送
 * * 配線:
 * - LoRa RX -> Pico GP4 (UART1 TX)
 * - LoRa TX -> Pico GP5 (UART1 RX)
 */

#include <Arduino.h>

// ピン定義 (Raspberry Pi Pico)
// GP4, GP5は Hardware UART1 に対応しています
#define LORA_TX_PIN 4
#define LORA_RX_PIN 5

// シリアル設定
#define USB_BAUD_RATE  115200
#define LORA_BAUD_RATE 115200 // LoRaモジュールの設定に合わせて変更してください (例: 9600)

void setup() {
  // 1. PCとの通信 (USB)
  Serial.begin(USB_BAUD_RATE);
  
  // 2. LoRaモジュールとの通信 (UART1)
  // Earle Philhower版コアでは setTX/setRX でピンを指定できます
  Serial2.setTX(LORA_TX_PIN);
  Serial2.setRX(LORA_RX_PIN);
  Serial2.begin(LORA_BAUD_RATE);

  // 起動待機 (USB接続待ち) - 運用時はコメントアウトしても良い
  // while (!Serial) { delay(10); } 
}

void loop() {
  // LoRaモジュールからデータが来ているか確認
  if (Serial2.available() > 0) {
    
    // 改行コードまで読み込む
    // 想定データ: "millis,lat,lon,press,temp,ax,ay,az,gx,gy,gz,mx,my,mz"
    String receivedData = Serial2.readStringUntil('\n');
    
    // 空行やノイズを除去
    receivedData.trim();
    if (receivedData.length() == 0) return;

    // --- RSSI (電波強度) の処理 ---
    
    // パターンA: モジュールが自動でRSSIを末尾に付与しないタイプの場合
    // ここでGS側でRSSIを取得し、文字列に追加する必要があります。
    // ※多くの透明モード(Transparent)モジュールはRSSIを出しません。
    //   その場合、モジュール固有のコマンドでRSSIを読むか、
    //   Ebyte E32/E220などは設定で「パケットの最後にRSSIバイトを追加」できます。
    
    // とりあえずPythonコードを動かすために、ここでのRSSI取得をシミュレーションします
    // ★実運用では、使用するLoRaモジュールのライブラリを使って本当のRSSIを入れてください
    int currentRssi = getRssiFromModule(); 

    // --- PCへ転送 ---
    // フォーマット: "データ本体,RSSI"
    Serial.print(receivedData);
    Serial.print(",");
    Serial.println(currentRssi);
  }
}

// RSSIを取得する関数 (ご使用のモジュールに合わせて書き換えてください)
int getRssiFromModule() {
  // 例: Ebyte E220/E32などで「RSSI Byte Enable」にしている場合は、
  // Serial2の最後の1バイトがRSSIなので、それを読む処理が必要になります。
  
  // 仮の実装: ここではダミー値またはモジュールからの読み出し処理を書きます
  // モジュールからRSSIが取れない場合は、エラー値(-999)などを送ると良いでしょう
  return -65; // ダミー値
}