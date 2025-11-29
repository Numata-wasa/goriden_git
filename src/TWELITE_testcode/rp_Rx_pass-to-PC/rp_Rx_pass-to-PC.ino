/*
 * 受信側 Raspberry Pi Pico (Rx)
 * 役割: TWELITEからのデータをそのままPCへ転送する (パススルー)
 * * [配線確認]
 * Pico GP0 (TX)  ---> TWELITE RX
 * Pico GP1 (RX)  ---> TWELITE TX
 * Pico 3V3       ---> TWELITE VCC
 * Pico GND       ---> TWELITE GND
 */

void setup() {
  // PCとの通信 (高速でOK)
  Serial.begin(115200);
  
  // TWELITEとの通信
  // ※TWELITE側の設定(Baud Rate)と必ず一致させてください。
  // 前回の流れ通り、安定性のために 38400 にしています。
  Serial2.setTX(4);
  Serial2.setRX(5);
  Serial2.begin(115200); 

  // 起動確認用 (Pythonを接続する前にシリアルモニタで確認する場合用)
  // Python接続時はこれがノイズになる可能性がありますが、
  // 先ほどのPythonコードはヘッダ判別機能があるので問題ありません。
  delay(2000);
  // Serial.println("--- Rx Bridge Started ---"); 
}

void loop() {
  // TWELITEからデータが来ているかチェック
  if (Serial2.available()) {
    // 1バイト読み込む
    uint8_t c = Serial2.read();
    
    // そのままPCへバイナリとして書き出す
    // ※ Serial.print ではなく Serial.write を使うのが重要です
    // (バイナリデータが来た場合に文字化けさせないため)
    Serial.write(c);
  }
}