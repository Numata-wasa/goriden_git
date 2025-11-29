void setup() {
  // 1. PCとのデバッグ通信用 (USB)
  Serial.begin(115200);

  // 2. TWELITEとの通信設定 (UART1を使用)
  // Raspberry Pi PicoのUART1は、Arduinoでは「Serial2」として扱います。
  
  // ピンの割り当て (物理6番=GP4, 物理7番=GP5)
  Serial2.setTX(4);
  Serial2.setRX(5);
  
  // TWELITEのボーレートに合わせて開始 (通常115200)
  Serial2.begin(115200); 

  // 起動待機
  delay(2000);
  Serial.println("System Start");
}

void loop() {2
  // 送信する文字列
  String msg = "Hello TWELITE from Arduino\r\n";

  // TWELITEへ送信
  Serial2.print(msg);

  // PCのシリアルモニタにも表示（確認用）
  Serial.print("Sent: ");
  Serial.print(msg);

  delay(500);

  // 送信する文字列
  String msg2 = "Konnnichiwa watasiha oriza\r\n";

  // TWELITEへ送信
  Serial2.print(msg2);

  // PCのシリアルモニタにも表示（確認用）
  Serial.print("Sent: ");
  Serial.print(msg2);

  delay(500);
}