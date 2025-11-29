/* 送信側 Pico (Tx)
 * 設定: Mode=D (透過), ID=1, Baud=38400
 */
void setup() {
  Serial2.setTX(4);
  Serial2.setRX(5);
  Serial2.begin(115200); 
  Serial.begin(115200);
  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);
}

void loop() {
  // millis, lat, lon, press
  String payload = String(millis()) + ",35.68,139.76,1013";
  
  // 改行コード付きで送信
  Serial2.println(payload);
  
  // デバッグ
  Serial.println("Tx: " + payload);
  delay(100);
}