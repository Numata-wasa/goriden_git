/*
 * 受信側 Pico (Rx)
 * TWELITE UART App (必ず書式モード m=0x01 に設定されていること)
 */

void setup() {

  Serial.begin(115200);  // PC用
  delay(100);
  Serial.println("ready...");


  Serial2.setTX(4);
  Serial2.setRX(5);
  Serial2.begin(115200); // TWELITE用

  delay(1000);
  Serial.println("start");

}

void loop() {
  // TWELITEからデータが来たらPCへ流す
  if (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c);
    //Serial.println(c);
  }
  }
}