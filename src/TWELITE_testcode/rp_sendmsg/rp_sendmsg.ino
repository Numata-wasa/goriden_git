/*
 * 送信側 Pico (Tx) : ASCIIコマンド送信対応版
 * TWELITE設定: m=A, i=1
 */

unsigned long lastTime = 0;
// ダミーデータ
float dummyLat = 35.6895;
float dummyLon = 139.6917;
float dummyPress = 1013.25;

void setup() {
  Serial2.setTX(4);
  Serial2.setRX(5);
  Serial2.begin(115200); // TWELITEとの通信 (Baudrate確認!)
  Serial.begin(115200);  // PCデバッグ用
  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);
}

/* * TWELITEにデータを送るための関数
 * アスキー形式 (:7800[Payload][Checksum]) を生成して送信します
 */
void sendTweliteData(String msg) {
  // 1. コマンドと宛先
  // 0x78: 任意の相手に送るコマンド
  // 0x00: 宛先ID (親機 = 0)
  String cmd = "7800"; 

  // 2. データ部分をHEX文字列に変換
  // 例: "ABC" -> "414243"
  for (int i = 0; i < msg.length(); i++) {
    char c = msg.charAt(i);
    // 1桁のHEXの場合、頭に0をつける (例: A -> 0A)
    if (c < 16) cmd += "0";
    cmd += String(c, HEX);
  }

  // 3. チェックサム計算 (TWELITE仕様)
  // 全バイトを足して、2の補数をとる
  unsigned long sum = 0;
  for (int i = 0; i < cmd.length(); i += 2) {
    String byteStr = cmd.substring(i, i + 2);
    sum += strtoul(byteStr.c_str(), NULL, 16);
  }
  
  unsigned char checksum = (unsigned char)((~sum + 1) & 0xFF);
  String checksumStr = String(checksum, HEX);
  if (checksumStr.length() < 2) checksumStr = "0" + checksumStr; // ゼロ埋め
  
  // 4. 全部つなげて送信 (: + コマンド + チェックサム + 改行)
  cmd.toUpperCase();
  checksumStr.toUpperCase();
  String packet = ":" + cmd + checksumStr;
  
  Serial2.print(packet + "\r\n"); // ★最後に必ず改行コードを送る
  
  // デバッグ表示
  Serial.println("Sent: " + packet);
}

void loop() {
  if (millis() - lastTime > 200) { // 200ms間隔
    lastTime = millis();

    // データの更新
    dummyLat += (random(-10, 11) * 0.0001);
    dummyLon += (random(-10, 11) * 0.0001);
    dummyPress += (random(-10, 11) * 0.1);

    // 送信データの作成
    String payload = String(millis()) + "," + String(dummyLat, 6) + "," + String(dummyLon, 6) + "," + String(dummyPress, 2);

    // コマンド生成関数を使って送信
    sendTweliteData(payload);
  }
}