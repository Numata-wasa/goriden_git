/*
 * Rocket Telemetry Dummy Generator for TWELITE
 * Board: Raspberry Pi Pico
 * Interface: Serial2 (UART1)
 * Pins: TX = GP4, RX = GP5
 * Baud Rate: 115200
 */

// ピン定義
const int TX_PIN = 4;
const int RX_PIN = 5;

// 送信間隔 (ms) - Python側のtime.sleep(0.1)に合わせる
const int INTERVAL_MS = 100;

unsigned long previousMillis = 0;

void setup() {
  pinMode(14, OUTPUT);
  // USBシリアル (デバッグ用)
  Serial.begin(115200);

  // TWELITE用シリアル (Serial2を使用)
  // Earle Philhower版コアではsetTX/setRXでピンを割り当て可能
  Serial2.setTX(TX_PIN);
  Serial2.setRX(RX_PIN);
  Serial2.begin(115200); // Python側のBAUD_RATEに合わせる

  // 乱数の初期化 (未接続のアナログピンのノイズをシードにする)
  randomSeed(analogRead(26));
  digitalWrite(14, HIGH);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= INTERVAL_MS) {
    previousMillis = currentMillis;

    // --- ダミーデータの生成 (Pythonコードの挙動を模倣) ---
    int time = millis();
    // 1. GPS (東京周辺)
    // random(min, max) は整数を返すため、100000倍して割ることで小数を生成
    float lat = 35.6895 + (random(-100, 100) / 100000.0);
    float lon = 139.6917 + (random(-100, 100) / 100000.0);

    // 2. 環境データ (気圧、温度)
    // 気圧は少しずつ変化させる（上昇している風に）
    float press = 1013.25 - (random(0, 5000) / 100.0); 
    float temp = 25.0 + (random(-100, 100) / 100.0);

    // 3. 9軸センサ (加速度, ジャイロ, 地磁気)
    float ax = random(-200, 200) / 100.0;
    float ay = random(-200, 200) / 100.0;
    float az = random(-200, 200) / 100.0;

    float gx = random(-10000, 10000) / 100.0;
    float gy = random(-10000, 10000) / 100.0;
    float gz = random(-10000, 10000) / 100.0;

    float mx = random(-5000, 5000) / 100.0;
    float my = random(-5000, 5000) / 100.0;
    float mz = random(-5000, 5000) / 100.0;

    // --- CSVデータの作成 ---
    // フォーマット: lat,lon,press,temp,ax,ay,az,gx,gy,gz,mx,my,mz
    String dataString = "";
    dataString += String(time) + ",";
    dataString += String(lat, 6) + ",";
    dataString += String(lon, 6) + ",";
    dataString += String(press, 2) + ",";
    dataString += String(temp, 2) + ",";
    dataString += String(ax, 2) + "," + String(ay, 2) + "," + String(az, 2) + ",";
    dataString += String(gx, 2) + "," + String(gy, 2) + "," + String(gz, 2) + ",";
    dataString += String(mx, 2) + "," + String(my, 2) + "," + String(mz, 2);
    
    // 改行コードを追加して送信
    Serial2.println(dataString);
    
    // デバッグ用にUSBシリアルにも同じ内容を表示
    Serial.println("Sent: " + dataString);
  }
}