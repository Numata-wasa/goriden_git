/***************************************************************************
 * This sketch reads data from a GPS module via Hardware Serial
 * and logs it to an SD card in CSV format every second, regardless of
 * whether the GPS location has been updated.
 ***************************************************************************/
//エラーランプの点灯の設定
const int error_ledpin = 41; //エラーランプ
const int stat_ledpin = 42; //STATランプ（GPSに合わせて点灯させたい）



// 共通ライブラリ
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Wire.h> // I2C通信に必要
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <QMC5883LCompass.h>


// ===== GPS =====
static const int RXPin = 7, TXPin = 6; // GPSのTXをESP32のRX(7)に、RXをTX(6)に接続
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial ss(1); // ハードウェアシリアルの1番を使用
// ===============

// ===== SD =====
#define SD_CS_PIN      2  // チップセレクトピン
#define SPI_MOSI_PIN   4  // MOSIピン
#define SPI_MISO_PIN   3  // MISOピン
#define SPI_SCK_PIN    1  // SCKピン

SPIClass spi;
const char* fileName = "/fulldata.csv";
File dataFile;
// ===============

// ===== BME =====
#define SEALEVELPRESSURE_HPA (1011.4) //試射前に海面（川）の気圧確認
// I2Cピンの定義
#define I2C_SDA 17
#define I2C_SCL 16

Adafruit_BME280 bme; // I2C
// ==============

// ★追加: 時間管理用の変数
unsigned long lastLogTime = 0;
const long logInterval = 50; // ログを記録する間隔 (ミリ秒) 今は２０Hz 
//================================================================
// BMI088 I2C Addresses
// SDOピンのGND/VCC接続によってアドレスが変わる場合があります
//================================================================
#define ACC_ADDRESS 0x19  // 加速度センサーのI2Cアドレス (SDO=VCC)
#define GYRO_ADDRESS 0x69 // ジャイロセンサーのI2Cアドレス (SDO=VCC)

//================================================================
// BMI088 Register Addresses (抜粋)
//================================================================
// --- Accelerometer Registers ---
#define ACC_CHIP_ID      0x00 // デバイスID (常に0x1E)
#define ACC_DATA_START   0x12 // 加速度データ開始アドレス
#define ACC_CONF         0x40 // ODR, BWP設定
#define ACC_RANGE        0x41 // 測定レンジ設定
#define ACC_PWR_CONF     0x7C // 電源設定
#define ACC_PWR_CTRL     0x7D // センサー有効化
#define ACC_SOFTRESET    0x7E // ソフトリセット

// --- Gyroscope Registers ---
#define GYRO_CHIP_ID     0x00 // デバイスID (常に0x0F)
#define GYRO_DATA_START  0x02 // ジャイロデータ開始アドレス
#define GYRO_RANGE       0x0F // 測定レンジ設定
#define GYRO_BANDWIDTH   0x10 // ODR, Filter Bandwidth設定
#define GYRO_LPM1        0x11 // 電源モード設定
#define GYRO_SOFTRESET   0x14 // ソフトリセット

//================================================================
// 物理量変換のためのスケールファクタ
//================================================================
// 加速度の測定レンジ (±3g) に対応
// LSB Sensitivity: 2^15 / 3g = 32768 / 3
const float ACC_SCALE = 3.0f / 32768.0f; 
const float G_TO_MS2 = 9.80665f; // 重力加速度

// ジャイロの測定レンジ (±2000 dps) に対応
// LSB Sensitivity: 2^15 / 2000dps = 32768 / 2000
const float GYRO_SCALE = 2000.0f / 32768.0f;

// センサーデータを格納する構造体
struct SensorData {
  float ax, ay, az; // Accelerometer data [m/s^2]
  float gx, gy, gz; // Gyroscope data [dps]
};

int cx = 0;
int cy = 0;
int cz = 0;


SensorData sensorData;

QMC5883LCompass compass;

//================================================================
// I2C Register Read/Write Helper Functions
//================================================================
void writeRegister(uint8_t i2c_addr, uint8_t reg_addr, uint8_t value) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.write(value);
  Wire.endTransmission();
}

void readRegisters(uint8_t i2c_addr, uint8_t reg_addr, uint8_t* buffer, uint8_t len) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.endTransmission(false); // falseで接続を維持
  Wire.requestFrom(i2c_addr, len);
  for (uint8_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }
}


void bmisetup(){
  Wire.setClock(400000); // I2Cクロックを400kHzに設定

  //---- Accelerometer Initialization ----
  Serial.println("Initializing Accelerometer...");
  // 1. ソフトリセット
  writeRegister(ACC_ADDRESS, ACC_SOFTRESET, 0xB6);
  delay(100);
  
  // 2. センサーを有効化
  writeRegister(ACC_ADDRESS, ACC_PWR_CTRL, 0x04);
  delay(50);

  // 3. 測定レンジを±3gに設定
  writeRegister(ACC_ADDRESS, ACC_RANGE, 0x02); // 0x00: ±3g, 0x01: ±6g, 0x02: ±12g, 0x03: ±24g

  // 4. ODRを100Hz, Normal BWPに設定
  writeRegister(ACC_ADDRESS, ACC_CONF, 0xA8); // ODR=100Hz, BWP=Normal

  // 5. 電源モードをActiveに設定
  writeRegister(ACC_ADDRESS, ACC_PWR_CONF, 0x00);
  delay(10);
  Serial.println("Accelerometer Initialized.");

  //---- Gyroscope Initialization ----
  Serial.println("Initializing Gyroscope...");
  // 1. ソフトリセット
  writeRegister(GYRO_ADDRESS, GYRO_SOFTRESET, 0xB6);
  delay(100);

  // 2. 測定レンジを±2000dpsに設定
  writeRegister(GYRO_ADDRESS, GYRO_RANGE, 0x00); // 0x00: ±2000dps, 0x01: ±1000dps, ...

  // 3. ODRを100Hz, Bandwidthを12Hzに設定
  writeRegister(GYRO_ADDRESS, GYRO_BANDWIDTH, 0x88); // ODR=100Hz, BW=12Hz

  // 4. 電源モードをNormalに設定
  writeRegister(GYRO_ADDRESS, GYRO_LPM1, 0x00);
  delay(50);
  Serial.println("Gyroscope Initialized.");
  Serial.println("----------------------------------------");


}

void bmieget(){
  uint8_t buffer[6];

  //---- Read Accelerometer Data ----
  readRegisters(ACC_ADDRESS, ACC_DATA_START, buffer, 6);
  // 16-bitの生データに結合 (リトルエンディアン)
  int16_t raw_ax = (int16_t)((buffer[1] << 8) | buffer[0]);
  int16_t raw_ay = (int16_t)((buffer[3] << 8) | buffer[2]);
  int16_t raw_az = (int16_t)((buffer[5] << 8) | buffer[4]);
  // 物理量に変換
  sensorData.ax = raw_ax * ACC_SCALE * G_TO_MS2;
  sensorData.ay = raw_ay * ACC_SCALE * G_TO_MS2;
  sensorData.az = raw_az * ACC_SCALE * G_TO_MS2;

  //---- Read Gyroscope Data ----
  readRegisters(GYRO_ADDRESS, GYRO_DATA_START, buffer, 6);
  // 16-bitの生データに結合 (リトルエンディアン)
  int16_t raw_gx = (int16_t)((buffer[1] << 8) | buffer[0]);
  int16_t raw_gy = (int16_t)((buffer[3] << 8) | buffer[2]);
  int16_t raw_gz = (int16_t)((buffer[5] << 8) | buffer[4]);
  // 物理量に変換
  sensorData.gx = raw_gx * GYRO_SCALE;
  sensorData.gy = raw_gy * GYRO_SCALE;
  sensorData.gz = raw_gz * GYRO_SCALE;

}

void qmcsetup(){

    compass.init();
    Serial.println("QMC initialized");

}

void qmcget(){
    compass.read();

    cx = compass.getX();
    cy = compass.getY();
    cz = compass.getZ();

}

void setup() {
    pinMode(error_ledpin, OUTPUT ); //ERRORのLEDを
    pinMode(stat_ledpin, OUTPUT );  //STAT

    // シリアルモニターを開始
    Serial.begin(115200);
    //while (!Serial); //シリアルモニタが開くまで待つのはコメントアウトしてます
    Serial.println("\nGPS Data Logger to SD Card (1-second interval)"); //面倒なのでそのまま

    // SPIバスを初期化
    spi.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);

    //I2C開始
    Wire.begin(I2C_SDA, I2C_SCL);

    bool status = bme.begin(0x76); //addres

    bmisetup();
    qmcsetup();

    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring or I2C address!");
        digitalWrite(error_ledpin, HIGH);
        while (1) delay(10);
    }
    // GPS用のシリアルポートを初期化
    ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

    // SDカードを初期化
    Serial.println("Initializing SD card...");
    if (!SD.begin(SD_CS_PIN, spi)) {
        Serial.println("Card failed, or not present. Halting.");
        digitalWrite(error_ledpin, HIGH);
        while (1); // 処理を停止
    }
    Serial.println("SD card initialized.");

    // CSVヘッダーを書き込み
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
        dataFile.println("Date,Time,Lat,Lng,Sat,Alt,Temp,Pres,PrAl,Humi,ax,ay,az,gx,gy,gz,cx,cy,cz");
        dataFile.close();
        Serial.print("Header written to ");
        Serial.println(fileName);
    } else {
        Serial.print("Error opening ");
        digitalWrite(error_ledpin, HIGH);
        Serial.println(fileName);
    }
    Serial.println();

    qmcsetup();


}

void loop() {
    // GPSモジュールから受信したデータをTinyGPS++に渡す
    while (ss.available() > 0) {
        gps.encode(ss.read());
    }

    //GPSが受信できていたらSTATLEDが点灯
    if (gps.location.isUpdated()){
        digitalWrite(stat_ledpin, HIGH);
    } else {
        digitalWrite(stat_ledpin, LOW);
    }


    // ★修正: 前回のログ記録から指定した時間が経過したかチェック
    if (millis() - lastLogTime >= logInterval) {
        // 最後にログを記録した時間を更新
        lastLogTime = millis();

        String dataString = "";

        // --- 日付 ---
        if (gps.date.isValid()) {
            char dateBuffer[11];
            snprintf(dateBuffer, sizeof(dateBuffer), "%04d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());
            dataString += String(dateBuffer);
        } else {
            dataString += "N/A"; // データが無効な場合
        }
        dataString += ",";

        // --- 時刻 ---
        if (gps.time.isValid()) {
            char timeBuffer[9];
            snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
            dataString += String(timeBuffer);
        } else {
            dataString += "N/A"; // データが無効な場合
        }
        dataString += ",";
        
        // --- 緯度 ---
        if (gps.location.isValid()) {
            dataString += String(gps.location.lat(), 6);
        } else {
            dataString += "0.000000"; // データが無効な場合
        }
        dataString += ",";

        // --- 経度 ---
        if (gps.location.isValid()) {
            dataString += String(gps.location.lng(), 6);
        } else {
            dataString += "0.000000"; // データが無効な場合
        }
        dataString += ",";

        // --- 衛星の数 ---
        if (gps.satellites.isValid()) {
            dataString += String(gps.satellites.value());
        } else {
            dataString += "0"; // データが無効な場合
        }
        dataString += ",";

        // --- 高度 ---
        if (gps.altitude.isValid()) {
            dataString += String(gps.altitude.meters());
        } else {
            dataString += "0.0"; // データが無効な場合
        }
        dataString += ",";
        // ==================================

        // ============= BME ================
        dataString += String(bme.readTemperature());
        dataString += ",";
        dataString += String(bme.readPressure() / 100.0F);
        dataString += ",";
        dataString += String(bme.readAltitude(SEALEVELPRESSURE_HPA));
        dataString += ",";
        dataString += String(bme.readHumidity());
        dataString += ",";
        // ==================================

        // =========== BMI ==================
        bmieget();

        dataString += String(sensorData.ax, 4);
        dataString += ",";
        dataString += String(sensorData.ay, 4);
        dataString += ",";
        dataString += String(sensorData.az, 4);
        dataString += ",";
        dataString += String(sensorData.gx, 4);
        dataString += ",";
        dataString += String(sensorData.gy, 4);
        dataString += ",";
        dataString += String(sensorData.gz, 4);
        dataString += ",";

        // =========== QMC ==================
        qmcget();

        dataString += String(cx);
        dataString += ",";
        dataString += String(cy);
        dataString += ",";
        dataString += String(cz);
        // ==================================



        // データをシリアルモニタに表示
        // 本番（直前）に消す！！
        //Serial.println("--- Logging data (Interval) ---");
        //Serial.println(dataString);

        // データをSDカードに追記
        dataFile = SD.open(fileName, FILE_APPEND);
        if (dataFile) {
            dataFile.println(dataString);
            dataFile.close();
            //Serial.println("Data saved to SD card.");
        } else {
            Serial.print("Error opening ");
            digitalWrite(error_ledpin, HIGH);
            Serial.println(fileName);
        }
        //Serial.println("-------------------------");
    }
}