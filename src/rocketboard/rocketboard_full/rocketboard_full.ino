/***************************************************************************
 * 高速ロギング（200Hz）対応版
 * * 修正:
 * 1. SD.open() を setup() に移動し、ファイルを開きっぱなしにする
 * 2. 1秒ごとに dataFile.flush() を呼び出すタイマーを追加
 * 3. loop() 内の String を廃止し、char配列 (logBuffer) と snprintf() に変更
 * 4. GPS時刻の1/100秒取得を .cs() から .centisecond() に修正
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
File dataFile; // ★グローバル変数として保持
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
const long logInterval = 5; // ログを記録する間隔 (ミリ秒) 200Hz

// ★SDカードフラッシュ（物理書き込み）用タイマー
unsigned long lastFlushTime = 0;
const long flushInterval = 1000; // 1秒 (1000ms) に1回、バッファをSDに書き込む
//================================================================
// BMI088 I2C Addresses
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
const float ACC_SCALE = 3.0f / 32768.0f;
const float G_TO_MS2 = 9.80665f; // 重力加速度
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
  writeRegister(ACC_ADDRESS, ACC_SOFTRESET, 0xB6);
  delay(100);
  writeRegister(ACC_ADDRESS, ACC_PWR_CTRL, 0x04);
  delay(50);
  writeRegister(ACC_ADDRESS, ACC_RANGE, 0x02); // ±12g
  writeRegister(ACC_ADDRESS, ACC_CONF, 0xA8); // ODR=100Hz
  writeRegister(ACC_ADDRESS, ACC_PWR_CONF, 0x00);
  delay(10);
  Serial.println("Accelerometer Initialized.");

  //---- Gyroscope Initialization ----
  Serial.println("Initializing Gyroscope...");
  writeRegister(GYRO_ADDRESS, GYRO_SOFTRESET, 0xB6);
  delay(100);
  writeRegister(GYRO_ADDRESS, GYRO_RANGE, 0x00); // ±2000dps
  writeRegister(GYRO_ADDRESS, GYRO_BANDWIDTH, 0x88); // ODR=100Hz
  writeRegister(GYRO_ADDRESS, GYRO_LPM1, 0x00);
  delay(50);
  Serial.println("Gyroscope Initialized.");
  Serial.println("----------------------------------------");
}

void bmieget(){
  uint8_t buffer[6];

  //---- Read Accelerometer Data ----
  readRegisters(ACC_ADDRESS, ACC_DATA_START, buffer, 6);
  int16_t raw_ax = (int16_t)((buffer[1] << 8) | buffer[0]);
  int16_t raw_ay = (int16_t)((buffer[3] << 8) | buffer[2]);
  int16_t raw_az = (int16_t)((buffer[5] << 8) | buffer[4]);
  sensorData.ax = raw_ax * ACC_SCALE * G_TO_MS2;
  sensorData.ay = raw_ay * ACC_SCALE * G_TO_MS2;
  sensorData.az = raw_az * ACC_SCALE * G_TO_MS2;

  //---- Read Gyroscope Data ----
  readRegisters(GYRO_ADDRESS, GYRO_DATA_START, buffer, 6);
  int16_t raw_gx = (int16_t)((buffer[1] << 8) | buffer[0]);
  int16_t raw_gy = (int16_t)((buffer[3] << 8) | buffer[2]);
  int16_t raw_gz = (int16_t)((buffer[5] << 8) | buffer[4]);
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
    Serial.println("\nHigh-Speed GPS/IMU Data Logger");

    // SPIバスを初期化
    spi.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);

    //I2C開始
    Wire.begin(I2C_SDA, I2C_SCL);

    bool status = bme.begin(0x76); //address
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

    // ★★★ 変更点: ファイルを一度だけ開く ★★★
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
        // ヘッダーに millis を追加
        dataFile.println("Date,Time_cs,millis,Lat,Lng,Sat,Alt,Temp,Pres,PrAl,Humi,ax,ay,az,gx,gy,gz,cx,cy,cz");
        
        // ★★★ 変更点: ここで close() しない ★★★
        // dataFile.close(); 
        
        Serial.print("Header written to ");
        Serial.println(fileName);
    } else {
        Serial.print("Error opening ");
        digitalWrite(error_ledpin, HIGH);
        Serial.println(fileName);
        while (1); // 停止
    }
    Serial.println("Start logging");

    // qmcsetup(); // 2回呼び出されていたので1つコメントアウト
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

    unsigned long currentMillis = millis();

    // ★修正: メインの高速ロギングタイマー
    if (currentMillis - lastLogTime >= logInterval) {
        // 最後にログを記録した時間を更新
        lastLogTime = currentMillis;

        // ★★★ 変更点: String を廃止し、char 配列 (C言語文字列) を使用 ★★★
        char logBuffer[512]; // ログ一行を格納するバッファ。サイズは余裕を持って
        char tempBuffer[50]; // 浮動小数点数などを一時的に文字列に変換するため

        // --- センサーデータを先にすべて取得 ---
        // (BME)
        float temp = bme.readTemperature();
        float pres = bme.readPressure() / 100.0F;
        float alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
        float hum = bme.readHumidity();
        // (BMI)
        bmieget();
        // (QMC)
        qmcget();

        // --- GPS Date ---
        if (gps.date.isValid()) {
            snprintf(tempBuffer, sizeof(tempBuffer), "%04d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());
        } else {
            strcpy(tempBuffer, "N/A");
        }
        strcpy(logBuffer, tempBuffer); // バッファの先頭にコピー
        strcat(logBuffer, ","); // 連結

        // --- GPS Time (1/100秒まで記録する) ---
        if (gps.time.isValid()) {
            // ★★★★★★★ ここを .centisecond() に修正 ★★★★★★★
            snprintf(tempBuffer, sizeof(tempBuffer), "%02d:%02d:%02d.%02d", 
                     gps.time.hour(), 
                     gps.time.minute(), 
                     gps.time.second(),
                     gps.time.centisecond()); // ★修正済み★
        } else {
            strcpy(tempBuffer, "N/A");
        }
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");

        // --- Millis ---
        // ltoa (long to alpha) で数値を文字列に変換
        ltoa(currentMillis, tempBuffer, 10);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");

        // --- GPS Location ---
        dtostrf(gps.location.isValid() ? gps.location.lat() : 0.0, 4, 6, tempBuffer); // (値, 最小幅, 少数点以下桁数, 格納先)
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(gps.location.isValid() ? gps.location.lng() : 0.0, 4, 6, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");

        // --- GPS Satellites & Altitude ---
        ltoa(gps.satellites.isValid() ? gps.satellites.value() : 0, tempBuffer, 10);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(gps.altitude.isValid() ? gps.altitude.meters() : 0.0, 4, 2, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");

        // --- BME ---
        dtostrf(temp, 4, 2, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(pres, 4, 2, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(alt, 4, 2, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(hum, 4, 2, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");

        // --- BMI ---
        dtostrf(sensorData.ax, 4, 4, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(sensorData.ay, 4, 4, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(sensorData.az, 4, 4, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(sensorData.gx, 4, 4, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(sensorData.gy, 4, 4, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(sensorData.gz, 4, 4, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");

        // --- QMC ---
        ltoa(cx, tempBuffer, 10);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        ltoa(cy, tempBuffer, 10);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        ltoa(cz, tempBuffer, 10);
        strcat(logBuffer, tempBuffer);
        // 最後のデータなので , は不要

        // ★★★ 変更点: バッファ (RAM) に高速書き込み ★★★
        if (dataFile) {
            dataFile.println(logBuffer);
        } else {
            Serial.println("SD card file handle lost!");
            digitalWrite(error_ledpin, HIGH);
        }
    }


    // ★★★ 追加: SDカードへの物理書き込み (Flush) タイマー ★★★
    // 5msのログ処理とは別に、1秒に1回だけ実行
    if (currentMillis - lastFlushTime >= flushInterval) {
        lastFlushTime = currentMillis;

        if (dataFile) {
            dataFile.flush(); // バッファをSDカードに書き込む
            // Serial.println("Flashed buffer to SD."); // デバッグ用
        } else {
            Serial.println("SD card file handle lost! Cannot flush.");
            digitalWrite(error_ledpin, HIGH);
        }
    }
}