/***************************************************************************
 * 最終形態: 高速バイナリロガー (FreeRTOS 3タスク構成)
 * * - Task A (Core 0, Prio 2): sensorTask
 * - vTaskDelayUntil() を使い、正確な 5ms (200Hz) 間隔でセンサーを読み取る
 * - データを「キュー」に送信
 * - Task B (Core 1, Prio 1): sdWriteTask
 * - キューからデータを受信
 * - dataFile.write() で「バイナリ struct」をSDバッファに高速書き込み
 * - Task C (Core 1, Prio 0): sdFlushTask
 * - 1秒に1回、dataFile.flush() を実行 (低優先度)
 ***************************************************************************/

// 共通ライブラリ
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Wire.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <QMC5883LCompass.h>

//エラーランプ
const int error_ledpin = 41; 
const int stat_ledpin = 42; 

// ===== GPS =====
static const int RXPin = 7, TXPin = 6; 
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial ss(1); 
// ===============

// ===== SD =====
#define SD_CS_PIN      2  
#define SPI_MOSI_PIN   4  
#define SPI_MISO_PIN   3  
#define SPI_SCK_PIN    1  

SPIClass spi;
const char* fileName = "/fulldata.bin"; // ★CSVではなくバイナリ
File dataFile; 
// ===============

// ===== BME =====
#define SEALEVELPRESSURE_HPA (1011.4) 
#define I2C_SDA 17
#define I2C_SCL 16

Adafruit_BME280 bme; 
// ==============

// ... (BMI088, QMC5883L の定義 ...
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

QMC5883LCompass compass;

//================================================================
// ★★★ バイナリロギング用 構造体 (Struct) ★★★
//================================================================
// 構造体のパッキングを指示 (メモリ配置を最適化)
#pragma pack(push, 1) 
struct LogEntry {
  uint32_t timestamp; // millis()
  
  // BME
  float temp;
  float pres;
  float alt;
  float hum;
  
  // BMI (Acc)
  float ax;
  float ay;
  float az;
  
  // BMI (Gyro)
  float gx;
  float gy;
  float gz;
  
  // QMC
  int16_t cx; // int ではなく int16_t に
  int16_t cy;
  int16_t cz;
  
  // GPS
  int32_t lat; // float(6桁) は 32bit int に
  int32_t lng;
  float    gps_alt;
  uint8_t  sats;
  uint16_t date_year;
  uint8_t  date_month;
  uint8_t  date_day;
  uint8_t  time_hour;
  uint8_t  time_min;
  uint8_t  time_sec;
  uint8_t  time_cs; // 1/100秒
};
#pragma pack(pop)

//================================================================
// ★★★ FreeRTOS タスク設定 ★★★
//================================================================
TaskHandle_t hSensorTask;
TaskHandle_t hSdWriteTask;
TaskHandle_t hSdFlushTask;
QueueHandle_t xQueue; // タスク間通信用のキュー

// キューに溜められるログの最大数
#define QUEUE_LENGTH 50


// (I2Cヘルパー関数 writeRegister, readRegisters は変更なし)
void writeRegister(uint8_t i2c_addr, uint8_t reg_addr, uint8_t value) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.write(value);
  Wire.endTransmission();
}
void readRegisters(uint8_t i2c_addr, uint8_t reg_addr, uint8_t* buffer, uint8_t len) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.endTransmission(false); 
  Wire.requestFrom(i2c_addr, len);
  for (uint8_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }
}

// (bmisetup は変更なし)
void bmisetup(){
  Wire.setClock(400000); 
  Serial.println("Initializing Accelerometer...");
  writeRegister(ACC_ADDRESS, ACC_SOFTRESET, 0xB6); delay(100);
  writeRegister(ACC_ADDRESS, ACC_PWR_CTRL, 0x04); delay(50);
  writeRegister(ACC_ADDRESS, ACC_RANGE, 0x02); 
  writeRegister(ACC_ADDRESS, ACC_CONF, 0xA8); // ODR=100Hz
  writeRegister(ACC_ADDRESS, ACC_PWR_CONF, 0x00); delay(10);
  Serial.println("Accelerometer Initialized.");
  Serial.println("Initializing Gyroscope...");
  writeRegister(GYRO_ADDRESS, GYRO_SOFTRESET, 0xB6); delay(100);
  writeRegister(GYRO_ADDRESS, GYRO_RANGE, 0x00); 
  writeRegister(GYRO_ADDRESS, GYRO_BANDWIDTH, 0x88); // ODR=100Hz
  writeRegister(GYRO_ADDRESS, GYRO_LPM1, 0x00); delay(50);
  Serial.println("Gyroscope Initialized.");
}

// (qmcsetup は変更なし)
void qmcsetup(){
    compass.init();
    Serial.println("QMC initialized");
}


//================================================================
// ★ Task A (Core 0, Prio 2): センサー読み取り & キュー送信
//================================================================
void sensorTask(void *pvParameters) {
  Serial.println("Sensor Task started on Core 0 (Prio 2)");
  
  // このタスク専用の構造体インスタンス
  LogEntry entry; 
  uint8_t buffer[6]; // BMI読み取り用

  // --- 正確な5ms周期タイマーのセットアップ ---
  const TickType_t xFrequency = pdMS_TO_TICKS(5); // 5msをTickに変換
  TickType_t xLastWakeTime = xTaskGetTickCount(); // 現在時刻を取得

  for (;;) {
    // --- 5ms周期で正確に待機 ---
    // (次の 5ms の倍数の時刻までスリープする)
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // --- 処理開始 (ここが5msごとに実行される) ---
    entry.timestamp = millis();

    // GPS (非常に高速)
    while (ss.available() > 0) {
        gps.encode(ss.read());
    }

    // BME
    entry.temp = bme.readTemperature();
    entry.pres = bme.readPressure() / 100.0F;
    entry.alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
    entry.hum = bme.readHumidity();

    // BMI (Acc)
    readRegisters(ACC_ADDRESS, ACC_DATA_START, buffer, 6);
    entry.ax = (int16_t)((buffer[1] << 8) | buffer[0]) * ACC_SCALE * G_TO_MS2;
    entry.ay = (int16_t)((buffer[3] << 8) | buffer[2]) * ACC_SCALE * G_TO_MS2;
    entry.az = (int16_t)((buffer[5] << 8) | buffer[4]) * ACC_SCALE * G_TO_MS2;
    
    // BMI (Gyro)
    readRegisters(GYRO_ADDRESS, GYRO_DATA_START, buffer, 6);
    entry.gx = (int16_t)((buffer[1] << 8) | buffer[0]) * GYRO_SCALE;
    entry.gy = (int16_t)((buffer[3] << 8) | buffer[2]) * GYRO_SCALE;
    entry.gz = (int16_t)((buffer[5] << 8) | buffer[4]) * GYRO_SCALE;

    // QMC
    compass.read();
    entry.cx = compass.getX();
    entry.cy = compass.getY();
    entry.cz = compass.getZ();

    // GPS (GPSは1秒に1回しか更新されないが、常に最新の値を記録)
    entry.lat = gps.location.isValid() ? (int32_t)(gps.location.lat() * 1e6) : 0;
    entry.lng = gps.location.isValid() ? (int32_t)(gps.location.lng() * 1e6) : 0;
    entry.gps_alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    entry.sats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
    
    if (gps.date.isValid()) {
      entry.date_year = gps.date.year();
      entry.date_month = gps.date.month();
      entry.date_day = gps.date.day();
    }
    if (gps.time.isValid()) {
      entry.time_hour = gps.time.hour();
      entry.time_min = gps.time.minute();
      entry.time_sec = gps.time.second();
      entry.time_cs = gps.time.centisecond();
    }

    // --- データをキューに送信 ---
    // (キューが一杯の場合、1msだけ待つ。待てない場合は諦めて次のループへ)
    xQueueSend(xQueue, &entry, pdMS_TO_TICKS(1));
  }
}

//================================================================
// ★ Task B (Core 1, Prio 1): SDカードへのバイナリ書き込み
//================================================================
void sdWriteTask(void *pvParameters) {
  Serial.println("SD Write Task started on Core 1 (Prio 1)");
  LogEntry receivedEntry;

  for (;;) {
    // --- キューにデータが来るまで無限に待機 (スリープ) ---
    if (xQueueReceive(xQueue, &receivedEntry, portMAX_DELAY) == pdPASS) {
      
      // --- データを受信したら、SDバッファにバイナリを書き込む ---
      if (dataFile) {
        // これが最速の書き込み方法
        dataFile.write((const uint8_t*)&receivedEntry, sizeof(LogEntry));
      } else {
        digitalWrite(error_ledpin, HIGH);
      }
      
      // GPSが受信できていたらSTATLEDが点灯 (ここで制御)
      if (receivedEntry.sats > 0){
          digitalWrite(stat_ledpin, HIGH);
      } else {
          digitalWrite(stat_ledpin, LOW);
      }
    }
  }
}

//================================================================
// ★ Task C (Core 1, Prio 0): SDカードのフラッシュ (低優先度)
//================================================================
void sdFlushTask(void *pvParameters) {
  Serial.println("SD Flush Task started on Core 1 (Prio 0)");
  for (;;) {
    // --- 1秒待機 ---
    vTaskDelay(pdMS_TO_TICKS(1000)); 
    
    // --- 1秒に1回、物理書き込みを実行 ---
    if (dataFile) {
      dataFile.flush();
      // Serial.println("Flashed to SD."); // デバッグ用
    }
  }
}


//================================================================
// ★★★ メインの setup (Core 1) ★★★
//================================================================
void setup() {
    pinMode(error_ledpin, OUTPUT ); 
    pinMode(stat_ledpin, OUTPUT );  
    digitalWrite(error_ledpin, LOW);
    digitalWrite(stat_ledpin, LOW);

    Serial.begin(115200);
    Serial.println("\nBinary High-Speed Data Logger");

    spi.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);
    
    Wire.begin(I2C_SDA, I2C_SCL);
    bool status = bme.begin(0x76); 
    bmisetup();
    qmcsetup();

    if (!status) {
        Serial.println("BME280 init failed!");
        digitalWrite(error_ledpin, HIGH);
        while (1) delay(10);
    }
    
    ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

    Serial.println("Initializing SD card...");
    if (!SD.begin(SD_CS_PIN, spi)) {
        Serial.println("SD init failed!");
        digitalWrite(error_ledpin, HIGH);
        while (1); 
    }
    Serial.println("SD card initialized.");

    // ★バイナリファイルとして開く (FILE_WRITE)
    dataFile = SD.open(fileName, FILE_WRITE);
    if (!dataFile) {
        Serial.print("Error opening ");
        digitalWrite(error_ledpin, HIGH);
        Serial.println(fileName);
        while (1); 
    }
    Serial.print("Binary file opened: ");
    Serial.println(fileName);
    Serial.print("LogEntry struct size: ");
    Serial.println(sizeof(LogEntry)); // 構造体のサイズを確認

    // ★★★ タスクとキューの作成 ★★★

    // 1. キューを作成 ( LogEntry構造体を QUEUE_LENGTH 個格納できる )
    xQueue = xQueueCreate(QUEUE_LENGTH, sizeof(LogEntry));
    if (xQueue == NULL) {
      Serial.println("Queue creation failed!");
      while(1);
    }

    // 2. SD書き込みタスク (Core 1, Prio 1)
    xTaskCreatePinnedToCore(
        sdWriteTask,   
        "SDWriteTask", 
        4096,          
        NULL,          
        1,             // 優先度
        &hSdWriteTask, 
        1              // Core 1
    );

    // 3. SDフラッシュタスク (Core 1, Prio 0 - 最低)
    xTaskCreatePinnedToCore(
        sdFlushTask,   
        "SDFlushTask", 
        2048,          
        NULL,          
        0,             // 優先度
        &hSdFlushTask, 
        1              // Core 1
    );

    // 4. センサー読み取りタスク (Core 0, Prio 2 - 最高)
    xTaskCreatePinnedToCore(
        sensorTask,    
        "SensorTask",  
        4096,          
        NULL,          
        2,             // 優先度
        &hSensorTask,  
        0              // Core 0
    );

    // Arduinoのloopタスクはもう不要なので削除
    vTaskDelete(NULL); 
}

//================================================================
// ★★★ loop() はもう使われません ★★★
//================================================================
void loop() {
  // vTaskDelete(NULL) で削除されたため、ここには到達しない
}