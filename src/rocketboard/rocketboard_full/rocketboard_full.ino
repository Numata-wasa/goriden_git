/***************************************************************************
 * 最終形態: 高速バイナリロガー (FreeRTOS 3タスク構成)
 * + GPS更新時にLEDを2回点滅させる機能を追加
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

// ===== GPS (Core 0) =====
static const int RXPin = 7, TXPin = 6;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial ss(1);
// ===============

// ===== SD (Core 1) =====
#define SD_CS_PIN      2
#define SPI_MOSI_PIN   4
#define SPI_MISO_PIN   3
#define SPI_SCK_PIN    1

SPIClass spi;
const char* fileName = "/fulldata.bin";
File dataFile;
// ===============

// ===== BME (Core 0) =====
#define SEALEVELPRESSURE_HPA (1027.949818)
#define I2C_SDA 17
#define I2C_SCL 16

Adafruit_BME280 bme;
// ==============

// ... (BMI088, QMC5883L の定義は変更なし) ...
//================================================================
// BMI088 I2C Addresses
#define ACC_ADDRESS 0x19
#define GYRO_ADDRESS 0x69
// BMI088 Register Addresses (抜粋)
#define ACC_CHIP_ID      0x00
#define ACC_DATA_START   0x12
#define ACC_CONF         0x40
#define ACC_RANGE        0x41
#define ACC_PWR_CONF     0x7C
#define ACC_PWR_CTRL     0x7D
#define ACC_SOFTRESET    0x7E
#define GYRO_CHIP_ID     0x00
#define GYRO_DATA_START  0x02
#define GYRO_RANGE       0x0F
#define GYRO_BANDWIDTH   0x10
#define GYRO_LPM1        0x11
#define GYRO_SOFTRESET   0x14
// 物理量変換のためのスケールファクタ
const float ACC_SCALE = 12.0f / 32768.0f;
const float G_TO_MS2 = 9.80665f;
const float GYRO_SCALE = 2000.0f / 32768.0f;
QMC5883LCompass compass;
//================================================================


//================================================================
// ★★★ バイナリロギング用 構造体 (Struct) ★★★
// (前回の gps_updated を含むバージョンから変更なし)
//================================================================
#pragma pack(push, 1)
struct LogEntry {
  uint32_t timestamp;
  float temp, pres, alt, hum;
  float ax, ay, az;
  float gx, gy, gz;
  int16_t cx, cy, cz;
  int32_t lat, lng;
  float    gps_alt;
  uint8_t  sats;
  uint16_t date_year;
  uint8_t  date_month, date_day;
  uint8_t  time_hour, time_min, time_sec, time_cs;
  uint8_t  gps_updated; // GPS座標が更新されたかのフラグ
};
#pragma pack(pop)

//================================================================
// ★★★ FreeRTOS タスク設定 ★★★
//================================================================
TaskHandle_t hSensorTask;
TaskHandle_t hSdWriteTask;
TaskHandle_t hSdFlushTask;
QueueHandle_t xQueue;
#define QUEUE_LENGTH 50

// ★★★ LED点滅制御用 (グローバル変数) ★★★
// 0:Idle, 1:Blink1-ON, 2:Blink1-OFF(間), 3:Blink2-ON
byte ledBlinkState = 0;
unsigned long ledStateChangeTime = 0;
const long blinkOnDuration = 50;  // 1回の点灯時間 (ミリ秒)
const long blinkOffDuration = 80; // 点滅と点滅の間の消灯時間 (ミリ秒)


// (I2Cヘルパー関数, bmisetup, qmcsetup は変更なし)
void writeRegister(uint8_t i2c_addr, uint8_t reg_addr, uint8_t value) {
  Wire.beginTransmission(i2c_addr); Wire.write(reg_addr); Wire.write(value); Wire.endTransmission();
}
void readRegisters(uint8_t i2c_addr, uint8_t reg_addr, uint8_t* buffer, uint8_t len) {
  Wire.beginTransmission(i2c_addr); Wire.write(reg_addr); Wire.endTransmission(false);
  Wire.requestFrom(i2c_addr, len);
  for (uint8_t i = 0; i < len; i++) { buffer[i] = Wire.read(); }
}
void bmisetup(){
  Wire.setClock(400000);
  Serial.println("Initializing Accelerometer...");
  writeRegister(ACC_ADDRESS, ACC_SOFTRESET, 0xB6); delay(100);
  writeRegister(ACC_ADDRESS, ACC_PWR_CTRL, 0x04); delay(50);
  writeRegister(ACC_ADDRESS, ACC_RANGE, 0x02);
  writeRegister(ACC_ADDRESS, ACC_CONF, 0xA9);
  writeRegister(ACC_ADDRESS, ACC_PWR_CONF, 0x00); delay(10);
  Serial.println("Accelerometer Initialized.");
  Serial.println("Initializing Gyroscope...");
  writeRegister(GYRO_ADDRESS, GYRO_SOFTRESET, 0xB6); delay(100);
  writeRegister(GYRO_ADDRESS, GYRO_RANGE, 0x00);
  writeRegister(GYRO_ADDRESS, GYRO_BANDWIDTH, 0x04);
  writeRegister(GYRO_ADDRESS, GYRO_LPM1, 0x00); delay(50);
  Serial.println("Gyroscope Initialized.");
}
void qmcsetup(){
    compass.init();
    Serial.println("QMC initialized");
}


//================================================================
// ★ Task A (Core 0, Prio 2): センサー初期化 & 読み取り
// (変更なし)
//================================================================
void sensorTask(void *pvParameters) {

  Serial.println("Sensor Task started on Core 0 (Prio 2)");
  // --- Core 0 ハードウェア初期化 ---
  Serial.println("Initializing GPS Serial (UART) on Core 0...");
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  Serial.println("GPS Serial Initialized.");
  Serial.println("Initializing I2C on Core 0...");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  bool status = bme.begin(0x76);
  if (!status) {
      Serial.println("!!! FATAL: BME280 init failed on Core 0!");
      digitalWrite(error_ledpin, HIGH);
      while(1) vTaskDelay(100);
  }
  Serial.println("BME280 Initialized.");

  bmisetup();
  qmcsetup();
  Serial.println("All I2C sensors initialized on Core 0.");


  LogEntry entry;
  uint8_t buffer[6];
  const TickType_t xFrequency = pdMS_TO_TICKS(5);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    entry.timestamp = millis();
    while (ss.available() > 0) { gps.encode(ss.read()); }
    entry.gps_updated = gps.location.isUpdated() ? 1 : 0;

    // --- センサーデータ読み取り ---
    entry.temp = bme.readTemperature();
    entry.pres = bme.readPressure() / 100.0F;
    entry.alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
    entry.hum = bme.readHumidity();
    readRegisters(ACC_ADDRESS, ACC_DATA_START, buffer, 6);
    entry.ax = (int16_t)((buffer[1] << 8) | buffer[0]) * ACC_SCALE * G_TO_MS2;
    entry.ay = (int16_t)((buffer[3] << 8) | buffer[2]) * ACC_SCALE * G_TO_MS2;
    entry.az = (int16_t)((buffer[5] << 8) | buffer[4]) * ACC_SCALE * G_TO_MS2;
    readRegisters(GYRO_ADDRESS, GYRO_DATA_START, buffer, 6);
    entry.gx = (int16_t)((buffer[1] << 8) | buffer[0]) * GYRO_SCALE;
    entry.gy = (int16_t)((buffer[3] << 8) | buffer[2]) * GYRO_SCALE;
    entry.gz = (int16_t)((buffer[5] << 8) | buffer[4]) * GYRO_SCALE;
    compass.read();
    entry.cx = compass.getX(); entry.cy = compass.getY(); entry.cz = compass.getZ();
    // --- GPSデータ格納 ---
    entry.lat = gps.location.isValid() ? (int32_t)(gps.location.lat() * 1e6) : 0;
    entry.lng = gps.location.isValid() ? (int32_t)(gps.location.lng() * 1e6) : 0;
    entry.gps_alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    entry.sats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
    if (gps.date.isValid()) {
      entry.date_year = gps.date.year(); entry.date_month = gps.date.month(); entry.date_day = gps.date.day();
    }
    if (gps.time.isValid()) {
      entry.time_hour = gps.time.hour(); entry.time_min = gps.time.minute(); entry.time_sec = gps.time.second(); entry.time_cs = gps.time.centisecond();
    }
    xQueueSend(xQueue, &entry, pdMS_TO_TICKS(1));
  }
}

//================================================================
// ★ Task B (Core 1, Prio 1): SDカードへのバイナリ書き込み & 2回点滅
// (LED点滅ロジックをステートマシンに変更)
//================================================================
void sdWriteTask(void *pvParameters) {
  Serial.println("SD Write Task started on Core 1 (Prio 1)");
  LogEntry receivedEntry;

  for (;;) {
    // --- 1. キューにデータが来るまで待機 ---
    // (タイムアウトなしで待つ)
    if (xQueueReceive(xQueue, &receivedEntry, portMAX_DELAY) == pdPASS) {

      // --- 2. SDカードへの書き込み ---
      if (dataFile) {
        dataFile.write((const uint8_t*)&receivedEntry, sizeof(LogEntry));
      } else {
        digitalWrite(error_ledpin, HIGH);
      }

      // ★★★ 3. GPS更新をトリガーに点滅開始 ★★★
      if (receivedEntry.gps_updated == 1 && ledBlinkState == 0) {
          digitalWrite(stat_ledpin, HIGH); // 1回目の点灯開始
          ledBlinkState = 1;               // 状態を「1回目の点灯中」へ
          ledStateChangeTime = millis();     // 状態が変わった時刻を記録
      }
    }

    // ★★★ 4. LED点滅ステートマシン (毎回実行) ★★★
    unsigned long currentTime = millis();
    
    // 状態1: 1回目の点灯中か？
    if (ledBlinkState == 1 && currentTime - ledStateChangeTime >= blinkOnDuration) {
        digitalWrite(stat_ledpin, LOW);  // 消灯
        ledBlinkState = 2;               // 状態を「点滅の合間の消灯中」へ
        ledStateChangeTime = currentTime;    // 状態が変わった時刻を記録
    }
    // 状態2: 点滅の合間の消灯中か？
    else if (ledBlinkState == 2 && currentTime - ledStateChangeTime >= blinkOffDuration) {
        digitalWrite(stat_ledpin, HIGH); // 2回目の点灯開始
        ledBlinkState = 3;               // 状態を「2回目の点灯中」へ
        ledStateChangeTime = currentTime;    // 状態が変わった時刻を記録
    }
    // 状態3: 2回目の点灯中か？
    else if (ledBlinkState == 3 && currentTime - ledStateChangeTime >= blinkOnDuration) {
        digitalWrite(stat_ledpin, LOW);  // 最終的な消灯
        ledBlinkState = 0;               // 状態を「アイドル」に戻す
    }
    
    // 他のタスクにCPUを譲る
    vTaskDelay(pdMS_TO_TICKS(1));

  } // 無限ループの終わり
}

//================================================================
// ★ Task C (Core 1, Prio 0): SDカードのフラッシュ
// (変更なし)
//================================================================
void sdFlushTask(void *pvParameters) {
  Serial.println("SD Flush Task started on Core 1 (Prio 0)");
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (dataFile) {
      dataFile.flush();
    }
  }
}


//================================================================
// ★★★ メインの setup (Core 1) ★★★
// (変更なし)
//================================================================
void setup() {
    pinMode(error_ledpin, OUTPUT );
    pinMode(stat_ledpin, OUTPUT );
    digitalWrite(error_ledpin, LOW);
    digitalWrite(stat_ledpin, LOW);

    Serial.begin(115200);
    Serial.println("\nBinary Logger - GPS Double Blink Version");

    spi.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);

    Serial.println("Initializing SD card (on Core 1)...");
    if (!SD.begin(SD_CS_PIN, spi)) {
        Serial.println("SD init failed!");
        digitalWrite(error_ledpin, HIGH);
        while (1);
    }
    Serial.println("SD card initialized.");

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
    Serial.println(sizeof(LogEntry));

    // --- タスクとキューの作成 ---
    xQueue = xQueueCreate(QUEUE_LENGTH, sizeof(LogEntry));
    if (xQueue == NULL) {
      Serial.println("Queue creation failed!");
      while(1);
    }

    xTaskCreatePinnedToCore(sdWriteTask, "SDWriteTask", 4096, NULL, 1, &hSdWriteTask, 1);
    xTaskCreatePinnedToCore(sdFlushTask, "SDFlushTask", 2048, NULL, 0, &hSdFlushTask, 1);
    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 2, &hSensorTask, 0);
}

//================================================================
// ★★★ loop() (Core 1, Prio 1) ★★★
//================================================================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(100));
}