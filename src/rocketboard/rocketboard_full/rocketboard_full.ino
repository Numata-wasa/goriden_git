/***************************************************************************
 * 高速ロギング デュアルコア (FreeRTOS) 対応版
 *
 * Core 1 (loop): 5msタイマーでの高速ロギングとSDフラッシュを担当
 * Core 0 (sensorReadTask): 全てのI2Cセンサーの読み取りを専門に担当
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
static const int RXPin = 7, TXPin = 6; 
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial ss(1); 
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
#define SEALEVELPRESSURE_HPA (1011.4) 
#define I2C_SDA 17
#define I2C_SCL 16

Adafruit_BME280 bme; // I2C
// ==============

// ===== 時間管理 =====
unsigned long lastLogTime = 0;
const long logInterval = 5; // 200Hz

unsigned long lastFlushTime = 0;
const long flushInterval = 1000; // 1秒に1回フラッシュ
//================================================================

// ... (BMI088, QMC5883L の定義) ...
// (元のコードの #define ACC_ADDRESS ... から SensorData sensorData; までをそのまま挿入)
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
// ★★★ デュアルコア対応 ★★★
//================================================================
TaskHandle_t hSensorTask; // センサータスクのハンドル
SemaphoreHandle_t dataMutex; // データ保護用のミューテックス

// 2つのコアで共有するセンサーデータ
// BMEのデータを追加
struct SharedSensorData {
  SensorData bmi; // BMI088
  float temp, pres, alt, hum; // BME280
  int cx, cy, cz; // QMC5883L
};

// volatile: コンパイラの最適化を無効にし、常にメモリから読み込む
volatile SharedSensorData g_sharedData; 
//================================================================


// ... (I2Cヘルパー関数 writeRegister, readRegisters は変更なし) ...
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


// ... (bmisetup は変更なし) ...
void bmisetup(){
  // (元のコードと全く同じ)
  Wire.setClock(400000); // I2Cクロックを400kHzに設定
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

// ★修正: bmieget はローカル変数に読み込む
SensorData bmieget_local(){
  uint8_t buffer[6];
  SensorData localData; // ローカルの構造体

  //---- Read Accelerometer Data ----
  readRegisters(ACC_ADDRESS, ACC_DATA_START, buffer, 6);
  int16_t raw_ax = (int16_t)((buffer[1] << 8) | buffer[0]);
  int16_t raw_ay = (int16_t)((buffer[3] << 8) | buffer[2]);
  int16_t raw_az = (int16_t)((buffer[5] << 8) | buffer[4]);
  localData.ax = raw_ax * ACC_SCALE * G_TO_MS2;
  localData.ay = raw_ay * ACC_SCALE * G_TO_MS2;
  localData.az = raw_az * ACC_SCALE * G_TO_MS2;

  //---- Read Gyroscope Data ----
  readRegisters(GYRO_ADDRESS, GYRO_DATA_START, buffer, 6);
  int16_t raw_gx = (int16_t)((buffer[1] << 8) | buffer[0]);
  int16_t raw_gy = (int16_t)((buffer[3] << 8) | buffer[2]);
  int16_t raw_gz = (int16_t)((buffer[5] << 8) | buffer[4]);
  localData.gx = raw_gx * GYRO_SCALE;
  localData.gy = raw_gy * GYRO_SCALE;
  localData.gz = raw_gz * GYRO_SCALE;

  return localData;
}

// ... (qmcsetup は変更なし) ...
void qmcsetup(){
    compass.init();
    Serial.println("QMC initialized");
}

// ★修正: qmcget は引数のポインタに書き込む
void qmcget_local(int* p_cx, int* p_cy, int* p_cz){
    compass.read();
    *p_cx = compass.getX();
    *p_cy = compass.getY();
    *p_cz = compass.getZ();
}


//================================================================
// ★★★ Core 0 で実行されるセンサー読み取り専門タスク ★★★
//================================================================
void sensorReadTask(void *pvParameters) {
  Serial.println("Sensor Read Task started on Core 0.");

  // センサー読み取り用のローカル変数
  SensorData localBmi;
  float localTemp, localPres, localAlt, localHum;
  int localCx, localCy, localCz;

  // 無限ループ
  for (;;) {
    
    // --- 1. I2Cセンサーからデータを読み取る ---
    // (この処理はロックの外で行う)
    localBmi = bmieget_local();
    qmcget_local(&localCx, &localCy, &localCz);
    localTemp = bme.readTemperature();
    localPres = bme.readPressure() / 100.0F;
    localAlt = bme.readAltitude(SEALEVELPRESSURE_HPA);
    localHum = bme.readHumidity();

    // --- 2. データをロックして共有メモリにコピー ---
    // (portMAX_DELAY = ロックが取れるまで無限に待つ)
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      
      // グローバル共有変数に一括コピー
      g_sharedData.bmi = localBmi;
      g_sharedData.temp = localTemp;
      g_sharedData.pres = localPres;
      g_sharedData.alt = localAlt;
      g_sharedData.hum = localHum;
      g_sharedData.cx = localCx;
      g_sharedData.cy = localCy;
      g_sharedData.cz = localCz;

      // ロックを解除
      xSemaphoreGive(dataMutex);
    }
    
    // --- 3. 少し待つ ---
    // (CPUを100%使い切らないよう、OSに処理を返す)
    // 1ms待機。センサーのODR (100Hz=10ms) に合わせて調整可能
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}


//================================================================
// ★★★ Core 1 で実行されるメインの setup ★★★
//================================================================
void setup() {
    pinMode(error_ledpin, OUTPUT ); 
    pinMode(stat_ledpin, OUTPUT );  

    Serial.begin(115200);
    Serial.println("\nDual-Core High-Speed Data Logger");

    spi.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);
    
    // ★重要: Core 0 タスクより先にI2Cを初期化
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

    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
        dataFile.println("Date,Time_cs,millis,Lat,Lng,Sat,Alt,Temp,Pres,PrAl,Humi,ax,ay,az,gx,gy,gz,cx,cy,cz");
        Serial.print("Header written to ");
        Serial.println(fileName);
    } else {
        Serial.print("Error opening ");
        digitalWrite(error_ledpin, HIGH);
        Serial.println(fileName);
        while (1); 
    }
    Serial.println();

    // ★★★ デュアルコア処理の開始 ★★★
    
    // 1. ミューテックス（ロック機構）を作成
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
      Serial.println("Mutex creation failed!");
      while(1);
    }

    // 2. センサー読み取りタスクを Core 0 に割り当てて開始
    xTaskCreatePinnedToCore(
        sensorReadTask,   // 実行する関数
        "SensorTask",     // タスク名
        4096,             // スタックサイズ (bytes)
        NULL,             // タスク引数
        1,                // 優先度 (0=低, 1=中, ...)
        &hSensorTask,     // タスクハンドル
        0                 // 実行コア (0 = Core 0)
    );
}

//================================================================
// ★★★ Core 1 で実行されるメインの loop (ロギング担当) ★★★
//================================================================
void loop() {
    // --- GPS処理 (Core 1 で実行) ---
    while (ss.available() > 0) {
        gps.encode(ss.read());
    }
    if (gps.location.isUpdated()){
        digitalWrite(stat_ledpin, HIGH);
    } else {
        digitalWrite(stat_ledpin, LOW);
    }

    unsigned long currentMillis = millis();

    // --- メインの高速ロギングタイマー (Core 1) ---
    if (currentMillis - lastLogTime >= logInterval) {
        lastLogTime = currentMillis;

        char logBuffer[512]; 
        char tempBuffer[50]; 
        
        // --- 1. 共有メモリからデータを安全にコピーする ---
        // (このブロックは素早く抜ける)
        SharedSensorData localData; // ログ整形用のローカルコピー
        
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) { // 1msだけ待つ
          // 共有データをローカルにコピー
          localData = g_sharedData;
          // ロックを解除
          xSemaphoreGive(dataMutex);
        } else {
          // 1ms待ってもロックが取れない = 異常
          Serial.println("Log task could not get mutex!");
          // (エラー時のデータをどうするか？ とりあえず古いデータでログる)
        }

        // --- 2. センサー読み取り以外の処理 (文字列フォーマット) ---
        // (ロックの外で、時間をかけて実行する)

        // --- GPS Date ---
        if (gps.date.isValid()) {
            snprintf(tempBuffer, sizeof(tempBuffer), "%04d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());
        } else {
            strcpy(tempBuffer, "N/A");
        }
        strcpy(logBuffer, tempBuffer); 
        strcat(logBuffer, ","); 

        // --- GPS Time ---
        if (gps.time.isValid()) {
            snprintf(tempBuffer, sizeof(tempBuffer), "%02d:%02d:%02d.%02d", 
                     gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
        } else {
            strcpy(tempBuffer, "N/A");
        }
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");

        // --- Millis ---
        ltoa(currentMillis, tempBuffer, 10);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");

        // --- GPS Location ---
        dtostrf(gps.location.isValid() ? gps.location.lat() : 0.0, 4, 6, tempBuffer);
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

        // --- BME (コピー済みのローカルデータから) ---
        dtostrf(localData.temp, 4, 2, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(localData.pres, 4, 2, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(localData.alt, 4, 2, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(localData.hum, 4, 2, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");

        // --- BMI (コピー済みのローカルデータから) ---
        dtostrf(localData.bmi.ax, 4, 4, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(localData.bmi.ay, 4, 4, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(localData.bmi.az, 4, 4, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(localData.bmi.gx, 4, 4, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(localData.bmi.gy, 4, 4, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        dtostrf(localData.bmi.gz, 4, 4, tempBuffer);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");

        // --- QMC (コピー済みのローカルデータから) ---
        ltoa(localData.cx, tempBuffer, 10);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        ltoa(localData.cy, tempBuffer, 10);
        strcat(logBuffer, tempBuffer);
        strcat(logBuffer, ",");
        ltoa(localData.cz, tempBuffer, 10);
        strcat(logBuffer, tempBuffer);

        // --- 3. SDバッファへの書き込み ---
        if (dataFile) {
            dataFile.println(logBuffer);
        } else {
            Serial.println("SD card file handle lost!");
            digitalWrite(error_ledpin, HIGH);
        }
    }

    // --- SDカードへの物理書き込みタイマー (Core 1) ---
    if (currentMillis - lastFlushTime >= flushInterval) {
        lastFlushTime = currentMillis;
        if (dataFile) {
            dataFile.flush(); 
        } else {
            Serial.println("SD card file handle lost! Cannot flush.");
            digitalWrite(error_ledpin, HIGH);
        }
    }
}