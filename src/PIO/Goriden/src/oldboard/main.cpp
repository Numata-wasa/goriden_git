/***************************************************************************
 * 最終形態: 高速バイナリロガー (FreeRTOS 3タスク構成)
 * + GPS更新時にLEDを2回点滅させる機能を追加
 ***************************************************************************/

// 共通ライブラリ
#include <Arduino.h>
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
#define SEALEVELPRESSURE_HPA (1011.4)
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
// (オイラー角をクォータニオンに変更)
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
  float q0, q1, q2, q3; // クォータニオン (q0 = スカラー部)
};
#pragma pack(pop)

// ★★★ オイラー角計算用 IMUデータ構造体 ★★★
struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
  int16_t cx, cy, cz;
  uint32_t timestamp;
};

//================================================================
// ★★★ FreeRTOS タスク設定 ★★★
//================================================================
TaskHandle_t hSensorTask;
TaskHandle_t hSdWriteTask;
TaskHandle_t hSdFlushTask;
TaskHandle_t hEulerTask;
TaskHandle_t hSerialPrintTask;
QueueHandle_t xQueue;
QueueHandle_t xIMUQueue;
#define QUEUE_LENGTH 50
#define IMU_QUEUE_LENGTH 10

// ★★★ 共有クォータニオン (タスク間で参照) ★★★
volatile float sharedQ0 = 1.0f; // 初期値は単位クォータニオン
volatile float sharedQ1 = 0.0f;
volatile float sharedQ2 = 0.0f;
volatile float sharedQ3 = 0.0f;

// ★★★ シリアル表示用 最新エントリの共有コピー ★★★
LogEntry latestEntry;
portMUX_TYPE entryMux = portMUX_INITIALIZER_UNLOCKED;

// ★★★ LED点滅制御用 (グローバル変数) ★★★
// 0:Idle, 1:Blink1-ON, 2:Blink1-OFF(間), 3:Blink2-ON
byte ledBlinkState = 0;
unsigned long ledStateChangeTime = 0;
const long blinkOnDuration = 50;  // 1回の点灯時間 (ミリ秒)
const long blinkOffDuration = 40; // 点滅と点滅の間の消灯時間 (ミリ秒)


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
// ★ Task D (Core 0, Prio 1): クォータニオン計算 (相補フィルタ)
//================================================================
// ヘルパー関数: 加速度からクォータニオンを算出
void accelToQuaternion(float ax, float ay, float az, float &q0, float &q1, float &q2, float &q3) {
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return;
  ax /= norm; ay /= norm; az /= norm;
  
  // ロール・ピッチをクォータニオンに変換 (ヨーは0と仮定)
  float roll  = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  
  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);
  
  q0 = cr * cp;
  q1 = sr * cp;
  q2 = cr * sp;
  q3 = -sr * sp;
}

// ヘルパー関数: クォータニオンの正規化
void normalizeQuaternion(float &q0, float &q1, float &q2, float &q3) {
  float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (norm > 0.0f) {
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
  }
}

// ヘルパー関数: ジャイロをクォータニオン微分に変換して積分
void updateQuaternionFromGyro(float &q0, float &q1, float &q2, float &q3, 
                              float gx, float gy, float gz, float dt) {
  // deg->rad に変換
  float gxRad = gx * PI / 180.0f;
  float gyRad = gy * PI / 180.0f;
  float gzRad = gz * PI / 180.0f;
  
  // クォータニオン微分: dq = 0.5 * q * w (w: 角速度ベクトル)
  float dq0 = 0.5f * (-q1 * gxRad - q2 * gyRad - q3 * gzRad) * dt;
  float dq1 = 0.5f * ( q0 * gxRad + q2 * gzRad - q3 * gyRad) * dt;
  float dq2 = 0.5f * ( q0 * gyRad - q1 * gzRad + q3 * gxRad) * dt;
  float dq3 = 0.5f * ( q0 * gzRad + q1 * gyRad - q2 * gxRad) * dt;
  
  q0 += dq0;
  q1 += dq1;
  q2 += dq2;
  q3 += dq3;
  
  normalizeQuaternion(q0, q1, q2, q3);
}

// ヘルパー関数: チルト補正付き地磁気からヨーを推定してクォータニオンに反映
void correctQuaternionWithMag(float &q0, float &q1, float &q2, float &q3,
                              int16_t cx, int16_t cy, int16_t cz) {
  // 地磁気ベクトルを正規化
  float norm = sqrtf(cx * cx + cy * cy + cz * cz);
  if (norm == 0.0f) return;
  float mx = cx / norm, my = cy / norm, mz = cz / norm;
  
  // クォータニオンで地磁気ベクトルを回転させて地球座標系にマップ
  float rotMx = (1 - 2 * (q2 * q2 + q3 * q3)) * mx + 2 * (q1 * q2 - q0 * q3) * my + 2 * (q1 * q3 + q0 * q2) * mz;
  float rotMy = 2 * (q1 * q2 + q0 * q3) * mx + (1 - 2 * (q1 * q1 + q3 * q3)) * my + 2 * (q2 * q3 - q0 * q1) * mz;
  
  // 望ましいヨーを計算 (地磁気の水平成分から)
  float desiredYaw = atan2f(-rotMy, rotMx);
  
  // 現在のヨーを抽出
  float roll = atan2f(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
  float pitch = asinf(2 * (q0 * q2 - q3 * q1));
  float yaw = atan2f(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
  
  // ヨー誤差を少量修正 (低ゲイン補正)
  float yawError = desiredYaw - yaw;
  // yawErrorを[-pi, pi]に正規化
  while (yawError > PI) yawError -= 2 * PI;
  while (yawError < -PI) yawError += 2 * PI;
  
  // ヨー補正をクォータニオンに適用 (小さなクォータニオン回転)
  float corrQ0 = cosf(yawError * 0.05f * 0.5f);  // 低ゲイン: 0.05
  float corrQ3 = sinf(yawError * 0.05f * 0.5f);
  
  // クォータニオン乗算: q = corrQ * q
  float newQ0 = corrQ0 * q0 - corrQ3 * q3;
  float newQ1 = corrQ0 * q1 - corrQ3 * q2;
  float newQ2 = corrQ0 * q2 + corrQ3 * q1;
  float newQ3 = corrQ0 * q3 + corrQ3 * q0;
  
  q0 = newQ0; q1 = newQ1; q2 = newQ2; q3 = newQ3;
  normalizeQuaternion(q0, q1, q2, q3);
}

void eulerTask(void *pvParameters) {
  Serial.println("Quaternion Task started on Core 0 (Prio 1)");

  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // 単位クォータニオンで初期化
  const float alpha = 0.98f;  // 相補フィルタ係数 (ジャイロ信頼度)
  uint32_t lastTime = 0;
  bool initialized = false;
  IMUData imu;

  for (;;) {
    if (xQueueReceive(xIMUQueue, &imu, portMAX_DELAY) == pdPASS) {

      if (!initialized) {
        // 初回: 加速度からクォータニオンを初期化
        accelToQuaternion(imu.ax, imu.ay, imu.az, q0, q1, q2, q3);
        lastTime = imu.timestamp;
        initialized = true;
      } else {
        float dt = (imu.timestamp - lastTime) / 1000.0f; // 秒
        lastTime = imu.timestamp;
        if (dt <= 0.0f || dt > 0.5f) continue; // 異常な dt はスキップ

        // ジャイロでクォータニオンを更新
        float q0_gyro = q0, q1_gyro = q1, q2_gyro = q2, q3_gyro = q3;
        updateQuaternionFromGyro(q0_gyro, q1_gyro, q2_gyro, q3_gyro, imu.gx, imu.gy, imu.gz, dt);

        // 加速度からクォータニオンを推定
        float q0_acc, q1_acc, q2_acc, q3_acc;
        accelToQuaternion(imu.ax, imu.ay, imu.az, q0_acc, q1_acc, q2_acc, q3_acc);

        // 相補フィルタ: ジャイロ優先 + 加速度補正
        q0 = alpha * q0_gyro + (1.0f - alpha) * q0_acc;
        q1 = alpha * q1_gyro + (1.0f - alpha) * q1_acc;
        q2 = alpha * q2_gyro + (1.0f - alpha) * q2_acc;
        q3 = alpha * q3_gyro + (1.0f - alpha) * q3_acc;
        normalizeQuaternion(q0, q1, q2, q3);

        // 地磁気でヨーを補正
        correctQuaternionWithMag(q0, q1, q2, q3, imu.cx, imu.cy, imu.cz);
      }

      // 共有変数に書き込み
      sharedQ0 = q0;
      sharedQ1 = q1;
      sharedQ2 = q2;
      sharedQ3 = q3;
    }
  }
}

//================================================================
// ★ Task A (Core 0, Prio 2): センサー初期化 & 読み取り
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
    // --- オイラー角計算タスクへ IMU データを送信 ---
    IMUData imuData;
    imuData.ax = entry.ax; imuData.ay = entry.ay; imuData.az = entry.az;
    imuData.gx = entry.gx; imuData.gy = entry.gy; imuData.gz = entry.gz;
    imuData.cx = entry.cx; imuData.cy = entry.cy; imuData.cz = entry.cz;
    imuData.timestamp = entry.timestamp;
    xQueueSend(xIMUQueue, &imuData, 0); // ノンブロッキング送信

    // --- 最新のクォータニオンをログエントリに格納 ---
    entry.q0 = sharedQ0;
    entry.q1 = sharedQ1;
    entry.q2 = sharedQ2;
    entry.q3 = sharedQ3;

    // --- シリアル表示タスク用に最新エントリをコピー ---
    portENTER_CRITICAL(&entryMux);
    latestEntry = entry;
    portEXIT_CRITICAL(&entryMux);

    xQueueSend(xQueue, &entry, pdMS_TO_TICKS(1));
  }
}

//================================================================
// ★ Task E (Core 1, Prio 0): シリアルモニタへの定期出力
//================================================================
void serialPrintTask(void *pvParameters) {
  Serial.println("Serial Print Task started on Core 1 (Prio 0)");
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1秒間隔

    LogEntry e;
    portENTER_CRITICAL(&entryMux);
    e = latestEntry;
    portEXIT_CRITICAL(&entryMux);

    Serial.printf("[%lu] T:%.1f P:%.1f A:%.1f H:%.1f "
                  "Acc:%.2f,%.2f,%.2f Gyr:%.1f,%.1f,%.1f "
                  "Mag:%d,%d,%d "
                  "GPS:lat:%.6f,lng:%.6f,alt:%.1f,sat:%d "
                  "Date:%d/%d/%d %d:%02d:%02d.%02d "
                  "Quat:%.4f,%.4f,%.4f,%.4f upd:%d\n",
                  e.timestamp,
                  e.temp, e.pres, e.alt, e.hum,
                  e.ax, e.ay, e.az,
                  e.gx, e.gy, e.gz,
                  e.cx, e.cy, e.cz,
                  e.lat / 1e6, e.lng / 1e6, e.gps_alt, e.sats,
                  e.date_year, e.date_month, e.date_day,
                  e.time_hour, e.time_min, e.time_sec, e.time_cs,
                  e.q0, e.q1, e.q2, e.q3,
                  e.gps_updated);
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
    vTaskDelay(pdMS_TO_TICKS(1000)); // 起動直後の安定化のための短い遅延
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
      digitalWrite(error_ledpin, HIGH);
      while(1);
    }
    xIMUQueue = xQueueCreate(IMU_QUEUE_LENGTH, sizeof(IMUData));
    if (xIMUQueue == NULL) {
      Serial.println("IMU Queue creation failed!");
      digitalWrite(error_ledpin, HIGH);
      while(1);
    }

    xTaskCreatePinnedToCore(sdWriteTask,     "SDWriteTask",    4096, NULL, 1, &hSdWriteTask,    1);
    xTaskCreatePinnedToCore(sdFlushTask,     "SDFlushTask",    2048, NULL, 0, &hSdFlushTask,    1);
    xTaskCreatePinnedToCore(serialPrintTask, "SerialPrint",    4096, NULL, 0, &hSerialPrintTask,1);
    xTaskCreatePinnedToCore(eulerTask,       "EulerTask",      4096, NULL, 1, &hEulerTask,      0);
    xTaskCreatePinnedToCore(sensorTask,      "SensorTask",     4096, NULL, 2, &hSensorTask,     0);
}

//================================================================
// ★★★ loop() (Core 1, Prio 1) ★★★
//================================================================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(100));
}