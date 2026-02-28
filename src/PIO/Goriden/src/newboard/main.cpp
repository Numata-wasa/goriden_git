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
#include <esp_log.h>
#include "BMI2_BMM1.h"
// BMI270 + BMM150 (AUX接続用)

//エラーランプ
const int error_ledpin = 41;
const int stat_ledpin = 42;

// ===== GPS (Core 0) =====
static const int RXPin = 11, TXPin = 12;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial ss(1);
// ===============

// ===== SD (Core 1) =====
#define SD_CS_PIN      13
#define SPI_MOSI_PIN   14
#define SPI_MISO_PIN   47
#define SPI_SCK_PIN    21

SPIClass spi;
const char* fileName = "/fulldata.bin";
File dataFile;
// ===============

// ===== BME (Core 0) =====
#define SEALEVELPRESSURE_HPA (1011.4)
// I2Cピン：Arduinoフレームワークのデフォルト定義を使用
// カスタムピン: SDA=17, SCL=16

Adafruit_BME280 bme;
bool bmeAvailable = false;  // BME280が正常に初期化されたかのフラグ
bool bmi270Available = false;  // BMI270が正常に初期化されたかのフラグ
// ==============

// ===== サーボ制御 (ESP32 LEDC PWM) =====
#define SERVO_PIN_1 5         // サーボ1のピン
#define SERVO_PIN_2 4         // サーボ2のピン
#define SERVO_PWM_CHANNEL_1 0 // LEDC チャネル1
#define SERVO_PWM_CHANNEL_2 1 // LEDC チャネル2
#define SERVO_PWM_FREQ 50     // 50Hz (サーボ標準)
#define SERVO_PWM_BITS 16     // 16bit resolution
#define SERVO_CENTER_US 1500  // サーボ中央値 [μs]
#define SERVO_RANGE_US 500    // ±範囲 [μs] (1000-2000μs対応)
#define SERVO_UPDATE_FREQ 20  // サーボ更新頻度 [Hz]

// ===== 姿勢制御 PID パラメータ =====
const float target_roll  = 0.0f;         // 目標ロール [rad]
const float target_pitch = -PI / 2.0f;   // 目標ピッチ [rad] (-90度)
const float target_yaw   = 0.0f;         // 目標ヨー [rad]

const float Kp_roll  = 200.0f;  // ロール比例ゲイン
const float Ki_roll  = 10.0f;   // ロール積分ゲイン
const float Kd_roll  = 50.0f;   // ロール微分ゲイン

const float Kp_pitch = 200.0f;  // ピッチ比例ゲイン
const float Ki_pitch = 10.0f;   // ピッチ積分ゲイン
const float Kd_pitch = 50.0f;   // ピッチ微分ゲイン

// PID制御用構造体
struct PIDController {
  float error_integral;  // 積分項
  float prev_error;      // 前フレームの誤差
};

volatile float servo_cmd_roll = 0.0f;   // ロール制御コマンド [-1, 1]
volatile float servo_cmd_pitch = 0.0f;  // ピッチ制御コマンド [-1, 1]
PIDController pid_roll, pid_pitch;      // PIDコントローラ
// ==============

// ===== BMI270 + BMM150 (Library) =====
BMI2_BMM1_Class imuSensor;  // BMI2_BMM1ライブラリのインスタンス（名前を変更）

// スケール係数（ライブラリから得られる生データを物理値に変換）
#define ACC_SCALE_2G (1.0f / 16384.0f)        // ±2g設定時の加速度スケール [g]
#define GYRO_SCALE_2000DPS (1.0f / 16.4f)     // ±2000dps設定時のジャイロスケール [deg/s]
#define MAG_SCALE (0.3f)                       // 磁気スケール [uT]
const float G_TO_MS2 = 9.80665f;               // gからm/s^2への変換係数
// =======================================
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
  float roll, pitch, yaw; // オイラー角 [rad]
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


//================================================================
// ★★★ サーボ制御補助関数 ★★★
//================================================================
// ヘルパー関数: 角度誤差を[-pi, pi]に正規化
float wrapAngleError(float error) {
  while (error > PI) error -= 2 * PI;
  while (error < -PI) error += 2 * PI;
  return error;
}

// ヘルパー関数: PID制御で制御コマンドを計算
void calculatePIDControl(float error, PIDController &pid, float Kp, float Ki, float Kd, float dt, float &output) {
  // 誤差を正規化
  error = wrapAngleError(error);
  
  // 積分項を更新
  pid.error_integral += error * dt;
  // 積分項の飽和防止 [-1, 1]
  pid.error_integral = fmaxf(-1.0f, fminf(1.0f, pid.error_integral));
  
  // 微分項
  float error_derivative = (error - pid.prev_error) / dt;
  pid.prev_error = error;
  
  // PID出力
  output = Kp * error + Ki * pid.error_integral + Kd * error_derivative;
  // 出力を[-1, 1]に制限
  output = fmaxf(-1.0f, fminf(1.0f, output));
}

// ヘルパー関数: サーボピンをPWM制御
void setServoUS(int channel, float cmd) {
  // cmd: [-1, 1]
  // 出力範囲: 1000-2000 μs (中央1500)
  int pulse_width = (int)(SERVO_CENTER_US + cmd * SERVO_RANGE_US);
  // PWM周期に対応するduty cycle を計算: 50Hz = 20000μs周期
  // 各周期 = 20000μs / 2^16 ≈ 0.305μs
  // duty = pulse_width / 20000 * 2^16
  int duty = (int)(pulse_width * 65536 / 20000);
  duty = fmaxf(0, fminf(65535, duty));
  ledcWrite(channel, duty);
}

//================================================================
// ★★★ BMI270 初期化関数（ライブラリ使用） ★★★
//================================================================
bool bmi270setup(){
  Serial.println("Initializing BMI270+BMM150 with Library...");
  
  if (!imuSensor.begin(&Serial)) {
    Serial.println("!!! WARNING: IMU initialization failed!");
    return false;
  }
  
  Serial.println("BMI270+BMM150 initialized successfully.");
  return true;
}


//================================================================
// ★ Task D (Core 0, Prio 1): クォータニオン計算 (相補フィルタ)
//================================================================
// ヘルパー関数: 角度誤差を[-pi, pi]に正規化
float wrapAngleError(float error) {
  while (error > PI) error -= 2 * PI;
  while (error < -PI) error += 2 * PI;
  return error;
}

// ヘルパー関数: PID制御でサーボ出力を計算
void calculatePIDControl(float error, PIDController &pid, float Kp, float Ki, float Kd, float dt, float &output) {
  // 誤差を正規化
  error = wrapAngleError(error);
  
  // 積分項を更新
  pid.error_integral += error * dt;
  // 積分項の飽和防止 [-1, 1]
  pid.error_integral = fmaxf(-1.0f, fminf(1.0f, pid.error_integral));
  
  // 微分項
  float error_derivative = (error - pid.prev_error) / dt;
  pid.prev_error = error;
  
  // PID出力
  output = Kp * error + Ki * pid.error_integral + Kd * error_derivative;
  // 出力を[-1, 1]に制限
  output = fmaxf(-1.0f, fminf(1.0f, output));
}

// ヘルパー関数: サーボピンをセット
void setServoUS(int pin, int channel, float cmd) {
  // cmd: [-1, 1]
  // 出力範囲: 1000-2000 μs (中心1500)
  int pulse_width = (int)(SERVO_CENTER_US + cmd * SERVO_RANGE_US);
  // PWM周期に対応するduty cycleを計算: 50Hz = 20000μs周期
  // 各周期 = 20000μs / 2^16 ≈ 0.305μs
  // duty = pulse_width / 20000 * 2^16
  int duty = (int)(pulse_width * 65536 / 20000);
  duty = fmaxf(0, fminf(65535, duty));
  ledcWrite(channel, duty);
}

// ヘルパー関数: クォータニオン計算 (相補フィルタ

void accelToQuaternion(float ax, float ay, float az, float &q0, float &q1, float &q2, float &q3) {
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) {
    // 加速度が0の場合は単位クォータニオンを設定
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    return;
  }
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

// ヘルパー関数: クォータニオンからオイラー角を計算
void quaternionToEuler(float q0, float q1, float q2, float q3, float &roll, float &pitch, float &yaw) {
  // Roll (x軸周りの回転)
  roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
  
  // Pitch (y軸周りの回転) - asinf引数をクランプ
  float pitchArg = 2.0f * (q0 * q2 - q3 * q1);
  pitchArg = fmaxf(-1.0f, fminf(1.0f, pitchArg));
  pitch = asinf(pitchArg);
  
  // Yaw (z軸周りの回転)
  yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

// ヘルパー関数: クォータニオンの正規化
void normalizeQuaternion(float &q0, float &q1, float &q2, float &q3) {
  float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (norm > 0.0f) {
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
  } else {
    // ノルムが0の場合は単位クォータニオンを設定
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
  }
  
  // NaNチェック：もしNaNが含まれていたら単位クォータニオンにリセット
  if (isnan(q0) || isnan(q1) || isnan(q2) || isnan(q3)) {
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
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
  if (norm == 0.0f) return;  // 地磁気データが無効な場合は何もしない
  float mx = cx / norm, my = cy / norm, mz = cz / norm;
  
  // クォータニオンで地磁気ベクトルを回転させて地球座標系にマップ
  float rotMx = (1 - 2 * (q2 * q2 + q3 * q3)) * mx + 2 * (q1 * q2 - q0 * q3) * my + 2 * (q1 * q3 + q0 * q2) * mz;
  float rotMy = 2 * (q1 * q2 + q0 * q3) * mx + (1 - 2 * (q1 * q1 + q3 * q3)) * my + 2 * (q2 * q3 - q0 * q1) * mz;
  
  // 望ましいヨーを計算 (地磁気の水平成分から)
  float desiredYaw = atan2f(-rotMy, rotMx);
  
  // 現在のヨーを抽出
  float roll = atan2f(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
  // asinf引数を[-1, 1]にクランプしてNaNを防ぐ
  float pitchArg = 2 * (q0 * q2 - q3 * q1);
  pitchArg = fmaxf(-1.0f, fminf(1.0f, pitchArg));
  float pitch = asinf(pitchArg);
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

        // 地磁気センサを無視: ヨーの地磁気補正は行わない
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
// ★ Task F (Core 1, Prio 2): サーボ制御タスク (PID制御)
//================================================================
void servoControlTask(void *pvParameters) {
  Serial.println("Servo Control Task started on Core 1 (Prio 2)");
  
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / SERVO_UPDATE_FREQ); // SERVO_UPDATE_FREQ Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  // PIDコントローラを初期化
  pid_roll.error_integral = 0.0f;
  pid_roll.prev_error = 0.0f;
  pid_pitch.error_integral = 0.0f;
  pid_pitch.prev_error = 0.0f;
  
  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // 現在のオイラー角を取得（最新ログエントリから読み込み）
    portENTER_CRITICAL(&entryMux);
    float current_roll = latestEntry.roll;
    float current_pitch = latestEntry.pitch;
    portEXIT_CRITICAL(&entryMux);
    
    // 制御周期を計算
    float dt = (float)xFrequency / 1000.0f; // 秒
    
    // 誤差を計算
    float error_roll = current_roll - target_roll;
    float error_pitch = current_pitch - target_pitch;
    
    // PID制御でサーボコマンドを計算
    float cmd_roll = 0.0f, cmd_pitch = 0.0f;
    calculatePIDControl(error_roll, pid_roll, Kp_roll, Ki_roll, Kd_roll, dt, cmd_roll);
    calculatePIDControl(error_pitch, pid_pitch, Kp_pitch, Ki_pitch, Kd_pitch, dt, cmd_pitch);
    
    // 共有変数に保存
    servo_cmd_roll = cmd_roll;
    servo_cmd_pitch = cmd_pitch;
    
    // サーボにPWM信号を出力
    setServoUS(SERVO_PWM_CHANNEL_1, servo_cmd_roll);   // サーボ1
    setServoUS(SERVO_PWM_CHANNEL_2, servo_cmd_pitch);  // サーボ2
    
    // DebugSerialで制御状態を出力（オプション）
    // Serial.printf("[Servo] Roll: %.2f->%.2f, cmd: %.2f | Pitch: %.2f->%.2f, cmd: %.2f\n",
    //              current_roll, target_roll, servo_cmd_roll,
    //              current_pitch, target_pitch, servo_cmd_pitch);
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
  Wire.begin(17, 16);  // SDA=17, SCL=16
  Wire.setClock(400000);

  bool status = bme.begin(0x76);
  if (!status) {
      Serial.println("!!! WARNING: BME280 init failed on Core 0! Using default values.");
      bmeAvailable = false;
  } else {
      Serial.println("BME280 Initialized.");
      bmeAvailable = true;
  }

  if (bmi270setup()) {
    bmi270Available = true;
    Serial.println("BMI270/BMM150 initialization successful.");
  } else {
    bmi270Available = false;
    Serial.println("!!! WARNING: BMI270 init failed! Using zero values for IMU data.");
  }
  
  Serial.println("All I2C sensors initialization completed.");


  LogEntry entry;
  imu_data_t imu;  // ライブラリからの加速度・ジャイロデータ用
  const TickType_t xFrequency = pdMS_TO_TICKS(5);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    entry.timestamp = millis();
    while (ss.available() > 0) { gps.encode(ss.read()); }
    entry.gps_updated = gps.location.isUpdated() ? 1 : 0;

    // --- センサーデータ読み取り ---
    if (bmeAvailable) {
      entry.temp = bme.readTemperature();
      entry.pres = bme.readPressure() / 100.0F;
      entry.alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
      entry.hum = bme.readHumidity();
    } else {
      // BME280が利用できない場合は定数値を使用
      entry.temp = 25.0;   // 温度 25℃
      entry.pres = 1013.0; // 気圧 1013 hPa
      entry.alt = 100.0;   // 高度 100m
      entry.hum = 50.0;    // 湿度 50%
    }
    
    if (bmi270Available) {
      // ★★★ ライブラリを使用してデータ取得 ★★★
      imuSensor.readGyroAccel(imu, true);  // true = 生データを取得
      
      // 加速度 [g] から [m/s^2] に変換
      entry.ax = (imu.acc.x * ACC_SCALE_2G) * G_TO_MS2;
      entry.ay = (imu.acc.y * ACC_SCALE_2G) * G_TO_MS2;
      entry.az = (imu.acc.z * ACC_SCALE_2G) * G_TO_MS2;
      
      // ジャイロ [deg/s]
      entry.gx = imu.gyr.x * GYRO_SCALE_2000DPS;
      entry.gy = imu.gyr.y * GYRO_SCALE_2000DPS;
      entry.gz = imu.gyr.z * GYRO_SCALE_2000DPS;
      
      // 地磁気センサを無視するため0固定
      entry.cx = 0;
      entry.cy = 0;
      entry.cz = 0;
    } else {
      // BMI270が利用できない場合は0を使用
      entry.ax = 0.0;
      entry.ay = 0.0;
      entry.az = 0.0;
      entry.gx = 0.0;
      entry.gy = 0.0;
      entry.gz = 0.0;
      entry.cx = 0;
      entry.cy = 0;
      entry.cz = 0;
    }
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
    
    // --- クォータニオンからオイラー角を計算 ---
    quaternionToEuler(entry.q0, entry.q1, entry.q2, entry.q3, entry.roll, entry.pitch, entry.yaw);

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
                  "Quat:%.4f,%.4f,%.4f,%.4f Euler:%.2f,%.2f,%.2f upd:%d\n",
                  e.timestamp,
                  e.temp, e.pres, e.alt, e.hum,
                  e.ax, e.ay, e.az,
                  e.gx, e.gy, e.gz,
                  e.cx, e.cy, e.cz,
                  e.lat / 1e6, e.lng / 1e6, e.gps_alt, e.sats,
                  e.date_year, e.date_month, e.date_day,
                  e.time_hour, e.time_min, e.time_sec, e.time_cs,
                  e.q0, e.q1, e.q2, e.q3,
                  e.roll, e.pitch, e.yaw,
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
    // I2Cエラーメッセージを抑制
    esp_log_level_set("*", ESP_LOG_WARN); // 警告レベル以上のみ表示
    
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

    // --- LEDC PWM 初期化（サーボ制御用） ---
    Serial.println("Initializing LEDC PWM for servo control...");
    ledcSetup(SERVO_PWM_CHANNEL_1, SERVO_PWM_FREQ, SERVO_PWM_BITS);
    ledcSetup(SERVO_PWM_CHANNEL_2, SERVO_PWM_FREQ, SERVO_PWM_BITS);
    ledcAttachPin(SERVO_PIN_1, SERVO_PWM_CHANNEL_1);
    ledcAttachPin(SERVO_PIN_2, SERVO_PWM_CHANNEL_2);
    // サーボを中央値で初期化
    setServoUS(SERVO_PWM_CHANNEL_1, 0.0f);
    setServoUS(SERVO_PWM_CHANNEL_2, 0.0f);
    Serial.println("LEDC PWM initialized.");

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
    xTaskCreatePinnedToCore(servoControlTask,"ServoControl",   2048, NULL, 2, NULL,             1);  // サーボ制御タスク
    xTaskCreatePinnedToCore(eulerTask,       "EulerTask",      4096, NULL, 1, &hEulerTask,      0);
    xTaskCreatePinnedToCore(sensorTask,      "SensorTask",     4096, NULL, 2, &hSensorTask,     0);
}

//================================================================
// ★★★ loop() (Core 1, Prio 1) ★★★
//================================================================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(100));
}