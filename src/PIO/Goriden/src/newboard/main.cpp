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

// ===== サーボ制御 (Raw LEDC PWM) =====
#define SERVO_PIN_1 4         // サーボ1のピン
#define SERVO_PIN_2 5         // サーボ2のピン
#define SERVO_UPDATE_FREQ 50  // サーボ更新頻度 [Hz]

// LEDC PWM 設定
#define SERVO1_LEDC_CH 2      // LEDCチャネル2
#define SERVO2_LEDC_CH 3      // LEDCチャネル3
#define SERVO_FREQ     50     // サーボPWM周波数 50Hz
#define SERVO_RES_BITS 14     // 14bit分解能 (0-16383)
#define SERVO_MIN_US   500    // 0°のパルス幅 [us]
#define SERVO_MAX_US   2400   // 180°のパルス幅 [us]

const int SERVO_NEUTRAL_ANGLE = 90;      // 中立角 [deg]
const int SERVO_MAX_DEFLECTION_DEG = 7;  // 最大舵角 ±[deg]
const int SERVO1_DIR = -1;               // サーボ1の正負（逆なら -1）
const int SERVO2_DIR = +1;               // サーボ2の正負（逆なら -1）

// 角度 -> LEDC duty変換ヘルパー
void servoWriteAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  // 14bit: 16384 steps, 50Hz = 20000us period
  // duty = pulseUs * 16384 / 20000
  uint32_t pulseUs = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  uint32_t duty = (pulseUs * 16384UL) / 20000UL;
  ledcWrite(channel, duty);
}

// ===== 姿勢制御 目標クォータニオン =====
// 目標: オイラー角 pitch=-π/2, roll=0, yaw=0
// クォータニオン: q=(cos(-π/4), 0, sin(-π/4), 0)
const float q0_target = 0.7071f;
const float q1_target = 0.0000f;
const float q2_target = -0.7071f;
const float q3_target = 0.0000f;

const float Kp_roll  = 1.3f;   // ロール比例ゲイン
const float Ki_roll  = 0.0f;   // ロール積分ゲイン (P制御のみなので0)
const float Kd_roll  = 0.35f;  // ロール微分ゲイン

const float Kp_pitch = 1.5f;   // ピッチ比例ゲイン
const float Ki_pitch = 0.0f;   // ピッチ積分ゲイン
const float Kd_pitch = 0.45f;  // ピッチ微分ゲイン

const float ERROR_DEADBAND = 0.01f;      // 小誤差デッドバンド
const float SERVO_MAX_STEP_DEG = 15.0f;  // 1周期での最大角度変化 [deg]
const float DTERM_DT_MAX = 0.12f;        // これ以上のdtではD項を無効化 [s]
const float ACCEL_TRUST_MIN = 0.15f;     // accelTrust下限（ドリフト対策）
const float LAUNCH_DETECT_ACC = 35.0f;   // 発射判定の加速度閾値 [m/s^2]（≈4g）

const float VERTICAL_BIAS_ALPHA = 0.01f; // 待機中の鉛直軸バイアス学習率
const float PRELAUNCH_STATIC_ACC_TOL = 2.0f; // 待機判定の|accNorm-1g|許容[m/s^2]
const float PRELAUNCH_STATIC_GYRO_TOL = 30.0f; // 待機判定の角速度許容[deg/s]
const float ZUPT_VEL_DAMP = 0.85f; // 静止判定時の速度減衰係数
const float ZUPT_VEL_EPS = 0.08f;  // 静止判定時に0へ丸める速度閾値 [m/s]

// PID制御用構造体
struct PIDController {
  float error_integral;  // 積分項
  float prev_error;      // 前フレームの誤差
};

volatile float servo_cmd_roll = 0.0f;   // ロール制御コマンド [-1, 1]
volatile float servo_cmd_pitch = 0.0f;  // ピッチ制御コマンド [-1, 1]
PIDController pid_roll, pid_pitch;      // PIDコントローラ
volatile float gy_ref = 0.0f;           // 初期キャリブレーション時の gy_body 値
volatile bool servo_calibrated = false; // キャリブレーション完了フラグ
volatile float servo1_shared = 90.0f;   // サーボ1の角度（sdWriteTaskで読み取り用）
volatile float servo2_shared = 90.0f;   // サーボ2の角度（sdWriteTaskで読み取り用）
// ==============

// ===== BMI270 + BMM150 (Library) =====
BMI2_BMM1_Class imuSensor;  // BMI2_BMM1ライブラリのインスタンス（名前を変更）

// スケール係数（ライブラリから得られる生データを物理値に変換）
#define ACC_SCALE_16G (1.0f / 2048.0f)        // ±16g設定時の加速度スケール [g]
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
  float servo1_angle, servo2_angle; // サーボ角度 [deg]
  float q_err_x, q_err_y, q_err_z; // 誤差クォータニオン要素
  float accelTrust_value; // 加速度信頼度 [0.0-1.0]
  uint8_t control_enabled; // 制御有効フラグ
  float integrated_altitude; // 積分高度 [m]
  float integrated_vz; // 鉛直軸速度 [m/s]（現設定: x軸）
  float az_freefall; // 鉛直軸加速度（重力補正後）[m/s^2]（現設定: x軸）
  uint8_t launch_detected; // 発射検知フラグ
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

// ★★★ ログ用：誤差Q値と accelTrust 値 ★★★
volatile float log_q_err_x = 0.0f;
volatile float log_q_err_y = 0.0f;
volatile float log_q_err_z = 0.0f;
volatile float log_accelTrust = 0.0f;
// =============""

// ★★★ 発射・制御フラグ ★★★
volatile bool launchDetected = false;  // 発射を検知したか
volatile bool controlEnabled = false;  // 制御を有効にするか（発射検知から2秒後）
volatile float launchAltitude = 0.0f; // 発射時の高度
volatile uint32_t launchDetectionTime = 0; // 発射検知時刻 [ms]

// ★★★ 加速度積分による高度計算 ★★★
volatile float integrated_vz = 0.0f;      // 鉛直軸速度 [m/s]（現設定: x軸）
volatile float integrated_altitude = 0.0f; // 統合高度 [m]
volatile uint32_t last_integration_time = 0; // 前フレームのタイムスタンプ [ms]
volatile float last_az_freefall = 0.0f;   // 鉛直軸加速度（重力補正後）[m/s^2]（現設定: x軸）
volatile float vertical_acc_bias = G_TO_MS2; // 鉛直軸の重力・オフセット推定値（地上座標Z軸）
volatile bool vertical_bias_initialized = false;
// =============="
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

//================================================================
// ★★★ BMI270 初期化関数（ライブラリ使用） ★★★
//================================================================
bool bmi270setup(){
  Serial.println("Initializing BMI270+BMM150 with Library...");
  Serial.println("[IMU] Calling imuSensor.begin()...");
  
  if (!imuSensor.begin(&Serial)) {
    Serial.println("!!! ERROR: IMU initialization failed!");
    Serial.println("[IMU] Check: I2C connection, sensor address, Wire setup");
    return false;
  }
  
  Serial.println("[IMU] SUCCESS: BMI270+BMM150 initialized.");
  
  // テスト読み込み
  Serial.println("[IMU] Performing test read (5 times with separate reads)...");
  for (int i = 0; i < 5; i++) {
    struct bmi2_sens_data test_imu;
    int testResult = imuSensor.readGyroAccel(test_imu, true);
    Serial.printf("[Test %d-combined] result=%d | acc=(%6d,%6d,%6d) gyr=(%6d,%6d,%6d)\n",
      i, testResult,
      test_imu.acc.x, test_imu.acc.y, test_imu.acc.z,
      test_imu.gyr.x, test_imu.gyr.y, test_imu.gyr.z);
    
    // 個別読み込みのテスト
    float ax, ay, az;
    float gx, gy, gz;
    imuSensor.readAcceleration(ax, ay, az);
    imuSensor.readGyroscope(gx, gy, gz);
    Serial.printf("[Test %d-separate] acc_float=(%.4f,%.4f,%.4f) gyr_float=(%.4f,%.4f,%.4f)\n",
      i, ax, ay, az, gx, gy, gz);
    
    delay(100);
  }
  
  return true;
}


//================================================================
// ★ Task D (Core 0, Prio 1): クォータニオン計算 (相補フィルタ)
//================================================================
// ヘルパー関数: 加速度からクォータニオンを算出
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
  const float alphaMin = 0.98f;   // 通常時の相補フィルタ係数 (ジャイロ信頼度)
  const float alphaMax = 1.00f;   // 危険区間では加速度補正を無効化
  const float gravity = 9.80665f; // [m/s^2]
  const float accRejectLow = 0.70f * gravity;   // 慣性飛行(0g近傍)などで補正を切る下限
  const float accRejectHigh = 1.30f * gravity;  // 推力加速などで補正を切る上限
  const float gyroRejectDps = 220.0f;           // 高角速度時は加速度補正を切る
  const float trustRise = 0.08f;   // 信頼度を上げる速度
  const float trustFall = 0.30f;   // 信頼度を下げる速度(早めに遮断)
  uint32_t lastTime = 0;
  bool initialized = false;
  float accelTrust = 1.0f; // 0.0=加速度補正なし(gyroのみ), 1.0=通常補正
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

        float accNorm = sqrtf(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);
        float gyroAbsMax = fmaxf(fabsf(imu.gx), fmaxf(fabsf(imu.gy), fabsf(imu.gz)));

        // ロケット推力中(>1g)・慣性飛行中(<1g, 特に0g近傍)・高角速度時は加速度補正を弱める
        bool accelReliable = (accNorm >= accRejectLow) && (accNorm <= accRejectHigh) && (gyroAbsMax <= gyroRejectDps);
        float targetTrust = accelReliable ? 1.0f : 0.0f;

        // 急な切替で姿勢が跳ねないよう一次遅れで平滑化
        if (targetTrust > accelTrust) {
          accelTrust += trustRise;
        } else {
          accelTrust -= trustFall;
        }
        accelTrust = constrain(accelTrust, ACCEL_TRUST_MIN, 1.0f);  // 下限0.15でドリフト対策
        
        // ログ用に accelTrust を保存
        log_accelTrust = accelTrust;

        float alpha = alphaMax - (alphaMax - alphaMin) * accelTrust;

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
// ★ Task F (Core 1, Prio 2): サーボ姿勢制御タスク (PID)
//================================================================
void servoControlTask(void *pvParameters) {
  Serial.println("Servo Control Task started on Core 1 (Prio 2)");
  
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / SERVO_UPDATE_FREQ);  // 50ms (20Hz)
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t prevControlMs = 0;
  float servo1_cmd = (float)SERVO_NEUTRAL_ANGLE;
  float servo2_cmd = (float)SERVO_NEUTRAL_ANGLE;
  
  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    uint32_t nowMs = millis();
    float dtControl = (prevControlMs == 0) ? (1.0f / SERVO_UPDATE_FREQ) : ((nowMs - prevControlMs) / 1000.0f);
    prevControlMs = nowMs;
    bool disableDterm = (dtControl <= 0.0f) || (dtControl > DTERM_DT_MAX);
    
    // 共有クォータニオンからオイラー角を計算
    float q0 = sharedQ0, q1 = sharedQ1, q2 = sharedQ2, q3 = sharedQ3;
    
    // クォータニオンの妥当性チェック
    float qnorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (isnan(qnorm) || qnorm < 0.1f) {
      // 異常値 -> サーボを中央に固定
      servoWriteAngle(SERVO1_LEDC_CH, SERVO_NEUTRAL_ANGLE);
      servoWriteAngle(SERVO2_LEDC_CH, SERVO_NEUTRAL_ANGLE);
      continue;
    }
    
    // ===== クォータニオン差分制御（ジンバルロック回避） =====
    // 誤差クォータニオン = q_target^-1 * q_current
    // q_target^-1 = (q0_target, -q1_target, -q2_target, -q3_target)
    float q_err_w = q0_target*q0 + q1_target*q1 + q2_target*q2 + q3_target*q3;
    float q_err_x = -q0_target*q1 + q1_target*q0 - q2_target*q3 + q3_target*q2;
    float q_err_y = -q0_target*q2 + q1_target*q3 + q2_target*q0 - q3_target*q1;
    float q_err_z = -q0_target*q3 - q1_target*q2 + q2_target*q1 + q3_target*q0;
    
    // 誤差クォータニオンの正規化
    float q_err_norm = sqrtf(q_err_w*q_err_w + q_err_x*q_err_x + q_err_y*q_err_y + q_err_z*q_err_z);
    if (q_err_norm > 0.01f) {
      q_err_w /= q_err_norm;
      q_err_x /= q_err_norm;
      q_err_y /= q_err_norm;
      q_err_z /= q_err_norm;
    }
    
    // ログ用に誤差Q値を保存
    log_q_err_x = q_err_x;
    log_q_err_y = q_err_y;
    log_q_err_z = q_err_z;
    
    // 回転軸（機体座標系）: (q_err_x, q_err_y, q_err_z) / sin(angle/2)
    // 簡略化: 小さなアングル近似で誤差軸ベクトルを直接使用
    float error_x = 2.0f * q_err_x;  // x軸周りの回転誤差（roll）
    float error_y = 2.0f * q_err_y;  // y軸周りの回転誤差（pitch）

    // 小誤差デッドバンド（微振動抑制）
    if (fabsf(error_x) < ERROR_DEADBAND) error_x = 0.0f;
    if (fabsf(error_y) < ERROR_DEADBAND) error_y = 0.0f;
    
    // PD制御
    // ロール軸 (x軸)
    float d_error_x = disableDterm ? 0.0f : (error_x - pid_roll.prev_error);
    float output_x = Kp_roll * error_x + Kd_roll * d_error_x;
    output_x = fmaxf(-1.0f, fminf(1.0f, output_x));
    pid_roll.prev_error = error_x;
    
    // ピッチ軸 (y軸)
    float d_error_y = disableDterm ? 0.0f : (error_y - pid_pitch.prev_error);
    float output_y = Kp_pitch * error_y + Kd_pitch * d_error_y;
    output_y = fmaxf(-1.0f, fminf(1.0f, output_y));
    pid_pitch.prev_error = error_y;
    
    // サーボ目標角度（各軸1サーボ、符号は SERVOx_DIR で切替）
    float servo1_target = (float)SERVO_NEUTRAL_ANGLE + (float)SERVO1_DIR * ((float)SERVO_MAX_DEFLECTION_DEG * output_x);
    float servo2_target = (float)SERVO_NEUTRAL_ANGLE + (float)SERVO2_DIR * ((float)SERVO_MAX_DEFLECTION_DEG * output_y);
    
    // 発射検知から1.5秒以上経過で制御開始
    uint32_t currentTime = millis();
    uint32_t timeSinceLaunch = launchDetected ? (currentTime - launchDetectionTime) : 0;
    const uint32_t CONTROL_DELAY_MS = 1500; // 2秒の遅延
    
    if (timeSinceLaunch < CONTROL_DELAY_MS) {
      servo1_target = (float)SERVO_NEUTRAL_ANGLE;
      servo2_target = (float)SERVO_NEUTRAL_ANGLE;
      controlEnabled = false;
    } else {
      controlEnabled = true;
    }

    // スルーレート制限（急激な角度変化を抑制）
    float delta1 = servo1_target - servo1_cmd;
    if (delta1 > SERVO_MAX_STEP_DEG) delta1 = SERVO_MAX_STEP_DEG;
    if (delta1 < -SERVO_MAX_STEP_DEG) delta1 = -SERVO_MAX_STEP_DEG;
    servo1_cmd += delta1;

    float delta2 = servo2_target - servo2_cmd;
    if (delta2 > SERVO_MAX_STEP_DEG) delta2 = SERVO_MAX_STEP_DEG;
    if (delta2 < -SERVO_MAX_STEP_DEG) delta2 = -SERVO_MAX_STEP_DEG;
    servo2_cmd += delta2;

    // サーボ角度（最大舵角に制限）
    int servo1_angle = (int)servo1_cmd;
    int servo2_angle = (int)servo2_cmd;
    const int SERVO_MAX_ANGLE = SERVO_NEUTRAL_ANGLE + SERVO_MAX_DEFLECTION_DEG;
    const int SERVO_MIN_ANGLE = SERVO_NEUTRAL_ANGLE - SERVO_MAX_DEFLECTION_DEG;
    servo1_angle = constrain(servo1_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    servo2_angle = constrain(servo2_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    
    servoWriteAngle(SERVO1_LEDC_CH, servo1_angle);
    servoWriteAngle(SERVO2_LEDC_CH, servo2_angle);
    
    // サーボ角度を共有変数に保存（sdWriteTaskで読み取り）
    servo1_shared = servo1_angle;
    servo2_shared = servo2_angle;
    
    // デバッグ出力フルバージョン (1Hz, 誤差クォータニオン)
    static int debugCount = 0;
    if (++debugCount >= 20) {
      debugCount = 0;
      Serial.print("[QErr] curr=(");
      Serial.print(q0, 3); Serial.print(",");
      Serial.print(q1, 3); Serial.print(",");
      Serial.print(q2, 3); Serial.print(",");
      Serial.print(q3, 3); Serial.print(") ");
      Serial.print("| q_err=(");
      Serial.print(q_err_w, 3); Serial.print(",");
      Serial.print(q_err_x, 3); Serial.print(",");
      Serial.print(q_err_y, 3); Serial.print(",");
      Serial.print(q_err_z, 3); Serial.print(") ");
      Serial.print("| err_axis(x,y)=(");
      Serial.print(error_x, 3); Serial.print(",");
      Serial.print(error_y, 3); Serial.print(") ");
      Serial.print("| out(x,y)=(");
      Serial.print(output_x, 3); Serial.print(",");
      Serial.print(output_y, 3); Serial.print(") ");
      Serial.print("| tgt(s1,s2)=(");
      Serial.print(servo1_target, 1); Serial.print(",");
      Serial.print(servo2_target, 1); Serial.print(") ");
      Serial.print("| s1="); Serial.print(servo1_angle);
      Serial.print(" s2="); Serial.println(servo2_angle);
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
      entry.alt = 0.0;     // 高度は加速度積分で計算（後で上書き）
      entry.hum = 50.0;    // 湿度 50%
    }
    
    // --- GPSデータ格納（高度は使用しない） ---
    entry.lat = gps.location.isValid() ? (int32_t)(gps.location.lat() * 1e6) : 0;
    entry.lng = gps.location.isValid() ? (int32_t)(gps.location.lng() * 1e6) : 0;
    entry.gps_alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;  // ログには記録するが、制御には使わない
    entry.sats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
    if (gps.date.isValid()) {
      entry.date_year = gps.date.year(); entry.date_month = gps.date.month(); entry.date_day = gps.date.day();
    }
    if (gps.time.isValid()) {
      entry.time_hour = gps.time.hour(); entry.time_min = gps.time.minute(); entry.time_sec = gps.time.second(); entry.time_cs = gps.time.centisecond();
    }
    
    if (bmi270Available) {
      // ★★★ ライブラリを使用してデータ取得 ★★★
      int readResult = imuSensor.readGyroAccel(imu, true);  // true = 生データを取得
      
      if (readResult != 0) {
        Serial.printf("[IMU ERROR] readGyroAccel failed with code: %d\n", readResult);
      }
      
      // 加速度 [g] から [m/s^2] に変換
      entry.ax = (imu.acc.x * ACC_SCALE_16G) * G_TO_MS2;
      entry.ay = (imu.acc.y * ACC_SCALE_16G) * G_TO_MS2;
      entry.az = (imu.acc.z * ACC_SCALE_16G) * G_TO_MS2;
      
      // ジャイロ [deg/s]
      entry.gx = imu.gyr.x * GYRO_SCALE_2000DPS;
      entry.gy = imu.gyr.y * GYRO_SCALE_2000DPS;
      entry.gz = imu.gyr.z * GYRO_SCALE_2000DPS;
      
      // デバッグ: 詳細なデータをシリアルに出力（100フレームごと）
      static int imuDebugCount = 0;
      if (++imuDebugCount % 100 == 0) {
        Serial.printf("[IMU Data] raw_acc=(%6d,%6d,%6d) raw_gyr=(%6d,%6d,%6d)\n",
          imu.acc.x, imu.acc.y, imu.acc.z, imu.gyr.x, imu.gyr.y, imu.gyr.z);
        Serial.printf("[IMU Conv] ax=%.2f ay=%.2f az=%.2f | gx=%.2f gy=%.2f gz=%.2f\n",
          entry.ax, entry.ay, entry.az, entry.gx, entry.gy, entry.gz);
      }
      
      // 地磁気センサを無視するため0固定
      entry.cx = 0;
      entry.cy = 0;
      entry.cz = 0;
      
      // ★★★ 加速度積分による高度計算 ★★★
      // （GPS高度は使わない、加速度直積分のみ）
      if (last_integration_time == 0) {
        // 初回：タイムスタンプのみ記録
        last_integration_time = entry.timestamp;
        last_az_freefall = 0.0f;
      } else {
        float dt = (entry.timestamp - last_integration_time) / 1000.0f; // [s]
        if (dt > 0 && dt < 0.5f) {  // 異常な dt は無視
          // クォータニオンで機体加速度を地上座標へ回し、鉛直成分(Z)を取得
          float q0w = sharedQ0;
          float q1w = sharedQ1;
          float q2w = sharedQ2;
          float q3w = sharedQ3;
          float qnorm = sqrtf(q0w*q0w + q1w*q1w + q2w*q2w + q3w*q3w);
          if (qnorm > 1e-6f) {
            q0w /= qnorm;
            q1w /= qnorm;
            q2w /= qnorm;
            q3w /= qnorm;
          } else {
            q0w = 1.0f; q1w = 0.0f; q2w = 0.0f; q3w = 0.0f;
          }

          float acc_world_z =
              (2.0f * (q1w * q3w - q0w * q2w)) * entry.ax +
              (2.0f * (q2w * q3w + q0w * q1w)) * entry.ay +
              (1.0f - 2.0f * (q1w * q1w + q2w * q2w)) * entry.az;

          // 待機中は静止状態のときだけ鉛直軸バイアスを学習（地上座標Z軸）
          float accNorm = sqrtf(entry.ax * entry.ax + entry.ay * entry.ay + entry.az * entry.az);
          float gyroAbsMax = fmaxf(fabsf(entry.gx), fmaxf(fabsf(entry.gy), fabsf(entry.gz)));
          bool isStatic = (fabsf(accNorm - G_TO_MS2) < PRELAUNCH_STATIC_ACC_TOL) &&
                          (gyroAbsMax < PRELAUNCH_STATIC_GYRO_TOL);

          if (!launchDetected && isStatic) {
            if (!vertical_bias_initialized) {
              vertical_acc_bias = acc_world_z;
              vertical_bias_initialized = true;
            } else {
              vertical_acc_bias = (1.0f - VERTICAL_BIAS_ALPHA) * vertical_acc_bias + VERTICAL_BIAS_ALPHA * acc_world_z;
            }
          }

          // 鉛直軸（地上座標Z軸）加速度からバイアスを差し引く（上向きをプラス）
          float vertical_freefall = acc_world_z - vertical_acc_bias;
          last_az_freefall = vertical_freefall;  // ログ用に保存（フィールド名は互換維持）
          
          // 速度積分: v = v_prev + a * dt
          integrated_vz += vertical_freefall * dt;

          // 発射後に静止へ戻った区間では速度をゼロへ収束させる（ZUPT）
          if (launchDetected && isStatic) {
            integrated_vz *= ZUPT_VEL_DAMP;
            if (fabsf(integrated_vz) < ZUPT_VEL_EPS) {
              integrated_vz = 0.0f;
            }
          }
          
          // 高度積分: h = h_prev + v * dt + 0.5 * a * dt^2
          integrated_altitude += integrated_vz * dt + 0.5f * vertical_freefall * dt * dt;
        }
        last_integration_time = entry.timestamp;
      }
      
      // ★★★ 発射検知: 加速度ノルムが閾値を超えたら ★★★
      if (!launchDetected) {
        float accNorm = sqrtf(entry.ax*entry.ax + entry.ay*entry.ay + entry.az*entry.az);
        if (accNorm >= LAUNCH_DETECT_ACC) {
          launchDetected = true;
          launchDetectionTime = millis(); // 発射検知時刻を記録
          launchAltitude = integrated_altitude; // 発射時の積分高度を基準0にする
          integrated_vz = 0.0f; // 発射前ドリフトの速度成分をリセット
          Serial.println("[LAUNCH] Detected!");
        }
      }

      // 計算した高度をエントリに設定（BME280がない場合）
      // 地上待機中の積分ドリフト影響を避けるため、発射までは0固定
      // 発射後は「発射時=0m」の相対高度を使う
      if (!bmeAvailable) {
        float relativeAlt = launchDetected ? (integrated_altitude - launchAltitude) : 0.0f;
        entry.alt = fmaxf(0.0f, relativeAlt);
      }
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
    
    // --- サーボ角度を記録 ---
    entry.servo1_angle = servo1_shared;
    entry.servo2_angle = servo2_shared;
    
    // --- ログ用：誤差Q値と accelTrust、制御フラグを記録 ---
    entry.q_err_x = log_q_err_x;
    entry.q_err_y = log_q_err_y;
    entry.q_err_z = log_q_err_z;
    entry.accelTrust_value = log_accelTrust;
    entry.control_enabled = controlEnabled ? 1 : 0;
    
    // --- ログ用：積分高度、速度、加速度、発射フラグを記録 ---
    entry.integrated_altitude = integrated_altitude;
    entry.integrated_vz = integrated_vz;
    entry.az_freefall = last_az_freefall;
    entry.launch_detected = launchDetected ? 1 : 0;

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

    // --- LEDC PWM サーボ初期化 (SPI/SDの後に実行) ---
    Serial.println("Initializing LEDC Servo...");
    ledcSetup(SERVO1_LEDC_CH, SERVO_FREQ, SERVO_RES_BITS);
    ledcSetup(SERVO2_LEDC_CH, SERVO_FREQ, SERVO_RES_BITS);
    ledcAttachPin(SERVO_PIN_1, SERVO1_LEDC_CH);
    ledcAttachPin(SERVO_PIN_2, SERVO2_LEDC_CH);
    servoWriteAngle(SERVO1_LEDC_CH, SERVO_NEUTRAL_ANGLE);  // 中央
    servoWriteAngle(SERVO2_LEDC_CH, SERVO_NEUTRAL_ANGLE);  // 中央
    
    // サーボ初期化確認シーケンス（個別テスト）
    Serial.println("[Servo] Calibration sequence starting...");
    const int SERVO_MIN_ANGLE = SERVO_NEUTRAL_ANGLE - SERVO_MAX_DEFLECTION_DEG;
    const int SERVO_MAX_ANGLE = SERVO_NEUTRAL_ANGLE + SERVO_MAX_DEFLECTION_DEG;

    // サーボ1 単体テスト
    Serial.println("[Servo Test] Servo1 only");
    delay(500);
    servoWriteAngle(SERVO1_LEDC_CH, SERVO_MIN_ANGLE);  // 最小舵角
    servoWriteAngle(SERVO2_LEDC_CH, SERVO_NEUTRAL_ANGLE);
    delay(500);
    servoWriteAngle(SERVO1_LEDC_CH, SERVO_MAX_ANGLE); // 最大舵角
    servoWriteAngle(SERVO2_LEDC_CH, SERVO_NEUTRAL_ANGLE);
    delay(500);

    // サーボ2 単体テスト
    Serial.println("[Servo Test] Servo2 only");
    servoWriteAngle(SERVO1_LEDC_CH, SERVO_NEUTRAL_ANGLE);
    servoWriteAngle(SERVO2_LEDC_CH, SERVO_MIN_ANGLE);
    delay(500);
    servoWriteAngle(SERVO1_LEDC_CH, SERVO_NEUTRAL_ANGLE);
    servoWriteAngle(SERVO2_LEDC_CH, SERVO_MAX_ANGLE);
    delay(500);

    servoWriteAngle(SERVO1_LEDC_CH, SERVO_NEUTRAL_ANGLE);  // センターに戻す
    servoWriteAngle(SERVO2_LEDC_CH, SERVO_NEUTRAL_ANGLE);
    delay(500);
    Serial.println("[Servo] Calibration sequence complete.");
    
    Serial.println("LEDC Servo initialized.");
    Serial.println("[Servo] Waiting for calibration... Keep horizontal for 5 seconds!");

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

    // --- 全タスク起動 ---
    xTaskCreatePinnedToCore(sdWriteTask,      "SDWriteTask",    4096, NULL, 1, &hSdWriteTask,     1);
    xTaskCreatePinnedToCore(sdFlushTask,      "SDFlushTask",    2048, NULL, 0, &hSdFlushTask,     1);
    xTaskCreatePinnedToCore(serialPrintTask,  "SerialPrint",    4096, NULL, 0, &hSerialPrintTask, 1);
    xTaskCreatePinnedToCore(servoControlTask, "ServoControl",   4096, NULL, 2, NULL,              1);
    xTaskCreatePinnedToCore(eulerTask,        "EulerTask",      4096, NULL, 1, &hEulerTask,       0);
    xTaskCreatePinnedToCore(sensorTask,       "SensorTask",     4096, NULL, 2, &hSensorTask,      0);
}

//================================================================
// ★★★ loop() (Core 1, Prio 1) ★★★
//================================================================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(100));
}