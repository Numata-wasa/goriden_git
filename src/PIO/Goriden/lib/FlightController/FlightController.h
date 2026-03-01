/**
 * @file FlightController.h
 * @brief ロケット飛行制御システム - モジュール化バージョン
 * @version 2.0.0
 * @date 2026-03-01
 * 
 * @description
 * ESP32を用いたロケット飛行制御システムのコアライブラリ。
 * 以下の機能を提供：
 * - IMU（BMI270）センサー読み取りと初期化
 * - クォータニオンベースの姿勢推定（相補フィルタ）
 * - 加速度積分による高度推定（ZUPT補正付き）
 * - PID制御による姿勢制御
 * - 目標クォータニオンへの追従制御
 */

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include "BMI2_BMM1.h"

// ==================== 定数定義 ====================

// 重力加速度
const float G_TO_MS2 = 9.80665f;  // [m/s^2]

// IMU スケール係数
const float ACC_SCALE_16G = (1.0f / 2048.0f);        // ±16g設定時 [g]
const float GYRO_SCALE_2000DPS = (1.0f / 16.4f);     // ±2000dps設定時 [deg/s]

// 相補フィルタパラメータ
const float ALPHA_MIN = 0.98f;   // 通常時のジャイロ信頼度
const float ALPHA_MAX = 1.00f;   // 高加速度時（加速度補正無効化）

// 加速度信頼度パラメータ
const float ACC_REJECT_LOW = 0.70f;   // 加速度補正を切る下限 [g倍]
const float ACC_REJECT_HIGH = 1.30f;  // 加速度補正を切る上限 [g倍]
const float GYRO_REJECT_DPS = 220.0f; // 高角速度時の閾値 [deg/s]
const float TRUST_RISE = 0.08f;       // 信頼度上昇速度
const float TRUST_FALL = 0.30f;       // 信頼度低下速度（早めに遮断）
const float ACCEL_TRUST_MIN = 0.15f;  // 信頼度下限（ドリフト対策）

// 高度推定パラメータ
const float LAUNCH_DETECT_ACC = 15.0f;           // 発射判定閾値 [m/s^2]
const float VERTICAL_BIAS_ALPHA = 0.01f;         // バイアス学習率
const float PRELAUNCH_STATIC_ACC_TOL = 2.0f;    // 静止判定：加速度許容 [m/s^2]
const float PRELAUNCH_STATIC_GYRO_TOL = 30.0f;  // 静止判定：角速度許容 [deg/s]
const float ZUPT_VEL_DAMP = 0.85f;               // ZUPT速度減衰係数
const float ZUPT_VEL_EPS = 0.08f;                // ZUPT速度閾値 [m/s]

// PID制御パラメータ（デフォルト値）
const float DEFAULT_KP_ROLL = 1.0f;
const float DEFAULT_KD_ROLL = 0.32f;
const float DEFAULT_KP_PITCH = 1.2f;
const float DEFAULT_KD_PITCH = 0.4f;
const float ERROR_DEADBAND = 0.03f;      // 小誤差デッドバンド
const float DTERM_DT_MAX = 0.12f;        // D項無効化の最大dt [s]

// ==================== 構造体定義 ====================

/**
 * @brief IMUデータ構造体
 */
struct IMUData {
  float ax, ay, az;        // 加速度 [m/s^2]
  float gx, gy, gz;        // 角速度 [deg/s]
  int16_t cx, cy, cz;      // 地磁気 [生値]
  uint32_t timestamp;      // タイムスタンプ [ms]
};

/**
 * @brief クォータニオン構造体
 */
struct Quaternion {
  float w, x, y, z;        // スカラー部w, ベクトル部(x,y,z)
  
  Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
  Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
  
  // 正規化
  void normalize();
  
  // NaNチェックとリセット
  void sanitize();
};

/**
 * @brief オイラー角構造体
 */
struct EulerAngles {
  float roll, pitch, yaw;  // [rad]
  
  EulerAngles() : roll(0.0f), pitch(0.0f), yaw(0.0f) {}
};

/**
 * @brief 高度推定状態
 */
struct AltitudeState {
  float altitude;           // 積分高度 [m]
  float velocity;           // 鉛直速度 [m/s]
  float acceleration;       // 重力補正後加速度 [m/s^2]
  float bias;               // 鉛直軸バイアス [m/s^2]
  bool launchDetected;      // 発射検知フラグ
  float launchAltitude;     // 発射時高度 [m]
  bool biasInitialized;     // バイアス初期化フラグ
  uint32_t lastTime;        // 前回タイムスタンプ [ms]
  
  AltitudeState() : altitude(0.0f), velocity(0.0f), acceleration(0.0f),
                    bias(G_TO_MS2), launchDetected(false), launchAltitude(0.0f),
                    biasInitialized(false), lastTime(0) {}
  
  // 相対高度を取得（発射時=0m）
  float getRelativeAltitude() const {
    if (!launchDetected) return 0.0f;
    return fmaxf(0.0f, altitude - launchAltitude);
  }
};

/**
 * @brief PIDコントローラ構造体
 */
struct PIDController {
  float kp, ki, kd;         // ゲイン
  float errorIntegral;      // 積分項
  float prevError;          // 前回誤差
  
  PIDController() : kp(1.0f), ki(0.0f), kd(0.0f), errorIntegral(0.0f), prevError(0.0f) {}
  PIDController(float kp_, float ki_, float kd_) : kp(kp_), ki(ki_), kd(kd_), errorIntegral(0.0f), prevError(0.0f) {}
  
  // PID計算
  float compute(float error, float dt, bool disableDterm = false);
  
  // リセット
  void reset() { errorIntegral = 0.0f; prevError = 0.0f; }
};

// ==================== クラス定義 ====================

/**
 * @brief 姿勢推定クラス（相補フィルタ）
 */
class AttitudeEstimator {
public:
  AttitudeEstimator();
  
  /**
   * @brief IMUデータから姿勢を更新
   * @param imu IMUデータ
   * @return 更新されたクォータニオン
   */
  Quaternion update(const IMUData& imu);
  
  /**
   * @brief 現在のクォータニオンを取得
   */
  Quaternion getQuaternion() const { return quaternion_; }
  
  /**
   * @brief 現在のオイラー角を取得
   */
  EulerAngles getEulerAngles() const;
  
  /**
   * @brief 加速度信頼度を取得
   */
  float getAccelTrust() const { return accelTrust_; }
  
  /**
   * @brief リセット
   */
  void reset();

private:
  Quaternion quaternion_;
  float accelTrust_;
  uint32_t lastTime_;
  bool initialized_;
  
  // ヘルパー関数
  Quaternion accelToQuaternion(float ax, float ay, float az);
  void updateFromGyro(float gx, float gy, float gz, float dt);
};

/**
 * @brief 高度推定クラス（加速度積分 + ZUPT）
 */
class AltitudeEstimator {
public:
  AltitudeEstimator();
  
  /**
   * @brief 高度を更新
   * @param imu IMUデータ（機体座標系）
   * @param attitude 姿勢クォータニオン
   * @return 更新された高度状態
   */
  AltitudeState update(const IMUData& imu, const Quaternion& attitude);
  
  /**
   * @brief 現在の高度状態を取得
   */
  AltitudeState getState() const { return state_; }
  
  /**
   * @brief リセット
   */
  void reset();

private:
  AltitudeState state_;
  
  // 機体座標系→地上座標系の変換（鉛直成分抽出）
  float transformToWorldZ(float ax, float ay, float az, const Quaternion& q);
  
  // 静止状態判定
  bool isStatic(const IMUData& imu);
};

/**
 * @brief 姿勢制御クラス（クォータニオンベースPID）
 */
class AttitudeController {
public:
  AttitudeController();
  
  /**
   * @brief 目標クォータニオンを設定
   */
  void setTargetQuaternion(const Quaternion& target) { targetQuat_ = target; }
  
  /**
   * @brief 制御出力を計算
   * @param current 現在のクォータニオン
   * @param dt サンプリング周期 [s]
   * @param rollOut ロール軸制御出力 [-1, 1]
   * @param pitchOut ピッチ軸制御出力 [-1, 1]
   */
  void compute(const Quaternion& current, float dt, float& rollOut, float& pitchOut);
  
  /**
   * @brief PIDゲインを設定
   */
  void setRollGains(float kp, float ki, float kd) { pidRoll_ = PIDController(kp, ki, kd); }
  void setPitchGains(float kp, float ki, float kd) { pidPitch_ = PIDController(kp, ki, kd); }
  
  /**
   * @brief 誤差クォータニオンを取得（デバッグ用）
   */
  Quaternion getErrorQuaternion() const { return errorQuat_; }
  
  /**
   * @brief リセット
   */
  void reset();

private:
  Quaternion targetQuat_;
  Quaternion errorQuat_;
  PIDController pidRoll_;
  PIDController pidPitch_;
  
  // 誤差クォータニオン計算: q_error = q_target^-1 * q_current
  Quaternion computeErrorQuaternion(const Quaternion& target, const Quaternion& current);
  
  // 角度誤差正規化 [-pi, pi]
  float wrapAngleError(float error);
};

/**
 * @brief 統合飛行制御クラス
 */
class FlightController {
public:
  FlightController();
  
  /**
   * @brief 初期化（IMUセンサー含む）
   * @param imuSensor BMI270センサーのポインタ
   * @return 初期化成功: true
   */
  bool begin(BMI2_BMM1_Class* imuSensor);
  
  /**
   * @brief 制御ループ更新（メインループで呼び出し）
   * @return 制御出力が有効: true
   */
  bool update();
  
  /**
   * @brief 現在の姿勢を取得
   */
  Quaternion getQuaternion() const { return attitudeEstimator_.getQuaternion(); }
  EulerAngles getEulerAngles() const { return attitudeEstimator_.getEulerAngles(); }
  
  /**
   * @brief 現在の高度状態を取得
   */
  AltitudeState getAltitudeState() const { return altitudeEstimator_.getState(); }
  
  /**
   * @brief 制御出力を取得
   * @param roll ロール軸出力 [-1, 1]
   * @param pitch ピッチ軸出力 [-1, 1]
   */
  void getControlOutput(float& roll, float& pitch) const;
  
  /**
   * @brief 目標クォータニオンを設定
   */
  void setTargetQuaternion(const Quaternion& target) { attitudeController_.setTargetQuaternion(target); }
  
  /**
   * @brief PIDゲインを調整
   */
  void setRollGains(float kp, float ki, float kd) { attitudeController_.setRollGains(kp, ki, kd); }
  void setPitchGains(float kp, float ki, float kd) { attitudeController_.setPitchGains(kp, ki, kd); }
  
  /**
   * @brief システムリセット
   */
  void reset();

private:
  BMI2_BMM1_Class* imuSensor_;
  AttitudeEstimator attitudeEstimator_;
  AltitudeEstimator altitudeEstimator_;
  AttitudeController attitudeController_;
  
  float rollOutput_;
  float pitchOutput_;
  
  // IMUデータ読み取り
  bool readIMU(IMUData& imu);
};

// ==================== ユーティリティ関数 ====================

/**
 * @brief クォータニオン→オイラー角変換
 */
EulerAngles quaternionToEuler(const Quaternion& q);

/**
 * @brief 加速度→クォータニオン変換（ヨー=0仮定）
 */
Quaternion accelToQuaternion(float ax, float ay, float az);

/**
 * @brief クォータニオン乗算
 */
Quaternion quaternionMultiply(const Quaternion& q1, const Quaternion& q2);

/**
 * @brief クォータニオン共役（逆回転）
 */
Quaternion quaternionConjugate(const Quaternion& q);

#endif // FLIGHT_CONTROLLER_H
