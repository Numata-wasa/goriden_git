/**
 * @file FlightController.cpp
 * @brief ロケット飛行制御システム - 実装
 */

#include "FlightController.h"
#include <cmath>

// ==================== Quaternion メソッド ====================

void Quaternion::normalize() {
  float norm = sqrtf(w*w + x*x + y*y + z*z);
  if (norm > 0.0f) {
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
  } else {
    w = 1.0f; x = 0.0f; y = 0.0f; z = 0.0f;
  }
}

void Quaternion::sanitize() {
  if (isnan(w) || isnan(x) || isnan(y) || isnan(z)) {
    w = 1.0f; x = 0.0f; y = 0.0f; z = 0.0f;
  }
}

// ==================== PIDController メソッド ====================

float PIDController::compute(float error, float dt, bool disableDterm) {
  // 誤差を [-pi, pi] に正規化
  while (error > PI) error -= 2.0f * PI;
  while (error < -PI) error += 2.0f * PI;
  
  // 積分項更新
  errorIntegral += error * dt;
  errorIntegral = constrain(errorIntegral, -1.0f, 1.0f);
  
  // 微分項
  float derivative = 0.0f;
  if (!disableDterm && dt > 0.0f) {
    derivative = (error - prevError) / dt;
  }
  prevError = error;
  
  // PID出力
  float output = kp * error + ki * errorIntegral + kd * derivative;
  return constrain(output, -1.0f, 1.0f);
}

// ==================== ユーティリティ関数 ====================

EulerAngles quaternionToEuler(const Quaternion& q) {
  EulerAngles euler;
  
  // Roll (x軸周り)
  euler.roll = atan2f(2.0f * (q.w * q.x + q.y * q.z), 
                      1.0f - 2.0f * (q.x * q.x + q.y * q.y));
  
  // Pitch (y軸周り) - asinf引数をクランプ
  float pitchArg = 2.0f * (q.w * q.y - q.z * q.x);
  pitchArg = constrain(pitchArg, -1.0f, 1.0f);
  euler.pitch = asinf(pitchArg);
  
  // Yaw (z軸周り)
  euler.yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y), 
                     1.0f - 2.0f * (q.y * q.y + q.z * q.z));
  
  return euler;
}

Quaternion accelToQuaternion(float ax, float ay, float az) {
  float norm = sqrtf(ax*ax + ay*ay + az*az);
  if (norm == 0.0f) {
    return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
  }
  ax /= norm; ay /= norm; az /= norm;
  
  // ロール・ピッチを計算（ヨー=0）
  float roll = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
  
  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);
  
  return Quaternion(cr * cp, sr * cp, cr * sp, -sr * sp);
}

Quaternion quaternionMultiply(const Quaternion& q1, const Quaternion& q2) {
  return Quaternion(
    q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
    q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
    q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
    q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
  );
}

Quaternion quaternionConjugate(const Quaternion& q) {
  return Quaternion(q.w, -q.x, -q.y, -q.z);
}

// ==================== AttitudeEstimator ====================

AttitudeEstimator::AttitudeEstimator() 
  : quaternion_(1.0f, 0.0f, 0.0f, 0.0f),
    accelTrust_(1.0f),
    lastTime_(0),
    initialized_(false) {}

Quaternion AttitudeEstimator::update(const IMUData& imu) {
  if (!initialized_) {
    // 初回: 加速度からクォータニオンを初期化
    quaternion_ = accelToQuaternion(imu.ax, imu.ay, imu.az);
    lastTime_ = imu.timestamp;
    initialized_ = true;
    return quaternion_;
  }
  
  float dt = (imu.timestamp - lastTime_) / 1000.0f;  // [s]
  lastTime_ = imu.timestamp;
  
  if (dt <= 0.0f || dt > 0.5f) {
    return quaternion_;  // 異常なdtはスキップ
  }
  
  // 加速度ノルムと角速度最大値
  float accNorm = sqrtf(imu.ax*imu.ax + imu.ay*imu.ay + imu.az*imu.az);
  float gyroAbsMax = fmaxf(fabsf(imu.gx), fmaxf(fabsf(imu.gy), fabsf(imu.gz)));
  
  // 加速度信頼度判定
  float gravity = G_TO_MS2;
  bool accelReliable = (accNorm >= ACC_REJECT_LOW * gravity) && 
                       (accNorm <= ACC_REJECT_HIGH * gravity) &&
                       (gyroAbsMax <= GYRO_REJECT_DPS);
  
  float targetTrust = accelReliable ? 1.0f : 0.0f;
  
  // 平滑化
  if (targetTrust > accelTrust_) {
    accelTrust_ += TRUST_RISE;
  } else {
    accelTrust_ -= TRUST_FALL;
  }
  accelTrust_ = constrain(accelTrust_, ACCEL_TRUST_MIN, 1.0f);
  
  // 相補フィルタ係数
  float alpha = ALPHA_MAX - (ALPHA_MAX - ALPHA_MIN) * accelTrust_;
  
  // ジャイロでクォータニオン更新
  Quaternion qGyro = quaternion_;
  updateFromGyro(imu.gx, imu.gy, imu.gz, dt);
  qGyro = quaternion_;
  
  // 加速度からクォータニオン推定
  Quaternion qAcc = accelToQuaternion(imu.ax, imu.ay, imu.az);
  
  // 相補フィルタ
  quaternion_.w = alpha * qGyro.w + (1.0f - alpha) * qAcc.w;
  quaternion_.x = alpha * qGyro.x + (1.0f - alpha) * qAcc.x;
  quaternion_.y = alpha * qGyro.y + (1.0f - alpha) * qAcc.y;
  quaternion_.z = alpha * qGyro.z + (1.0f - alpha) * qAcc.z;
  
  quaternion_.normalize();
  quaternion_.sanitize();
  
  return quaternion_;
}

EulerAngles AttitudeEstimator::getEulerAngles() const {
  return quaternionToEuler(quaternion_);
}

void AttitudeEstimator::reset() {
  quaternion_ = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
  accelTrust_ = 1.0f;
  lastTime_ = 0;
  initialized_ = false;
}

Quaternion AttitudeEstimator::accelToQuaternion(float ax, float ay, float az) {
  return ::accelToQuaternion(ax, ay, az);
}

void AttitudeEstimator::updateFromGyro(float gx, float gy, float gz, float dt) {
  // deg->rad 変換
  float gxRad = gx * PI / 180.0f;
  float gyRad = gy * PI / 180.0f;
  float gzRad = gz * PI / 180.0f;
  
  // クォータニオン微分: dq = 0.5 * q * w
  float dw = 0.5f * (-quaternion_.x * gxRad - quaternion_.y * gyRad - quaternion_.z * gzRad) * dt;
  float dx = 0.5f * ( quaternion_.w * gxRad + quaternion_.y * gzRad - quaternion_.z * gyRad) * dt;
  float dy = 0.5f * ( quaternion_.w * gyRad - quaternion_.x * gzRad + quaternion_.z * gxRad) * dt;
  float dz = 0.5f * ( quaternion_.w * gzRad + quaternion_.x * gyRad - quaternion_.y * gxRad) * dt;
  
  quaternion_.w += dw;
  quaternion_.x += dx;
  quaternion_.y += dy;
  quaternion_.z += dz;
  
  quaternion_.normalize();
}

// ==================== AltitudeEstimator ====================

AltitudeEstimator::AltitudeEstimator() : state_() {}

AltitudeState AltitudeEstimator::update(const IMUData& imu, const Quaternion& attitude) {
  if (state_.lastTime == 0) {
    // 初回
    state_.lastTime = imu.timestamp;
    return state_;
  }
  
  float dt = (imu.timestamp - state_.lastTime) / 1000.0f;  // [s]
  state_.lastTime = imu.timestamp;
  
  if (dt <= 0.0f || dt > 0.5f) {
    return state_;  // 異常なdtはスキップ
  }
  
  // 機体座標系→地上座標系の鉛直成分を抽出
  float accWorldZ = transformToWorldZ(imu.ax, imu.ay, imu.az, attitude);
  
  // 静止判定
  bool isStaticNow = isStatic(imu);
  
  // 発射前の静止状態でバイアス学習
  if (!state_.launchDetected && isStaticNow) {
    if (!state_.biasInitialized) {
      state_.bias = accWorldZ;
      state_.biasInitialized = true;
    } else {
      state_.bias = (1.0f - VERTICAL_BIAS_ALPHA) * state_.bias + VERTICAL_BIAS_ALPHA * accWorldZ;
    }
  }
  
  // 重力補正
  float freefall = accWorldZ - state_.bias;
  state_.acceleration = freefall;
  
  // 速度積分
  state_.velocity += freefall * dt;
  
  // ZUPT: 発射後の静止時に速度を減衰
  if (state_.launchDetected && isStaticNow) {
    state_.velocity *= ZUPT_VEL_DAMP;
    if (fabsf(state_.velocity) < ZUPT_VEL_EPS) {
      state_.velocity = 0.0f;
    }
  }
  
  // 高度積分
  state_.altitude += state_.velocity * dt + 0.5f * freefall * dt * dt;
  
  // 発射検知
  if (!state_.launchDetected) {
    float accNorm = sqrtf(imu.ax*imu.ax + imu.ay*imu.ay + imu.az*imu.az);
    if (accNorm >= LAUNCH_DETECT_ACC) {
      state_.launchDetected = true;
      state_.launchAltitude = state_.altitude;
      state_.velocity = 0.0f;  // リセット
    }
  }
  
  return state_;
}

void AltitudeEstimator::reset() {
  state_ = AltitudeState();
}

float AltitudeEstimator::transformToWorldZ(float ax, float ay, float az, const Quaternion& q) {
  // クォータニオンで回転: R(q) * a
  // Z成分のみ: R_31*ax + R_32*ay + R_33*az
  return (2.0f * (q.x * q.z - q.w * q.y)) * ax +
         (2.0f * (q.y * q.z + q.w * q.x)) * ay +
         (1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * az;
}

bool AltitudeEstimator::isStatic(const IMUData& imu) {
  float accNorm = sqrtf(imu.ax*imu.ax + imu.ay*imu.ay + imu.az*imu.az);
  float gyroAbsMax = fmaxf(fabsf(imu.gx), fmaxf(fabsf(imu.gy), fabsf(imu.gz)));
  
  return (fabsf(accNorm - G_TO_MS2) < PRELAUNCH_STATIC_ACC_TOL) &&
         (gyroAbsMax < PRELAUNCH_STATIC_GYRO_TOL);
}

// ==================== AttitudeController ====================

AttitudeController::AttitudeController() 
  : targetQuat_(1.0f, 0.0f, 0.0f, 0.0f),
    errorQuat_(1.0f, 0.0f, 0.0f, 0.0f),
    pidRoll_(DEFAULT_KP_ROLL, 0.0f, DEFAULT_KD_ROLL),
    pidPitch_(DEFAULT_KP_PITCH, 0.0f, DEFAULT_KD_PITCH) {}

void AttitudeController::compute(const Quaternion& current, float dt, float& rollOut, float& pitchOut) {
  // 誤差クォータニオン計算
  errorQuat_ = computeErrorQuaternion(targetQuat_, current);
  
  // 回転軸誤差（小角度近似）
  float errorRoll = 2.0f * errorQuat_.x;
  float errorPitch = 2.0f * errorQuat_.y;
  
  // デッドバンド
  if (fabsf(errorRoll) < ERROR_DEADBAND) errorRoll = 0.0f;
  if (fabsf(errorPitch) < ERROR_DEADBAND) errorPitch = 0.0f;
  
  // D項無効化判定
  bool disableDterm = (dt <= 0.0f) || (dt > DTERM_DT_MAX);
  
  // PID計算
  rollOut = pidRoll_.compute(errorRoll, dt, disableDterm);
  pitchOut = pidPitch_.compute(errorPitch, dt, disableDterm);
}

void AttitudeController::reset() {
  pidRoll_.reset();
  pidPitch_.reset();
  errorQuat_ = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
}

Quaternion AttitudeController::computeErrorQuaternion(const Quaternion& target, const Quaternion& current) {
  // q_error = q_target^-1 * q_current
  Quaternion targetInv = quaternionConjugate(target);
  Quaternion error = quaternionMultiply(targetInv, current);
  error.normalize();
  return error;
}

float AttitudeController::wrapAngleError(float error) {
  while (error > PI) error -= 2.0f * PI;
  while (error < -PI) error += 2.0f * PI;
  return error;
}

// ==================== FlightController ====================

FlightController::FlightController() 
  : imuSensor_(nullptr),
    rollOutput_(0.0f),
    pitchOutput_(0.0f) {}

bool FlightController::begin(BMI2_BMM1_Class* imuSensor) {
  imuSensor_ = imuSensor;
  
  if (!imuSensor_) {
    return false;
  }
  
  // BMI270初期化
  if (!imuSensor_->begin(&Serial)) {
    Serial.println("[FlightController] IMU initialization failed!");
    return false;
  }
  
  Serial.println("[FlightController] IMU initialized successfully.");
  reset();
  return true;
}

bool FlightController::update() {
  if (!imuSensor_) {
    return false;
  }
  
  // IMUデータ読み取り
  IMUData imu;
  if (!readIMU(imu)) {
    return false;
  }
  
  // 姿勢推定
  Quaternion attitude = attitudeEstimator_.update(imu);
  
  // 高度推定
  AltitudeState altState = altitudeEstimator_.update(imu, attitude);
  
  // 制御計算
  float dt = 0.05f;  // 仮定: 20Hz更新（実際はタイムスタンプから計算すべき）
  attitudeController_.compute(attitude, dt, rollOutput_, pitchOutput_);
  
  return true;
}

void FlightController::getControlOutput(float& roll, float& pitch) const {
  roll = rollOutput_;
  pitch = pitchOutput_;
}

void FlightController::reset() {
  attitudeEstimator_.reset();
  altitudeEstimator_.reset();
  attitudeController_.reset();
  rollOutput_ = 0.0f;
  pitchOutput_ = 0.0f;
}

bool FlightController::readIMU(IMUData& imu) {
  imu_data_t raw;
  imuSensor_->readGyroAccel(raw, true);  // 生データ取得
  
  // 加速度変換 [g] -> [m/s^2]
  imu.ax = (raw.acc.x * ACC_SCALE_16G) * G_TO_MS2;
  imu.ay = (raw.acc.y * ACC_SCALE_16G) * G_TO_MS2;
  imu.az = (raw.acc.z * ACC_SCALE_16G) * G_TO_MS2;
  
  // ジャイロ [deg/s]
  imu.gx = raw.gyr.x * GYRO_SCALE_2000DPS;
  imu.gy = raw.gyr.y * GYRO_SCALE_2000DPS;
  imu.gz = raw.gyr.z * GYRO_SCALE_2000DPS;
  
  // 地磁気（未使用）
  imu.cx = 0;
  imu.cy = 0;
  imu.cz = 0;
  
  imu.timestamp = millis();
  
  return true;
}
