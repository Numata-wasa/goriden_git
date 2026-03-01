/**
 * @file example_basic.cpp
 * @brief FlightControllerライブラリの基本的な使用例
 * 
 * このサンプルは以下を実装します：
 * - IMU初期化と姿勢推定
 * - 高度推定（加速度積分）
 * - 目標姿勢へのPID制御
 * - シリアル出力によるモニタリング
 */

#include <Arduino.h>
#include <Wire.h>
#include "BMI2_BMM1.h"
#include "FlightController.h"

// ==================== グローバル変数 ====================

BMI2_BMM1_Class imuSensor;
FlightController flightController;

// サーボピン定義（例）
#define SERVO_PIN_ROLL  4
#define SERVO_PIN_PITCH 5

// ==================== 初期化 ====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== FlightController Basic Example ===");
  
  // I2C初期化
  Serial.println("[1/3] Initializing I2C...");
  Wire.begin(17, 16);  // SDA=17, SCL=16
  Wire.setClock(400000);
  Serial.println("I2C initialized at 400kHz.");
  
  // FlightController初期化（IMU初期化含む）
  Serial.println("[2/3] Initializing FlightController...");
  if (!flightController.begin(&imuSensor)) {
    Serial.println("ERROR: FlightController initialization failed!");
    while(1) {
      delay(1000);
    }
  }
  Serial.println("FlightController initialized successfully.");
  
  // 目標クォータニオン設定
  // 例: 機体を傾けた状態を目標とする
  // Quat(0.1829, -0.7056, -0.1778, -0.6611)
  Serial.println("[3/3] Setting target quaternion...");
  Quaternion targetQuat(0.1829f, -0.7056f, -0.1778f, -0.6611f);
  flightController.setTargetQuaternion(targetQuat);
  
  // PIDゲイン設定（オプション: デフォルト値を使う場合は不要）
  flightController.setRollGains(1.0f, 0.0f, 0.32f);   // Kp, Ki, Kd
  flightController.setPitchGains(1.2f, 0.0f, 0.4f);
  
  Serial.println("Setup complete!");
  Serial.println("\nFormat: Time | Quaternion | Euler[deg] | Altitude[m] | Velocity[m/s] | Control[roll,pitch]");
  Serial.println("------------------------------------------------------------------------");
  
  delay(1000);
}

// ==================== メインループ ====================

void loop() {
  static uint32_t lastUpdate = 0;
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  
  // センサー・制御更新（200Hz = 5ms周期）
  if (now - lastUpdate >= 5) {
    lastUpdate = now;
    
    // FlightController更新（センサー読み取り→姿勢推定→高度推定→制御計算）
    if (!flightController.update()) {
      Serial.println("ERROR: FlightController update failed!");
      return;
    }
    
    // 制御出力を取得
    float rollCmd, pitchCmd;
    flightController.getControlOutput(rollCmd, pitchCmd);
    
    // サーボ制御（例: -1～1 を 0～180度に変換）
    int servoRoll = 90 + (int)(90.0f * rollCmd);
    int servoPitch = 90 + (int)(90.0f * pitchCmd);
    servoRoll = constrain(servoRoll, 0, 180);
    servoPitch = constrain(servoPitch, 0, 180);
    
    // サーボ出力（実装例）
    // analogWrite(SERVO_PIN_ROLL, servoRoll);
    // analogWrite(SERVO_PIN_PITCH, servoPitch);
  }
  
  // シリアル出力（1Hz = 1秒周期）
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    
    // 姿勢取得
    Quaternion q = flightController.getQuaternion();
    EulerAngles euler = flightController.getEulerAngles();
    
    // 高度状態取得
    AltitudeState alt = flightController.getAltitudeState();
    
    // 制御出力取得
    float rollCmd, pitchCmd;
    flightController.getControlOutput(rollCmd, pitchCmd);
    
    // フォーマット出力
    Serial.printf("%6lu | Q(%5.3f,%6.3f,%6.3f,%6.3f) | E(%6.1f,%6.1f,%6.1f) | Alt:%7.2f | Vel:%6.2f | Ctrl(%5.2f,%5.2f) | Launch:%d\n",
                  now,
                  q.w, q.x, q.y, q.z,
                  euler.roll * RAD_TO_DEG, euler.pitch * RAD_TO_DEG, euler.yaw * RAD_TO_DEG,
                  alt.getRelativeAltitude(),
                  alt.velocity,
                  rollCmd, pitchCmd,
                  alt.launchDetected ? 1 : 0);
  }
}


// ==================== 補足情報 ====================

/*
 * 【シリアル出力の見方】
 * 
 * Time: ミリ秒単位のタイムスタンプ
 * Q(w,x,y,z): クォータニオン（正規化済み）
 * E(roll,pitch,yaw): オイラー角 [deg]
 * Alt: 相対高度 [m]（発射時=0m、発射前は0固定）
 * Vel: 鉛直速度 [m/s]
 * Ctrl(roll,pitch): 制御出力 [-1.0, 1.0]
 * Launch: 発射検知フラグ（0=待機中、1=発射後）
 * 
 * 
 * 【期待される動作】
 * 
 * 1. 地上待機中（～5秒）:
 *    - Alt: 0.00m（固定）
 *    - Vel: 0.00m/s（バイアス学習中）
 *    - Launch: 0
 * 
 * 2. 発射検知（加速度>15m/s^2）:
 *    - Launch: 0→1 に変化
 *    - Alt: 徐々に増加開始
 * 
 * 3. 推力加速中:
 *    - Alt: 急速に増加
 *    - Vel: プラス値で増加
 * 
 * 4. 慣性飛行中:
 *    - Vel: 徐々に減少（重力による減速）
 * 
 * 5. 頂点（アポジー）:
 *    - Vel: 0.00m/s 付近
 *    - Alt: 最大値
 * 
 * 6. 降下中:
 *    - Vel: マイナス値（下向き）
 *    - Alt: 徐々に減少
 * 
 * 7. 着地後:
 *    - Vel: ZUPTにより 0.00m/s へ収束
 *    - Alt: 正の値で固定（負にはならない）
 * 
 * 
 * 【トラブルシューティング】
 * 
 * ■ Altがマイナスになる
 *   → バイアス学習不足: 起動後5秒以上静止させる
 *   → ZUPTパラメータ調整: FlightController.h の ZUPT_VEL_DAMP を小さく
 * 
 * ■ オイラー角が暴れる
 *   → 加速度補正の問題: ACC_REJECT_HIGH/LOW を調整
 *   → ジャイロドリフト: 相補フィルタ係数ALPHA_MIN を大きく（0.99など）
 * 
 * ■ 発射検知しない
 *   → 閾値が高すぎ: LAUNCH_DETECT_ACC を下げる（10.0など）
 * 
 * ■ 制御出力が飽和（±1.0）
 *   → PIDゲインが大きすぎ: Kp/Kd を小さく
 *   → 目標クォータニオンが遠すぎ: 目標姿勢を再確認
 */
