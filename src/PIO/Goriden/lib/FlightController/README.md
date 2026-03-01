# FlightController ライブラリ

ESP32を用いたロケット飛行制御システムのコアライブラリ

## 概要

このライブラリは、ロケット・モデルロケット・ドローン等の飛行体制御に必要な以下の機能を提供します：

- **姿勢推定**: IMU（BMI270）からクォータニオンベースの姿勢を推定（相補フィルタ）
- **高度推定**: 加速度積分による高度推定（ZUPT補正付き）
- **姿勢制御**: 目標クォータニオンへの追従制御（PID）
- **発射検知**: 加速度閾値による自動検知

## 特徴

### 1. クォータニオンベースの姿勢推定
- ジンバルロック回避
- 相補フィルタによる加速度・ジャイロ融合
- 高加速度・慣性飛行時の自動補正切り替え

### 2. 高度推定（加速度積分）
- 地上座標系への座標変換（クォータニオン回転）
- 発射前の鉛直軸バイアス自動学習
- ZUPT（Zero-velocity Update）による速度ドリフト補正
- 発射検知による相対高度計算（発射時=0m）

### 3. 姿勢制御
- クォータニオン誤差ベースのPD制御
- デッドバンドによる微振動抑制
- スルーレート制限対応（外部実装）

## インストール

PlatformIOプロジェクトの場合：

```
Goriden/
  lib/
    FlightController/
      FlightController.h
      FlightController.cpp
    BMI2_BMM1/        # 依存ライブラリ
      ...
```

## 基本的な使用方法

### 1. 初期化

```cpp
#include <Arduino.h>
#include <Wire.h>
#include "BMI2_BMM1.h"
#include "FlightController.h"

BMI2_BMM1_Class imuSensor;
FlightController flightController;

void setup() {
  Serial.begin(115200);
  Wire.begin(17, 16);  // SDA, SCL
  Wire.setClock(400000);
  
  // 飛行制御システム初期化（IMU初期化含む）
  if (!flightController.begin(&imuSensor)) {
    Serial.println("FlightController initialization failed!");
    while(1);
  }
  
  // 目標クォータニオン設定（例: Quat(0.1829, -0.7056, -0.1778, -0.6611)）
  Quaternion target(0.1829f, -0.7056f, -0.1778f, -0.6611f);
  flightController.setTargetQuaternion(target);
  
  // PIDゲイン調整（オプション）
  flightController.setRollGains(1.0f, 0.0f, 0.32f);   // Kp, Ki, Kd
  flightController.setPitchGains(1.2f, 0.0f, 0.4f);
}
```

### 2. メインループ

```cpp
void loop() {
  // 制御ループ更新（200Hz推奨）
  flightController.update();
  
  // 姿勢取得
  Quaternion q = flightController.getQuaternion();
  EulerAngles euler = flightController.getEulerAngles();
  
  // 高度状態取得
  AltitudeState alt = flightController.getAltitudeState();
  
  // 制御出力取得
  float rollCmd, pitchCmd;
  flightController.getControlOutput(rollCmd, pitchCmd);
  
  // サーボ制御（例: -1～1 を 0～180度に変換）
  int servo1 = 90 + (int)(90.0f * rollCmd);
  int servo2 = 90 + (int)(90.0f * pitchCmd);
  
  // デバッグ出力
  Serial.printf("Q: %.3f,%.3f,%.3f,%.3f | Euler: %.2f,%.2f,%.2f | Alt: %.2f m | Vel: %.2f m/s\n",
                q.w, q.x, q.y, q.z,
                euler.roll * RAD_TO_DEG, euler.pitch * RAD_TO_DEG, euler.yaw * RAD_TO_DEG,
                alt.getRelativeAltitude(), alt.velocity);
  
  delay(5);  // 200Hz
}
```

### 3. FreeRTOSタスク統合例

```cpp
void sensorTask(void *pvParameters) {
  for (;;) {
    flightController.update();
    
    // データをキューに送信
    LogData data;
    data.quaternion = flightController.getQuaternion();
    data.altitude = flightController.getAltitudeState();
    xQueueSend(xQueue, &data, 0);
    
    vTaskDelay(pdMS_TO_TICKS(5));  // 200Hz
  }
}

void controlTask(void *pvParameters) {
  for (;;) {
    float rollCmd, pitchCmd;
    flightController.getControlOutput(rollCmd, pitchCmd);
    
    // サーボ出力
    servoWrite(SERVO1, 90 + (int)(90.0f * rollCmd));
    servoWrite(SERVO2, 90 + (int)(90.0f * pitchCmd));
    
    vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz
  }
}
```

## API リファレンス

### FlightController クラス

#### `bool begin(BMI2_BMM1_Class* imuSensor)`
IMUセンサーを初期化し、システムを起動します。

**パラメータ:**
- `imuSensor`: BMI270センサーのポインタ

**戻り値:**
- `true`: 初期化成功
- `false`: 初期化失敗

#### `bool update()`
センサー読み取り→姿勢推定→高度推定→制御計算を実行します。
メインループまたはタスクで周期的に呼び出してください（推奨: 200Hz）。

**戻り値:**
- `true`: 更新成功
- `false`: 更新失敗

#### `Quaternion getQuaternion()`
現在の姿勢クォータニオンを取得します。

**戻り値:**
- `Quaternion`: 正規化されたクォータニオン (w, x, y, z)

#### `EulerAngles getEulerAngles()`
現在のオイラー角を取得します。

**戻り値:**
- `EulerAngles`: roll, pitch, yaw [rad]

#### `AltitudeState getAltitudeState()`
現在の高度状態を取得します。

**戻り値:**
- `AltitudeState`: 高度・速度・加速度・発射フラグ等

#### `void getControlOutput(float& roll, float& pitch)`
制御出力を取得します。

**パラメータ:**
- `roll`: ロール軸出力 [-1.0, 1.0]
- `pitch`: ピッチ軸出力 [-1.0, 1.0]

#### `void setTargetQuaternion(const Quaternion& target)`
目標クォータニオンを設定します。

**パラメータ:**
- `target`: 目標姿勢のクォータニオン

#### `void setRollGains(float kp, float ki, float kd)`
ロール軸のPIDゲインを設定します。

**パラメータ:**
- `kp`: 比例ゲイン
- `ki`: 積分ゲイン（通常0.0）
- `kd`: 微分ゲイン

#### `void setPitchGains(float kp, float ki, float kd)`
ピッチ軸のPIDゲインを設定します。

#### `void reset()`
システム全体をリセットします（姿勢・高度・制御状態）。

---

### AltitudeState 構造体

```cpp
struct AltitudeState {
  float altitude;           // 積分高度 [m]（絶対値）
  float velocity;           // 鉛直速度 [m/s]
  float acceleration;       // 重力補正後加速度 [m/s^2]
  float bias;               // 鉛直軸バイアス [m/s^2]
  bool launchDetected;      // 発射検知フラグ
  float launchAltitude;     // 発射時高度 [m]
  
  // 相対高度を取得（発射時=0m、発射前は0）
  float getRelativeAltitude() const;
};
```

---

### Quaternion 構造体

```cpp
struct Quaternion {
  float w, x, y, z;        // スカラー部w, ベクトル部(x,y,z)
  
  void normalize();        // 正規化
  void sanitize();         // NaNチェック＆リセット
};
```

---

### EulerAngles 構造体

```cpp
struct EulerAngles {
  float roll, pitch, yaw;  // [rad]
};
```

## パラメータチューニング

### PIDゲイン調整

デフォルト値:
- Roll: Kp=1.0, Kd=0.32
- Pitch: Kp=1.2, Kd=0.4

調整指針:
1. **Kpを増やす**: 応答速度向上、ただしオーバーシュート増加
2. **Kdを増やす**: ダンピング強化、振動抑制
3. **制御周期**: 20Hz（50ms）推奨

### 高度推定パラメータ

`FlightController.h` の定数を変更：

```cpp
const float LAUNCH_DETECT_ACC = 15.0f;           // 発射判定閾値 [m/s^2]
const float VERTICAL_BIAS_ALPHA = 0.01f;         // バイアス学習率（小さい=平滑）
const float ZUPT_VEL_DAMP = 0.85f;               // 速度減衰係数（小さい=強力）
const float ZUPT_VEL_EPS = 0.08f;                // 速度閾値 [m/s]
```

調整指針:
- **ドリフトが大きい**: `ZUPT_VEL_DAMP`を小さく（0.8など）
- **静止検知が敏感すぎる**: `PRELAUNCH_STATIC_ACC_TOL`を大きく
- **発射誤検知**: `LAUNCH_DETECT_ACC`を大きく

## トラブルシューティング

### 姿勢が不安定
- **原因**: 加速度補正が高加速度時に効きすぎ
- **対策**: `ACC_REJECT_HIGH`を小さく（1.2など）、`GYRO_REJECT_DPS`を小さく

### 高度がマイナスになる
- **確認**: `getRelativeAltitude()`を使用しているか？
- **対策**: 発射前の静止時間を長く取る（バイアス学習時間確保）

### 制御が遅れる
- **原因**: 更新周期が遅い
- **対策**: `update()`の呼び出し周期を短く（200Hz推奨）

### 発射検知しない
- **確認**: `LAUNCH_DETECT_ACC`が高すぎないか
- **対策**: 閾値を下げる（10.0など）、ただし誤検知注意

## ハードウェア要件

- **マイコン**: ESP32シリーズ（ESP32-S3推奨）
- **IMU**: BMI270（±16g, ±2000dps設定）
- **I2C周波数**: 400kHz
- **更新周期**: 
  - センサー読み取り: 200Hz（5ms）
  - 制御更新: 20Hz（50ms）

## ライセンス

（ライセンス情報を記載）

## 変更履歴

### v2.0.0 (2026-03-01)
- モジュール化バージョン
- クラスベース設計に変更
- ヘッダー・実装ファイル分離

### v1.0.0 (2026-02-28)
- 初期実装（main.cpp統合版）
- クォータニオン姿勢推定
- ZUPT高度推定
- PID制御

## 参考資料

### クォータニオン関連
- [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508)
- [Understanding Quaternions](https://www.3dgep.com/understanding-quaternions/)

### IMU統合
- [Complementary Filter Design](https://www.olliw.eu/2013/imu-data-fusing/)
- [ZUPT for Pedestrian Navigation](https://ieeexplore.ieee.org/document/4637857)

## サポート

問題・質問がある場合は、GitHubのIssueを利用してください。

---

**注意**: このライブラリは実験的な飛行制御システムです。実際の飛行試験前に地上試験を十分に行い、安全を確保してください。
