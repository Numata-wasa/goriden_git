#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
// Madgwickの代わりにAdafruit AHRS内のNXP Fusion(Kalman Filter)を使用
#include <Adafruit_AHRS.h>

// BNO055のインスタンス
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// EKF (NXP Sensor Fusion) のインスタンス
// このアルゴリズムはKalman Filterの一種で、計算負荷は高いですがPicoなら余裕で動作します
Adafruit_NXPSensorFusion filter;

// サンプリングレート
const float SENSOR_RATE = 100.0f; 

void setup() {
  Serial.begin(115200);
  
  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();

  if (!bno.begin()) {
    Serial.print("BNO055が見つかりません");
    while (1);
  }

  // 生データモード (AMG)
  bno.setMode(OPERATION_MODE_AMG);

  // フィルタ初期化 (サンプリングレートを指定)
  filter.begin(SENSOR_RATE);

  delay(1000);
}

void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long currentMicros = micros();
  
  // 100Hz周期で実行
  if (currentMicros - lastUpdate >= (1000000 / SENSOR_RATE)) {
    // 経過時間を秒単位で計算（NXPフィルタのupdateに渡すため正確に計算）
    float dt = (currentMicros - lastUpdate) / 1000000.0;
    lastUpdate = currentMicros;

    // --- センサーデータの取得 ---
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    // --- 単位の調整 ---
    // NXP Fusion (Adafruit実装) の期待する単位:
    // Gyro: deg/s (度/秒)
    // Accel: m/s^2 (MadgwickではGでしたが、こちらはm/s^2が標準)
    // Mag: uT (マイクロテスラ)

    float gx = gyro.x();
    float gy = gyro.y();
    float gz = gyro.z();

    float ax = accel.x(); // m/s^2 のままでOK
    float ay = accel.y();
    float az = accel.z();

    float mx = mag.x();
    float my = mag.y();
    float mz = mag.z();

    // ★重要: 地磁気キャリブレーション補正（オフセット）
    // 以下の数値は MotionCal 等で測定した値をハードコーディングして入れる場所です。
    // 生データを使う以上、これをやらないとEKFでも方位はずれます。
    // mx -= 0.0; 
    // my -= 0.0;
    // mz -= 0.0;

    // --- フィルタ更新 (EKF / NXP Fusion) ---
    // update関数は (gx, gy, gz, ax, ay, az, mx, my, mz) の順
    // ※内部でKalmanゲインの計算と共分散行列の更新が行われます
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    // --- 結果の出力 ---
    // ロール、ピッチ、ヨーを取得
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float yaw = filter.getYaw();

    //Serial.print("Orientation: ");
    Serial.print(yaw);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.println(roll);
  }
}