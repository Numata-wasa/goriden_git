#include <Wire.h>
#include "BMI2_BMM1.h"

#define SDA_PIN 17
#define SCL_PIN 16

// ±2000 dps設定時のスケール係数（BMI270標準）
#define GYRO_SCALE_2000DPS (1.0f / 16.4f)

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!IMU.begin(&Serial)) {
    Serial.println("IMU init failed!");
    while (1);
  }
  Serial.println("IMU initialized successfully.");
}

void loop() {
  imu_data_t imu;
  mag_data_t mag;

  // 生データ取得
  IMU.readGyroAccel(imu, true);   // 第二引数 true でRawデータを読む（この関数が存在する！）
  IMU.readAuxMag(mag);

  // 加速度 [g] に変換
  float ax = imu.acc.x / 16384.0f;
  float ay = imu.acc.y / 16384.0f;
  float az = imu.acc.z / 16384.0f;

  // ジャイロ [°/s] に変換
  float gx = imu.gyr.x * GYRO_SCALE_2000DPS;
  float gy = imu.gyr.y * GYRO_SCALE_2000DPS;
  float gz = imu.gyr.z * GYRO_SCALE_2000DPS;

  // 磁気 [uT] に変換（BMM150標準換算）
  float mx = mag.x * 0.3f;
  float my = mag.y * 0.3f;
  float mz = mag.z * 0.3f;

  Serial.printf("Accel[g]: %.3f, %.3f, %.3f\n", ax, ay, az);
  Serial.printf("Gyro[dps]: %.3f, %.3f, %.3f\n", gx, gy, gz);
  Serial.printf("Mag[uT]: %.3f, %.3f, %.3f\n\n", mx, my, mz);

  delay(500);
}
