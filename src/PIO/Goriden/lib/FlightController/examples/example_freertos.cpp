/**
 * @file example_freertos.cpp
 * @brief FlightControllerライブラリのFreeRTOS統合例
 * 
 * このサンプルは以下を実装します：
 * - マルチタスク構成（センサー・制御・ロギング）
 * - タスク間データ共有（キュー使用）
 * - SDカードロギング
 * - サーボ制御
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "BMI2_BMM1.h"
#include "FlightController.h"

// ==================== ハードウェア定義 ====================

// I2C
#define I2C_SDA 17
#define I2C_SCL 16

// SPI & SD
#define SD_CS_PIN   13
#define SPI_MOSI    14
#define SPI_MISO    47
#define SPI_SCK     21

// サーボ
#define SERVO_PIN_ROLL  4
#define SERVO_PIN_PITCH 5

// LED
#define LED_STATUS 42
#define LED_ERROR  41

// ==================== グローバル変数 ====================

BMI2_BMM1_Class imuSensor;
FlightController flightController;
SPIClass spi;
File dataFile;

// ==================== データ構造体 ====================

struct FlightData {
  uint32_t timestamp;
  Quaternion quaternion;
  EulerAngles euler;
  AltitudeState altitude;
  float rollControl;
  float pitchControl;
  int servoRoll;
  int servoPitch;
};

// ==================== FreeRTOS ====================

TaskHandle_t hSensorTask;
TaskHandle_t hControlTask;
TaskHandle_t hLoggingTask;
QueueHandle_t xFlightDataQueue;

#define QUEUE_LENGTH 50

// ==================== タスク1: センサー読み取り（Core 0, Prio 2, 200Hz） ====================

void sensorTask(void *pvParameters) {
  Serial.println("[Task] Sensor Task started on Core 0 (Prio 2)");
  
  const TickType_t xFrequency = pdMS_TO_TICKS(5);  // 5ms = 200Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // FlightController更新（センサー読み取り→姿勢推定→高度推定→制御計算）
    if (!flightController.update()) {
      digitalWrite(LED_ERROR, HIGH);
      continue;
    }
    
    // データ収集
    FlightData data;
    data.timestamp = millis();
    data.quaternion = flightController.getQuaternion();
    data.euler = flightController.getEulerAngles();
    data.altitude = flightController.getAltitudeState();
    flightController.getControlOutput(data.rollControl, data.pitchControl);
    
    // サーボ角度計算
    data.servoRoll = 90 + (int)(90.0f * data.rollControl);
    data.servoPitch = 90 + (int)(90.0f * data.pitchControl);
    data.servoRoll = constrain(data.servoRoll, 0, 180);
    data.servoPitch = constrain(data.servoPitch, 0, 180);
    
    // キューに送信（ノンブロッキング）
    xQueueSend(xFlightDataQueue, &data, 0);
  }
}

// ==================== タスク2: サーボ制御（Core 1, Prio 2, 20Hz） ====================

void controlTask(void *pvParameters) {
  Serial.println("[Task] Control Task started on Core 1 (Prio 2)");
  
  // サーボピン初期化（例: LEDC PWM使用）
  ledcSetup(0, 50, 14);  // チャネル0, 50Hz, 14bit
  ledcSetup(1, 50, 14);  // チャネル1, 50Hz, 14bit
  ledcAttachPin(SERVO_PIN_ROLL, 0);
  ledcAttachPin(SERVO_PIN_PITCH, 1);
  
  const TickType_t xFrequency = pdMS_TO_TICKS(50);  // 50ms = 20Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  FlightData data;
  
  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // キューから最新データを受信
    if (xQueueReceive(xFlightDataQueue, &data, 0) == pdPASS) {
      // サーボ出力（LEDC PWM: 0-16383で制御）
      // 500us(0deg) - 2400us(180deg) を 14bit分解能で変換
      uint32_t dutyRoll = map(data.servoRoll, 0, 180, 500 * 16384 / 20000, 2400 * 16384 / 20000);
      uint32_t dutyPitch = map(data.servoPitch, 0, 180, 500 * 16384 / 20000, 2400 * 16384 / 20000);
      
      ledcWrite(0, dutyRoll);
      ledcWrite(1, dutyPitch);
      
      // ステータスLED点滅
      static bool ledState = false;
      ledState = !ledState;
      digitalWrite(LED_STATUS, ledState ? HIGH : LOW);
    }
  }
}

// ==================== タスク3: データロギング（Core 1, Prio 1） ====================

void loggingTask(void *pvParameters) {
  Serial.println("[Task] Logging Task started on Core 1 (Prio 1)");
  
  FlightData data;
  uint32_t lastFlush = 0;
  
  for (;;) {
    // キューからデータを受信（無限待機）
    if (xQueueReceive(xFlightDataQueue, &data, portMAX_DELAY) == pdPASS) {
      
      // SDカードに書き込み（バイナリ形式）
      if (dataFile) {
        dataFile.write((const uint8_t*)&data, sizeof(FlightData));
      } else {
        digitalWrite(LED_ERROR, HIGH);
      }
      
      // 定期的にフラッシュ（1秒ごと）
      uint32_t now = millis();
      if (now - lastFlush >= 1000) {
        if (dataFile) {
          dataFile.flush();
        }
        lastFlush = now;
      }
      
      // シリアル出力（デバッグ用、1Hz）
      static uint32_t lastPrint = 0;
      if (now - lastPrint >= 1000) {
        Serial.printf("[%6lu] Q(%5.3f,%6.3f,%6.3f,%6.3f) E(%6.1f,%6.1f,%6.1f) Alt:%7.2f Vel:%6.2f Srv(%3d,%3d) L:%d\n",
                      data.timestamp,
                      data.quaternion.w, data.quaternion.x, data.quaternion.y, data.quaternion.z,
                      data.euler.roll * RAD_TO_DEG, data.euler.pitch * RAD_TO_DEG, data.euler.yaw * RAD_TO_DEG,
                      data.altitude.getRelativeAltitude(),
                      data.altitude.velocity,
                      data.servoRoll, data.servoPitch,
                      data.altitude.launchDetected ? 1 : 0);
        lastPrint = now;
      }
    }
  }
}

// ==================== セットアップ ====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  digitalWrite(LED_STATUS, LOW);
  digitalWrite(LED_ERROR, LOW);
  
  Serial.println("=== FlightController FreeRTOS Example ===");
  
  // I2C初期化
  Serial.println("[1/4] Initializing I2C...");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Serial.println("I2C initialized.");
  
  // SD初期化
  Serial.println("[2/4] Initializing SD card...");
  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, spi)) {
    Serial.println("ERROR: SD card initialization failed!");
    digitalWrite(LED_ERROR, HIGH);
    while(1) delay(1000);
  }
  
  dataFile = SD.open("/flight.bin", FILE_WRITE);
  if (!dataFile) {
    Serial.println("ERROR: Failed to open file!");
    digitalWrite(LED_ERROR, HIGH);
    while(1) delay(1000);
  }
  Serial.printf("SD card ready. File: /flight.bin (struct size: %d bytes)\n", sizeof(FlightData));
  
  // FlightController初期化
  Serial.println("[3/4] Initializing FlightController...");
  if (!flightController.begin(&imuSensor)) {
    Serial.println("ERROR: FlightController initialization failed!");
    digitalWrite(LED_ERROR, HIGH);
    while(1) delay(1000);
  }
  
  // 目標クォータニオン設定
  Quaternion targetQuat(0.1829f, -0.7056f, -0.1778f, -0.6611f);
  flightController.setTargetQuaternion(targetQuat);
  flightController.setRollGains(1.0f, 0.0f, 0.32f);
  flightController.setPitchGains(1.2f, 0.0f, 0.4f);
  Serial.println("FlightController initialized.");
  
  // FreeRTOSキュー作成
  Serial.println("[4/4] Creating FreeRTOS tasks...");
  xFlightDataQueue = xQueueCreate(QUEUE_LENGTH, sizeof(FlightData));
  if (xFlightDataQueue == NULL) {
    Serial.println("ERROR: Queue creation failed!");
    digitalWrite(LED_ERROR, HIGH);
    while(1) delay(1000);
  }
  
  // タスク作成
  xTaskCreatePinnedToCore(sensorTask,  "SensorTask",  4096, NULL, 2, &hSensorTask,  0);  // Core 0, Prio 2
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, NULL, 2, &hControlTask, 1);  // Core 1, Prio 2
  xTaskCreatePinnedToCore(loggingTask, "LoggingTask", 4096, NULL, 1, &hLoggingTask, 1);  // Core 1, Prio 1
  
  Serial.println("All tasks created. System ready!");
  digitalWrite(LED_STATUS, HIGH);
  delay(1000);
  digitalWrite(LED_STATUS, LOW);
}

// ==================== メインループ ====================

void loop() {
  // FreeRTOSタスクに任せるため、ここでは何もしない
  vTaskDelay(pdMS_TO_TICKS(1000));
}


// ==================== 補足情報 ====================

/*
 * 【タスク構成】
 * 
 * Core 0:
 *   - sensorTask (Prio 2, 200Hz): IMU読み取り→姿勢推定→高度推定→制御計算
 * 
 * Core 1:
 *   - controlTask (Prio 2, 20Hz): サーボ出力
 *   - loggingTask (Prio 1): SDカード書き込み（ブロッキング可）
 * 
 * 
 * 【データフロー】
 * 
 *   sensorTask → Queue → controlTask (サーボ出力)
 *                     ↘
 *                      loggingTask (SD書き込み)
 * 
 * 
 * 【ログファイル解析】
 * 
 * Pythonでバイナリログを読み取る例：
 * 
 * import struct
 * import matplotlib.pyplot as plt
 * 
 * # 構造体サイズ: 72バイト（環境依存、要確認）
 * struct_size = 72
 * 
 * with open("flight.bin", "rb") as f:
 *     data = f.read()
 * 
 * num_records = len(data) // struct_size
 * timestamps = []
 * altitudes = []
 * velocities = []
 * 
 * for i in range(num_records):
 *     chunk = data[i*struct_size:(i+1)*struct_size]
 *     # unpackフォーマットは構造体定義に合わせる
 *     # 例: 'I4f3f9fii' など
 *     values = struct.unpack('I4f3f9fii', chunk)
 *     timestamps.append(values[0])
 *     altitudes.append(values[12])  # altitude.altitude
 *     velocities.append(values[13])  # altitude.velocity
 * 
 * plt.subplot(2,1,1)
 * plt.plot(timestamps, altitudes)
 * plt.ylabel('Altitude [m]')
 * 
 * plt.subplot(2,1,2)
 * plt.plot(timestamps, velocities)
 * plt.ylabel('Velocity [m/s]')
 * plt.xlabel('Time [ms]')
 * 
 * plt.show()
 * 
 * 
 * 【パフォーマンス最適化】
 * 
 * - センサータスクを高優先度にし、制御の応答性を確保
 * - ロギングタスクを低優先度にし、SD書き込み遅延の影響を最小化
 * - キューサイズを十分に取り、データロスを防ぐ
 * - Core 0とCore 1で負荷を分散
 */
