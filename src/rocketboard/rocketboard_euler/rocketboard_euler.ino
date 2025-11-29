/***************************************************************************
 * This sketch reads data from a 9DoF sensor setup (BMI088 + QMC5883L)
 * and outputs calculated Euler angles (Roll, Pitch, Yaw) to the
 * Serial Monitor every 50ms (20Hz).
 *
 * It uses the MadgwickAHRS library for sensor fusion.
 * PLEASE INSTALL the 'Madgwick' library by Paul V. (jrem)
 * via the Arduino Library Manager.
 ***************************************************************************/

//エラーランプの点灯の設定
const int error_ledpin = 41; //エラーランプ
const int stat_ledpin = 42; //STATランプ（GPSに合わせて点灯させたい）

// 共通ライブラリ
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Wire.h> // I2C通信に必要
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <QMC5883LCompass.h>

// ★追加: センサーフュージョンライブラリ
#include <MadgwickAHRS.h>

// ===== GPS =====
static const int RXPin = 7, TXPin = 6;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial ss(1);
// ===============

// ===== SD =====
#define SD_CS_PIN      2  // チップセレクトピン
#define SPI_MOSI_PIN   4  // MOSIピン
#define SPI_MISO_PIN   3  // MISOピン
#define SPI_SCK_PIN    1  // SCKピン

SPIClass spi;
const char* fileName = "/fulldata.csv";
File dataFile;
// ===============

// ===== BME =====
#define SEALEVELPRESSURE_HPA (1011.4)
#define I2C_SDA 17
#define I2C_SCL 16

Adafruit_BME280 bme; // I2C
// ==============

// ★変更: 時間管理用の変数 (オイラー角計算インターバル)
unsigned long lastCalcTime = 0; // lastLogTime -> lastCalcTime
const long calcInterval = 50; // 計算間隔 (ミリ秒) 20Hz 

//================================================================
// BMI088 I2C Addresses
//================================================================
#define ACC_ADDRESS 0x19  // 加速度センサー (SDO=VCC)
#define GYRO_ADDRESS 0x69 // ジャイロ (SDO=VCC)

//================================================================
// BMI088 Register Addresses (抜粋)
//================================================================
// --- Accelerometer Registers ---
#define ACC_CHIP_ID      0x00
#define ACC_DATA_START   0x12
#define ACC_CONF         0x40
#define ACC_RANGE        0x41
#define ACC_PWR_CONF     0x7C
#define ACC_PWR_CTRL     0x7D
#define ACC_SOFTRESET    0x7E

// --- Gyroscope Registers ---
#define GYRO_CHIP_ID     0x00
#define GYRO_DATA_START  0x02
#define GYRO_RANGE       0x0F
#define GYRO_BANDWIDTH   0x10
#define GYRO_LPM1        0x11
#define GYRO_SOFTRESET   0x14

//================================================================
// 物理量変換のためのスケールファクタ
//================================================================
const float ACC_SCALE = 3.0f / 32768.0f; 
const float G_TO_MS2 = 9.80665f; // 重力加速度
const float GYRO_SCALE = 2000.0f / 32768.0f;

// センサーデータを格納する構造体
struct SensorData {
  float ax, ay, az; // Accelerometer data [m/s^2]
  float gx, gy, gz; // Gyroscope data [dps]
};

int cx = 0;
int cy = 0;
int cz = 0;

SensorData sensorData;
QMC5883LCompass compass;

// ★追加: Madgwickフィルタのインスタンスと時間管理
Madgwick filter;
unsigned long lastUpdateTime = 0; // フィルタ更新用の高精度時間 (micros)  <- ★変更: この行は削除します

//================================================================
// I2C Register Read/Write Helper Functions
//================================================================
void writeRegister(uint8_t i2c_addr, uint8_t reg_addr, uint8_t value) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.write(value);
  Wire.endTransmission();
}

void readRegisters(uint8_t i2c_addr, uint8_t reg_addr, uint8_t* buffer, uint8_t len) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(i2c_addr, len);
  for (uint8_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }
}

//================================================================
// BMI088 Setup
//================================================================
void bmisetup(){
  Wire.setClock(400000); 

  //---- Accelerometer Initialization ----
  Serial.println("Initializing Accelerometer...");
  writeRegister(ACC_ADDRESS, ACC_SOFTRESET, 0xB6);
  delay(100);
  writeRegister(ACC_ADDRESS, ACC_PWR_CTRL, 0x04);
  delay(50);
  writeRegister(ACC_ADDRESS, ACC_RANGE, 0x00); // ±3g
  writeRegister(ACC_ADDRESS, ACC_CONF, 0xA8); // ODR=100Hz, BWP=Normal
  writeRegister(ACC_ADDRESS, ACC_PWR_CONF, 0x00);
  delay(10);
  Serial.println("Accelerometer Initialized.");

  //---- Gyroscope Initialization ----
  Serial.println("Initializing Gyroscope...");
  writeRegister(GYRO_ADDRESS, GYRO_SOFTRESET, 0xB6);
  delay(100);
  writeRegister(GYRO_ADDRESS, GYRO_RANGE, 0x00); // ±2000dps
  writeRegister(GYRO_ADDRESS, GYRO_BANDWIDTH, 0x88); // ODR=100Hz, BW=12Hz
  writeRegister(GYRO_ADDRESS, GYRO_LPM1, 0x00); // Normal mode
  delay(50);
  Serial.println("Gyroscope Initialized.");
  Serial.println("----------------------------------------");
}

//================================================================
// BMI088 Data Get
//================================================================
void bmieget(){
  uint8_t buffer[6];

  //---- Read Accelerometer Data ----
  readRegisters(ACC_ADDRESS, ACC_DATA_START, buffer, 6);
  int16_t raw_ax = (int16_t)((buffer[1] << 8) | buffer[0]);
  int16_t raw_ay = (int16_t)((buffer[3] << 8) | buffer[2]);
  int16_t raw_az = (int16_t)((buffer[5] << 8) | buffer[4]);
  sensorData.ax = raw_ax * ACC_SCALE * G_TO_MS2;
  sensorData.ay = raw_ay * ACC_SCALE * G_TO_MS2;
  sensorData.az = raw_az * ACC_SCALE * G_TO_MS2;

  //---- Read Gyroscope Data ----
  readRegisters(GYRO_ADDRESS, GYRO_DATA_START, buffer, 6);
  int16_t raw_gx = (int16_t)((buffer[1] << 8) | buffer[0]);
  int16_t raw_gy = (int16_t)((buffer[3] << 8) | buffer[2]);
  int16_t raw_gz = (int16_t)((buffer[5] << 8) | buffer[4]);
  sensorData.gx = raw_gx * GYRO_SCALE;
  sensorData.gy = raw_gy * GYRO_SCALE;
  sensorData.gz = raw_gz * GYRO_SCALE;
}

//================================================================
// QMC5883L Setup
//================================================================
void qmcsetup(){
    compass.init();
    Serial.println("QMC initialized");
}

//================================================================
// QMC5883L Data Get
//================================================================
void qmcget(){
    compass.read();
    cx = compass.getX();
    cy = compass.getY();
    cz = compass.getZ();
}

//================================================================
// SETUP
//================================================================
void setup() {
    pinMode(error_ledpin, OUTPUT );
    pinMode(stat_ledpin, OUTPUT );

    Serial.begin(115200);
    Serial.println("\n9DoF Euler Angle Serial Output");
    Serial.println("Initializing sensors...");

    // SPIバスを初期化 (SDカード用)
    spi.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);

    //I2C開始
    Wire.begin(I2C_SDA, I2C_SCL);

    // BME280 初期化
    bool status = bme.begin(0x76); 
    bmisetup(); // BMI088 初期化
    qmcsetup(); // QMC5883L 初期化

    if (!status) {
        Serial.println("Could not find a valid BME280 sensor!");
        digitalWrite(error_ledpin, HIGH);
        while (1) delay(10);
    }

    // GPS用のシリアルポートを初期化
    ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

    // SDカードを初期化 (元のコードのまま、ヘッダー書き込み)
    Serial.println("Initializing SD card...");
    if (!SD.begin(SD_CS_PIN, spi)) {
        Serial.println("Card failed, or not present. Halting.");
        digitalWrite(error_ledpin, HIGH);
        while (1);
    }
    Serial.println("SD card initialized.");

    // CSVヘッダーを書き込み
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
        dataFile.println("Date,Time,Lat,Lng,Sat,Alt,Temp,Pres,PrAl,Humi,ax,ay,az,gx,gy,gz,cx,cy,cz");
        dataFile.close();
        Serial.print("Header written to ");
        Serial.println(fileName);
    } else {
        Serial.print("Error opening ");
        digitalWrite(error_ledpin, HIGH);
        Serial.println(fileName);
    }
    Serial.println();

    qmcsetup(); // (元のコードのまま2回目)

    // ★追加: フィルタのサンプリング周波数と開始時間を設定
    filter.begin(1.0f / (calcInterval / 1000.0f)); // ★変更: この行をアンコメント（有効化）します
    // lastUpdateTime = micros(); // ★変更: この行は削除します (dt計算が不要なため)
    Serial.println("Sensor fusion filter initialized. Starting loop...");
}

//================================================================
// LOOP
//================================================================
void loop() {
    // GPSモジュールから受信したデータをTinyGPS++に渡す
    while (ss.available() > 0) {
        gps.encode(ss.read());
    }

    //GPSが受信できていたらSTATLEDが点灯
    if (gps.location.isUpdated()){
        digitalWrite(stat_ledpin, HIGH);
    } else {
        digitalWrite(stat_ledpin, LOW);
    }

    // ★修正: オイラー角の計算と出力をインターバルごとに行う
    if (millis() - lastCalcTime >= calcInterval) {
        lastCalcTime = millis();

        // 1. 全てのセンサーデータを取得
        bmieget(); // 加速度 (m/s^2), ジャロ (dps)
        qmcget();  // 地磁気 (raw)

        // 2. デルタタイム(dt)を計算 (秒単位) <- ★変更: dtの計算は不要なため、以下の3行を削除します
        // unsigned long now = micros();
        // float dt = (now - lastUpdateTime) / 1000000.0f;
        // lastUpdateTime = now;

        // 3. フィルタを更新
        // MadgwickAHRS.h は ジャイロ(dps), 加速度(G), 地磁気(任意) を想定
        
        // 加速度(m/s^2)を G (9.8m/s^2) 単位に変換
        float ax_g = sensorData.ax / G_TO_MS2;
        float ay_g = sensorData.ay / G_TO_MS2;
        float az_g = sensorData.az / G_TO_MS2;

        // ジャイロは dps (sensorData.gx, gy, gz)
        // 地磁気は raw (cx, cy, cz)
        
        // 9DoF (IMU + Mags) フィルタ更新
        // (dt を使うオーバーロード) <- ★変更: 9引数のバージョンを呼び出します
        filter.update(sensorData.gx, sensorData.gy, sensorData.gz,
                      ax_g, ay_g, az_g,
                      (float)cx, (float)cy, (float)cz); // ★変更: 最後の引数 'dt' を削除

        // 4. オイラー角を取得 (度)
        float roll = filter.getRoll();
        float pitch = filter.getPitch();
        float yaw = filter.getYaw();

        // 5. シリアルモニタに出力
        Serial.print("Roll: ");
        Serial.print(roll, 2); // 小数点以下2桁
        Serial.print(", Pitch: ");
        Serial.print(pitch, 2);
        Serial.print(", Yaw: ");
        Serial.println(yaw, 2);

        // 元のSDカード書き込みロジックは無効化しています
        // もしオイラー角もSDに書き込みたい場合は、
        // ヘッダーとdataStringに "Roll,Pitch,Yaw" を追加してください
    }
}