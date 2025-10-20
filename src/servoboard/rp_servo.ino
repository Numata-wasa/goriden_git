//45から2秒待機してしせいせちぎょ

const int led = 25;


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
/* 10msごとにBNO055の加速度を取得してシリアルに表示します。
   Raspberry Pi Pico (RP2040) で I2C を SDA=GP20 / SCL=GP21 に割当て。
*/

static const uint8_t SDA_PIN = 20;  // GP20
static const uint8_t SCL_PIN = 21;  // GP21
static const uint32_t BNO055interval_ms = 10;

Adafruit_BNO055 bno(55, 0x28, &Wire);  // ADR=GNDで0x28

uint32_t last_ms = 0;

Servo myServo1;

int servot = 0;

void setup() {
  myServo1.attach(18);
  pinMode(led, OUTPUT);


  Serial.begin(115200);
  // USBシリアルが開くのを少し待つ（必要なら）
  for (uint32_t t = millis(); millis() - t < 1000 && !Serial; ) { }

  // I2Cピンを指定して開始（PicoはこれでOK）
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  Serial.println("Orientation Sensor Raw Data Test\n");

  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1) { delay(1000); }
  }

  delay(1000);

  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C\n");

  // 外部クリスタル未使用（ボードに載っている場合は true でもOK）
  bno.setExtCrystalUse(false);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
   myServo1.write(90);

  digitalWrite(led, HIGH);
}

void loop() {
  const uint32_t now = millis();
  if (now - last_ms >= BNO055interval_ms) {
    last_ms += BNO055interval_ms;

    // 加速度センサ値の取得
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    /*
    Serial.print(" Ac_xyz: ");
    Serial.print(acc.x(), 3);
    Serial.print(", ");
    Serial.print(acc.y(), 3);
    Serial.print(", ");
    Serial.print(acc.z(), 3);
    Serial.println();
    */
      
  float pal1 = (acc.x() * acc.x()) +
               (acc.y() * acc.y()) +
               (acc.z() * acc.z());
  float pal2 = sqrt(pal1);

    Serial.println(pal2);


  myServo1.write(90);

  if (pal2 > 20) {
    delay(500);
    while(servot < 10){
      myServo1.write(110);
      delay(500);
      myServo1.write(70);
      delay(500);

      servot = servot + 1;
    }
    myServo1.write(90); 

    while(true){
      digitalWrite(led, HIGH);
      delay(500);
      digitalWrite(led, LOW);
      delay(500);
  }
  }

 
    

    // （必要ならここで他ベクトルやキャリブ状態も読む）
    // uint8_t sys, g, a, m;
    // bno.getCalibration(&sys, &g, &a, &m);
    // Serial.printf("Calib Sys=%d Gyro=%d Accel=%d Mag=%d\n", sys, g, a, m);
  }

  // 他の処理があればここに
}