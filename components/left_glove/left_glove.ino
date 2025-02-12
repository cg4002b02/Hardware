#include <MPU6050.h>
#include <Arduino.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  // MPU setup
  setup_mpu();
}

void setup_mpu() {
  Serial.println("Init MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("No MPU");
    delay(500);
  }

  mpu.calibrateGyro();
  mpu.setThreshold(3);
}

void get_imu_readings() {
  Vector rawGyro = mpu.readRawGyro();
  Vector normGyro = mpu.readNormalizeGyro();
 
  Serial.print("Gyro Xnorm = ");
  Serial.print(normGyro.XAxis);
  Serial.print(" Gyro Ynorm = ");
  Serial.print(normGyro.YAxis);
  Serial.print(" Gyro Znorm = ");
  Serial.println(normGyro.ZAxis);

  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();
 
  Serial.print("Accel Xnorm = ");
  Serial.print(normAccel.XAxis);
  Serial.print(" Accel Ynorm = ");
  Serial.print(normAccel.YAxis);
  Serial.print(" Accel Znorm = ");
  Serial.println(normAccel.ZAxis);
 
  delay(50);
}


void loop() {
  get_imu_readings();

  delay(50);
}