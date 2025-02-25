#include <MPU6050.h>
#include <Arduino.h>

#define IMU_WINDOW_SIZE 20

MPU6050 mpu;

struct imuData_t
{
  float ax, ay, az; // Acceleration
  float gx, gy, gz; // Gyroscope
};

bool isMotion = false;
int16_t imuWindowIndex = IMU_WINDOW_SIZE;
imuData_t imuDataArray[IMU_WINDOW_SIZE];

void setup()
{
  Serial.begin(115200);

  // MPU setup
  setup_mpu();
}

void setup_mpu()
{
  Serial.println("Init MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("No MPU");
    delay(500);
  }

  mpu.calibrateGyro();
  mpu.setThreshold(3);
  mpu.setDLPFMode(MPU6050_DLPF_2); // mode 2 for general movement detection
}

imuData_t getImuReadings()
{
  imuData_t imuData;
  Vector normGyro = mpu.readNormalizeGyro();
  Vector normAccel = mpu.readNormalizeAccel();

  imuData.gx = normGyro.XAxis;
  imuData.gy = normGyro.YAxis;
  imuData.gz = normGyro.ZAxis;
  imuData.ax = normAccel.XAxis;
  imuData.ay = normAccel.YAxis;
  imuData.az = normAccel.ZAxis;

  return imuData;
}

float computeMagnitude(float x, float y, float z)
{
  return sqrt(x * x + y * y + z * z);
}

float prevAccel[3] = {-2.53, 0.09, 7.73}; // -2.53, AccelYnorm:-0.09, AccelZnorm:7.73
const float ACCEL_THRESHOLD = 9;
bool isMotionDetected(imuData_t imuData)
{
  // Compute magnitude of acceleration & gyro data
  float accelMag = computeMagnitude(imuData.ax - prevAccel[0], imuData.ay - prevAccel[1], imuData.az - prevAccel[2]);

  prevAccel[0] = imuData.ax;
  prevAccel[1] = imuData.ay;
  prevAccel[2] = imuData.az;

  // Check if motion is detected
  return (accelMag > ACCEL_THRESHOLD);
}

void sendImuData(imuData_t imuData)
{
  if (imuWindowIndex < IMU_WINDOW_SIZE)
  {
    Serial.print("GyroXnorm:");
    Serial.print(imuData.gx);
    Serial.print(",");
    Serial.print(" GyroYnorm:");
    Serial.print(imuData.gy);
    Serial.print(",");
    Serial.print(" GyroZnorm:");
    Serial.print(imuData.gz);
    Serial.print(",");

    Serial.print("AccelXnorm:");
    Serial.print(imuData.ax);
    Serial.print(",");
    Serial.print(" AccelYnorm:");
    Serial.print(imuData.ay);
    Serial.print(",");
    Serial.print(" AccelZnorm:");
    Serial.println(imuData.az);

    imuDataArray[imuWindowIndex] = imuData;
    ++imuWindowIndex;
    
    if (imuWindowIndex == IMU_WINDOW_SIZE) {
      // send imuDataArray
      Serial.println("imuDataArray is full, sending data...");
    }
  }
  else
  {
    // Serial.print("GyroXnorm:");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(" GyroYnorm:");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(" GyroZnorm:");
    // Serial.print(0);
    // Serial.print(",");

    // Serial.print("AccelXnorm:");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(" AccelYnorm:");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(" AccelZnorm:");
    // Serial.println(0);
  }
}

void loop()
{
  imuData_t imuData = getImuReadings();

  isMotion = isMotionDetected(imuData);
  if (imuWindowIndex == IMU_WINDOW_SIZE && isMotion)
  {
    imuWindowIndex = 0;
  }

  sendImuData(imuData);

  delay(50);
}