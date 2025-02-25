#include <MPU6050.h>
#include <Arduino.h>
#include <FastLED.h>
#include <IRremote.h>
#include <Wire.h>

#define LED_PIN 5
#define BUTTON_PIN 2
#define IR_RECEIVER_PIN 4
#define IR_EMITTER_PIN 3

#define NUM_LEDS 3
#define LED_BRIGHTNESS 10
// #define WINDOW_SIZE 5 // Sliding window size
#define IMU_WINDOW_SIZE 20

MPU6050 mpu;
CRGB leds[NUM_LEDS];

// Circular buffer arrays
// double accelBuffer[WINDOW_SIZE] = {0};
// double gyroBuffer[WINDOW_SIZE] = {0};
// int bufferIndex = 0;
// bool bufferFilled = false;

struct gameState_t
{
  int health;
  int ammo;
};

gameState_t gameState = {100, 6};

struct imuData_t
{
  double ax, ay, az; // Acceleration
  double gx, gy, gz; // Gyroscope
};

bool buttonPressed = false;
const float GRYO_THRESHOLD = 112;
const float ACCEL_THRESHOLD = 12;

bool isMotion = false;
int imuWindowIndex = IMU_WINDOW_SIZE;
imuData_t imuDataArray[IMU_WINDOW_SIZE];

void setup()
{

  Serial.begin(115200);

  // LED strip setup
  setupLed();

  // MPU setup
  setupMpu();

  // IR transmitter
  setupIrTransmitter();

  // Button setup
  pinMode(BUTTON_PIN, INPUT);
}

void setupLed()
{
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
}

void setupMpu()
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

void setupIrTransmitter()
{
  IrSender.begin(IR_EMITTER_PIN);
  Serial.println("IR Transmitter initialized.");
}

void sendIrSignal()
{
  uint8_t address = 0x11;
  uint8_t command = 0x22;

  Serial.println("Sending IR signal...");
  IrSender.sendNEC(address, command, 0); // NEC protocol
  Serial.print("Sent NEC IR Signal - Address = ");
  Serial.print(address, HEX);
  Serial.print(", Command = ");
  Serial.println(command, HEX);
}

float computeMagnitude(float x, float y, float z)
{
  return sqrt(x * x + y * y + z * z);
}

float prevAccel[3] = {2.73, 0.53, 8.25}; // AccelXnorm:2.73, AccelYnorm:0.53, AccelZnorm:8.25
const float ACCEL_THRESHOLD_2 = 9;
bool isMotionDetected2(imuData_t imuData)
{
  // Compute magnitude of acceleration & gyro data
  double accelMag = computeMagnitude(imuData.ax - prevAccel[0], imuData.ay - prevAccel[1], imuData.az - prevAccel[2]);

  prevAccel[0] = imuData.ax;
  prevAccel[1] = imuData.ay;
  prevAccel[2] = imuData.az;

  // Check if motion is detected
  return (accelMag > ACCEL_THRESHOLD_2);
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

void shootAmmo()
{
  if (!buttonPressed && digitalRead(BUTTON_PIN) == HIGH)
  {
    if (gameState.ammo > 0)
    {
      Serial.println("shoot");
      buttonPressed = true;
      sendIrSignal();
      --gameState.ammo;
    }
    else
    {
      Serial.println("No ammo left. Please reload...");
    }
  }
  if (digitalRead(BUTTON_PIN) == LOW)
  {
    buttonPressed = false;
  }
}

void updateLed()
{
  int numFullLed = gameState.ammo / 2;
  bool hasHalfLed = gameState.ammo % 2 == 1;

  for (int i = 0; i < numFullLed; ++i)
  {
    leds[i] = CRGB(0, 255, 0);
  }
  for (int j = numFullLed; j < NUM_LEDS; ++j)
  {
    leds[j] = CRGB(0, 0, 0);
  }
  if (hasHalfLed)
  {
    leds[numFullLed] = CRGB(255, 255, 0);
  }

  FastLED.show();
}

void getGameState()
{
  if (Serial.available() > 0)
  {
    String input = Serial.readStringUntil('\n');
    int separatorIndex = input.indexOf(',');
    if (separatorIndex != -1)
    {
      String healthStr = input.substring(0, separatorIndex);
      String ammoStr = input.substring(separatorIndex + 1);
      gameState.health = healthStr.toInt();
      gameState.ammo = ammoStr.toInt();
      Serial.print("Updated gameState - Health = ");
      Serial.print(gameState.health);
      Serial.print(", Ammo = ");
      Serial.println(gameState.ammo);
    }
  }
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

    if (imuWindowIndex == IMU_WINDOW_SIZE)
    {
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

  // mimic receiving gameState from game engine
  getGameState();

  isMotion = isMotionDetected2(imuData);
  if (imuWindowIndex == IMU_WINDOW_SIZE && isMotion)
  {
    imuWindowIndex = 0;
  }

  sendImuData(imuData);

  // button logic
  shootAmmo();
  updateLed();
  Serial.println(digitalRead(BUTTON_PIN) == HIGH);

  // auto reload
  if (gameState.ammo == 0)
  {
    Serial.println("Reloading...");
    delay(1500);
    Serial.println("Done reloading");
    gameState.ammo = 6;
  }

  delay(50); // period = 50ms = 20Hz -> 20 samples per second
}