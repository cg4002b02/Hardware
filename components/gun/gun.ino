#include "CRC.h"
#include "CRC8.h" 
#include <MPU6050.h>
#include <Arduino.h>
#include <FastLED.h>
#include <IRremote.h>
#include <Wire.h>

#define HANDSHAKE 'h'
#define ACK 1
#define IMU_DATA 2
#define HIT 3
#define GUN 4 
#define HEADER '$'
#define IMU_WINDOW_SIZE 30

#define LED_PIN 5
#define BUTTON_PIN 2
#define IR_RECEIVER_PIN 4
#define IR_EMITTER_PIN 3

#define NUM_LEDS 3
#define LED_BRIGHTNESS 10

MPU6050 mpu;
CRGB leds[NUM_LEDS];

CRC8 crc(0x07);

void (*reset) (void) = 0;

enum ProtoState { DISCONNECTED, HANDSHAKE_INITIATED, WAITING_FOR_ACK, CONFIRMED};
ProtoState protoState = DISCONNECTED;

enum UpdateState { IDLE, READING_UPDATE };
UpdateState currentUpdateState = IDLE;  
byte updateBuffer[3];
int updateIndex = 0;  
unsigned long updateStartTime = 0;           
const unsigned long UPDATE_TIMEOUT = 100;      

unsigned long points = 100;
unsigned long ammo = 6;

int16_t aX, aY, aZ;
int16_t gX, gY, gZ;

// -----------------------------------------------------------------------------
// Stop-and-wait for GUN packet
// -----------------------------------------------------------------------------
bool hasSentGun = false;
bool hasAcknowledgedGun = false;
unsigned long timeoutStart = 0;
const unsigned long TIMEOUT_VAL = 500;
unsigned long lastGunSendTime = 0;
unsigned long lastIMUSendTime = 0;
const unsigned long IMU_INTERVAL = 1100;
bool lastsentGunindex = true;

// -----------------------------------------------------------------------------
// Stop-and-wait for HIT packet
// -----------------------------------------------------------------------------
bool hasSentHit = false;
bool hasAcknowledgedHit = false;
unsigned long timeoutStartHit = 0;
const unsigned long TIMEOUT_VALHIT = 750;
unsigned long lastHitSendTime = 0;
bool lastsentHitindex = true;

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
const float ACCEL_THRESHOLD = 9;

bool isMotion = false;
int imuWindowIndex = IMU_WINDOW_SIZE;

char incoming;

struct Datapacket {
  int8_t type; //bhhhhhhhbbbh
  int16_t aX; 
  int16_t aY; 
  int16_t aZ;
  int16_t gX;
  int16_t gY; 
  int16_t gZ;  
  int16_t y; //consider as padding
  int8_t p; //padding
  int8_t r; //padding
  int8_t start_move; //also padding
  int16_t checksum;
};

struct Ackpacket {
  int8_t type;
  int16_t padding_1;
  int16_t padding_2;
  int16_t padding_3; 
  int16_t padding_4;
  int16_t padding_5;
  int16_t padding_6;
  int16_t padding_7;
  int16_t padding_8;
  int8_t padding_9; 
  int16_t checksum;
};

struct Gunpacket {
  int8_t type;
  int8_t ammo_state;
  int8_t trigger_state;
  int16_t padding_1;
  int16_t padding_2;
  int16_t padding_3;
  int16_t padding_4;
  int16_t padding_5;
  int16_t padding_6;
  int16_t padding_7;
  int16_t padding_8;
  int8_t checksum;
};

Gunpacket lastGunpacket; 

uint8_t calculateCRC8(uint8_t *data, int len) {
  crc.restart();
  crc.add(data, len);
  return crc.getCRC();
}

//---------------------------------------------------------
// Setup: flush serial buffer and start in DISCONNECTED state
//---------------------------------------------------------
void setup() {
    Serial.begin(115200);
    // Flush any residual data
    if (Serial.available() > 0) {
      Serial.read();
    }
    //reset();

    // LED strip setup
    setupLed();

    // MPU setup
    setup_mpu();

    // IR transmitter
    setupIrTransmitter();

    // Button setup
    pinMode(BUTTON_PIN, INPUT);

    protoState = DISCONNECTED;
}

void setup_mpu()
{
  // Serial.println("Init MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    // Serial.println("No MPU");
    delay(500);
  }

  // mpu.calibrateGyro();
  // mpu.setThreshold(3);
  // mpu.setDLPFMode(MPU6050_DLPF_2); // mode 2 for general movement detection
}

void setupLed()
{
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
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

void shootAmmo(unsigned long currentMillis)
{
  if (!buttonPressed && digitalRead(BUTTON_PIN) == HIGH)
  {
    if (gameState.ammo > 0)
    {
      // Serial.println("shoot");
      sendGun();

      hasSentGun = true;
      hasAcknowledgedGun = false;
      timeoutStart = currentMillis;

      buttonPressed = true;
      sendIrSignal();
      --gameState.ammo;
    }
    else
    {
      // Serial.println("No ammo left. Please reload...");
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

// void getGameState()
// {
//   if (Serial.available() > 0)
//   {
//     String input = Serial.readStringUntil('\n');
//     int separatorIndex = input.indexOf(',');
//     if (separatorIndex != -1)
//     {
//       String healthStr = input.substring(0, separatorIndex);
//       String ammoStr = input.substring(separatorIndex + 1);
//       gameState.health = healthStr.toInt();
//       gameState.ammo = ammoStr.toInt();
//       Serial.print("Updated gameState - Health = ");
//       Serial.print(gameState.health);
//       Serial.print(", Ammo = ");
//       Serial.println(gameState.ammo);
//     }
//   }
// }

// Loop: first filter handshake traffic; only once CONFIRMED, process data

float computeMagnitude(float x, float y, float z)
{
  return sqrt(x * x + y * y + z * z);
}

float prevAccel[3] = {-2.53, 0.09, 7.73}; // -2.53, AccelYnorm:-0.09, AccelZnorm:7.73
// const float ACCEL_THRESHOLD = 9;
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

void initiateHandshake() {
  if (Serial.available()) {
    incoming = Serial.read();
    switch(incoming) {
      case HANDSHAKE:
        protoState = HANDSHAKE_INITIATED;
        if (protoState == DISCONNECTED || protoState == HANDSHAKE_INITIATED) {
          sendAck();
          protoState = WAITING_FOR_ACK;
        }
        break;
      case 'a':
        if (protoState == WAITING_FOR_ACK) {
          protoState = CONFIRMED;
        }
        break;
      case 'r':  // Reset command from Python, if any.
        protoState = DISCONNECTED;
        reset();
        break;

      default:
        // Ignore any other data.
        break;
    }
  }
}

void updateGameState() {
  if (Serial.available()) {
    incoming = Serial.read();
    
    // Check for an update packet header from Python.
    if (incoming == HEADER) {
      currentUpdateState = READING_UPDATE;
      updateIndex = 0;
      updateStartTime = millis();
      return;
    }
    
    if (currentUpdateState == READING_UPDATE) {
      if (millis() - updateStartTime > UPDATE_TIMEOUT) {
          currentUpdateState = IDLE;
          updateIndex = 0;
          return;
      } else {
          updateBuffer[updateIndex++] = incoming;
          if (updateIndex >= 3) {
              int value = (updateBuffer[2] << 8) | updateBuffer[1];  // little-endian
              switch (updateBuffer[0]) {
                case 'B':  
                  ammo = value;
                  break;
                case 'E':  
                  points = value;
                  break;
                case 'W':  
                  ammo = 6;
                  break;
              }
              currentUpdateState = IDLE;
          }
      }
    } else {
      // Process other non-update data (if any) here.
      switch (incoming) {
        case 'r':  // Reset state
          protoState = DISCONNECTED;
          reset();
          break;
        case 'g':
          // Gun ACK from Python.
          hasAcknowledgedGun = true;
          hasSentGun = false;          
          break;
        case 'H':
          hasAcknowledgedHit = true;
          hasSentHit = false;          // Hit ACK from Python.
          break;
        default:
          break;
      }
    }
  }
}

void loop() {  
  // If handshake is not confirmed, process only handshake-related bytes.
  if (protoState != CONFIRMED) {
    initiateHandshake();

    // Do nothing else until handshake is CONFIRMED.
    return;
  }

  // Assert: protoState is CONFIRMED
  // Now that handshake is CONFIRMED, process update packets FROM python
  updateGameState();

  imuData_t imuData = getImuReadings();

  // isMotion = isMotionDetected(imuData);
  // if (imuWindowIndex == IMU_WINDOW_SIZE && isMotion)
  // {
  //   imuWindowIndex = 0;
  // }

  // // send data in batch of IMU_WINDOW_SIZE
  // if (imuWindowIndex < IMU_WINDOW_SIZE) {
  //   sendData();
  //   ++imuWindowIndex;
  // }
  sendData();

  // temp auto reload
  if (gameState.ammo == 0)
  {
    // Serial.println("Reloading...");
    // delay(500);
    // Serial.println("Done reloading");
    gameState.ammo = 6;
  }

  unsigned long currentMillis = millis();

  if (!hasSentGun) {
    shootAmmo(currentMillis);
  }
  updateLed();

  if (hasSentGun && !hasAcknowledgedGun && (currentMillis - timeoutStart >= TIMEOUT_VAL)) {
      resendGunpacket();
      timeoutStart = currentMillis;
  }
  
  delay(50); // period = 50ms = 20Hz -> 20 samples per second
}

//---------------------------------------------------------
// Packet Sending Functions
//---------------------------------------------------------
void sendData() {
  // Serial.print("ax: ");
  // Serial.print(imuData.ax);
  // Serial.print(", ");
  // Serial.print("ay: ");
  // Serial.print(imuData.ay);
  // Serial.print(", ");
  // Serial.print("az: ");
  // Serial.print(imuData.az);
  // Serial.print(", ");
  // Serial.print("gx: ");
  // Serial.print(imuData.gx);
  // Serial.print(", ");
  // Serial.print("gy: ");
  // Serial.print(imuData.gy);
  // Serial.print(", ");
  // Serial.print("gz: ");
  // Serial.println(imuData.gz);

  Vector normGyro = mpu.readNormalizeGyro();
  Vector normAccel = mpu.readNormalizeAccel();

  Datapacket packet;
  packet.type = IMU_DATA;
  packet.aX = normAccel.XAxis;
  packet.aY = normAccel.YAxis;
  packet.aZ = normAccel.ZAxis;
  packet.gX = normGyro.XAxis;
  packet.gY = normGyro.YAxis;
  packet.gZ = normGyro.ZAxis;
  packet.y = buttonPressed ? 1 : 0;
  packet.p = 0;
  packet.r = 0;
  packet.start_move = 0; 
  packet.checksum = getChecksum(packet); 
  Serial.write((uint8_t *)&packet, sizeof(packet));
}

void sendAck() {
  Ackpacket packet;
  packet.type = ACK;
  packet.padding_1 = 0;
  packet.padding_2 = 0;
  packet.padding_3 = 0;
  packet.padding_4 = 0;
  packet.padding_5 = 0;
  packet.padding_6 = 0;
  packet.padding_7 = 0;
  packet.padding_8 = 0;
  packet.padding_9 = 0;
  packet.checksum = getAckChecksum(packet);
  Serial.write((uint8_t *)&packet, sizeof(packet));
}

void sendGun() {
  int index = random(0, 2); //toggle dummy data
  lastGunpacket.type = GUN;
  lastGunpacket.ammo_state = gameState.ammo;
  lastGunpacket.trigger_state = index; 
  lastGunpacket.padding_1 = 0;
  lastGunpacket.padding_2 = 0;
  lastGunpacket.padding_3 = 0;
  lastGunpacket.padding_4 = 0;
  lastGunpacket.padding_5 = 0;
  lastGunpacket.padding_6 = 0;
  lastGunpacket.padding_7 = 0;
  lastGunpacket.padding_8 = 0;
  lastGunpacket.checksum = getGunChecksum(lastGunpacket);
  Serial.write((uint8_t *)&lastGunpacket, sizeof(lastGunpacket));
}

void resendGunpacket() {
  Serial.write((uint8_t *)&lastGunpacket, sizeof(lastGunpacket));
}

long getChecksum(Datapacket packet){
  return packet.type ^ packet.aX ^ packet.aY ^ packet.aZ ^ packet.gX ^ packet.gY ^ packet.gZ ^ packet.y ^ packet.p ^ packet.r ^ packet.start_move;
}

long getAckChecksum(Ackpacket packet){
  return packet.type ^ packet.padding_1 ^ packet.padding_2 ^ packet.padding_3 ^ packet.padding_4 ^ packet.padding_5 ^ packet.padding_6 ^ packet.padding_7 ^ packet.padding_8 ^ packet.padding_9;
}

long getGunChecksum(Gunpacket lastGunpacket) {
    uint8_t *data = (uint8_t *)&lastGunpacket;
    int len = sizeof(Gunpacket) - sizeof(lastGunpacket.checksum);
    crc.restart();
    crc.add(data, len);
    return (long) crc.getCRC();
}
