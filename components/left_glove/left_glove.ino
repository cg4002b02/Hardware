#include "CRC.h"
#include "CRC8.h" 
#include <MPU6050.h>
#include <Arduino.h>

#define HANDSHAKE 'h'
#define ACK 1
#define IMU_DATA 2
#define HIT 3
#define GUN 4 
#define HEADER '$'
#define IMU_WINDOW_SIZE 30

CRC8 crc(0x07);

//void (*reset) (void) = 0;

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

char incoming;

MPU6050 mpu;

struct imuData_t
{
  float ax, ay, az; // Acceleration
  float gx, gy, gz; // Gyroscope
};

bool isMotion = false;
int16_t imuWindowIndex = IMU_WINDOW_SIZE;

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
    setup_mpu();
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

// Loop: first filter handshake traffic; only once CONFIRMED, process data

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
        //reset();
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
          //reset();
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
bool initialGryoCalibrated = false;

void loop() {
  // If handshake is not confirmed, process only handshake-related bytes.
  if (protoState != CONFIRMED) {
    initiateHandshake();

    // Do nothing else until handshake is CONFIRMED or when gyro is sending values for all axis.
    return;
  }

  // Assert: protoState is CONFIRMED
  // Now that handshake is CONFIRMED, process update packets FROM python
  updateGameState();

  // stream data
  sendData();
  
  delay(50); // period = 50ms = 20Hz -> 20 samples per second
}

//0x506583775D62

//---------------------------------------------------------
// Packet Sending Functions
//---------------------------------------------------------
void sendData(){
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
  packet.y = 0;
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

int16_t getChecksum(Datapacket packet){
  return packet.type ^ packet.aX ^ packet.aY ^ packet.aZ ^ packet.gX ^ packet.gY ^ packet.gZ ^ packet.y ^ packet.p ^ packet.r ^ packet.start_move;
}

int16_t getAckChecksum(Ackpacket packet){
  return packet.type ^ packet.padding_1 ^ packet.padding_2 ^ packet.padding_3 ^ packet.padding_4 ^ packet.padding_5 ^ packet.padding_6 ^ packet.padding_7 ^ packet.padding_8 ^ packet.padding_9;
}
