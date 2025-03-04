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
#define IMU_WINDOW_SIZE 20

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
boolean hasSentGun = false;
boolean hasAcknowledgedGun = false;
unsigned long timeoutStart = 0;
const unsigned long TIMEOUT_VAL = 500;
unsigned long lastGunSendTime = 0;
unsigned long lastIMUSendTime = 0;
const unsigned long IMU_INTERVAL = 1100;
boolean lastsentGunindex = true;

// -----------------------------------------------------------------------------
// Stop-and-wait for HIT packet
// -----------------------------------------------------------------------------
boolean hasSentHit = false;
boolean hasAcknowledgedHit = false;
unsigned long timeoutStartHit = 0;
const unsigned long TIMEOUT_VALHIT = 750;
unsigned long lastHitSendTime = 0;
boolean lastsentHitindex = true;

// Sensor dummy data (randomise between 3 for testing)
int accX[][1] = {{1}, {5}, {-3}};
int accY[][1] = {{2}, {6}, {-4}};
int accZ[][1] = {{4}, {7}, {-2}};
int gyrX[][1] = {{100}, {-200}, {50}};
int gyrY[][1] = {{-150}, {250}, {-100}};
int gyrZ[][1] = {{100}, {-300}, {200}};
//int yaw[][1] = {{340}, {90}, {180}};
//int pitch[][1] = {{200}, {150}, {50}};
//int roll[][1] = {{134}, {45}, {90}};

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

struct Hitpacket {
  int8_t type;
  int8_t hit_status;
  int16_t health_status;
  int16_t padding_1;
  int16_t padding_2;
  int16_t padding_3;
  int16_t padding_4;
  int16_t padding_5;
  int16_t padding_6;
  int16_t padding_7;
  int8_t padding_8;
  int8_t checksum;
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
Hitpacket lastHitpacket;

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
    while (Serial.available() > 0) {
      Serial.read();
    }
    //reset();
    setup_mpu();
    protoState = DISCONNECTED;
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


void loop(){
  char incoming;
  
  // If handshake is not confirmed, process only handshake-related bytes.
  if (protoState != CONFIRMED) {
    while (Serial.available()) {
      incoming = Serial.read();
      switch(incoming) {
        case HANDSHAKE:
          protoState = HANDSHAKE_INITIATED;
          if (protoState == DISCONNECTED || protoState == HANDSHAKE_INITIATED) {
            sendack();
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
    // Do nothing else until handshake is CONFIRMED.
    return;
  }


  imuData_t imuData = getImuReadings();

  isMotion = isMotionDetected(imuData);

  if (imuWindowIndex == IMU_WINDOW_SIZE && isMotion)
  {
    imuWindowIndex = 0;
  }

  if (protoState == CONFIRMED) {
    if (imuWindowIndex < IMU_WINDOW_SIZE) {
      senddata(imuData);
      // Serial.print("sending packet #");
      // Serial.println(imuWindowIndex);
      ++imuWindowIndex;      
    }
  }
  
  // Now that handshake is CONFIRMED, process update packets FROM python
  while (Serial.available()) {
    incoming = Serial.read();
    
    // Check for an update packet header from Python.
    if (incoming == HEADER) {
      currentUpdateState = READING_UPDATE;
      updateIndex = 0;
      updateStartTime = millis();
      continue;
    }
    
    if (currentUpdateState == READING_UPDATE) {
      if (millis() - updateStartTime > UPDATE_TIMEOUT) {
          currentUpdateState = IDLE;
          updateIndex = 0;
          continue;
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
  
  // Process periodic tasks (sending sensor data, gun/hit packets, etc.) if handshake is CONFIRMED.
  unsigned long currentMillis = millis();

  if (protoState == CONFIRMED && !hasSentGun && (currentMillis - lastGunSendTime >= 1000)) {
      sendgun();
      if (ammo > 0){
        ammo -= 1;
      } else {
        ammo = 0;
      }
      lastGunSendTime = currentMillis;
      hasSentGun = true;
      hasAcknowledgedGun = false;
      timeoutStart = lastGunSendTime;
  }

  if (protoState == CONFIRMED && hasSentGun && !hasAcknowledgedGun && (currentMillis - timeoutStart >= TIMEOUT_VAL)) {
      resendGunpacket();
      timeoutStart = currentMillis;
      lastGunSendTime = currentMillis;
  }

  if (protoState == CONFIRMED && !hasSentHit && (currentMillis - lastHitSendTime >= 1500)) {
      sendhit();
      lastHitSendTime = currentMillis;
      hasSentHit = true;
      hasAcknowledgedHit = false;
      timeoutStartHit = lastHitSendTime;
  }

  if (protoState == CONFIRMED && hasSentHit && !hasAcknowledgedHit && (currentMillis - timeoutStartHit >= TIMEOUT_VALHIT)) {
      resendHitpacket();
      timeoutStartHit = currentMillis;
      lastHitSendTime = currentMillis;
  }

  delay(50); // period = 50ms = 20Hz -> 20 samples per second
}

//---------------------------------------------------------
// Packet Sending Functions
//---------------------------------------------------------
void senddata(imuData_t imuData){
  // Serial.println("inside senddata");
  Datapacket packet;
  packet.type = IMU_DATA;
  int index = random(0, 3); //toggle dummy data
  packet.aX = imuData.ax;
  packet.aY = imuData.ay;
  packet.aZ = imuData.az;
  packet.gX = imuData.gx;
  packet.gY = imuData.gy;
  packet.gZ = imuData.gz;
  packet.y = 0;
  packet.p = 0;
  packet.r = 0;
  packet.start_move = 0; 
  packet.checksum = getChecksum(packet); 
  Serial.write((uint8_t *)&packet, sizeof(packet));
}

void sendack() {
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

void sendhit() {
  int index = random(0, 3); //toggle dummy data
  lastHitpacket.type = HIT;
  lastHitpacket.hit_status = 0;
  lastHitpacket.health_status = points; 
  lastHitpacket.padding_1 = 0;
  lastHitpacket.padding_2 = 0;
  lastHitpacket.padding_3 = 0;
  lastHitpacket.padding_4 = 0;
  lastHitpacket.padding_5 = 0;
  lastHitpacket.padding_6 = 0;
  lastHitpacket.padding_7 = 0;
  lastHitpacket.padding_8 = 0;
  lastHitpacket.checksum = getHitChecksum(lastHitpacket);
  Serial.write((uint8_t *)&lastHitpacket, sizeof(lastHitpacket));
}

void sendgun() {
  int index = random(0, 2); //toggle dummy data
  lastGunpacket.type = GUN;
  lastGunpacket.ammo_state = ammo;
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

void resendHitpacket() {
  Serial.write((uint8_t *)&lastHitpacket, sizeof(lastHitpacket));
}

long getChecksum(Datapacket packet){
  return packet.type ^ packet.aX ^ packet.aY ^ packet.aZ ^ packet.gX ^ packet.gY ^ packet.gZ ^ packet.y ^ packet.p ^ packet.r ^ packet.start_move;
}

long getAckChecksum(Ackpacket packet){
  return packet.type ^ packet.padding_1 ^ packet.padding_2 ^ packet.padding_3 ^ packet.padding_4 ^ packet.padding_5 ^ packet.padding_6 ^ packet.padding_7 ^ packet.padding_8 ^ packet.padding_9;
}

long getHitChecksum(Hitpacket lastHitpacket) {
    uint8_t *data = (uint8_t *)&lastHitpacket;
    int len = sizeof(Hitpacket) - sizeof(lastHitpacket.checksum);
    crc.restart();
    crc.add(data, len);
    return (long) crc.getCRC();
}

long getGunChecksum(Gunpacket lastGunpacket) {
    uint8_t *data = (uint8_t *)&lastGunpacket;
    int len = sizeof(Gunpacket) - sizeof(lastGunpacket.checksum);
    crc.restart();
    crc.add(data, len);
    return (long) crc.getCRC();
}
