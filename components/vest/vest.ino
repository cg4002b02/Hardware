#include "CRC.h"
#include "CRC8.h"
#include <Arduino.h>
#include <IRremote.h>
#include <FastLED.h>
#include <IRremote.h>

#define HANDSHAKE 'h'
#define ACK 1
#define IMU_DATA 2
#define HIT 3
#define GUN 4 
#define HEADER '$'
#define IMU_WINDOW_SIZE 30

#define IR_RECEIVER_PIN 4
#define BUZZER_PIN 5
#define LED_PIN 2

#define BULLET_DAMAGE 5
#define NUM_LEDS 10
#define LED_BRIGHTNESS 10

// Global variables

CRC8 crc(0x07);

void (*reset) (void) = 0;

enum ProtoState { DISCONNECTED, HANDSHAKE_INITIATED, WAITING_FOR_ACK, CONFIRMED};
ProtoState protoState = DISCONNECTED;

enum UpdateState { IDLE, READING_UPDATE };
UpdateState currentUpdateState = IDLE;  
byte updateBuffer[3];
int updateIndex = 0;  
unsigned long updateStartTime = 0;           
const unsigned long UPDATE_TIMEOUT = 500;    

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

Hitpacket lastHitpacket;
char incoming;



CRGB leds[NUM_LEDS];

struct gameState_t
{
  int health;
  int ammo;
};

gameState_t gameState = {100, 6};
int got_shot = false;

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
boolean hasSentHit = false;
boolean hasAcknowledgedHit = false;
unsigned long timeoutStartHit = 0;
const unsigned long TIMEOUT_VALHIT = 750;
const unsigned long TIMEOUT_LAST_SENT_HIT = 2000;
unsigned long lastHitSendTime = 0;
boolean lastsentHitindex = true;

void setup()
{
  Serial.begin(115200);

  Serial.println("Initializing...");
  // LED strip setup
  setup_led();

  // IR receiver
  setup_ir_receiver();

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  Serial.println("Done init");
}

void setup_led()
{
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);

  white_led();
}

void setup_ir_receiver()
{
  IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK); // Enable feedback LED if available
  Serial.println("IR Receiver initialized.");
}

void receive_ir_signal(unsigned long currentMillis)
{
  if (IrReceiver.decode())
  {
    // Serial.println("IR signal received.");

    // Serial.print("IR Protocol: ");
    // Serial.println(IrReceiver.decodedIRData.protocol);
    if (IrReceiver.decodedIRData.protocol == NEC)
    { // Check if NEC protocol is received
      uint8_t address = IrReceiver.decodedIRData.address;
      uint8_t command = IrReceiver.decodedIRData.command;

      if (address != 0x11 || command != 0x22)
      {
        // Serial.println("Invalid address or command received.");
        IrReceiver.begin(IR_RECEIVER_PIN);
        return;
      }

      // Serial.print("Received NEC Signal - Address: ");
      // Serial.print(address, HEX);
      // Serial.print(", Command: ");
      // Serial.println(command, HEX);

      sendhit();
      got_shot = true;
      timeoutStartHit = currentMillis;
      hasSentHit = true;
      hasAcknowledgedHit = false;

      // TODO: might not need since getting gameStates from python, but have to trigger play_buzzer() when shot. Maybe if previous health != current health, play_buzzer()
      if (gameState.health > 0)
      {
        gameState.health -= BULLET_DAMAGE;
        play_buzzer();
      }
    }
    else
    {
      // Serial.println("Unknown or unsupported IR protocol received.");
    }

    // Reset IR Receiver
    // IrReceiver.resume();
    IrReceiver.begin(IR_RECEIVER_PIN);
    delay(10); // Short delay before receiving next signal
  }
  else
  {
    // Serial.println("No IR signal received.");
  }
}

void white_led() {
  for (int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = CRGB(255, 255, 255);
  }
}

void update_led()
{
  int num_full_led = gameState.health / 10;
  bool has_half_led = gameState.health % 2 == 1;

  for (int i = 0; i < num_full_led; ++i)
  {
    leds[i] = CRGB(0, 255, 0);
  }
  for (int j = num_full_led; j < NUM_LEDS; ++j)
  {
    leds[j] = CRGB(0, 0, 0);
  }
  if (has_half_led)
  {
    leds[num_full_led] = CRGB(255, 255, 0);
  }

  FastLED.show();
}

void play_buzzer()
{
  tone(BUZZER_PIN, 1000); // Start playing a 1000 Hz tone
  delay(100);             // Wait for 100 ms
  noTone(BUZZER_PIN);     // Stop playing the tone
}

// void getGameState() {
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
//       Serial.print("Updated gameState - Health: ");
//       Serial.print(gameState.health);
//       Serial.print(", Ammo: ");
//       Serial.println(gameState.ammo);
//     }
//   }
// }

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
                  gameState.ammo = value;
                  break;
                case 'E':  
                  gameState.health = value;
                  break;
                case 'W':  
                  gameState.ammo = 6;
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

long getAckChecksum(Ackpacket packet){
  return packet.type ^ packet.padding_1 ^ packet.padding_2 ^ packet.padding_3 ^ packet.padding_4 ^ packet.padding_5 ^ packet.padding_6 ^ packet.padding_7 ^ packet.padding_8 ^ packet.padding_9;
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

void sendhit() {
  int index = random(0, 3); //toggle dummy data
  lastHitpacket.type = HIT;
  lastHitpacket.hit_status = 0;
  lastHitpacket.health_status = gameState.health; 
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

long getHitChecksum(Hitpacket lastHitpacket) {
  uint8_t *data = (uint8_t *)&lastHitpacket;
  int len = sizeof(Hitpacket) - sizeof(lastHitpacket.checksum);
  crc.restart();
  crc.add(data, len);
  return (long) crc.getCRC();
}


void resendHitpacket() {
  Serial.write((uint8_t *)&lastHitpacket, sizeof(lastHitpacket));
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

  unsigned long currentMillis = millis();

  if (!hasSentHit) {
    receive_ir_signal(currentMillis);
    // sendhit();
    // lastHitSendTime = currentMillis;
    // hasSentHit = true;
    // hasAcknowledgedHit = false;
    // timeoutStartHit = lastHitSendTime;
  }

  // sendhit every 2 seconds
  if (currentMillis - lastHitSendTime >= TIMEOUT_LAST_SENT_HIT) {
    sendhit();
    lastHitSendTime = currentMillis;
  }

  if (hasSentHit && !hasAcknowledgedHit && (currentMillis - timeoutStartHit >= TIMEOUT_VALHIT)) {
      resendHitpacket();
      timeoutStartHit = currentMillis;
  }

  update_led();

  // delay(50);
}
