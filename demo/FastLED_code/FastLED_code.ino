#include <MPU6050.h>
#include <Arduino.h>
#include <FastLED.h>
#include <IRremote.h>

#define LED_PIN     5
#define NUM_LEDS    10
#define BUTTON_PIN 2
#define IR_RECEIVER_PIN 4
#define IR_EMITTER_PIN 3

MPU6050 mpu;
IRsend irsend;
CRGB leds[NUM_LEDS];

int health = 10;
int ammo = 6;
bool buttonPressed = false;

void setup() {

  Serial.begin(115200);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(10);

  Serial.println("Init MPU6050");
  // while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  // {
  //   Serial.println("No MPU");
  //   delay(500);
  // }

  // receiver
  setupIR();

  // emitter
  setupIR2();

  // button
  Serial.println("init button pin");
  pinMode(BUTTON_PIN, INPUT);
 
  // mpu.calibrateGyro();
  // mpu.setThreshold(3);
}

void setupIR() {
    IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK); // Enable feedback LED if available
    Serial.println("IR Receiver initialized.");
}
void setupIR2() {
    IrSender.begin(IR_EMITTER_PIN);
    Serial.println("IR Transmitter initialized.");
}

void sendIRSignal() {
    uint8_t address = 0x1; // 1-byte player ID
    uint8_t command = millis() & 0xFF; // Simulating a unique command

    Serial.println("Sending IR signal...");
    IrSender.sendNEC(address, command, 0); // NEC protocol
    Serial.print("Sent NEC IR Signal - Address: ");
    Serial.print(address, HEX);
    Serial.print(", Command: ");
    Serial.println(command, HEX);
}

void receiveIRSignal() {
    if (IrReceiver.decode()) {
        Serial.println("IR signal received.");

        if (IrReceiver.decodedIRData.protocol == NEC) { // Check if NEC protocol is received
            uint8_t address = IrReceiver.decodedIRData.address;
            uint8_t command = IrReceiver.decodedIRData.command;

            Serial.print("Received NEC Signal - Address: ");
            Serial.print(address, HEX);
            Serial.print(", Command: ");
            Serial.println(command, HEX);
        } else {
            Serial.println("Unknown or unsupported IR protocol received.");
        }

        // Reset IR Receiver
        IrReceiver.resume();
        delay(10); // Short delay before receiving next signal
    } else {
        // Serial.println("No IR signal received.");
    }
}

void loop() {
  // Vector rawGyro = mpu.readRawGyro();
  // Vector normGyro = mpu.readNormalizeGyro();
 
  // Serial.print("Gyro Xnorm = ");
  // Serial.print(normGyro.XAxis);
  // Serial.print(" Gyro Ynorm = ");
  // Serial.print(normGyro.YAxis);
  // Serial.print(" Gyro Znorm = ");
  // Serial.println(normGyro.ZAxis);

  // Vector rawAccel = mpu.readRawAccel();
  // Vector normAccel = mpu.readNormalizeAccel();
 
  // Serial.print("Accel Xnorm = ");
  // Serial.print(normAccel.XAxis);
  // Serial.print(" Accel Ynorm = ");
  // Serial.print(normAccel.YAxis);
  // Serial.print(" Accel Znorm = ");
  // Serial.println(normAccel.ZAxis);
 
  // delay(50);

  if (health == -1) health = 10;
  if (ammo == -1) ammo = 6;

  // Serial.print(health);
  // 10 leds
  // for (int i = 0; i < health; i++) {
  //   if (health >= 7) 
  //     leds[i] = CRGB(0, 255, 0);
  //   else if (health >= 4)
  //     leds[i] = CRGB(255, 255, 0);
  //   else
  //     leds[i] = CRGB(255, 0, 0);
  // }


  // button logic
  if (!buttonPressed && digitalRead(BUTTON_PIN) == HIGH) {
    Serial.println("shoot");
    buttonPressed = true;
    // emitter
    sendIRSignal();
  }
  // uint8_t address = 0x1;
  // uint8_t command = 0x2;
  // irsend.sendNEC(address, command, 0);
  if (digitalRead(BUTTON_PIN) == LOW) {
    buttonPressed = false;
  }

  // receiver
  receiveIRSignal();


  // 6 leds for ammos
  for (int i = 0; i < ammo; i++) {
    if (ammo > 4)
      leds[i] = CRGB(0, 255, 0);
    else if (ammo > 2)
      leds[i] = CRGB(255, 255, 0);
    else
      leds[i] = CRGB(255, 0, 0);
  }

  for (int j = health; j < NUM_LEDS; j++) {
    leds[j] = CRGB(0, 0, 0);
  }
  FastLED.show();
  // delay(50);
  health--;

}