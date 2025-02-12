#include <MPU6050.h>
#include <Arduino.h>
#include <FastLED.h>
#include <IRremote.h>

#define LED_PIN           5
#define BUTTON_PIN        2
#define IR_RECEIVER_PIN   4
#define IR_EMITTER_PIN    3

#define NUM_LEDS          3
#define LED_BRIGHTNESS      10

MPU6050 mpu;
// IRsend irsend;
CRGB leds[NUM_LEDS];

int health = 10;
int ammo = 6;
bool buttonPressed = false;

void setup() {

  Serial.begin(115200);

  // LED strip setup
  setup_led();

  // MPU setup
  setup_mpu();

  // IR receiver
  // setup_ir_receiver();

  // IR transmitter
  setup_ir_transmitter();

  // Button
  pinMode(BUTTON_PIN, INPUT);
}

void setup_led() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
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

// void setup_ir_receiver() {
//     IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK); // Enable feedback LED if available
//     Serial.println("IR Receiver initialized.");
// }
void setup_ir_transmitter() {
    IrSender.begin(IR_EMITTER_PIN);
    Serial.println("IR Transmitter initialized.");
}

void send_ir_signal() {
    uint8_t address = 0x1; // 1-byte player ID
    uint8_t command = millis() & 0xFF; // Simulating a unique command

    Serial.println("Sending IR signal...");
    IrSender.sendNEC(address, command, 0); // NEC protocol
    Serial.print("Sent NEC IR Signal - Address: ");
    Serial.print(address, HEX);
    Serial.print(", Command: ");
    Serial.println(command, HEX);
}

// void receive_ir_signal() {
//     if (IrReceiver.decode()) {
//         Serial.println("IR signal received.");

//         if (IrReceiver.decodedIRData.protocol == NEC) { // Check if NEC protocol is received
//             uint8_t address = IrReceiver.decodedIRData.address;
//             uint8_t command = IrReceiver.decodedIRData.command;

//             Serial.print("Received NEC Signal - Address: ");
//             Serial.print(address, HEX);
//             Serial.print(", Command: ");
//             Serial.println(command, HEX);
//         } else {
//             Serial.println("Unknown or unsupported IR protocol received.");
//         }

//         // Reset IR Receiver
//         IrReceiver.resume();
//         delay(10); // Short delay before receiving next signal
//     } else {
//         // Serial.println("No IR signal received.");
//     }
// }

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
 
  // delay(50);
}

void shoot_ammo() {
  if (!buttonPressed && digitalRead(BUTTON_PIN) == HIGH) {
    if (ammo > 0) {
      Serial.println("shoot");
      buttonPressed = true;
      send_ir_signal();
      --ammo;
    } else {
      Serial.println("No ammo left. Please reload...");
    }
  }
  if (digitalRead(BUTTON_PIN) == LOW) {
    buttonPressed = false;
  }
}

void update_led() {
  int num_full_led = ammo / 2;
  bool has_half_led = ammo % 2 == 1;

  for (int i = 0; i < num_full_led; ++i) {
    leds[i] = CRGB(0, 255, 0);
  }
  for (int j = num_full_led; j < NUM_LEDS; ++j) {
    leds[j] = CRGB(0, 0, 0);
  }
  if (has_half_led) {
    leds[num_full_led] = CRGB(255, 255, 0);
  }

  FastLED.show();
}

void loop() {
  get_imu_readings();

  // button logic
  shoot_ammo();

  update_led();
  if (ammo == 0) {
    Serial.println("Reloading...");
    delay(1500);
    Serial.println("Done reloading");
    ammo = 6;
  }

  delay(50);
}