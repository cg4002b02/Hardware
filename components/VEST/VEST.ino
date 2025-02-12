/*
  Testing out IR emitter/receiver and button with Arduino Bluno Beetle
*/
#include <Arduino.h>
#include <IRremote.h>
#include <FastLED.h>
#include <IRremote.h>

#define IR_RECEIVER_PIN     4
#define BUZZER_PIN          5
#define LED_PIN             2

#define BULLET_DAMAGE       5
#define NUM_LEDS            10
#define LED_BRIGHTNESS      10

// Global variables
CRGB leds[NUM_LEDS];
int got_shot = false;
int health = 100;

void setup() {
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

void setup_led() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);

  update_led();
}

void setup_ir_receiver() {
    IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK); // Enable feedback LED if available
    Serial.println("IR Receiver initialized.");
}

void receive_ir_signal() {
    if (IrReceiver.decode()) {
        Serial.println("IR signal received.");

        Serial.print("IR Protocol: ");
        Serial.println(IrReceiver.decodedIRData.protocol);
        if (IrReceiver.decodedIRData.protocol == NEC) { // Check if NEC protocol is received
            uint8_t address = IrReceiver.decodedIRData.address;
            uint8_t command = IrReceiver.decodedIRData.command;

            Serial.print("Received NEC Signal - Address: ");
            Serial.print(address, HEX);
            Serial.print(", Command: ");
            Serial.println(command, HEX);

            got_shot = true;
            if (health > 0) {
              health -= BULLET_DAMAGE;
              update_led();
              play_buzzer();
            }
        } else {
            Serial.println("Unknown or unsupported IR protocol received.");
        }

        // Reset IR Receiver
        // IrReceiver.resume();
        IrReceiver.begin(IR_RECEIVER_PIN);
        delay(10); // Short delay before receiving next signal
    } else {
        // Serial.println("No IR signal received.");
    }
}

void update_led() {
  int num_full_led = health / 10;
  bool has_half_led = health % 2 == 1;

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

void play_buzzer() {
    // tone(BUZZER_PIN, 1000, 1000);
    // play buzzer for 1 second
    tone(BUZZER_PIN, 1000); // Start playing a 1000 Hz tone
    delay(100);            // Wait for 1 second
    noTone(BUZZER_PIN);     // Stop playing the tone
}

void loop() {
    receive_ir_signal();

    // test buzzer
    // got_shot = !got_shot;
    // if (got_shot && health > 0) {
    //     health -= BULLET_DAMAGE;
    //     update_led();
    //     play_buzzer();
    // }

    // got_shot = false;
    // delay(50);
}