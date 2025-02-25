#include <Arduino.h>
#include <IRremote.h>
#include <FastLED.h>
#include <IRremote.h>

#define IR_RECEIVER_PIN 4
#define BUZZER_PIN 5
#define LED_PIN 2

#define BULLET_DAMAGE 5
#define NUM_LEDS 10
#define LED_BRIGHTNESS 10

// Global variables
CRGB leds[NUM_LEDS];

struct gameState_t
{
  int health;
  int ammo;
};

gameState_t gameState = {100, 6};
int got_shot = false;

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

  update_led();
}

void setup_ir_receiver()
{
  IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK); // Enable feedback LED if available
  Serial.println("IR Receiver initialized.");
}

void receive_ir_signal()
{
  if (IrReceiver.decode())
  {
    Serial.println("IR signal received.");

    Serial.print("IR Protocol: ");
    Serial.println(IrReceiver.decodedIRData.protocol);
    if (IrReceiver.decodedIRData.protocol == NEC)
    { // Check if NEC protocol is received
      uint8_t address = IrReceiver.decodedIRData.address;
      uint8_t command = IrReceiver.decodedIRData.command;

      if (address != 0x11 || command != 0x22)
      {
        Serial.println("Invalid address or command received.");
        IrReceiver.begin(IR_RECEIVER_PIN);
        return;
      }

      Serial.print("Received NEC Signal - Address: ");
      Serial.print(address, HEX);
      Serial.print(", Command: ");
      Serial.println(command, HEX);

      got_shot = true;
      if (gameState.health > 0)
      {
        gameState.health -= BULLET_DAMAGE;
        play_buzzer();
      }
    }
    else
    {
      Serial.println("Unknown or unsupported IR protocol received.");
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

void getGameState() {
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
      Serial.print("Updated gameState - Health: ");
      Serial.print(gameState.health);
      Serial.print(", Ammo: ");
      Serial.println(gameState.ammo);
    }
  }
}

void loop()
{
  receive_ir_signal();
  update_led();

  // mimic receiving gameState from game engine
  getGameState();

  // delay(50);
}