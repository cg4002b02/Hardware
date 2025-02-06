/*
  Testing out IR emitter/receiver and button with Arduino Bluno Beetle
*/
#include <Arduino.h>
#include <IRremote.h>

// Pin Mapping
#define IR_RECEIVER_PIN 4
#define BUZZER_PIN 5

int got_shot = false;

void setup() {
    Serial.begin(115200);
    // Initialize IR
    // setupIR();

    // Buzzer
    pinMode(BUZZER_PIN, OUTPUT);
}

void setupIR() {
    IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK); // Enable feedback LED if available
    Serial.println("IR Receiver initialized.");
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
        Serial.println("No IR signal received.");
    }
}

void loop() {
    // receiveIRSignal();

    // test buzzer
    got_shot = !got_shot;
    if (got_shot) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(1000);
      digitalWrite(BUZZER_PIN, LOW);
    }

    delay(1000);
}