#include <Arduino.h>
#include <Wire.h>

// Define the pin connected to the PPM signal
#define PPM_PIN 2

// Separate variables for each channel
volatile uint16_t channel1;
volatile uint16_t channel2;
volatile uint16_t channel3;
volatile uint16_t channel4;
volatile uint16_t channel5;
volatile uint16_t channel6;
volatile uint16_t channel7;
volatile uint16_t channel8;

volatile uint8_t currentChannel = 0;

void readPPM();

void setup() {
  Serial.begin(9600);  // Start serial communication for debugging
  Serial.println("Start of program");
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, FALLING);  // Set up an interrupt on the falling edge
}

void loop() {
  // Print PPM values
  Serial.print("Channel 1: ");
  Serial.print(channel1);
  Serial.print(" us\t");

  Serial.print("Channel 2: ");
  Serial.print(channel2);
  Serial.print(" us\t");

  Serial.print("Channel 3: ");
  Serial.print(channel3);
  Serial.print(" us\t");

  Serial.print("Channel 4: ");
  Serial.print(channel4);
  Serial.print(" us\t");

  Serial.print("Channel 5: ");
  Serial.print(channel5);
  Serial.print(" us\t");

  Serial.print("Channel 6: ");
  Serial.print(channel6);
  Serial.print(" us\t");

  Serial.print("Channel 7: ");
  Serial.print(channel7);
  Serial.print(" us\t");

  Serial.print("Channel 8: ");
  Serial.print(channel8);
  Serial.print(" us\t");

  Serial.println();
  delay(20);  // Adjust as needed
}

void readPPM() {
  static uint32_t lastTime = 0;
  uint32_t currentTime = micros();  // Current time in microseconds
  uint32_t interval = currentTime - lastTime;
  lastTime = currentTime;

  if (interval >= 3000) {
    // Sync pulse detected (interval longer than a frame period, e.g., 3 ms)
    currentChannel = 0;
  } else {
    switch (currentChannel) {
      case 0:
        channel1 = interval;
        break;
      case 1:
        channel2 = interval;
        break;
      case 2:
        channel3 = interval;
        break;
      case 3:
        channel4 = interval;
        break;
      case 4:
        channel5 = interval;
        break;
      case 5:
        channel6 = interval;
        break;
      case 6:
        channel7 = interval;
        break;
      case 7:
        channel8 = interval;
        break;
    }
    currentChannel++;
  }
}
