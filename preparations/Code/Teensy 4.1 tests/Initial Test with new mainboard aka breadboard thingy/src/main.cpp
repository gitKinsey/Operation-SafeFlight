#include<Arduino.h>
#include<Wire.h>
// Define the pin connected to the PPM signal
#define PPM_PIN 2

// Array to store channel values
volatile uint16_t ppmValues[8];  // Assuming 8 channels for this example
volatile uint8_t currentChannel = 0;

void readPPM();

void setup() {

  Serial.begin(9600);  // Start serial communication for debugging
  Serial.print("Start of programm ");
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, FALLING);  // Set up an interrupt on the falling edge
}

void loop() {
  // Print PPM values
  for (int i = 0; i < 8; i++) {
    Serial.print("Channel ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(ppmValues[i]);
    Serial.print(" us\t");
  }
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
  } else if (currentChannel < 8) {
    ppmValues[currentChannel] = interval;
    currentChannel++;
  }
}
