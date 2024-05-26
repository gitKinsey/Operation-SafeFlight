#include <Arduino.h>
#include <Wire.h>
#include <TimerOne.h> // Include the TimerOne library

// Define the pin connected to the PPM signal input
#define PPM_IN_PIN 2
// Define the pin connected to the PPM signal output
#define PPM_OUT_PIN 9

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

// PPM settings
const uint16_t PPM_FRAME_LENGTH = 22500;  // Total length of the PPM frame in microseconds
const uint16_t PPM_SYNC_PULSE_LENGTH = 300;  // Length of the sync pulse in microseconds
const uint16_t PPM_PAUSE_LENGTH = 300;  // Length of the pause between pulses in microseconds

void readPPM();
void sendPPM();

void setup() {
  Serial.begin(9600);  // Start serial communication for debugging
  Serial.println("Start of program");
  pinMode(PPM_IN_PIN, INPUT);
  pinMode(PPM_OUT_PIN, OUTPUT);
  digitalWrite(PPM_OUT_PIN, HIGH);  // Start with the output pin high
  
  attachInterrupt(digitalPinToInterrupt(PPM_IN_PIN), readPPM, FALLING);  // Set up an interrupt on the falling edge

  // Set up Timer1 for PPM output generation
  Timer1.initialize(100); // Initialize Timer1 with a period of 100 microseconds (will be updated dynamically)
  Timer1.attachInterrupt(sendPPM); // Attach the interrupt service routine (ISR)
}

void loop() {
  // Example condition to modify channel1 if it is greater than 1500
  if (channel1 > 1500) {
    channel1 -= 100;
  }

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

void sendPPM() {
  static uint8_t channelIndex = 0;
  static uint32_t lastMicros = 0;
  static bool state = true;
  static uint32_t nextPulseTime = PPM_SYNC_PULSE_LENGTH;

  uint16_t channelValues[] = {
    channel1, channel2, channel3, channel4,
    channel5, channel6, channel7, channel8
  };

  uint32_t currentMicros = micros();

  if (state) {
    // Generate the pulse
    digitalWrite(PPM_OUT_PIN, LOW);
    Timer1.setPeriod(PPM_PAUSE_LENGTH);  // Set the period for the low pulse
    state = false;
  } else {
    // Generate the space between pulses
    digitalWrite(PPM_OUT_PIN, HIGH);

    if (channelIndex < 8) {
      nextPulseTime = channelValues[channelIndex] - PPM_PAUSE_LENGTH;
      Timer1.setPeriod(nextPulseTime);  // Set the period for the high pulse
      channelIndex++;
    } else {
      // All channels sent, reset for the next frame
      nextPulseTime = PPM_FRAME_LENGTH - (currentMicros - lastMicros);
      Timer1.setPeriod(nextPulseTime);  // Set the period for the high pulse
      channelIndex = 0;
      lastMicros = currentMicros;
    }

    state = true;
  }
}
