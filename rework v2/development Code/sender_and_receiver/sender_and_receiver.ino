#include <Arduino.h>

// Define the pin connected to the PPM signal (every Pin on Teensy 4.1 is interrupt compatible)
#define PPM_PIN 13

// Variables
volatile uint16_t channel1;
volatile uint16_t channel2;
volatile uint16_t channel3;
volatile uint16_t channel4;
volatile uint16_t channel5;
volatile uint16_t channel6;
volatile uint16_t channel7;
volatile uint16_t channel8;

// Define the number of channels and the total number of pulses
#define NUM_CHANNELS 8
#define TOTAL_PULSES (NUM_CHANNELS + 1)

// Define the PPM output pin
#define PPM_PIN_OUT 12
 
// Define the PPM frame duration in microseconds
#define FRAME_DURATION 20000

// Define the minimum and maximum pulse widths in microseconds
#define MIN_PULSE_WIDTH 1000
#define MAX_PULSE_WIDTH 2000

// Array to store the pulse widths for each channel
uint16_t pulseWidths[NUM_CHANNELS];

// Variables to track the start time of the frame
uint32_t frameStartTime;

// Variable to keep track of the current channel being read
volatile uint8_t currentChannel = 0;

void setup() {
  // Set the PPM output pin as an output
  Serial.begin(9600);
  pinMode(PPM_PIN_OUT, OUTPUT);
  digitalWrite(PPM_PIN_OUT, LOW);

  // Set the PPM input pin as an input
  pinMode(PPM_PIN, INPUT);

  // Attach an interrupt to the PPM input pin
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, FALLING);

  // Calculate the start time of the frame
  frameStartTime = micros();


}

void loop() {
  // Copy volatile variables to non-volatile array
  Serial.print("HI");
  noInterrupts();
  pulseWidths[0] = channel1;
  pulseWidths[1] = channel2;
  pulseWidths[2] = channel3;
  pulseWidths[3] = channel4;
  pulseWidths[4] = channel5;
  pulseWidths[5] = channel6;
  pulseWidths[6] = channel7;
  pulseWidths[7] = channel8;
  interrupts();

  // Generate PPM signal
  uint32_t pulseStartTime = micros();

  // Send the pulse widths for each channel
  for (int i = 0; i < NUM_CHANNELS; i++) {
    digitalWrite(PPM_PIN_OUT, HIGH);
    digitalWrite(PPM_PIN_OUT, LOW);
    delayMicroseconds(pulseWidths[i]);
  }

  // Calculate the time taken and adjust to match FRAME_DURATION
  uint32_t elapsedTime = micros() - pulseStartTime;
  uint32_t remainingTime = FRAME_DURATION - elapsedTime;

  if (remainingTime > 0) {
    digitalWrite(PPM_PIN_OUT, LOW);
    delayMicroseconds(remainingTime);
  }

  // Update frame start time
  frameStartTime = micros();

}

void readPPM() {
  static uint32_t lastTime = 0;  // Variable to store the time of the previous pulse
  uint32_t currentTime = micros();  // Get the current time in microseconds
  uint32_t interval = currentTime - lastTime;  // Calculate the time interval since the last pulse
  lastTime = currentTime;  // Update lastTime to the current time for the next interval calculation

  if (interval >= 3000) { // Check if the interval is 3000 microseconds or more
    // A sync pulse is detected (interval longer than 3 milliseconds)
    // This sync pulse indicates the start of a new frame period
    currentChannel = 0;  // Reset to the first channel
  } else {
    // If the interval is less than 3000 microseconds, process the pulse for the current channel
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
    currentChannel++;  // Move to the next channel
  }
}