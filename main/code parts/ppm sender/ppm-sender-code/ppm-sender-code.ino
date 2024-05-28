#include <Arduino.h>

// PPM settings
#define PPM_OUT_PIN 2  // Output pin for PPM signal
#define NUM_CHANNELS 8  // Number of PPM channels
#define PPM_PERIOD 20000  // Total PPM frame length in microseconds (20ms)
#define PULSE_LENGTH 300  // Length of sync pulse in microseconds
#define MIN_CHANNEL_PULSE 1000  // Minimum channel pulse length in microseconds
#define MAX_CHANNEL_PULSE 2000  // Maximum channel pulse length in microseconds

// Channel values (in microseconds)
uint16_t channelValues[NUM_CHANNELS] = {1300, 1800, 1600, 1680, 1700, 1600, 1500, 1500};

void setup() {
  pinMode(PPM_OUT_PIN, OUTPUT);
  digitalWrite(PPM_OUT_PIN, HIGH);
}

void loop() {
  uint32_t frameStartTime = micros();
  uint32_t lastPulseEndTime = frameStartTime;
  uint32_t currentTime;

  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    // Calculate the exact time to wait before the next pulse
    uint32_t pulseStartTime = lastPulseEndTime + (channelValues[i] - PULSE_LENGTH);

    // Wait for the time to send the next pulse
    while (micros() < pulseStartTime) {
      // Busy wait
    }

    // Send the channel pulse
    digitalWrite(PPM_OUT_PIN, LOW);
    delayMicroseconds(PULSE_LENGTH);
    digitalWrite(PPM_OUT_PIN, HIGH);

    // Update the last pulse end time
    lastPulseEndTime = pulseStartTime + PULSE_LENGTH;
  }

  // Calculate remaining time for the sync pulse
  uint32_t timeSpent = micros() - frameStartTime;
  uint32_t syncPulseLength = PPM_PERIOD - timeSpent;

  // Ensure sync pulse length is at least the pulse length
  if (syncPulseLength > PULSE_LENGTH) {
    delayMicroseconds(syncPulseLength - PULSE_LENGTH);
  }

  // Send the sync pulse
  digitalWrite(PPM_OUT_PIN, LOW);
  delayMicroseconds(PULSE_LENGTH);
  digitalWrite(PPM_OUT_PIN, HIGH);
}
