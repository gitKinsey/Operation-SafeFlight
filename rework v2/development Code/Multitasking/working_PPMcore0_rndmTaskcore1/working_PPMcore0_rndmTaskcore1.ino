#include <Arduino.h>
#include <FreeRTOS.h>

// Define the pin connected to the PPM signal
#define PPM_PIN 13

// Variables to store the pulse width for each channel
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

// Interrupt handler for the PPM signal
void readPPM() {
  static uint32_t lastTime = 0;  // Variable to store the time of the previous pulse
  uint32_t currentTime = micros();  // Get the current time in microseconds
  uint32_t interval = currentTime - lastTime;  // Calculate the time interval since the last pulse
  lastTime = currentTime;  // Update lastTime to the current time for the next interval calculation

  if (interval >= 3000) { // Check if the interval is 3000 microseconds or more (sync pulse)
    currentChannel = 0;  // Reset to the first channel
  } else {
    // Process the pulse for the current channel
    switch (currentChannel) {
      case 0: channel1 = interval; break;
      case 1: channel2 = interval; break;
      case 2: channel3 = interval; break;
      case 3: channel4 = interval; break;
      case 4: channel5 = interval; break;
      case 5: channel6 = interval; break;
      case 6: channel7 = interval; break;
      case 7: channel8 = interval; break;
    }
    currentChannel++;  // Move to the next channel
  }
}

// PPM reading and signal generation task (runs on Core 0)
void readAndGeneratePPMTask(void *pvParameters) {
  while (true) {
    // Copy the volatile variables to non-volatile for signal generation
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
      delayMicroseconds(pulseWidths[i]);
      digitalWrite(PPM_PIN_OUT, LOW);
    }

    // Calculate the time taken and adjust to match FRAME_DURATION
    uint32_t elapsedTime = micros() - pulseStartTime;
    uint32_t remainingTime = FRAME_DURATION - elapsedTime;

    if (remainingTime > 0) {
      digitalWrite(PPM_PIN_OUT, LOW);
      delayMicroseconds(remainingTime);
    }

    vTaskDelay(20 / portTICK_PERIOD_MS); // Prevent overloading Core 0
  }
}

// Simple task on Core 1 (just incrementing a number)
void incrementTask(void *pvParameters) {
  uint32_t count = 0;
  while (true) {
    // Simply increment a counter and print it
    count++;
    Serial.print("Count: ");
    Serial.println(count);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
  }
}

void setup() {
  // Set up serial and I/O pins
  Serial.begin(9600);
  pinMode(PPM_PIN_OUT, OUTPUT);
  digitalWrite(PPM_PIN_OUT, LOW);
  pinMode(PPM_PIN, INPUT);

  // Attach interrupt for PPM signal
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, FALLING);

  // Start the FreeRTOS tasks
  xTaskCreatePinnedToCore(readAndGeneratePPMTask, "PPM Task", 2048, NULL, 1, NULL, 0); // Core 0
  xTaskCreatePinnedToCore(incrementTask, "Increment Task", 1024, NULL, 1, NULL, 1); // Core 1
}

void loop() {
  // The loop is empty as tasks are running on FreeRTOS
}
