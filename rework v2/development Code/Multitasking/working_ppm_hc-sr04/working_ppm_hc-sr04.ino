#include <Arduino.h>
#include <FreeRTOS.h>
#include <NewPing.h>  // Include the NewPing library

// Define GPIO pins for HC-SR04
#define TRIG_PIN  1  // Adjust based on wiring
#define ECHO_PIN  2  // Adjust based on wiring

// Define the pin connected to the PPM signal
#define PPM_PIN 13

// Variables to store the pulse width for each channel
volatile uint16_t channel1 = 1500;
volatile uint16_t channel2 = 1500;
volatile uint16_t channel3 = 1500;
volatile uint16_t channel4 = 1500;
volatile uint16_t channel5 = 1500;
volatile uint16_t channel6 = 1500;
volatile uint16_t channel7 = 1500;
volatile uint16_t channel8 = 1500;

// Define the number of channels and the total number of pulses
#define NUM_CHANNELS 8
#define TOTAL_PULSES (NUM_CHANNELS + 1)

// Define the PPM output pin
#define PPM_PIN_OUT 12

// Define the PPM frame duration in microseconds
#define FRAME_DURATION 20000

// Define the maximum distance for HC-SR04
#define MAX_DISTANCE 205

// Array to store the pulse widths for each channel
uint16_t pulseWidths[NUM_CHANNELS];
volatile uint8_t currentChannel = 0;

// Create NewPing object for HC-SR04
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// Function prototypes
void hc_sr04_task(void *pvParameters);
void ppm_task(void *pvParameters);
void readPPM();

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
      case 0:
        // Correct the first channel by subtracting 100 microseconds to compensate for the error
        channel1 = (interval >= 50) ? interval - 50 : 0;  // Prevent negative values
        break;
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
void ppm_task(void *pvParameters) {
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
      delayMicroseconds(pulseWidths[i]);  // Adjust pulse width for each channel
      digitalWrite(PPM_PIN_OUT, LOW);
    }

    // Calculate the time taken and adjust to match FRAME_DURATION
    uint32_t elapsedTime = micros() - pulseStartTime;
    uint32_t remainingTime = FRAME_DURATION - elapsedTime;

    if (remainingTime > 0) {
      digitalWrite(PPM_PIN_OUT, LOW);
      delayMicroseconds(remainingTime);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // Prevent overloading Core 0
  }
}

// HC-SR04 distance measuring task (runs on Core 1)
void hc_sr04_task(void *pvParameters) {
    while (1) {
        // Measure the distance using NewPing
        unsigned int distance = sonar.ping_cm();  // Get distance in cm

        if (distance == 0) {
          distance = MAX_DISTANCE;  // If no measurement, set distance to max distance
        }

        // Print result
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");

        vTaskDelay(pdMS_TO_TICKS(500)); // Delay 500ms
    }
}

void setup() {
    // Set up serial and I/O pins
    Serial.begin(115200);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(PPM_PIN_OUT, OUTPUT);
    digitalWrite(PPM_PIN_OUT, LOW);
    pinMode(PPM_PIN, INPUT);

    // Attach interrupt for PPM signal
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, FALLING);

    // Start the FreeRTOS tasks
    xTaskCreatePinnedToCore(hc_sr04_task, "HC-SR04 Task", 2048, NULL, 2, NULL, 1); // Core 1
    xTaskCreatePinnedToCore(ppm_task, "PPM Task", 2048, NULL, 1, NULL, 0); // Core 0
}

void loop() {
  // The loop is empty as tasks are running on FreeRTOS
}
