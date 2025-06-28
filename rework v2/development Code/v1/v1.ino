#include <Arduino.h>
#include <FreeRTOS.h>
#include <NewPing.h>  // Include the NewPing library
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Replace with your multiplexer address (default for TCA9548A is 0x70)
#define MULTIPLEXER_ADDRESS 0x70
#define SENSOR_ADDRESS 0x29  // Address of the VL53L0X sensor
#define MAX_LIST_SIZE 4  // Maximum number of elements in the list

int availableVL53l0x[MAX_LIST_SIZE];
int currentVL53l0xListSize = 0;  // Tracks the number of elements in the list

// Define GPIO pins for HC-SR04
#define TRIG_PIN_FRONT_LEFT  1  // Adjust based on wiring
#define ECHO_PIN_FRONT_LEFT  2  // Adjust based on wiring

#define TRIG_PIN_FRONT_RIGHT  3  // Adjust based on wiring
#define ECHO_PIN_FRONT_RIGHT  6  // Adjust based on wiring

#define TRIG_PIN_REAR_RIGHT   7  // Adjust based on wiring
#define ECHO_PIN_REAR_RIGHT   8  // Adjust based on wiring

#define TRIG_PIN_REAR_LEFT    9  // Adjust based on wiring
#define ECHO_PIN_REAR_LEFT    10  // Adjust based on wiring

unsigned int distance_ultrasonic_front_left;
unsigned int distance_ultrasonic_front_right;
unsigned int distance_ultrasonic_rear_left;
unsigned int distance_ultrasonic_rear_right;

// variables for avoiding part
int distanceThreshold_xy = 100;         // distance threshold for the xy plane
int distanceThreshold_z = 150;          // distance threshold for the z plane
const uint16_t MIN_THROTTLE = 1000;     // Minimum throttle value for safety
const uint16_t MAX_THROTTLE = 2000;     // Maximum throttle value for safet

// Define the pin connected to the PPM signal
#define PPM_PIN 13

// Variables to store the pulse width for each channel
volatile uint16_t channel1 = 1500;
volatile uint16_t channel2 = 1500;
volatile uint16_t channel3 = 1500;
volatile uint16_t channel4 = 1000;
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
#define MIN_PULSE_WIDTH 1000
#define MAX_PULSE_WIDTH 2000

// Define the maximum distance for HC-SR04
#define MAX_DISTANCE 205

// Array to store the pulse widths for each channel
uint16_t pulseWidths[NUM_CHANNELS];
volatile uint8_t currentChannel = 0;

// Create NewPing object for HC-SR04
NewPing sonar_front_left(TRIG_PIN_FRONT_LEFT, ECHO_PIN_FRONT_LEFT, MAX_DISTANCE);
NewPing sonar_front_right(TRIG_PIN_FRONT_RIGHT, ECHO_PIN_FRONT_RIGHT, MAX_DISTANCE);
NewPing sonar_rear_left(TRIG_PIN_REAR_LEFT, ECHO_PIN_REAR_LEFT, MAX_DISTANCE);
NewPing sonar_rear_right(TRIG_PIN_REAR_RIGHT, ECHO_PIN_REAR_RIGHT, MAX_DISTANCE);

Adafruit_VL53L0X lox;  // Create a VL53L0X object
int active_channel = 0; // Active channel of the multiplexer
uint16_t vl53l0xMeasurements[4];

//variables for the sensor data
float tofFront = 400;
float tofBack = 400;
float tofTop = 400;
float tofBottom = 400;

float tofReadOuts[4] = {tofFront, tofBack, tofTop, tofBottom};

int adjustValue = 100;

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
    if (channel6 >= 1700) {  // this is mode 1 => no sensor read outs
  // copying the volatile variables into the array (don't modify the channel variables, just add the necessary value in the noInterrupt part)
  noInterrupts();  // making sure that the data from the ppm input doesn't change during read out (really important) => everything crucial that shouldn't be disrupted by interrupts goes here aka, reading sensor values, performing the calculations, sending the ppm signal out again
  pulseWidths[0] = channel1;
  pulseWidths[1] = channel2;
  pulseWidths[2] = channel3;
  pulseWidths[3] = channel4;
  pulseWidths[4] = channel5;
  pulseWidths[5] = channel6;
  pulseWidths[6] = channel7;
  pulseWidths[7] = channel8;
  interrupts();
  sendPPM();
  
} else if (channel6 >= 1300 && channel6 < 1700) {  // this is mode 2 => only height-controlling sensors are read out
  // check if one of the sensors' value is too close to the wall or another obstacle:
  if (tofTop <= distanceThreshold_z) {
    channel4 -= adjustValue;  // throttle down
  } else if (tofBottom <= distanceThreshold_z) {
    channel4 += adjustValue;  // Throttle up
  }
  channel4 = constrain(channel4, MIN_THROTTLE, MAX_THROTTLE);  // makes sure that the channel value stays between min and max throttle if something went wrong before

  noInterrupts();  // making sure that the data from the ppm input doesn't change during read out
  pulseWidths[0] = channel1;
  pulseWidths[1] = channel2;
  pulseWidths[2] = channel3;
  pulseWidths[3] = channel4;
  pulseWidths[4] = channel5;
  pulseWidths[5] = channel6;
  pulseWidths[6] = channel7;
  pulseWidths[7] = channel8;
  interrupts();

  // sending the ppm signal out
  sendPPM();
  
} else {  // this is mode 3 => all sensors are read out
  // for future Cedi: Put all the maths here (for full-on collision avoidance):
  // check if any sensors are under the threshold and then adjust the logic, e.g., just have 6 direction variables which get plus 1 if the threshold is undercut
  
  // create variables to save the necessary adjustments
  int forward = 0;
  int backward = 0;
  int left = 0;
  int right = 0;
  int up = 0;
  int down = 0;

  // first height control (same as in mode 2)
  if (tofTop <= distanceThreshold_z) {
    channel4 = constrain(channel4, MIN_THROTTLE + adjustValue, MAX_THROTTLE);  // if throttle is already at 1000 (threshold)
    channel4 -= adjustValue;  // throttle down
  } else if (tofBottom <= distanceThreshold_z) {
    channel4 = constrain(channel4, MIN_THROTTLE, MAX_THROTTLE - adjustValue);  // if throttle is already at 2000 (threshold)
    channel4 += adjustValue;  // Throttle up
  }

  // forward-backward control for the ultrasonic sensors
  if (distance_ultrasonic_front_left < distanceThreshold_xy) {
    backward++;
    right++;
  } else if (distance_ultrasonic_front_right < distanceThreshold_xy) {
    backward++;
    left++;
  } else if (distance_ultrasonic_rear_left < distanceThreshold_xy) {
    forward++;
    right++;
  } else if (distance_ultrasonic_rear_right < distanceThreshold_xy) {
    forward++;
    left++;
  }

  // for the remaining tof sensors
  if (tofFront < distanceThreshold_xy) {
    backward++;
  } else if (tofBack < distanceThreshold_xy) {
    forward++;
  }

  // manipulating the signal for the ppm
  channel1 = channel1 - (adjustValue * right) + (adjustValue * left);
  if (channel1 < 1000) {
    channel1 = 1000;
  } else if (channel1 > 2000) {
    channel1 = 2000;
  }
  channel2 = channel2 - (adjustValue * forward) + (adjustValue * backward);
  if (channel2 < 1000) {
    channel2 = 1000;
  } else if (channel2 > 2000) {
    channel2 = 2000;
  }
  channel4 = channel4 - (adjustValue * down) + (adjustValue * up);
  if (channel4 < 1000) {
    channel4 = 1000;
  } else if (channel4 > 2000) {
    channel4 = 2000;
  }

  // constraining all the values before putting them into the array values for sending out
  channel1 = constrain(channel1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  channel2 = constrain(channel2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  channel4 = constrain(channel4, MIN_THROTTLE, MAX_THROTTLE);

  // saving the channel values into the array values for sending out
  noInterrupts();  // making sure that the data from the ppm input doesn't change during read out
  pulseWidths[0] = channel1;
  pulseWidths[1] = channel2;
  pulseWidths[2] = channel3;
  pulseWidths[3] = channel4;
  pulseWidths[4] = channel5;
  pulseWidths[5] = channel6;
  pulseWidths[6] = channel7;
  pulseWidths[7] = channel8;
  interrupts();
  sendPPM();
}
    vTaskDelay(10 / portTICK_PERIOD_MS); // Prevent overloading Core 0
  }
}

// HC-SR04 distance measuring task (runs on Core 1)
void Sensor_task(void *pvParameters) {
    while (1) {
        measurement_ultrasonic(sonar_front_left, distance_ultrasonic_front_left);
        measurement_ultrasonic(sonar_front_right, distance_ultrasonic_front_right);
        measurement_ultrasonic(sonar_rear_left, distance_ultrasonic_rear_left);
        measurement_ultrasonic(sonar_rear_right, distance_ultrasonic_rear_right);
        readAllVL53L0x();
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay 500ms
    }
}

void setup() {
    // Set up serial and I/O pins
    Serial.begin(115200);
    pinMode(PPM_PIN_OUT, OUTPUT);
    digitalWrite(PPM_PIN_OUT, LOW);
    pinMode(PPM_PIN, INPUT);

    // Attach interrupt for PPM signal
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, FALLING);

      // Initialize I2C communication with custom pins (optional for ESP32)
    Wire.begin(4, 5); // SDA = pin 4, SCL = pin 5
    initializeVl53l0x();

    // Start the FreeRTOS tasks
    xTaskCreatePinnedToCore(Sensor_task, "Sensor_task", 4096, NULL, 2, NULL, 1); // Core 1
    xTaskCreatePinnedToCore(ppm_task, "PPM Task", 4096, NULL, 1, NULL, 0); // Core 0
}

void loop() {
  // The loop is empty as tasks are running on FreeRTOS
}

void measurement_ultrasonic(NewPing &sonar, unsigned int &distance) {
  // Measure the distance using NewPing
  distance = sonar.ping_cm();  // Get distance in cm

  if (distance == 0) {
    distance = MAX_DISTANCE;  // If no measurement, set distance to max distance
  }

  // Print result
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}


void selectMultiplexerChannel(uint8_t channel) {
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  Wire.write(1 << channel);  // Activate the specified channel
  Wire.endTransmission();
}

// Function to read data from the VL53L0X sensor
float measurement_vl53l0x() {
  VL53L0X_RangingMeasurementData_t measure;

  // Start the ranging test
  lox.rangingTest(&measure, false);

  // Check if the measurement is valid
  if (measure.RangeStatus != 4) { // If the measurement is not out of range
    Serial.print("Distance: ");
    Serial.print(measure.RangeMilliMeter / 10.0); // Convert to cm
    Serial.println(" cm");
    return measure.RangeMilliMeter / 10.0;  // Return the valid measurement in cm
  } else {
    Serial.println("Out of range");
    return 205;
  }
}


void initializeVl53l0x() {
  for (int i = 0; i < 4; i++) {
    selectMultiplexerChannel(i);
    if (!lox.begin(SENSOR_ADDRESS)) {
      Serial.print("Attempt ");
      Serial.print(i + 1);
      Serial.println(" failed to initialize VL53L0X!");
    } else {
      Serial.println("VL53L0X sensor initialized.");
      addToList(i);
    }
  }
}

void addToList(int value) {
  if (currentVL53l0xListSize < MAX_LIST_SIZE) {
    availableVL53l0x[currentVL53l0xListSize] = value;
    currentVL53l0xListSize++;
  } else {
    Serial.println("List is full");
  }
}

int selectElement(int index) {
  if (index >= 0 && index < currentVL53l0xListSize) {
    return availableVL53l0x[index];  // Return the element at the specified index
  } else {
    Serial.println("Invalid index");
    return -1;  // Return -1 if the index is out of range
  }
}

void readAllVL53L0x() {
  for (int i = 0; i < currentVL53l0xListSize; i++) {
    int sensorIndex = availableVL53l0x[i];  // Get the sensor index from the available list

    // Select the corresponding multiplexer channel for the current VL53L0X sensor
    selectMultiplexerChannel(sensorIndex);

    // Read distance from the sensor and store it in the tofReadOuts array
    float distance = measurement_vl53l0x();
    if (distance == -1) {  // If measurement failed
      Serial.print("Sensor ");
      Serial.print(sensorIndex);
      Serial.println(" failed to read distance.");
      tofReadOuts[i] = 205;
      continue;  // Skip this sensor and move to the next one
    }

    if (sensorIndex == 0) {
      tofFront = distance;  // Front sensor
    } else if (sensorIndex == 1) {
      tofBack = distance;   // Back sensor
    } else if (sensorIndex == 2) {
      tofTop = distance;    // Top sensor
    } else if (sensorIndex == 3) {
      tofBottom = distance; // Bottom sensor
    }
  }

  // Update the tofReadOuts array
  tofReadOuts[0] = tofFront;
  tofReadOuts[1] = tofBack;
  tofReadOuts[2] = tofTop;
  tofReadOuts[3] = tofBottom;
}
void sendPPM(){
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


}