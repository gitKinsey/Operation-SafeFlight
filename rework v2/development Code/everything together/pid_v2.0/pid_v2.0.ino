#include <Arduino.h>
#include <FreeRTOS.h>
#include <NewPing.h>  // Include the NewPing library
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <PID_v1.h>  // Add PID library

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
const uint16_t MAX_THROTTLE = 2000;     // Maximum throttle value for safety

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

// Define variables for PID control
// Z-axis (height) control
double height_setpoint = 100.0;      // Target height in cm
double height_input = 0.0;           // Current height
double height_output = 0.0;          // PID output for height

// Forward/backward control
double forward_setpoint = 100.0;     // Target distance in cm (from obstacles)
double forward_input = 0.0;          // Current distance
double forward_output = 0.0;         // PID output for forward/backward

// Lateral (left/right) control
double lateral_setpoint = 100.0;     // Target distance in cm (from obstacles)
double lateral_input = 0.0;          // Current distance
double lateral_output = 0.0;         // PID output for left/right

// Create PID instances
// PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID height_PID(&height_input, &height_output, &height_setpoint, 2.5, 0.2, 1.0, DIRECT);
PID forward_PID(&forward_input, &forward_output, &forward_setpoint, 2.0, 0.05, 0.8, DIRECT);
PID lateral_PID(&lateral_input, &lateral_output, &lateral_setpoint, 2.0, 0.05, 0.8, DIRECT);

// Define variables for smoothing
float prev_channel1 = 1500;
float prev_channel2 = 1500;
float prev_channel4 = 1500;

// Declare variables for PID tuning parameters
float height_Kp = 3.0, height_Ki = 0.3, height_Kd = 1.2;
float forward_Kp = 2.5, forward_Ki = 0.07, forward_Kd = 1.0;
float lateral_Kp = 2.5, lateral_Ki = 0.07, lateral_Kd = 1.0;


// Function prototypes
void Sensor_task(void *pvParameters);
void ppm_task(void *pvParameters);
void readPPM();
void measurement_ultrasonic(NewPing &sonar, unsigned int &distance);
void selectMultiplexerChannel(uint8_t channel);
float measurement_vl53l0x();
void initializeVl53l0x();
void addToList(int value);
int selectElement(int index);
void readAllVL53L0x();
void sendPPM();
void initPIDControllers();
double calculateLateralInput();
double calculateForwardInput();
double calculateHeightInput();
void updatePPMWithPID();
float medianFilter(float values[], int size);
void updateDynamicThresholds();
float calculateResponseFactor(float current_distance, float threshold);

// Median filter to reduce sensor noise
float medianFilter(float values[], int size) {
    // Create a copy of the array to sort
    float temp[size];
    for (int i = 0; i < size; i++) {
        temp[i] = values[i];
    }
    
    // Simple bubble sort (efficient enough for small arrays)
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                float t = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = t;
            }
        }
    }
    
    // Return the median value
    return temp[size / 2];
}

// Calculate a progressive response factor that increases as distance decreases
float calculateResponseFactor(float current_distance, float threshold) {
    const float CRITICAL_DISTANCE = threshold * 0.5; // 50% of threshold is critical
    const float MAX_FACTOR = 1.5; // Maximum scaling factor for very close obstacles
    
    if (current_distance >= threshold) {
        return 1.0; // Normal response when outside threshold
    } else if (current_distance <= CRITICAL_DISTANCE) {
        return MAX_FACTOR; // Maximum response when very close
    } else {
        // Linear scaling between normal and maximum response
        float normalized_distance = (threshold - current_distance) / (threshold - CRITICAL_DISTANCE);
        return 1.0 + normalized_distance * (MAX_FACTOR - 1.0);
    }
}

// Dynamic threshold adjustment based on velocity
void updateDynamicThresholds() {
    // Estimate velocity from throttle and pitch values
    float throttle_percentage = (channel4 - MIN_THROTTLE) / (float)(MAX_THROTTLE - MIN_THROTTLE);
    float pitch_value = abs(channel2 - 1500) / 500.0; // Normalized pitch value (0-1)
    
    // Adjust forward threshold based on estimated forward velocity
    float forward_velocity_factor = pitch_value * throttle_percentage;
    float min_forward_threshold = 100; // Minimum safe distance (cm)
    float max_forward_threshold = 250; // Maximum look-ahead distance (cm)
    
    // Dynamic threshold calculation
    float dynamic_forward_threshold = min_forward_threshold + 
                                     (max_forward_threshold - min_forward_threshold) * forward_velocity_factor;
    
    // Update PID setpoint with the new dynamic threshold
    forward_setpoint = dynamic_forward_threshold;
    
    // Adjust lateral threshold similarly
    lateral_setpoint = min_forward_threshold + 
                      (max_forward_threshold - min_forward_threshold) * forward_velocity_factor * 0.8;
}


// Function to calculate weighted average from ultrasonic sensors for lateral control
double calculateLateralInput() {
    // Weight by angle (45 degrees) and normalize
    double left_distance = (distance_ultrasonic_front_left + distance_ultrasonic_rear_left) / 2.0;
    double right_distance = (distance_ultrasonic_front_right + distance_ultrasonic_rear_right) / 2.0;
    
    // Calculate the minimum distance to either side
    double min_distance = min(left_distance, right_distance);
    
    // Return the minimum distance as our lateral input
    return min_distance;
}

// Improved sensor fusion with priority weighting
double calculateForwardInput() {
    // Weight the TOF sensor higher than ultrasonic for forward measurements
    double front_ultrasonic = (distance_ultrasonic_front_left + distance_ultrasonic_front_right) / 2.0;
    
    // If TOF sensor is giving valid readings (not at max range)
    if (tofFront < 200) {
        // Use TOF with higher weight (70%) when available
        return tofFront * 0.7 + front_ultrasonic * 0.3;
    } else {
        // Fall back to ultrasonic when TOF is maxed out
        return front_ultrasonic;
    }
}

// Function to calculate height input
double calculateHeightInput() {
    // Use the bottom TOF sensor for height control
    // If it's malfunctioning, we could use other sensors or a default value
    if (tofBottom < 200) {
        return tofBottom;  // Valid reading
    } else {
        // If no valid reading, maintain last known height
        // (or implement a fallback strategy)
        return 100; // Default safe height
    }
}

void initPIDControllers() {
    // Height control (Z-axis)
    height_setpoint = 100.0;  
    height_PID.SetTunings(height_Kp, height_Ki, height_Kd);  // Use variable PID constants
    height_PID.SetOutputLimits(-350, 350);  // Narrower to avoid overcorrection
    height_PID.SetMode(AUTOMATIC);

    // Forward/backward (Pitch control)
    forward_setpoint = distanceThreshold_xy;
    forward_PID.SetTunings(forward_Kp, forward_Ki, forward_Kd);  // Use variable PID constants
    forward_PID.SetOutputLimits(-250, 250);  // Adjusted for smoother corrections
    forward_PID.SetMode(AUTOMATIC);

    // Lateral (Roll control)
    lateral_setpoint = distanceThreshold_xy;
    lateral_PID.SetTunings(lateral_Kp, lateral_Ki, lateral_Kd);  // Use variable PID constants
    lateral_PID.SetOutputLimits(-250, 250);
    lateral_PID.SetMode(AUTOMATIC);

    // Reduce sample time for more frequent updates (improves fast response)
    height_PID.SetSampleTime(30);
    forward_PID.SetSampleTime(30);
    lateral_PID.SetSampleTime(30);
}


// Enhanced collision avoidance with progressive response
void updatePPMWithPID() {
    // Update dynamic thresholds based on current velocity
    updateDynamicThresholds();
    
    // Calculate improved sensor inputs with weighted importance
    height_input = calculateHeightInput();
    forward_input = calculateForwardInput();
    lateral_input = calculateLateralInput();
    
    // Compute PID outputs
    height_PID.Compute();
    forward_PID.Compute();
    lateral_PID.Compute();
    
    // Progressive response factor - more aggressive as distance decreases
    float height_factor = calculateResponseFactor(height_input, distanceThreshold_z);
    float forward_factor = calculateResponseFactor(forward_input, forward_setpoint);
    float lateral_factor = calculateResponseFactor(lateral_input, lateral_setpoint);
    
    // Apply PID outputs to channels with progressive scaling
    float new_channel4 = 1500 + height_output * height_factor;  // Throttle
    float new_channel2 = 1500 + forward_output * forward_factor; // Pitch
    float new_channel1 = 1500 + lateral_output * lateral_factor; // Roll
    
    // Apply smoother filtering to prevent jerky movements
    float alpha = 0.7; // Smoothing factor (0-1), higher = less smoothing
    
    channel1 = prev_channel1 * (1-alpha) + new_channel1 * alpha;
    channel2 = prev_channel2 * (1-alpha) + new_channel2 * alpha;
    channel4 = prev_channel4 * (1-alpha) + new_channel4 * alpha;
    
    prev_channel1 = channel1;
    prev_channel2 = channel2;
    prev_channel4 = channel4;
    
    // Constrain all values to be within valid PPM range
    channel1 = constrain(channel1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    channel2 = constrain(channel2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    channel3 = constrain(channel3, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    channel4 = constrain(channel4, MIN_THROTTLE, MAX_THROTTLE);
}

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
        // Correct the first channel by subtracting 50 microseconds to compensate for the error
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
// Ensuring it runs at 20Hz (every 50ms)
void ppm_task(void *pvParameters) {
  // Initialize PID controllers
  initPIDControllers();
  
  const TickType_t xFrequency = pdMS_TO_TICKS(50); // 50ms = 20Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while (true) {
    // Synchronize with the absolute time to ensure consistent frequency
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    if (channel6 >= 1700) {  // Mode 1 => no sensor read outs (Pass-through mode)
      // Just copy the volatile variables into the array without modification
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
      sendPPM();
    
    } else if (channel6 >= 1300 && channel6 < 1700) {  // Mode 2 => only height-controlling sensors
      // Update height_input with the current Z-axis measurements
      height_input = tofBottom;  // Use bottom sensor for height control
      height_setpoint = distanceThreshold_z;  // Target height clearance
      
      // Compute PID output for height control
      height_PID.Compute();
      
      // Calculate response factor for height
      float height_factor = calculateResponseFactor(height_input, distanceThreshold_z);
      
      // Apply height control using PID output with progressive factor
      float new_channel4 = 1500 + height_output * height_factor;
      
      // Apply smoothing
      float alpha = 0.7;
      channel4 = prev_channel4 * (1-alpha) + new_channel4 * alpha;
      prev_channel4 = channel4;
      
      // Ensure channel values stay within safe limits
      channel4 = constrain(channel4, MIN_THROTTLE, MAX_THROTTLE);
      
      // Copy values to pulse width array for sending
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
      
      // Send the PPM signal
      sendPPM();
      
    } else {  // Mode 3 => full collision avoidance with all sensors
      // Update all PID inputs with current sensor data and apply PID control
      updatePPMWithPID();
      
      // Copy the updated channel values to the pulse width array
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
      
      // Send the PPM signal
      sendPPM();
    }
  }
}

// Improved sensor task with filtering and fault detection
void Sensor_task(void *pvParameters) {
    const int HISTORY_SIZE = 3;
    float tofFrontHistory[HISTORY_SIZE] = {400, 400, 400};
    float tofBackHistory[HISTORY_SIZE] = {400, 400, 400};
    float tofTopHistory[HISTORY_SIZE] = {400, 400, 400};
    float tofBottomHistory[HISTORY_SIZE] = {400, 400, 400};
    int historyIndex = 0;
    
    while (1) {
        // Read ultrasonic sensors
        measurement_ultrasonic(sonar_front_left, distance_ultrasonic_front_left);
        measurement_ultrasonic(sonar_front_right, distance_ultrasonic_front_right);
        measurement_ultrasonic(sonar_rear_left, distance_ultrasonic_rear_left);
        measurement_ultrasonic(sonar_rear_right, distance_ultrasonic_rear_right);
        
        // Read ToF sensors
        readAllVL53L0x();
        
        // Store values in circular buffer for filtering
        tofFrontHistory[historyIndex] = tofFront;
        tofBackHistory[historyIndex] = tofBack;
        tofTopHistory[historyIndex] = tofTop;
        tofBottomHistory[historyIndex] = tofBottom;
        
        historyIndex = (historyIndex + 1) % HISTORY_SIZE;
        
        // Apply median filtering to reduce sensor noise
        tofFront = medianFilter(tofFrontHistory, HISTORY_SIZE);
        tofBack = medianFilter(tofBackHistory, HISTORY_SIZE);
        tofTop = medianFilter(tofTopHistory, HISTORY_SIZE);
        tofBottom = medianFilter(tofBottomHistory, HISTORY_SIZE);
        
        // Update the tofReadOuts array
        tofReadOuts[0] = tofFront;
        tofReadOuts[1] = tofBack;
        tofReadOuts[2] = tofTop;
        tofReadOuts[3] = tofBottom;
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Increased sampling rate from 500ms to 100ms
    }
}

void measurement_ultrasonic(NewPing &sonar, unsigned int &distance) {
  // Measure the distance using NewPing
  distance = sonar.ping_cm();  // Get distance in cm

  if (distance == 0) {
    distance = MAX_DISTANCE;  // If no measurement, set distance to max distance
  }

  // Print result (optional - can be removed to reduce serial traffic)
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

    // Read distance from the sensor
    float distance = measurement_vl53l0x();
    if (distance == -1) {  // If measurement failed
      Serial.print("Sensor ");
      Serial.print(sensorIndex);
      Serial.println(" failed to read distance.");
      distance = 205;  // Set to max distance
    }

    // Update the appropriate sensor variable
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
}

void sendPPM() {
  // Generate PPM signal
  uint32_t pulseStartTime = micros();
  
  // Send the pulse widths for each channel
  for (int i = 0; i < NUM_CHANNELS; i++) {
    digitalWrite(PPM_PIN_OUT, HIGH);
    delayMicroseconds(300);  // Consistent pulse width for sync
    digitalWrite(PPM_PIN_OUT, LOW);
    delayMicroseconds(pulseWidths[i] - 300);  // Remaining time for channel
  }
  
  // Calculate the time taken and adjust to match FRAME_DURATION
  uint32_t elapsedTime = micros() - pulseStartTime;
  int32_t remainingTime = FRAME_DURATION - elapsedTime;
  
  if (remainingTime > 0) {
    digitalWrite(PPM_PIN_OUT, LOW);
    delayMicroseconds(remainingTime);
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
    
    // Initialize PID controllers
    initPIDControllers();

    // Initialize smoothing variables
    prev_channel1 = 1500;
    prev_channel2 = 1500;
    prev_channel4 = 1500;

    // Start the FreeRTOS tasks
    xTaskCreatePinnedToCore(Sensor_task, "Sensor_task", 4096, NULL, 2, NULL, 1); // Core 1
    xTaskCreatePinnedToCore(ppm_task, "PPM Task", 4096, NULL, 1, NULL, 0); // Core 0
    
    Serial.println("System initialized and ready");
}

void loop() {
  // The loop is empty as tasks are running on FreeRTOS
}