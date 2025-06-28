#include <Arduino.h>
#include <FreeRTOS.h>
#include <NewPing.h>  // Include the NewPing library
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <PID_v1.h>

// --- PID CONFIG ---
constexpr double BASE_Kp_z = 2.0, BASE_Ki_z = 5.0, BASE_Kd_z = 1.0;
constexpr double BASE_Kp_x = 1.5, BASE_Ki_x = 4.0, BASE_Kd_x = 0.8;
constexpr double BASE_Kp_y = 1.5, BASE_Ki_y = 4.0, BASE_Kd_y = 0.8;

// --- Tuning Constants ---
constexpr int FORCE_SCALE = 100;
constexpr double SETPOINT_ADJUST_STEP = 3.0;
constexpr double LOWPASS_ALPHA = 0.2;
constexpr int SAFE_DISTANCE = 100;
constexpr int CRITICAL_DISTANCE = 50;
constexpr int DEADZONE = 10;
constexpr uint16_t MIN_THROTTLE = 1000;
constexpr uint16_t MAX_THROTTLE = 2000;

// --- PID-Instanzen ---
double z_input = 0, z_output = 0, z_setpoint = 150;
double x_input = 0, x_output = 0, x_setpoint = 0;
double y_input = 0, y_output = 0, y_setpoint = 0;

PID z_pid(&z_input, &z_output, &z_setpoint, BASE_Kp_z, BASE_Ki_z, BASE_Kd_z, DIRECT);
PID x_pid(&x_input, &x_output, &x_setpoint, BASE_Kp_x, BASE_Ki_x, BASE_Kd_x, DIRECT);
PID y_pid(&y_input, &y_output, &y_setpoint, BASE_Kp_y, BASE_Ki_y, BASE_Kd_y, DIRECT);

// --- Zustandsvariablen ---
unsigned long obstacleTimers[4] = {0, 0, 0, 0};
unsigned long stuckSince = 0;
bool rescueMode = false;
bool sensor_noise = false;

int last_front = 0, last_back = 0;
int last_escape_direction_x = 0, last_escape_direction_y = 0;
unsigned long last_clear_path_time = 0;

float filtered_front = 0;
float filtered_back = 0;
float filtered_diag_left = 0;
float filtered_diag_right = 0;

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

void ppm_task(void *pvParameters) {
  z_pid.SetOutputLimits(-300, 300);
  x_pid.SetOutputLimits(-100, 100);
  y_pid.SetOutputLimits(-100, 100);

  z_pid.SetMode(AUTOMATIC);
  x_pid.SetMode(AUTOMATIC);
  y_pid.SetMode(AUTOMATIC);

  while (true) {
    int mode = channel6;

    if (mode >= 1700) {
      updateAndSendPPM();

    } else if (mode >= 1300 && mode < 1700) {
      z_input = (tofTop + (2000 - tofBottom)) / 2.0;
      z_pid.Compute();
      channel4 = constrain(channel4 + (int)z_output, MIN_THROTTLE, MAX_THROTTLE);
      updateAndSendPPM();

    } else {
      // Sensorwerte einlesen & filtern
      int raw_front = min(distance_ultrasonic_front_left, distance_ultrasonic_front_right);
      int raw_back = min(distance_ultrasonic_rear_left, distance_ultrasonic_rear_right);
      int raw_diag_left = min(distance_ultrasonic_front_left, distance_ultrasonic_rear_left);
      int raw_diag_right = min(distance_ultrasonic_front_right, distance_ultrasonic_rear_right);

      filtered_front = (1.0 - LOWPASS_ALPHA) * filtered_front + LOWPASS_ALPHA * raw_front;
      filtered_back = (1.0 - LOWPASS_ALPHA) * filtered_back + LOWPASS_ALPHA * raw_back;
      filtered_diag_left = (1.0 - LOWPASS_ALPHA) * filtered_diag_left + LOWPASS_ALPHA * raw_diag_left;
      filtered_diag_right = (1.0 - LOWPASS_ALPHA) * filtered_diag_right + LOWPASS_ALPHA * raw_diag_right;

      int front = (int)filtered_front;
      int back = (int)filtered_back;
      int diag_left = (int)filtered_diag_left;
      int diag_right = (int)filtered_diag_right;

      if (abs(front - back) < DEADZONE) front = back = (front + back) / 2;
      if (abs(diag_left - diag_right) < DEADZONE) diag_left = diag_right = (diag_left + diag_right) / 2;

      sensor_noise = (abs(front - last_front) > 30 || abs(back - last_back) > 30);
      last_front = front;
      last_back = back;

      bool critical = false;
      int directions[4] = {front, back, diag_left, diag_right};
      for (int i = 0; i < 4; i++) {
        if (directions[i] < CRITICAL_DISTANCE) {
          if (millis() - obstacleTimers[i] > 300) critical = true;
        } else {
          obstacleTimers[i] = millis();
        }
      }

      if (front < 50 && back < 50 && diag_left < 50 && diag_right < 50) {
        if (stuckSince == 0) stuckSince = millis();
        if (millis() - stuckSince > 1000) rescueMode = true;
      } else {
        stuckSince = 0;
        rescueMode = false;
      }

      if (critical || rescueMode) {
        channel1 = 1500;
        channel2 = 1900;
        channel3 = 1500;
        channel4 = constrain(channel4 + 100, MIN_THROTTLE, MAX_THROTTLE);

      } else {
        float front_w = constrain(map(front, 30, 150, 1.0, 0.0), 0.0, 1.0);
        float back_w = constrain(map(back, 30, 150, 1.0, 0.0), 0.0, 1.0);
        float left_w = constrain(map(diag_left, 30, 150, 1.0, 0.0), 0.0, 1.0);
        float right_w = constrain(map(diag_right, 30, 150, 1.0, 0.0), 0.0, 1.0);

        double x_force = back_w - front_w;
        double y_force = left_w - right_w;

        if (x_force != 0 || y_force != 0) {
          last_escape_direction_x = x_force * 100;
          last_escape_direction_y = y_force * 100;
          last_clear_path_time = millis();
        }

        if (millis() - last_clear_path_time > 300 && x_force == 0 && y_force == 0) {
          x_force = last_escape_direction_x / 100.0;
          y_force = last_escape_direction_y / 100.0;
        }

        bool near = front < SAFE_DISTANCE || back < SAFE_DISTANCE || diag_left < SAFE_DISTANCE || diag_right < SAFE_DISTANCE;
        if (near) {
          x_pid.SetTunings(BASE_Kp_x * 1.5, BASE_Ki_x * 1.2, BASE_Kd_x);
          y_pid.SetTunings(BASE_Kp_y * 1.5, BASE_Ki_y * 1.2, BASE_Kd_y);
        } else {
          x_pid.SetTunings(BASE_Kp_x, BASE_Ki_x, BASE_Kd_x);
          y_pid.SetTunings(BASE_Kp_y, BASE_Ki_y, BASE_Kd_y);
        }

        x_input = x_force * FORCE_SCALE;
        y_input = y_force * FORCE_SCALE;

        if (!sensor_noise) {
          x_pid.Compute();
          y_pid.Compute();
          double scale = near ? 0.6 : 1.0;
          channel1 += (int)(x_output * scale);
          channel2 += (int)(y_output * scale);
        }

        if (diag_left < 50 || diag_right < 50) {
          channel3 = 1500;
        }
      }

      if (tofTop < 50) z_setpoint = max(z_setpoint - SETPOINT_ADJUST_STEP, 100.00);
      if (tofBottom < 50) z_setpoint = min(z_setpoint + SETPOINT_ADJUST_STEP, 300.00);

      z_input = (tofTop + (2000 - tofBottom)) / 2.0;
      z_pid.Compute();
      channel4 = constrain(channel4 + (int)z_output, MIN_THROTTLE, MAX_THROTTLE);

      channel1 = constrain(channel1, 1000, 2000);
      channel2 = constrain(channel2, 1000, 2000);

      updateAndSendPPM();
    }
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
void updateAndSendPPM() {
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
}
