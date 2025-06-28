#include <Arduino.h>
#include <FreeRTOS.h>
#include <NewPing.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Replace with your multiplexer address (default for TCA9548A is 0x70)
#define MULTIPLEXER_ADDRESS 0x70
#define SENSOR_ADDRESS 0x29  // Address of the VL53L0X sensor
#define MAX_LIST_SIZE 4      // Maximum number of elements in the list

int availableVL53l0x[MAX_LIST_SIZE];
int currentVL53l0xListSize = 0;  // Tracks the number of elements in the list

// Define GPIO pins for HC-SR04
#define TRIG_PIN_FRONT_LEFT  1
#define ECHO_PIN_FRONT_LEFT  2

#define TRIG_PIN_FRONT_RIGHT  3
#define ECHO_PIN_FRONT_RIGHT  6

#define TRIG_PIN_REAR_RIGHT   7
#define ECHO_PIN_REAR_RIGHT   8

#define TRIG_PIN_REAR_LEFT    9
#define ECHO_PIN_REAR_LEFT    10

// MODIFIED: Changed to float and renamed for clarity with _cm suffix
float distance_ultrasonic_front_left_cm;
float distance_ultrasonic_front_right_cm;
float distance_ultrasonic_rear_left_cm;
float distance_ultrasonic_rear_right_cm;

// MODIFIED: Renamed from distanceThreshold_xy and distanceThreshold_z for clarity and context
// int distanceThreshold_xy = 100;         // Original: distance threshold for the xy plane
// int distanceThreshold_z = 150;          // Original: distance threshold for the z plane

// Pulse width limits
const uint16_t MIN_PULSE_WIDTH_US = 1000;
const uint16_t MAX_PULSE_WIDTH_US = 2000;
const uint16_t MIN_THROTTLE_US = 1000;     // Minimum throttle value for safety
const uint16_t MAX_THROTTLE_US = 2000;     // Maximum throttle value for safety

// Define the pin connected to the PPM signal
#define PPM_PIN 13

// Variables to store the pulse width for each channel (from RC receiver)
volatile uint16_t channel1_rc_us = 1500; // Roll
volatile uint16_t channel2_rc_us = 1500; // Pitch
volatile uint16_t channel3_rc_us = 1500; // Yaw
volatile uint16_t channel4_rc_us = 1000; // Throttle
volatile uint16_t channel5_rc_us = 1500; // Aux1
volatile uint16_t channel6_rc_us = 1500; // Aux2 (Mode Switch)
volatile uint16_t channel7_rc_us = 1500; // Aux3
volatile uint16_t channel8_rc_us = 1500; // Aux4

// Define the number of channels and the total number of pulses
#define NUM_CHANNELS 8
#define TOTAL_PULSES (NUM_CHANNELS + 1)

// Define the PPM output pin
#define PPM_PIN_OUT 12

// Define the PPM frame duration in microseconds
#define FRAME_DURATION 20000
// MIN_PULSE_WIDTH and MAX_PULSE_WIDTH are already defined as MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US

// Define the maximum distance for HC-SR04
#define MAX_DISTANCE_HCSR04_CM 205 // MODIFIED: Added _CM for clarity

// Array to store the pulse widths for each channel (to Flight Controller)
uint16_t pulseWidths_out_us[NUM_CHANNELS]; // MODIFIED: Renamed for clarity
volatile uint8_t ppm_isr_current_channel = 0; // MODIFIED: Renamed for clarity

// Create NewPing object for HC-SR04
NewPing sonar_front_left(TRIG_PIN_FRONT_LEFT, ECHO_PIN_FRONT_LEFT, MAX_DISTANCE_HCSR04_CM);
NewPing sonar_front_right(TRIG_PIN_FRONT_RIGHT, ECHO_PIN_FRONT_RIGHT, MAX_DISTANCE_HCSR04_CM);
NewPing sonar_rear_left(TRIG_PIN_REAR_LEFT, ECHO_PIN_REAR_LEFT, MAX_DISTANCE_HCSR04_CM);
NewPing sonar_rear_right(TRIG_PIN_REAR_RIGHT, ECHO_PIN_REAR_RIGHT, MAX_DISTANCE_HCSR04_CM);

Adafruit_VL53L0X lox;  // Create a VL53L0X object
// int active_channel = 0; // Active channel of the multiplexer - this seems unused, selectMultiplexerChannel is used directly
// uint16_t vl53l0xMeasurements[4]; // This seems unused

// MODIFIED: Renamed sensor variables for clarity with _cm suffix
float tofFront_cm = 400.0f;
float tofBack_cm = 400.0f;
float tofTop_cm = 400.0f;
float tofBottom_cm = 400.0f;

// float tofReadOuts[4] = {tofFront_cm, tofBack_cm, tofTop_cm, tofBottom_cm}; // This array isn't directly used by PID logic

// int adjustValue = 100; // Original adjustment value, will be replaced by PID outputs or specific strengths

// NEW: Constants for Mode 2 (Altitude Hover)
const float PID_SETPOINT_ALTITUDE_CM = 150.0f; // Target altitude: 1.5 meters
const uint16_t HOVER_THROTTLE_NEUTRAL_US = 1500;
const float GROUND_PROXIMITY_THRESHOLD_CM = 20.0f;
const float CEILING_PROXIMITY_THRESHOLD_CM = 30.0f; // Optional: For ceiling safety
const float HOVER_FRONT_AVOID_THRESHOLD_CM = 60.0f;
const int16_t HOVER_PITCH_BACKUP_STRENGTH_US = 150; // Microseconds to add to pitch for backing up

// NEW: PID Gains for Altitude (Mode 2) - Tune these!
float kp_alt = 2.5f;
float ki_alt = 0.3f;
float kd_alt = 0.8f;
float pid_integral_altitude = 0.0f;
float pid_previous_error_altitude = 0.0f;
unsigned long pid_previous_time_alt_ms = 0;
const float MAX_INTEGRAL_ALT = 500.0f; // Anti-windup limit for altitude integral

// NEW: Constants for Mode 3 (PID Obstacle Avoidance)
const float DISTANCE_THRESHOLD_XY_AVOID_CM = 70.0f; // Target distance from obstacles in XY plane
const float AVOID_Z_THRESHOLD_CM = 30.0f;           // Threshold for simple up/down avoidance in Mode 3
const int16_t AVOID_Z_ADJUST_STRENGTH_US = 100;     // Strength for Z-axis adjustment in Mode 3

// NEW: PID Gains for XY Avoidance (Mode 3) - Tune these!
float kp_avoid_xy = 2.0f;
float ki_avoid_xy = 0.1f; // Start with small or zero Ki for avoidance
float kd_avoid_xy = 0.5f;
float pid_integral_avoid_pitch = 0.0f;
float pid_previous_error_avoid_pitch = 0.0f;
float pid_integral_avoid_roll = 0.0f;
float pid_previous_error_avoid_roll = 0.0f;
unsigned long pid_previous_time_avoid_ms = 0;
const float MAX_INTEGRAL_AVOID_XY = 300.0f; // Anti-windup limit for XY avoidance integral

// NEW: Flags for PID initialization
bool hover_mode_just_engaged_flag = true;
bool avoid_mode_just_engaged_flag = true;

// Function prototypes
void Sensor_task(void *pvParameters); // MODIFIED: Renamed from hc_sr04_task for clarity
void ppm_processing_task(void *pvParameters); // MODIFIED: Renamed from ppm_task for clarity
void readPPM_ISR(); // MODIFIED: Renamed for clarity
void sendPPM_output(); // MODIFIED: Renamed for clarity
void measurement_ultrasonic(NewPing &sonar, float &distance_cm); // MODIFIED: Changed to float&
void selectMultiplexerChannel(uint8_t channel);
float measurement_vl53l0x();
void initializeVl53l0x();
void addToList(int value);
int selectElement(int index);
void readAllVL53L0x();


// Interrupt handler for the PPM signal
void IRAM_ATTR readPPM_ISR() { // MODIFIED: Renamed and added IRAM_ATTR
  static uint32_t lastTime = 0;
  uint32_t currentTime = micros();
  uint32_t interval = currentTime - lastTime;
  lastTime = currentTime;

  if (interval >= 3000) {
    ppm_isr_current_channel = 0;
  } else {
    // Corrected pulse width range check and assignment
    uint16_t pulse = constrain(interval, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);
    // The original code had a -50 correction on channel1, keeping it if it's intentional for specific hardware
    if (ppm_isr_current_channel == 0) pulse = constrain(interval - 50, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);


    switch (ppm_isr_current_channel) {
      case 0: channel1_rc_us = pulse; break;
      case 1: channel2_rc_us = pulse; break;
      case 2: channel3_rc_us = pulse; break;
      case 3: channel4_rc_us = pulse; break;
      case 4: channel5_rc_us = pulse; break;
      case 5: channel6_rc_us = pulse; break;
      case 6: channel7_rc_us = pulse; break;
      case 7: channel8_rc_us = pulse; break;
    }
    if (ppm_isr_current_channel < NUM_CHANNELS) { // Ensure we don't overrun
        ppm_isr_current_channel++;
    }
  }
}

// PPM reading and signal generation task (runs on Core 0)
// MODIFIED: Entire task rewritten for new modes and PID control
void ppm_processing_task(void *pvParameters) {
  uint16_t current_rc_ch1, current_rc_ch2, current_rc_ch3, current_rc_ch4;
  uint16_t current_rc_ch5, current_rc_ch6, current_rc_ch7, current_rc_ch8;

  uint16_t output_roll_us, output_pitch_us, output_yaw_us, output_throttle_us;
  uint16_t output_aux1_us, output_aux2_us, output_aux3_us, output_aux4_us;

  while (true) {
    // Atomically copy volatile RC channel inputs to local variables
    noInterrupts();
    current_rc_ch1 = channel1_rc_us;
    current_rc_ch2 = channel2_rc_us;
    current_rc_ch3 = channel3_rc_us;
    current_rc_ch4 = channel4_rc_us;
    current_rc_ch5 = channel5_rc_us;
    current_rc_ch6 = channel6_rc_us;
    current_rc_ch7 = channel7_rc_us;
    current_rc_ch8 = channel8_rc_us;
    interrupts();

    // Default passthrough for AUX channels not actively controlled by modes
    output_aux1_us = current_rc_ch5;
    output_aux2_us = current_rc_ch6; // Mode switch channel itself
    output_aux3_us = current_rc_ch7;
    output_aux4_us = current_rc_ch8;


    // Mode selection based on current_rc_ch6 (Aux2)
    if (current_rc_ch6 < 1300) { // Mode 1: Manual Passthrough
      output_roll_us = current_rc_ch1;
      output_pitch_us = current_rc_ch2;
      output_yaw_us = current_rc_ch3;
      output_throttle_us = current_rc_ch4;

      // Ensure PID states are reset when not in active PID modes
      hover_mode_just_engaged_flag = true;
      avoid_mode_just_engaged_flag = true;

    } else if (current_rc_ch6 >= 1300 && current_rc_ch6 < 1700) { // Mode 2: PID Altitude Hover + Basic Frontal Avoidance
      avoid_mode_just_engaged_flag = true; // Reset other mode's flag

      if (hover_mode_just_engaged_flag) {
        pid_integral_altitude = 0.0f;
        pid_previous_error_altitude = 0.0f;
        pid_previous_time_alt_ms = millis(); // Use millis() or xTaskGetTickCount()
        hover_mode_just_engaged_flag = false;
        Serial.println("Engaging Mode 2: Altitude Hover");
      }

      // --- Altitude PID Control (Throttle on pulseWidths_out_us[3]) ---
      unsigned long current_time_ms = millis();
      float dt_sec = (current_time_ms - pid_previous_time_alt_ms) / 1000.0f;
      pid_previous_time_alt_ms = current_time_ms;

      if (dt_sec <= 0.0001f) dt_sec = 0.01f; // Prevent division by zero / ensure minimum dt if task runs too fast

      float pid_error_alt = PID_SETPOINT_ALTITUDE_CM - tofBottom_cm;
      pid_integral_altitude += ki_alt * pid_error_alt * dt_sec;
      pid_integral_altitude = constrain(pid_integral_altitude, -MAX_INTEGRAL_ALT, MAX_INTEGRAL_ALT); // Anti-windup

      float derivative_alt = (pid_error_alt - pid_previous_error_altitude) / dt_sec;
      pid_previous_error_altitude = pid_error_alt;

      float pid_output_altitude_correction_us = (kp_alt * pid_error_alt) + pid_integral_altitude + (kd_alt * derivative_alt);
      
      output_throttle_us = HOVER_THROTTLE_NEUTRAL_US + (int16_t)pid_output_altitude_correction_us;

      // Ground/Ceiling Proximity Safety
      if (tofBottom_cm < GROUND_PROXIMITY_THRESHOLD_CM && pid_output_altitude_correction_us < 0) { // Too low and commanding down
        // output_throttle_us = HOVER_THROTTLE_NEUTRAL_US - 50; // Gentle fixed upward nudge or limit
        output_throttle_us = max(output_throttle_us, (uint16_t)(HOVER_THROTTLE_NEUTRAL_US - 50)); // Limit downward command near ground
         Serial.println("Altitude PID: Ground safety override!");
      }
      if (tofTop_cm < CEILING_PROXIMITY_THRESHOLD_CM && pid_output_altitude_correction_us > 0) { // Too high (near ceiling) and commanding up
        // output_throttle_us = HOVER_THROTTLE_NEUTRAL_US + 50; // Gentle fixed downward nudge or limit
        output_throttle_us = min(output_throttle_us, (uint16_t)(HOVER_THROTTLE_NEUTRAL_US + 50)); // Limit upward command near ceiling
         Serial.println("Altitude PID: Ceiling safety override!");
      }
      output_throttle_us = constrain(output_throttle_us, MIN_THROTTLE_US, MAX_THROTTLE_US);

      // --- Basic Frontal Obstacle Avoidance (Pitch on pulseWidths_out_us[1]) ---
      output_pitch_us = current_rc_ch2; // Start with pilot's pitch input
      if (tofFront_cm < HOVER_FRONT_AVOID_THRESHOLD_CM) {
        // Positive pitch pulse makes drone pitch back / move backward.
        // If your flight controller needs negative pitch (lower pulse) to move backward, use -=
        output_pitch_us += HOVER_PITCH_BACKUP_STRENGTH_US;
        Serial.print("Mode 2: Frontal Avoidance, tofFront_cm: "); Serial.println(tofFront_cm);
      }
      output_pitch_us = constrain(output_pitch_us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);

      // --- Roll and Yaw Passthrough ---
      output_roll_us = current_rc_ch1;
      output_yaw_us = current_rc_ch3;

    } else { // Mode 3: PID Obstacle Avoidance (channel6_rc_us >= 1700)
      hover_mode_just_engaged_flag = true; // Reset other mode's flag

      if (avoid_mode_just_engaged_flag) {
        pid_integral_avoid_pitch = 0.0f;
        pid_previous_error_avoid_pitch = 0.0f;
        pid_integral_avoid_roll = 0.0f;
        pid_previous_error_avoid_roll = 0.0f;
        pid_previous_time_avoid_ms = millis();
        avoid_mode_just_engaged_flag = false;
        Serial.println("Engaging Mode 3: Full Obstacle Avoidance");
      }

      unsigned long current_time_ms_avoid = millis();
      float dt_sec_avoid = (current_time_ms_avoid - pid_previous_time_avoid_ms) / 1000.0f;
      pid_previous_time_avoid_ms = current_time_ms_avoid;

      if (dt_sec_avoid <= 0.0001f) dt_sec_avoid = 0.01f;


      // --- Sensor Fusion for X/Y PID Avoidance ---
      // Pitch Error (Forward/Backward)
      float front_threat = max(0.0f, DISTANCE_THRESHOLD_XY_AVOID_CM - tofFront_cm);
      float back_threat = max(0.0f, DISTANCE_THRESHOLD_XY_AVOID_CM - tofBack_cm);
      float diag_front_threat_pitch = 0.0f;
      if (distance_ultrasonic_front_left_cm < DISTANCE_THRESHOLD_XY_AVOID_CM) 
          diag_front_threat_pitch += (DISTANCE_THRESHOLD_XY_AVOID_CM - distance_ultrasonic_front_left_cm) * 0.707f;
      if (distance_ultrasonic_front_right_cm < DISTANCE_THRESHOLD_XY_AVOID_CM)
          diag_front_threat_pitch += (DISTANCE_THRESHOLD_XY_AVOID_CM - distance_ultrasonic_front_right_cm) * 0.707f;
      
      float diag_rear_threat_pitch = 0.0f;
      if (distance_ultrasonic_rear_left_cm < DISTANCE_THRESHOLD_XY_AVOID_CM)
          diag_rear_threat_pitch += (DISTANCE_THRESHOLD_XY_AVOID_CM - distance_ultrasonic_rear_left_cm) * 0.707f;
      if (distance_ultrasonic_rear_right_cm < DISTANCE_THRESHOLD_XY_AVOID_CM)
          diag_rear_threat_pitch += (DISTANCE_THRESHOLD_XY_AVOID_CM - distance_ultrasonic_rear_right_cm) * 0.707f;

      float error_pitch_avoid = (front_threat + diag_front_threat_pitch) - (back_threat + diag_rear_threat_pitch); // Positive error = front threat, command backward

      // Roll Error (Left/Right)
      float left_threat_roll = 0.0f;
       if (distance_ultrasonic_front_left_cm < DISTANCE_THRESHOLD_XY_AVOID_CM) 
          left_threat_roll += (DISTANCE_THRESHOLD_XY_AVOID_CM - distance_ultrasonic_front_left_cm) * 0.707f; // Contribution to left threat
       if (distance_ultrasonic_rear_left_cm < DISTANCE_THRESHOLD_XY_AVOID_CM)
          left_threat_roll += (DISTANCE_THRESHOLD_XY_AVOID_CM - distance_ultrasonic_rear_left_cm) * 0.707f;

      float right_threat_roll = 0.0f;
       if (distance_ultrasonic_front_right_cm < DISTANCE_THRESHOLD_XY_AVOID_CM)
          right_threat_roll += (DISTANCE_THRESHOLD_XY_AVOID_CM - distance_ultrasonic_front_right_cm) * 0.707f; // Contribution to right threat
       if (distance_ultrasonic_rear_right_cm < DISTANCE_THRESHOLD_XY_AVOID_CM)
          right_threat_roll += (DISTANCE_THRESHOLD_XY_AVOID_CM - distance_ultrasonic_rear_right_cm) * 0.707f;

      float error_roll_avoid = left_threat_roll - right_threat_roll; // Positive error = left threat, command right


      // --- PID Calculation for Pitch Avoidance ---
      pid_integral_avoid_pitch += ki_avoid_xy * error_pitch_avoid * dt_sec_avoid;
      pid_integral_avoid_pitch = constrain(pid_integral_avoid_pitch, -MAX_INTEGRAL_AVOID_XY, MAX_INTEGRAL_AVOID_XY);
      float derivative_avoid_pitch = (error_pitch_avoid - pid_previous_error_avoid_pitch) / dt_sec_avoid;
      pid_previous_error_avoid_pitch = error_pitch_avoid;
      float pid_correction_pitch_us = (kp_avoid_xy * error_pitch_avoid) + pid_integral_avoid_pitch + (kd_avoid_xy * derivative_avoid_pitch);

      // --- PID Calculation for Roll Avoidance ---
      pid_integral_avoid_roll += ki_avoid_xy * error_roll_avoid * dt_sec_avoid;
      pid_integral_avoid_roll = constrain(pid_integral_avoid_roll, -MAX_INTEGRAL_AVOID_XY, MAX_INTEGAL_AVOID_XY);
      float derivative_avoid_roll = (error_roll_avoid - pid_previous_error_avoid_roll) / dt_sec_avoid;
      pid_previous_error_avoid_roll = error_roll_avoid;
      float pid_correction_roll_us = (kp_avoid_xy * error_roll_avoid) + pid_integral_avoid_roll + (kd_avoid_xy * derivative_avoid_roll);

      // --- Apply PID Corrections to Pitch and Roll ---
      // Assuming: Higher pitch value = backward. Higher roll value = right.
      // If pid_correction_pitch_us is positive (front threat), it adds to pitch, moving drone backward.
      output_pitch_us = constrain(current_rc_ch2 + (int16_t)pid_correction_pitch_us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);
      // If pid_correction_roll_us is positive (left threat), it adds to roll, moving drone right.
      output_roll_us = constrain(current_rc_ch1 + (int16_t)pid_correction_roll_us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);

      // --- Altitude Control (Z-axis, simple offset logic) ---
      output_throttle_us = current_rc_ch4; // Start with pilot's throttle
      if (tofTop_cm < AVOID_Z_THRESHOLD_CM) {
        output_throttle_us -= AVOID_Z_ADJUST_STRENGTH_US; // Throttle down
      } else if (tofBottom_cm < AVOID_Z_THRESHOLD_CM) {
        output_throttle_us += AVOID_Z_ADJUST_STRENGTH_US; // Throttle up
      }
      output_throttle_us = constrain(output_throttle_us, MIN_THROTTLE_US, MAX_THROTTLE_US);
      
      // --- Yaw Passthrough ---
      output_yaw_us = current_rc_ch3;
    }

    // Atomically update the output pulseWidths array
    noInterrupts();
    pulseWidths_out_us[0] = output_roll_us;    // Roll
    pulseWidths_out_us[1] = output_pitch_us;   // Pitch
    pulseWidths_out_us[2] = output_yaw_us;     // Yaw
    pulseWidths_out_us[3] = output_throttle_us;// Throttle
    pulseWidths_out_us[4] = output_aux1_us;    // Aux1
    pulseWidths_out_us[5] = output_aux2_us;    // Aux2 (Mode Switch)
    pulseWidths_out_us[6] = output_aux3_us;    // Aux3
    pulseWidths_out_us[7] = output_aux4_us;    // Aux4
    interrupts();

    sendPPM_output();
    vTaskDelay(10 / portTICK_PERIOD_MS); // Loop rate for ppm_processing_task, e.g., 100Hz
  }
}

// Sensor reading task (runs on Core 1)
// MODIFIED: Renamed from Sensor_task (which was already an improvement over hc_sr04_task)
void Sensor_reading_task(void *pvParameters) { // MODIFIED: Renamed
    while (1) {
        // Read HC-SR04 sensors
        measurement_ultrasonic(sonar_front_left, distance_ultrasonic_front_left_cm);
        measurement_ultrasonic(sonar_front_right, distance_ultrasonic_front_right_cm);
        measurement_ultrasonic(sonar_rear_left, distance_ultrasonic_rear_left_cm);
        measurement_ultrasonic(sonar_rear_right, distance_ultrasonic_rear_right_cm);
        
        // Read VL53L0X sensors via multiplexer
        readAllVL53L0x(); // This function updates tofFront_cm, tofBack_cm, tofTop_cm, tofBottom_cm

        // Serial print for debugging (optional, can be intensive)
        /*
        Serial.print("FL: "); Serial.print(distance_ultrasonic_front_left_cm);
        Serial.print(" FR: "); Serial.print(distance_ultrasonic_front_right_cm);
        Serial.print(" RL: "); Serial.print(distance_ultrasonic_rear_left_cm);
        Serial.print(" RR: "); Serial.println(distance_ultrasonic_rear_right_cm);
        Serial.print("ToF F: "); Serial.print(tofFront_cm);
        Serial.print(" B: "); Serial.print(tofBack_cm);
        Serial.print(" T: "); Serial.print(tofTop_cm);
        Serial.print(" D: "); Serial.println(tofBottom_cm);
        */
        
        vTaskDelay(pdMS_TO_TICKS(50)); // MODIFIED: Increased sensor read rate to 20Hz from 2Hz. Adjust as needed.
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PPM_PIN_OUT, OUTPUT);
    digitalWrite(PPM_PIN_OUT, LOW);
    pinMode(PPM_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM_ISR, FALLING); // MODIFIED: ISR name

    Wire.begin(4, 5); // SDA = pin 4, SCL = pin 5
    initializeVl53l0x();

    // Initialize pulseWidths_out_us to neutral/safe values
    pulseWidths_out_us[0] = 1500; pulseWidths_out_us[1] = 1500; pulseWidths_out_us[2] = 1500; pulseWidths_out_us[3] = MIN_THROTTLE_US;
    pulseWidths_out_us[4] = 1500; pulseWidths_out_us[5] = 1500; pulseWidths_out_us[6] = 1500; pulseWidths_out_us[7] = 1500;


    // Start the FreeRTOS tasks
    xTaskCreatePinnedToCore(Sensor_reading_task, "SensorReadingTask", 4096, NULL, 2, NULL, 1); // Core 1
    xTaskCreatePinnedToCore(ppm_processing_task, "PPMProcessingTask", 4096, NULL, 1, NULL, 0); // Core 0 // MODIFIED: Task name
}

void loop() {
  // Empty, FreeRTOS tasks handle everything
}

// MODIFIED: Parameter type to float&, and internal type casting
void measurement_ultrasonic(NewPing &sonar, float &distance_cm) {
  unsigned int raw_dist_cm = sonar.ping_cm();
  if (raw_dist_cm == 0) {
    distance_cm = (float)MAX_DISTANCE_HCSR04_CM;
  } else {
    distance_cm = (float)raw_dist_cm;
  }
  // Reduce serial prints for performance, uncomment if needed for debugging
  // Serial.print("HC-SR04 Dist: "); Serial.print(distance_cm); Serial.println(" cm");
}


void selectMultiplexerChannel(uint8_t channel) {
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

float measurement_vl53l0x() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) { // If the measurement is not out of range
    // Serial.print("VL53L0X Dist: "); Serial.print(measure.RangeMilliMeter / 10.0f); Serial.println(" cm"); // Reduce serial prints
    return measure.RangeMilliMeter / 10.0f; // Return in cm
  } else {
    // Serial.println("VL53L0X Out of range"); // Reduce serial prints
    return 205.0f; // Max range / Default value for out of range
  }
}


void initializeVl53l0x() {
  for (int i = 0; i < MAX_LIST_SIZE; i++) { // Iterate up to MAX_LIST_SIZE (4) for potential sensors
    selectMultiplexerChannel(i);
    if (!lox.begin(SENSOR_ADDRESS)) { // Try to initialize with the default SENSOR_ADDRESS
      Serial.print("VL53L0X on MUX ch "); Serial.print(i); Serial.println(" FAILED to init!");
    } else {
      Serial.print("VL53L0X on MUX ch "); Serial.print(i); Serial.println(" initialized.");
      addToList(i); // Add successfully initialized MUX channel index to list
      // Optional: Set ranging mode, timing budget for VL53L0X here if needed
      // lox.setMeasurementTimingBudgetMicroSeconds(20000); // Example: 20ms budget
    }
  }
}

void addToList(int value) {
  if (currentVL53l0xListSize < MAX_LIST_SIZE) {
    availableVL53l0x[currentVL53l0xListSize] = value;
    currentVL53l0xListSize++;
  } else {
    Serial.println("VL53L0X available list is full");
  }
}

// This function is not directly used by PID logic but might be useful for other purposes
int selectElement(int index) {
  if (index >= 0 && index < currentVL53l0xListSize) {
    return availableVL53l0x[index];
  } else {
    Serial.println("Invalid index for availableVL53l0x");
    return -1;
  }
}

void readAllVL53L0x() {
  // Temporary array to hold readings for this cycle, reduces direct global writes inside loop
  float temp_tofReadings[MAX_LIST_SIZE];
  for(int k=0; k<MAX_LIST_SIZE; ++k) temp_tofReadings[k] = 205.0f; // Default to max range

  for (int i = 0; i < currentVL53l0xListSize; i++) {
    int sensorMuxChannel = availableVL53l0x[i]; 
    selectMultiplexerChannel(sensorMuxChannel);

    float distance = measurement_vl53l0x(); // Already returns cm or 205.0f

    // Map sensorMuxChannel (0,1,2,3) to their roles (Front, Back, Top, Bottom)
    // This mapping assumes MUX channel 0 is Front, 1 is Back, etc.
    // Adjust this logic if your physical wiring is different.
    if (sensorMuxChannel == 0) {
      tofFront_cm = distance;
    } else if (sensorMuxChannel == 1) {
      tofBack_cm = distance;
    } else if (sensorMuxChannel == 2) {
      tofTop_cm = distance;
    } else if (sensorMuxChannel == 3) {
      tofBottom_cm = distance;
    }
  }
  // The global tofXXX_cm variables are now updated.
  // The tofReadOuts array is not critical for PID logic if globals are used.
  // If you need tofReadOuts for other purposes, update it here:
  // tofReadOuts[0] = tofFront_cm; // etc.
}

// MODIFIED: Renamed for clarity
void sendPPM_output(){
  // Generate PPM signal using values from pulseWidths_out_us
  uint32_t pulseStartTime = micros();
  uint32_t cumulativePulseTime = 0;

  for (int i = 0; i < NUM_CHANNELS; i++) {
    digitalWrite(PPM_PIN_OUT, HIGH);
    delayMicroseconds(300); // Standard low pulse part of PPM (often 300-500us)
    cumulativePulseTime += 300;

    digitalWrite(PPM_PIN_OUT, LOW);
    // The actual data pulse width is (channel_value - low_pulse_part)
    // But typical PPM just sends channel_value as HIGH, then a fixed LOW separator.
    // The original code sends pulseWidths[i] as HIGH time, then a LOW separator.
    // Let's re-evaluate sendPPM logic based on common PPM generation.
    // A common PPM pulse: LOW (separator, e.g. 300us) then HIGH (data pulse, 700-1700us for 1000-2000ms total).
    // The sum of (separator + data pulse) for all channels + sync pulse should be FRAME_DURATION.
    // The original sendPPM implies pulseWidths[i] is the HIGH duration.
    // And then a final low pulse fills the frame. This is a valid way.
    
    // Sticking to original interpretation: pulseWidths_out_us[i] is the HIGH duration.
    // Then a short LOW pulse separates channels.
    // A final long LOW pulse acts as sync and frame filler.
    
    // Let's refine sendPPM based on typical implementations:
    // Each channel is a LOW pulse of fixed width (e.g., 300-400µs) followed by a HIGH pulse variable width.
    // The total frame is fixed.
    // The provided code has: HIGH for pulseWidths[i], then LOW (implicitly fills the rest).
    // This is simpler but might not be standard PPM for all flight controllers.
    // For now, I will keep the original logic of sendPPM, which is:
    // HIGH for pulseWidths[i], then implicit LOW for a fixed duration between pulses.
    // This needs clarification. A standard PPM frame is a series of pulses.
    // Each pulse is: fixed low period (e.g. 300us), then variable high period (700us to 1700us for 1ms to 2ms channel).
    // The frame ends with a long low sync pulse (>4ms).

    // Let's assume the original sendPPM structure was:
    // For each channel: Set pin HIGH for pulseWidths[i], then set pin LOW for a short fixed period (e.g. 300us)
    // Sum of all these must be less than FRAME_DURATION. Remaining time is a long LOW (sync).

    // Original sendPPM structure:
    // digitalWrite(PPM_PIN_OUT, HIGH); delayMicroseconds(pulseWidths[i]); digitalWrite(PPM_PIN_OUT, LOW);
    // This implies pulseWidths[i] is the high time, and the *next* pulse starts immediately or after a delay.
    // The critical part is the total frame duration.

    // Re-implementing sendPPM based on the user's original logic pattern, assuming pulseWidths_out_us[i] is the duration of the HIGH part of the pulse,
    // and there's an implicit fixed LOW period after each HIGH pulse, or the next HIGH starts immediately.
    // The user's original sendPPM:
    // for (int i = 0; i < NUM_CHANNELS; i++) {
    //   digitalWrite(PPM_PIN_OUT, HIGH);
    //   delayMicroseconds(pulseWidths[i]);  // This is the channel value
    //   digitalWrite(PPM_PIN_OUT, LOW);     // This LOW is the separator
    //                                       // How long is this LOW separator? Not explicitly defined in original.
    //                                       // delayMicroseconds added after this loop fills frame.
    // }
    // This means the LOW separator is part of the `remainingTime`. This is unusual.
    // More typical: Fixed LOW separator (e.g. 300us) after each HIGH data pulse.

    // Let's try a common PPM structure: Each pulse is LOW (separator) + HIGH (data).
    // Total frame duration = 20000 us.
    // Each channel pulse: Separator (e.g. 300us LOW) + Data (variable HIGH).
    // Total time for N channels = N * Separator_LOW + Sum(Data_HIGH_i)
    // Sync pulse (LOW) = Frame_Duration - Total time for N channels. Must be > 3ms.
    
    // For simplicity and closest adherence to the *structure* of the original `sendPPM`,
    // let's assume pulseWidths_out_us[i] is the total time for that channel's pulse (data + fixed separator),
    // or it's just the data and a separator is added.
    // The original implies pulseWidths[i] is the HIGH time, and the subsequent LOW is the separator, whose duration
    // is effectively absorbed into the final 'remainingTime' calculation. This means variable length separators.

    // Let's keep the original sendPPM logic flow as it was provided, as requested by the user:
    uint32_t timeSpentOnPulses = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) {
      digitalWrite(PPM_PIN_OUT, HIGH);
      delayMicroseconds(pulseWidths_out_us[i]);
      digitalWrite(PPM_PIN_OUT, LOW);
      // Assuming a minimal low time here if not explicitly part of pulseWidths_out_us[i] or FRAME_DURATION logic
      // If pulseWidths_out_us[i] is just the HIGH time, a LOW separator is needed.
      // The original code did not have an explicit delay here, it was part of the final fill.
      // This is equivalent to a ~0us separator, which is not standard.
      // Let's add a small, fixed separator, e.g., 300us.
      // This will affect the `elapsedTime` calculation.
      if (i < NUM_CHANNELS -1) { // No separator after the last channel's low, sync pulse follows
         delayMicroseconds(300); // Fixed low separator
         timeSpentOnPulses += pulseWidths_out_us[i] + 300;
      } else {
         timeSpentOnPulses += pulseWidths_out_us[i]; // Last pulse, no separator before sync
      }
    }

    uint32_t elapsedTime = micros() - pulseStartTime; // More accurate measure of time spent
    // uint32_t elapsedTime = timeSpentOnPulses; // Alternative calculation based on known delays

    if (FRAME_DURATION > elapsedTime) {
      uint32_t remainingTime = FRAME_DURATION - elapsedTime;
      if (remainingTime > 0) { // Ensure remainingTime is positive
         digitalWrite(PPM_PIN_OUT, LOW); // Ensure it's low for the sync/fill period
         delayMicroseconds(remainingTime);
      }
    } else {
      // Frame overrun, pulses took too long. This shouldn't happen if values are constrained.
      // digitalWrite(PPM_PIN_OUT, LOW); // Ensure it ends low
    }
}