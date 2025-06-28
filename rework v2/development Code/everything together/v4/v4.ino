#include <Arduino.h>
#include <FreeRTOS.h>
#include <NewPing.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Multiplexer and Sensor Addresses
#define MULTIPLEXER_ADDRESS 0x70
#define SENSOR_ADDRESS 0x29
#define MAX_LIST_SIZE 4

int availableVL53l0x[MAX_LIST_SIZE];
int currentVL53l0xListSize = 0;

// HC-SR04 Pins
#define TRIG_PIN_FRONT_LEFT  1
#define ECHO_PIN_FRONT_LEFT  2
#define TRIG_PIN_FRONT_RIGHT 3
#define ECHO_PIN_FRONT_RIGHT 6
#define TRIG_PIN_REAR_RIGHT  7
#define ECHO_PIN_REAR_RIGHT  8
#define TRIG_PIN_REAR_LEFT   9
#define ECHO_PIN_REAR_LEFT   10

// Sensor data variables (used by both tasks, consider mutex if contention becomes an issue)
unsigned int distance_ultrasonic_front_left;
unsigned int distance_ultrasonic_front_right;
unsigned int distance_ultrasonic_rear_left;
unsigned int distance_ultrasonic_rear_right;
float tofFront = (float)MAX_DISTANCE;
float tofBack = (float)MAX_DISTANCE;
float tofTop = (float)MAX_DISTANCE;
float tofBottom = (float)MAX_DISTANCE;
// This array is mostly for debugging or external checking now
float tofReadOuts[4] = {MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE};


// Avoidance Parameters
int distanceThreshold_xy = 100; // cm
int distanceThreshold_z = 150;  // cm
const uint16_t MIN_THROTTLE = 1000;
const uint16_t MAX_THROTTLE = 2000;
const uint16_t HOVER_THROTTLE_NEUTRAL = 1500; // Approx. throttle for hover, TUNE THIS!

// PPM Input Pin
#define PPM_PIN 13

// PPM Channel Variables
volatile uint16_t channel1 = 1500; // Roll
volatile uint16_t channel2 = 1500; // Pitch
volatile uint16_t channel3 = 1500; // Yaw
volatile uint16_t channel4 = 1000; // Throttle
volatile uint16_t channel5 = 1500; // Aux1
volatile uint16_t channel6 = 1500; // Aux2 (Avoidance Mode Switch)
volatile uint16_t channel7 = 1500; // Aux3 (Hover Mode Switch)
volatile uint16_t channel8 = 1500; // Aux4

#define NUM_CHANNELS 8

// PPM Output Pin
#define PPM_PIN_OUT 12
#define FRAME_DURATION 20000
#define MIN_PULSE_WIDTH 1000
#define MAX_PULSE_WIDTH 2000

#define MAX_DISTANCE 205 // cm (for HC-SR04 and default for TOF out of range)

uint16_t pulseWidths[NUM_CHANNELS];
volatile uint8_t ppm_input_current_channel = 0;

// Sensor Objects
NewPing sonar_front_left(TRIG_PIN_FRONT_LEFT, ECHO_PIN_FRONT_LEFT, MAX_DISTANCE);
NewPing sonar_front_right(TRIG_PIN_FRONT_RIGHT, ECHO_PIN_FRONT_RIGHT, MAX_DISTANCE);
NewPing sonar_rear_left(TRIG_PIN_REAR_LEFT, ECHO_PIN_REAR_LEFT, MAX_DISTANCE);
NewPing sonar_rear_right(TRIG_PIN_REAR_RIGHT, ECHO_PIN_REAR_RIGHT, MAX_DISTANCE);
Adafruit_VL53L0X lox;

// --- PID Controller Structures and Constants ---
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float previous_error;
    float output_min;
    float output_max;
} PIDController;

// PIDs for XY obstacle avoidance
PIDController pid_roll = {0.5f, 0.01f, 0.1f, 0.0f, 0.0f, -200.0f, 200.0f};   // Kp, Ki, Kd, integral, prev_err, min_out, max_out
PIDController pid_pitch = {0.5f, 0.01f, 0.1f, 0.0f, 0.0f, -200.0f, 200.0f};

// PID for Altitude Hold (Hover Mode) - GAINS NEED CAREFUL TUNING!
PIDController pid_altitude = {2.5f, 0.1f, 0.75f, 0.0f, 0.0f, -300.0f, 300.0f}; 
// Example gains: Kp_alt, Ki_alt, Kd_alt, integral, prev_err, min_throttle_adj, max_throttle_adj

const float PID_DT = 0.01f; // Loop time for ppm_task in seconds (10ms)
const int baseAdjustValue = 100; // For non-PID Z adjustments

// --- Hover Mode Variables ---
const float TARGET_HOVER_ALTITUDE_CM = 150.0f; // 1.5 meters
bool hover_mode_active_internal = false; // Internal state for hover mode

// --- Function Prototypes --- (Same as your provided list)
void ppm_task(void *pvParameters);
void Sensor_task(void *pvParameters);
void readPPM();
void sendPPM();
void measurement_ultrasonic(NewPing &sonar, unsigned int &distance_var); // Changed param name slightly
void selectMultiplexerChannel(uint8_t channel);
float measurement_vl53l0x();
void initializeVl53l0x();
void addToList(int value);
void readAllVL53L0x();
float calculate_threat_level(float current_distance_val, int threshold_val); // Changed param names slightly
float calculate_pid(PIDController *pid, float error, float dt);


// --- Interrupt handler for the PPM signal (PPM Input) ---
void readPPM() {
  static uint32_t lastTime = 0;
  uint32_t currentTime = micros();
  uint32_t interval = currentTime - lastTime;
  lastTime = currentTime;

  if (interval >= 3000) {
    ppm_input_current_channel = 0;
  } else {
    if (ppm_input_current_channel < NUM_CHANNELS) {
        switch (ppm_input_current_channel) {
            case 0: channel1 = (interval >= 50) ? interval - 50 : MIN_PULSE_WIDTH; break;
            case 1: channel2 = interval; break;
            case 2: channel3 = interval; break;
            case 3: channel4 = interval; break;
            case 4: channel5 = interval; break;
            case 5: channel6 = interval; break; // Avoidance Mode
            case 6: channel7 = interval; break; // Hover Mode
            case 7: channel8 = interval; break;
        }
    }
    ppm_input_current_channel++;
  }
}

// Helper function to calculate threat level (0.0 to 1.0)
float calculate_threat_level(float current_distance_val, int threshold_val) { // param names updated
    if (current_distance_val < 0) current_distance_val = 0;
    if (threshold_val <= 0) return 0.0f;
    if (current_distance_val < (float)threshold_val) {
        return ( (float)threshold_val - current_distance_val ) / (float)threshold_val;
    }
    return 0.0f;
}

// --- PID Calculation Function ---
// (Using your provided PID calculation, ensure it handles anti-windup if needed or relies on output clamping)
float calculate_pid(PIDController *pid, float error, float dt) {
    float P_out = pid->Kp * error;
    
    // Basic Anti-windup: only integrate if output is not saturated
    // Or if integral term would reduce saturation
    float potential_output_no_I = P_out + (pid->Kd * (error - pid->previous_error) / dt);
    if (!((potential_output_no_I >= pid->output_max && error * pid->Ki > 0) ||
          (potential_output_no_I <= pid->output_min && error * pid->Ki < 0))) {
        pid->integral += error * dt;
    }
    // Clamp integral to prevent excessive buildup (e.g. limit to contribute max 70-80% of total output_max)
    float max_integral_val = (pid->output_max / (pid->Ki != 0 ? pid->Ki : 1.0f)) * 0.7f ; // Avoid div by zero
    if (pid->integral > max_integral_val) pid->integral = max_integral_val;
    if (pid->integral < -max_integral_val) pid->integral = -max_integral_val;
    
    float I_out = pid->Ki * pid->integral;
    float derivative = (error - pid->previous_error) / dt;
    float D_out = pid->Kd * derivative;
    float output = P_out + I_out + D_out;

    if (output > pid->output_max) output = pid->output_max;
    else if (output < pid->output_min) output = pid->output_min;
    
    pid->previous_error = error;
    return output;
}


// --- PPM reading, algorithm, and signal generation task (runs on Core 0) ---
void ppm_task(void *pvParameters) {
  // Using global sensor variables directly in this task for now, as per your provided code.
  // For more robustness, copying them locally with a mutex is better.

  uint16_t pilot_ch1, pilot_ch2, pilot_ch3, pilot_ch4, pilot_ch5, pilot_ch6_mode, pilot_ch7_hover, pilot_ch8;

  while (true) {
    // --- Read pilot inputs (volatile channels) ---
    noInterrupts();
    pilot_ch1 = channel1; pilot_ch2 = channel2; pilot_ch3 = channel3; pilot_ch4 = channel4;
    pilot_ch5 = channel5; pilot_ch6_mode = channel6; pilot_ch7_hover = channel7; pilot_ch8 = channel8;
    interrupts();

    // --- Constrain pilot inputs to valid PPM range ---
    pilot_ch1 = constrain(pilot_ch1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    pilot_ch2 = constrain(pilot_ch2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    pilot_ch3 = constrain(pilot_ch3, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    pilot_ch4 = constrain(pilot_ch4, MIN_THROTTLE, MAX_THROTTLE); // Pilot's desired throttle
    pilot_ch5 = constrain(pilot_ch5, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    pilot_ch6_mode = constrain(pilot_ch6_mode, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    pilot_ch7_hover = constrain(pilot_ch7_hover, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    pilot_ch8 = constrain(pilot_ch8, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

    // Initialize output pulse widths with pilot's constrained intent
    pulseWidths[0] = pilot_ch1;
    pulseWidths[1] = pilot_ch2;
    pulseWidths[2] = pilot_ch3;
    pulseWidths[3] = pilot_ch4; // This will be overridden by hover or avoidance if active
    pulseWidths[4] = pilot_ch5;
    pulseWidths[5] = pilot_ch6_mode; // Output the mode switch channel
    pulseWidths[6] = pilot_ch7_hover; // Output the hover switch channel
    pulseWidths[7] = pilot_ch8;


    // --- Determine Hover Mode State ---
    if (pilot_ch7_hover > 1700) { // CH7 High -> Hover Mode Active
        if (!hover_mode_active_internal) { // Just entered hover mode
            hover_mode_active_internal = true;
            pid_altitude.integral = 0.0f; // Reset altitude PID integral
            pid_altitude.previous_error = 0.0f;
            // Serial.println("Hover Mode ENGAGED");
        }
    } else { // CH7 Low/Mid -> Hover Mode Inactive
        if (hover_mode_active_internal) {
            // Serial.println("Hover Mode DISENGAGED");
        }
        hover_mode_active_internal = false;
        pid_altitude.integral = 0.0f; // Reset altitude PID when not active
    }

    // --- Main Control Logic ---
    if (hover_mode_active_internal) {
        // --- HOVER MODE ACTIVE ---
        // 1. Altitude PID for Throttle
        float altitude_error = TARGET_HOVER_ALTITUDE_CM - tofBottom; // Use global tofBottom
        if (tofBottom >= MAX_DISTANCE -1 && altitude_error > 0) { // Safety: if sensor maxed out and error says go up
            altitude_error = 0; // Don't try to climb if altitude reading is unreliable (at max)
        }
        float altitude_pid_output = calculate_pid(&pid_altitude, altitude_error, PID_DT);
        pulseWidths[3] = constrain(HOVER_THROTTLE_NEUTRAL + (int16_t)altitude_pid_output, MIN_THROTTLE, MAX_THROTTLE);

        // 2. Default Roll/Pitch/Yaw to neutral for hover
        pulseWidths[0] = 1500; // Neutral Roll
        pulseWidths[1] = 1500; // Neutral Pitch
        // pulseWidths[2] = 1500; // Option: Neutral Yaw or maintain pilot_ch3

        // 3. If Avoidance Mode 3 is also active, let its PIDs adjust Roll/Pitch and Z-safety override throttle
        if (pilot_ch6_mode < 1300) { // Avoidance Mode 3 active
            // Z-axis obstacle avoidance (overrides altitude PID if too close to top/bottom)
            if(tofTop <=(float)distanceThreshold_z) {
                uint16_t current_throttle = pulseWidths[3];
                current_throttle = constrain(current_throttle,(uint16_t)(MIN_THROTTLE+baseAdjustValue),MAX_THROTTLE);
                pulseWidths[3] = current_throttle - baseAdjustValue; // Throttle down
            } else if(tofBottom <=(float)distanceThreshold_z && tofBottom < TARGET_HOVER_ALTITUDE_CM * 0.75f) { // If very close to ground
                uint16_t current_throttle = pulseWidths[3];
                current_throttle = constrain(current_throttle,MIN_THROTTLE,(uint16_t)(MAX_THROTTLE-baseAdjustValue));
                pulseWidths[3] = current_throttle + baseAdjustValue; // Throttle up
            }

            // XY obstacle avoidance PIDs (adjusts from neutral hover setpoints)
            float f_threat=0.0f, b_threat=0.0f, l_threat=0.0f, r_threat=0.0f, threat;
            threat=calculate_threat_level((float)distance_ultrasonic_front_left,distanceThreshold_xy); if(threat>0.0f){b_threat+=threat; r_threat+=threat;}
            threat=calculate_threat_level((float)distance_ultrasonic_front_right,distanceThreshold_xy); if(threat>0.0f){b_threat+=threat; l_threat+=threat;}
            threat=calculate_threat_level((float)distance_ultrasonic_rear_left,distanceThreshold_xy); if(threat>0.0f){f_threat+=threat; r_threat+=threat;}
            threat=calculate_threat_level((float)distance_ultrasonic_rear_right,distanceThreshold_xy); if(threat>0.0f){f_threat+=threat; l_threat+=threat;}
            threat=calculate_threat_level(tofFront,distanceThreshold_xy); if(threat>0.0f){b_threat+=threat;}
            threat=calculate_threat_level(tofBack,distanceThreshold_xy); if(threat>0.0f){f_threat+=threat;}

            float roll_err = r_threat - l_threat; float roll_adj = calculate_pid(&pid_roll, roll_err, PID_DT);
            float pitch_err = b_threat - f_threat; float pitch_adj = calculate_pid(&pid_pitch, pitch_err, PID_DT);
            
            pulseWidths[0] = 1500 - (int16_t)roll_adj; 
            pulseWidths[1] = 1500 + (int16_t)pitch_adj;
        } else { // If not in Mode 3 avoidance (e.g., Mode 1 or 2 with Hover)
            pid_roll.integral=0.0f; pid_roll.previous_error=0.0f;
            pid_pitch.integral=0.0f; pid_pitch.previous_error=0.0f;
        }
    } else { // --- STANDARD FLIGHT MODES (Hover Mode INACTIVE) ---
        pid_altitude.integral = 0.0f; // Reset altitude PID integral if hover is not active

        if (pilot_ch6_mode >= 1700) { // Mode 1: Manual
            pid_roll.integral=0.0f; pid_roll.previous_error=0.0f;
            pid_pitch.integral=0.0f; pid_pitch.previous_error=0.0f;
            // pulseWidths already set to pilot inputs at the start of the loop
        } else if (pilot_ch6_mode >= 1300) { // Mode 2: Height Assist (non-hover)
            pid_roll.integral=0.0f; pid_roll.previous_error=0.0f;
            pid_pitch.integral=0.0f; pid_pitch.previous_error=0.0f;
            
            // Apply Z-axis adjustments to pilot's current throttle
            if(tofTop <=(float)distanceThreshold_z) {
                pulseWidths[3]=constrain(pilot_ch4,(uint16_t)(MIN_THROTTLE+baseAdjustValue),MAX_THROTTLE);
                pulseWidths[3]-=baseAdjustValue;
            } else if(tofBottom <=(float)distanceThreshold_z) {
                pulseWidths[3]=constrain(pilot_ch4,MIN_THROTTLE,(uint16_t)(MAX_THROTTLE-baseAdjustValue));
                pulseWidths[3]+=baseAdjustValue;
            }
        } else { // Mode 3: Full Avoidance (non-hover)
            // Apply Z-axis adjustments to pilot's current throttle
            if(tofTop <=(float)distanceThreshold_z) {
                pulseWidths[3]=constrain(pilot_ch4,(uint16_t)(MIN_THROTTLE+baseAdjustValue),MAX_THROTTLE);
                pulseWidths[3]-=baseAdjustValue;
            } else if(tofBottom <=(float)distanceThreshold_z) {
                pulseWidths[3]=constrain(pilot_ch4,MIN_THROTTLE,(uint16_t)(MAX_THROTTLE-baseAdjustValue));
                pulseWidths[3]+=baseAdjustValue;
            }
            
            // Apply XY PIDs to pilot's current roll/pitch
            float f_threat=0.0f, b_threat=0.0f, l_threat=0.0f, r_threat=0.0f, threat;
            threat=calculate_threat_level((float)distance_ultrasonic_front_left,distanceThreshold_xy); if(threat>0.0f){b_threat+=threat; r_threat+=threat;}
            threat=calculate_threat_level((float)distance_ultrasonic_front_right,distanceThreshold_xy); if(threat>0.0f){b_threat+=threat; l_threat+=threat;}
            threat=calculate_threat_level((float)distance_ultrasonic_rear_left,distanceThreshold_xy); if(threat>0.0f){f_threat+=threat; r_threat+=threat;}
            threat=calculate_threat_level((float)distance_ultrasonic_rear_right,distanceThreshold_xy); if(threat>0.0f){f_threat+=threat; l_threat+=threat;}
            threat=calculate_threat_level(tofFront,distanceThreshold_xy); if(threat>0.0f){b_threat+=threat;}
            threat=calculate_threat_level(tofBack,distanceThreshold_xy); if(threat>0.0f){f_threat+=threat;}

            float roll_err = r_threat - l_threat; float roll_adj = calculate_pid(&pid_roll, roll_err, PID_DT);
            float pitch_err = b_threat - f_threat; float pitch_adj = calculate_pid(&pid_pitch, pitch_err, PID_DT);
            
            pulseWidths[0] = pilot_ch1 - (int16_t)roll_adj; 
            pulseWidths[1] = pilot_ch2 + (int16_t)pitch_adj;
        }
    }

    // Final constraint on all channels before sending
    pulseWidths[0]=constrain(pulseWidths[0],MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
    pulseWidths[1]=constrain(pulseWidths[1],MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
    pulseWidths[2]=constrain(pulseWidths[2],MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
    pulseWidths[3]=constrain(pulseWidths[3],MIN_THROTTLE,MAX_THROTTLE);
    pulseWidths[4]=constrain(pulseWidths[4],MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
    pulseWidths[5]=constrain(pulseWidths[5],MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
    pulseWidths[6]=constrain(pulseWidths[6],MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
    pulseWidths[7]=constrain(pulseWidths[7],MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
    
    sendPPM();
    vTaskDelay(pdMS_TO_TICKS(10)); // ~100Hz loop for algorithm
  }
}

// --- Sensor reading task (runs on Core 1) ---
void Sensor_task(void *pvParameters) {
    while (1) {
        // Read all sensors and update global variables
        // This part is assumed to be fast enough for the 50ms delay.
        measurement_ultrasonic(sonar_front_left, distance_ultrasonic_front_left);
        measurement_ultrasonic(sonar_front_right, distance_ultrasonic_front_right);
        measurement_ultrasonic(sonar_rear_left, distance_ultrasonic_rear_left);
        measurement_ultrasonic(sonar_rear_right, distance_ultrasonic_rear_right);
        
        // Read all TOF sensors
        // This function updates global tofFront, tofBack, tofTop, tofBottom
        readAllVL53L0x(); 

        vTaskDelay(pdMS_TO_TICKS(50)); // Sensor update rate (20Hz)
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000); // Wait for serial, but timeout
    Serial.println("Booting up with Hover Mode, PID Obstacle Avoidance...");

    pinMode(PPM_PIN_OUT, OUTPUT); digitalWrite(PPM_PIN_OUT, LOW);
    pinMode(PPM_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, FALLING);

    Wire.begin(4, 5); // Ensure these are correct I2C pins for your ESP32-S3-Zero
    Serial.println("I2C Initialized.");
    
    initializeVl53l0x(); // Initialize all TOF sensors

    Serial.println("Creating FreeRTOS tasks...");
    xTaskCreatePinnedToCore(Sensor_task, "Sensor_task", 4096, NULL, 2, NULL, 1); 
    xTaskCreatePinnedToCore(ppm_task, "PPM_Algo_Task", 4096, NULL, 1, NULL, 0); 
    Serial.println("Tasks created. System running.");
}

// --- Loop (not used with FreeRTOS) ---
void loop() { vTaskDelete(NULL); }


// --- Sensor and Utility Function Implementations ---
// (Copied from your previous "original sendPPM" version for completeness,
//  and assuming `readAllVL53L0x` now correctly updates all global TOF vars)

void measurement_ultrasonic(NewPing &sonar, unsigned int &distance_var) { // param name changed
  unsigned int dist_cm = sonar.ping_cm();
  distance_var = (dist_cm == 0 || dist_cm > MAX_DISTANCE) ? MAX_DISTANCE : dist_cm;
}

void selectMultiplexerChannel(uint8_t channel) {
  if (channel > 7) return; // Max 8 channels (0-7) for TCA9548A
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  Wire.write(1 << channel);
  if (Wire.endTransmission() != 0) {
      // Serial.print("MUX Select Fail ch: "); Serial.println(channel);
  }
  delay(1); // Small delay after MUX channel switch
}

float measurement_vl53l0x() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // false for faster ranging
  if (measure.RangeStatus != 4 && measure.RangeMilliMeter > 0) { // if not out of range and valid
    return (float)measure.RangeMilliMeter / 10.0f; // Convert mm to cm
  }
  return (float)MAX_DISTANCE; // Return max distance if out of range or error
}

void initializeVl53l0x() {
  Serial.println("Initializing VL53L0X sensors...");
  currentVL53l0xListSize = 0; // Reset list
  for (uint8_t i = 0; i < 4; i++) { // Assuming up to 4 VL53L0X sensors on MUX channels 0-3
    selectMultiplexerChannel(i);
    // Serial.print("Attempting to init VL53L0X on MUX channel "); Serial.print(i);
    if (lox.begin(SENSOR_ADDRESS, false, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY)) {
      lox.setMeasurementTimingBudgetMicroSeconds(20000); // 20ms budget for faster reads
      addToList(i); // Add successfully initialized MUX channel index
      // Serial.println(" ...SUCCESS!");
    } else {
      // Serial.println(" ...FAILED!");
    }
  }
  Serial.print("Available VL53L0X sensors: "); Serial.println(currentVL53l0xListSize);
}

void addToList(int value) { // value is the MUX channel index
  if (currentVL53l0xListSize < MAX_LIST_SIZE) {
    // Check if already in list to prevent duplicates if called multiple times
    bool found = false;
    for(int i=0; i<currentVL53l0xListSize; ++i) { if(availableVL53l0x[i] == value) {found=true; break;}}
    if(!found) {
        availableVL53l0x[currentVL53l0xListSize++] = value;
    }
  } else {
    // Serial.println("VL53L0X available list is full");
  }
}

void readAllVL53L0x() {
  // This function now directly updates global tofFront, tofBack, tofTop, tofBottom
  // It iterates through successfully initialized sensors.
  
  // Temporarily set to MAX_DISTANCE to indicate no valid reading yet for this cycle
  // for sensors that might not be in the available list.
  // However, only available sensors will update their respective globals.
  // If a sensor was previously available and now isn't, its global var will retain the old value
  // until Sensor_task attempts recovery or it's re-initialized.
  // A better approach might be to set all to MAX_DISTANCE here if not read.

  float temp_tofFront = MAX_DISTANCE, temp_tofBack = MAX_DISTANCE;
  float temp_tofTop = MAX_DISTANCE, temp_tofBottom = MAX_DISTANCE;

  for (int i = 0; i < currentVL53l0xListSize; i++) {
    int mux_channel = availableVL53l0x[i]; // Get the MUX channel for this sensor
    selectMultiplexerChannel(mux_channel);
    float distance = measurement_vl53l0x();

    // Assign to the correct temporary TOF variable based on MUX channel
    // This mapping depends on how you've wired them to the MUX channels.
    // Assuming: Channel 0 = Front, 1 = Back, 2 = Top, 3 = Bottom
    switch(mux_channel) {
        case 0: temp_tofFront = distance; break;
        case 1: temp_tofBack  = distance; break;
        case 2: temp_tofTop   = distance; break;
        case 3: temp_tofBottom= distance; break;
    }
  }
  // Update global variables
  tofFront = temp_tofFront;
  tofBack = temp_tofBack;
  tofTop = temp_tofTop;
  tofBottom = temp_tofBottom;

  // Update tofReadOuts array (for debugging or external checking)
  tofReadOuts[0] = tofFront;
  tofReadOuts[1] = tofBack;
  tofReadOuts[2] = tofTop;
  tofReadOuts[3] = tofBottom;
}

void sendPPM(){ // Your original sendPPM
    uint32_t pulseStartTime = micros();
    for (int i = 0; i < NUM_CHANNELS; i++) {
      digitalWrite(PPM_PIN_OUT, HIGH);
      delayMicroseconds(pulseWidths[i]);
      digitalWrite(PPM_PIN_OUT, LOW);
    }
    uint32_t elapsedTime = micros() - pulseStartTime;
    uint32_t remainingTime = FRAME_DURATION - elapsedTime;
    if (remainingTime > 0) {
      digitalWrite(PPM_PIN_OUT, LOW); 
      delayMicroseconds(remainingTime);
    }
}