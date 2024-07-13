// for simplicity removed the multiplexer stuff


#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// TOF sensor address
#define SENSOR_ADDRESS 0x29       // I2C address of the VL53L0X sensor

float MeasurementTof = 4000;     // Default distance value in mm
Adafruit_VL53L0X lox;

// PPM settings and variables
#define PPM_PIN 2  // PPM input pin
#define PPM_OUT_PIN 6  // PPM output pin
#define NUM_CHANNELS 8  // Number of PPM channels
#define PPM_PERIOD 20000  // Total PPM frame length in microseconds (20ms)
#define PULSE_LENGTH 300  // Length of sync pulse in microseconds
#define MIN_CHANNEL_PULSE 1000  // Minimum channel pulse length in microseconds
#define MAX_CHANNEL_PULSE 2000  // Maximum channel pulse length in microseconds

volatile uint16_t channel1 = 1500;
volatile uint16_t channel2 = 1500;
volatile uint16_t channel3 = 1500;
volatile uint16_t channel4 = 1000;
volatile uint16_t channel5;
volatile uint16_t channel6;
volatile uint16_t channel7;
volatile uint16_t channel8;

volatile uint8_t currentChannel = 0;

uint16_t channelValues[NUM_CHANNELS];

// PID control variables
float targetAltitude = 1000.0; // Target altitude in mm
float kp = 1.0, ki = 0.1, kd = 0.05;
float previousError = 0, integral = 0;
unsigned long lastTime = 0;

// Function declarations
void readPPM();
void sendPPM();
void doMeasurementTOF();
void updatePID(float currentAltitude);

void setup() {
  Serial.begin(9600);  // Start serial communication for debugging
  Serial.println("Start of program");
  
  pinMode(PPM_PIN, INPUT_PULLUP); // Enable internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, FALLING);  // Set up an interrupt on PPM_PIN

  pinMode(PPM_OUT_PIN, OUTPUT);
  digitalWrite(PPM_OUT_PIN, HIGH);


  // TOF setup
  Wire.begin(); // Start the I2C communication
  if (!lox.begin(SENSOR_ADDRESS)) {
    Serial.println("Failed to boot VL53L0X sensor");
    while (1);  // Stop further execution if sensor initialization fails
  }

  lastTime = millis(); // Initialize lastTime for PID calculation
}

void loop() {
  // Print PPM values
  noInterrupts(); // Temporarily disable interrupts to safely read the volatile variables
  uint16_t ch1 = channel1;
  uint16_t ch2 = channel2;
  uint16_t ch3 = channel3;
  uint16_t ch4 = channel4;
  uint16_t ch5 = channel5;
  uint16_t ch6 = channel6;
  uint16_t ch7 = channel7;
  uint16_t ch8 = channel8;
  interrupts(); // Re-enable interrupts

  // Update TOF measurement and PID control
  doMeasurementTOF();
  updatePID(MeasurementTof);

  // Update channel values for PPM output
  channelValues[0] = ch1;
  channelValues[1] = ch2;
  channelValues[2] = ch3;
  channelValues[3] = ch4;
  channelValues[4] = ch5;
  channelValues[5] = ch6;
  channelValues[6] = ch7;
  channelValues[7] = ch8;

  sendPPM();
  delay(20);  // 50Hz refresh rate
}

void sendPPM() {
  uint32_t frameStartTime = micros();
  uint32_t lastPulseEndTime = frameStartTime;

  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    // Calculate the exact time to wait before the next pulse
    uint32_t pulseStartTime = lastPulseEndTime + (channelValues[i] - PULSE_LENGTH);

    // Wait for the time to send the next pulse
    while (micros() < pulseStartTime) {
      // Busy wait
    }

    // Send the channel pulse
    digitalWrite(PPM_OUT_PIN, HIGH);
    delayMicroseconds(PULSE_LENGTH);
    digitalWrite(PPM_OUT_PIN, LOW);

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
  digitalWrite(PPM_OUT_PIN, HIGH);
  delayMicroseconds(PULSE_LENGTH);
  digitalWrite(PPM_OUT_PIN, LOW);
}

void readPPM() {
  static uint32_t lastTime = 0;
  uint32_t currentTime = micros();
  uint32_t interval = currentTime - lastTime;
  lastTime = currentTime;

  if (interval >= 3000) {
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

// TOF measurement function
void doMeasurementTOF() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4) {
    MeasurementTof = measure.RangeMilliMeter;
  } else {
    MeasurementTof = 4000; // Default to 4000 mm if measurement is invalid
  }
}

// PID control function
void updatePID(float currentAltitude) {
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - lastTime) / 1000.0;
  float error = targetAltitude - currentAltitude;
  integral += error * elapsedTime;
  float derivative = (error - previousError) / elapsedTime;

  float output = kp * error + ki * integral + kd * derivative;

  // Adjust channel4 based on PID output
  channel4 = constrain(channel4 + output, MIN_CHANNEL_PULSE, MAX_CHANNEL_PULSE);

  previousError = error;
  lastTime = currentTime;
}
