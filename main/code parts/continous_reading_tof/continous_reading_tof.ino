#include "Adafruit_VL53L0X.h"
#include <Wire.h>

#define MULTIPLEXER_ADDRESS 0x70  // I2C address of the multiplexer 
#define SENSOR_ADDRESS 0x29       // I2C address of the VL53L0X sensor

// Define functions
void selectChannel(uint8_t channel);
void startContinuousMeasurement();
void readContinuousMeasurement();
void printArray(float array[], int size);

#define number_of_tof 4
int current_tof_sensor = 1;
int current_channel_for_initial = 0;
int active_tof = 0; // Array start with index = 0

float ReadOutsTof[number_of_tof] = {400, 400, 400, 400}; // Store all the ToF readouts

Adafruit_VL53L0X lox; // Just a name for the VL53L0X sensor

void setup() {
  Serial.begin(9600);
  // Initialize Wire (connection between microcontroller and i2c multiplexer)
  Wire.begin();
  
  // Test if the multiplexer is recognized
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Serial.println("Multiplexer detected.");
  } else {
    Serial.println("Multiplexer not detected. Check connections and address.");
    while (1);  // Stop further execution if multiplexer is not detected
  }
  
  // Initialize all sensors
  for(current_tof_sensor = 0; current_tof_sensor < number_of_tof; current_tof_sensor++) {
    selectChannel(current_channel_for_initial);
    if(!lox.begin(SENSOR_ADDRESS)){
        Serial.print("Failed to boot VL53L0X sensor ");
        Serial.println(current_tof_sensor + 1);
        while(1);
    }
    lox.startContinuous();  // Start continuous measurements
    current_channel_for_initial++;
    Serial.print(current_tof_sensor + 1);
    Serial.println(". VL53L0X sensor up and running");
  } 
  Serial.println("All VL53L0X sensors initialized and running...");
}

void loop() {
  for(int i = 0; i < number_of_tof; i++){
    selectChannel(i);
    readContinuousMeasurement();
    ReadOutsTof[i] = ReadOutsTof[i];  // ReadOutsTof is updated with the latest measurement
  }
  printArray(ReadOutsTof, number_of_tof);
}

// Function to select a channel on the multiplexer
void selectChannel(uint8_t channel) {
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  Wire.write(1 << channel);
  active_tof = channel;
  Wire.endTransmission();
}


void readContinuousMeasurement() {
  VL53L0X_RangingMeasurementData_t measure;
  
  // Get the current measurement
  lox.getRangingMeasurementData(&measure);
  
  // Check if measurement is valid
  if (measure.RangeStatus != 4) {
    ReadOutsTof[active_tof] = measure.RangeMilliMeter / 10.0; // Convert to cm
  } else {
    ReadOutsTof[active_tof] = 400; // Out of range
  }
  
  // Optional delay to control the measurement rate
  delay(15);  // Adjust the delay to achieve the desired rate
}

void printArray(float array[], int size) {
    Serial.println("Array contents:");
    for (int i = 0; i < size; i++) {
        Serial.print("Element ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(array[i]);
    }
}
