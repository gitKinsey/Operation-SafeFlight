#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include <NewPing.h>
#include <math.h>
#include <MPU6050.h>
#include <I2Cdev.h>

// Multiplexer setup
#define MULTIPLEXER_ADDRESS 0x70 // I2C address of the multiplexer
#define SENSOR_ADDRESS 0x29      // I2C address of the VL53L0X sensor

// Variables for the sensor data
volatile float TofFront = 400;
volatile float TofBack = 400;
volatile float TofTop = 400;
volatile float TofBottom = 400;

volatile float ultrasonicDistanceFrontLeft = 0;
volatile float ultrasonicDistanceFrontRight = 0;
volatile float ultrasonicDistanceBackLeft = 0;
volatile float ultrasonicDistanceBackRight = 0;

// ToF setup 
#define number_of_tof 4
int current_channel_for_initial = 0;
int active_tof = 0; // array start with index = 0
float MeasurementTof = 0;
float ReadOutsTof[number_of_tof]; // Store all the ToF readouts
Adafruit_VL53L0X sensors[number_of_tof]; // Array for continuous measurement

// HC-SR04 setup variables
#define TRIGGER_PIN_FRONT_LEFT 10
#define ECHO_PIN_FRONT_LEFT 11
#define TRIGGER_PIN_FRONT_RIGHT 12
#define ECHO_PIN_FRONT_RIGHT 13
#define TRIGGER_PIN_BACK_LEFT 14
#define ECHO_PIN_BACK_LEFT 15
#define TRIGGER_PIN_BACK_RIGHT 18
#define ECHO_PIN_BACK_RIGHT 19
#define MAX_DISTANCE 200 // Maximum distance to ping

// Sensor instances
NewPing frontLeftSensor(TRIGGER_PIN_FRONT_LEFT, ECHO_PIN_FRONT_LEFT, MAX_DISTANCE);
NewPing frontRightSensor(TRIGGER_PIN_FRONT_RIGHT, ECHO_PIN_FRONT_RIGHT, MAX_DISTANCE);
NewPing backLeftSensor(TRIGGER_PIN_BACK_LEFT, ECHO_PIN_BACK_LEFT, MAX_DISTANCE);
NewPing backRightSensor(TRIGGER_PIN_BACK_RIGHT, ECHO_PIN_BACK_RIGHT, MAX_DISTANCE);

// Task handles
TaskHandle_t SensorTaskHandle = NULL;
TaskHandle_t UartTaskHandle = NULL;


//warn beeper setup variables
#define beeper_pin 8


// Function declarations
void readSensors(void *parameter);
void sendData(void *parameter);
void doMeasurementTOF(int sensorIndex);
void selectChannel(uint8_t channel);
void doMeasurementUltrasonic(NewPing &sensor, const char *sensorName);

//beeper specific functions
void playTone(int frequency, int duration);
void alertTone();
void errorTone();
void successTone();

// Function to read sensor values to run on core 0
void readSensors(void *parameter) {
    Wire.begin(); // Start the I2C connection between the microcontroller and the multiplexer 
    // Test if the multiplexer is recognized
    Wire.beginTransmission(MULTIPLEXER_ADDRESS);
    if (Wire.endTransmission() == 0) {
        Serial.println("Multiplexer detected.");
    } else {
        Serial.println("Multiplexer not detected. Check connections and address.");
        while (1);  // Stop further execution if multiplexer is not detected
    }

    // Initialize each ToF sensor
    for(int i = 0; i < number_of_tof; i++) {
        selectChannel(i);
        if(!sensors[i].begin(SENSOR_ADDRESS)){
            Serial.print("Failed to boot VL53L0X sensor ");
            Serial.println(i);
            errorTone(); // Play error tone if sensor fails to initialize
            while(1); // Stop execution if the sensor fails to initialize
        }
        Serial.print(i + 1);
        Serial.println(". VL53L0X sensor up and running");
        successTone(); // Play success tone after successful initialization
    } 
    Serial.println("All VL53L0X sensors initialized and running...");

    // Continuous reading loop
    while (true) {
        for(int i = 0; i < number_of_tof; i++) {
            selectChannel(i);
            doMeasurementTOF(i); // Measure ToF using the index
            ReadOutsTof[i] = MeasurementTof; // Store the measurement
        }

        // Update the volatile variables
        TofFront = ReadOutsTof[0];
        TofBack = ReadOutsTof[1];
        TofTop = ReadOutsTof[2];
        TofBottom = ReadOutsTof[3];

        // Read out all the HC-SR04 sensors
        doMeasurementUltrasonic(frontLeftSensor, "Front Left");
        doMeasurementUltrasonic(frontRightSensor, "Front Right");
        doMeasurementUltrasonic(backLeftSensor, "Back Left");
        doMeasurementUltrasonic(backRightSensor, "Back Right");

        // Delay to control the reading interval
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// Function to send data over UART to run on core 1
void sendData(void *parameter) {
    Serial2.begin(115200, SERIAL_8N1, 16, 17);  // TX on GPIO 17, RX on GPIO 16

    // Continuous sending loop
    while (true) {
        // Send sensor values over UART
        Serial2.write((uint8_t*)&TofFront, sizeof(TofFront));
        Serial2.write((uint8_t*)&TofBack, sizeof(TofBack));
        Serial2.write((uint8_t*)&TofTop, sizeof(TofTop));
        Serial2.write((uint8_t*)&TofBottom, sizeof(TofBottom));
        Serial2.write((uint8_t*)&ultrasonicDistanceBackLeft, sizeof(ultrasonicDistanceBackLeft));
        Serial2.write((uint8_t*)&ultrasonicDistanceBackRight, sizeof(ultrasonicDistanceBackRight));
        Serial2.write((uint8_t*)&ultrasonicDistanceFrontLeft, sizeof(ultrasonicDistanceFrontLeft));
        Serial2.write((uint8_t*)&ultrasonicDistanceFrontRight, sizeof(ultrasonicDistanceFrontRight));

        // Delay to control the interval between UART transmissions.
        vTaskDelay(35 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(9600);  // Start USB serial communication for debugging

    // Setup multicore on ESP32
    xTaskCreatePinnedToCore(
        readSensors,
        "Sensor Task",
        2048,
        NULL,
        1,
        &SensorTaskHandle,
        0  // Run on Core 0
    );

    xTaskCreatePinnedToCore(
        sendData,
        "UART Task",
        2048,
        NULL,
        1,
        &UartTaskHandle,
        1  // Run on Core 1
    );
}

void loop() {
    // Do nothing here; tasks are running in the background
}

// Function to select a channel on the multiplexer
void selectChannel(uint8_t channel) {
    Wire.beginTransmission(MULTIPLEXER_ADDRESS);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

// Function to perform ToF measurement
void doMeasurementTOF(int sensorIndex) {
    VL53L0X_RangingMeasurementData_t measure;

    // Start measurement for the selected sensor
    sensors[sensorIndex].rangingTest(&measure, false);

    // Check if measurement is valid
    if (measure.RangeStatus != 4) {
        MeasurementTof = measure.RangeMilliMeter;
    } else {
        MeasurementTof = 400; // Out of range default value
    }
}

// Function to perform ultrasonic measurement
void doMeasurementUltrasonic(NewPing &sensor, const char *sensorName) {
    unsigned int distance = sensor.ping_cm();

    // Save the distance into the appropriate variable
    if (strcmp(sensorName, "Front Left") == 0) {
        ultrasonicDistanceFrontLeft = distance;
    } else if (strcmp(sensorName, "Front Right") == 0) {
        ultrasonicDistanceFrontRight = distance;
    } else if (strcmp(sensorName, "Back Left") == 0) {
        ultrasonicDistanceBackLeft = distance;
    } else if (strcmp(sensorName, "Back Right") == 0) {
        ultrasonicDistanceBackRight = distance;
    }
    
    if (distance == 0) {
        Serial.print(sensorName);
        Serial.println(": Out of range");
    } else {
        Serial.print(sensorName);
        Serial.print(": ");
        Serial.print(distance);
        Serial.println(" cm");
    }
}


//beeper functions 

void playTone(int frequency, int duration){
  tone(beeper_pin, frequency, duration);
  delay(duration * 1.30); // to account for the extra time it takes to play the tone
  noTone(beeper_pin);
}

void alertTone(){
  playTone(3000, 200); 
  delay(100);
  playTone(3000, 200);
}

void errorTone(){
  playTone(500, 200);
  delay(100);
  playTone(500, 200);
  delay(100);
  playTone(500, 200);
}

void successTone() { //different frequenzies to make out the succestone better
  playTone(1000, 100);
  delay(50);
  playTone(1500, 100);
  delay(50);
  playTone(2000, 200);
}

//sources (other sources are mentioned in the code)
//esp32 multithread: https://randomnerdtutorials.com/esp32-dual-core-arduino-ide/ 
//uart: https://www.circuitbasics.com/how-to-set-up-uart-communication-for-arduino/
//      https://www.luisllamas.es/en/esp32-uart/
//      https://forum.arduino.cc/t/communication-between-two-arduino-uno-via-tx-and-rx/1150462/5
//      https://www.pjrc.com/teensy/td_uart.html
//      https://mischianti.org/esp32-s3-devkitc-1-high-resolution-pinout-and-specs/

// Licence
//  MIT License
//  Copyright (c) 2024 Cedi

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
