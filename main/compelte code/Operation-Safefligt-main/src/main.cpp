
//version with newPing (thanks to : mjs513 for adapting the newPing library for the teensy 4.1)
//link to library: https://github.com/mjs513/NewPing_t4/tree/master
//needs to be tested first though



//for more sensors, just switch channels easy, like in the setup, add all the sensor with selectChannel(channel of the sensor)
// and copy paste the initialising sequence
//then in the loop, just switch the channels and you're good to go :)
//future Cedi, for project connect the multiplexer to sda 4 and scl 5 on the teensy 4.1 (so the analoge pins a4 and a5 )

//IMPORTANT NODES:
//  - Channel 6 is for changig flight stuff 
//      - like, it has 3 stages, its the most left hand side switch on the rc sender (its the idle switch)
//            - so most down stage (pppm = 2000) should be let everything through
//                - for future cedi, just add a if statement after all the channels get read out, if the condition is fufilled (if stage is middle one or top one) go into another if clause to decide which one it is
//                  else: just after the if (which contains all the rest of the code, like with reading out the sensors and stuuff) send the ppm signal as is
//            - middle stage should be a hovermode, so that only the top and bottom sensors get used
//            - top stage is the all sensor colidions avoidn system 
//  - connect hc-sro4 to 5v BUT INBETWEEN TRIG AND ECHO PUT A 1K OHM RESISTOR !!!!!!!!!!!!

#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include <NewPing.h>

//multiplexer stuff
#define MULTIPLEXER_ADDRESS 0x70  // I2C address of the multiplexer 
#define SENSOR_ADDRESS 0x29       // I2C address of the VL53L0X sensor

//PPM stuff
// Define the pin connected to the PPM signal (Input pin)
#define PPM_PIN 2
// PPM settings (for sending ppm signal again)
#define PPM_OUT_PIN 3  // Output pin for PPM signal
#define NUM_CHANNELS 8  // Number of PPM channels
#define PPM_PERIOD 20000  // Total PPM frame length in microseconds
#define PULSE_LENGTH 300  // Length of sync pulse in microseconds
#define MIN_CHANNEL_PULSE 1000  // Minimum channel pulse length in microseconds
#define MAX_CHANNEL_PULSE 2000  // Maximum channel pulse length in microseconds


// Separate variables for each channel (of the array, for the ppm signals)
volatile uint16_t channel1; //unit16_t is an interger that is able to hold 2^16 numbers, so max ig smt with 65k
volatile uint16_t channel2; //volatile is a variable that can change without action beeing taken aka, not getten activly changed in code (par example: an interput that occurs changes it (which is the case here))
volatile uint16_t channel3;
volatile uint16_t channel4;
volatile uint16_t channel5;
volatile uint16_t channel6;
volatile uint16_t channel7;
volatile uint16_t channel8; //same as unit16_t just 2^8 this time haha (works also with unit32_t)
volatile uint8_t currentChannel = 0;
uint16_t channelValues[NUM_CHANNELS] = {channel1, channel2, channel3, channel4, channel5, channel6, channel7, channel8}; //create a arry that holds unit16_t (bc thats the datatype of the variables) with all the channel values


//Tof variables
#define number_of_tof 4
int current_tof_sensor = 1;
int current_channel_for_initial = 0;
int active_tof = 0; //array start with index = 0
char receivedChar;
float MeasurementTof = 0;
float TofFront = 400; //array index 0 (change via active_tof)
float TofBack = 400; // array index 1 (change via active_tof)
float TofTop = 400; //array index 2 (change via active_tof)
float TofBottom = 400; //array index 3 (change via active_tof)
float ReadOutsTof[number_of_tof] = {TofFront, TofBack, TofTop, TofBottom}; // Store all the Tof readouts
Adafruit_VL53L0X lox; //just a name for the VL53lox sensor


//hc-sro4 Setup stuff
#define TRIGGER_PIN_FRONT_LEFT 10
#define ECHO_PIN_FRONT_LEFT 11
#define TRIGGER_PIN_FRONT_RIGHT 12
#define ECHO_PIN_FRONT_RIGHT 13
#define TRIGGER_PIN_BACK_LEFT 14
#define ECHO_PIN_BACK_LEFT 15
#define TRIGGER_PIN_BACK_RIGHT 16 //maybe this has to be changed, bc in testing it only worked with trig on 9 and echo on 8 (needs further testing), works with those pins just a connection problem with the Breadboard
#define ECHO_PIN_BACK_RIGHT 17
#define MAX_DISTANCE 200 // Maximum distance i wanna ping, maybe needs to get changed 

//giving each Sensor NewPing class, from: https://stackoverflow.com/questions/35186703/arduino-hc-sr04-newping-code-not-working
NewPing frontLeftSensor(TRIGGER_PIN_FRONT_LEFT, ECHO_PIN_FRONT_LEFT, MAX_DISTANCE);
NewPing frontRightSensor(TRIGGER_PIN_FRONT_RIGHT, ECHO_PIN_FRONT_RIGHT, MAX_DISTANCE);
NewPing backLeftSensor(TRIGGER_PIN_BACK_LEFT, ECHO_PIN_BACK_LEFT, MAX_DISTANCE);
NewPing backRightSensor(TRIGGER_PIN_BACK_RIGHT, ECHO_PIN_BACK_RIGHT, MAX_DISTANCE);


//defining all the funcitons
void readPPM();
void selectChannel(uint8_t channel);
void doMeasurementTOF();
void printArray(float array[], int size);
void doMeasurementUltrasonic(NewPing &sensor, const char *sensorName);
void sendPPM();


void setup() {
  Serial.begin(9600);  // Start serial communication for debugging
  Serial.println("Start of program");

  //ppm setup
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, FALLING);  // Set up an interrupt on the falling edge
  pinMode(PPM_OUT_PIN, OUTPUT);
  digitalWrite(PPM_OUT_PIN, HIGH);


  //tof setup
  // Initialize Wire (connection between microcontroller and i2c multiplexer), first sda than scl (doesn't work on teensy, need to plug it into a4 and a5 on teensy)
  Wire.begin(); //start the i2c connection between the microcontroller and the multiplexer 
  // Test if the multiplexer is recognized
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Serial.println("Multiplexer detected.");
  } else {
    Serial.println("Multiplexer not detected. Check connections and address.");
    while (1);  // Stop further execution if multiplexer is not detected
  }
  //if not working just delete the for loop :), nevermind works (or should be working when last tested on esp32 (need to be tested on teensy 4.1), works aswell)
  for(current_tof_sensor = 1; current_tof_sensor <= number_of_tof; current_tof_sensor++) { // for loop for checking every connected sensor 
  //(just checks for the number of sensor enterd above)
    selectChannel(current_channel_for_initial);
    if(!lox.begin(SENSOR_ADDRESS)){
        Serial.print("Failed to boot VL53L0X sensor ");
        Serial.println(current_tof_sensor);
        while(1);
    }
    current_channel_for_initial++;
    Serial.print(current_tof_sensor);
    Serial.println(". Vl53L0X sensor up and running");
  } 
  Serial.println("All VL53L0X sensors initialized and running...");


}

void loop() {
    //making sure that the data from the ppm input doesn't change during read out (really important) => everything crucial that shoudnt be disrupted by interupts goes here aka, reading sensor values, performing the calculations, sending the ppm signal out again
  noInterrupts(); //makes that the values don't change whilst getting read out => won't change till next cycle of loop()
  //deactivated the interputs shortly said 
  //reading out all the Tof Sensors
  for(int i = 1; i <=4; i++){
    selectChannel(active_tof);
    doMeasurementTOF();
    ReadOutsTof[active_tof] = MeasurementTof;
  }
  printArray(ReadOutsTof, number_of_tof);


  //read out all the HC-sro4 sensors
  doMeasurementUltrasonic(frontLeftSensor, "Front Left");
  doMeasurementUltrasonic(frontRightSensor, "Front Right");
  doMeasurementUltrasonic(backLeftSensor, "Back Left");
  doMeasurementUltrasonic(backRightSensor, "Back Right");
  // delay(1000); //no delay here, rn just for debuging purposes 


  //for future Cedi: Put all the maths here:






  //coping the values of the channels into the array (for sending it out again, is more elegant this way plus abit easier)
  channelValues[0] = channel1; 
  channelValues[1] = channel2;
  channelValues[2] = channel3;
  channelValues[3] = channel4;
  channelValues[4] = channel5;
  channelValues[5] = channel6;
  channelValues[6] = channel7;
  channelValues[7] = channel8;

  //sending out the ppm signals (last part of code again)
  sendPPM();
  interrupts(); //activate the interupts again
  //from here on can go every part of the code that is not crucialy depending on the ppm signal or the sensors themselfs, par example turning on a led or smt 




}

//ppm functions (doesn't need to be called, gets called asoon as an interupt happends)
void readPPM() { 
  static uint32_t lastTime = 0;
  uint32_t currentTime = micros();  // Current time in microseconds
  uint32_t interval = currentTime - lastTime;
  lastTime = currentTime;

  if (interval >= 3000) {
    // Sync pulse detected (interval longer than a frame period, e.g., 3 ms)
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

//sending function for PPM signals 

void sendPPM(){
  uint32_t frameStartTime = micros();
  uint32_t lastPulseEndTime = frameStartTime;
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    // Calculate the exact time to wait before the next pulse
    uint32_t pulseStartTime = lastPulseEndTime + (channelValues[i] - PULSE_LENGTH);
    // Wait for the time to send the next pulse
    while (micros() < pulseStartTime) {
    }
    // Send the channel pulse
    digitalWrite(PPM_OUT_PIN, LOW);
    delayMicroseconds(PULSE_LENGTH);
    digitalWrite(PPM_OUT_PIN, HIGH);
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
  digitalWrite(PPM_OUT_PIN, LOW);
  delayMicroseconds(PULSE_LENGTH);
  digitalWrite(PPM_OUT_PIN, HIGH);
}

//tof functions
void selectChannel(uint8_t channel) { // Function to select a channel on the multiplexer
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  Wire.write(1 << channel);
  active_tof = channel;
  Wire.endTransmission();
}

void doMeasurementTOF(){ //Function to measure the distance with Tof
  VL53L0X_RangingMeasurementData_t measure;
  
  // Start measurement
  lox.rangingTest(&measure, false);
  
  // Check if measurement is valid
  if (measure.RangeStatus != 4) {
    MeasurementTof = measure.RangeMilliMeter;
  } else {
    MeasurementTof = 400; //just always back to 400 to not interfear with anything ig
  }
  // delay(20); //idk how much probs i can just leave that one out ig
}

void printArray(float array[], int size) { //if needed for debuging, a function to print out the array
    Serial.println("Array contents:");
    for (int i = 0; i < size; i++) {
        Serial.print("Element ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(array[i]);
    }
}


//Hc-sro4 functions
void doMeasurementUltrasonic(NewPing &sensor, const char *sensorName) {
  unsigned int distance = sensor.ping_cm();
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





//Licence
// MIT License
// Copyright (c) 2024 Cedi

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
