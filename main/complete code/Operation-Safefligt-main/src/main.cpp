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

// version with newPing (thanks to : mjs513 for adapting the newPing library for the teensy 4.1)
// link to library: https://github.com/mjs513/NewPing_t4/tree/master
// needs to be tested first though, should be working tested in a seperate file with 4 sensors

// for more sensors, just switch channels easy, like in the setup, add all the sensor with selectChannel(channel of the sensor)
//  and copy paste the initialising sequence
// then in the loop, just switch the channels and you're good to go :)
// future Cedi, for project connect the multiplexer to sda 4 and scl 5 on the teensy 4.1 (so the analoge pins a4 and a5 )

// IMPORTANT NODES:
//   - Channel 6 is for changig flight stuff
//       - like, it has 3 stages, its the most left hand side switch on the rc sender (its the idle switch)
//             - so most down stage (pppm = 2000) should be let everything through
//                 - for future cedi, just add a if statement after all the channels get read out, if the condition is fufilled (if stage is middle one or top one) go into another if clause to decide which one it is
//                   else: just after the if (which contains all the rest of the code, like with reading out the sensors and stuuff) send the ppm signal as is
//             - middle stage should be a hovermode, so that only the top and bottom sensors get used
//             - top stage is the all sensor colidions avoidn system
//   - connect hc-sro4 to 5v BUT INBETWEEN TRIG AND ECHO PUT A 1K OHM RESISTOR !!!!!!!!!!!!

// evtl for the avoidn part some links:
//   https://www.programiz.com/cpp-programming/multidimensional-arrays

// Cedi wennd nömme witerchonsch met c++ lueg mol uf die website do: https://www.w3schools.com/cpp/cpp_arrays.asp

#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include <NewPing.h>
#include <math.h>
#include <MPU6050.h>
#include <I2Cdev.h>

// multiplexer setup variables
#define MULTIPLEXER_ADDRESS 0x70 // I2C address of the multiplexer
#define SENSOR_ADDRESS 0x29      // I2C address of the VL53L0X sensor

// PPM setup variables
//  Define the pin connected to the PPM signal (Input pin)
#define PPM_PIN 2
// PPM settings (for sending ppm signal again)
#define PPM_PIN_OUT 6  // Output pin for PPM signal
#define NUM_CHANNELS 8 // Number of PPM channels
#define TOTAL_PULSES (NUM_CHANNELS + 1)
#define FRAME_DURATION 20000 // Total PPM frame length in microseconds
#define MIN_PULSE_WIDTH 1000 // Minimum channel pulse length in microseconds
#define MAX_PULSE_WIDTH 2000 // Maximum channel pulse length in microseconds
uint32_t frameStartTime;

// Separate variables for each channel (of the array, for the ppm signals)
volatile uint16_t channel1; // unit16_t is an interger that is able to hold 2^16 numbers, so max ig smt with 65k
volatile uint16_t channel2; // volatile is a variable that can change without action beeing taken aka, not getten activly changed in code (par example: an interput that occurs changes it (which is the case here))
volatile uint16_t channel3;
volatile uint16_t channel4;
volatile uint16_t channel5;
volatile uint16_t channel6;
volatile uint16_t channel7;
volatile uint16_t channel8; // same as unit16_t just 2^8 this time haha (works also with unit32_t)
volatile uint8_t currentChannel = 0;
uint16_t pulseWidths[NUM_CHANNELS];
const uint16_t MIN_THROTTLE = 1000; // Minimum throttle value for safety
const uint16_t MAX_THROTTLE = 2000; // Maximum throttle value for safety

// Tof setup variables
#define number_of_tof 4
int current_tof_sensor = 1;
int current_channel_for_initial = 0;
int active_tof = 0; // array start with index = 0
char receivedChar;
float MeasurementTof = 0;
float TofFront = 400;                                                      // array index 0 (change via active_tof)
float TofBack = 400;                                                                                                                                                                                                                                                                        // array index 1 (change via active_tof)
float TofTop = 400;                                                                      // array index 2 (change via active_tof)
float TofBottom = 400;                                                     // array index 3 (change via active_tof)
float ReadOutsTof[number_of_tof] = {TofFront, TofBack, TofTop, TofBottom}; // Store all the Tof readouts, from https://www.w3schools.com/cpp/cpp_arrays.asp

// for symplicity of coding i'll create to seperate variables for top and bottom (in reagard of chapter 2, aka only hight controlling system of the drone)
float TofTopValue = ReadOutsTof[3];
float TofBottomValue = ReadOutsTof[2];

Adafruit_VL53L0X lox; // just a name for the VL53lox sensor
Adafruit_VL53L0X sensors[number_of_tof]; //is needed for continuous measurement

// hc-sro4 Setup setup variables
#define TRIGGER_PIN_FRONT_LEFT 10
#define ECHO_PIN_FRONT_LEFT 11
#define TRIGGER_PIN_FRONT_RIGHT 12
#define ECHO_PIN_FRONT_RIGHT 13
#define TRIGGER_PIN_BACK_LEFT 14
#define ECHO_PIN_BACK_LEFT 15
#define TRIGGER_PIN_BACK_RIGHT 16 // maybe this has to be changed, bc in testing it only worked with trig on 9 and echo on 8 (needs further testing), works with those pins just a connection problem with the Breadboard
#define ECHO_PIN_BACK_RIGHT 17
#define MAX_DISTANCE 200 // Maximum distance i wanna ping, maybe needs to get changed

// giving each Sensor NewPing class, from: https://stackoverflow.com/questions/35186703/arduino-hc-sr04-newping-code-not-working
NewPing frontLeftSensor(TRIGGER_PIN_FRONT_LEFT, ECHO_PIN_FRONT_LEFT, MAX_DISTANCE);
NewPing frontRightSensor(TRIGGER_PIN_FRONT_RIGHT, ECHO_PIN_FRONT_RIGHT, MAX_DISTANCE);
NewPing backLeftSensor(TRIGGER_PIN_BACK_LEFT, ECHO_PIN_BACK_LEFT, MAX_DISTANCE);
NewPing backRightSensor(TRIGGER_PIN_BACK_RIGHT, ECHO_PIN_BACK_RIGHT, MAX_DISTANCE);

// hc-sro4 values
float ultrasonicDistanceFrontLeft = 0;
float ultrasonicDistanceFrontRight = 0;
float ultrasonicDistanceBackLeft = 0;
float ultrasonicDistanceBackRight = 0;

// MPU6050 setup variables
MPU6050 mpu;                       // setting up a mpu6050
int16_t accel_x, accel_y, accel_z; // variables for the acceleration of the drone
int16_t gyro_x, gyro_y, gyro_z;    // varibales for the oriantaion of the drone
int mpu_channel = 4;               // on the mulitplexer is the mpu channel 4

// variables for avoiding part
int distanceThreshold_xy = 100; // distance threshold for the xy plane
int distanceThreshold_z = 150;  // distance threshold for the z plane

// all the variables needed fot the threshold Detection
const int thresholdArraySize = 4;
int tofBrokenThreshold[thresholdArraySize]; // array to hold if the 4 sensors are under the threshold

//warn beeper setup variables
#define beeper_pin 8
// defining all the funcitons
void readPPM();
void selectChannel(uint8_t channel);
void doMeasurementTOF();
void printArray(float array[], int size);
void doMeasurementUltrasonic(NewPing &sensor, const char *sensorName);
void sendPPM();
void ppmSetup();
void tofSetup();
void mpuSetup();
void doMeasurementMPU6050();
void landDrone();
void thresholdDetectionToF();
//beeper specific functions
void playTone(int frequency, int duration);
void alertTone();
void errrorTone();
void successTone();

void setup()
{
  Serial.begin(9600); // Start serial communication for debugging
  Serial.println("Start of program");

  // setting up all the neccesarry parts of the code:
  // ppm input setup
  ppmSetup();

  // tof sensor initialization
  tofSetup();

  // mpu setup
  mpuSetup();

  //beeper setup
  pinMode(beeper_pin, OUTPUT);
}

void loop()
{
  // making sure that the data from the ppm input doesn't change during read out (really important) => everything crucial that shoudnt be disrupted by interupts goes here aka, reading sensor values, performing the calculations, sending the ppm signal out again
  noInterrupts(); // makes that the values don't change whilst getting read out => won't change till next cycle of loop()
  // deactivated the interputs shortly said
  // check which mode the drone should be flying in: (the idle switch on the drone controlls this, lowest (switch is pointing downwards) stage = channel 1, middel stage = channel 2, highst stage = channel 3)
  // the idle switch controlles channel 6 on the ppm stream
  // if mode 1 is selected, then we can skipp all the sensor read outs
  // for mode 2 read outs of the hc-sro4 can be ignored and only the top and bottom tof sensors will be used
  // for mode 3, all the sensors need to be read out, which is the slowest of them all (unfortunately)
  if (channel6 >= 1700)
  {
    // this is mode 1 => no sensor read outs
    // coping the values of the channels into the array (for sending it out again, is more elegant this way plus abit easier)
    pulseWidths[0] = channel1;
    pulseWidths[1] = channel2;
    pulseWidths[2] = channel3;
    pulseWidths[3] = channel4;
    pulseWidths[4] = channel5;
    pulseWidths[5] = channel6;
    pulseWidths[6] = channel7;
    pulseWidths[7] = channel8;
    // just directly to sending again, everything else will be ignored
    interrupts(); // activate the interupts again
    // sending out the ppm signals
    sendPPM();
  }
  else if (channel6 < 1700 && channel6 < 1300) // https://www.w3schools.com/cpp/cpp_conditions_elseif.asp
  {
    // this is mode 2 => only hight controlling sensor are read out

    // read out the tof sensors, in the array is bottom = 3 and top = 2
    selectChannel(2); // for top sensor
    doMeasurementTOF();
    TofTopValue = MeasurementTof;

    selectChannel(3); // for bottom sensor
    doMeasurementTOF();
    TofBottomValue = MeasurementTof;

    // check if one of the sensors value is to close to the wall or an other obstacle:
    if (TofTopValue <= distanceThreshold_z)
    {
      // the throttle needs to be adjusted to be able to steer the hight of the drone
      if (channel4 <= MIN_THROTTLE)
      {
        channel4 = MIN_THROTTLE;
      }
      else
      {
        channel4 -= 50; // throttle down
      }
    }
    else if (TofBottomValue <= distanceThreshold_z)

    {
      // the throttle needs to be adjusted to be able to steer the hight of the drone
      if (channel4 <= MAX_THROTTLE)
      {
        channel4 = MAX_THROTTLE;
      }
      else
      {
        channel4 += 50; // throttle down
      }
    }
    // coping the values of the channels into the array (for sending it out again, is more elegant this way plus abit easier)
    pulseWidths[0] = channel1;
    pulseWidths[1] = channel2;
    pulseWidths[2] = channel3;
    pulseWidths[3] = channel4;
    pulseWidths[4] = channel5;
    pulseWidths[5] = channel6;
    pulseWidths[6] = channel7;
    pulseWidths[7] = channel8;
    // the last part of this chapter (i call it chapter, meant is ofc the else if statement, or rather just part of the code)
    interrupts(); // activate the interupts again
    // sending out the ppm signals
    sendPPM();
    // from here on can go every part of the code that is not crucialy depending on the ppm signal or the sensors themselfs, par example turning on a led or smt
  }
  else
  {
    // this is mode 3 => all sensors are read out
    //  reading out all the Tof Sensors
    for (int i = 1; i < 4; i++)
    {
      selectChannel(active_tof);
      doMeasurementTOF();
      ReadOutsTof[active_tof] = MeasurementTof;
      active_tof++;
    }
    active_tof = 0;
    printArray(ReadOutsTof, number_of_tof);

    // read out all the HC-sro4 sensors
    doMeasurementUltrasonic(frontLeftSensor, "Front Left");
    doMeasurementUltrasonic(frontRightSensor, "Front Right");
    doMeasurementUltrasonic(backLeftSensor, "Back Left");
    doMeasurementUltrasonic(backRightSensor, "Back Right");

    // for future Cedi: Put all the maths here (for full on collision avoidance):
    // check if any sensors are under the threshold and then adjusting the logic, ig just have 6 direction variables which get plus 1 if the threshold is undercut (at the end all of those ned to be set to zero again (after the signal got send out))
    int forward = 0;
    int backward = 0;
    int left = 0;
    int right = 0;
    int up = 0;
    int down = 0;
    int adjustValue = 25;

    thresholdDetectionToF();

    // add 1 to the direction variables if the threshold is undercut from one of the sensors
    if (tofBrokenThreshold[0] != 0)
    {
      forward++;
    }
    else if (tofBrokenThreshold[1] != 0)
    {
      backward++;
    }
    else if (tofBrokenThreshold[2] != 0)
    {
      up++;
    }
    else if (tofBrokenThreshold[3] != 0)
    {
      down++;
    }

    if (ultrasonicDistanceFrontLeft < distanceThreshold_xy)
    {
      backward++;
      right++;
    }
    else if (ultrasonicDistanceFrontRight < distanceThreshold_xy)
    {
      backward++;
      left++;
    }
    else if (ultrasonicDistanceBackLeft < distanceThreshold_xy)
    {
      forward++;
      right++;
    }
    else if (ultrasonicDistanceBackRight < distanceThreshold_xy)
    {
      forward++;
      left++;
    }
    // manipulating the signal for the ppm
    channel1 = channel1 - (adjustValue * right) + (adjustValue * left);
    channel2 = channel2 - (adjustValue * forward) + (adjustValue * backward);
    channel4 = channel4 - (adjustValue * down) + (adjustValue * up);

    // coping the values of the channels into the array (for sending it out again, is more elegant this way plus abit easier)
    pulseWidths[0] = channel1;
    pulseWidths[1] = channel2;
    pulseWidths[2] = channel3;
    pulseWidths[3] = channel4;
    pulseWidths[4] = channel5;
    pulseWidths[5] = channel6;
    pulseWidths[6] = channel7;
    pulseWidths[7] = channel8;
    // this has to be the last part of this chapter
    interrupts(); // activate the interupts again
    // sending out the ppm signals
    sendPPM();
    // from here on can go every part of the code that is not crucialy depending on the ppm signal or the sensors themselfs, par example turning on a led or smt
  }

  // theoretically here can go the code, which is not connected to the avoiding system or ppm system, for example if i wanna play music or smt with the drone (probably there won't be anything here)
}

// defining all the functions here (needs to be done like this in PlatformIO):

// ppm functions (doesn't need to be called, gets called asoon as an interupt happends)
void readPPM()
{
  static uint32_t lastTime = 0;
  uint32_t currentTime = micros(); // Current time in microseconds
  uint32_t interval = currentTime - lastTime;
  lastTime = currentTime;

  if (interval >= 3000)
  {
    // Sync pulse detected (interval longer than a frame period, e.g., 3 ms)
    currentChannel = 0;
  }
  else
  {
    switch (currentChannel)
    {
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

// sending function for PPM signals
void sendPPM()
{
  // Generate PPM signal
  uint32_t pulseStartTime = micros();

  // Send the pulse widths for each channel
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    digitalWrite(PPM_PIN_OUT, LOW);
    digitalWrite(PPM_PIN_OUT, HIGH);
    delayMicroseconds(pulseWidths[i]);
  }

  // Calculate the time taken and adjust to match FRAME_DURATION
  uint32_t elapsedTime = micros() - pulseStartTime;
  uint32_t remainingTime = FRAME_DURATION - elapsedTime;

  if (remainingTime > 0)
  {
    digitalWrite(PPM_PIN_OUT, LOW);
    delayMicroseconds(remainingTime);
  }

  // Update frame start time
  frameStartTime = micros();
}

// multiplexer
void selectChannel(uint8_t channel)
{ // Function to select a channel on the multiplexer
  Wire.begin();
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  Wire.write(1 << channel);
  active_tof = channel;
  Wire.endTransmission();
}

// tof stuff
void doMeasurementTOF()
{ // Function to measure the distance with Tof

  //single measurement:
  // VL53L0X_RangingMeasurementData_t measure;

  // Start measurement
  //lox.rangingTest(&measure, false);
  // Check if measurement is valid
//   if (measure.RangeStatus != 4)
//   {
//     MeasurementTof = measure.RangeMilliMeter;
//   }
//   else
//   {
//     MeasurementTof = 4000; // just always back to 400 to not interfear with anything ig
//   }

//continous measurement:
  MeasurementTof = sensors[active_tof].readRange();

}

// print the tof sensor values
void printArray(float array[], int size)
{ // if needed for debuging, a function to print out the array
  Serial.println("Array contents:");
  for (int i = 0; i < size; i++)
  {
    Serial.print("Element ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(array[i]);
  }
}

// Hc-sro4 functions
void doMeasurementUltrasonic(NewPing &sensor, const char *sensorName)
{
  unsigned int distance = sensor.ping_cm();

  // save the distance into the according variable, reference: https://cplusplus.com/reference/cstring/strcmp/ (the comparing of strings)
  if (strcmp(sensorName, "Front Left") == 0)
  {
    ultrasonicDistanceFrontLeft = distance;
  }
  else if (strcmp(sensorName, "Front Right") == 0)
  {
    ultrasonicDistanceFrontRight = distance;
  }
  else if (strcmp(sensorName, "Back Left") == 0)
  {
    ultrasonicDistanceBackLeft = distance;
  }
  else if (strcmp(sensorName, "Back Right") == 0)
  {
    ultrasonicDistanceBackRight = distance;
  }
  if (distance == 0)
  {
    Serial.print(sensorName);
    Serial.println(": Out of range");
  }
  else
  {
    Serial.print(sensorName);
    Serial.print(": ");
    Serial.print(distance);
    Serial.println(" cm");
  }
}

// MPU6050 stuff
void doMeasurementMPU6050() // from https://github.com/ElectronicCats/mpu6050 (i looked at the docs and example files and grabed the stuff i need)
{
  // Select channel 5 on the multiplexer
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  Wire.write(1 << mpu_channel); // we have 4 TOF on channel 0,1,2,3 and now the mpu6050 on channel 4
  Wire.endTransmission();

  // Initialize the MPU6050 sensor
  Wire.begin();
  mpu.initialize();

  // Read the accelerometer and gyroscope data
  mpu.getMotion6(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);

  // Calculate the angle using the accelerometer data
  float angle_x = atan2(accel_y, accel_z) * 180 / M_PI;
  float angle_y = atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180 / M_PI;

  // Print the accelerometer and gyroscope values
  Serial.print("MPU6050 sensor values: ");
  Serial.print("X: ");
  Serial.print(accel_x);
  Serial.print(", Y: ");
  Serial.print(accel_y);
  Serial.print(", Z: ");
  Serial.print(accel_z);
  Serial.print(" | Angle X: ");
  Serial.print(angle_x);
  Serial.print(", Angle Y: ");
  Serial.println(angle_y);
}

// avoing part of the code:
// first off all the landing sequenze:
void landDrone()
{

  float landingThreshold = 15.0; // threshold for landing, when reached it should just dropp down

  // for a smooth landing the other channels will be ignored ig and => put to the default 1500, only necessair for the pich yaw and roll
  channel1 = 1500;
  channel2 = 1500;
  channel3 = 1500;

  // the throttle value gets slowly decreased to ensure a smooth landing ig
  if (TofBottomValue > landingThreshold)
  {
    if (channel4 > MIN_THROTTLE)
    { // as long as the trhottle is still greater than the minimum it should be degreased
      channel4 -= 15;

      if (channel4 < MIN_THROTTLE)
      { // the trhottle value can't go beneath 1000, would end in chaos
        channel4 = MIN_THROTTLE;
      }
    }
    else
    {
      channel4 = MIN_THROTTLE; // throttle shoudn't go below minimum, gets ensured here
    }
  }
  else
  { // if distance to ground is safe set throtle to minimum, aka when distance is <= landing Altitude
    channel4 = MIN_THROTTLE;
  }

  // sending out the ppm signals
  sendPPM();
}

void thresholdDetectionToF()
{
  // tof sensors
  for (int i = 0; i <= 3; i++)
  { // check if the tof sensor is under the threshold, i = 0 => front sensor, i = 1 => back sensor
    if (ReadOutsTof[i] < distanceThreshold_xy)
    {
      tofBrokenThreshold[i] = i;
    }
    else
    {
      tofBrokenThreshold[i] = 0; // so for later, in the avoiding function or smt i need to check if the array[i] != 0 if yes the sensor on the array with i is to close and needs to be adapted
    }
  }
}

// Setup functions:
// setup for the ppm Input:
void ppmSetup()
{
  // ppm setup
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, FALLING); // Set up an interrupt on the falling edge
  pinMode(PPM_PIN_OUT, OUTPUT);
  digitalWrite(PPM_PIN_OUT, LOW);

  // Calculate the start time of the frame
  frameStartTime = micros();
}

// setup for the VL53lox Sensors (aka Tof):
void tofSetup()
{
  // tof setup
  //  Initialize Wire (connection between microcontroller and i2c multiplexer), first sda than scl (doesn't work on teensy, need to plug it into a4 and a5 on teensy)
  Wire.begin(); // start the i2c connection between the microcontroller and the multiplexer
  // Test if the multiplexer is recognized
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  if (Wire.endTransmission() == 0)
  {
    Serial.println("Multiplexer detected.");
    successTone();
  }
  else
  {
    Serial.println("Multiplexer not detected. Check connections and address.");
    errorTone();
    while (1)
      ; // Stop further execution if multiplexer is not detected
  }
  // if not working just delete the for loop :), nevermind works (or should be working when last tested on esp32 (need to be tested on teensy 4.1), works aswell)
  for (int current_tof_sensor = 0; current_tof_sensor < number_of_tof; current_tof_sensor++) {
    // Select the current channel on the multiplexer
    selectChannel(current_channel_for_initial);
    
    // Initialize the sensor at the selected channel
    if (!sensors[current_tof_sensor].begin(SENSOR_ADDRESS)) {
        Serial.print("Failed to boot VL53L0X sensor ");
        Serial.println(current_tof_sensor + 1); // Adjusted to be 1-based for better readability
        for (int i = 0; i < current_tof_sensor +1; i++)
        alertTone();
        while (1); // Halt execution
    }
    
    // Set up the sensor for continuous measurement
    sensors[current_tof_sensor].setMeasurementTimingBudgetMicroSeconds(15000); // Set timing budget to 15ms
    sensors[current_tof_sensor].startRangeContinuous(); // Start continuous measurements
    
    // Move to the next channel for the next sensor
    current_channel_for_initial++;
    
    // Print the sensor status
    Serial.print(current_tof_sensor + 1); // Adjusted to be 1-based for better readability
    Serial.println(". VL53L0X sensor up and running");
  
}

// Print completion message
Serial.println("All VL53L0X sensors initialized and running...");
successTone();
}

// mpu6050 setup
void mpuSetup()
{
  selectChannel(mpu_channel);
  mpu.initialize();

  // Verify connection
  if (mpu.testConnection())
  {
    Serial.println("MPU6050 connection successful");
  }
  else
  {
    Serial.println("MPU6050 connection failed");
    errorTone();
    errorTone();
    while (1)
      ; // Stop further execution if MPU6050 is not detected
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
<<<<<<< Tabnine <<<<<<<
/**//+
 * @brief Initializes a NewPing object for the front left sensor.//+
 *//+
 * This function sets up a NewPing object to measure distances using the HC-SR04 sensor.//+
 * The sensor is connected to the specified trigger and echo pins, and the maximum//+
 * distance it can measure is set to 400 cm.//+
 *//+
 * @param TRIGGER_PIN_FRONT_LEFT The pin number connected to the trigger pin of the HC-SR04 sensor.//+
 * @param ECHO_PIN_FRONT_LEFT The pin number connected to the echo pin of the HC-SR04 sensor.//+
 * @param MAX_DISTANCE The maximum distance the sensor can measure in centimeters.//+
 *//+
 * @return A NewPing object initialized for the front left sensor.//+
 *///+
NewPing frontLeftSensor(TRIGGER_PIN_FRONT_LEFT, ECHO_PIN_FRONT_LEFT, MAX_DISTANCE);//+
>>>>>>> Tabnine >>>>>>>// {"conversationId":"2e99d033-6148-47ca-a751-bf3c214b9d32","source":"instruct"}