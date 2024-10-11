//import all the necessary libraries
#include <Arduino.h>


//ppm Pins setup
#define PPM_PIN 2                     // PPM input pin 
#define PPM_PIN_OUT 3                 // PPM output pin


// Variables for the ppm channels
volatile uint16_t channel1;
volatile uint16_t channel2;
volatile uint16_t channel3;
volatile uint16_t channel4;
volatile uint16_t channel5;
volatile uint16_t channel6;
volatile uint16_t channel7;
volatile uint16_t channel8;


//ppm signal setup settings
#define NUM_CHANNELS 8                  // number of channels
#define TOTAL_PULSES (NUM_CHANNELS + 1) // number of pulses
#define FRAME_DURATION 20000            // Define the PPM frame duration in microseconds
#define MIN_PULSE_WIDTH 1000            // minimum pulse width
#define MAX_PULSE_WIDTH 2000            // maximum pulse width
uint16_t pulseWidths[NUM_CHANNELS];     // Array to store the pulse widths for each channel
uint32_t frameStartTime;                // Variables to track the start time of the frame
volatile uint8_t currentChannel = 0;    // Variable to keep track of the current channel being read


//variables for the sensor data
float tofFront = 400;
float tofBack = 400;
float tofTop = 400;
float tofBottom = 400;

float ultrasonicDistanceFrontLeft = 400;
float ultrasonicDistanceFrontRight = 400;
float ultrasonicDistanceBackLeft = 400;
float ultrasonicDistanceBackRight = 400;

// variables for avoiding part
int distanceThreshold_xy = 100;         // distance threshold for the xy plane
int distanceThreshold_z = 150;          // distance threshold for the z plane
const uint16_t MIN_THROTTLE = 1000;     // Minimum throttle value for safety
const uint16_t MAX_THROTTLE = 2000;     // Maximum throttle value for safety


//other variables
const int ledPin = 13;                  // setup built in led


//define all the functions here:
void uartSetup();
void ppmSetup();
void readPPM();
void sendPPM();
void sensorReadout();
bool validateChecksum(uint8_t* buffer, int bufferSize);



void setup() {
  //Start a serial connection with the computer for debuging
  Serial.begin(9600);


  //start the other setupfunctions
  uartSetup();
  ppmSetup();
}



void loop() {
  // first the teensy should request all the sensor data from the esp32
  sensorReadout();

  // then check which mode the drone should be flying in: (the idle switch on the drone controlls this, lowest (switch is pointing downwards) stage = channel 1, middel stage = channel 2, highst stage = channel 3)
  // the idle switch controlles channel 6 on the ppm stream
  // if mode 1 is selected, then we can skipp all the sensor read outs
  // for mode 2 read outs of the hc-sro4 can be ignored and only the top and bottom tof sensors will be used
  // for mode 3, all the sensors need to be read out, which is the slowest of them all (unfortunately)

  if (channel6 >= 1700){                 // this is mode 1 => no sensor read outs
  //copying the volatile variables into the array (don't modify the channel variables, just add the neccesary value in the noInterrupt part)
  noInterrupts();                        // making sure that the data from the ppm input doesn't change during read out (really important) => everything crucial that shoudnt be disrupted by interupts goes here aka, reading sensor values, performing the calculations, sending the ppm signal out again
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



  }else if (channel6 >= 1300 && channel6 < 1700){     // this is mode 2 => only hight controlling sensor are read out
  // check if one of the sensors value is to close to the wall or an other obstacle:
    if (tofTop <= distanceThreshold_z)
    {
      channel4 -= 50;                 // throttle down
      
    }
    else if (tofBottom <= distanceThreshold_z)

    {
      channel4 += 50;                 // Throttle up

    }
    channel4 = constrain(channel4, MIN_THROTTLE, MAX_THROTTLE);  //makes sure that the channel value stays inbetween min and max throtle if smt went wrong before


    noInterrupts();                   // making sure that the data from the ppm input doesn't change during read out (really important) => everything crucial that shoudnt be disrupted by interupts goes here aka, reading sensor values, performing the calculations, sending the ppm signal out again
    pulseWidths[0] = channel1;
    pulseWidths[1] = channel2;
    pulseWidths[2] = channel3;
    pulseWidths[3] = channel4;
    pulseWidths[4] = channel5;
    pulseWidths[5] = channel6;
    pulseWidths[6] = channel7;
    pulseWidths[7] = channel8;
    interrupts();

    //sending the ppm signal out
    sendPPM();



  }else{
    // this is mode 3 => all sensors are read out
    // for future Cedi: Put all the maths here (for full on collision avoidance):
    // check if any sensors are under the threshold and then adjusting the logic, ig just have 6 direction variables which get plus 1 if the threshold is undercut (at the end all of those ned to be set to zero again (after the signal got send out))
    
    //create variables to save the neccessary adjustments
    int forward = 0;
    int backward = 0;
    int left = 0;
    int right = 0;
    int up = 0;
    int down = 0;
    int adjustValue = 25;

    //first hight control (same as in mode 2)
    if (tofTop <= distanceThreshold_z)
    {
      channel4 = constrain(channel4, MIN_THROTTLE + 50, MAX_THROTTLE);  //if throttle is already at 1000 (threshold )
      channel4 -= 50;                 // throttle down
      
    }
    else if (tofBottom <= distanceThreshold_z)

    {
      channel4 = constrain(channel4, MIN_THROTTLE, MAX_THROTTLE - 50);  //if throttle is already at 2000 (threshold)
      channel4 += 50;                 // Throttle up

    }

    //forward backwards  control
    //for the ultrasonic sensors
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

    //for the remaining tof sensors
    if(tofFront < distanceThreshold_xy){
      backward++;
    }
    else if(tofBack < distanceThreshold_xy){
      forward++;
    }


    // manipulating the signal for the ppm
    channel1 = channel1 - (adjustValue * right) + (adjustValue * left);
    if(channel1 < 1000){
      channel1 = 1000;
    }else if(channel1 > 2000){
      channel1 = 2000;
    }
    channel2 = channel2 - (adjustValue * forward) + (adjustValue * backward);
    if(channel2 < 1000){
      channel2 = 1000;
    }else if(channel2 > 2000){
      channel2 = 2000;
    }
    channel4 = channel4 - (adjustValue * down) + (adjustValue * up);
    if(channel4 < 1000){
      channel4 = 1000;
    }else if(channel4 > 2000){
      channel4 = 2000;
    }


    //constraining all the values before putting them into the array values for sending out
    //makes sure that the channel value stays inbetween min and max throtle if smt went wrong before
    channel1 = constrain(channel1,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH); 
    channel2 = constrain(channel2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    channel4 = constrain(channel4, MIN_THROTTLE, MAX_THROTTLE); 


    //saving the channel values into the array values for sending out
    noInterrupts();                   // making sure that the data from the ppm input doesn't change during read out (really important) => everything crucial that shoudnt be disrupted by interupts goes here aka, reading sensor values, performing the calculations, sending the ppm signal out again
    pulseWidths[0] = channel1;
    pulseWidths[1] = channel2;
    pulseWidths[2] = channel3;
    pulseWidths[3] = channel4;
    pulseWidths[4] = channel5;
    pulseWidths[5] = channel6;
    pulseWidths[6] = channel7;
    pulseWidths[7] = channel8;
    interrupts();

    //sending the ppm signal out
    sendPPM();

  }
}



void sendPPM(){
  // Generate PPM signal
  uint32_t pulseStartTime = micros();

  // Send the pulse widths for each channel
  for (int i = 0; i < NUM_CHANNELS; i++) {
    digitalWrite(PPM_PIN_OUT, LOW);
    digitalWrite(PPM_PIN_OUT, HIGH);
    delayMicroseconds(pulseWidths[i]);
  }

  // Calculate the time taken and adjust to match FRAME_DURATION
  uint32_t elapsedTime = micros() - pulseStartTime;
  uint32_t remainingTime = FRAME_DURATION - elapsedTime;

  if (remainingTime > 0) {
    digitalWrite(PPM_PIN_OUT, LOW);
    delayMicroseconds(remainingTime);
  }
  // Update frame start time
  frameStartTime = micros();


}



void readPPM() {
  static uint32_t lastTime = 0;  // Variable to store the time of the previous pulse
  uint32_t currentTime = micros();  // Get the current time in microseconds
  uint32_t interval = currentTime - lastTime;  // Calculate the time interval since the last pulse
  lastTime = currentTime;  // Update lastTime to the current time for the next interval calculation

  if (interval >= 3000) { // Check if the interval is 3000 microseconds or more
    // A sync pulse is detected (interval longer than 3 milliseconds)
    // This sync pulse indicates the start of a new frame period
    currentChannel = 0;  // Reset to the first channel
  } else {
    // If the interval is less than 3000 microseconds, process the pulse for the current channel
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
    currentChannel++;  // Move to the next channel
  }
}



void ppmSetup(){
  // Set the PPM output pin as an output
  pinMode(PPM_PIN_OUT, OUTPUT);
  digitalWrite(PPM_PIN_OUT, LOW);

  // Set the PPM input pin as an input
  pinMode(PPM_PIN, INPUT);

  // Attach an interrupt to the PPM input pin
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, FALLING);

  // Calculate the start time of the frame
  frameStartTime = micros();
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);  
}



void uartSetup(){
  // Serial1 for ESP32 communication (using RX1 and TX1 pins)
  Serial1.begin(115200); // Adjust baud rate to match the ESP32
}



void sensorReadout(){
    static uint8_t buffer[sizeof(float) * 8 + 3]; // Data packet size (sensor data + start + checksum + end)
    static uint8_t index = 0;                    
    uint8_t receivedByte;                         //variable to save the incoming packet


    while (Serial1.available()) {       //check if serial is availabe
        receivedByte = Serial1.read();  //read the data packet


        //receiving the data packet
        // Check for start delimiter (at the beginning of each packet a x0AA is placed to mark the beginning of a new packet, at the end a x0FF is placed to mark the end)
        if (receivedByte == 0xAA && index == 0) {   //check if the 0xAA is there or not, aka see if a new packet started, index == 0 means that currently no packet is beeing processed
            index++;  // Start recording data (startes bc it moves past the delimiter (starting 0xAA part))
        }
        else if (index > 0) { //if the byte isn^t the delimiter here the received data is stored into the buffer, at the index place
            buffer[index++] = receivedByte;
            
            // If buffer is full (i.e., packet received), happends bc the buffer size is beeing declared at the beginning
            if (index == sizeof(buffer)) {
                // Validate the end delimiter and checksum
                if (buffer[sizeof(buffer) - 1] == 0xFF && validateChecksum(buffer, sizeof(buffer))) {
                    // Extract sensor values if the packet is valid, this part isn't from me: https://www.tutorialspoint.com/c_standard_library/c_function_memcpy.htm, and with the assistent of the Tabnine AI
                    memcpy(&tofFront, &buffer[1], sizeof(float));   //first is the destination variable, then from where to pull the data (the source is here the buffer (at the specified index)), then the amount of bytes copied (here 4 bc float uses 4, thats why sizeof(float))
                    memcpy(&tofBack, &buffer[1 + sizeof(float)], sizeof(float));
                    memcpy(&tofTop, &buffer[1 + 2 * sizeof(float)], sizeof(float));
                    memcpy(&tofBottom, &buffer[1 + 3 * sizeof(float)], sizeof(float));
                    memcpy(&ultrasonicDistanceFrontLeft, &buffer[1 + 4 * sizeof(float)], sizeof(float));
                    memcpy(&ultrasonicDistanceFrontRight, &buffer[1 + 5 * sizeof(float)], sizeof(float));
                    memcpy(&ultrasonicDistanceBackLeft, &buffer[1 + 6 * sizeof(float)], sizeof(float));
                    memcpy(&ultrasonicDistanceBackRight, &buffer[1 + 7 * sizeof(float)], sizeof(float));

                    //from here it's again my code

                    // Reset the buffer index for the next packet
                    index = 0;
                } else {
                    // Invalid packet, discard it
                    index = 0;
                }
            }
        }
    }
}


// Function to validate the checksum, not coded by myself: Source Tabnine AI
bool validateChecksum(uint8_t* buffer, int bufferSize) {
    uint8_t checksum = 0x00;
    
    // XOR over all data bytes, starting at buffer[1], excluding start, checksum, and end
    for (int i = 1; i < bufferSize - 2; i++) { 
        checksum ^= buffer[i];
    }
    
    // Compare the calculated checksum with the received checksum (second-to-last byte)
    return (checksum == buffer[bufferSize - 2]);
}


//sources (other sources are mentioned in the code)
//uart: https://www.circuitbasics.com/how-to-set-up-uart-communication-for-arduino/
//      https://www.luisllamas.es/en/esp32-uart/
//      https://forum.arduino.cc/t/communication-between-two-arduino-uno-via-tx-and-rx/1150462/5
//      https://www.pjrc.com/teensy/td_uart.html
//      https://mischianti.org/esp32-s3-devkitc-1-high-resolution-pinout-and-specs/
//c++:  https://cplusplus.com/doc/tutorial/
//      https://www.tutorialspoint.com/c_standard_library/c_function_memcpy.htm 
//      https://www.tutorialspoint.com/c_standard_library/time_h.htm 


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
