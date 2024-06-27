#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

//multiplexer stuff
#define MULTIPLEXER_ADDRESS 0x70  // I2C address of the multiplexer 
#define SENSOR_ADDRESS 0x29       // I2C address of the VL53L0X sensor


int TofBottomSensor = 0;
float TofBottom = 400; //array index 3 (change via active_tof)
Adafruit_VL53L0X lox; //just a name for the VL53lox sensor
float MeasurementTof = 400;
int active_tof = 0; //array start with index = 0

// Define the pin connected to the PPM signal (every Pin on teensy 4.1 is interrupt compatible)
#define PPM_PIN 2 

// Variables 
volatile uint16_t channel1 = 1500; // uint16_t = int with 2^16 different values, initialize to 1500us
volatile uint16_t channel2 = 1500;
volatile uint16_t channel3 = 1500;
volatile uint16_t channel4 = 1000;
volatile uint16_t channel5;
volatile uint16_t channel6;
volatile uint16_t channel7;
volatile uint16_t channel8;

volatile uint8_t currentChannel = 0; // Volatile: value can change at any time without action being taken, uint8_T: stores values between 0 and 255 (only integers)

// PPM settings
#define PPM_OUT_PIN 3  // Output pin for PPM signal (changed from 2 to 3 to avoid conflict with PPM_PIN)
#define NUM_CHANNELS 8  // Number of PPM channels
#define PPM_PERIOD 20000  // Total PPM frame length in microseconds (20ms)
#define PULSE_LENGTH 300  // Length of sync pulse in microseconds
#define MIN_CHANNEL_PULSE 1000  // Minimum channel pulse length in microseconds
#define MAX_CHANNEL_PULSE 2000  // Maximum channel pulse length in microseconds

uint16_t channelValues[NUM_CHANNELS]; // Create an array that holds uint16_t with all the channel values

// Define the functions
void readPPM();
void sendPPM();
void selectChannel(uint8_t channel);
void doMeasurementTOF();


void setup() {
  Serial.begin(9600);  // Start serial communication for debugging
  Serial.println("Start of program");
  pinMode(PPM_PIN, INPUT_PULLUP); // Enable internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, FALLING);  // Set up an interrupt on PPM_PIN

  pinMode(PPM_OUT_PIN, OUTPUT);
  digitalWrite(PPM_OUT_PIN, HIGH);

  // Initialize channelValues with default values
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = 1500; // Default value for each channel
  }


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
  selectChannel(TofBottomSensor);
  if(!lox.begin(SENSOR_ADDRESS)){
        Serial.print("Failed to boot VL53L0X sensor ");
        Serial.println(current_tof_sensor);
  }
  
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
  //math parts

  if(channel6 < 1600){
    selectChannel(TofBottomSensor);
    doMeasurementTOF();

    while(MeasurementTof <= 200){
      if(MeasurementTof <=75){
        channel4 += 100;
      }
      else{
        channel4 += 50;
      }
      interrupts();
      sendPPM();
      noInterrupts();
      selectChannel(TofBottomSensor);
      doMeasurementTOF();
      uint16_t ch1 = channel1;
      uint16_t ch2 = channel2;
      uint16_t ch3 = channel3;
      // uint16_t ch4 = channel4; //to not update the channel 4, the drone will ignore now the inputs for the height 
      uint16_t ch5 = channel5;
      uint16_t ch6 = channel6;
      uint16_t ch7 = channel7;
      uint16_t ch8 = channel8;
    }

  }

  

  // Update channel values for PPM output
  channelValues[0] = ch1;
  channelValues[1] = ch2;
  channelValues[2] = ch3;
  channelValues[3] = ch4;
  channelValues[4] = ch5;
  channelValues[5] = ch6;
  channelValues[6] = ch7;
  channelValues[7] = ch8;
  interrupts(); // Re-enable interrupts

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
    Serial.print("Sending pulse for channel ");
    Serial.print(i + 1);
    Serial.print(" with length ");
    Serial.print(channelValues[i]);
    Serial.println(" us");

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
  Serial.println("Sending sync pulse");
  digitalWrite(PPM_OUT_PIN, HIGH);
  delayMicroseconds(PULSE_LENGTH);
  digitalWrite(PPM_OUT_PIN, LOW);
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




void selectChannel(uint8_t channel) { // Function to select a channel on the multiplexer
  Wire.begin();
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  Wire.write(1 << channel);
  active_tof = channel;
  Wire.endTransmission();
}


//tof stuff
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
}

