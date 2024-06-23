#include <Arduino.h>
#include <Wire.h>

// Define the pin connected to the PPM signal (every Pin on teensy 4.1 is interupt compatible)
#define PPM_PIN 2 

//Variables 
volatile uint16_t channel1; //unit16_t = int with 2^16 different values A
volatile uint16_t channel2;
volatile uint16_t channel3;
volatile uint16_t channel4;
volatile uint16_t channel5;
volatile uint16_t channel6;
volatile uint16_t channel7;
volatile uint16_t channel8;

volatile uint8_t currentChannel = 0; //Volatile: value can change at any time witout action beeing taken, uint8_T: stores values between 0 and 255 (only intergers)

//define the Functions (important, otherwise won't work in Platfrom IO)
void readPPM();

void setup() {
  Serial.begin(9600);  // Start serial communication for debugging
  Serial.println("Start of program");
  pinMode(PPM_PIN, INPUT); //make the PPM_PIN an input pin, is important to regongnice the interupts 
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, FALLING);  // Set up an interrupt on PPM_PIN
  //when an interrupt happends (so like when  ppm signal comes in), the function readPPM gets called
}

void loop() {
  // Print PPM values
  Serial.print("Channel 1: ");
  Serial.print(channel1);
  Serial.print(" us\t");

  Serial.print("Channel 2: ");
  Serial.print(channel2);
  Serial.print(" us\t");

  Serial.print("Channel 3: ");
  Serial.print(channel3);
  Serial.print(" us\t");

  Serial.print("Channel 4: ");
  Serial.print(channel4);
  Serial.print(" us\t");

  Serial.print("Channel 5: ");
  Serial.print(channel5);
  Serial.print(" us\t");

  Serial.print("Channel 6: ");
  Serial.print(channel6);
  Serial.print(" us\t");

  Serial.print("Channel 7: ");
  Serial.print(channel7);
  Serial.print(" us\t");

  Serial.print("Channel 8: ");
  Serial.print(channel8);
  Serial.print(" us\t");

  if (channel1 >= 1600){
    Serial.println("all avoid mode");
  }

  Serial.println();
  delay(20);  // rn on 20 bc we have a 50Hz "refrech Rate" so aka 50 times per second => 20 milisec per revolution, well maybe needs to be adjusted to 10 or 0 to get every signal 
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


