#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Replace with your multiplexer address (default for TCA9548A is 0x70)
#define MULTIPLEXER_ADDRESS 0x70
#define SENSOR_ADDRESS 0x29  // Address of the VL53L0X sensor

// Onboard LED pin (change this based on your board, GPIO21 for ESP32-S3-Zero)
#define LED_PIN 21

Adafruit_VL53L0X lox;  // Create a VL53L0X object
int active_channel = 0; // Active channel of the multiplexer

void setup() {
  // Start the serial communication for debugging
  Serial.begin(115200);

  // Initialize I2C communication with custom pins (optional for ESP32)
  Wire.begin(4, 5); // SDA = pin 4, SCL = pin 5

  // Initialize the LED pin
  pinMode(LED_PIN, OUTPUT);
  selectMultiplexerChannel(0);

  // Initialize VL53L0X sensor
  if (!lox.begin(SENSOR_ADDRESS)) {
    Serial.println("Failed to initialize VL53L0X!");
  }

  Serial.println("VL53L0X sensor initialized.");
}

void loop() {
  // Loop through channels 0 to 7 (assuming the TCA9548A multiplexer
    selectMultiplexerChannel(0);
    Serial.print("Reading from channel: ");
    Serial.println(1);

    // Take measurement with the VL53L0X sensor
    takeMeasurement();

    delay(500);  // Wait for 500ms before switching to the next channel
  
}

// Function to select a channel on the multiplexer
void selectMultiplexerChannel(uint8_t channel) {
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  Wire.write(1 << channel);  // Activate the specified channel
  Wire.endTransmission();
}

// Function to read data from the VL53L0X sensor
void takeMeasurement() {
  VL53L0X_RangingMeasurementData_t measure;

  // Start the ranging test
  lox.rangingTest(&measure, false);

  // Check if the measurement is valid
  if (measure.RangeStatus != 4) { // If the measurement is not out of range
    Serial.print("Distance: ");
    Serial.print(measure.RangeMilliMeter / 10.0); // Convert to cm
    Serial.println(" cm");
    digitalWrite(LED_PIN, HIGH);  // Turn the LED on to show reading success
  } else {
    Serial.println("Out of range");
    digitalWrite(LED_PIN, LOW);   // Turn the LED off to indicate failure
  }
}
