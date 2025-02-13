#include <Wire.h>

// Define the I2C address of the multiplexer (for TCA9548A, the default is 0x70)
#define MULTIPLEXER_ADDRESS 0x70

// Onboard LED pin (change this based on your board, GPIO21 for ESP32-S3-Zero)
#define LED_PIN 21

void setup() {
  // Start the serial communication for debugging
  Serial.begin(9600);
  
  // Initialize the LED pin
  pinMode(LED_PIN, OUTPUT);

  // Initialize I2C communication with custom pins (optional, for ESP32)
  Wire.begin(4, 5);  // SDA = pin 4, SCL = pin 5

  // Check if the multiplexer is connected
  Serial.println("Checking for multiplexer...");

  if (isMultiplexerConnected(MULTIPLEXER_ADDRESS)) {
    Serial.print("Multiplexer found at address 0x");
    Serial.println(MULTIPLEXER_ADDRESS, HEX);
    digitalWrite(LED_PIN, HIGH);  // Turn LED ON to indicate the multiplexer is found
  } else {
    Serial.print("No device found at address 0x");
    Serial.println(MULTIPLEXER_ADDRESS, HEX);
    digitalWrite(LED_PIN, LOW);   // Turn LED OFF to indicate failure
  }
}

void loop() {
// Check if the multiplexer is connected
  Serial.println("Checking for multiplexer...");

  if (isMultiplexerConnected(MULTIPLEXER_ADDRESS)) {
    Serial.print("Multiplexer found at address 0x");
    Serial.println(MULTIPLEXER_ADDRESS, HEX);
    digitalWrite(LED_PIN, HIGH);  // Turn LED ON to indicate the multiplexer is found
  } else {
    Serial.print("No device found at address 0x");
    Serial.println(MULTIPLEXER_ADDRESS, HEX);
    digitalWrite(LED_PIN, LOW);   // Turn LED OFF to indicate failure
  }
  delay(1000);
}

/**
 * Function to check if a device responds at a given I2C address
 */
bool isMultiplexerConnected(uint8_t address) {
  Wire.beginTransmission(address);
  uint8_t error = Wire.endTransmission();
  
  // If error is 0, the device is present
  return (error == 0);
}
