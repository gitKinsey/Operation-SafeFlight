#include <Wire.h>

// I2C multiplexer address
const int multiplexer_address = 0x70;

// Addresses of devices connected to the multiplexer
const int device_addresses[] = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};

// Variables to hold data from each device
int hans, peter, john, paul, george, ringo, freddy, david;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Initialize each device
  hans = initDevice(0);
  peter = initDevice(1);
  john = initDevice(2);
  paul = initDevice(3);
  george = initDevice(4);
  ringo = initDevice(5);
  freddy = initDevice(6);
  david = initDevice(7);
}

void loop() {
  // Read data from each device
  hans = readData(0);
  peter = readData(1);
  john = readData(2);
  paul = readData(3);
  george = readData(4);
  ringo = readData(5);
  freddy = readData(6);
  david = readData(7);

  // Example: Print data from each device
  Serial.print("Data from Hans: ");
  Serial.println(hans);
  Serial.print("Data from Peter: ");
  Serial.println(peter);
  Serial.print("Data from John: ");
  Serial.println(john);
  Serial.print("Data from Paul: ");
  Serial.println(paul);
  Serial.print("Data from George: ");
  Serial.println(george);
  Serial.print("Data from Ringo: ");
  Serial.println(ringo);
  Serial.print("Data from Freddy: ");
  Serial.println(freddy);
  Serial.print("Data from David: ");
  Serial.println(david);

  delay(1000); // Delay for readability, adjust as needed
}

int initDevice(int device) {
  selectDevice(device);
  // Perform any initialization for the device if needed
  return readData(device);
}

void selectDevice(int device) {
  // Select the device through the multiplexer
  Wire.beginTransmission(multiplexer_address);
  Wire.write(1 << device);
  Wire.endTransmission();
}

int readData(int device) {
  // Read data from the selected device
  selectDevice(device);
  Wire.requestFrom(device_addresses[device], 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return -1; // Return -1 if no data available
}
