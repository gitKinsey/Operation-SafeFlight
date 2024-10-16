#include <MPU6050.h>
#include <I2Cdev.h>
#include <Wire.h>

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  Serial.println("Initializing Accelerometer / Gyroscope");
  mpu.initialize();

  Serial.println("Testing if Accelerometer / Gyroscope works");
  Serial.println(mpu.testConnection() ? "Success" : "Fail");
}

void loop() {
  // Get gyroscope data
  mpu.getRotation(&gx, &gy, &gz);
  
  Serial.println("Gyroscope rotations:");
  Serial.print("X: ");
  Serial.print(gx);
  Serial.print(" ");
  Serial.print("Y: ");
  Serial.print(gy);
  Serial.print(" ");
  Serial.print("Z: ");
  Serial.println(gz);

  // Get accelerometer data (optional)
  mpu.getAcceleration(&ax, &ay, &az);
  
  Serial.println("Accelerometer readings:");
  Serial.print("X: ");
  Serial.print(ax);
  Serial.print(" ");
  Serial.print("Y: ");
  Serial.print(ay);
  Serial.print(" ");
  Serial.print("Z: ");
  Serial.println(az);

  delay(1000); // Wait for 1 second before repeating
}
