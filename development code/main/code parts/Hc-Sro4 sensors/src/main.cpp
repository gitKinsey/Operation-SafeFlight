#include <Arduino.h>
#include <NewPing.h>

#define TRIGGER_PIN_FRONT_LEFT 10
#define ECHO_PIN_FRONT_LEFT 11
// #define TRIGGER_PIN_FRONT_RIGHT 12
// #define ECHO_PIN_FRONT_RIGHT 13
// #define TRIGGER_PIN_BACK_LEFT 14
// #define ECHO_PIN_BACK_LEFT 15
// #define TRIGGER_PIN_BACK_RIGHT 16
// #define ECHO_PIN_BACK_RIGHT 17
#define MAX_DISTANCE 200 // Maximum distance i wanna ping, maybe needs to get changed 

//definign variables
void doMeasurementUltrasonic(NewPing &sensor, const char *sensorName);


//giving each Sensor NewPing class, from: https://stackoverflow.com/questions/35186703/arduino-hc-sr04-newping-code-not-working

NewPing frontLeftSensor(TRIGGER_PIN_FRONT_LEFT, ECHO_PIN_FRONT_LEFT, MAX_DISTANCE);
// NewPing frontRightSensor(TRIGGER_PIN_FRONT_RIGHT, ECHO_PIN_FRONT_RIGHT, MAX_DISTANCE);
// NewPing backLeftSensor(TRIGGER_PIN_BACK_LEFT, ECHO_PIN_BACK_LEFT, MAX_DISTANCE);
// NewPing backRightSensor(TRIGGER_PIN_BACK_RIGHT, ECHO_PIN_BACK_RIGHT, MAX_DISTANCE);


void setup() {
  Serial.begin(9600);
}

void loop() {
  doMeasurementUltrasonic(frontLeftSensor, "Front Left");
  // doMeasurementUltrasonic(frontRightSensor, "Front Right");
  // doMeasurementUltrasonic(backLeftSensor, "Back Left");
  // doMeasurementUltrasonic(backRightSensor, "Back Right");
  delay(1000);
}

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
