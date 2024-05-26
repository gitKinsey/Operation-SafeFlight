#include "Adafruit_VL53L0X.h"

//ultrasonic sensor
const int trigPin = 7;
const int echoPin = 6;
long duration;
int distance;
char check = false;
//tof sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(9600);
  //ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //tof
  while (!Serial) {
    delay(1);
  }
  Serial.println(" VL53L0X testing...");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }
  // power
  Serial.println(F("VL53L0X on and running..."));
}
void loop() {
  //ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print("Ultrasonic Distance: ");
  Serial.println(distance);

  //tof
  VL53L0X_RangingMeasurementData_t measure;
  Serial.print("Reading a measurement for tof...");
  lox.rangingTest(&measure, false);  //if true => get debug data printout

  if (measure.RangeStatus != 4) {  //phase failures have incorrect data
    Serial.print("Tof distance:");
    Serial.println(measure.RangeMilliMeter / 10);
  } else {
    Serial.println("out of range for tof");
  }

  delay(3000);



  //tof();
  //check = true;
}

void tof() {
  if (check == false) {
    unsigned long StartTime = millis();
    int i = 0;
    for (i = 1; i < 1001; i++) {
      VL53L0X_RangingMeasurementData_t measure;
      Serial.print("Reading a measurement for tof...");
      lox.rangingTest(&measure, false);  //if true => get debug data printout

      if (measure.RangeStatus != 4) {  //phase failures have incorrect data
        Serial.print("Tof distance:");
        Serial.println(measure.RangeMilliMeter / 10);
      } else {
        Serial.println("out of range for tof");
      }
    }
    unsigned long CurrentTime = millis();
    unsigned long ElapsedTime = CurrentTime - StartTime;
    Serial.print("time for 100 measurements: ");
    Serial.println(ElapsedTime);
  }
}
