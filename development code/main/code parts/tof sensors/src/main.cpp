
//for more sensors, just switch channels easy, like in the setup, add all the sensor with selectChannel(channel of the sensor)
// and copy paste the initialising sequence
//then in the loop, just switch the channels and you're good to go :)
//future Cedi, for project connect the multiplexer to sda 4 and scl 5 on the teensy 4.1 (so the analoge pins a4 and a5 )

#include "Adafruit_VL53L0X.h"
#include <Wire.h>

#define MULTIPLEXER_ADDRESS 0x70  // I2C address of the multiplexer 
#define SENSOR_ADDRESS 0x29       // I2C address of the VL53L0X sensor


//define functions
void selectChannel(uint8_t channel);
void doMeasurementTOF();
// void recvOnechar();
void printArray(float array[], int size);

#define number_of_tof 4
int current_tof_sensor = 1;
int current_channel_for_initial = 0;
int active_tof = 0; //array start with index = 0
char receivedChar;
float MeasurementTof = 0;

float TofFront = 400; //array index 0 (change via active_tof)
float TofBack = 400; // array index 1 (change via active_tof)
float TofTop = 400; //array index 2 (change via active_tof)
float TofBottom = 400; //array index 3 (change via active_tof)


float ReadOutsTof[number_of_tof] = {TofFront, TofBack, TofTop, TofBottom}; // Store all the Tof readouts

Adafruit_VL53L0X lox; //just a name for the VL53lox sensor


void setup() {
  Serial.begin(9600);
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
  

  //if not working just delete the for loop :), nevermind works (or should be working when last tested on esp32 (need to be tested on teensy 4.1))
  for(current_tof_sensor = 1; current_tof_sensor <= number_of_tof; current_tof_sensor++) { // for loop for checking every connected sensor 
  //(just checks for the number of sensor enterd above)
    selectChannel(current_channel_for_initial);
    if(!lox.begin(SENSOR_ADDRESS)){
        Serial.print("Failed to boot VL53L0X sensor ");
        Serial.println(current_tof_sensor);
        while(1);
    }
    current_channel_for_initial++;
    Serial.print(current_tof_sensor);
    Serial.println(". Vl53L0X sensor up and running");
  } 
  Serial.println("All VL53L0X sensors initialized and running...");
}
void loop() {

  for(int i = 1; i <=4; i++){
    selectChannel(active_tof);
    doMeasurementTOF();
    ReadOutsTof[active_tof] = MeasurementTof;
  }
  printArray(ReadOutsTof, number_of_tof);

//next session was only for debugging purposes, and to see if everything works as it should 
  // recvOnechar();

  //   if (receivedChar == '1') {
  //       selectChannel(0);
  //       Serial.println("Sensor 1");
  //       doMeasurementTOF();
  //   } else if (receivedChar == '2') {
  //       selectChannel(1);
  //       Serial.println("Sensor 2");
  //       doMeasurementTOF();
  //   } else if (receivedChar == '3') {
  //       selectChannel(2);
  //       Serial.println("Sensor 3");
  //       doMeasurementTOF();
  //   } else if (receivedChar == '4') {
  //       selectChannel(3);
  //       Serial.println("Sensor 4");
  //       doMeasurementTOF();
  //   }
}

// Function to select a channel on the multiplexer
void selectChannel(uint8_t channel) {
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  Wire.write(1 << channel);
  active_tof = channel;
  Wire.endTransmission();
}

void doMeasurementTOF(){
  VL53L0X_RangingMeasurementData_t measure;
  
  // Start measurement
  lox.rangingTest(&measure, false);
  
  // Check if measurement is valid
  if (measure.RangeStatus != 4) {
    // Serial.print("Distance: ");
    // Serial.print(measure.RangeMilliMeter / 10.0);
    // Serial.println(" cm");
    MeasurementTof = measure.RangeMilliMeter;
  } else {
    // Serial.println("Out of range");
    MeasurementTof = 400; //just always back to 400 to not interfear with anything ig
  }
  
  // delay(20); //idk how much probs i can just leave that one out ig
}

// void recvOnechar(){ //just for debuging purposes, when not connected to drone you can type the wished tof sensor in the console to get its output (well rn atleast), not neccessary no longer
//   if (Serial.available() > 0){
//     receivedChar = Serial.read();
//   }
// }

void printArray(float array[], int size) {
    Serial.println("Array contents:");
    for (int i = 0; i < size; i++) {
        Serial.print("Element ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(array[i]);
    }
}

