#define TRIG_PIN 1  // Pin connected to the Trig pin of the HC-SR04
#define ECHO_PIN 2  // Pin connected to the Echo pin of the HC-SR04

long duration;
int distance;

void setup() {
  // Start the serial communication
  Serial.begin(115200);

  // Set the TRIG_PIN as output and ECHO_PIN as input
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize the pins
  digitalWrite(TRIG_PIN, LOW);  // Ensure the trigger pin is LOW initially
}

void loop() {
  // Send a pulse to the trigger pin
  digitalWrite(TRIG_PIN, HIGH);   // Send a pulse to trigger
  delayMicroseconds(10);          // Keep the pulse for 10ms
  digitalWrite(TRIG_PIN, LOW);    // Stop the pulse

  // Measure the pulse duration from the Echo pin
  duration = pulseIn(ECHO_PIN, HIGH);  // Measure how long the Echo pin stays HIGH

  // Calculate the distance in centimeters (speed of sound = 34300 cm/s)
  distance = duration * 0.0344 / 2;  // Divide by 2 to account for the round trip

  // Output the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(500);  // Delay before taking another reading
}
