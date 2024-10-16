const int trigPin = 7;
const int echoPin = 6;
long duration;
int distance;
int measurements = 5; // Number of measurements to take for averaging
bool takeMeasurement = false;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'r') {
      takeMeasurement = true; // Set flag to take measurements
    }
  }

  if (takeMeasurement) {
    int totalDistance = 0;
    for (int i = 0; i < measurements; i++) {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = duration * 0.034 / 2;
      totalDistance += distance;
    }
    int averageDistance = totalDistance / measurements;
    Serial.print("Average Distance: ");
    Serial.println(averageDistance);
    takeMeasurement = false; // Reset flag
  }
}
