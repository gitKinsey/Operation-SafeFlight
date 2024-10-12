void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  // USB Serial for computer communication
  Serial.begin(9600); 
  
  // Serial1 for ESP32 communication (using RX1 and TX1 pins)
  Serial1.begin(115200); // Adjust baud rate to match the ESP32
}

void loop() {
  // Read data from ESP32 (UART) and print it to the computer (USB)
 if (Serial.available()) {
    // Read data as a byte array from the computer via USB
    // Read one byte from the computer and send it directly to ESP32
    char fromComputer = Serial.read();  
    Serial1.write(fromComputer);  // Send the raw byte to ESP32 via UART
    
}

  if (Serial1.available()) {
    String fromESP32 = Serial1.readStringUntil('\n');
    Serial.println("Received from ESP32: " + fromESP32); // Send data to computer
    digitalWrite(13, LOW);
  }

  delay(100); // Small delay to allow processing
  digitalWrite(13, HIGH);
}
