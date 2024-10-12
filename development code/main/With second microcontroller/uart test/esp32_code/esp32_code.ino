void setup() {
  // Initialize Serial Monitor for debugging (USB Serial)
  pinMode(13, OUTPUT);  // Initialize pin 13 (LED)

  Serial.begin(9600);  // Start USB serial communication for debugging

  // Initialize UART2 (Serial2) for communication with Teensy
  Serial2.begin(115200, SERIAL_8N1, 16, 17);  // TX on GPIO 17, RX on GPIO 16
}

void loop() {
  // Send data to Teensy via UART (Serial2)
  Serial2.println("Hello from ESP32");

  // Check if data is received from Teensy
  if (Serial2.available()) {
    String fromTeensy = Serial2.readStringUntil('\n');      // Read incoming string from Teensy
    Serial.println("Received from Teensy: " + fromTeensy);  // Print to Serial Monitor for debugging

    // Trim any extra spaces or newline characters from the received string
    fromTeensy.trim();  // Clean up the string

    // Compare the cleaned-up string with "0"
    if (fromTeensy == "0") {
      Serial.println("Matched 0");  // Debug message
      Serial2.println("Received 0");
    } else {
      Serial.println("Did not match 0");  // Debug message
      Serial2.println("Received everything but 0");
    }
  }

  delay(1000);  // Delay for 1 second before sending the next
}