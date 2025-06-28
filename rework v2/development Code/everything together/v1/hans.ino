#include <CRSF.h>

// Use hardware serial if available, else use SoftwareSerial (needs to be installed separately)
// For this example, we'll use Serial1. Change if necessary.
CRSF crsf(Serial1);

void setup() {
  Serial.begin(115200);     // Serial monitor output
  Serial1.begin(420000);    // CRSF baud rate for ELRS

  while (!Serial) {
    ; // Wait for serial to connect
  }

  Serial.println("Starting CRSF receiver...");
  crsf.begin();
}

void loop() {
  crsf.update(); // Must be called frequently to parse incoming packets

  // Check if new RC data is available
  if (crsf.hasRC()) {
    // Print first 8 channels
    Serial.println("RC Channels:");
    for (int i = 0; i < 8; i++) {
      uint16_t value = crsf.rcData[i]; // Raw channel value (typically 172–1811)
      Serial.print("Ch");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(value);
    }
    Serial.println("---");
    delay(500); // Slow down printout
  }
}
