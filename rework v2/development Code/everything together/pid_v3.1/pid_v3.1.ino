#include <Arduino.h>
#include <FreeRTOS.h>
#include <NewPing.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <PID_v1.h>

// --- Konstanten-Definitionen zusammengefasst ---
// PID-Konstanten
constexpr double BASE_Kp_z = 2.0, BASE_Ki_z = 5.0, BASE_Kd_z = 1.0;
constexpr double BASE_Kp_x = 1.5, BASE_Ki_x = 4.0, BASE_Kd_x = 0.8;
constexpr double BASE_Kp_y = 1.5, BASE_Ki_y = 4.0, BASE_Kd_y = 0.8;

// Tuning-Konstanten
constexpr int FORCE_SCALE = 100;
constexpr double SETPOINT_ADJUST_STEP = 3.0;
constexpr double LOWPASS_ALPHA = 0.2;
constexpr int SAFE_DISTANCE = 100;
constexpr int CRITICAL_DISTANCE = 50;
constexpr int DEADZONE = 10;

// PPM-Konstanten
constexpr uint16_t MIN_THROTTLE = 1000;     // Minimum throttle value for safety
constexpr uint16_t MAX_THROTTLE = 2000;     // Maximum throttle value for safety
constexpr uint32_t FRAME_DURATION = 20000;  // PPM frame duration in microseconds

// Sensorkonstanten
constexpr int MAX_DISTANCE = 205;          // Maximum distance for ultrasonic sensors
constexpr int MULTIPLEXER_ADDRESS = 0x70;  // I2C address for TCA9548A multiplexer
constexpr int SENSOR_ADDRESS = 0x29;       // I2C address for VL53L0X
constexpr int MAX_LIST_SIZE = 4;           // Maximum number of VL53L0X sensors

// Debug-Konstanten
constexpr int DEBUG_MODE = 1;              // 0 = Off, 1 = Basic, 2 = Verbose

// --- Globale Variablen ---
// PID-Instanzen
double z_input = 0, z_output = 0, z_setpoint = 150;
double x_input = 0, x_output = 0, x_setpoint = 0;
double y_input = 0, y_output = 0, y_setpoint = 0;

PID z_pid(&z_input, &z_output, &z_setpoint, BASE_Kp_z, BASE_Ki_z, BASE_Kd_z, DIRECT);
PID x_pid(&x_input, &x_output, &x_setpoint, BASE_Kp_x, BASE_Ki_x, BASE_Kd_x, DIRECT);
PID y_pid(&y_input, &y_output, &y_setpoint, BASE_Kp_y, BASE_Ki_y, BASE_Kd_y, DIRECT);

// Zustandsvariablen
unsigned long obstacleTimers[4] = {0, 0, 0, 0};
unsigned long stuckSince = 0;
bool rescueMode = false;
bool sensor_noise = false;

int last_front = 0, last_back = 0;
int last_escape_direction_x = 0, last_escape_direction_y = 0;
unsigned long last_clear_path_time = 0;

float filtered_front = 0;
float filtered_back = 0;
float filtered_diag_left = 0;
float filtered_diag_right = 0;

// Sensor-Initialisierungen
int availableVL53l0x[MAX_LIST_SIZE];
int currentVL53l0xListSize = 0;

// Ultraschall-Sensoren Pin-Definitionen
#define TRIG_PIN_FRONT_LEFT  1
#define ECHO_PIN_FRONT_LEFT  2
#define TRIG_PIN_FRONT_RIGHT 3
#define ECHO_PIN_FRONT_RIGHT 6
#define TRIG_PIN_REAR_RIGHT  7
#define ECHO_PIN_REAR_RIGHT  8
#define TRIG_PIN_REAR_LEFT   9
#define ECHO_PIN_REAR_LEFT   10

// Ultraschall-Sensorwerte
unsigned int distance_ultrasonic_front_left;
unsigned int distance_ultrasonic_front_right;
unsigned int distance_ultrasonic_rear_left;
unsigned int distance_ultrasonic_rear_right;

// PPM-Pin-Definitionen
#define PPM_PIN 13       // Input Pin für PPM-Signal
#define PPM_PIN_OUT 12   // Output Pin für modifiziertes PPM-Signal

// PPM-Kanal-Werte
volatile uint16_t channel1 = 1500;
volatile uint16_t channel2 = 1500;
volatile uint16_t channel3 = 1500;
volatile uint16_t channel4 = 1000;
volatile uint16_t channel5 = 1500;
volatile uint16_t channel6 = 1500;
volatile uint16_t channel7 = 1500;
volatile uint16_t channel8 = 1500;

// PPM-Ausgabe-Werte
#define NUM_CHANNELS 8
#define TOTAL_PULSES (NUM_CHANNELS + 1)
uint16_t pulseWidths[NUM_CHANNELS];
volatile uint8_t currentChannel = 0;

// VL53L0X-Sensoren
Adafruit_VL53L0X lox;
int active_channel = 0;
uint16_t vl53l0xMeasurements[4];

// ToF-Sensorwerte mit Standardwerten
float tofFront = 400;
float tofBack = 400;
float tofTop = 400;
float tofBottom = 400;
float tofReadOuts[4] = {tofFront, tofBack, tofTop, tofBottom};

// NewPing-Objekte für Ultraschall-Sensoren
NewPing sonar_front_left(TRIG_PIN_FRONT_LEFT, ECHO_PIN_FRONT_LEFT, MAX_DISTANCE);
NewPing sonar_front_right(TRIG_PIN_FRONT_RIGHT, ECHO_PIN_FRONT_RIGHT, MAX_DISTANCE);
NewPing sonar_rear_left(TRIG_PIN_REAR_LEFT, ECHO_PIN_REAR_LEFT, MAX_DISTANCE);
NewPing sonar_rear_right(TRIG_PIN_REAR_RIGHT, ECHO_PIN_REAR_RIGHT, MAX_DISTANCE);

// RTOS-Synchronisation
SemaphoreHandle_t ppmMutex;
SemaphoreHandle_t sensorMutex;

// Flight-Mode Enum
enum FlightMode {
  PASSTHROUGH,
  HEIGHT_CONTROL, 
  FULL_AVOIDANCE
};

// --- Funktionsprototypen ---
void Sensor_task(void *pvParameters);
void ppm_task(void *pvParameters);
void readPPM();
void updateAndSendPPM();
void sendPPM();
void measurement_ultrasonic(NewPing &sonar, unsigned int &distance);
float measurement_vl53l0x();
void selectMultiplexerChannel(uint8_t channel);
void initializeVl53l0x();
void addToList(int value);
int selectElement(int index);
void readAllVL53L0x();
float applyLowPassFilter(float oldValue, float newValue, float alpha);
void debugPrint(const char* message, int level = 1);
void debugPrintValue(const char* label, float value, int level = 1);
FlightMode getCurrentMode();
void controlHeight();
void processProximityData();

// --- Hilfsfunktionen ---
float applyLowPassFilter(float oldValue, float newValue, float alpha) {
  return (1.0 - alpha) * oldValue + alpha * newValue;
}

void debugPrint(const char* message, int level) {
  if (DEBUG_MODE >= level) {
    Serial.println(message);
  }
}

void debugPrintValue(const char* label, float value, int level) {
  if (DEBUG_MODE >= level) {
    Serial.print(label);
    Serial.print(": ");
    Serial.println(value);
  }
}

FlightMode getCurrentMode() {
  if (channel6 >= 1700) return PASSTHROUGH;
  if (channel6 >= 1300) return HEIGHT_CONTROL;
  return FULL_AVOIDANCE;
}

// --- PPM-Signal-Verarbeitung ---
void readPPM() {
  static uint32_t lastTime = 0;
  uint32_t currentTime = micros();
  uint32_t interval = currentTime - lastTime;
  lastTime = currentTime;

  if (interval >= 3000) {
    currentChannel = 0;
  } else {
    switch (currentChannel) {
      case 0:
        // Korrektur für Kanal 1
        channel1 = (interval >= 50) ? interval - 50 : 0;
        break;
      case 1: channel2 = interval; break;
      case 2: channel3 = interval; break;
      case 3: channel4 = interval; break;
      case 4: channel5 = interval; break;
      case 5: channel6 = interval; break;
      case 6: channel7 = interval; break;
      case 7: channel8 = interval; break;
    }
    currentChannel++;
  }
}

void sendPPM() {
  // Kritischer Abschnitt für die PPM-Übertragung
  portENTER_CRITICAL_ISR(NULL);
  
  uint32_t pulseStartTime = micros();
  
  // Pulsweitenmodulation für jeden Kanal senden
  for (int i = 0; i < NUM_CHANNELS; i++) {
    digitalWrite(PPM_PIN_OUT, HIGH);
    delayMicroseconds(pulseWidths[i]);
    digitalWrite(PPM_PIN_OUT, LOW);
    delayMicroseconds(300); // Feste Pause zwischen Pulsen
  }
  
  // Restzeit bis zum nächsten Frame berechnen und warten
  uint32_t elapsedTime = micros() - pulseStartTime;
  uint32_t remainingTime = FRAME_DURATION - elapsedTime;
  
  if (remainingTime > 0 && remainingTime < FRAME_DURATION) {
    delayMicroseconds(remainingTime);
  }
  
  portEXIT_CRITICAL_ISR(NULL);
}

void updateAndSendPPM() {
  // PPM-Werte thread-sicher aktualisieren
  if (xSemaphoreTake(ppmMutex, portMAX_DELAY) == pdTRUE) {
    pulseWidths[0] = constrain(channel1, MIN_THROTTLE, MAX_THROTTLE);
    pulseWidths[1] = constrain(channel2, MIN_THROTTLE, MAX_THROTTLE);
    pulseWidths[2] = constrain(channel3, MIN_THROTTLE, MAX_THROTTLE);
    pulseWidths[3] = constrain(channel4, MIN_THROTTLE, MAX_THROTTLE);
    pulseWidths[4] = constrain(channel5, MIN_THROTTLE, MAX_THROTTLE);
    pulseWidths[5] = constrain(channel6, MIN_THROTTLE, MAX_THROTTLE);
    pulseWidths[6] = constrain(channel7, MIN_THROTTLE, MAX_THROTTLE);
    pulseWidths[7] = constrain(channel8, MIN_THROTTLE, MAX_THROTTLE);
    xSemaphoreGive(ppmMutex);
  }
  
  sendPPM();
}

// --- Sensor-Funktionen ---
void measurement_ultrasonic(NewPing &sonar, unsigned int &distance) {
  distance = sonar.ping_cm();
  
  if (distance == 0) {
    distance = MAX_DISTANCE;
  }
  
  debugPrintValue("Distance", distance, 2);
}

float measurement_vl53l0x() {
  VL53L0X_RangingMeasurementData_t measure;
  
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4) {
    float distance = measure.RangeMilliMeter / 10.0;
    debugPrintValue("ToF Distance", distance, 2);
    return distance;
  } else {
    debugPrint("ToF out of range", 2);
    return MAX_DISTANCE;
  }
}

void selectMultiplexerChannel(uint8_t channel) {
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(10); // Kurze Verzögerung für Stabilität
}

void initializeVl53l0x() {
  debugPrint("Initializing VL53L0X sensors...", 1);
  
  for (int i = 0; i < 4; i++) {
    selectMultiplexerChannel(i);
    delay(50); // Stabilitätsdelay
    
    if (!lox.begin(SENSOR_ADDRESS)) {
      debugPrint("Failed to initialize VL53L0X sensor!", 1);
    } else {
      debugPrintValue("VL53L0X sensor initialized on channel", i, 1);
      addToList(i);
    }
  }
  
  debugPrintValue("Total VL53L0X sensors found", currentVL53l0xListSize, 1);
}

void addToList(int value) {
  if (currentVL53l0xListSize < MAX_LIST_SIZE) {
    availableVL53l0x[currentVL53l0xListSize] = value;
    currentVL53l0xListSize++;
  } else {
    debugPrint("VL53L0X list is full", 1);
  }
}

int selectElement(int index) {
  if (index >= 0 && index < currentVL53l0xListSize) {
    return availableVL53l0x[index];
  } else {
    debugPrint("Invalid sensor index", 1);
    return -1;
  }
}

void readAllVL53L0x() {
  static uint8_t consecutiveFailures[4] = {0, 0, 0, 0};
  static float lastValidReadings[4] = {400, 400, 400, 400};
  
  // Thread-sicher Sensordaten lesen
  if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < currentVL53l0xListSize; i++) {
      int sensorIndex = availableVL53l0x[i];
      selectMultiplexerChannel(sensorIndex);
      
      float distance = measurement_vl53l0x();
      
      // Fehlerbehandlung und Plausibilitätsprüfung
      if (distance == MAX_DISTANCE || (abs(distance - lastValidReadings[sensorIndex]) > 50 && lastValidReadings[sensorIndex] < 200)) {
        consecutiveFailures[sensorIndex]++;
        
        if (consecutiveFailures[sensorIndex] > 3) {
          debugPrintValue("Using fallback value for sensor", sensorIndex, 1);
          distance = lastValidReadings[sensorIndex];
        }
      } else {
        consecutiveFailures[sensorIndex] = 0;
        lastValidReadings[sensorIndex] = distance;
      }
      
      // Werte den entsprechenden Sensoren zuordnen
      switch (sensorIndex) {
        case 0: tofFront = distance; break;
        case 1: tofBack = distance; break;
        case 2: tofTop = distance; break;
        case 3: tofBottom = distance; break;
      }
    }
    
    // Array für externen Zugriff aktualisieren
    tofReadOuts[0] = tofFront;
    tofReadOuts[1] = tofBack;
    tofReadOuts[2] = tofTop;
    tofReadOuts[3] = tofBottom;
    
    xSemaphoreGive(sensorMutex);
  }
}

// --- Flugkontrollfunktionen ---
void controlHeight() {
  if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
    // Setpoint-Anpassung basierend auf Sensordaten
    if (tofTop < 50) z_setpoint = max(z_setpoint - SETPOINT_ADJUST_STEP, 100.00);
    if (tofBottom < 50) z_setpoint = min(z_setpoint + SETPOINT_ADJUST_STEP, 300.00);
    
    // Z-PID-Controller berechnen
    z_input = (tofTop + (2000 - tofBottom)) / 2.0;
    xSemaphoreGive(sensorMutex);
  }
  
  z_pid.Compute();
  
  // Throttle anpassen basierend auf PID-Output
  if (xSemaphoreTake(ppmMutex, portMAX_DELAY) == pdTRUE) {
    channel4 = constrain(channel4 + (int)z_output, MIN_THROTTLE, MAX_THROTTLE);
    xSemaphoreGive(ppmMutex);
  }
}

void processProximityData() {
  // Ultraschall-Sensorwerte einlesen & filtern
  if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
    int raw_front = min(distance_ultrasonic_front_left, distance_ultrasonic_front_right);
    int raw_back = min(distance_ultrasonic_rear_left, distance_ultrasonic_rear_right);
    int raw_diag_left = min(distance_ultrasonic_front_left, distance_ultrasonic_rear_left);
    int raw_diag_right = min(distance_ultrasonic_front_right, distance_ultrasonic_rear_right);
    
    // Low-Pass-Filter anwenden
    filtered_front = applyLowPassFilter(filtered_front, raw_front, LOWPASS_ALPHA);
    filtered_back = applyLowPassFilter(filtered_back, raw_back, LOWPASS_ALPHA);
    filtered_diag_left = applyLowPassFilter(filtered_diag_left, raw_diag_left, LOWPASS_ALPHA);
    filtered_diag_right = applyLowPassFilter(filtered_diag_right, raw_diag_right, LOWPASS_ALPHA);
    
    xSemaphoreGive(sensorMutex);
  }
  
  int front = (int)filtered_front;
  int back = (int)filtered_back;
  int diag_left = (int)filtered_diag_left;
  int diag_right = (int)filtered_diag_right;
  
  // Deadzone-Ausgleich für stabile Werte
  if (abs(front - back) < DEADZONE) front = back = (front + back) / 2;
  if (abs(diag_left - diag_right) < DEADZONE) diag_left = diag_right = (diag_left + diag_right) / 2;
  
  // Plausibilitätsprüfung
  sensor_noise = (abs(front - last_front) > 30 || abs(back - last_back) > 30);
  last_front = front;
  last_back = back;
  
  // Überprüfen auf kritische Situationen
  bool critical = false;
  int directions[4] = {front, back, diag_left, diag_right};
  for (int i = 0; i < 4; i++) {
    if (directions[i] < CRITICAL_DISTANCE) {
      if (millis() - obstacleTimers[i] > 300) critical = true;
    } else {
      obstacleTimers[i] = millis();
    }
  }
  
  // Prüfen auf feststeckenden Zustand
  if (front < 50 && back < 50 && diag_left < 50 && diag_right < 50) {
    if (stuckSince == 0) stuckSince = millis();
    if (millis() - stuckSince > 1000) rescueMode = true;
  } else {
    stuckSince = 0;
    rescueMode = false;
  }
  
  // Rettungsmodus oder kritische Situation
  if (critical || rescueMode) {
    if (xSemaphoreTake(ppmMutex, portMAX_DELAY) == pdTRUE) {
      channel1 = 1500;
      channel2 = 1900; // Schnell nach vorne
      channel3 = 1500;
      channel4 = constrain(channel4 + 100, MIN_THROTTLE, MAX_THROTTLE); // Erhöhter Schub
      xSemaphoreGive(ppmMutex);
    }
    
    debugPrint("RESCUE MODE ACTIVATED", 1);
  } else {
    // Normale Hindernisvermeidung
    float front_w = constrain(map(front, 30, 150, 1.0, 0.0), 0.0, 1.0);
    float back_w = constrain(map(back, 30, 150, 1.0, 0.0), 0.0, 1.0);
    float left_w = constrain(map(diag_left, 30, 150, 1.0, 0.0), 0.0, 1.0);
    float right_w = constrain(map(diag_right, 30, 150, 1.0, 0.0), 0.0, 1.0);
    
    double x_force = back_w - front_w;
    double y_force = left_w - right_w;
    
    // Speichern der letzten Ausweichrichtung
    if (x_force != 0 || y_force != 0) {
      last_escape_direction_x = x_force * 100;
      last_escape_direction_y = y_force * 100;
      last_clear_path_time = millis();
    }
    
    // Verwenden der gespeicherten Richtung, wenn alle Sensoren freie Bahn melden
    if (millis() - last_clear_path_time > 300 && x_force == 0 && y_force == 0) {
      x_force = last_escape_direction_x / 100.0;
      y_force = last_escape_direction_y / 100.0;
    }
    
    // PID-Tuning basierend auf Hindernisproximität
    bool near = front < SAFE_DISTANCE || back < SAFE_DISTANCE || diag_left < SAFE_DISTANCE || diag_right < SAFE_DISTANCE;
    if (near) {
      x_pid.SetTunings(BASE_Kp_x * 1.5, BASE_Ki_x * 1.2, BASE_Kd_x);
      y_pid.SetTunings(BASE_Kp_y * 1.5, BASE_Ki_y * 1.2, BASE_Kd_y);
    } else {
      x_pid.SetTunings(BASE_Kp_x, BASE_Ki_x, BASE_Kd_x);
      y_pid.SetTunings(BASE_Kp_y, BASE_Ki_y, BASE_Kd_y);
    }
    
    // PID-Berechnungen durchführen
    x_input = x_force * FORCE_SCALE;
    y_input = y_force * FORCE_SCALE;
    
    if (!sensor_noise) {
      x_pid.Compute();
      y_pid.Compute();
      
      // Ausgangswerte anwenden
      double scale = near ? 0.6 : 1.0;
      
      if (xSemaphoreTake(ppmMutex, portMAX_DELAY) == pdTRUE) {
        channel1 += (int)(x_output * scale);
        channel2 += (int)(y_output * scale);
        
        // Kollisionsschutz für die Drehung
        if (diag_left < 50 || diag_right < 50) {
          channel3 = 1500; // Neutralposition für Drehung
        }
        
        xSemaphoreGive(ppmMutex);
      }
    }
  }
}

// --- Tasks ---
void ppm_task(void *pvParameters) {
  // PID-Controller-Setup
  z_pid.SetOutputLimits(-300, 300);
  x_pid.SetOutputLimits(-100, 100);
  y_pid.SetOutputLimits(-100, 100);
  
  z_pid.SetMode(AUTOMATIC);
  x_pid.SetMode(AUTOMATIC);
  y_pid.SetMode(AUTOMATIC);
  
  while (true) {
    FlightMode mode = getCurrentMode();
    
    switch (mode) {
      case PASSTHROUGH:
        // Direkte Weiterleitung der Steuerdaten
        updateAndSendPPM();
        break;
        
      case HEIGHT_CONTROL:
        // Nur Höhenkontrolle aktiv
        controlHeight();
        updateAndSendPPM();
        break;
        
      case FULL_AVOIDANCE:
        // Vollständige Kollisionsvermeidung
        processProximityData();
        controlHeight();
        updateAndSendPPM();
        break;
    }
    
    // Task-Delay für RTOS-Scheduling
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void Sensor_task(void *pvParameters) {
  while (true) {
    // Ultraschall-Sensoren auslesen
    measurement_ultrasonic(sonar_front_left, distance_ultrasonic_front_left);
    measurement_ultrasonic(sonar_front_right, distance_ultrasonic_front_right);
    measurement_ultrasonic(sonar_rear_left, distance_ultrasonic_rear_left);
    measurement_ultrasonic(sonar_rear_right, distance_ultrasonic_rear_right);
    
    // TOF-Sensoren auslesen
    readAllVL53L0x();
    
    // Diagnostische Ausgabe
    if (DEBUG_MODE >= 1) {
      static unsigned long lastDebugOutput = 0;
      if (millis() - lastDebugOutput > 1000) {
        debugPrint("--- Sensor Status ---", 1);
        debugPrintValue("Front", filtered_front, 1);
        debugPrintValue("Back", filtered_back, 1);
        debugPrintValue("Left", filtered_diag_left, 1);
        debugPrintValue("Right", filtered_diag_right, 1);
        debugPrintValue("Top", tofTop, 1);
        debugPrintValue("Bottom", tofBottom, 1);
        debugPrintValue("Mode", (int)getCurrentMode(), 1);
        lastDebugOutput = millis();
      }
    }
    
    // Mehr dynamische Anpassung der Abtastrate basierend auf Nähe zu Hindernissen
    int minDistance = min(min(distance_ultrasonic_front_left, distance_ultrasonic_front_right), 
                       min(distance_ultrasonic_rear_left, distance_ultrasonic_rear_right));
    
    int delay_time = map(constrain(minDistance, 30, 150), 30, 150, 50, 200);
    vTaskDelay(pdMS_TO_TICKS(delay_time));
  }
}

// --- Hauptfunktionen ---
void setup() {
  // Serielle Kommunikation initialisieren
  Serial.begin(115200);
  debugPrint("Initializing flight control system...", 1);
  
  // Pin-Modus einstellen
  pinMode(PPM_PIN_OUT, OUTPUT);
  digitalWrite(PPM_PIN_OUT, LOW);
  pinMode(PPM_PIN, INPUT);
  
  // Interrupt für PPM-Signalerfassung einrichten
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, FALLING);
  
  // I2C-Kommunikation initialisieren
  Wire.begin(4, 5); // SDA = Pin 4, SCL = Pin 5
  
  // VL53L0X-Sensoren initialisieren
  initializeVl53l0x();
  
  // Mutexes für Thread-Sicherheit erstellen
  ppmMutex = xSemaphoreCreateMutex();
  sensorMutex = xSemaphoreCreateMutex();
  
  // FreeRTOS-Tasks erstellen
  xTaskCreatePinnedToCore(
    Sensor_task,           // Task-Funktion
    "Sensor_task",         // Task-Name
    4096,                  // Stack-Größe
    NULL,                  // Parameter
    2,                     // Priorität (höher)
    NULL,                  // Task-Handle
    1                      // Core 1
  );
  
  xTaskCreatePinnedToCore(
    ppm_task,              // Task-Funktion
    "PPM Task",            // Task-Name
    4096,                  // Stack-Größe
    NULL,                  // Parameter
    1,                     // Priorität (niedriger)
    NULL,                  // Task-Handle
    0                      // Core 0
  );
  
  debugPrint("Setup complete. Flight control system active.", 1);
}

void loop() {
  // Die loop-Funktion bleibt leer, da FreeRTOS-Tasks verwendet werden
}