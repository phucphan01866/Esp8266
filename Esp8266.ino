// === Project: IoT Control with ESP8266 and Blynk ===
// Optimized for readability, modularity, and Blynk integration

// === Blynk Configuration ===
#define BLYNK_TEMPLATE_ID "TMPL6pTMyOqrJ"
#define BLYNK_TEMPLATE_NAME "IOTPROJECTCK"
#define BLYNK_AUTH_TOKEN "Q4Ey4Ax1wdSfL9c4dThwAtnzM7LquQJg"
#define BLYNK_PRINT Serial
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "PTIT.HCM_SV";
char pass[] = "";

// === Libraries ===
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Servo.h>
#include <DHT.h>

// === Pin and Sensor Configuration ===
#define SERVO_HIGH_PIN D2    // Servo 360 for roof control (first servo)
#define SERVO_LOW_PIN D4     // Servo 360 for roof control (second servo)
#define DC_PIN D6            // Relay for water pump
#define LED_PIN D5           // Relay for LED
#define LIGHT_SENSOR_PIN D0  // Light sensor
#define SOIL_SENSOR_PIN A0   // Soil moisture sensor
#define DHT_PIN D7           // DHT11 sensor

// Sensor and actuator objects
DHT dht(DHT_PIN, DHT11);
Servo servoHigh;            // First servo (360-degree)
Servo servoLow;             // Second servo (360-degree)
float temperature = 0.0;    // Air temperature (°C)
float humidity = 0.0;       // Air humidity (%)
int soilMoisture = 0;       // Soil moisture (raw ADC value)
bool autoLightMode = true;  // Auto control LED based on light sensor
bool autoPumpMode = true;   // Auto control pump based on soil moisture
bool autoRoofMode = true;   // Auto control roof based on air condition
int manualLightState = 0;   // Store manual state for LED (0=OFF, 1=ON)
int manualPumpState = 0;    // Store manual state for Pump (0=OFF, 1=ON)
int lastServoHighCommand = -1; // Last command for servoHigh
int lastServoLowCommand = -1;  // Last command for servoLow
bool isRoofClosed = true;
bool isServoHighRunning = false;
bool isServoLowRunning = false;
unsigned long servoHighStartTime = 0;
unsigned long servoLowStartTime = 0;
unsigned long SERVO_HIGH_DURATION = 3000; // 3 seconds
unsigned long SERVO_LOW_DURATION = 3000;  // 3 seconds

// === Timer for Periodic Updates ===
BlynkTimer timer;

// === Setup Function ===
void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Configure pin modes
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(SOIL_SENSOR_PIN, INPUT);
  pinMode(DC_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Initialize servos
  servoHigh.attach(SERVO_HIGH_PIN);
  servoLow.attach(SERVO_LOW_PIN);

  // Initialize DHT sensor
  dht.begin();

  // Initialize Blynk
  Blynk.begin(auth, ssid, pass);

  // Setup timer for periodic sensor updates (every 10 seconds)
  timer.setInterval(1000L, updateSensors);

  // Ensure initial states
  digitalWrite(DC_PIN, LOW);          // Pump off
  digitalWrite(LED_PIN, LOW);         // LED off
  servoHigh.writeMicroseconds(1500);  // Stop servoHigh
  servoLow.writeMicroseconds(1500);   // Stop servoLow
  Blynk.virtualWrite(V5, 0);          // Set default Slider V5 to Stop
  Blynk.virtualWrite(V8, 0);          // Set default Slider V8 to Stop
  Blynk.virtualWrite(V9, 0);          // Set default Roof Auto Mode
}

// === Main Loop ===
void loop() {
  if (Blynk.connected()) {
    Blynk.run();  // Handle Blynk communication
  } else {
    servoHigh.writeMicroseconds(1500);
    servoLow.writeMicroseconds(1500);
    Serial.println("Lost Blynk connection, stopping servos");
  }
  timer.run();  // Run timer for sensor updates
  updateRoofMovement();
}

// === Blynk Callbacks ===
// Control LED Manual On/Off (V0)
BLYNK_WRITE(V0) {
  if (!autoLightMode) {
    manualLightState = param.asInt();
    digitalWrite(LED_PIN, manualLightState);
    Serial.println(manualLightState ? "LED ON (Manual)" : "LED OFF (Manual)");
  }
}

// Control Pump Manual On/Off (V1)
BLYNK_WRITE(V1) {
  if (!autoPumpMode) {
    manualPumpState = param.asInt();
    digitalWrite(DC_PIN, manualPumpState);
    Serial.println(manualPumpState ? "Pump ON (Manual)" : "Pump OFF (Manual)");
  }
}

// Control First Servo (V5) - Slider: 0=Stop, 1=Left (CCW), 2=Right (CW)
BLYNK_WRITE(V5) {
  int servoCommand = param.asInt();
  if (servoCommand != lastServoHighCommand && !autoRoofMode) {
    lastServoHighCommand = servoCommand;
    if (servoCommand == 1) {
      servoHigh.writeMicroseconds(1700);  // Rotate CCW (Left)
      Serial.println("ServoHigh Rotating CCW (Left)");
    } else if (servoCommand == 2) {
      servoHigh.writeMicroseconds(1300);  // Rotate CW (Right)
      Serial.println("ServoHigh Rotating CW (Right)");
    } else {
      servoHigh.writeMicroseconds(1500);  // Stop servo
      Serial.println("ServoHigh Stopped");
    }
  }
}

// Control LED Auto/Manual Mode (V6)
BLYNK_WRITE(V6) {
  autoLightMode = !param.asInt();  // 0=Auto, 1=Manual
  Serial.println(autoLightMode ? "LED Auto Mode" : "LED Manual Mode");
  if (autoLightMode) {
    updateLedBasedOnLightSensor();
  } else {
    digitalWrite(LED_PIN, manualLightState);
    Blynk.virtualWrite(V0, manualLightState);
  }
}

// Control Pump Auto/Manual Mode (V7)
BLYNK_WRITE(V7) {
  autoPumpMode = !param.asInt();  // 0=Auto, 1=Manual
  Serial.println(autoPumpMode ? "Pump Auto Mode" : "Pump Manual Mode");
  if (autoPumpMode) {
    updateSoilStatus();
  } else {
    digitalWrite(DC_PIN, manualPumpState);
    Blynk.virtualWrite(V1, manualPumpState);
  }
}

// Control Second Servo (V8) - Slider: 0=Stop, 1=Left (CCW), 2=Right (CW)
BLYNK_WRITE(V8) {
  int servoCommand = param.asInt();
  if (servoCommand != lastServoLowCommand && !autoRoofMode) {
    lastServoLowCommand = servoCommand;
    if (servoCommand == 1) {
      servoLow.writeMicroseconds(1700);  // Rotate CCW (Left)
      Serial.println("ServoLow Rotating CCW (Left)");
    } else if (servoCommand == 2) {
      servoLow.writeMicroseconds(1300);  // Rotate CW (Right)
      Serial.println("ServoLow Rotating CW (Right)");
    } else {
      servoLow.writeMicroseconds(1500);  // Stop servo
      Serial.println("ServoLow Stopped");
    }
  }
}

// Control Roof Auto/Manual Mode (V9)
BLYNK_WRITE(V9) {
  autoRoofMode = !param.asInt();  // 0=Auto, 1=Manual
  Serial.println(autoRoofMode ? "Roof Auto Mode" : "Roof Manual Mode");
  if (!autoRoofMode) {
    servoHigh.writeMicroseconds(1500);
    servoLow.writeMicroseconds(1500);
    Serial.println("Servos stopped due to manual roof mode");
  }
}

// === Sensor and Actuator Functions ===
// Update LED state based on light sensor (auto mode)
void updateLedBasedOnLightSensor() {
  if (autoLightMode) {
    bool lightState = digitalRead(LIGHT_SENSOR_PIN);
    digitalWrite(LED_PIN, !lightState);   // Turn on LED if dark
    Blynk.virtualWrite(V0, !lightState);  // Update Blynk UI
    Serial.print("Light sensor: ");
    Serial.println(lightState ? "Light" : "Dark");
  }
}

// Read and process soil moisture
void updateSoilStatus() {
  soilMoisture = analogRead(SOIL_SENSOR_PIN);
  Serial.print("Soil moisture: ");
  Serial.print(soilMoisture);
  Blynk.virtualWrite(V4, soilMoisture);  // Send to Blynk
  if (autoPumpMode) {
    if (soilMoisture > 800) {
      Serial.println(" - Dry");
      digitalWrite(DC_PIN, HIGH);  // Activate pump
      Blynk.virtualWrite(V1, 1);   // Update Blynk UI
    } else if (soilMoisture > 300) {
      Serial.println(" - Moist");
      digitalWrite(DC_PIN, LOW);  // Deactivate pump
      Blynk.virtualWrite(V1, 0);  // Update Blynk UI
    } else {
      Serial.println(" - Wet");
      digitalWrite(DC_PIN, LOW);  // Deactivate pump
      Blynk.virtualWrite(V1, 0);  // Update Blynk UI
    }
  }
}

// Read temperature and humidity from DHT sensor
void updateAirStatus() {
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Error reading DHT11 sensor!");
    return;
  }
  Blynk.virtualWrite(V2, temperature);
  Blynk.virtualWrite(V3, humidity);
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("°C, Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
  if (autoRoofMode) {
    if (humidity > 70 || temperature > 30) {  // Rain or hot -> Close roof
      if (!isRoofClosed) {
        Serial.println("Dong mai che");
        isRoofClosed = true;
        roofMove(0);
      }
    } else if (humidity < 60 && temperature < 30) {  // Cool and dry -> Open roof
      if (isRoofClosed) {
        Serial.println("Mo mai che");
        isRoofClosed = false;
        roofMove(1);
      }
    }
  }
}

void roofMove(bool direction) {
  if (!isServoLowRunning && !isServoHighRunning && autoRoofMode) {
    isServoLowRunning = true;
    isServoHighRunning = true;
    servoHighStartTime = millis();
    servoLowStartTime = millis();
    if (direction) {  // Open roof
      servoHigh.writeMicroseconds(1300);  // Unified PWM
      servoLow.writeMicroseconds(1700);
      Serial.println("Opening roof");
    } else {  // Close roof
      servoHigh.writeMicroseconds(1700);
      servoLow.writeMicroseconds(1300);
      Serial.println("Closing roof");
    }
  }
}

void updateRoofMovement() {
  if (isServoHighRunning && millis() - servoHighStartTime >= SERVO_HIGH_DURATION) {
    servoHigh.writeMicroseconds(1500);  // Stop
    isServoHighRunning = false;
    Serial.println("Servo tren da dung");
  }
  if (isServoLowRunning && millis() - servoLowStartTime >= SERVO_LOW_DURATION) {
    servoLow.writeMicroseconds(1500);  // Stop
    isServoLowRunning = false;
    Serial.println("Servo duoi da dung");
  }
}

// Test servo by sweeping (for debugging)
void servoTest() {
  servoHigh.writeMicroseconds(1300);  // Rotate CW
  servoLow.writeMicroseconds(1300);   // Rotate CW
  Serial.println("Servo Test: Rotating CW");
  delay(3000);
  servoHigh.writeMicroseconds(1700);  // Rotate CCW
  servoLow.writeMicroseconds(1700);   // Rotate CCW
  Serial.println("Servo Test: Rotating CCW");
  delay(3000);
  servoHigh.writeMicroseconds(1500);  // Stop
  servoLow.writeMicroseconds(1500);   // Stop
  Serial.println("Servo Test: Stopped");
  delay(3000);
}

// Periodic sensor update function
void updateSensors() {
  updateLedBasedOnLightSensor();
  updateSoilStatus();
  updateAirStatus();
}

// === Blynk Connected Callback ===
BLYNK_CONNECTED() {
  Serial.println("Connected to Blynk server");
  Blynk.syncVirtual(V0, V1, V5, V6, V7, V8, V9);  // Sync initial states
}