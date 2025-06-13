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
#define LIGHT_SENSOR_PIN D0 // Light sensor
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
  timer.setInterval(500L, updateSensors);

  // Ensure initial states
  digitalWrite(DC_PIN, LOW);          // Pump off
  digitalWrite(LED_PIN, LOW);         // LED off
  servoHigh.writeMicroseconds(1500);  // Stop servoHigh
  servoLow.writeMicroseconds(1500);   // Stop servoLow
  Blynk.virtualWrite(V5, 0);          // Set default Slider V5 to Stop
  Blynk.virtualWrite(V8, 0);          // Set default Slider V8 to Stop
  roofMove(0); 
}

// === Main Loop ===
void loop() {
  Blynk.run();  // Handle Blynk communication
  timer.run();  // Run timer for sensor updates
  
  //digitalWrite(D5, !digitalRead(D0));
  //Serial.println(digitalRead(D0));
  //delay(500);
  

  updateRoofMovement();
}

// === Blynk Callbacks ===
// Control LED Manual On/Off (V0)
BLYNK_WRITE(V0) {
  if (!autoLightMode) {
    digitalWrite(LED_PIN, param.asInt());  // Manual control
    Serial.println(param.asInt() ? "LED ON (Manual)" : "LED OFF (Manual)");
  }
}

// Control Pump Manual On/Off (V1)
BLYNK_WRITE(V1) {
  if (!autoPumpMode) {
    digitalWrite(DC_PIN, param.asInt());  // Manual control
    Serial.println(param.asInt() ? "Pump ON (Manual)" : "Pump OFF (Manual)");
  }
}

// Control First Servo (V5) - Slider: 0=Stop, 1=Left (CCW), 2=Right (CW)
BLYNK_WRITE(V5) {
  int servoCommand = param.asInt();
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

// Control LED Auto/Manual Mode (V6)
BLYNK_WRITE(V6) {
  autoLightMode = !param.asInt();  // 0=Auto, 1=Manual
  Serial.println(autoLightMode ? "LED Auto Mode" : "LED Manual Mode");
  if (autoLightMode) {
    updateLedBasedOnLightSensor();  // Immediately apply auto mode
  } else {
    Blynk.virtualWrite(V0, digitalRead(LED_PIN));  // Sync manual state
  }
}

// Control Pump Auto/Manual Mode (V7)
BLYNK_WRITE(V7) {
  autoPumpMode = !param.asInt();  // 0=Auto, 1=Manual
  Serial.println(autoPumpMode ? "Pump Auto Mode" : "Pump Manual Mode");
  if (autoPumpMode) {
    updateSoilStatus();  // Immediately apply auto mode
  } else {
    Blynk.virtualWrite(V1, digitalRead(DC_PIN));  // Sync manual state
  }
}

// Control Second Servo (V8) - Slider: 0=Stop, 1=Left (CCW), 2=Right (CW)
BLYNK_WRITE(V8) {
  int servoCommand = param.asInt();
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

// === Sensor and Actuator Functions ===
// Update LED state based on light sensor (auto mode)
void updateLedBasedOnLightSensor() {
  if (autoLightMode) { // LIGHT_SENSOR_PIN = 0 -> sáng 
    bool lightState = digitalRead(LIGHT_SENSOR_PIN);
    digitalWrite(LED_PIN, lightState);   // Turn on LED if dark
    Blynk.virtualWrite(V0, lightState);  // Update Blynk UI
    Serial.print("Light sensor: "); 
    Serial.println(!lightState ? "Light" : "Dark");
  }
}

// Read and process soil moisture
void updateSoilStatus() {
  soilMoisture = analogRead(SOIL_SENSOR_PIN);
  Serial.print("Soil moisture: ");
  Serial.print(soilMoisture);
  Blynk.virtualWrite(V4, soilMoisture);  // Send to Blynk


  // Determine soil condition and control pump in auto mode
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

bool isRoofClosed = true;
bool isServoHighRunning = false;
bool isServoLowRunning = false;
unsigned long servoHighStartTime = 0;
unsigned long servoLowStartTime = 0;
unsigned long SERVO_HIGH_DURATION = 3000; // 3 giây
unsigned long SERVO_LOW_DURATION = 3000; // 3 giây

// Read temperature and humidity from DHT sensor
void updateAirStatus() {
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();


  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Error reading DHT11 sensor!");
    return;
  }

  // Send to Blynk
  Blynk.virtualWrite(V2, temperature);
  Blynk.virtualWrite(V3, humidity);

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("°C, Humidity: ");
  Serial.print(humidity);
  Serial.println("%");

  if (autoRoofMode) {
    //Logic servo dựa trên nhiệt độ và độ ẩm không khí
    if (humidity > 70 || temperature > 30) {  // Mưa hoặc nóng -> Đóng mái
      if (!isRoofClosed) {      
        Serial.println("Dong mai che");
        isRoofClosed = 1;
        roofMove(0); 
      }
    } else if (humidity < 60 && temperature < 30) {  // Thoáng mát -> Mở mái
      if (isRoofClosed) {
        Serial.println("Mo mai che");
        isRoofClosed = 0;
        roofMove(1);        
      }
    }
  }
}

void roofMove(bool direction) { 
  if (!isServoLowRunning && !isServoHighRunning) {
    isServoLowRunning = true;
    isServoHighRunning = true;
    servoHighStartTime = millis();
    servoLowStartTime = millis();
    if (direction) {
      servoHigh.writeMicroseconds(1200); // Xử lý từ dòng này trướcv
      servoLow.writeMicroseconds(1800);
    } else {
      servoHigh.writeMicroseconds(1800); // Đóng
      servoLow.writeMicroseconds(1200);
    }
  }
}

// Gọi hàm này thường xuyên trong loop() hoặc timer
void updateRoofMovement() {
  if (isServoHighRunning && millis() - servoHighStartTime >= SERVO_HIGH_DURATION) {
    servoHigh.writeMicroseconds(1500); // Dừng
    isServoHighRunning = false;
    Serial.println("Servo trên đã dừng");
  }
    if (isServoLowRunning && millis() - servoLowStartTime >= SERVO_LOW_DURATION) {
    servoLow.writeMicroseconds(1500); // Dừng
    isServoLowRunning = false;
    Serial.println("Servo dưới đã dừng");
  }
}

// Test servo by sweeping (for debugging)
void servoTTest() {
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
  Blynk.syncVirtual(V0, V1, V5, V6, V7, V8);  // Sync initial states
}