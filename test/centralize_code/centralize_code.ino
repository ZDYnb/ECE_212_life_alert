/*******************************************************
 *  Includes
 *******************************************************/
#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h" // PBA algorithm from SparkFun
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
/*******************************************************
 *  WiFi and Firebase Settings
 *******************************************************/
const char* WIFI_SSID     = "ZDYNB";
const char* WIFI_PASSWORD = "20030911";
const char* FIREBASE_URL  = "https://lifealert-40baf-default-rtdb.firebaseio.com/sensorData.json";

/*******************************************************
 * MAX30102 Sensor
 *******************************************************/
MAX30105 particleSensor;

// Global heart-rate variables
float g_beatsPerMinute = 0.0;
int   g_beatAvg        = 0;
byte  g_rates[4];
byte  g_rateSpot       = 0;
long  g_lastBeat       = 0;
float temperature = 0.0f;
/*******************************************************
 * MPU6050
 *******************************************************/
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
float totalAcc;
float totalGyro;
unsigned long fallStartTime = 0;
bool fallDetected = false;

#define ACC_THRESHOLD 2.5 // Acceleration threshold (g)
#define GYRO_THRESHOLD 300 // Angular velocity threshold (°/s)
#define FALL_TIME_THRESHOLD 100 // 100ms false alarm filter


/*******************************************************
 * Dummy GPS data
 *******************************************************/
#define RXD2 16 
#define TXD2 17  

float g_lat = 45.58;
float g_lng = -122.68;

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);  // Use UART2

/*******************************************************
 * Emergency Button on pin 12 (active LOW)
 *******************************************************/
const int buttonPin = 12;
volatile bool g_emergencyTriggered = false;
void IRAM_ATTR handleEmergency() {
  g_emergencyTriggered = true;
}

/*******************************************************
 * FreeRTOS Task Handles & Mutex
 *******************************************************/
TaskHandle_t heartRateTaskHandle;
TaskHandle_t uploadTaskHandle;
SemaphoreHandle_t i2cMutex;

/*******************************************************
 * Function Declarations
 *******************************************************/

// FreeRTOS tasks
void heartRateTask(void* pvParameters);
void uploadTask(void* pvParameters);

// Setup
void setupWiFi();
void initMAX30102();
void initMPU6050();

// Heart rate logic
void checkBeatAndComputeBPM(long irValue);

// Sensor reading
void readMPU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz);
float readDieTemperature(); // read from MAX30102

// Firebase uploading
String createJson(float bpm, int avgBpm, bool emergency,
                  float lat, float lng,
                  float temperature,
                  float ax, float ay, float az,
                  float gx, float gy, float gz);
void sendDataToFirebase(const String &jsonPayload);

/*******************************************************
 * setup()
 *******************************************************/
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  // Connect to WiFi
  setupWiFi();

  // GPS module baud rate
  gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2); 
  // Create a mutex for the I2C bus
  i2cMutex = xSemaphoreCreateMutex();

  // Initialize MAX30102
  initMAX30102();

  // Initialize MPU6050
  initMPU6050();

  // Emergency button
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleEmergency, FALLING);

  // Create the high-frequency heart-rate task
  xTaskCreatePinnedToCore(
    heartRateTask,        
    "HeartRateTask",      
    4096,                 
    NULL,
    2,    // priority
    &heartRateTaskHandle,
    0     // pinned to core 0
  );

  // Create the lower-frequency upload task
  xTaskCreatePinnedToCore(
    uploadTask,
    "UploadTask",
    4096,
    NULL,
    1,    // lower priority
    &uploadTaskHandle,
    1     // pinned to core 1
  );
}

/*******************************************************
 * loop() - no code needed; tasks run in background
 *******************************************************/
void loop() {
  // No code here.
}

/*******************************************************
 * heartRateTask (high frequency ~100Hz)
 * Reads IR from MAX30102, calculates BPM using PBA
 *******************************************************/
const unsigned long imuTempInterval = 1000;
static unsigned long lastImuTempTime = 0; 
void heartRateTask(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms -> ~100Hz
  TickType_t xLastWakeTime    = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    long irValue = particleSensor.getIR();
    checkBeatAndComputeBPM(irValue);

    readMPU(ax, ay, az, gx, gy, gz);

    temperature = readDieTemperature();

  if (gpsSerial.available() > 0) {
      char c = gpsSerial.read();
      gps.encode(c); 

      if (gps.location.isUpdated()) {
          g_lat = gps.location.lat();
          g_lng = gps.location.lng();
      }
  }
  }

  vTaskDelete(NULL);
}

/*******************************************************
 * uploadTask (lower frequency ~1Hz)
 * Reads IMU, reads temperature, checks emergency, 
 * builds JSON, sends to Firebase
 *******************************************************/
void uploadTask(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1 second
  TickType_t xLastWakeTime    = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    // 3) Copy heart-rate data
    float currentBpm = g_beatsPerMinute;
    int currentAvg   = g_beatAvg;

    // 4) Check emergency
    bool localEmergency = g_emergencyTriggered;
    g_emergencyTriggered = false;

    // 5) Build JSON
    String jsonData = createJson(currentBpm, currentAvg,
                                 localEmergency,
                                 g_lat, g_lng,
                                 temperature,
                                 ax, ay, az,
                                 gx, gy, gz, totalAcc, totalGyro, fallDetected);

    // 6) Send to Firebase
    sendDataToFirebase(jsonData);

    // 7) Debug Print
    Serial.println(jsonData);
  }
  vTaskDelete(NULL);
}

/*******************************************************
 * checkBeatAndComputeBPM()
 * from heartRate.h (SparkFun PBA)
 *******************************************************/
void checkBeatAndComputeBPM(long irValue) {
  if (checkForBeat(irValue) == true) {
    long delta = millis() - g_lastBeat;
    g_lastBeat = millis();

    float bpm = 60.0 / (delta / 1000.0);
    if (bpm < 255 && bpm > 20) {
      g_beatsPerMinute = bpm;
      g_rates[g_rateSpot++] = (byte)bpm;
      g_rateSpot %= 4;

      int total = 0;
      for (int i = 0; i < 4; i++) {
        total += g_rates[i];
      }
      g_beatAvg = total / 4;
    }
  }
}

/*******************************************************
 * initMAX30102()
 *******************************************************/
void initMAX30102() {
  Serial.println("Initializing MAX30102...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Check wiring!");
    while (1) { delay(10); }
  }

  // Example configuration
  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  particleSensor.setPulseAmplitudeRed(0x0A); 
  particleSensor.setPulseAmplitudeGreen(0);  
  particleSensor.enableDIETEMPRDY(); // Enable internal temperature

  Serial.println("MAX30102 ready. Place your finger on the sensor.");
}

/*******************************************************
 * initMPU6050()
 *******************************************************/
void initMPU6050() {
  Serial.println("Initializing MPU6050...");
  
  // If you have not called Wire.begin(...) above, you'd do it here:
  // Wire.begin(21,22); // or default pins if your board needs it
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    Serial.println("MPU6050 initialized successfully");
}

/*******************************************************
 * readMPU() 
 * reads accelerometer & gyro data
 *******************************************************/
void readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  ax = ax / 16384.0; // 2g range
  ay = ay / 16384.0;
  az = az / 16384.0;
  gx = gx / 131.0; // 250°/s range
  gy = gy / 131.0;
  gz = gz / 131.0;
  // Calculate total acceleration (sqrt(ax^2 + ay^2 + az^2))
  totalAcc = sqrt(ax * ax + ay * ay + az * az);
  // Calculate the absolute value of angular velocity
  totalGyro = sqrt(gx * gx + gy * gy + gz * gz);
// Detect a fall
  if (totalAcc > ACC_THRESHOLD && totalGyro > GYRO_THRESHOLD) {
    if (!fallDetected) {
        fallStartTime = millis(); // Record the fall time
        fallDetected = true;
    } else if (millis() - fallStartTime > FALL_TIME_THRESHOLD) {
        Serial.println("⚠️ Fall detected!");
    }
  } else {
    fallDetected = false;
  }

}

/*******************************************************
 * readDieTemperature() 
 * Now actually read from MAX30102 instead of returning 25.0
 *******************************************************/
float readDieTemperature() {
  return particleSensor.readTemperature(); // real sensor reading
  // If sensor is not working or you didn't enable dietemp:
  // you might get random or 0 values
}

/*******************************************************
 * createJson()
 * Same format as before
 *******************************************************/
String createJson(float bpm, int avgBpm, bool emergency,
                  float lat, float lng,
                  float temperature,
                  int16_t ax, int16_t ay, int16_t az,
                  int16_t gx, int16_t gy, int16_t gz, float totalAcc, float totalGyro, bool fallDetected)
{
  StaticJsonDocument<512> doc;
  doc["timestamp"]   = millis();
  doc["heart_rate"]  = bpm;
  doc["avg_bpm"]     = avgBpm;
  doc["emergency"]   = emergency;
  doc["temperature"] = temperature;

  JsonObject locObj   = doc.createNestedObject("location");
  locObj["lat"]       = lat;
  locObj["lng"]       = lng;

  JsonObject imuObj   = doc.createNestedObject("imu");
  imuObj["ax"]        = ax;
  imuObj["ay"]        = ay;
  imuObj["az"]        = az;
  imuObj["gx"]        = gx;
  imuObj["gy"]        = gy;
  imuObj["gz"]        = gz;
  imuObj["totalAcc"]  = totalAcc;
  imuObj["totalGyro"] = totalGyro;
  imuObj["fallDetected"] = fallDetected;


  String output;
  serializeJson(doc, output);
  return output;
}

/*******************************************************
 * sendDataToFirebase()
 *******************************************************/
void sendDataToFirebase(const String &jsonPayload) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(FIREBASE_URL);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(jsonPayload);
    if (httpResponseCode > 0) {
      Serial.printf("Firebase POST response code: %d\n", httpResponseCode);
    } else {
      Serial.printf("Error in POST: %d\n", httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("WiFi not connected. Cannot send data.");
  }
}

/*******************************************************
 * setupWiFi()
 *******************************************************/
void setupWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}