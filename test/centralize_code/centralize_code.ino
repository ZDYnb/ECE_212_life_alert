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
#include <time.h>

const char* apiKey = "";
const char* googleApiUrl = "";

/*******************************************************
 *  WiFi and Firebase Settings
 *******************************************************/

#define WIFI_SSID ""
#define WIFI_PASSWORD ""
const char* FIREBASE_URL  = "";
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
float temperature      = 0.0f;

/*******************************************************
 * MPU6050
 *******************************************************/
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
float acc_ax, acc_ay, acc_az, gyro_gx, gyro_gy, gyro_gz;
float totalAcc;
float totalGyro;
unsigned long fallStartTime = 0;
bool fallDetected = false;

#define ACC_THRESHOLD 1.4    // Acceleration threshold (g)
#define GYRO_THRESHOLD 250   // Angular velocity threshold (°/s)
#define FALL_TIME_THRESHOLD 20// 100ms false alarm filter

/*******************************************************
 * GPS data (via TinyGPSPlus)
 *******************************************************/
#define RXD2 16 
#define TXD2 17  

float g_lat = 45.5180288;
float g_lng = -122.6801152;

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);  // Use UART2

/*******************************************************
 * Emergency Button on pin 12 (active LOW)
 *******************************************************/
const int buttonPin = 13;
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
void setupTime();
void initMAX30102();
void initMPU6050();

// Heart rate logic
void checkBeatAndComputeBPM(long irValue);

// Sensor reading
void readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz);
float readDieTemperature(); // read from MAX30102

// Build JSON & send to ThingsBoard
String createJson(float bpm, int avgBpm, bool emergency,
                  float lat, float lng,
                  float temperature,
                  float acc_ax, float acc_ay, float acc_az,
                  float gyro_gx, float gyro_gy, float gyro_gz,
                  float totalAcc, float totalGyro,
                  bool fallDetected);
void sendDataToFirebase(const String &jsonPayload);

void getGeolocation();
String getWiFiAccessPoints();

/*******************************************************
 * setup()
 *******************************************************/
void setup() {
  Serial.begin(115200);
  while (!Serial) { 
    delay(10);
  }
  
  // Connect to WiFi
  setupWiFi();

  // Set up for time
  setupTime();

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

  // Create the high-frequency heart-rate task (Core 0)
// 低频数据上传任务 (Core 0) - 主要运行 WiFi / Firebase
xTaskCreatePinnedToCore(uploadTask, "UploadTask", 10240, NULL, 1, &uploadTaskHandle, 0);

// 高频心率监测任务 (Core 1) - 避免 WiFi 干扰
xTaskCreatePinnedToCore(heartRateTask, "HeartRateTask", 3072, NULL, 2, &heartRateTaskHandle, 1);

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
void heartRateTask(void* pvParameters) {
  // 期待 10ms 间隔 (~100Hz)
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  TickType_t xLastWakeTime    = xTaskGetTickCount();

  // 用来记录两次循环之间的间隔
  static unsigned long lastLoopMillis = 0;

  while (true) {
    // 1) 等待固定时间 (10ms)
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // 2) 记录本次循环起始时间，用于测量整体循环的毫秒级间隔
    // unsigned long currentMillis = millis();
    // unsigned long loopDelta = currentMillis - lastLoopMillis;
    // lastLoopMillis = currentMillis;
    //Serial.printf("[heartRateTask] %lu ms since last loop\n", loopDelta);

    // 3) 开始分步骤测量微秒级时间
    // unsigned long stepStart = micros();

    // (a) Read IR and compute BPM
    long irValue = particleSensor.getIR();
    checkBeatAndComputeBPM(irValue);

    // unsigned long afterIR = micros();
    //Serial.printf("   IR & BPM processing: %lu us\n", afterIR - stepStart);

    // (b) Read IMU data
    readMPU(ax, ay, az, gx, gy, gz);

    // unsigned long afterIMU = micros();
    //Serial.printf("   IMU read: %lu us\n", afterIMU - stepStart);

  }
  vTaskDelete(NULL);
}
/*******************************************************
 * uploadTask (lower frequency ~1Hz)
 * Builds JSON, sends to ThingsBoard
 *******************************************************/
void uploadTask(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1 second
  TickType_t xLastWakeTime    = xTaskGetTickCount();
  int locationUpdateCounter = 0;  
  const int LOCATION_UPDATE_INTERVAL = 30; 
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);



    // // (B) Record the start time of this loop iteration
    // unsigned long loopStart = micros();


    // 1) Copy heart-rate data
    float currentBpm = g_beatsPerMinute;
    int currentAvg   = g_beatAvg;

    // 2) Check emergency
    bool localEmergency = g_emergencyTriggered;
    g_emergencyTriggered = false;
   
       // 4) Read temperature
    // unsigned long stepStart = micros();
    temperature = readDieTemperature();
    // unsigned long afterTemp = micros();
    // Serial.printf("[uploadTask]   Die temperature read: %lu us\n", afterTemp - stepStart);
    // stepStart = afterTemp;

    // (d) Read GPS if available
    if (gpsSerial.available() > 0) {
      char c = gpsSerial.read();
      gps.encode(c);
      if (gps.location.isUpdated()) {
        g_lat = gps.location.lat();
        g_lng = gps.location.lng();
      }
    }

    if (locationUpdateCounter >= LOCATION_UPDATE_INTERVAL) {
    getGeolocation();
    locationUpdateCounter = 0; 
    }
    locationUpdateCounter++;

    // unsigned long afterGPS = micros();
    // Serial.printf("   GPS check: %lu us\n", afterGPS - stepStart);
    // 3) Build JSON
    String jsonData = createJson(
      currentBpm, currentAvg,
      localEmergency,
      g_lat, g_lng,
      temperature,
      acc_ax, acc_ay, acc_az,
      gyro_gx, gyro_gy, gyro_gz,
      totalAcc, totalGyro,
      fallDetected
    );
    // unsigned long afterJSON = micros();
    // Serial.printf("              JSON build: %lu us\n", afterJSON - afterGPS);
    // 6) Send to Firebase
    sendDataToFirebase(jsonData);
    // unsigned long afterFirebase = micros();
    // Serial.printf("              Firebase send: %lu us\n", afterFirebase - stepStart);
    // 5) Debug Print
    Serial.println(jsonData);
    // Serial.println("--------------------------------------------------");

    //     // (H) Measure total loop time
    // unsigned long loopEnd = micros();
    // Serial.printf("[uploadTask] Total loop time: %lu us\n\n", (loopEnd - loopStart));
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

    g_beatsPerMinute = 60 / (delta / 1000.0);
    if (g_beatsPerMinute < 255 && g_beatsPerMinute > 20) {
      g_rates[g_rateSpot++] = (byte)g_beatsPerMinute;
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
    while (1) {
      delay(10);
    }
  }

  // Example configuration
  byte ledBrightness = 0x1F; // Options: 0=Off to 255=50mA
  byte sampleAverage = 8;    // Options: 1, 2, 4, 8, 16, 32
  byte ledMode       = 2;    // Options: 1=Red only, 2=Red+IR, 3=Red+IR+Green
  int sampleRate     = 3200;  // Options: 50,100,200,400,800,1000,1600,3200
  int pulseWidth     = 411;  // Options: 69,118,215,411
  int adcRange       = 4096; // Options: 2048,4096,8192,16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); 
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
  // Wire.begin(21,22); // If needed on some boards
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 initialized successfully");
}

/*******************************************************
 * readMPU() 
 * Reads accelerometer & gyro data
 *******************************************************/
void readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  acc_ax = ax / 16384.0;  // 2g range => 16384 LSB/g
  acc_ay = ay / 16384.0;
  acc_az = az / 16384.0;
  gyro_gx = gx / 131.0;    // 250°/s range => 131 LSB/(°/s)
  gyro_gy = gy / 131.0;
  gyro_gz = gz / 131.0;

  // Calculate total acceleration
  totalAcc = sqrt(acc_ax * acc_ax + acc_ay * acc_ay + acc_az * acc_az);
  // Calculate total angular velocity
  totalGyro = sqrt(gyro_gx * gyro_gx + gyro_gy * gyro_gy + gyro_gz * gyro_gz);

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
 * Reads from MAX30102's internal temperature sensor
 *******************************************************/
float readDieTemperature() {
  return particleSensor.readTemperature(); 
}

/*******************************************************
 * createJson()
 * Build a JSON payload with sensor data
 *******************************************************/
String createJson(float bpm, int avgBpm, bool emergency,
                  float lat, float lng,
                  float temperature,
                  float acc_ax, float acc_ay, float acc_az,
                  float gyro_gx, float gyro_gy, float gyro_gz,
                  float totalAcc, float totalGyro,
                  bool fallDetected)
{
  StaticJsonDocument<512> doc;

  struct tm timeinfo;
if (getLocalTime(&timeinfo)) {
    // 计算 Unix 时间戳
    time_t unixTimestamp = mktime(&timeinfo)-7 * 3600;

    // 存储 Unix 时间戳到 Firebase
    doc["timestamp"] = unixTimestamp;
} else {
    doc["timestamp"] = 0;  // 如果时间同步失败，设为 0
}

  doc["heart_rate"]  = bpm;
  doc["avg_bpm"]     = avgBpm;
  doc["emergency"]   = emergency;
  doc["temperature"] = temperature - 3.5;

  JsonObject locObj = doc.createNestedObject("location");
  locObj["lat"]     = lat;
  locObj["lng"]     = lng;

  JsonObject imuObj = doc.createNestedObject("imu");
  imuObj["acc_ax"]       = acc_ax;
  imuObj["acc_ay"]       = acc_ay;
  imuObj["acc_az"]       = acc_az;
  imuObj["gyro_gx"]      = gyro_gx;
  imuObj["gyro_gy"]      = gyro_gy;
  imuObj["gyro_gz"]      = gyro_gz;
  imuObj["totalAcc"]     = totalAcc;
  imuObj["totalGyro"]    = totalGyro;
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

/*******************************************************
 * setupTime()
 * Set timezone and sync via NTP
 *******************************************************/

void setupTime() {
  // 时区偏移量（秒）
  const long gmtOffset_sec = -8 * 3600;     // UTC-8 (冬季)
  const long daylightOffset_sec = 3600;     // 夏令时 +1 小时 (UTC-7)

  // 使用 NTP 服务器进行时间同步
  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");

  Serial.println("同步NTP时间...");
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nNTP 时间同步成功！");
}

/*******************************************************
 * setupgooglewifi()
 *******************************************************/
String getWiFiAccessPoints() {

    WiFi.disconnect();
    delay(500);
    int numNetworks = WiFi.scanNetworks(false, false, 5);
    WiFi.reconnect();
    
    if (numNetworks == 0) {
        Serial.println("⚠️ 未扫描到 WiFi 网络");
        return "{\"wifiAccessPoints\": []}";
    }

    String jsonPayload;
    jsonPayload.reserve(1024);

    jsonPayload = "{\"wifiAccessPoints\": [";
    for (int i = 0; i < numNetworks; i++) {
        jsonPayload += "{\"macAddress\": \"" + WiFi.BSSIDstr(i) + "\",";
        jsonPayload += "\"signalStrength\": " + String(WiFi.RSSI(i)) + "}";

        if (i < numNetworks - 1) {
            jsonPayload += ",";
        }
    }
    jsonPayload += "]}";

    return jsonPayload;
}

void getGeolocation() {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(googleApiUrl);
        http.addHeader("Content-Type", "application/json");

        String jsonPayload = getWiFiAccessPoints();
        int httpResponseCode = http.POST(jsonPayload);

        if (httpResponseCode > 0) {
            String response = http.getString();

            DynamicJsonDocument doc(1024);
            DeserializationError error = deserializeJson(doc, response);
            
            if (!error) {
                g_lat = doc["location"]["lat"];
                g_lng = doc["location"]["lng"];
            }
        }

        http.end();
    }
}

