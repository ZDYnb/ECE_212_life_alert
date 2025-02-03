#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// WiFi credentials – replace with your own.
const char* ssid     = "";
const char* password = "";

// Firebase Realtime Database URL – replace with your actual URL.
const char* firebaseURL = "https://lifealert-40baf-default-rtdb.firebaseio.com/sensorData.json";

// Create an instance of the MPU6050 object.
Adafruit_MPU6050 mpu;

// Data Structures
struct IMUData {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};

struct LocationData {
  float lat;
  float lng;
};

struct HealthData {
  int heart_rate;
  int spo2;
  float temp;
};

// Function prototypes
void initMPU();
IMUData readIMU();
LocationData readLocation();
HealthData readHealth();
String packageData();
void sendDataToFirebase(const String &data);
void setupWiFi();

// ✅ Fixed IMU reading function (Accelerometer and Gyroscope properly mapped)
IMUData readIMU() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  
  IMUData imu;
  imu.ax = accel.acceleration.x;  // ✅ Correct mapping
  imu.ay = accel.acceleration.y;
  imu.az = accel.acceleration.z;
  imu.gx = gyro.gyro.x;  // ✅ Correct mapping
  imu.gy = gyro.gyro.y;
  imu.gz = gyro.gyro.z;
  
  return imu;
}

// Simulated GPS data
LocationData readLocation() {
  LocationData loc;
  loc.lat = 31.23;
  loc.lng = 121.47;
  return loc;
}

// Simulated Health Data
HealthData readHealth() {
  HealthData health;
  health.heart_rate = 72;
  health.spo2 = 98;
  health.temp = 36.5;
  return health;
}

// ✅ Package Data into JSON
String packageData() {
  StaticJsonDocument<256> doc;
  
  // Add timestamp
  doc["timestamp"] = millis();
  
  // Add location data
  LocationData loc = readLocation();
  JsonObject locObj = doc.createNestedObject("location");
  locObj["lat"] = loc.lat;
  locObj["lng"] = loc.lng;
  
  // Add IMU data
  IMUData imu = readIMU();
  JsonObject imuObj = doc.createNestedObject("imu");
  JsonObject accelObj = imuObj.createNestedObject("acceleration");
  accelObj["x"] = imu.ax;
  accelObj["y"] = imu.ay;
  accelObj["z"] = imu.az;
  JsonObject gyroObj = imuObj.createNestedObject("gyro");
  gyroObj["x"] = imu.gx;
  gyroObj["y"] = imu.gy;
  gyroObj["z"] = imu.gz;
  
  // Add health data
  HealthData health = readHealth();
  JsonObject healthObj = doc.createNestedObject("health");
  healthObj["heart_rate"] = health.heart_rate;
  healthObj["spo2"] = health.spo2;
  healthObj["temp"] = health.temp;
  
  String output;
  serializeJson(doc, output);
  return output;
}

// ✅ Send Data to Firebase
void sendDataToFirebase(const String &data) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(firebaseURL);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(data);
    if (httpResponseCode > 0) {
      Serial.printf("Firebase POST response code: %d\n", httpResponseCode);
    } else {
      Serial.printf("Error in POST: %d\n", httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("WiFi not connected. Cannot send data to Firebase.");
  }
}

// ✅ Initialize MPU6050 Sensor
void initMPU() {
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

// ✅ Connect to WiFi
void setupWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Wire.begin();
  
  // Initialize MPU6050.
  initMPU();
  
  // Connect to WiFi.
  setupWiFi();
}

void loop() {
  String jsonData = packageData();
  Serial.println(jsonData);
  
  // Upload data to Firebase.
  sendDataToFirebase(jsonData);
  
  delay(1000); // Upload every 5 seconds; adjust as needed.
}

