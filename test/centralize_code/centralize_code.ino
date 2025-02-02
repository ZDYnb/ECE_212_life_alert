#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// WiFi credentials – replace with your own.
const char* ssid     = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// Create a WebServer instance on port 80.
WebServer server(80);

// Create an instance of the MPU6050 object.
Adafruit_MPU6050 mpu;

//-------------------------
// Data Structures
//-------------------------
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

//-------------------------
// Function Prototypes
//-------------------------
void initMPU();
void printMPUConfig();
IMUData readIMU();
LocationData readLocation();
HealthData readHealth();
String packageData();
void outputData(const String &data);

void setup_for_motion_sensor();
void setup_for_heart_rate_sensor();
void setup_for_geolocation();
void setup_for_internet();
void setupWiFi();
void handleRoot();
void handleData();

//-------------------------
// Sensor Reading Functions
//-------------------------
IMUData readIMU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  IMUData imu;
  imu.ax = a.acceleration.x;
  imu.ay = a.acceleration.y;
  imu.az = a.acceleration.z;
  imu.gx = g.gyro.x;
  imu.gy = g.gyro.y;
  imu.gz = g.gyro.z;
  
  return imu;
}

LocationData readLocation() {
  // Simulated location data
  LocationData loc;
  loc.lat = 31.23;
  loc.lng = 121.47;
  return loc;
}

HealthData readHealth() {
  // Simulated health data
  HealthData health;
  health.heart_rate = 72;
  health.spo2 = 98;
  health.temp = 36.5;
  return health;
}

//-------------------------
// Data Packaging Function
//-------------------------
String packageData() {
  StaticJsonDocument<256> doc;
  
  // Add a timestamp (using millis() for now)
  doc["timestamp"] = millis();
  
  // Add location data
  LocationData loc = readLocation();
  JsonObject locObj = doc.createNestedObject("location");
  locObj["lat"] = loc.lat;
  locObj["lng"] = loc.lng;
  
  // Add IMU data (attitude information)
  IMUData imu = readIMU();
  JsonObject imuObj = doc.createNestedObject("imu");
  // Acceleration data in m/s^2
  JsonObject accelObj = imuObj.createNestedObject("acceleration");
  accelObj["x"] = imu.ax;
  accelObj["y"] = imu.ay;
  accelObj["z"] = imu.az;
  // Gyroscope data in rad/s
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

//-------------------------
// Data Output Function
//-------------------------
void outputData(const String &data) {
  Serial.println(data);
  // Future extension: send data over HTTP or WebSocket if desired.
}

//-------------------------
// MPU6050 Initialization Functions
//-------------------------
void initMPU() {
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  // Configure accelerometer range, gyro range, and filter bandwidth.
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void printMPUConfig() {
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("±2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("±4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("±8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("±16G");
      break;
  }
  
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("±250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("±500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("±1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("±2000 deg/s");
      break;
  }
  
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }
  
  Serial.println("");
}

//-------------------------
// Setup Functions for Different Parts
//-------------------------
void setup_for_motion_sensor() {
  initMPU();
  printMPUConfig();
}

void setup_for_heart_rate_sensor() {
  // Placeholder for future heart rate sensor initialization
}

void setup_for_geolocation() {
  // Placeholder for future geolocation sensor initialization
}

void setup_for_internet() {
  // Placeholder for future Internet-related setup (e.g., cloud connectivity)
}

//-------------------------
// WiFi Setup Function
//-------------------------
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

//-------------------------
// Web Server Handlers
//-------------------------
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>MPU6050 Sensor Data</title></head>"
                "<body><h1>MPU6050 Sensor Data Visualization</h1>"
                "<pre id='data'></pre>"
                "<script>"
                "function fetchData() {"
                "  fetch('/data')"
                "    .then(response => response.json())"
                "    .then(data => {"
                "      document.getElementById('data').textContent = JSON.stringify(data, null, 2);"
                "    })"
                "    .catch(error => console.error('Error:', error));"
                "}"
                "setInterval(fetchData, 1000);"
                "fetchData();"
                "</script></body></html>";
  server.send(200, "text/html", html);
}

void handleData() {
  String jsonData = packageData();
  server.send(200, "application/json", jsonData);
}

//-------------------------
// Setup Function
//-------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);  // Wait for Serial connection if needed (e.g., on Leonardo)
  }
  Wire.begin();
  
  // Initialize each module (for now, only motion sensor is real; others are placeholders)
  setup_for_motion_sensor();
  setup_for_heart_rate_sensor();
  setup_for_geolocation();
  setup_for_internet();
  
  // Connect to WiFi
  setupWiFi();
  
  // Configure web server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("Web server started");
  
  delay(100);
}

//-------------------------
// Main Loop
//-------------------------
void loop() {
  // Handle any incoming HTTP requests
  server.handleClient();
}
