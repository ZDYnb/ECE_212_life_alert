#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

#define ACC_THRESHOLD 2.5 // Acceleration threshold (g)
#define GYRO_THRESHOLD 300 // Angular velocity threshold (°/s)
#define FALL_TIME_THRESHOLD 100 // 100ms false alarm filter

unsigned long fallStartTime = 0;
bool fallDetected = false;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    Serial.println("MPU6050 initialized successfully");
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;

    // Read acceleration & angular velocity data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 打印处理前的原始数据
  Serial.print("Raw Data -> ");
  Serial.print("AX: "); Serial.print(ax);
  Serial.print(" AY: "); Serial.print(ay);
  Serial.print(" AZ: "); Serial.print(az);
  Serial.print(" | GX: "); Serial.print(gx);
  Serial.print(" GY: "); Serial.print(gy);
  Serial.print(" GZ: "); Serial.println(gz);
  
    // Convert to actual physical units
    acc_x = ax / 16384.0; // 2g range
    acc_y = ay / 16384.0;
    acc_z = az / 16384.0;
    gyro_x = gx / 131.0; // 250°/s range
    gyro_y = gy / 131.0;
    gyro_z = gz / 131.0;

    // Calculate total acceleration (sqrt(ax^2 + ay^2 + az^2))
    float totalAcc = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);

    // Calculate the absolute value of angular velocity
    float totalGyro = sqrt(gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);

    Serial.print("Acc: "); Serial.print(totalAcc); Serial.print(" g, ");
    Serial.print("Gyro: "); Serial.print(totalGyro); Serial.println(" °/s");

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

    delay(100);
}
