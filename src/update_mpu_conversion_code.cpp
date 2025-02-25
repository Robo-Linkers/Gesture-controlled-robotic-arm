#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu;

// Kalman filter variables
float angleX = 0, angleY = 0, biasX = 0, biasY = 0;
float P[2][2] = {{1, 0}, {0, 1}};

// Constants
const float kalmanQ = 0.001, kalmanR = 0.03, dt = 0.01;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 initialization failed!");
        while (1);
    }
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    
    // Read sensor data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert raw data to angles
    float accelAngleX = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
    float accelAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
    
    float gyroRateX = gx / 131.0, gyroRateY = gy / 131.0;

    // Apply Kalman filter
    kalmanFilter(angleX, biasX, accelAngleX, gyroRateX);
    kalmanFilter(angleY, biasY, accelAngleY, gyroRateY);

    // Debugging - Print angles
    Serial.print("X: "); Serial.print(angleX);
    Serial.print(" | Y: "); Serial.println(angleY);

    delay(10);
}

// Kalman filter function
void kalmanFilter(float &angle, float &bias, float newAngle, float newRate) {
    float rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + kalmanQ);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += kalmanQ * dt;

    float S = P[0][0] + kalmanR;
    float K[2] = {P[0][0] / S, P[1][0] / S};

    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];
}
