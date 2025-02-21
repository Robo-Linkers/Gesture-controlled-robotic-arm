#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo baseServo, shoulderServo, elbowServo, wristServo;

//kalman filter variables
float angleX, angleY, biasX = 0, biasY = 0;
float P[2][2] = {{1, 0}, {0, 1}};

//servo angles
int baseAngle = 90, shoulderAngle = 90, elbowAngle = 90, wristAngle = 90;

//constants
const float kalmanQ = 0.001, kalmanR = 0.03, dt = 0.01;
const int movementThreshold = 2;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 failed!");
        while (1);
    }

    //attach servos to respective pins
    baseServo.attach(9);
    shoulderServo.attach(10);
    elbowServo.attach(11);
    wristServo.attach(12);

    updateServos();
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    
    //read sensor data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    //convert raw data to angles
    float accelAngleX = atan2(ay, sqrt(ax * ax + az * az)) * 180 / M_PI;
    float accelAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;
    
    float gyroRateX = gx / 131.0, gyroRateY = gy / 131.0;

    //apply Kalman filter
    kalmanFilter(angleX, biasX, accelAngleX, gyroRateX);
    kalmanFilter(angleY, biasY, accelAngleY, gyroRateY);

    //process gestures based on angles
    processGesture(angleX, angleY);

    //update servos with new angles
    updateServos();

    //debugging
    Serial.print("X: "); Serial.print(angleX);
    Serial.print(" | Y: "); Serial.println(angleY);

    delay(10);
}

//gesture-based movement
void processGesture(float angleX, float angleY) {
    if (angleX > 30) shoulderAngle = constrain(shoulderAngle + 2, 0, 180); //forward tilt
    else if (angleX < -30) shoulderAngle = constrain(shoulderAngle - 2, 0, 180); //backward tilt

    if (angleY > 30) elbowAngle = constrain(elbowAngle + 2, 0, 180); //right tilt
    else if (angleY < -30) elbowAngle = constrain(elbowAngle - 2, 0, 180); //left tilt

    if (angleX > 45 && angleY > 45) wristAngle = constrain(wristAngle + 5, 0, 180); //extreme tilt
}

//kalman filter function
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

//update servo positions
void updateServos() {
    baseServo.write(baseAngle);
    shoulderServo.write(shoulderAngle);
    elbowServo.write(elbowAngle);
    wristServo.write(wristAngle);
}
