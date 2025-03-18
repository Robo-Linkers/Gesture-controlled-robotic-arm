#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050.h>

// Wi-Fi and MQTT Credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";
const char* mqttServer = "broker.emqx.io";
const int mqttPort = 1883;
const char* mqttUser = "Your_MQTT_Username";
const char* mqttPassword = "Your_MQTT_Password";

// MQTT client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// MPU6050 Sensors
MPU6050 mpuServo, mpuStepper;

// FSR Configuration
#define FSR1_PIN 34 // FSR for Gripper
#define FSR2_PIN 35 // Second FSR for additional grip check
int fsr1Value, fsr2Value;

// Kalman Filter Variables
float angleServoX, angleServoY, angleStepperX;
float biasServoX = 0, biasServoY = 0, biasStepperX = 0;
float P[2][2] = {{1, 0}, {0, 1}};
const float kalmanQ = 0.001, kalmanR = 0.03, dt = 0.05; // Smooth update delay

// Function Prototypes
void connectToWiFi();
void ensureWiFiConnection();
void connectToMQTT();
void ensureMQTTConnection();
void kalmanFilter(float &angle, float &bias, float newAngle, float newRate);
void sendAnglesToMQTT(float angleServoX, float angleServoY, float angleStepperX, bool gripActive);

// Delay for smoother updates
unsigned long previousMillis = 0;
const long interval = 200; // Adjusted update interval for smooth transition

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // Initialize MPU6050 sensors
    mpuServo.initialize();
    mpuStepper.initialize();
    
    if (!mpuServo.testConnection() || !mpuStepper.testConnection()) {
        Serial.println("MPU6050 initialization failed!");
        while (1);
    }
    
    mqttClient.setServer(mqttServer, mqttPort);
    connectToWiFi();
    connectToMQTT();
}

void loop() {
    ensureWiFiConnection();
    ensureMQTTConnection();

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Read MPU6050 data for Servos
        int16_t ax, ay, az, gx, gy, gz;
        mpuServo.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        float accelAngleX = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
        float accelAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
        float gyroRateX = gx / 131.0, gyroRateY = gy / 131.0;
        kalmanFilter(angleServoX, biasServoX, accelAngleX, gyroRateX);
        kalmanFilter(angleServoY, biasServoY, accelAngleY, gyroRateY);

        // Read MPU6050 data for Stepper
        int16_t ax2, ay2, az2, gx2, gy2, gz2;
        mpuStepper.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
        float accelStepperX = atan2(ay2, sqrt(ax2 * ax2 + az2 * az2)) * 180.0 / M_PI;
        float gyroStepperX = gx2 / 131.0;
        kalmanFilter(angleStepperX, biasStepperX, accelStepperX, gyroStepperX);

        // Read FSR sensor values
        fsr1Value = analogRead(FSR1_PIN);
        fsr2Value = analogRead(FSR2_PIN);
        bool gripActive = (fsr1Value > 500 || fsr2Value > 500);

        // Send data via MQTT
        sendAnglesToMQTT(angleServoX, angleServoY, angleStepperX, gripActive);
    }
}

void sendAnglesToMQTT(float angleServoX, float angleServoY, float angleStepperX, bool gripActive) {
    char msg[50];
    snprintf(msg, sizeof(msg), "SX:%.2f,SY:%.2f,ST:%.2f,GR:%d", angleServoX, angleServoY, angleStepperX, gripActive);
    mqttClient.publish("robot/angles", msg);
}

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
void ensureMQTTConnection() {
    if (!mqttClient.connected()) {
        connectToMQTT();
    }
}
void connectToMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("ESP32_DATA_ACQ", mqttUser, mqttPassword)) {
            Serial.println("Connected to MQTT Broker.");
        } else {
            Serial.print("Failed. State=");
            Serial.println(mqttClient.state());
            delay(2000);
        }
    }
}
void ensureWiFiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();
    }
}
void connectToWiFi() {
    Serial.print("Connecting to Wi-Fi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWi-Fi Connected.");
}