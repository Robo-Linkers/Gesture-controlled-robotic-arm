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

// MQTT Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// MPU6050
MPU6050 mpu;

// FSR Configuration
#define FSR_PIN A0 // Analog pin for FSR
int fsrValue;

// Kalman filter variables
float angleX, angleY, biasX = 0, biasY = 0;
float P[2][2] = {{1, 0}, {0, 1}};

// Constants
const float kalmanQ = 0.001, kalmanR = 0.03, dt = 0.01;

// Function Prototypes
void connectToWiFi();
void ensureWiFiConnection();
void connectToMQTT();
void ensureMQTTConnection();
void kalmanFilter(float &angle, float &bias, float newAngle, float newRate);
void sendAnglesToMQTT(float angleX, float angleY);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 initialization failed!");
        while (1);
    }
    mqttClient.setServer("broker.emqx.io", 1883);
    connectToWiFi();
    connectToMQTT();
}

void loop() {
    ensureWiFiConnection();
    ensureMQTTConnection();

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Read FSR value
    fsrValue = analogRead(FSR_PIN);
    Serial.print("FSR Value: ");
    Serial.println(fsrValue);

    // Convert raw data to angles
    float accelAngleX = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
    float accelAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
    float gyroRateX = gx / 131.0, gyroRateY = gy / 131.0;

    // Apply Kalman filter
    kalmanFilter(angleX, biasX, accelAngleX, gyroRateX);
    kalmanFilter(angleY, biasY, accelAngleY, gyroRateY);

    // Send angles to MQTT
    sendAnglesToMQTT(angleX, angleY);
    delay(10);
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

void ensureWiFiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();
    }
}

void connectToMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("ESP32_PUB", mqttUser, mqttPassword)) {
            Serial.println("Connected to MQTT Broker.");
        } else {
            Serial.print("Failed. State=");
            Serial.println(mqttClient.state());
            delay(2000);
        }
    }
}

void ensureMQTTConnection() {
    if (!mqttClient.connected()) {
        connectToMQTT();
    }
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

void sendAnglesToMQTT(float angleX, float angleY) {
    char msg[50];
    snprintf(msg, sizeof(msg), "X:%.2f,Y:%.2f", angleX, angleY);
    if (!mqttClient.publish("sensor/angles", msg)) {
        Serial.println("Failed to publish angles!");
    } else {
        Serial.print("Published angles: ");
        Serial.println(msg);
    }
}
