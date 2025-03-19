#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050.h>

//Wi-Fi and MQTT Credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";
const char* mqttServer = "broker.emqx.io";
const int mqttPort = 1883;
const char* mqttUser = "Your_MQTT_Username";
const char* mqttPassword = "Your_MQTT_Password";

//MQTT client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

//MPU6050
MPU6050 mpu;

//FSR Configuration
#define FSR_PIN A0 //analog pin for FSR
int fsrValue;

//kalman filter variables
float angleX, angleY, biasX = 0, biasY = 0;
float P[2][2] = {{1, 0}, {0, 1}};

//constants
const float kalmanQ = 0.001, kalmanR = 0.03, dt = 0.01;

//function Prototypes
void connectToWiFi();
void ensureWiFiConnection();
void connectToMQTT();
void ensureMQTTConnection();
void kalmanFilter(float &angle, float &bias, float newAngle, float newRate);
void sendAnglesToMQTT(float angleX, float angleY);

unsigned long previousMillis = 0;
const long interval = 3000; //interval for publishing data (in millisecs)

/**
 * @brief this function initializes the serial communication, I2C, MPU6050, Wi-Fi, and MQTT.
 * 
 */
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

/**
 * @brief the main loop to ensure connections, read sensors, apply Kalman filter, and send data. And adds delay by utilizing millis() function.
 * 
 */
void loop() {
    ensureWiFiConnection();
    ensureMQTTConnection();

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        //reads FSR value
        fsrValue = analogRead(FSR_PIN);
        Serial.print("FSR Value: ");
        Serial.println(fsrValue);

        //convert raw data to angles
        float accelAngleX = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
        float accelAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
        float gyroRateX = gx / 131.0, gyroRateY = gy / 131.0;

        //apply Kalman filter
        kalmanFilter(angleX, biasX, accelAngleX, gyroRateX);
        kalmanFilter(angleY, biasY, accelAngleY, gyroRateY);

        //send angles to MQTT
        sendAnglesToMQTT(angleX, angleY);
    }
}

/**
 * @brief connects to the specified Wi-Fi network.
 * 
 * this function attempts to connect to the Wi-Fi network using the provided SSID and password.
 * it waits until the connection is established and prints the connection status to the Serial monitor.
 * 
 */
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

/**
 * @brief ensures that the device is connected to Wi-Fi.
 * 
 * this function checks the Wi-Fi connection status and reconnects if the connection is lost.
 * 
 */
void ensureWiFiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();
    }
}

/**
 * @brief connects to the MQTT broker.
 * 
 * this function attempts to connect to the MQTT broker using the provided credentials.
 * it waits until the connection is established and prints the connection status to the Serial monitor.
 * 
 */
void connectToMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("ESP32", mqttUser, mqttPassword)) {
            Serial.println("Connected to MQTT Broker.");
        } else {
            Serial.print("Failed. State=");
            Serial.println(mqttClient.state());
            delay(2000);
        }
    }
}

/**
 * @brief ensures that the device is connected to the MQTT broker.
 * 
 * this function checks the MQTT connection status and reconnects if the connection is lost.
 * 
 */
void ensureMQTTConnection() {
    if (!mqttClient.connected()) {
        connectToMQTT();
    }
}

/**
 * @brief Kalman Filter concept for Sensor Fusion, how we use it here ? and why ?
 * 
 * the Kalman filter is an optimal recursive estimation algorithm that 
 * combines multiple noisy measurements to produce a more accurate 
 * estimate of an unknown variable. It is widely used in sensor fusion 
 * applications, such as combining accelerometer and gyroscope data 
 * for better angle estimation.
 * 
 * in our code (i.e) this data acquisition part, the Kalman filter is applied to the angle estimation 
 * of an MPU6050 sensor (IMU) by fusing:
 *   - accelerometer data (which provides an absolute reference for tilt)
 *   - gyroscope data (which provides the rate of change of the angle)
 * 
 * the filter consists of two main steps:
 * 1. prediction step:
 *    - the current state (angle) is predicted based on the previous state 
 *      and the gyroscope rate.
 *    - the uncertainty (error covariance) is updated.
 * 2. update (correction) step:
 *    - the accelerometer measurement is used to correct the predicted 
 *      angle, reducing the error.
 *    - a Kalman gain is computed to determine how much weight should be 
 *      given to the accelerometer reading versus the predicted angle.
 *    - the final angle estimate is updated by incorporating the new 
 *      measurement, and the error covariance is adjusted.
 * 
 * the Kalman filter helps reduce noise and drift from the gyroscope while 
 * compensating for the sudden changes and inaccuracies in the accelerometer 
 * readings. This results in a more stable and accurate angle estimation, 
 * which is then sent to an MQTT broker for further processing.
 * 
 */

/**
 * @brief applies the Kalman filter to the given angles.
 * 
 * @param angle Reference to the angle variable to be updated.
 * @param bias Reference to the bias variable to be updated.
 * @param newAngle The new angle measurement.
 * @param newRate The new rate measurement.
 * 
 * this function applies the Kalman filter algorithm to the given angles and updates the angle and bias variables.
 * 
 */

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

/**
 * @brief sends the calculated angles to the MQTT broker.
 * 
 * @param angleX The X-axis angle to be sent.
 * @param angleY The Y-axis angle to be sent.
 * 
 * this function formats the angles as a string and sends them to the MQTT broker. it prints the status of the publishing operation to the Serial monitor output.
 * 
 */
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
