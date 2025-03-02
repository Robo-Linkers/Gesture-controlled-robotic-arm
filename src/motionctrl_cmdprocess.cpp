#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_PWMServoDriver.h>

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

// Pin Definitions
#define FLEX_SENSOR_PIN A0
#define TOUCH_SENSOR_PIN 4
#define MPU_SDA 21
#define MPU_SCL 22
#define PCA9685_SDA 21
#define PCA9685_SCL 22
#define WIFI_LED 2 // LED to indicate WiFi connection
#define MQTT_LED 5 // LED to indicate MQTT connection

// PCA9685 Configuration
#define PCA9685_I2C_ADDRESS 0x40
#define SERVO_MIN 150
#define SERVO_MAX 600

// Gesture Codes
#define GESTURE_FIST 0x01
#define GESTURE_OPEN_HAND 0x02
#define GESTURE_LEFT 0x03
#define GESTURE_RIGHT 0x04

// Timing
unsigned long lastReadTime = 0;
const unsigned long readInterval = 500;

// Movement Thresholds
#define WRIST_SPEED_THRESHOLD 3.0
#define BASE_MOVEMENT_THRESHOLD 1.5
#define ELBOW_MOVEMENT_THRESHOLD 3.0

// Global Variables
int gestureCode = 0;
Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Function Prototypes
void connectToWiFi();
void ensureWiFiConnection();
void connectToMQTT();
void ensureMQTTConnection();
int readFlexSensor();
void readMPU6050(float* accelX, float* accelY, float* accelZ);
int debounceTouchSensor(int pin);
void combineGestureCodes(int flexCode, float accelX, float accelY, float accelZ, int* gestureCode);
void sendGestureCode(int gestureCode);
void controlServoMotor(int channel, int angle);
void moveArm(float accelX, float accelY, float accelZ);
void updateLEDs();

void setup() {
    Serial.begin(115200);
    pinMode(WIFI_LED, OUTPUT);
    pinMode(MQTT_LED, OUTPUT);
    digitalWrite(WIFI_LED, LOW);
    digitalWrite(MQTT_LED, LOW);
    connectToWiFi();
    connectToMQTT();
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip.");
        while (1);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    pwm.begin();
    pwm.setPWMFreq(60);
}

void loop() {
    ensureWiFiConnection();
    ensureMQTTConnection();
    updateLEDs();
    unsigned long currentTime = millis();
    if (currentTime - lastReadTime >= readInterval) {
        lastReadTime = currentTime;
        int flexCode = readFlexSensor();
        float accelX, accelY, accelZ;
        readMPU6050(&accelX, &accelY, &accelZ);
        int touchCode = debounceTouchSensor(TOUCH_SENSOR_PIN);
        combineGestureCodes(flexCode, accelX, accelY, accelZ, &gestureCode);
        sendGestureCode(gestureCode);
        moveArm(accelX, accelY, accelZ);
    }
    mqttClient.loop();
}

void connectToWiFi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
}

void ensureWiFiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();
    }
}

void connectToMQTT() {
    while (!mqttClient.connected()) {
        if (mqttClient.connect("ESP32", mqttUser, mqttPassword)) {
            mqttClient.subscribe("robotic_arm/commands");
        } else {
            delay(2000);
        }
    }
}

void ensureMQTTConnection() {
    if (!mqttClient.connected()) {
        connectToMQTT();
    }
}

void updateLEDs() {
    digitalWrite(WIFI_LED, WiFi.status() == WL_CONNECTED ? HIGH : LOW);
    digitalWrite(MQTT_LED, mqttClient.connected() ? HIGH : LOW);
}

int readFlexSensor() {
    int value = analogRead(FLEX_SENSOR_PIN);
    return (value < 500) ? GESTURE_FIST : GESTURE_OPEN_HAND;
}

void readMPU6050(float* accelX, float* accelY, float* accelZ) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    *accelX = a.acceleration.x;
    *accelY = a.acceleration.y;
    *accelZ = a.acceleration.z;
}

int debounceTouchSensor(int pin) {
    int stableValue = digitalRead(pin);
    delay(50);
    return (digitalRead(pin) == stableValue) ? stableValue : LOW;
}

void combineGestureCodes(int flexCode, float accelX, float accelY, float accelZ, int* gestureCode) {
    if (flexCode == GESTURE_FIST && accelX > BASE_MOVEMENT_THRESHOLD) {
        *gestureCode = GESTURE_LEFT;
    } else if (flexCode == GESTURE_OPEN_HAND && accelX < -BASE_MOVEMENT_THRESHOLD) {
        *gestureCode = GESTURE_RIGHT;
    }
}

void sendGestureCode(int gestureCode) {
    char msg[10];
    snprintf(msg, sizeof(msg), "0x%02X", gestureCode);
    mqttClient.publish("robotic_arm/gestures", msg);
}

void controlServoMotor(int channel, int angle) {
    int pulseWidth = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(channel, 0, pulseWidth);
}

void moveArm(float accelX, float accelY, float accelZ) {
    if (fabs(accelX) > WRIST_SPEED_THRESHOLD) {
        return;
    }

    int baseAngle = map(accelX, -8, 8, 0, 180);
    controlServoMotor(0, baseAngle);

    if (fabs(accelX) > BASE_MOVEMENT_THRESHOLD) {
        int shoulderAngle = map(accelY, -8, 8, 0, 180);
        controlServoMotor(1, shoulderAngle);
    }

    if (fabs(accelX) > ELBOW_MOVEMENT_THRESHOLD) {
        int elbowAngle = map(accelZ, -8, 8, 0, 180);
        controlServoMotor(2, elbowAngle);
    }
}
