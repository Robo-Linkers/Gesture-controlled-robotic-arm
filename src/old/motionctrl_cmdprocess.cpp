#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_PCA9685.h>

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

// Pin Definitions for Sensors
#define FLEX_SENSOR_PIN A0
#define TOUCH_SENSOR_PIN 4
#define MPU_SDA 21
#define MPU_SCL 22
#define PCA9685_SDA 21
#define PCA9685_SCL 22

// PCA9685 Configuration
#define PCA9685_I2C_ADDRESS 0x40
#define SERVO_MIN 150 // Minimum pulse length count
#define SERVO_MAX 600 // Maximum pulse length count

// Gesture Codes
#define GESTURE_FIST 0x01
#define GESTURE_OPEN_HAND 0x02
#define GESTURE_LEFT 0x03
#define GESTURE_RIGHT 0x04

// Timing for non-blocking delay and idle state
unsigned long lastReadTime = 0;
unsigned long lastCommandTime = 0;
const unsigned long readInterval = 500; // 500 ms interval
const unsigned long idleTimeout = 10000; // 10 seconds

// Global Variables
int gestureCode = 0;
Adafruit_MPU6050 mpu;
Adafruit_PCA9685 pwm = Adafruit_PCA9685();

// Function Prototypes
void connectToWiFi(void);
void ensureWiFiConnection(void);
void connectToMQTT(void);
void ensureMQTTConnection(void);
int readFlexSensor(void);
void readMPU6050(int* orientationCode);
int debounceTouchSensor(int pin);
void combineGestureCodes(int flexCode, int orientationCode, int touchCode, int* gestureCode);
void sendGestureCode(int gestureCode);
void controlServoMotor(int channel, int angle);
void moveArm(int baseAngle, int shoulderAngle, int elbowAngle);
void disableUnusedComponents(void);
void debugSensors(int flexCode, int orientationCode, int touchCode);

// MQTT Callback Function
void mqttCallback(char* topic, byte* message, unsigned int length) {
    lastCommandTime = millis(); // Reset idle timer
    char command = message[0];
    if (command == GESTURE_FIST) {
        controlServoMotor(0, 90); // Grip
    } else if (command == GESTURE_OPEN_HAND) {
        controlServoMotor(0, 0); // Release
    } else if (command == GESTURE_LEFT) {
        moveArm(90, 45, 30); // Example movement
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize WiFi
    connectToWiFi();

    // Initialize MQTT
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(mqttCallback);
    connectToMQTT();

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip.");
        while (1);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Initialize PCA9685 Servo Driver
    pwm.begin();
    pwm.setPWMFreq(60); // Set frequency to 60 Hz
}

void loop() {
    ensureWiFiConnection();
    ensureMQTTConnection();

    unsigned long currentTime = millis();

    if (currentTime - lastCommandTime >= idleTimeout) {
        disableUnusedComponents();
    }

    if (currentTime - lastReadTime >= readInterval) {
        lastReadTime = currentTime;

        int flexCode = readFlexSensor();
        int orientationCode;
        readMPU6050(&orientationCode);
        int touchCode = debounceTouchSensor(TOUCH_SENSOR_PIN);

        combineGestureCodes(flexCode, orientationCode, touchCode, &gestureCode);
        debugSensors(flexCode, orientationCode, touchCode);

        sendGestureCode(gestureCode);
    }

    mqttClient.loop();
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
        if (mqttClient.connect("ESP32", mqttUser, mqttPassword)) {
            Serial.println("Connected to MQTT Broker.");
            mqttClient.subscribe("robotic_arm/commands");
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

int readFlexSensor() {
    int value = analogRead(FLEX_SENSOR_PIN);
    if (value < 500) {
        return GESTURE_FIST;
    } else {
        return GESTURE_OPEN_HAND;
    }
}

void readMPU6050(int* orientationCode) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    if (a.acceleration.x > 1.0) {
        *orientationCode = GESTURE_LEFT;
    } else if (a.acceleration.x < -1.0) {
        *orientationCode = GESTURE_RIGHT;
    } else {
        *orientationCode = 0x00; // No tilt
    }
}

int debounceTouchSensor(int pin) {
    int stableValue = digitalRead(pin);
    delay(50);
    if (digitalRead(pin) == stableValue) {
        return stableValue;
    }
    return LOW;
}

void combineGestureCodes(int flexCode, int orientationCode, int touchCode, int* gestureCode) {
    if (touchCode == HIGH) {
        if (flexCode == GESTURE_FIST) {
            *gestureCode = GESTURE_FIST;
        } else if (flexCode == GESTURE_OPEN_HAND) {
            *gestureCode = GESTURE_OPEN_HAND;
        }
    } else {
        *gestureCode = orientationCode;
    }
}

void sendGestureCode(int gestureCode) {
    char msg[10];
    snprintf(msg, sizeof(msg), "0x%02X", gestureCode);
    if (!mqttClient.publish("robotic_arm/gestures", msg)) {
        Serial.println("Failed to publish gesture code!");
    } else {
        Serial.print("Published gesture code: ");
        Serial.println(msg);
    }
}

void controlServoMotor(int channel, int angle) {
    int pulseWidth = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(channel, 0, pulseWidth);
}

void moveArm(int baseAngle, int shoulderAngle, int elbowAngle) {
    controlServoMotor(0, baseAngle);    // Base servo
    controlServoMotor(1, shoulderAngle); // Shoulder servo
    controlServoMotor(2, elbowAngle);   // Elbow servo
}

void disableUnusedComponents() {
    for (int channel = 0; channel < 16; channel++) {
        pwm.setPWM(channel, 0, 0); // Disable all servos
    }
}

void debugSensors(int flexCode, int orientationCode, int touchCode) {
    Serial.print("Flex Code: "); Serial.print(flexCode);
    Serial.print(", Orientation Code: "); Serial.print(orientationCode);
    Serial.print(", Touch Code: "); Serial.println(touchCode);
}