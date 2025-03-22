#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <AccelStepper.h>

// PWM Servo Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo Channels
// #define BASE_SERVO 0
#define SHOULDER_SERVO 0
#define ELBOW_SERVO 1
#define WRIST_SERVO 2  

// Stepper Motor (Base Rotation)
#define STEP_PIN 26
#define DIR_PIN 27
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Servo Angles
int shoulderAngle = 90, elbowAngle = 90, wristAngle = 90;
bool gripperClosed = false;

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

// Function Prototypes
void setup();
void loop();
void updateServos();
void mqttCallback(char* topic, byte* message, unsigned int length);
void controlStepper(float angle);
int angleToPulse(int angle);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    pwm.begin();
    pwm.setPWMFreq(50);

    stepper.setMaxSpeed(200);
    stepper.setAcceleration(100);

    WiFi.begin(ssid, password);
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(mqttCallback);

    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("WiFi connected!");

    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("ESP32_ACTUATOR", mqttUser, mqttPassword)) {
            Serial.println("Connected to MQTT.");
            mqttClient.subscribe("robot/angles");
        } else {
            Serial.println("Failed. Retrying...");
            delay(2000);
        }
    }
    updateServos();
}

void loop() {
    mqttClient.loop();
    stepper.run();
}

void mqttCallback(char* topic, byte* message, unsigned int length) {
    String data;
    for (unsigned int i = 0; i < length; i++) {
        data += (char)message[i];
    }

    float angleServoX, angleServoY, angleStepperX;
    int grip;
    sscanf(data.c_str(), "SX:%f,SY:%f,ST:%f,GR:%d", &angleServoX, &angleServoY, &angleStepperX, &grip);

    shoulderAngle = map(angleServoX, -90, 90, 0, 180);
    elbowAngle = map(angleServoY, -90, 90, 0, 180);
    gripperClosed = grip;

    controlStepper(angleStepperX);
    updateServos();
}

void controlStepper(float angle) {
    stepper.moveTo(map(angle, -90, 90, -1000, 1000));
}

void updateServos() {
    pwm.setPWM(SHOULDER_SERVO, 0, angleToPulse(shoulderAngle));
    pwm.setPWM(ELBOW_SERVO, 0, angleToPulse(elbowAngle));
    pwm.setPWM(WRIST_SERVO, 0, angleToPulse(gripperClosed ? 0 : 180));
}

int angleToPulse(int angle) {
    int pulse = map(angle, 0, 180, 150, 600);  // 150-600 is a good range for most servos
    return pulse;
}
