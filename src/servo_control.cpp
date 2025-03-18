#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <MPU6050.h>

// Create the PWM driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo channels on PCA9685
#define BASE_SERVO 0
#define SHOULDER_SERVO 1
#define ELBOW_SERVO 2
#define WRIST_SERVO 3  // Gripper control

// FSR Configuration
#define FSR_PIN A0 // Analog pin for FSR
int fsrValue;

// Servo angle variables
int baseAngle = 90, shoulderAngle = 90, elbowAngle = 90, wristAngle = 90;

const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";
const char* mqttServer = "broker.emqx.io";
const int mqttPort = 1883;
const char* mqttUser = "Your_MQTT_Username";
const char* mqttPassword = "Your_MQTT_Password";

// WiFi and MQTT Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Function Prototypes
void updateServos();
void mqttCallback(char* topic, byte* message, unsigned int length);
void parseAndProcessData(String data);
int angleToPulse(int angle);
void updateGripper();

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize PCA9685
    pwm.begin();
    pwm.setPWMFreq(50);  // Standard servo frequency (50Hz)

    pinMode(FSR_PIN, INPUT); // Initialize FSR sensor

    // Connect to WiFi and MQTT
    WiFi.begin(ssid, password);
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(mqttCallback);
    
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("WiFi connected!");

    connectToMQTT();
    updateServos();
}

void loop() {
    mqttClient.loop();
    updateGripper(); // Update gripper movement based on FSR sensor
}

void connectToMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("ESP32_SUB", mqttUser, mqttPassword)) {
            Serial.println("Connected to MQTT Broker.");
            mqttClient.subscribe("sensor/angles");
        } else {
            Serial.print("Failed. State=");
            Serial.println(mqttClient.state());
            delay(2000);
        }
    }
}

void mqttCallback(char* topic, byte* message, unsigned int length) {
    String data;
    for (unsigned int i = 0; i < length; i++) {
        data += (char)message[i];
    }
    parseAndProcessData(data);
    updateServos();
}

void parseAndProcessData(String data) {
    float angleX = 0, angleY = 0;
    sscanf(data.c_str(), "X:%f,Y:%f", &angleX, &angleY);
    
    // Process angles
    if (angleX > 30) shoulderAngle = constrain(shoulderAngle + 2, 0, 180);
    else if (angleX < -30) shoulderAngle = constrain(shoulderAngle - 2, 0, 180);
    
    if (angleY > 30) elbowAngle = constrain(elbowAngle + 2, 0, 180);
    else if (angleY < -30) elbowAngle = constrain(elbowAngle - 2, 0, 180);
}

void updateServos() {
    pwm.setPWM(BASE_SERVO, 0, angleToPulse(baseAngle));
    pwm.setPWM(SHOULDER_SERVO, 0, angleToPulse(shoulderAngle));
    pwm.setPWM(ELBOW_SERVO, 0, angleToPulse(elbowAngle));
    pwm.setPWM(WRIST_SERVO, 0, angleToPulse(wristAngle));
}

// Convert angle (0-180) to PWM pulse (servo range: 500-2500µs)
int angleToPulse(int angle) {
    int pulse = map(angle, 0, 180, 150, 600);  // 150-600 is a good range for most servos
    return pulse;
}

// New function to handle gripper based on FSR sensor
void updateGripper() {
    fsrValue = analogRead(FSR_PIN);  // Read FSR sensor value
    Serial.print("FSR Value: ");
    Serial.println(fsrValue);
    
    // Adjust gripper (wrist servo) based on force
    if (fsrValue > 500) {
        wristAngle = constrain(wristAngle - 5, 10, 180); // Close gripper
    } else {
        wristAngle = constrain(wristAngle + 5, 10, 180); // Open gripper
    }
    
    pwm.setPWM(WRIST_SERVO, 0, angleToPulse(wristAngle));
}
