#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Create the PWM driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo channels on PCA9685
#define BASE_SERVO 0
#define SHOULDER_SERVO 1
#define ELBOW_SERVO 2
#define WRIST_SERVO 3
#define SMOOTHING_FACTOR 0.5 


// Servo angle variables
int baseAngle = 90, shoulderAngle = 90, elbowAngle = 90, wristAngle = 90;

// WiFi and MQTT Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Function Prototypes
void updateServos();
void mqttCallback(char* topic, byte* message, unsigned int length);
void parseAndProcessData(String data);
int angleToPulse(int angle);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize PCA9685
    pwm.begin();
    pwm.setPWMFreq(50);  // Standard servo frequency (50Hz)

    // Connect to WiFi and MQTT
    WiFi.begin("WIFI-SSD", "WIFI-PASSWORD");
    mqttClient.setServer("broker.emqx.io", 1883);
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
}

void connectToMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("ESP32_SUB", "Your_MQTT_Username", "Your_MQTT_Password")) {
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
    Serial.print("Message received on topic: ");
    Serial.println(topic);

    // Serial.print("Raw Message: ");
    // for (unsigned int i = 0; i < length; i++) {
    //     Serial.print((char)message[i]);
    // }
    Serial.println();

    // Process message
    String data;
    for (unsigned int i = 0; i < length; i++) {
        data += (char)message[i];
    }
    
    // Serial.print("Parsed Data: ");
    // Serial.println(data);

    parseAndProcessData(data);
    updateServos();
}


void parseAndProcessData(String data) {
    float angleX = 0, angleY = 0;
    sscanf(data.c_str(), "X:%f,Y:%f", &angleX, &angleY);

    // Apply nonlinear scaling for more sensitivity
    float scaledX = pow(abs(angleX), 1.5) * (angleX < 0 ? -1 : 1);
    float scaledY = pow(abs(angleY), 1.5) * (angleY < 0 ? -1 : 1);

    // Map values to servo angles
    shoulderAngle = constrain(map(scaledX, -15, 15, 0, 180), 0, 180);
    elbowAngle = constrain(map(scaledY, -15, 15, 0, 180), 0, 180);

    // Base and wrist should also rotate based on angle changes
    baseAngle = constrain(map(scaledX, -15, 15, 45, 135), 0, 180);  // Allow slight turning range
    wristAngle = constrain(map(scaledY, -15, 15, 60, 120), 0, 180); // Smaller range for wrist

    // Debug prints
    Serial.print("Mapped Base: "); Serial.println(baseAngle);
    Serial.print("Mapped Shoulder: "); Serial.println(shoulderAngle);
    Serial.print("Mapped Elbow: "); Serial.println(elbowAngle);
    Serial.print("Mapped Wrist: "); Serial.println(wristAngle);

    updateServos();
}

void updateServos() {
    Serial.println("Updating servos...");

    Serial.print("Base: ");
    Serial.print(baseAngle);
    Serial.print(" -> Pulse: ");
    Serial.println(angleToPulse(baseAngle));

    Serial.print("Shoulder: ");
    Serial.print(shoulderAngle);
    Serial.print(" -> Pulse: ");
    Serial.println(angleToPulse(shoulderAngle));

    Serial.print("Elbow: ");
    Serial.print(elbowAngle);
    Serial.print(" -> Pulse: ");
    Serial.println(angleToPulse(elbowAngle));

    Serial.print("Wrist: ");
    Serial.print(wristAngle);
    Serial.print(" -> Pulse: ");
    Serial.println(angleToPulse(wristAngle));

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
