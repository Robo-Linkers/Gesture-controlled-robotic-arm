#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050.h>

// Wi-Fi and MQTT Credentials
const char *ssid = "Your_SSID";
const char *password = "Your_PASSWORD";
const char *mqttServer = "broker.emqx.io";
const int mqttPort = 1883;
const char *mqttUser = "Your_MQTT_Username";
const char *mqttPassword = "Your_MQTT_Password";

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

/**
 * @brief Initializes the serial communication, I2C bus, and MPU6050 sensors.
 *        Connects to WiFi and MQTT broker.
 * @details
 * - Initializes serial communication with a baud rate of 115200.
 * - Initializes the I2C bus.
 * - Initializes two MPU6050 sensors and checks for a successful connection.
 *   If either sensor fails to connect, the function enters an infinite loop.
 * - Connects to WiFi with the provided ssid and password.
 * - Connects to the MQTT broker at the provided server and port.
 */
void setup()
{
    Serial.begin(115200);
    Wire.begin();

    // Initialize MPU6050 sensors
    mpuServo.initialize();
    mpuStepper.initialize();

    if (!mpuServo.testConnection() || !mpuStepper.testConnection())
    {
        Serial.println("MPU6050 initialization failed!");
        while (1)
            ;
    }

    mqttClient.setServer(mqttServer, mqttPort);
    connectToWiFi();
    connectToMQTT();
}

/**
 * @brief Main loop of the program.
 *        Ensures WiFi and MQTT connections are established and maintained.
 *        Reads data from the MPU6050 sensors and the FSR sensors.
 *        Applies Kalman filter to the data for smoother transitions.
 *        Sends the filtered data to the MQTT broker.
 * @details
 * - Ensures WiFi connection with the provided ssid and password.
 * - Ensures MQTT connection with the provided server and port.
 * - Reads data from the MPU6050 sensors for the servos and the stepper motor.
 * - Applies the Kalman filter to the data to reduce noise and jitter.
 * - Reads data from the FSR sensors to detect grip activity.
 * - Sends the filtered data to the MQTT broker.
 */
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
        float accelStepperY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
        float gyroRateX = gx / 131.0, gyroStepperY = gy / 131.0;
        kalmanFilter(angleServoX, biasServoX, accelAngleX, gyroRateX);
        kalmanFilter(angleStepperY, biasStepperY, accelStepperY, gyroStepperY);

        
        // Read MPU6050 data for Stepper
        int16_t ax2, ay2, az2, gx2, gy2, gz2;
        mpuStepper.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
        float accelAngleX2 = atan2(ay2, sqrt(ax2 * ax2 + az2 * az2)) * 180.0 / M_PI;
        float gyroRateX2 = gx2 / 131.0;
        kalmanFilter(angleServoX2, biasServoX2, accelAngleX2, gyroRateX2);


        // Read FSR sensor values
        fsr1Value = analogRead(FSR1_PIN);
        fsr2Value = analogRead(FSR2_PIN);
        bool gripActive = (fsr1Value > 500 || fsr2Value > 500);

        // Send data via MQTT
        sendAnglesToMQTT(angleServoX, angleServoX2, angleStepperY, gripActive);
    }
}

/**
 * @brief Publishes the angles of the robotic arm's servos and
 *        the stepper motor, as well as the grip status, to the
 *        MQTT broker.
 * @param[in] angleServoX Angle of the servo motor controlling
 *                        the base rotation in degrees.
 * @param[in] angleServoY Angle of the servo motor controlling
 *                        the shoulder rotation in degrees.
 * @param[in] angleStepperX Angle of the stepper motor controlling
 *                          the elbow rotation in degrees.
 * @param[in] gripActive true if the grip is active, false otherwise.
 */
void sendAnglesToMQTT(float angleServoX, float angleServoY, float angleStepperX, bool gripActive)
{
    char msg[50];
    snprintf(msg, sizeof(msg), "SX:%.2f,SY:%.2f,ST:%.2f,GR:%d", angleServoX, angleServoY, angleStepperX, gripActive);
    mqttClient.publish("robot/angles", msg);
}

/**
 * @brief Applies the Kalman filter to the given angle and rate.
 * @details
 * - Computes the rate of change of the angle.
 * - Updates the angle and bias using the rate of change.
 * - Updates the covariance matrix using the rate of change and
 *   the measurement noise.
 * - Computes the Kalman gain.
 * - Updates the angle and bias using the Kalman gain.
 * - Updates the covariance matrix using the Kalman gain.
 * @param[in,out] angle The angle to filter.
 * @param[in,out] bias The bias of the angle.
 * @param[in] newAngle The new angle measurement.
 * @param[in] newRate The new rate measurement.
 */
void kalmanFilter(float &angle, float &bias, float newAngle, float newRate)
{
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
 * @brief Ensures that the MQTT client is connected.
 * @details
 * - Checks if the MQTT client is currently connected to the broker.
 * - If not connected, attempts to establish a connection by calling connectToMQTT().
 */
void ensureMQTTConnection()
{
    if (!mqttClient.connected())
    {
        connectToMQTT();
    }
}

/**
 * @brief Establishes a connection to the MQTT broker.
 * @details
 * - Connects to the MQTT broker using the client ID, username, and password.
 * - If the connection fails, prints the error state and waits for 2 seconds before retrying.
 * - Repeats until the connection is established.
 */
void connectToMQTT()
{
    while (!mqttClient.connected())
    {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("ESP32_DATA_ACQ", mqttUser, mqttPassword))
        {
            Serial.println("Connected to MQTT Broker.");
        }
        else
        {
            Serial.print("Failed. State=");
            Serial.println(mqttClient.state());
            delay(2000);
        }
    }
}

/**
 * @brief Ensures that the Wi-Fi connection is established.
 * @details
 * - Checks if the Wi-Fi connection is currently established.
 * - If not connected, attempts to establish a connection by calling connectToWiFi().
 */
void ensureWiFiConnection()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        connectToWiFi();
    }
}

/**
 * @brief Establishes a connection to the Wi-Fi network.
 * @details
 * - Initiates connection to the Wi-Fi using the provided SSID and password.
 * - Continuously checks the connection status until connected, displaying
 *   progress via serial output.
 * - Prints a confirmation message upon successful connection.
 */
void connectToWiFi()
{
    Serial.print("Connecting to Wi-Fi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWi-Fi Connected.");
}
