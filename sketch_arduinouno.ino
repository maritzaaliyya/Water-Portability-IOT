#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"
#include "GravityTDS.h"
#include "DFRobot_PH.h"
#include <EEPROM.h>

// Network credentials from arduino_secrets.h
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

// MQTT broker details
const char broker[] = "test.mosquitto.org";
int port = 1883;
const char topic_tds[] = "sensor/tds";
const char topic_ph[] = "sensor/ph";

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// Sensor pin definitions
#define PH_PIN A1
#define TdsSensorPin A0

// Sensor objects and variables
GravityTDS gravityTds;
DFRobot_PH ph;
float tdsValue = 0, phValue = 0;
float temperature = 25.0;  // Default temperature for compensation

// Time interval for sending data (5 seconds)
const long interval = 5000;
unsigned long previousMillis = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Wait for the serial connection
  while (!Serial);

  // Initialize WiFi connection
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }
  Serial.println("\nConnected to the network.");

  // Connect to MQTT broker
  Serial.print("Connecting to MQTT broker: ");
  Serial.println(broker);
  while (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    delay(5000);
  }
  Serial.println("Connected to MQTT broker.");

  // Initialize TDS and pH sensors
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  // Reference voltage on ADC (5.0V for Arduino Uno)
  gravityTds.setAdcRange(1024);  // 10-bit ADC for Arduino Uno
  gravityTds.begin();

  ph.begin();
}

void loop() {
  // Call poll() to maintain the MQTT connection
  mqttClient.poll();

  // Get current time
  unsigned long currentMillis = millis();

  // Send data every 5 seconds
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read and update TDS value
    gravityTds.setTemperature(temperature);  // Set temperature compensation
    gravityTds.update();  // Calculate TDS value
    tdsValue = gravityTds.getTdsValue();

    // Read and update pH value
    float phVoltage = analogRead(PH_PIN) / 1024.0 * 5000;  // Convert analog reading to voltage
    phValue = ph.readPH(phVoltage, temperature);  // Calculate pH with temperature compensation

    // Print sensor data to serial
    Serial.print("TDS: ");
    Serial.print(tdsValue, 0);
    Serial.println(" ppm");
    Serial.print("pH: ");
    Serial.println(phValue, 2);

    // Publish TDS value to MQTT
    mqttClient.beginMessage(topic_tds);
    mqttClient.print(tdsValue);
    mqttClient.endMessage();

    // Publish pH value to MQTT
    mqttClient.beginMessage(topic_ph);
    mqttClient.print(phValue);
    mqttClient.endMessage();

    Serial.println("Data published to MQTT.");
  }
}
