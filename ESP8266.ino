#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Update these with your WiFi settings
#define WIFI_SSID "POCO-X3" // Should be changed based on the connected network & The bandwidth should be 2.4 GHz 
#define WIFI_PASSWORD "ajsea8055" // Same as above

// MQTT Broker settings
const char* mqtt_server = "192.168.79.254"; // IP Address of The Connected Network
const int mqtt_port = 1883;
const char* mqtt_topic_bme680 = "sensors/BME680";
const char* mqtt_topic_mq135 = "sensors/MQ135";
const char* mqtt_topic_flame = "sensors/Flame"; // New MQTT topic for flame detection
// Define the analog pin connected to the MQ135 sensor
const int MQ135_PIN = A0;
// Define the digital pin connected to the flame detection sensor
const int FLAME_SENSOR_PIN = D5; // Connect D0 pin of the sensor to D5 of ESP8266
// Define the load resistance of the MQ135 sensor (in ohms)
const float RL_MQ135 = 10000.0; // Example resistance value, adjust according to your sensor
Adafruit_BME680 bme; // I2C
#if defined(ESP32)
#include <WiFi.h>
WiFiClient espClient;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
WiFiClient espClient;
#define DEVICE "ESP8266"
#endif

PubSubClient mqttClient(espClient);

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  // Connect to MQTT broker
  mqttClient.setServer(mqtt_server, mqtt_port);
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (mqttClient.connect(DEVICE)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }

  // Initialize BME680 sensor
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // Set flame sensor pin as input
  pinMode(FLAME_SENSOR_PIN, INPUT);
}

void loop() {
  // Read sensor data
  #define SEALEVELPRESSURE_HPA (1013.25)
  float temperature = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0;
  float humidity = bme.readHumidity();
  float gasResistance = bme.readGas() / 1000.0;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  // Read the analog value from the MQ135 sensor
  int sensorValue = analogRead(MQ135_PIN);
  // Convert the analog value to voltage
  float voltage = sensorValue * (3.3 / 1023.0);
  // Calculate the resistance of the sensor using voltage divider formula
  float Rs = (3.3 * RL_MQ135) / voltage - RL_MQ135;
  // Use a pre-calibrated equation to estimate the air quality index (AQI)
  float ppm = pow(10, (1.2693 * log10(Rs) - 2.4042));

  // Read flame sensor value
  int flameValue = digitalRead(FLAME_SENSOR_PIN);

  // Display sensor data in Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("Gas Resistance: ");
  Serial.print(gasResistance);
  Serial.println(" kOhms");
  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" meters");
  Serial.print("Air Quality (ppm): ");
  Serial.println(ppm);
  Serial.print("Flame Detected: ");
  Serial.println(flameValue == LOW ? "Yes" : "No");
  Serial.println();

  // Publish sensor data to MQTT topics
  publishSensorData(temperature, pressure, humidity, gasResistance, altitude, ppm, flameValue);

  delay(2000); // Wait before next reading
}

void publishSensorData(float temperature, float pressure, float humidity, float gasResistance, float altitude, float ppm, int flameValue) {
  // Create JSON payload with BME680 sensor data
  StaticJsonDocument<200> jsonDocBME680;
  jsonDocBME680["temperature"] = temperature;
  jsonDocBME680["pressure"] = pressure;
  jsonDocBME680["humidity"] = humidity;
  jsonDocBME680["gasResistance"] = gasResistance;
  jsonDocBME680["altitude"] = altitude;
  jsonDocBME680["airQuality"] = ppm;

  // Serialize JSON to string
  char jsonStrBME680[200];
  serializeJson(jsonDocBME680, jsonStrBME680);

  // Publish BME680 sensor data to MQTT topic
  if (mqttClient.publish(mqtt_topic_bme680, jsonStrBME680)) {
    Serial.println("Published BME680 sensor data to MQTT topic");
  } else {
    Serial.println("Failed to publish BME680 sensor data to MQTT topic");
  }

  // Create JSON payload with MQ135 sensor data
  StaticJsonDocument<100> jsonDocMQ135;
  jsonDocMQ135["airQuality"] = ppm;

  // Serialize JSON to string
  char jsonStrMQ135[100];
  serializeJson(jsonDocMQ135, jsonStrMQ135);

  // Publish MQ135 sensor data to MQTT topic
  if (mqttClient.publish(mqtt_topic_mq135, jsonStrMQ135)) {
    Serial.println("Published MQ135 sensor data to MQTT topic");
  } else {
    Serial.println("Failed to publish MQ135 sensor data to MQTT topic");
  }

  // Create JSON payload with flame sensor data
  StaticJsonDocument<100> jsonDocFlame;
  jsonDocFlame["flameDetected"] = (flameValue == LOW) ? 1 : 0;

  // Serialize JSON to string
  char jsonStrFlame[100];
  serializeJson(jsonDocFlame, jsonStrFlame);

  // Publish flame sensor data to MQTT topic
  if (mqttClient.publish(mqtt_topic_flame, jsonStrFlame)) {
    Serial.println("Published Flame sensor data to MQTT topic");
  } else {
    Serial.println("Failed to publish Flame sensor data to MQTT topic");
  }
}
