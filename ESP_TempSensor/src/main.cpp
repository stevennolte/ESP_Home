/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com  
*********/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
const char* mqtt_user = "steve";     // <-- Set your MQTT username
const char* mqtt_pass = "Doctor*9";     // <-- Set your MQTT password
// --- Configuration ---
const int FIRMWARE_VERSION = 13; // 1.3 becomes 13, 1.4 becomes 14, etc.
const char* CONFIG_URL = "https://raw.githubusercontent.com/stevennolte/ESP_Update_Test/main/Release/firmware.json"; // <-- Change this!
const char* ssid = "SSEI";
const char* password = "Nd14il!la";

// MQTT broker settings (local, e.g., Mosquitto or Home Assistant)
const char* mqtt_server = "192.168.1.19"; // <-- Change this!
const int mqtt_port = 1883;
const char* client_id = "ESP Temperature F";
const char* topic_temp = "home/esp/temperature_f";
const char* topic_reboot = "home/esp/reboot";

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;   
const int powerPin = 21;  

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastUpdateCheck = 0;
const unsigned long updateInterval = 5 * 60 * 1000UL; // 5 minutes

// --- OTA Update Functions ---
void performUpdate(const char* url) {
  HTTPClient http;
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("Failed to download binary, error: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return;
  }
  
  int contentLength = http.getSize();
  if (contentLength <= 0) {
    Serial.println("Content-Length header invalid.");
    http.end();
    return;
  }

  bool canBegin = Update.begin(contentLength);
  if (!canBegin) {
    Serial.println("Not enough space to begin OTA");
    http.end();
    return;
  }

  WiFiClient& stream = http.getStream();
  size_t written = Update.writeStream(stream);

  if (written != contentLength) {
    Serial.printf("Written only %d/%d bytes. Update failed.\n", written, contentLength);
    http.end();
    return;
  }
  
  if (!Update.end()) {
    Serial.println("Error occurred from Update.end(): " + String(Update.getError()));
    return;
  }

  Serial.println("Update successful! Rebooting...");
  ESP.restart();
}

void checkForUpdates() {
  Serial.println("Checking for updates...");
  HTTPClient http;
  String url = String(CONFIG_URL) + "?t=" + String(millis()); // cache buster
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("Failed to download config, error: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return;
  }

  String payload = http.getString();
  http.end();

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }

  int newVersion = doc["version"];
  const char* binaryUrl = doc["file"];
  
  Serial.printf("Current version: %d, Available version: %d\n", FIRMWARE_VERSION, newVersion);

  if (newVersion > FIRMWARE_VERSION) {
    Serial.println("New firmware available. Starting update...");
    performUpdate(binaryUrl);
  } else {
    Serial.println("No new update available.");
  }
}

// --- MQTT Functions ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (String(topic) == topic_reboot) {
    Serial.println("Reboot command received via MQTT!");
    ESP.restart();
  }
}

void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(client_id, mqtt_user, mqtt_pass)) { // <-- Use credentials
      client.subscribe(topic_reboot);
    } else {
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, HIGH); // Power on the DS18B20 sensor
  sensors.begin();

  setup_wifi();
  Serial.println("Connected to WiFi");

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  checkForUpdates();
  lastUpdateCheck = millis();
}

void loop() {
  Serial.print("startingloop");
  if (!client.connected()) {
    Serial.println("MQTT client not connected, attempting to reconnect...");
    reconnect();
  }
  Serial.println("MQTT client connected");
  client.loop();
  Serial.println("MQTT client looped");
  // Read and publish temperature
  digitalWrite(powerPin, HIGH); // Power on the DS18B20 sensor
  delay(750); // Wait for the sensor to stabilize
  sensors.requestTemperatures(); 
  float temperatureF = sensors.getTempFByIndex(0);

  Serial.print(temperatureF);
  Serial.println(" F");
  digitalWrite(powerPin, LOW); // Power off the DS18B20 sensor

  char tempString[8];
  dtostrf(temperatureF, 1, 2, tempString);
  client.publish(topic_temp, tempString);

  // Check for updates every 5 minutes
  if (millis() - lastUpdateCheck > updateInterval) {
    checkForUpdates();
    lastUpdateCheck = millis();
  }

  delay(5000);
}

