/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com  
*********/

#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <time.h>
#include <Preferences.h>
// Configuration constants - ensuring proper null termination
const char* mqtt_user = "steve";     
const char* mqtt_pass = "Doctor*9";     
const int FIRMWARE_VERSION = 1; 
const char* CONFIG_URL = "https://raw.githubusercontent.com/stevennolte/ESP_Home/ESP_PowerMonitor/main/Release/firmware.json";
const char* ssid = "SSEI";
const char* password = "Nd14il!la";

// MQTT broker settings
const char* mqtt_hostname = "homeassistant.local:8123"; 
const char* mqtt_server = "192.168.1.19"; 
const int mqtt_port = 1883;
const char* client_id = "ESP_Panel_Power_Monitor\0";
const char* topic_currentA = "home/esp/panel/current_a\0";
const char* topic_currentB = "home/esp/panel/current_b\0";
const char* topic_currentA_daily = "home/esp/panel/current_a_daily\0";
const char* topic_currentB_daily = "home/esp/panel/current_b_daily\0";
const char* topic_uptime = "home/esp/panel/uptime\0";
const char* topic_cpu_temp = "home/esp/panel/cpu_temp\0";
const char* topic_reboot = "home/esp/panel/reboot\0";

// GPIO voltage monitoring configuration
const int GPIO_CHANNEL_A = 1; // GPIO1 for current sensor A
const int GPIO_CHANNEL_B = 2; // GPIO2 for current sensor B
const float CURRENT_MULTIPLIER = 30.0; // CT clamp ratio (adjust based on your CT clamp)
const float GPIO_VOLTAGE_RANGE = 3.3; // ESP32 GPIO voltage range
const int GPIO_RESOLUTION = 4095; // 12-bit ADC resolution

// AC sampling configuration
const int SAMPLES_PER_MEASUREMENT = 128; // Number of samples for RMS calculation
const unsigned long SAMPLE_INTERVAL_US = 520; // ~1920 Hz sampling rate (32 samples per 60Hz cycle)
const float AC_OFFSET = 1.65; // Assuming 3.3V/2 DC offset for AC signal

// Current monitoring variables
float currentA_instant = 0.0;
float currentB_instant = 0.0;
float currentA_daily = 0.0;
float currentB_daily = 0.0;
unsigned long lastReadingTime = 0;
unsigned long lastDayReset = 0;
const unsigned long readingInterval = 5000; // Read every 5 seconds (RMS calculation takes time)

// Uptime and system monitoring variables
unsigned long lastSystemPublish = 0;
const unsigned long systemPublishInterval = 1000; // Publish system info every 30 seconds




WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;

unsigned long lastUpdateCheck = 0;
const unsigned long updateInterval = 5 * 60 * 1000UL; // 5 minutes

#pragma region OTA Update
// --- OTA Update Functions ---
void performUpdate(const char* url) {
  Serial.printf("Starting update from: %s\n", url);
  
  HTTPClient http;
  
  // Add better error handling for the update URL
  if (!http.begin(url)) {
    Serial.println("HTTPClient begin failed for update!");
    return;
  }
  
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
  
  // Add debugging for the URL
  String url = String(CONFIG_URL) + "?t=" + String(millis());
  Serial.printf("Update URL: %s\n", url.c_str());
  
  HTTPClient http;
  
  // Try to begin connection with better error handling
  if (!http.begin(url)) {
    Serial.println("HTTPClient begin failed!");
    return;
  }
  
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
#pragma endregion

// --- Current Monitoring Functions ---
float readCurrentRMS(int gpio_pin) {
  float sum_squares = 0.0;
  unsigned long start_time = micros();
  
  // Take multiple samples to calculate RMS
  for (int i = 0; i < SAMPLES_PER_MEASUREMENT; i++) {
    int adc_value = analogRead(gpio_pin);
    float voltage = (adc_value * GPIO_VOLTAGE_RANGE) / GPIO_RESOLUTION;
    
    // Remove DC offset to get AC component
    float ac_voltage = voltage - AC_OFFSET;
    
    // Square the voltage for RMS calculation
    sum_squares += ac_voltage * ac_voltage;
    
    // Wait for next sample (maintain consistent sampling rate)
    while (micros() - start_time < (i + 1) * SAMPLE_INTERVAL_US) {
      // Busy wait for precise timing
    }
  }
  
  // Calculate RMS voltage
  float rms_voltage = sqrt(sum_squares / SAMPLES_PER_MEASUREMENT);
  
  // Convert RMS voltage to RMS current using CT clamp ratio
  float rms_current = rms_voltage * CURRENT_MULTIPLIER;
  
  return rms_current;
}

void readCurrents() {
  currentA_instant = readCurrentRMS(GPIO_CHANNEL_A); // Read RMS from GPIO1
  currentB_instant = readCurrentRMS(GPIO_CHANNEL_B); // Read RMS from GPIO2
  
  // Accumulate daily values (in Amp-hours)
  // Convert current (A) to Amp-hours by multiplying with time interval in hours
  float timeInterval = (millis() - lastReadingTime) / 3600000.0; // Convert ms to hours
  
  if (lastReadingTime > 0) { // Skip first reading
    currentA_daily += currentA_instant * timeInterval;
    currentB_daily += currentB_instant * timeInterval;
  }
  
  lastReadingTime = millis();
  
  // Reset daily accumulation at midnight (24 hours)
  if (millis() - lastDayReset > 24 * 60 * 60 * 1000UL) {
    currentA_daily = 0.0;
    currentB_daily = 0.0;
    lastDayReset = millis();
    Serial.println("Daily current accumulation reset");
  }
}

void publishCurrents() {
  char currentA_str[10];
  char currentB_str[10];
  char currentA_daily_str[10];
  char currentB_daily_str[10];
  
  // Convert floats to strings
  dtostrf(currentA_instant, 1, 2, currentA_str);
  dtostrf(currentB_instant, 1, 2, currentB_str);
  dtostrf(currentA_daily, 1, 3, currentA_daily_str);
  dtostrf(currentB_daily, 1, 3, currentB_daily_str);
  
  // Publish instant values
  client.publish(topic_currentA, currentA_str);
  client.publish(topic_currentB, currentB_str);
  
  // Publish daily accumulated values
  client.publish(topic_currentA_daily, currentA_daily_str);
  client.publish(topic_currentB_daily, currentB_daily_str);
  
  Serial.printf("Current A: %.2f A, Current B: %.2f A\n", currentA_instant, currentB_instant);
  Serial.printf("Daily A: %.3f Ah, Daily B: %.3f Ah\n", currentA_daily, currentB_daily);
}

// --- System Monitoring Functions ---
float getCPUTemperature() {
  // ESP32 internal temperature sensor (in Celsius)
  return temperatureRead();
}

String getUptime() {
  unsigned long uptimeMillis = millis();
  unsigned long seconds = uptimeMillis / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  unsigned long days = hours / 24;
  
  seconds %= 60;
  minutes %= 60;
  hours %= 24;
  
  char uptimeStr[50];
  sprintf(uptimeStr, "%lud %02lu:%02lu:%02lu", days, hours, minutes, seconds);
  return String(uptimeStr);
}

void publishSystemInfo() {
  // Get CPU temperature
  float cpuTemp = getCPUTemperature();
  char tempStr[10];
  dtostrf(cpuTemp, 1, 1, tempStr);
  
  // Get uptime
  String uptimeStr = getUptime();
  
  // Publish to MQTT
  client.publish(topic_cpu_temp, tempStr);
  client.publish(topic_uptime, uptimeStr.c_str());
  
  Serial.printf("CPU Temperature: %.1f°C, Uptime: %s\n", cpuTemp, uptimeStr.c_str());
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
  
  // Validate WiFi credentials
  if (strlen(ssid) == 0 || strlen(password) == 0) {
    Serial.println("ERROR: WiFi credentials are empty!");
    return;
  }
  
  Serial.printf("Connecting to WiFi: %s\n", ssid);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 60) { // Max 30 seconds
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.printf("WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("");
    Serial.println("Failed to connect to WiFi!");
  }
}

String resolveMQTTServer() {
  // Validate mqtt_server string before using it
  if (strlen(mqtt_server) == 0 || strlen(mqtt_server) > 255) {
    Serial.println("ERROR: mqtt_server string appears corrupted!");
    Serial.printf("mqtt_server length: %d\n", strlen(mqtt_server));
    Serial.printf("mqtt_server content: '%s'\n", mqtt_server);
    return String("192.168.1.19"); // Hardcoded fallback
  }
  
  // For now, just use the direct IP to avoid any DNS issues
  Serial.printf("Using direct IP: %s\n", mqtt_server);
  
  // Create a clean copy of the server string to ensure it's properly null-terminated
  String serverIP = String(mqtt_server);
  serverIP.trim(); // Remove any whitespace
  
  // Validate the IP format
  IPAddress testIP;
  if (testIP.fromString(serverIP)) {
    Serial.printf("Valid IP address: %s\n", serverIP.c_str());
    return serverIP;
  } else {
    Serial.printf("Invalid IP address: %s, using fallback\n", serverIP.c_str());
    return String("192.168.1.19");
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
  delay(5000); // Give serial time to initialize
  Serial.println("ESP Power Monitor starting...");
  
  // Clear any potential memory corruption
  Serial.printf("Free heap at start: %d bytes\n", ESP.getFreeHeap());
 
  // Initialize GPIO pins for analog reading
  Serial.println("Initializing GPIO pins for current monitoring...");
  // GPIO1 and GPIO2 are automatically configured for analog input
  Serial.printf("Using GPIO%d for Current A and GPIO%d for Current B\n", GPIO_CHANNEL_A, GPIO_CHANNEL_B);
  
  Serial.println("Starting WiFi connection...");
  setup_wifi();
  Serial.println("Connected to WiFi");
  Serial.printf("Free heap after WiFi: %d bytes\n", ESP.getFreeHeap());
  delay(500);
  Serial.println("Resolving MQTT server...");
  String mqttServerIP = resolveMQTTServer();
  Serial.printf("Using MQTT server: %s\n", mqttServerIP.c_str());
  delay(500);
  Serial.println("Setting up MQTT client...");
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  Serial.printf("Free heap after MQTT: %d bytes\n", ESP.getFreeHeap());

  // Initialize timing variables
  lastReadingTime = millis();
  lastDayReset = millis();

  // Temporarily disable automatic update check
  // checkForUpdates();
  lastUpdateCheck = millis();
  
  Serial.println("Setup complete!");
  Serial.printf("Final free heap: %d bytes\n", ESP.getFreeHeap());
}

void loop() {
  // Monitor memory every loop iteration to catch corruption early
  static unsigned long lastMemoryCheck = 0;
  if (millis() - lastMemoryCheck > 10000) { // Check every 10 seconds
    unsigned long freeHeap = ESP.getFreeHeap();
    if (freeHeap < 1024) { // Critical memory warning
      Serial.printf("WARNING: Low memory! Free heap: %lu bytes\n", freeHeap);
    } else if (millis() - lastMemoryCheck > 30000) { // Report every 30 seconds
      Serial.printf("Free heap: %lu bytes\n", freeHeap);
    }
    lastMemoryCheck = millis();
  }
  
  if (!client.connected()) {
    Serial.println("MQTT client not connected, attempting to reconnect...");
    reconnect();
  }
  client.loop();
  
  // Read and publish currents at the specified interval
  if (millis() - lastReadingTime >= readingInterval) {
    readCurrents();
    publishCurrents();
  }

  // Check for updates every 5 minutes (temporarily disabled)
  // if (millis() - lastUpdateCheck > updateInterval) {
  //   checkForUpdates();
  //   lastUpdateCheck = millis();
  // }

  // Publish system info every 30 seconds
  if (millis() - lastSystemPublish > systemPublishInterval) {
    publishSystemInfo();
    lastSystemPublish = millis();
  }

  delay(100); // Small delay for loop stability
}


