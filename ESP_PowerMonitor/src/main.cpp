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
#include <math.h>

// NTP Time Configuration
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -5 * 3600;  // EST (UTC-5), adjust for your timezone
const int daylightOffset_sec = 3600;   // Daylight saving time offset (1 hour)
// Configuration constants - ensuring proper null termination
const char* mqtt_user = "steve";     
const char* mqtt_pass = "Doctor*9";     
const int FIRMWARE_VERSION = 4; 
const char* CONFIG_URL = "https://raw.githubusercontent.com/stevennolte/ESP_Home/main/ESP_PowerMonitor/Release/firmware.json";
const char* ssid = "SSEI";
const char* password = "Nd14il!la";

// MQTT broker settings
const char* mqtt_hostname = "homeassistant.local:8123"; 
const char* mqtt_server = "192.168.1.19"; 
const int mqtt_port = 1883;
const char* client_id = "ESP_Panel_Power_Monitor\0";
const char* topic_currentA = "home/esp/panel/current_a\0";
const char* topic_currentB = "home/esp/panel/current_b\0";
const char* topic_currentCombined = "home/esp/panel/current_combined\0";
const char* topic_currentA_daily = "home/esp/panel/current_a_daily\0";
const char* topic_currentB_daily = "home/esp/panel/current_b_daily\0";
const char* topic_currentCombined_daily = "home/esp/panel/current_combined_daily\0";
const char* topic_voltageA = "home/esp/panel/voltage_a\0";
const char* topic_voltageB = "home/esp/panel/voltage_b\0";
const char* topic_uptime = "home/esp/panel/uptime\0";
const char* topic_cpu_temp = "home/esp/panel/cpu_temp\0";
const char* topic_firmware_version = "home/esp/panel/firmware_version\0";
const char* topic_reboot = "home/esp/panel/reboot\0";

// GPIO voltage monitoring configuration
const int GPIO_CHANNEL_A = A0; // Use A0 (GPIO36) for current sensor A
const int GPIO_CHANNEL_B = A1; // Use A1 (GPIO39) for current sensor B
// CT Clamp Configuration - SCT-013-030 (30A/1V)
// The CT clamp outputs a sine wave centered at 1.65V (AC_OFFSET)
// The amplitude of the sine wave is proportional to the AC current flowing through the primary
// SCT-013-030 specifications: 30A primary = 1V RMS output
const float CT_CLAMP_MAX_CURRENT = 30.0; // Maximum rated current in Amps
const float CT_CLAMP_MAX_VOLTAGE = 1.0;  // Output voltage at max current (1V RMS)
const float VOLTAGE_TO_CURRENT_SCALE = CT_CLAMP_MAX_CURRENT / CT_CLAMP_MAX_VOLTAGE; // 30 A/V
const float GPIO_VOLTAGE_RANGE = 3.3; // ESP32 GPIO voltage range
const int GPIO_RESOLUTION = 4095; // 12-bit ADC resolution

// AC sampling configuration
const int SAMPLES_PER_MEASUREMENT = 128; // Number of samples for RMS calculation
const unsigned long SAMPLE_INTERVAL_US = 520; // ~1920 Hz sampling rate (32 samples per 60Hz cycle)
const float AC_OFFSET = 1.65; // Sine wave centered at 1.65V

// Current monitoring variables
float currentA_instant = 0.0;
float currentB_instant = 0.0;
float currentCombined_instant = 0.0;
float currentA_daily = 0.0;
float currentB_daily = 0.0;
float currentCombined_daily = 0.0;
float voltageA_rms = 0.0;
float voltageB_rms = 0.0;
float dcOffsetA = 0.0;
float dcOffsetB = 0.0;
unsigned long lastReadingTime = 0;
unsigned long lastDayReset = 0;
int lastResetDay = -1; // Track which day we last reset on
const unsigned long readingInterval = 1000; // Read every 1 second

// Uptime and system monitoring variables
unsigned long lastSystemPublish = 0;
const unsigned long systemPublishInterval = 1000; // Publish system info every 30 seconds




WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;

unsigned long lastUpdateCheck = 0;
const unsigned long updateInterval = 5 * 60 * 1000UL; // 5 minutes

// Function declarations
void saveDailyTotals();
void loadDailyTotals();
void resetDailyTotals();
void checkMidnightReset();
void printLocalTime();

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

// --- Time and Date Functions ---
void setupTime() {
  Serial.println("Setting up time synchronization...");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  // Wait for time to be set
  int attempts = 0;
  while (!time(nullptr) && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (time(nullptr)) {
    Serial.println("\nTime synchronized successfully!");
    printLocalTime();
  } else {
    Serial.println("\nFailed to synchronize time, will use 24-hour reset fallback");
  }
}

void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  
  char timeStr[50];
  strftime(timeStr, sizeof(timeStr), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  Serial.printf("Current time: %s\n", timeStr);
}

void checkMidnightReset() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    // Fallback to 24-hour timer if NTP fails
    if (millis() - lastDayReset > 24 * 60 * 60 * 1000UL) {
      resetDailyTotals();
    }
    return;
  }
  
  // Check if we've crossed into a new day
  int currentDay = timeinfo.tm_yday; // Day of year (0-365)
  
  if (lastResetDay == -1) {
    // First time setup - just record the current day
    lastResetDay = currentDay;
    // Save the initial day to storage
    preferences.begin("power_monitor", false);
    preferences.putInt("lastResetDay", lastResetDay);
    preferences.end();
  } else if (currentDay != lastResetDay) {
    // New day detected - reset daily totals
    resetDailyTotals();
    lastResetDay = currentDay;
    
    char timeStr[50];
    strftime(timeStr, sizeof(timeStr), "%A, %B %d %Y %H:%M:%S", &timeinfo);
    Serial.printf("New day detected at: %s\n", timeStr);
  }
}

void resetDailyTotals() {
  currentA_daily = 0.0;
  currentB_daily = 0.0;
  currentCombined_daily = 0.0;
  lastDayReset = millis();
  
  // Save reset values to persistent storage
  saveDailyTotals();
  
  Serial.println("Daily current accumulation reset at midnight");
}

// --- Persistent Storage Functions ---
void saveDailyTotals() {
  preferences.begin("power_monitor", false);
  preferences.putFloat("currentA_daily", currentA_daily);
  preferences.putFloat("currentB_daily", currentB_daily);
  preferences.putFloat("currentCombined_daily", currentCombined_daily); // Fixed key name
  preferences.putInt("lastResetDay", lastResetDay);
  preferences.end();
}

void loadDailyTotals() {
  preferences.begin("power_monitor", true); // Read-only
  
  currentA_daily = preferences.getFloat("currentA_daily", 0.0);
  currentB_daily = preferences.getFloat("currentB_daily", 0.0);
  currentCombined_daily = preferences.getFloat("currentComb_daily", 0.0);
  lastResetDay = preferences.getInt("lastResetDay", -1);
  
  preferences.end();
  
  Serial.printf("Loaded daily totals from storage: A=%.3f Ah, B=%.3f Ah, Combined=%.3f Ah\n", 
                currentA_daily, currentB_daily, currentCombined_daily);
}

// --- Current Monitoring Functions ---
float readCurrentRMS(int gpio_pin, float* rms_voltage_out, float* dc_offset_out) {
  float sum_squares = 0.0;
  float sum_voltages = 0.0; // For calculating actual DC offset
  unsigned long start_time = micros();
  
  // Take multiple samples to calculate RMS and measure DC offset
  for (int i = 0; i < SAMPLES_PER_MEASUREMENT; i++) {
    int adc_value = analogRead(gpio_pin);
    float voltage = (adc_value * GPIO_VOLTAGE_RANGE) / GPIO_RESOLUTION;
    
    // Accumulate voltage readings for DC offset calculation
    sum_voltages += voltage;
    
    // Remove DC offset to get AC component
    float ac_voltage = voltage - AC_OFFSET;
    
    // Square the voltage for RMS calculation
    sum_squares += ac_voltage * ac_voltage;
    
    // Wait for next sample (maintain consistent sampling rate)
    while (micros() - start_time < (i + 1) * SAMPLE_INTERVAL_US) {
      // Busy wait for precise timing
    }
  }
  
  // Calculate actual DC offset (average voltage)
  float measured_dc_offset = sum_voltages / SAMPLES_PER_MEASUREMENT;
  
  // Store the measured DC offset for debugging
  if (dc_offset_out != nullptr) {
    *dc_offset_out = measured_dc_offset;
  }
  
  // Calculate RMS voltage
  float rms_voltage = sqrt(sum_squares / SAMPLES_PER_MEASUREMENT);
  
  // Store the RMS voltage for output
  if (rms_voltage_out != nullptr) {
    *rms_voltage_out = rms_voltage;
  }
  
  // Convert RMS voltage to RMS current using CT clamp specifications
  // For SCT-013-030: 30A primary current = 1V RMS output
  // Therefore: I_primary = V_rms * 30 A/V
  float rms_current = rms_voltage * VOLTAGE_TO_CURRENT_SCALE;
  
  return rms_current;
}

void readCurrents() {
  currentA_instant = readCurrentRMS(GPIO_CHANNEL_A, &voltageA_rms, &dcOffsetA); // Read RMS from GPIO1
  currentB_instant = readCurrentRMS(GPIO_CHANNEL_B, &voltageB_rms, &dcOffsetB); // Read RMS from GPIO2
  
  // Calculate combined current (total of both channels)
  currentCombined_instant = currentA_instant + currentB_instant;
  
  // Accumulate daily values (in Amp-hours)
  // Convert current (A) to Amp-hours by multiplying with time interval in hours
  float timeInterval = (millis() - lastReadingTime) / 3600000.0; // Convert ms to hours
  
  if (lastReadingTime > 0) { // Skip first reading
    currentA_daily += currentA_instant * timeInterval;
    currentB_daily += currentB_instant * timeInterval;
    currentCombined_daily += currentCombined_instant * timeInterval;
    
    // Save to persistent storage every 60 readings (1 minute)
    static int saveCounter = 0;
    if (++saveCounter >= 60) {
      saveDailyTotals();
      saveCounter = 0;
    }
  }
  
  lastReadingTime = millis();
  
  // Check for midnight reset using actual time
  checkMidnightReset();
}

void publishCurrents() {
  char currentA_str[10];
  char currentB_str[10];
  char currentCombined_str[10];
  char currentA_daily_str[10];
  char currentB_daily_str[10];
  char currentCombined_daily_str[10];
  char voltageA_str[10];
  char voltageB_str[10];
  
  // Convert floats to strings
  dtostrf(currentA_instant, 1, 2, currentA_str);
  dtostrf(currentB_instant, 1, 2, currentB_str);
  dtostrf(currentCombined_instant, 1, 2, currentCombined_str);
  dtostrf(currentA_daily, 1, 3, currentA_daily_str);
  dtostrf(currentB_daily, 1, 3, currentB_daily_str);
  dtostrf(currentCombined_daily, 1, 3, currentCombined_daily_str);
  dtostrf(voltageA_rms, 1, 3, voltageA_str);
  dtostrf(voltageB_rms, 1, 3, voltageB_str);
  
  // Publish instant values
  client.publish(topic_currentA, currentA_str);
  client.publish(topic_currentB, currentB_str);
  client.publish(topic_currentCombined, currentCombined_str);
  
  // Publish daily accumulated values
  client.publish(topic_currentA_daily, currentA_daily_str);
  client.publish(topic_currentB_daily, currentB_daily_str);
  client.publish(topic_currentCombined_daily, currentCombined_daily_str);
  
  // Publish raw voltage measurements
  client.publish(topic_voltageA, voltageA_str);
  client.publish(topic_voltageB, voltageB_str);
  
  Serial.printf("Current A: %.2f A (%.3f V RMS), Current B: %.2f A (%.3f V RMS)\n", 
                currentA_instant, voltageA_rms, currentB_instant, voltageB_rms);
  Serial.printf("Combined: %.2f A, Daily A: %.3f Ah, Daily B: %.3f Ah, Daily Combined: %.3f Ah\n", 
                currentCombined_instant, currentA_daily, currentB_daily, currentCombined_daily);
  Serial.printf("DC Offset A: %.3f V, DC Offset B: %.3f V (Config: %.2f V)\n", 
                dcOffsetA, dcOffsetB, AC_OFFSET);
  Serial.printf("CT Clamp: %.0fA max / %.1fV max = %.0f A/V scaling\n", 
                CT_CLAMP_MAX_CURRENT, CT_CLAMP_MAX_VOLTAGE, VOLTAGE_TO_CURRENT_SCALE);
  
  // Print current time every few readings
  static int timeDisplayCounter = 0;
  if (++timeDisplayCounter >= 10) { // Show time every 10 readings (10 seconds)
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      char timeStr[30];
      strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
      Serial.printf("Current time: %s\n", timeStr);
    }
    timeDisplayCounter = 0;
  }
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
  
  // Get firmware version
  char versionStr[10];
  sprintf(versionStr, "%d", FIRMWARE_VERSION);
  
  // Publish to MQTT
  client.publish(topic_cpu_temp, tempStr);
  client.publish(topic_uptime, uptimeStr.c_str());
  client.publish(topic_firmware_version, versionStr);
  
  Serial.printf("CPU Temperature: %.1f°C, Uptime: %s, Firmware: v%d\n", cpuTemp, uptimeStr.c_str(), FIRMWARE_VERSION);
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
  // A0 and A1 are automatically configured for analog input
  Serial.printf("Using GPIO%d (A0) for Current A and GPIO%d (A1) for Current B\n", A0, A1);
  
  Serial.println("Starting WiFi connection...");
  setup_wifi();
  Serial.println("Connected to WiFi");
  Serial.printf("Free heap after WiFi: %d bytes\n", ESP.getFreeHeap());
  delay(500);
  
  // Setup time synchronization after WiFi is connected
  setupTime();
  delay(500);
  
  // Load daily totals from persistent storage
  loadDailyTotals();
  
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
  checkForUpdates();
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
  if (millis() - lastUpdateCheck > updateInterval) {
    checkForUpdates();
    lastUpdateCheck = millis();
  }

  // Publish system info every 30 seconds
  if (millis() - lastSystemPublish > systemPublishInterval) {
    publishSystemInfo();
    lastSystemPublish = millis();
  }

  delay(100); // Small delay for loop stability
}


