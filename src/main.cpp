/*
 * Maverick ET-732 BBQ Thermometer Decoder (ESP32) + WiFi & MQTT + NTP
 *
 * - 433.92 MHz OOK (Manchester ~2 kbps)
 * - Receiver on GPIO 23
 * - Serial output + MQTT publish of probe1 / probe2
 * - WiFi and MQTT configuration via WiFiManager portal
 * - NTP time sync for human-readable timestamps
 *
 * Required libraries:
 *   - WiFiManager (https://github.com/tzapu/WiFiManager)
 *   - PubSubClient
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <time.h>

// === Configuration ===
#define PIN_RX 23
#define STATE_START_PULSES  0
#define STATE_FIRST_BIT     1
#define STATE_DATA          2

// === Globals ===
volatile unsigned int start_pulse_counter = 0;
volatile unsigned int detection_state = STATE_START_PULSES;
volatile unsigned long last_interrupt_micros = 0;
volatile unsigned long last_interrupt_millis = 0;
volatile unsigned int nibble = 0;
volatile unsigned int data_array_index = 0;

volatile unsigned int shift_value = 0;
volatile unsigned int short_bit = 0;
volatile unsigned int add_1st_bit = 1;
volatile unsigned int current_byte = 0;
volatile unsigned int current_bit = 1;
volatile unsigned int bit_count = 0;

volatile uint8_t data_array[13];
volatile bool packet_ready = false;
volatile uint8_t packet_buffer[13];

uint8_t last_save_array[13];
uint32_t last_change_time = 0;
unsigned int probe1 = 0, probe2 = 0;
unsigned int probe1_array[6], probe2_array[6];

// === WiFi / MQTT ===
Preferences prefs;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

String mqtt_server = "192.168.";
uint16_t mqtt_port = 1883;
String mqtt_user = "";
String mqtt_pass = "";
String mqtt_topic = "bbq2mqtt";

const char* PREF_NAMESPACE = "bbq2mqtt";
const char* PREF_MQTT_SERVER = "mqtt_server";
const char* PREF_MQTT_PORT   = "mqtt_port";
const char* PREF_MQTT_USER   = "mqtt_user";
const char* PREF_MQTT_PASS   = "mqtt_pass";
const char* PREF_MQTT_TOPIC  = "mqtt_topic";

// === Helper functions ===
unsigned int quart(unsigned int param) {
  param &= 0x0F;
  if (param == 0x05) return 0;
  if (param == 0x06) return 1;
  if (param == 0x09) return 2;
  if (param == 0x0A) return 3;
  return 0xFF;
}

// === Forward declarations ===
void processPacket(uint8_t *buf, size_t len);
void mqttReconnect();
void saveMqttConfig(const String &server, uint16_t port, const String &user, const String &pass, const String &topic);
void setupTime();

// === Interrupt handler ===
void IRAM_ATTR handleInterrupt() {
  unsigned long nowMicros = micros();
  unsigned long nowMillis = millis();
  unsigned long time_since_last_ms = nowMillis - last_interrupt_millis;
  unsigned long tsl_micros = nowMicros - last_interrupt_micros;

  last_interrupt_micros = nowMicros;
  last_interrupt_millis = nowMillis;

  unsigned int bit_ok = 0;
  bool pinHigh = digitalRead(PIN_RX);

  // --- Preamble detection ---
  if (detection_state == STATE_START_PULSES) {
    if (((time_since_last_ms > 3) && (time_since_last_ms < 7)) && pinHigh) {
      start_pulse_counter++;
      if (start_pulse_counter == 8) {
        start_pulse_counter = 0;
        detection_state = STATE_FIRST_BIT;
      }
    } else if (tsl_micros > 400) {
      start_pulse_counter = 0;
    }
  }
  else if (detection_state == STATE_FIRST_BIT && pinHigh) {
    detection_state = STATE_DATA;
    current_bit = 1;
    current_byte = 0;
    shift_value = 0;
    data_array_index = 0;
    short_bit = 0;
    add_1st_bit = 1;
    bit_count = 1;
  }

  // --- Data decoding ---
  if (detection_state == STATE_DATA) {
    if ((tsl_micros > 90) && (tsl_micros < 390)) {
      if (short_bit == 0) short_bit = 1;
      else { short_bit = 0; bit_ok = 1; }
    }

    if ((tsl_micros > 390) && (tsl_micros < 650)) {
      if (short_bit == 1) {
        detection_state = STATE_START_PULSES;
      }
      bit_count++;
      current_bit = pinHigh;
      bit_ok = 1;
    }

    if (bit_ok) {
      if (add_1st_bit) {
        current_byte = 0x01;
        shift_value = 1;
        add_1st_bit = 0;
      }

      current_byte = (current_byte << 1) + current_bit;
      shift_value++;

      if (shift_value == 8) {
        if (data_array_index < 13) data_array[data_array_index++] = current_byte;
        shift_value = 0;
        current_byte = 0;
      }

      if (data_array_index >= 9) {
        for (uint8_t i = 0; i < 9; i++) packet_buffer[i] = data_array[i];
        packet_ready = true;
        start_pulse_counter = 0;
        detection_state = STATE_START_PULSES;
        data_array_index = 0;
      }
      bit_ok = 0;
    }
  }
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nMaverick ET-732 Decoder (ESP32) + MQTT + NTP");
  Serial.println("Receiver on GPIO 23");

  pinMode(PIN_RX, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RX), handleInterrupt, CHANGE);

  prefs.begin(PREF_NAMESPACE, false);

  // Load stored MQTT settings if available
  if (prefs.isKey(PREF_MQTT_SERVER)) {
    mqtt_server = prefs.getString(PREF_MQTT_SERVER, "");
    mqtt_port = (uint16_t)prefs.getUInt(PREF_MQTT_PORT, 1883);
    mqtt_user = prefs.getString(PREF_MQTT_USER, "");
    mqtt_pass = prefs.getString(PREF_MQTT_PASS, "");
    mqtt_topic = prefs.getString(PREF_MQTT_TOPIC, "bbq2mqtt");
    Serial.println("Loaded saved MQTT settings.");
  } else {
    Serial.println("No saved MQTT settings found.");
  }

  // WiFiManager setup
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("bbq2mqtt");

  WiFiManager wm;
  WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", mqtt_server.c_str(), 64);
  char portBuf[8]; sprintf(portBuf, "%u", mqtt_port);
  WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", portBuf, 6);
  WiFiManagerParameter custom_mqtt_user("user", "MQTT User", mqtt_user.c_str(), 32);
  WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", mqtt_pass.c_str(), 32);
  WiFiManagerParameter custom_mqtt_topic("topic", "MQTT Topic", mqtt_topic.c_str(), 64);

  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_pass);
  wm.addParameter(&custom_mqtt_topic);

  wm.setConfigPortalTimeout(30); // 30 sec portal
  wm.setCaptivePortalEnable(true);

  Serial.println("Starting WiFiManager config portal for 30 seconds...");
  bool portalResult = wm.startConfigPortal("bbq2mqtt_Config");

  if (portalResult) {
    Serial.println("Configuration complete or using saved credentials.");
  } else {
    Serial.println("Config portal timeout – trying saved WiFi credentials...");
  }

  // Connect to WiFi
  WiFi.begin();
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Connected to WiFi: %s, IP: %s\n",
                  WiFi.SSID().c_str(),
                  WiFi.localIP().toString().c_str());
    setupTime();
  } else {
    Serial.println("Failed to connect to WiFi. Restarting...");
    ESP.restart();
  }

  // Read parameters after portal
  String srv = custom_mqtt_server.getValue();
  String prt = custom_mqtt_port.getValue();
  String usr = custom_mqtt_user.getValue();
  String pwd = custom_mqtt_pass.getValue();
  String top = custom_mqtt_topic.getValue();
  uint16_t port = 1883;
  if (prt.length()) port = (uint16_t)atoi(prt.c_str());
  saveMqttConfig(srv, port, usr, pwd, top);

  mqttClient.setServer(mqtt_server.c_str(), mqtt_port);
}

// === Loop ===
void loop() {
  if (!mqttClient.connected()) mqttReconnect();
  mqttClient.loop();

  if (packet_ready) {
    noInterrupts();
    uint8_t local_buf[13];
    for (uint8_t i = 0; i < 13; i++) local_buf[i] = packet_buffer[i];
    packet_ready = false;
    interrupts();
    processPacket(local_buf, 9);
  }

  delay(50);
}

// === Process received packet ===
void processPacket(uint8_t *buf, size_t len) {
  uint8_t local_save[13];
  memset(local_save, 0, sizeof(local_save));
  for (uint8_t i = 0; i < len && i < sizeof(local_save); i++) local_save[i] = buf[i];

  // Header check
  if ((local_save[0] != 0xAA) ||
      (local_save[1] != 0x99) ||
      (local_save[2] != 0x95) ||
      (local_save[3] != 0x59)) return;

  // Duplicate filter
  if (memcmp((const void*)local_save, (const void*)last_save_array, sizeof(local_save)) == 0) return;
  memcpy((void*)last_save_array, (const void*)local_save, sizeof(local_save));
  last_change_time = millis();

  unsigned int p1 = 0, p2 = 0;
  unsigned int p2_arr[6], p1_arr[6];

  p2_arr[0] = quart(local_save[8] & 0x0F);
  p2_arr[1] = quart(local_save[8] >> 4);
  p2_arr[2] = quart(local_save[7] & 0x0F);
  p2_arr[3] = quart(local_save[7] >> 4);
  p2_arr[4] = quart(local_save[6] & 0x0F);

  p1_arr[0] = quart(local_save[6] >> 4);
  p1_arr[1] = quart(local_save[5] & 0x0F);
  p1_arr[2] = quart(local_save[5] >> 4);
  p1_arr[3] = quart(local_save[4] & 0x0F);
  p1_arr[4] = quart(local_save[4] >> 4);

  for (uint8_t i = 0; i <= 4; i++) {
    if (p1_arr[i] == 0xFF) p1_arr[i] = 0;
    if (p2_arr[i] == 0xFF) p2_arr[i] = 0;
    p1 += p1_arr[i] * (1 << (2 * i));
    p2 += p2_arr[i] * (1 << (2 * i));
  }

  p1 -= 532;
  p2 -= 532;

  probe1 = p1;
  probe2 = p2;

  // Time for display
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  char isoTime[32];
  if (now > 1000000000) {
    strftime(isoTime, sizeof(isoTime), "%Y-%m-%dT%H:%M:%S%z", &timeinfo);
  } else {
    snprintf(isoTime, sizeof(isoTime), "unsynced");
  }

  Serial.printf("[%s] Probe1: %d °C  Probe2: %d °C\n", isoTime, probe1, probe2);

  // MQTT publish
  if (mqttClient.connected() && mqtt_topic.length() > 0) {
    char payload[256];
    int len = snprintf(payload, sizeof(payload),
                       "{\"probe1\":%u,\"probe2\":%u,\"time\":\"%s\"}",
                       probe1, probe2, isoTime);
    if (len > 0) {
      if (mqttClient.publish(mqtt_topic.c_str(), payload))
        Serial.printf("MQTT published to %s: %s\n", mqtt_topic.c_str(), payload);
      else
        Serial.println("MQTT publish failed.");
    }
  }
}

// === MQTT reconnect ===
void mqttReconnect() {
  if (mqtt_server.length() == 0) return;
  if (mqttClient.connected()) return;

  Serial.printf("Connecting to MQTT broker %s:%u ...\n", mqtt_server.c_str(), mqtt_port);
  bool connected = false;
  if (mqtt_user.length() > 0)
    connected = mqttClient.connect("ET732_Client", mqtt_user.c_str(), mqtt_pass.c_str());
  else
    connected = mqttClient.connect("ET732_Client");

  if (connected)
    Serial.println("MQTT connected.");
  else {
    Serial.printf("MQTT connect failed, rc=%d\n", mqttClient.state());
    delay(2000);
  }
}

// === Save MQTT config to Preferences ===
void saveMqttConfig(const String &server, uint16_t port, const String &user, const String &pass, const String &topic) {
  mqtt_server = server;
  mqtt_port = port;
  mqtt_user = user;
  mqtt_pass = pass;
  mqtt_topic = topic;

  prefs.putString(PREF_MQTT_SERVER, mqtt_server);
  prefs.putUInt(PREF_MQTT_PORT, mqtt_port);
  prefs.putString(PREF_MQTT_USER, mqtt_user);
  prefs.putString(PREF_MQTT_PASS, mqtt_pass);
  prefs.putString(PREF_MQTT_TOPIC, mqtt_topic);

  Serial.println("MQTT settings saved.");
}

// === NTP time setup ===
void setupTime() {
  const long gmtOffset_sec = 3600;       // UTC+1 (adjust as needed)
  const int daylightOffset_sec = 3600;   // add 1h for DST if applicable
  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");

  Serial.print("Waiting for NTP time sync");
  time_t now = time(nullptr);
  int retry = 0;
  while (now < 1000000000 && retry < 20) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
    retry++;
  }
  Serial.println();

  if (now >= 1000000000) {
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    Serial.printf("Time synchronized: %04d-%02d-%02d %02d:%02d:%02d\n",
                  timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  } else {
    Serial.println("NTP time sync failed (fallback to millis).");
  }
}
