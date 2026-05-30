#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "common.h"
#include "heading_ctrl.h"

// ── Motor / ESC ───────────────────────────────────────────────────────────────
Servo esc_L;
Servo esc_R;

const int MOTOR_MIN = 1000;
const int MOTOR_MAX = 1600;
const int Motor_R   = 16;
const int Motor_L   =  4;

// ── Joystick axis config ──────────────────────────────────────────────────────
const int   X_MIN            =    0;
const int   X_MAX            = 4096;
const int   Y_MIN            =    0;
const int   Y_MAX            = 4096;
const int   X_DEADZONE       =   80;
const int   Y_DEADZONE       =   80;
const int   X_NEUTRAL        = 1840;
const int   Y_NEUTRAL        = 1880;
const float STEERING_STRENGTH= 0.25f;

// ── Ultrasonic sensors ────────────────────────────────────────────────────────
const int SENSOR_1_TRIG = 17;
const int SENSOR_1_ECHO =  5;
const int SENSOR_2_TRIG = 18;
const int SENSOR_2_ECHO = 19;
const int SENSOR_3_TRIG = 21;
const int SENSOR_3_ECHO = 22;

const unsigned long SENSOR_READ_INTERVAL_MS = 200;
const unsigned long SENSOR_GAP_MS           =  40;

// ── PSoC6 UART (from Infineon SensorDuck board) ───────────────────────────────
// Wire: PSoC P9_1 (TX) → GPIO27   |   PSoC P9_0 (RX) ← GPIO26   |   GND ↔ GND
#define PSOC_RX_PIN   27
#define PSOC_TX_PIN   26
#define PSOC_BAUD     115200

// ── WiFi / MQTT ───────────────────────────────────────────────────────────────
WiFiClient    wifiClient;
PubSubClient  mqttClient(wifiClient);

const uint8_t DEFAULT_ESPNOW_CHANNEL = 1;

const char *WIFI_SSID     = "Ducknet";
const char *WIFI_PASSWORD = "Ducknet123";

const char *MQTT_BROKER   = "10.42.0.1";
const uint16_t MQTT_PORT  = 1883;

const char *MQTT_CLIENT_ID      = "duck-firmware";
const char *MQTT_STATUS_TOPIC   = "duck/status";
const char *MQTT_STICK_TOPIC    = "duck/stick";
const char *MQTT_QUACK_TOPIC    = "duck/quack";
const char *MQTT_COMMAND_TOPIC  = "duck/cmd";
const char *MQTT_SENSORS_TOPIC  = "duck/sensors";
const char *MQTT_GAINS_TOPIC    = "duck/gains";      // subscribe: {"kp":0.025,"kd":0.015,"dead_zone":5.0}
const char *MQTT_GAINS_ACK_TOPIC= "duck/gains/ack"; // publish:  clamped gains echo
const char *MQTT_HEADING_TOPIC  = "duck/heading";   // publish:  current heading state

// ── Heading controller ────────────────────────────────────────────────────────
HeadingCtrl headingCtrl;

// Latest values from PSoC6 (updated by _drain_psoc_uart)
static float g_heading     = NAN;   // tilt-compensated when possible, else raw mag.hdg
static float g_heading_raw = NAN;   // simple 2-D mag.hdg for reference
static float g_gyro_z      = 0.0f;  // yaw rate °/s from BMI270
static bool  g_psoc_ok     = false;
static unsigned long g_psoc_last_ms = 0;

// ── Timing ────────────────────────────────────────────────────────────────────
unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastSensorReadMs         = 0;
unsigned long lastHeadingPublishMs     = 0;
unsigned long lastPacketReceivedMs     = 0;
unsigned long lastStickPacketReceivedMs= 0;

// ── Motor state ───────────────────────────────────────────────────────────────
float currentMotorLeft  = 0.0f;
float currentMotorRight = 0.0f;
const float SECS_ZEROTOMAX  = 1.5f;
float accelerating_for      = 0.0f;
int   time_last_received    = 0;
const int TIMEOUT_STOP_AFTER_RECV = 1000;
float prev_millis = 0.0f;

#define FULL 1000000
#define bool_str(x) ((x) > 0 ? "TRUE" : "FALSE")

// =============================================================================
// WiFi / ESP-NOW helpers
// =============================================================================
static uint8_t getEspNowChannel()
{
    return (WiFi.status() == WL_CONNECTED) ? WiFi.channel() : DEFAULT_ESPNOW_CHANNEL;
}

static void applyWifiChannel(uint8_t channel)
{
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

static void publishState(const char *topic, const String &payload)
{
    if (mqttClient.connected())
        mqttClient.publish(topic, payload.c_str(), true);
}

// =============================================================================
// PSoC6 UART — non-blocking JSON line reader
// =============================================================================
static char _rx_buf[256];
static int  _rx_pos = 0;

static void _parse_psoc_line(const char *line)
{
    JsonDocument doc;
    if (deserializeJson(doc, line) != DeserializationError::Ok) return;

    bool imu_ok = doc["ok"]["imu"] | false;
    bool mag_ok = doc["ok"]["mag"] | false;
    if (!mag_ok) return;

    float mag_hdg = doc["mag"]["hdg"]      | NAN;
    float mx      = doc["mag"]["field"]["x"]| NAN;
    float my      = doc["mag"]["field"]["y"]| NAN;
    float mz      = doc["mag"]["field"]["z"]| NAN;
    float ax      = doc["imu"]["accel"]["x"]| NAN;
    float ay      = doc["imu"]["accel"]["y"]| NAN;
    float az      = doc["imu"]["accel"]["z"]| NAN;
    float gz      = doc["imu"]["gyro"]["z"] | 0.0f;

    g_heading_raw = mag_hdg;
    g_gyro_z      = gz;
    g_psoc_ok     = true;
    g_psoc_last_ms= millis();

    // Prefer tilt-compensated heading when the IMU is valid
    if (imu_ok && !isnan(ax) && !isnan(mx)) {
        float th = HeadingCtrl::tilt_heading(ax, ay, az, mx, my, mz);
        g_heading = isnan(th) ? mag_hdg : th;
    } else {
        g_heading = mag_hdg;
    }
}

static void _drain_psoc_uart()
{
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        if (c == '\n') {
            if (_rx_pos > 0 && _rx_buf[_rx_pos - 1] == '\r') _rx_pos--;
            _rx_buf[_rx_pos] = '\0';
            if (_rx_pos > 0) _parse_psoc_line(_rx_buf);
            _rx_pos = 0;
        } else if (_rx_pos < (int)sizeof(_rx_buf) - 1) {
            _rx_buf[_rx_pos++] = c;
        } else {
            _rx_pos = 0;  // buffer overflow — resync on next newline
        }
    }
}

// =============================================================================
// Heading state → MQTT
// =============================================================================
static void _publish_heading()
{
    if (!mqttClient.connected()) return;

    char buf[192];
    snprintf(buf, sizeof(buf),
        "{\"hdg\":%.1f,\"hdg_raw\":%.1f,\"target\":%.1f,"
        "\"gyro_z\":%.2f,\"enabled\":%s,"
        "\"kp\":%.4f,\"kd\":%.4f,\"dz\":%.1f,"
        "\"psoc_ok\":%s}",
        (double)(isnan(g_heading)     ? -1.0f : g_heading),
        (double)(isnan(g_heading_raw) ? -1.0f : g_heading_raw),
        (double)(headingCtrl.target() < 0 ? -1.0f : headingCtrl.target()),
        (double)g_gyro_z,
        headingCtrl.enabled ? "true" : "false",
        (double)headingCtrl.kp,
        (double)headingCtrl.kd,
        (double)headingCtrl.dead_zone,
        g_psoc_ok ? "true" : "false");

    mqttClient.publish(MQTT_HEADING_TOPIC, buf, false);
}

// =============================================================================
// Ultrasonic sensors
// =============================================================================
static float readDistanceCm(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    unsigned long dur = pulseIn(echoPin, HIGH, 30000);
    return (dur == 0) ? -1.0f : (float)dur * 0.0343f * 0.5f;
}

static void appendDistanceJsonValue(String &json, float cm)
{
    if (cm < 0.0f) json += "null";
    else           json += String(cm, 1);
}

static void setupSensorPins()
{
    pinMode(SENSOR_1_TRIG, OUTPUT); pinMode(SENSOR_1_ECHO, INPUT);
    pinMode(SENSOR_2_TRIG, OUTPUT); pinMode(SENSOR_2_ECHO, INPUT);
    pinMode(SENSOR_3_TRIG, OUTPUT); pinMode(SENSOR_3_ECHO, INPUT);
    digitalWrite(SENSOR_1_TRIG, LOW);
    digitalWrite(SENSOR_2_TRIG, LOW);
    digitalWrite(SENSOR_3_TRIG, LOW);
}

static void readAndPublishSensors()
{
    float d1 = readDistanceCm(SENSOR_1_TRIG, SENSOR_1_ECHO); delay(SENSOR_GAP_MS);
    float d2 = readDistanceCm(SENSOR_2_TRIG, SENSOR_2_ECHO); delay(SENSOR_GAP_MS);
    float d3 = readDistanceCm(SENSOR_3_TRIG, SENSOR_3_ECHO);

    String payload = "{";
    payload += "\"sensor_1_cm\":"; appendDistanceJsonValue(payload, d1);
    payload += ",\"sensor_2_cm\":"; appendDistanceJsonValue(payload, d2);
    payload += ",\"sensor_3_cm\":"; appendDistanceJsonValue(payload, d3);
    payload += "}";
    publishState(MQTT_SENSORS_TOPIC, payload);
}

// =============================================================================
// MQTT
// =============================================================================
static void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    String message;
    message.reserve(length);
    for (unsigned int i = 0; i < length; i++)
        message += (char)payload[i];

    Serial.printf("[MQTT] %s: %s\n", topic, message.c_str());

    // ── duck/cmd ─────────────────────────────────────────────────────────────
    if (strcmp(topic, MQTT_COMMAND_TOPIC) == 0) {
        if (message == "stop") {
            esc_L.writeMicroseconds(MOTOR_MIN);
            esc_R.writeMicroseconds(MOTOR_MIN);
            publishState(MQTT_STATUS_TOPIC, "motors stopped");
        } else if (message == "drift_on") {
            headingCtrl.enabled = true;
            headingCtrl.unlock();  // re-lock on current heading when next packet arrives
            publishState(MQTT_STATUS_TOPIC, "drift correction ON");
        } else if (message == "drift_off") {
            headingCtrl.enabled = false;
            headingCtrl.unlock();
            publishState(MQTT_STATUS_TOPIC, "drift correction OFF");
        }
        return;
    }

    // ── duck/gains ────────────────────────────────────────────────────────────
    // Accepts JSON: {"kp":0.025,"kd":0.015,"dead_zone":5.0}
    // Any missing key keeps its current value.
    // Values outside safety limits are silently clamped (see heading_ctrl.h).
    if (strcmp(topic, MQTT_GAINS_TOPIC) == 0) {
        JsonDocument doc;
        if (deserializeJson(doc, message) != DeserializationError::Ok) {
            Serial.println("[MQTT] gains: bad JSON");
            return;
        }
        float new_kp = doc["kp"]        | headingCtrl.kp;
        float new_kd = doc["kd"]        | headingCtrl.kd;
        float new_dz = doc["dead_zone"] | headingCtrl.dead_zone;
        headingCtrl.set_gains(new_kp, new_kd, new_dz);

        Serial.printf("[MQTT] gains → kp=%.4f kd=%.4f dz=%.1f\n",
                      (double)headingCtrl.kp,
                      (double)headingCtrl.kd,
                      (double)headingCtrl.dead_zone);

        // Echo clamped values back so the PC knows what was actually applied
        char ack[96];
        snprintf(ack, sizeof(ack),
                 "{\"kp\":%.4f,\"kd\":%.4f,\"dead_zone\":%.1f}",
                 (double)headingCtrl.kp,
                 (double)headingCtrl.kd,
                 (double)headingCtrl.dead_zone);
        mqttClient.publish(MQTT_GAINS_ACK_TOPIC, ack, false);
    }
}

static void connectToWiFi()
{
    if (WiFi.status() == WL_CONNECTED) return;

    Serial.printf("Connecting to WiFi %s\n", WIFI_SSID);
    WiFi.setSleep(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(500); Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        applyWifiChannel(getEspNowChannel());
        Serial.printf("WiFi connected, IP: %s  channel: %d\n",
                      WiFi.localIP().toString().c_str(), WiFi.channel());
    } else {
        Serial.println("WiFi connection failed");
    }
}

static void connectToMqtt()
{
    if (WiFi.status() != WL_CONNECTED || mqttClient.connected()) return;

    unsigned long now = millis();
    if (now - lastMqttReconnectAttempt < 5000) return;
    lastMqttReconnectAttempt = now;

    Serial.printf("Connecting to MQTT %s:%d\n", MQTT_BROKER, MQTT_PORT);
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);

    if (mqttClient.connect(MQTT_CLIENT_ID)) {
        Serial.println("MQTT connected");
        mqttClient.subscribe(MQTT_COMMAND_TOPIC);
        mqttClient.subscribe(MQTT_GAINS_TOPIC);
        publishState(MQTT_STATUS_TOPIC, "online");
    } else {
        Serial.printf("MQTT failed, rc=%d\n", mqttClient.state());
    }
}

// =============================================================================
// Motor drive  (called from ESP-NOW callback)
// =============================================================================
static void driveEscFromStick(const StickData &stickData)
{
    unsigned long mills = millis();
    float delta = (mills - prev_millis) / 1000.0f;
    prev_millis = mills;
    accelerating_for += delta;

    if (mills - time_last_received > TIMEOUT_STOP_AFTER_RECV) {
        accelerating_for = 0;
    }
    time_last_received = mills;

    const int x = stickData.x;
    const int y = stickData.y;

    // ── Throttle ──────────────────────────────────────────────────────────────
    float throttle = 0.0f;
    if (y > Y_NEUTRAL + Y_DEADZONE) {
        throttle = constrain(
            map(y, Y_NEUTRAL + Y_DEADZONE, Y_MAX, 0, FULL), 0, FULL) / (float)FULL;
    }

    // ── Manual steering delta ─────────────────────────────────────────────────
    float steeringDelta = 0.0f;
    if (x < X_NEUTRAL - X_DEADZONE) {
        steeringDelta = STEERING_STRENGTH *
            (constrain(map(x, X_NEUTRAL - X_DEADZONE, X_MIN, 0, FULL), 0, FULL) / (float)FULL);
    } else if (x > X_NEUTRAL + X_DEADZONE) {
        steeringDelta = -STEERING_STRENGTH *
            (constrain(map(x, X_NEUTRAL + X_DEADZONE, X_MAX, 0, FULL), 0, FULL) / (float)FULL);
    }

    // ── Acceleration ramp ─────────────────────────────────────────────────────
    const float E = 5.0f;
    float accelProgress = constrain(pow(E, (accelerating_for / SECS_ZEROTOMAX) - 1), 0.0, (double)throttle);
    float real_throttle = constrain(accelProgress, 0.0f, throttle);

    if (throttle <= 0.0f) accelerating_for = 0.0f;

    // ── Heading drift correction ───────────────────────────────────────────────
    // Only active while moving forward. When the joystick is steering, the
    // controller unlocks and re-locks the target heading once it returns to center.
    float driftCorr = 0.0f;
    bool  isManualSteering = (fabsf(steeringDelta) > 0.001f);

    if (throttle > 0.05f) {
        float hdg = isnan(g_heading) ? g_heading_raw : g_heading;
        driftCorr = headingCtrl.update(hdg, g_gyro_z, isManualSteering)
                    * STEERING_STRENGTH;
    } else {
        // Stopped — release the heading lock so it re-locks on the new heading
        // when the duck starts moving again.
        headingCtrl.unlock();
    }

    // ── Mix and apply ─────────────────────────────────────────────────────────
    float goalLeft  = real_throttle - steeringDelta - driftCorr;
    float goalRight = real_throttle + steeringDelta + driftCorr;

    currentMotorLeft  = constrain(goalLeft,  0.0f, 1.0f + STEERING_STRENGTH);
    currentMotorRight = constrain(goalRight, 0.0f, 1.0f + STEERING_STRENGTH);

    esc_L.writeMicroseconds(MOTOR_MIN + (int)(currentMotorLeft  * (MOTOR_MAX - MOTOR_MIN)));
    esc_R.writeMicroseconds(MOTOR_MIN + (int)(currentMotorRight * (MOTOR_MAX - MOTOR_MIN)));

    Serial.printf("[DRIVE] thr=%.2f steer=%.3f drift=%.3f → L=%.2f R=%.2f\n",
                  (double)real_throttle, (double)steeringDelta, (double)driftCorr,
                  (double)currentMotorLeft, (double)currentMotorRight);

    if (mqttClient.connected()) {
        char buf[96];
        snprintf(buf, sizeof(buf),
                 "{\"x\":%d,\"y\":%d,\"left\":%.3f,\"right\":%.3f,\"drift\":%.3f}",
                 x, y, (double)goalLeft, (double)goalRight, (double)driftCorr);
        mqttClient.publish(MQTT_STICK_TOPIC, buf, false);
    }
}

// =============================================================================
// ESP-NOW receive callback
// =============================================================================
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    Serial.print("[MSG] From: "); print_mac_address(mac_addr);

    if (len < (int)sizeof(Message)) {
        Serial.printf(" | invalid length (%d)\n", len);
        return;
    }

    Message message;
    memcpy(&message, data, sizeof(Message));
    Serial.printf("| %s", msg_type_name(message.msg_type));

    if (message.msg_type == STICK_DATA) {
        Serial.printf(" x=%d y=%d\n", message.data.stick_data.x, message.data.stick_data.y);
        driveEscFromStick(message.data.stick_data);
        lastPacketReceivedMs      = millis();
        lastStickPacketReceivedMs = lastPacketReceivedMs;
    } else if (message.msg_type == QUACK) {
        Serial.printf(" i=%d\n", message.data.i);
        if (mqttClient.connected())
            publishState(MQTT_QUACK_TOPIC, String(message.data.i));
        lastPacketReceivedMs = millis();
    } else {
        Serial.println();
    }
}

// =============================================================================
// Setup
// =============================================================================
void setup()
{
    Serial.begin(115200);
    WiFi.setSleep(false);

    setupSensorPins();
    connectToWiFi();
    delay(2000);

    Serial.printf("Receiver MAC: %s\n", WiFi.macAddress().c_str());

    applyWifiChannel(getEspNowChannel());
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while (true) delay(1000);
    }
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("ESP-NOW ready");

    esc_L.attach(Motor_L, 1000, 2000);
    esc_R.attach(Motor_R, 1000, 2000);
    esc_L.writeMicroseconds(MOTOR_MIN);
    esc_R.writeMicroseconds(MOTOR_MIN);

    // PSoC6 sensor UART — must set RX buffer before begin()
    Serial2.setRxBufferSize(512);
    Serial2.begin(PSOC_BAUD, SERIAL_8N1, PSOC_RX_PIN, PSOC_TX_PIN);
    Serial.printf("PSoC6 UART ready (RX=GPIO%d, TX=GPIO%d)\n", PSOC_RX_PIN, PSOC_TX_PIN);

    mqttClient.setBufferSize(512);
    connectToMqtt();

    delay(3000);
    Serial.println("Ready. Publish to duck/gains to tune, duck/cmd drift_on to enable.");
}

// =============================================================================
// Loop
// =============================================================================
void loop()
{
    // WiFi / MQTT keepalive
    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();
    } else {
        applyWifiChannel(getEspNowChannel());
    }

    if (!mqttClient.connected()) {
        connectToMqtt();
    } else {
        mqttClient.loop();
    }

    // Drain PSoC UART — updates g_heading, g_gyro_z
    _drain_psoc_uart();

    // Ultrasonic sensors (blocking ~80 ms when active — every 200 ms)
    unsigned long now = millis();
    if (now - lastSensorReadMs >= SENSOR_READ_INTERVAL_MS) {
        lastSensorReadMs = now;
        readAndPublishSensors();
    }

    // Publish heading state ~1 Hz for monitoring / compass_viewer cross-check
    if (now - lastHeadingPublishMs >= 1000) {
        lastHeadingPublishMs = now;
        _publish_heading();
    }

    delay(10);
}
