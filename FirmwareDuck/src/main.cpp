#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>
#include <PubSubClient.h>
#include "common.h"

Servo esc_L;
Servo esc_R;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

const int X_MIN = 0;
const int X_MAX = 1024;
const int Y_MIN = 0;
const int Y_MAX = 1024;

const int MOTOR_MIN = 1000; // Minimaler PWM-Wert für die Motoren
const int MOTOR_MAX = 1300; // Maximaler PWM-Wert für die Motoren

const int X_DEADZONE = 60; // Toter Bereich für die X-Achse
const int Y_DEADZONE = 60; // Toter Bereich für die Y-Achse
const int X_CENTER = 312;  // Zentrum der X-Achse
const int Y_CENTER = 512;  // Zentrum der Y-Achse

const int Motor_R = 16;
const int Motor_L = 4;
const uint8_t DEFAULT_ESPNOW_CHANNEL = 6;

const char *WIFI_SSID = "Ducknet";
const char *WIFI_PASSWORD = "Ducknet123";

const char *MQTT_BROKER = "10.42.0.1";
const uint16_t MQTT_PORT = 1883;
const char *MQTT_CLIENT_ID = "duck-firmware";
const char *MQTT_STATUS_TOPIC = "duck/status";
const char *MQTT_STICK_TOPIC = "duck/stick";
const char *MQTT_QUACK_TOPIC = "duck/quack";
const char *MQTT_COMMAND_TOPIC = "duck/cmd";
const char *MQTT_SENSORS_TOPIC = "duck/sensors";

const int SENSOR_1_TRIG = 17;
const int SENSOR_1_ECHO = 5;
const int SENSOR_2_TRIG = 18;
const int SENSOR_2_ECHO = 19;
const int SENSOR_3_TRIG = 21;
const int SENSOR_3_ECHO = 22;

const unsigned long SENSOR_READ_INTERVAL_MS = 200;
const unsigned long SENSOR_GAP_MS = 40;

unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastSensorReadMs = 0;

static uint8_t getEspNowChannel()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    return WiFi.channel();
  }

  return DEFAULT_ESPNOW_CHANNEL;
}

static void applyWifiChannel(uint8_t channel)
{
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

static void publishState(const char *topic, const String &payload)
{
  if (mqttClient.connected())
  {
    mqttClient.publish(topic, payload.c_str(), true);
  }
}

static float readDistanceCm(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const unsigned long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0)
  {
    return -1.0f;
  }

  return static_cast<float>(duration) * 0.0343f * 0.5f;
}

static void appendDistanceJsonValue(String &json, float distanceCm)
{
  if (distanceCm < 0.0f)
  {
    json += "null";
  }
  else
  {
    json += String(distanceCm, 1);
  }
}

static void setupSensorPins()
{
  pinMode(SENSOR_1_TRIG, OUTPUT);
  pinMode(SENSOR_1_ECHO, INPUT);
  pinMode(SENSOR_2_TRIG, OUTPUT);
  pinMode(SENSOR_2_ECHO, INPUT);
  pinMode(SENSOR_3_TRIG, OUTPUT);
  pinMode(SENSOR_3_ECHO, INPUT);

  digitalWrite(SENSOR_1_TRIG, LOW);
  digitalWrite(SENSOR_2_TRIG, LOW);
  digitalWrite(SENSOR_3_TRIG, LOW);
}

static void readAndPublishSensors()
{
  const float distance1 = readDistanceCm(SENSOR_1_TRIG, SENSOR_1_ECHO);
  delay(SENSOR_GAP_MS);
  const float distance2 = readDistanceCm(SENSOR_2_TRIG, SENSOR_2_ECHO);
  delay(SENSOR_GAP_MS);
  const float distance3 = readDistanceCm(SENSOR_3_TRIG, SENSOR_3_ECHO);

  String payload = "{";
  payload += "\"sensor_1_cm\":";
  appendDistanceJsonValue(payload, distance1);
  payload += ",\"sensor_2_cm\":";
  appendDistanceJsonValue(payload, distance2);
  payload += ",\"sensor_3_cm\":";
  appendDistanceJsonValue(payload, distance3);
  payload += "}";

  Serial.print("[SENSORS] ");
  Serial.println(payload);

  publishState(MQTT_SENSORS_TOPIC, payload);
}

static void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("[MQTT] Nachricht auf ");
  Serial.print(topic);
  Serial.print(": ");

  String message;
  for (unsigned int i = 0; i < length; i++)
  {
    message += static_cast<char>(payload[i]);
  }

  Serial.println(message);

  if (strcmp(topic, MQTT_COMMAND_TOPIC) == 0)
  {
    if (message == "stop")
    {
      esc_L.writeMicroseconds(MOTOR_MIN);
      esc_R.writeMicroseconds(MOTOR_MIN);
      publishState(MQTT_STATUS_TOPIC, "motors stopped");
    }
  }
}

static void connectToWiFi()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    return;
  }

  Serial.print("Verbinde mit WiFi ");
  Serial.println(WIFI_SSID);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000)
  {
    delay(500);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    applyWifiChannel(getEspNowChannel());
    Serial.print("WiFi verbunden, IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Aktiver Kanal: ");
    Serial.println(WiFi.channel());
  }
  else
  {
    Serial.println("WiFi Verbindungsfehler");
  }
}

static void connectToMqtt()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    return;
  }

  if (mqttClient.connected())
  {
    return;
  }

  const unsigned long now = millis();
  if (now - lastMqttReconnectAttempt < 5000)
  {
    return;
  }
  lastMqttReconnectAttempt = now;

  Serial.print("Verbinde mit MQTT Broker ");
  Serial.print(MQTT_BROKER);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  if (mqttClient.connect(MQTT_CLIENT_ID))
  {
    Serial.println("MQTT verbunden");
    mqttClient.subscribe(MQTT_COMMAND_TOPIC);
    publishState(MQTT_STATUS_TOPIC, "online");
  }
  else
  {
    Serial.print("MQTT Verbindungsfehler, rc=");
    Serial.println(mqttClient.state());
  }
}

static int mapToEsc(int value, int inMin, int inMax)
{
  return constrain(map(value, inMin, inMax, MOTOR_MIN, MOTOR_MAX), MOTOR_MIN, MOTOR_MAX);
}

static void driveEscFromStick(const StickData &stickData)
{
  const int x = stickData.x;
  const int y = stickData.y;

  int motorLeft = MOTOR_MIN;
  int motorRight = MOTOR_MIN;

  if (x < X_CENTER - X_DEADZONE)
  {
    motorLeft = constrain(map(x, X_MIN, X_CENTER - X_DEADZONE, MOTOR_MAX, MOTOR_MIN), MOTOR_MIN, MOTOR_MAX);
    motorRight = MOTOR_MIN;
  }
  else if (x > X_CENTER + X_DEADZONE)
  {
    motorLeft = MOTOR_MIN;
    motorRight = constrain(map(x, X_CENTER + X_DEADZONE, X_MAX, MOTOR_MIN, MOTOR_MAX), MOTOR_MIN, MOTOR_MAX);
  }
  else
  {
    if (y > Y_CENTER + Y_DEADZONE)
    {
      const int throttle = constrain(map(y, Y_CENTER + Y_DEADZONE, Y_MAX, MOTOR_MIN, MOTOR_MAX), MOTOR_MIN, MOTOR_MAX);
      motorLeft = throttle;
      motorRight = throttle;
    }
  }

  esc_L.writeMicroseconds(motorLeft);
  esc_R.writeMicroseconds(motorRight);

  if (mqttClient.connected())
  {
    String payload = String("{\"x\":") + x + String(",\"y\":") + y + String(",\"left\":") + motorLeft + String(",\"right\":") + motorRight + String("}");
    publishState(MQTT_STICK_TOPIC, payload);
  }
}

// Callback-Funktion, die automatisch aufgerufen wird, wenn eine ESP-NOW-Nachricht empfangen wird
// Sie erhält die MAC-Adresse des Senders und die empfangenen Daten.
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len)
{
  // MAC address des Senders ausgeben
  Serial.print("[MSG] From: ");
  print_mac_address(mac_addr);

  Serial.print(" | Message: ");
  if (len >= static_cast<int>(sizeof(Message)))
  {
    Message message;
    memcpy(&message, data, sizeof(Message));

    Serial.print(msg_type_name(message.msg_type));

    if (message.msg_type == STICK_DATA)
    {
      Serial.print(" x=");
      Serial.print(message.data.stick_data.x);
      Serial.print(", y=");
      Serial.print(message.data.stick_data.y);

      driveEscFromStick(message.data.stick_data);
    }
    else if (message.msg_type == QUACK)
    {
      Serial.print(" i=");
      Serial.print(message.data.i);

      if (mqttClient.connected())
      {
        publishState(MQTT_QUACK_TOPIC, String(message.data.i));
      }
    }
  }
  else
  {
    Serial.print("ungültige Länge (");
    Serial.print(len);
    Serial.print(")");
  }

  Serial.print(" ");
  Serial.println();
}

// Initialisierung des ESP32
void setup()
{
  Serial.begin(115200);
  WiFi.setSleep(false);

  setupSensorPins();

  connectToWiFi();
  delay(2000); // Delay for monitor

  // Zur Information: Die MAC-Adresse des ESP32 wird hier ausgegeben,
  // damit du sie für den Sender verwenden kannst
  Serial.print("Empfänger MAC: ");
  Serial.println(WiFi.macAddress());

  // Initialisiere ESP-NOW
  applyWifiChannel(getEspNowChannel());
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW Fehler");
    while (true)
      delay(1000);
  }

  // Registriere die Callback-Funktion, die aufgerufen wird, wenn Daten empfangen werden
  // OnDataRecv wird automatisch aufgerufen, wenn eine ESP-NOW-Nachricht ankommt
  esp_now_register_recv_cb(OnDataRecv);

  // Bestätige, dass der Empfänger bereit ist
  Serial.println("Empfänger bereit");

  esc_L.attach(Motor_L, 1000, 2000);
  esc_R.attach(Motor_R, 1000, 2000);

  esc_L.writeMicroseconds(1000);
  esc_R.writeMicroseconds(1000);

  mqttClient.setBufferSize(256);
  connectToMqtt();

  delay(3000);
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    connectToWiFi();
  }
  else
  {
    applyWifiChannel(getEspNowChannel());
  }

  if (!mqttClient.connected())
  {
    connectToMqtt();
  }
  else
  {
    mqttClient.loop();
  }

  const unsigned long now = millis();
  if (now - lastSensorReadMs >= SENSOR_READ_INTERVAL_MS)
  {
    lastSensorReadMs = now;
    readAndPublishSensors();
  }

  delay(10);
}
