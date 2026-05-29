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
const uint8_t WIFI_CHANNEL = 6;

const char *WIFI_SSID = "Ducknet";
const char *WIFI_PASSWORD = "Ducknet123";

const char *MQTT_BROKER = "10.42.0.1";
const uint16_t MQTT_PORT = 1883;
const char *MQTT_CLIENT_ID = "duck-firmware";
const char *MQTT_STATUS_TOPIC = "duck/status";
const char *MQTT_STICK_TOPIC = "duck/stick";
const char *MQTT_QUACK_TOPIC = "duck/quack";
const char *MQTT_COMMAND_TOPIC = "duck/cmd";

unsigned long lastMqttReconnectAttempt = 0;

static void applyWifiChannel()
{
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
}

static void publishState(const char *topic, const String &payload)
{
  if (mqttClient.connected())
  {
    mqttClient.publish(topic, payload.c_str(), true);
  }
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
  WiFi.mode(WIFI_STA);
  applyWifiChannel();
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
    applyWifiChannel();
    Serial.print("WiFi verbunden, IP: ");
    Serial.println(WiFi.localIP());
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
  for (int i = 0; i < 6; i++)
  {
    printf("%02X", mac_addr[i]);
    if (i < 5)
      Serial.print(":");
  }

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

  connectToWiFi();
  delay(2000); // Delay for monitor

  // Zur Information: Die MAC-Adresse des ESP32 wird hier ausgegeben,
  // damit du sie für den Sender verwenden kannst
  Serial.print("Empfänger MAC: ");
  Serial.println(WiFi.macAddress());

  // Initialisiere ESP-NOW
  applyWifiChannel();
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

  if (!mqttClient.connected())
  {
    connectToMqtt();
  }
  else
  {
    mqttClient.loop();
  }

  delay(2000);
}
