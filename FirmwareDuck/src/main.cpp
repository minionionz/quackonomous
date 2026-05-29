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
const int X_MAX = 4096;
const int Y_MIN = 0;
const int Y_MAX = 4096;

const int MOTOR_MIN = 1000; // Minimaler PWM-Wert für die Motoren = stop
const int MOTOR_MAX = 1600; // Maximaler PWM-Wert für die Motoren

const int X_DEADZONE = 80;  // Toter Bereich um die X-Nullstellung
const int Y_DEADZONE = 80;  // Toter Bereich um die Y-Nullstellung
const int X_NEUTRAL = 1840; // Gemessene X-Nullstellung
const int Y_NEUTRAL = 1880; // Gemessene Y-Nullstellung
const float STEERING_STRENGTH= 0.25; // WARNING: This is added to the MOTOR_MAX value!

const int Motor_R = 16;
const int Motor_L = 4;
const uint8_t DEFAULT_ESPNOW_CHANNEL = 1;

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
unsigned long lastPacketReceivedMs = 0;
unsigned long lastStickPacketReceivedMs = 0;
bool motorsStoppedByTimeout = false;
const unsigned long PACKET_TIMEOUT_MS = 20000;

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

  // Serial.print("[SENSORS] ");
  // Serial.println(payload);

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


float currentMotorLeft = 0;
float currentMotorRight = 0;
const float SECS_ZEROTOMAX = 1.5;
float accelerating_for = 0.0;
// MOTOR_MIN
int time_last_received;
const int TIMEOUT_STOP_AFTER_RECV = 1000;

// We don't have floating points, so this is for mapping like 0.0 to 1.0.
#define FULL 1000000
#define bool_str(x) (x > 0)? "TRUE":"FALSE" // For debug printing
float prev_millis = 0;
static void driveEscFromStick(const StickData &stickData)
{
  unsigned long mills = millis();
  // For calculating acceleration based on time
  float delta = (mills - prev_millis) / 1000.0;
  prev_millis = mills;
  accelerating_for += delta;

  bool was_going_forward = currentMotorLeft >  0 && currentMotorRight >  0;
  bool was_going_left    = currentMotorLeft <= 0 && currentMotorRight >  0;
  bool was_going_right   = currentMotorLeft >  0 && currentMotorRight <= 0;
  Serial.printf("Was going: forward: %s | left: %s | right: %s\n",
      bool_str(was_going_forward),
      bool_str(was_going_left),
      bool_str(was_going_right)
  );

  // Reset motors when not receiving anything for some time
  // THIS DOES NOT WORK THE WAY WE THING IT DOES. THIS GETS EXECUTED ONLY ON THE PACKAGE AFTER A LONG PAUSE.
  if (mills - time_last_received > TIMEOUT_STOP_AFTER_RECV) {
    Serial.println("Didn't receive a package since a long time. Resetting acceleration.");
    accelerating_for = 0;
  }
  time_last_received = mills;

  // TODO: Gegenlenken wenn man aufhört, rechts zu lenken.
  const int x = stickData.x;
  const int y = stickData.y;

  // Values sent by the remote. Ranges from 0 to 1
  float goalMotorLeft  = 0;
  float goalMotorRight = 0;

  /* Map data to motor direction */
  bool is_going_left, is_going_right, is_going_forward;
  float throttle = 0;
  const float DIFF_BUFFER = 0.01;
  if (y > Y_NEUTRAL + Y_DEADZONE) {
    throttle      = constrain(map(y, Y_NEUTRAL + Y_DEADZONE, Y_MAX, 0, FULL), 0, FULL) / (float)FULL;
    is_going_forward = throttle > DIFF_BUFFER;
  }
  float steeringDelta = 0;
  // Driving left: goalMotorRight>LEAST
  if (x < X_NEUTRAL - X_DEADZONE) {
    steeringDelta =   STEERING_STRENGTH * (constrain(map(x, X_NEUTRAL - X_DEADZONE, X_MIN, 0, FULL), 0, FULL) / (float) FULL);
    is_going_left = steeringDelta > DIFF_BUFFER;
  }
  // Driving right: goalMotorLeft>LEAST
  else if (x > X_NEUTRAL + X_DEADZONE) {
    steeringDelta = - STEERING_STRENGTH * (constrain(map(x, X_NEUTRAL + X_DEADZONE, X_MAX, 0, FULL), 0, FULL) / (float) FULL);
    is_going_right = - steeringDelta > DIFF_BUFFER;
  }

  // Add left/right direction to throttle
  Serial.printf("THROT: %f | DELTA: %f\n", throttle, steeringDelta);
  // Add steering speed as a bias, so we don't get negative values when turning

  // TODO: calc accel only onto throttle
  // Accelerating
  // TODO: if (was_going_left && not is_going_forward) ...
  float E = 5;
  // from 0 to 1.
  float accelProgress = constrain(pow(E, (accelerating_for / SECS_ZEROTOMAX) - 1), 0.0, throttle);
  Serial.printf("GOING: %f / %f | PROGRESS: %f\n", accelerating_for, SECS_ZEROTOMAX, accelProgress);

  // The accelerated base speed + the full rotating speed
  float real_throttle = constrain(accelProgress, 0, throttle);
  // !!! So this might go _over_ the MOTOR_MAX value !!!
  goalMotorLeft  = real_throttle - steeringDelta;
  goalMotorRight = real_throttle + steeringDelta;

  // TODO: Data for later
  if (!is_going_forward) {
    accelerating_for = 0;
  }

  Serial.printf("Is going: Forward: %s %f | Left: %s %f | Right: %s %f\n",
      bool_str(is_going_forward), real_throttle,
      bool_str(is_going_left),  goalMotorLeft,
      bool_str(is_going_right), goalMotorRight
  );

  currentMotorLeft  = constrain(goalMotorLeft, 0, 1.0 + STEERING_STRENGTH);
  currentMotorRight = constrain(goalMotorRight, 0, 1.0 + STEERING_STRENGTH);
  Serial.printf("Left Motor going at: %f %%  |\t  ", currentMotorLeft);
  Serial.printf("Right Motor going at: %f %%\n", currentMotorRight);

  esc_L.writeMicroseconds(MOTOR_MIN + currentMotorLeft  * (MOTOR_MAX - MOTOR_MIN));
  esc_R.writeMicroseconds(MOTOR_MIN + currentMotorRight * (MOTOR_MAX - MOTOR_MIN));

  if (mqttClient.connected())
  {
    String payload = String("{\"x\":") + x + String(",\"y\":") + y + String(",\"left\":") + goalMotorLeft + String(",\"right\":") + goalMotorRight + String("}");
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
      lastPacketReceivedMs = millis();
      lastStickPacketReceivedMs = lastPacketReceivedMs;
      // if (motorsStoppedByTimeout)
      // {
      //   motorsStoppedByTimeout = false;
      //   publishState(MQTT_STATUS_TOPIC, "motors resumed");
      // }
    }
    else if (message.msg_type == QUACK)
    {
      Serial.print(" i=");
      Serial.print(message.data.i);

      if (mqttClient.connected())
      {
        publishState(MQTT_QUACK_TOPIC, String(message.data.i));
      }
      lastPacketReceivedMs = millis();
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

  // // Motoren stoppen, wenn seit PACKET_TIMEOUT_MS keine Fahrdaten mehr kamen
  // if (lastStickPacketReceivedMs != 0 && (now - lastStickPacketReceivedMs > PACKET_TIMEOUT_MS))
  // {
  //   if (!motorsStoppedByTimeout)
  //   {
  //     esc_L.writeMicroseconds(MOTOR_MIN);
  //     esc_R.writeMicroseconds(MOTOR_MIN);
  //     motorsStoppedByTimeout = true;
  //     publishState(MQTT_STATUS_TOPIC, "motors timeout");
  //   }
  // }

  delay(10);
}
