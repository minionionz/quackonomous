#pragma once

/* ── WiFi ────────────────────────────────────────────────────────────────── */
#define WIFI_SSID       "Ducknet"
#define WIFI_PASSWORD   "Ducknet123"
#define WIFI_SECURITY   CY_WCM_SECURITY_WPA2_AES_PSK

/* ── MQTT broker (plain TCP, port 1883) ─────────────────────────────────── */
/* Run:  mosquitto -v  on your laptop, then put its IP here. */
#define MQTT_BROKER     "10.42.0.1"
#define MQTT_PORT       (1883u)
#define MQTT_CLIENT_ID  "sensor-firmware"

/* ── Topic root ─────────────────────────────────────────────────────────── */
/* Published topics: duck/infineon/t, /imu/accel/x, /mag/hdg, /ok/imu, ...   */
#define MQTT_TOPIC_ROOT "duck/infineon"
