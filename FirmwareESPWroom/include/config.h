#pragma once

// MAC address of the Duck ESP32 (the receiver).
// Find it by running Serial.println(WiFi.macAddress()) on the Duck board.
// Format: { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }
static const uint8_t DUCK_MAC[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
