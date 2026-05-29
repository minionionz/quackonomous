#include <Arduino.h>

enum MSG_TYPE {
    QUACK,
    STICK_DATA,
};

#define enum_name(x) case x: return #x;
const char* msg_type_name(MSG_TYPE m) {
    switch (m) {
        enum_name(QUACK)
        enum_name(STICK_DATA)
        default: break;
    }
    return "UNKNOWN";
}

struct __attribute__((packed)) StickData {
    uint16_t x;
    uint16_t y;
};

struct __attribute__((packed)) Message {
    MSG_TYPE msg_type;
    union {
        uint16_t i;
        StickData stick_data;
    } data;
};

void print_mac_address(const uint8_t *mac_addr) {
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac_addr[i]);
    if (i < 5)
      Serial.print(":");
  }
  Serial.print(" ");
}

