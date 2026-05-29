#include <Arduino.h>

enum MSG_TYPE {
    QUACK,
    STICK_DATA,
};

const char* msg_type_name(MSG_TYPE) {
}

struct __attribute((packed)) Message {
    MSG_TYPE msg_type;
    union {
        uint16_t i;
        struct {
          uint16_t x;
          uint16_t y;

        };
    } data;
};

