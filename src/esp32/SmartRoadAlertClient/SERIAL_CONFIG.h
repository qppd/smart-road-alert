#ifndef SERIAL_CONFIG_H
#define SERIAL_CONFIG_H

#include <Arduino.h>

#define SERIAL_BAUD_RATE          115200
#define SERIAL_RX_BUFFER_SIZE     512
#define SERIAL_TX_BUFFER_SIZE     256
#define HANDSHAKE_TIMEOUT_MS      5000UL

#define HEARTBEAT_INTERVAL_MS     2000UL
#define MSG_TERMINATOR            '\n'


#define HANDSHAKE_HELLO           "HELLO"
#define HANDSHAKE_READY           "ESP32_READY"

typedef enum {
    SERIAL_DISCONNECTED      = 0,
    SERIAL_WAITING_HANDSHAKE = 1,
    SERIAL_CONNECTED         = 2
} SerialConnectionState;

typedef struct {
    char raw[SERIAL_RX_BUFFER_SIZE];
    bool valid;
} SerialMessage;

void serial_init(void);

void serial_update(void);

void serial_send(const char *message);

bool serial_has_message(void);

SerialMessage serial_get_message(void);

SerialConnectionState serial_get_state(void);

bool serial_is_connected(void);

#endif 
