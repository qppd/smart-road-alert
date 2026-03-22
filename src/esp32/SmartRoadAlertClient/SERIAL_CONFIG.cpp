#include "SERIAL_CONFIG.h"
#include <string.h>

static char          s_rx_buf[SERIAL_RX_BUFFER_SIZE];
static uint16_t      s_rx_idx            = 0;


static SerialMessage s_pending_msg;
static bool          s_msg_ready         = false;
static SerialConnectionState s_state     = SERIAL_DISCONNECTED;
static unsigned long s_last_heartbeat_ms = 0;
static unsigned long s_handshake_ts_ms   = 0;

/** millis() timestamp of the last message received from the host. */
static unsigned long s_last_host_rx_ms   = 0;

// ─── Private Helper Declarations ─────────────────────────────────────────────

static void _dispatch_line(const char *line);
static void _on_handshake_hello(void);
static void _tick_handshake(void);
static void _tick_connected(void);


void serial_init(void) {
    Serial.begin(SERIAL_BAUD_RATE);
    
    while (!Serial);

    s_rx_idx            = 0;
    s_msg_ready         = false;
    s_state             = SERIAL_WAITING_HANDSHAKE;
    s_handshake_ts_ms   = millis();
    s_last_heartbeat_ms = millis();
    s_last_host_rx_ms   = millis();

    Serial.println(F("[SERIAL] Initialised — awaiting HELLO from host."));
}

void serial_update(void) {
   
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());

        if (c == MSG_TERMINATOR) {
          
            s_rx_buf[s_rx_idx] = '\0';
            if (s_rx_idx > 0) {
                _dispatch_line(s_rx_buf);
            }
            s_rx_idx = 0;

        } else if (c == '\r') {
           

        } else {
            if (s_rx_idx < SERIAL_RX_BUFFER_SIZE - 1) {
                s_rx_buf[s_rx_idx++] = c;
            } else {
            
                s_rx_idx = 0;
                Serial.println(F("[SERIAL] WARN: RX buffer overflow — frame discarded."));
            }
        }
    }

    switch (s_state) {
        case SERIAL_WAITING_HANDSHAKE:
            _tick_handshake();
            break;
        case SERIAL_CONNECTED:
            _tick_connected();
            break;
        default:
            break;
    }
}

void serial_send(const char *message) {
    if (s_state != SERIAL_CONNECTED) {
        
        return;
    }
    Serial.println(message);
}

bool serial_has_message(void) {
    return s_msg_ready;
}

SerialMessage serial_get_message(void) {
    s_msg_ready = false;
    return s_pending_msg;
}

SerialConnectionState serial_get_state(void) {
    return s_state;
}

bool serial_is_connected(void) {
    return (s_state == SERIAL_CONNECTED);
}
static void _dispatch_line(const char *line) {
    if (s_state == SERIAL_WAITING_HANDSHAKE) {
        if (strcmp(line, HANDSHAKE_HELLO) == 0) {
            _on_handshake_hello();
        }
       

    } else if (s_state == SERIAL_CONNECTED) {
        /* Track host activity for silence detection. */
        s_last_host_rx_ms = millis();

        /*
         * Check whether the host has re-initiated handshake (e.g. after
         * reconnect on its side).  If so, re-run the handshake rather
         * than treating HELLO as a JSON message.
         */

        if (strcmp(line, HANDSHAKE_HELLO) == 0) {
            Serial.println(F("[SERIAL] Re-handshake requested by host."));
            _on_handshake_hello();
            return;
        }

        strncpy(s_pending_msg.raw, line, SERIAL_RX_BUFFER_SIZE - 1);
        s_pending_msg.raw[SERIAL_RX_BUFFER_SIZE - 1] = '\0';
        s_pending_msg.valid = true;
        s_msg_ready         = true;
    }
}

static void _on_handshake_hello(void) {
    Serial.println(HANDSHAKE_READY);   
    s_state             = SERIAL_CONNECTED;
    s_last_heartbeat_ms = millis();
    s_last_host_rx_ms   = millis();   /* reset silence timer on new connection */
    Serial.println(F("[SERIAL] Handshake complete — connected."));
}

static void _tick_handshake(void) {
    unsigned long now = millis();
    if (now - s_handshake_ts_ms >= HANDSHAKE_TIMEOUT_MS) {
        s_handshake_ts_ms = now;
        s_rx_idx = 0;   
        Serial.println(F("[SERIAL] Awaiting HELLO from host..."));
    }
}

static void _tick_connected(void) {
    unsigned long now = millis();
    if (now - s_last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS) {
        s_last_heartbeat_ms = now;
       
        Serial.println(F("{\"type\":\"heartbeat\"}"));
    }

    /* Host-silence detection: if the RPi stops sending (disconnect, crash,
     * etc.) reset to WAITING_HANDSHAKE so the next HELLO re-establishes the
     * link without needing a full ESP32 hardware reset. */
#if HOST_SILENCE_TIMEOUT_MS > 0
    if (now - s_last_host_rx_ms >= HOST_SILENCE_TIMEOUT_MS) {
        s_state           = SERIAL_WAITING_HANDSHAKE;
        s_handshake_ts_ms = now;
        s_rx_idx          = 0;
        Serial.println(F("[SERIAL] Host silent — reset to WAITING_HANDSHAKE."));
    }
#endif
}
