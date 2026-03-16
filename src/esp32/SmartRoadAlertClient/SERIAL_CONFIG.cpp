/**
 * SERIAL_CONFIG.cpp
 *
 * Smart Road Alert — ESP32 Serial Communication Module (Implementation)
 *
 * All serial I/O is non-blocking.  Timing is driven via millis().
 * No delay() or blocking while-loops appear anywhere in this file.
 *
 * Internal state machine:
 *
 *   SERIAL_WAITING_HANDSHAKE
 *       │  (received "HELLO" from host)
 *       ▼
 *   SERIAL_CONNECTED
 *       │  (heartbeat, normal message exchange)
 *       (host can reconnect by re-sending "HELLO" — state resets)
 */

#include "SERIAL_CONFIG.h"
#include <string.h>

// ─── Module-Private State ─────────────────────────────────────────────────────

/** Receive ring buffer and write index. */
static char          s_rx_buf[SERIAL_RX_BUFFER_SIZE];
static uint16_t      s_rx_idx            = 0;

/** Single-slot message mailbox for the application layer. */
static SerialMessage s_pending_msg;
static bool          s_msg_ready         = false;

/** Current connection state. */
static SerialConnectionState s_state     = SERIAL_DISCONNECTED;

/** millis() timestamp of last heartbeat transmission. */
static unsigned long s_last_heartbeat_ms = 0;

/** millis() timestamp when the current handshake wait began. */
static unsigned long s_handshake_ts_ms   = 0;

// ─── Private Helper Declarations ─────────────────────────────────────────────

static void _dispatch_line(const char *line);
static void _on_handshake_hello(void);
static void _tick_handshake(void);
static void _tick_connected(void);

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

void serial_init(void) {
    Serial.begin(SERIAL_BAUD_RATE);
    /* Wait for the USB CDC port to enumerate (hardware requirement). */
    while (!Serial) { /* intentional spin — only runs once during boot */ }

    s_rx_idx            = 0;
    s_msg_ready         = false;
    s_state             = SERIAL_WAITING_HANDSHAKE;
    s_handshake_ts_ms   = millis();
    s_last_heartbeat_ms = millis();

    Serial.println(F("[SERIAL] Initialised — awaiting HELLO from host."));
}

void serial_update(void) {
    /* ── 1. Drain hardware FIFO into line buffer (non-blocking) ── */
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());

        if (c == MSG_TERMINATOR) {
            /* End of frame — null-terminate and dispatch. */
            s_rx_buf[s_rx_idx] = '\0';
            if (s_rx_idx > 0) {
                _dispatch_line(s_rx_buf);
            }
            s_rx_idx = 0;

        } else if (c == '\r') {
            /* Ignore carriage-return (present in Windows line endings). */

        } else {
            if (s_rx_idx < SERIAL_RX_BUFFER_SIZE - 1) {
                s_rx_buf[s_rx_idx++] = c;
            } else {
                /* Buffer overflow — discard accumulated data and start fresh. */
                s_rx_idx = 0;
                Serial.println(F("[SERIAL] WARN: RX buffer overflow — frame discarded."));
            }
        }
    }

    /* ── 2. Drive periodic state logic ── */
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
        /* Silently drop — host is not yet connected. */
        return;
    }
    /* Serial.println() appends "\r\n"; the '\n' is the protocol frame terminator. */
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

// ─────────────────────────────────────────────────────────────────────────────
// Private Helpers
// ─────────────────────────────────────────────────────────────────────────────

/**
 * _dispatch_line
 *
 * Called whenever a complete, null-terminated line has been received.
 * Routes the line to the appropriate handler based on current state.
 */
static void _dispatch_line(const char *line) {
    if (s_state == SERIAL_WAITING_HANDSHAKE) {
        if (strcmp(line, HANDSHAKE_HELLO) == 0) {
            _on_handshake_hello();
        }
        /* Any other data during handshake is ignored. */

    } else if (s_state == SERIAL_CONNECTED) {
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

        /* Store in mailbox for the application layer. */
        strncpy(s_pending_msg.raw, line, SERIAL_RX_BUFFER_SIZE - 1);
        s_pending_msg.raw[SERIAL_RX_BUFFER_SIZE - 1] = '\0';
        s_pending_msg.valid = true;
        s_msg_ready         = true;
    }
}

/**
 * _on_handshake_hello
 *
 * Responds to the host's HELLO and transitions to CONNECTED state.
 */
static void _on_handshake_hello(void) {
    Serial.println(HANDSHAKE_READY);   /* Send ESP32_READY to host. */
    s_state             = SERIAL_CONNECTED;
    s_last_heartbeat_ms = millis();
    Serial.println(F("[SERIAL] Handshake complete — connected."));
}

/**
 * _tick_handshake
 *
 * Non-blocking periodic logic while waiting for the host to connect.
 * Prints a status reminder every HANDSHAKE_TIMEOUT_MS milliseconds
 * so that a newly attached host can see the device is alive.
 */
static void _tick_handshake(void) {
    unsigned long now = millis();
    if (now - s_handshake_ts_ms >= HANDSHAKE_TIMEOUT_MS) {
        s_handshake_ts_ms = now;
        s_rx_idx = 0;   /* Clear any partial garbage in the buffer. */
        Serial.println(F("[SERIAL] Awaiting HELLO from host..."));
    }
}

/**
 * _tick_connected
 *
 * Non-blocking periodic logic while in the connected state.
 * Transmits a lightweight heartbeat JSON packet at a fixed interval
 * so the host can detect link-loss.
 */
static void _tick_connected(void) {
    unsigned long now = millis();
    if (now - s_last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS) {
        s_last_heartbeat_ms = now;
        /* Heartbeat is a valid JSON object — host can parse or ignore it. */
        Serial.println(F("{\"type\":\"heartbeat\"}"));
    }
}
