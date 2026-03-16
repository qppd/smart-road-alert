/**
 * SERIAL_CONFIG.h
 *
 * Smart Road Alert — ESP32 Serial Communication Module
 *
 * Handles:
 *   - Serial initialization
 *   - Non-blocking incremental receive with buffer
 *   - Message send / receive API
 *   - Handshake state machine
 *   - Connection state tracking
 *   - Periodic heartbeat
 *
 * No blocking calls (delay / while-wait) are used anywhere in this module.
 * Communication timing is driven entirely by millis().
 */

#ifndef SERIAL_CONFIG_H
#define SERIAL_CONFIG_H

#include <Arduino.h>

// ─── Compile-time Configuration ───────────────────────────────────────────────

/** Serial baud rate. Must match Raspberry Pi setting. */
#define SERIAL_BAUD_RATE          115200

/** Maximum length of a single received line (bytes, including null terminator). */
#define SERIAL_RX_BUFFER_SIZE     512

/** Maximum length of a single message to transmit (bytes). */
#define SERIAL_TX_BUFFER_SIZE     256

/**
 * Milliseconds to wait for HELLO before re-printing the "awaiting" notice.
 * Does NOT block — checked inside serial_update().
 */
#define HANDSHAKE_TIMEOUT_MS      5000UL

/** Milliseconds between automatic heartbeat JSON packets. */
#define HEARTBEAT_INTERVAL_MS     2000UL

/**
 * Milliseconds without any message from the host before resetting to
 * SERIAL_WAITING_HANDSHAKE.  Allows the ESP32 to recover cleanly if the
 * Raspberry Pi reconnects without hardware-resetting the ESP32.
 * Must be safely larger than the RPi's PING_INTERVAL_S (5 s × 1000).
 * Set to 0UL to disable the timeout.
 */
#define HOST_SILENCE_TIMEOUT_MS   15000UL

/** Character that terminates every message frame. */
#define MSG_TERMINATOR            '\n'

// ─── Handshake Tokens ─────────────────────────────────────────────────────────

/** Token sent by the Raspberry Pi to initiate handshake. */
#define HANDSHAKE_HELLO           "HELLO"

/** Token sent by this device to confirm readiness. */
#define HANDSHAKE_READY           "ESP32_READY"

// ─── Types ────────────────────────────────────────────────────────────────────

/**
 * Connection state machine states.
 *
 * SERIAL_DISCONNECTED     — Serial port not yet initialised.
 * SERIAL_WAITING_HANDSHAKE — Waiting for HELLO from host.
 * SERIAL_CONNECTED        — Handshake complete; normal operation.
 */
typedef enum {
    SERIAL_DISCONNECTED      = 0,
    SERIAL_WAITING_HANDSHAKE = 1,
    SERIAL_CONNECTED         = 2
} SerialConnectionState;

/**
 * Container for a fully received, null-terminated message.
 *
 * raw[]  — null-terminated UTF-8 string (JSON line).
 * valid  — true when the struct contains an unread message.
 */
typedef struct {
    char raw[SERIAL_RX_BUFFER_SIZE];
    bool valid;
} SerialMessage;

// ─── Public API ───────────────────────────────────────────────────────────────

/**
 * Initialise the serial port and reset module state.
 * Call once from setup().
 */
void serial_init(void);

/**
 * Drive the serial state machine.
 * Call every iteration of loop() — never blocks.
 */
void serial_update(void);

/**
 * Enqueue a null-terminated UTF-8 string for transmission.
 * A newline is appended automatically.
 * Safe to call only when serial_is_connected() returns true.
 *
 * @param message  Null-terminated string to send (must fit in SERIAL_TX_BUFFER_SIZE).
 */
void serial_send(const char *message);

/**
 * Returns true if a complete, unread message is available.
 * Call serial_get_message() to consume it.
 */
bool serial_has_message(void);

/**
 * Returns the pending message and clears the ready flag.
 * Always check serial_has_message() first.
 */
SerialMessage serial_get_message(void);

/** Returns the current connection state enum value. */
SerialConnectionState serial_get_state(void);

/** Convenience wrapper — returns true only when fully connected. */
bool serial_is_connected(void);

#endif /* SERIAL_CONFIG_H */
