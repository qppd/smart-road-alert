/**
 * SmartRoadAlertClient.ino
 *
 * Smart Road Alert — ESP32 Firmware
 *
 * Main firmware entry-point.
 *
 * Responsibilities of this file:
 *   - Call serial_init() once in setup().
 *   - Call serial_update() every loop iteration (drives all serial I/O).
 *   - Dispatch received commands via handle_incoming_message().
 *   - Periodically collect sensor data and transmit via serial_send().
 *
 * All serial I/O details are encapsulated in SERIAL_CONFIG.h / SERIAL_CONFIG.cpp.
 * This file must NOT call Serial directly.
 *
 * Board:   ESP32 (any variant)
 * BaudRate: 115200
 * Protocol: Newline-terminated JSON
 */

#include "SERIAL_CONFIG.h"

// ─── Application Timing Constants ─────────────────────────────────────────────

/** Interval between sensor readings (milliseconds). */
static const unsigned long SENSOR_POLL_INTERVAL_MS = 500UL;

// ─── Application State ────────────────────────────────────────────────────────

/** millis() timestamp of the last sensor transmission. */
static unsigned long s_last_sensor_poll_ms = 0;

// ─── Private Function Declarations ────────────────────────────────────────────

static void handle_incoming_message(const SerialMessage &msg);
static void poll_and_send_sensor_data(void);

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    // Initialise serial communication module.
    // All baud-rate configuration is inside SERIAL_CONFIG.
    serial_init();

    // TODO: Initialise additional peripherals here (GPS, ultrasonic sensor, etc.)
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
    // ── 1. Drive serial state machine (non-blocking, must be called every loop) ──
    serial_update();

    // ── 2. Consume completed inbound messages ──
    if (serial_has_message()) {
        SerialMessage msg = serial_get_message();
        handle_incoming_message(msg);
    }

    // ── 3. Send sensor telemetry when the host is connected ──
    if (serial_is_connected()) {
        poll_and_send_sensor_data();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// handle_incoming_message()
//
// Parses and dispatches JSON command objects received from the Raspberry Pi.
//
// For a production system, replace strstr() matching with ArduinoJson parsing:
//   https://arduinojson.org/
// ─────────────────────────────────────────────────────────────────────────────

static void handle_incoming_message(const SerialMessage &msg) {
    // Guard: skip invalid messages
    if (!msg.valid || msg.raw[0] == '\0') {
        return;
    }

    // ── ping ──
    if (strstr(msg.raw, "\"cmd\":\"ping\"") != nullptr) {
        serial_send("{\"type\":\"pong\"}");
        return;
    }

    // ── status request ──
    if (strstr(msg.raw, "\"cmd\":\"status\"") != nullptr) {
        serial_send("{\"type\":\"status\",\"state\":\"ok\"}");
        return;
    }

    // ── unknown command — log for debugging ──
    // (Do NOT call Serial.print directly — use serial_send for JSON responses.)
    serial_send("{\"type\":\"error\",\"msg\":\"unknown command\"}");
}

// ─────────────────────────────────────────────────────────────────────────────
// poll_and_send_sensor_data()
//
// Reads sensor hardware (or stubs) and transmits a JSON telemetry packet.
// Called only when connected; respects SENSOR_POLL_INTERVAL_MS non-blockingly.
// ─────────────────────────────────────────────────────────────────────────────

static void poll_and_send_sensor_data(void) {
    unsigned long now = millis();

    // Rate-limit without blocking
    if (now - s_last_sensor_poll_ms < SENSOR_POLL_INTERVAL_MS) {
        return;
    }
    s_last_sensor_poll_ms = now;

    // ── Sensor readings ──
    // Replace these stub values with real hardware calls:
    //   speed    — from GPS module (e.g. TinyGPS++ parsed NMEA)
    //   distance — from ultrasonic or LIDAR sensor
    float speed    = 42.0f + static_cast<float>(now % 10);
    float distance = 12.3f + static_cast<float>(now % 5);

    // ── Build JSON payload ──
    char payload[SERIAL_TX_BUFFER_SIZE];
    snprintf(
        payload,
        sizeof(payload),
        "{\"type\":\"vehicle\",\"speed\":%.1f,\"distance\":%.1f}",
        speed,
        distance
    );

    serial_send(payload);
}

