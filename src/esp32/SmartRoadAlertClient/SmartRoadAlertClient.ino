/**
 * SmartRoadAlertClient.ino
 *
 * Smart Road Alert — ESP32 Firmware
 *
 * Responsibilities:
 *   1. Maintain the USB-serial handshake/heartbeat link with the host RPi
 *      (SERIAL_CONFIG — unchanged).
 *   2. Drive the P10 HUB75 LED matrix display (P10_LED_CONFIG — unchanged).
 *   3. Bridge HC-12 433 MHz radio traffic between the two RPis:
 *        RPi ──USB──► ESP32 ──HC-12──► remote ESP32 ──USB──► remote RPi
 *      • HC12_SEND  : RPi sends  {"type":"HC12_SEND","payload":"<escaped-json>"}
 *                     → ESP32 extracts & transmits payload via HC-12.
 *      • HC12_RECV  : HC-12 receives raw JSON from peer
 *                     → ESP32 wraps it as {"type":"HC12_RECV","payload":"<escaped-json>"}
 *                       and forwards to local RPi via USB.
 *   4. Send periodic vehicle telemetry to the RPi.
 *
 * Both ESP32 units run this identical firmware (symmetric).
 * All GPIO pin numbers are defined exclusively in PINS_CONFIG.h.
 */

#include "PINS_CONFIG.h"
#include "SERIAL_CONFIG.h"
#include "P10_LED_CONFIG.h"
#include "HC12_CONFIG.h"

static const unsigned long SENSOR_POLL_INTERVAL_MS    = 500UL;
static const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 500UL;

static unsigned long s_last_sensor_poll_ms = 0;
static unsigned long s_last_display_ms     = 0;

// Latest vehicle state, updated every sensor poll and reflected on the display
static float s_speed    = 0.0f;
static float s_distance = 0.0f;
static bool  s_safe     = true;

// ── Private prototypes ────────────────────────────────────────────────────────
static void handle_incoming_message(const SerialMessage &msg);
static void poll_and_send_sensor_data(void);
static void forward_hc12_to_rpi(const char *hc12_line);
static bool extract_json_string(const char *json, const char *key,
                                char *dst, size_t dstlen);
static void json_str_escape(char *dst, const char *src, size_t dstlen);

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    serial_init();
    setupP10();
    hc12_init();
    displayTextP10("Ready");
}

void loop() {
    serial_update();
    hc12_update();

    // Forward any HC-12 received frame to the RPi (wrapped as HC12_RECV)
    if (hc12_has_message()) {
        forward_hc12_to_rpi(hc12_get_message());
    }

    // Process commands/messages arriving from the RPi over USB
    if (serial_has_message()) {
        SerialMessage msg = serial_get_message();
        handle_incoming_message(msg);
    }

    if (serial_is_connected()) {
        poll_and_send_sensor_data();
    }

    // Refresh the P10 display at the configured interval
    unsigned long now = millis();
    if (now - s_last_display_ms >= DISPLAY_UPDATE_INTERVAL_MS) {
        s_last_display_ms = now;
        displayVehicleData(s_speed, s_distance, s_safe);
    }
}

// ─── HC-12 → RPi Bridge ───────────────────────────────────────────────────────

/**
 * Wrap a raw HC-12 received line in a HC12_RECV envelope and forward
 * it to the RPi over USB serial.
 *
 * Output format:
 *   {"type":"HC12_RECV","payload":"<json-escaped hc12_line>"}
 */
static void forward_hc12_to_rpi(const char *hc12_line) {
    char escaped[SERIAL_TX_BUFFER_SIZE];
    json_str_escape(escaped, hc12_line, sizeof(escaped));

    char out[SERIAL_TX_BUFFER_SIZE];
    snprintf(out, sizeof(out),
             "{\"type\":\"HC12_RECV\",\"payload\":\"%s\"}",
             escaped);
    serial_send(out);
}

// ─── RPi → HC-12 Bridge / Command Dispatcher ──────────────────────────────────

static void handle_incoming_message(const SerialMessage &msg) {
    if (!msg.valid || msg.raw[0] == '\0') {
        return;
    }

    // ── PING ──────────────────────────────────────────────────────────────────
    if (strstr(msg.raw, "\"cmd\":\"ping\"") != nullptr) {
        serial_send("{\"type\":\"pong\"}");
        return;
    }

    // ── STATUS ────────────────────────────────────────────────────────────────
    if (strstr(msg.raw, "\"cmd\":\"status\"") != nullptr) {
        serial_send("{\"type\":\"status\",\"state\":\"ok\"}");
        return;
    }

    // ── HC12_SEND — relay payload over the HC-12 radio link ──────────────────
    // Expected format: {"type":"HC12_SEND","payload":"<escaped-json-string>"}
    if (strstr(msg.raw, "\"type\":\"HC12_SEND\"") != nullptr) {
        char payload[SERIAL_RX_BUFFER_SIZE];
        if (extract_json_string(msg.raw, "payload", payload, sizeof(payload))) {
            hc12_send(payload);
        }
        return;
    }

    // ── HC12_TEST — diagnostic: send an arbitrary string over HC-12 ──────────
    // Expected format: {"cmd":"hc12_test","msg":"<text>"}
    if (strstr(msg.raw, "\"cmd\":\"hc12_test\"") != nullptr) {
        char test_msg[128];
        if (extract_json_string(msg.raw, "msg", test_msg, sizeof(test_msg))) {
            hc12_send(test_msg);
            serial_send("{\"type\":\"status\",\"state\":\"hc12_test_sent\"}");
        }
        return;
    }

    serial_send("{\"type\":\"error\",\"msg\":\"unknown command\"}");
}

// ─── Sensor Telemetry ─────────────────────────────────────────────────────────

static void poll_and_send_sensor_data(void) {
    unsigned long now = millis();

    // Rate-limit without blocking
    if (now - s_last_sensor_poll_ms < SENSOR_POLL_INTERVAL_MS) {
        return;
    }
    s_last_sensor_poll_ms = now;

    s_speed    = 42.0f + static_cast<float>(now % 10);
    s_distance = 12.3f + static_cast<float>(now % 5);
    s_safe     = (s_speed < 60.0f);

    // Build JSON payload
    char payload[SERIAL_TX_BUFFER_SIZE];
    snprintf(payload, sizeof(payload),
             "{\"type\":\"vehicle\",\"speed\":%.1f,\"distance\":%.1f}",
             s_speed, s_distance);
    serial_send(payload);
}

// ─── JSON String Utilities ────────────────────────────────────────────────────

/**
 * Extract the JSON string value for the first occurrence of @p key.
 *
 * Handles \" escape sequences within the value so that
 * {"payload":"{\"type\":\"RPI_PING\"}"} correctly returns {"type":"RPI_PING"}.
 *
 * Returns true on success; false if the key is not found in @p json.
 */
static bool extract_json_string(const char *json, const char *key,
                                char *dst, size_t dstlen) {
    // Build the search pattern: "key":"
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);

    const char *p = strstr(json, pattern);
    if (p == nullptr) return false;
    p += strlen(pattern);  // advance past the opening quote of the value

    size_t i = 0;
    while (*p != '\0' && i < dstlen - 1) {
        if (*p == '\\' && *(p + 1) != '\0') {
            // Handle escape sequence
            p++;
            switch (*p) {
                case '"':  dst[i++] = '"';  break;
                case '\\': dst[i++] = '\\'; break;
                case 'n':  dst[i++] = '\n'; break;
                case 'r':  dst[i++] = '\r'; break;
                case 't':  dst[i++] = '\t'; break;
                default:   dst[i++] = *p;   break;
            }
            p++;
        } else if (*p == '"') {
            break;  // unescaped closing quote — end of string value
        } else {
            dst[i++] = *p++;
        }
    }
    dst[i] = '\0';
    return true;
}

/**
 * JSON-escape @p src into @p dst so it can be safely embedded as a
 * JSON string value (between double-quote delimiters).
 *
 * Characters escaped:  "  \  \n  \r  \t
 * @p dstlen must include space for the null terminator.
 */
static void json_str_escape(char *dst, const char *src, size_t dstlen) {
    size_t i = 0;
    while (*src && i < dstlen - 2) {
        switch (*src) {
            case '"':  dst[i++] = '\\'; dst[i++] = '"';  src++; break;
            case '\\': dst[i++] = '\\'; dst[i++] = '\\'; src++; break;
            case '\n': dst[i++] = '\\'; dst[i++] = 'n';  src++; break;
            case '\r': dst[i++] = '\\'; dst[i++] = 'r';  src++; break;
            case '\t': dst[i++] = '\\'; dst[i++] = 't';  src++; break;
            default:   dst[i++] = *src++;                       break;
        }
    }
    dst[i] = '\0';
}

