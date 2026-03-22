#include "HC12_CONFIG.h"

#include <HardwareSerial.h>
#include <string.h>

// ── Internal state ────────────────────────────────────────────────────────────

static HardwareSerial s_hc12(2);           // ESP32 Hardware Serial2

static char     s_rx_buf[HC12_RX_BUF];    // Accumulation buffer (incomplete line)
static uint16_t s_rx_idx    = 0;           // Write index into s_rx_buf

static char     s_msg_buf[HC12_RX_BUF];   // Last complete received line
static bool     s_msg_ready = false;       // True when s_msg_buf holds new data

// AT command timing constants (milliseconds, per HC-12 datasheet)
static const uint16_t AT_ASSERT_MS  = 50;   // Wait after pulling SET LOW
static const uint16_t AT_RELEASE_MS = 80;   // Wait after releasing SET HIGH
static const uint16_t AT_TIMEOUT_MS = 1000; // Per-command response deadline

// ── Private prototype ─────────────────────────────────────────────────────────
static bool _at_cmd(const char *cmd, const char *expected_prefix);

// ─────────────────────────────────────────────────────────────────────────────
// hc12_init
// ─────────────────────────────────────────────────────────────────────────────

void hc12_init(void) {
    // SET pin HIGH → transparent (normal) mode on power-up
    pinMode(PIN_HC12_SET, OUTPUT);
    digitalWrite(PIN_HC12_SET, HIGH);
    delay(AT_RELEASE_MS);

    // Open UART2 — the same baud rate is used for both AT commands and
    // transparent data, so no speed switch is needed between modes.
    s_hc12.begin(HC12_BAUD, SERIAL_8N1, PIN_HC12_RX, PIN_HC12_TX);
    delay(100);

    // ── Enter AT-command mode ─────────────────────────────────────────────
    digitalWrite(PIN_HC12_SET, LOW);
    delay(AT_ASSERT_MS);

    bool ok = _at_cmd("AT", "OK");
    if (!ok) {
        Serial.println(F("[HC12] AT handshake failed — module using factory defaults."));
    } else {
        char cmd[16];

        // Set UART baud rate (matches HC12_BAUD)
        snprintf(cmd, sizeof(cmd), "AT+B%d", HC12_BAUD);
        if (!_at_cmd(cmd, "OK+B")) {
            Serial.println(F("[HC12] WARN: AT+Bxxxx not acknowledged."));
        }

        // Set channel (3-digit, zero-padded)
        snprintf(cmd, sizeof(cmd), "AT+C%03d", HC12_CHANNEL);
        if (!_at_cmd(cmd, "OK+C")) {
            Serial.println(F("[HC12] WARN: AT+Cxxx not acknowledged."));
        }

        // Set operating mode FU3 (normal-speed, full-function)
        if (!_at_cmd("AT+FU3", "OK+FU3")) {
            Serial.println(F("[HC12] WARN: AT+FU3 not acknowledged."));
        }

        // Set transmit power level
        snprintf(cmd, sizeof(cmd), "AT+P%d", HC12_POWER);
        if (!_at_cmd(cmd, "OK+P")) {
            Serial.println(F("[HC12] WARN: AT+Px not acknowledged."));
        }

        Serial.println(F("[HC12] Configuration applied."));
    }

    // ── Return to transparent mode ────────────────────────────────────────
    digitalWrite(PIN_HC12_SET, HIGH);
    delay(AT_RELEASE_MS);

    Serial.println(F("[HC12] Ready."));
}

// ─────────────────────────────────────────────────────────────────────────────
// hc12_update  — call every loop() iteration
// ─────────────────────────────────────────────────────────────────────────────

void hc12_update(void) {
    while (s_hc12.available() > 0) {
        char c = static_cast<char>(s_hc12.read());

        if (c == '\n') {
            // Newline terminates a message frame
            if (s_rx_idx > 0) {
                s_rx_buf[s_rx_idx] = '\0';
                strncpy(s_msg_buf, s_rx_buf, HC12_RX_BUF - 1);
                s_msg_buf[HC12_RX_BUF - 1] = '\0';
                s_msg_ready = true;
                s_rx_idx    = 0;
            }
        } else if (c == '\r') {
            // Carriage-return silently ignored
        } else {
            if (s_rx_idx < HC12_RX_BUF - 1) {
                s_rx_buf[s_rx_idx++] = c;
            } else {
                // Buffer overflow — discard the partial frame and reset
                s_rx_idx = 0;
                Serial.println(F("[HC12] WARN: RX buffer overflow — frame discarded."));
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// hc12_send
// ─────────────────────────────────────────────────────────────────────────────

bool hc12_send(const char *message) {
    if (message == nullptr || *message == '\0') return false;
    // println() appends \r\n; HC-12 transparent mode passes bytes as-is.
    s_hc12.println(message);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// hc12_has_message / hc12_get_message
// ─────────────────────────────────────────────────────────────────────────────

bool hc12_has_message(void) {
    return s_msg_ready;
}

const char *hc12_get_message(void) {
    s_msg_ready = false;
    return s_msg_buf;
}

// ─────────────────────────────────────────────────────────────────────────────
// _at_cmd — private AT command helper
//
// Sends @p cmd (appending \r\n), then reads response lines until one starts
// with @p expected_prefix or the deadline expires.
// Returns true on a matching response, false on timeout/mismatch.
// ─────────────────────────────────────────────────────────────────────────────

static bool _at_cmd(const char *cmd, const char *expected_prefix) {
    s_hc12.println(cmd);

    uint32_t deadline = millis() + AT_TIMEOUT_MS;
    char     resp[64];
    uint8_t  idx = 0;

    while ((long)(millis() - deadline) < 0) {
        if (!s_hc12.available()) {
            delay(5);
            continue;
        }
        char c = static_cast<char>(s_hc12.read());
        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                resp[idx] = '\0';
                if (strncmp(resp, expected_prefix, strlen(expected_prefix)) == 0) {
                    return true;
                }
                idx = 0;  // try next line
            }
        } else if (idx < static_cast<uint8_t>(sizeof(resp) - 1)) {
            resp[idx++] = c;
        }
    }
    return false;
}
