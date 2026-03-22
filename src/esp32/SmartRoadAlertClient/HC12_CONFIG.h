#ifndef HC12_CONFIG_H
#define HC12_CONFIG_H

/**
 * HC12_CONFIG.h
 *
 * Smart Road Alert — HC-12 433 MHz Wireless Bridge (ESP32 side)
 *
 * The ESP32 acts as a transparent bridge between the Raspberry Pi (USB)
 * and the remote RPi's ESP32 (HC-12 radio).
 *
 * Architecture:
 *   RPi  ──USB──►  ESP32  ──HC-12 radio──►  remote ESP32  ──USB──►  remote RPi
 *
 * Message flow (RPi → remote RPi):
 *   RPi sends:  {"type":"HC12_SEND","payload":"<escaped-json-string>"}
 *   ESP32 extracts payload, transmits the raw JSON string over HC-12.
 *
 * Message flow (remote RPi → local RPi):
 *   Remote ESP32 sends raw JSON over HC-12.
 *   Local ESP32 wraps it:  {"type":"HC12_RECV","payload":"<escaped-json-string>"}
 *   Local ESP32 forwards the wrapped message to its RPi via USB.
 *
 * Hardware wiring:
 *   HC-12 TXD → PIN_HC12_RX (GPIO16) — ESP32 receives from HC-12
 *   HC-12 RXD → PIN_HC12_TX (GPIO17) — ESP32 transmits to HC-12
 *   HC-12 SET → PIN_HC12_SET (GPIO22) — LOW = AT mode, HIGH = transparent
 *
 * Default module settings used here:
 *   Mode    : FU3   (normal-speed, full-function)
 *   Baud    : 9600 bps (factory default)
 *   Channel : CH001 (433.4 MHz)
 *   Power   : 8     (20 dBm / 100 mW — maximum)
 *
 * Public API:
 *   hc12_init()        — Configure module via AT commands, open UART.
 *   hc12_update()      — Call every loop(); drains UART RX bytes.
 *   hc12_send(msg)     — Transmit a null-terminated string over HC-12.
 *   hc12_has_message() — Returns true when a complete line is ready.
 *   hc12_get_message() — Returns pointer to the latest line; clears the flag.
 */

#include <Arduino.h>
#include "PINS_CONFIG.h"

// ── HC-12 Module Configuration ────────────────────────────────────────────────
#define HC12_BAUD       9600   // UART baud rate — matches AT+B9600 factory default
#define HC12_CHANNEL    1      // Radio channel 1 → 433.4 MHz
#define HC12_POWER      8      // Transmit power level 8 → 20 dBm (maximum range)
#define HC12_RX_BUF     512    // Receive line buffer size (bytes)

// ── Public API ────────────────────────────────────────────────────────────────

/**
 * Initialise the HC-12 module.
 *
 * Sequence:
 *   1. Drive SET HIGH (transparent mode).
 *   2. Open Serial2 at HC12_BAUD on PIN_HC12_RX / PIN_HC12_TX.
 *   3. Enter AT-command mode (SET LOW), apply baud/channel/mode/power.
 *   4. Return to transparent mode (SET HIGH).
 *
 * If AT communication fails the module continues with factory defaults.
 * Call once from setup().
 */
void hc12_init(void);

/**
 * Poll the HC-12 UART and accumulate received bytes into the line buffer.
 * Completes a message when '\\n' is received.  Call every loop() iteration.
 */
void hc12_update(void);

/**
 * Transmit @p message over HC-12 (appends \\r\\n).
 * Returns true on success, false if the pointer is null or empty.
 */
bool hc12_send(const char *message);

/**
 * Returns true when a complete newline-terminated line has been received
 * and is waiting to be consumed by hc12_get_message().
 */
bool hc12_has_message(void);

/**
 * Returns a pointer to the latest received line and clears the ready flag.
 * The returned pointer is valid until the next call to hc12_update() that
 * completes a new message.  Copy the contents if you need to keep them.
 */
const char *hc12_get_message(void);

#endif // HC12_CONFIG_H
