#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

// =============================================================================
// PINS_CONFIG.h — Single source of truth for all ESP32 GPIO pin assignments.
//
// Rules:
//   • Every GPIO number used in this project MUST be defined here.
//   • No other file may use a bare numeric literal for a pin number;
//     all other files must reference the macros defined below.
//   • Both ESP32 units run identical firmware, so this file is shared.
// =============================================================================

// ── P10 HUB75 Full-Color LED Matrix ──────────────────────────────────────────
// Three 32×16 panels chained, driven via HUB75 parallel interface.
// "64×8 DMA trick" forces 1/4-scan; see P10_LED_CONFIG.h for details.
#define PIN_P10_R1   4    // HUB75 R1 (upper-half red)
#define PIN_P10_G1   5    // HUB75 G1 (upper-half green)
#define PIN_P10_B1   15   // HUB75 B1 (upper-half blue)
#define PIN_P10_R2   25   // HUB75 R2 (lower-half red)
#define PIN_P10_G2   18   // HUB75 G2 (lower-half green)
#define PIN_P10_B2   26   // HUB75 B2 (lower-half blue)
#define PIN_P10_A    27   // HUB75 row-select A
#define PIN_P10_B    19   // HUB75 row-select B
#define PIN_P10_C    14   // HUB75 row-select C
#define PIN_P10_LAT  23   // HUB75 latch
#define PIN_P10_OE   13   // HUB75 output-enable (active-LOW)
#define PIN_P10_CLK  21   // HUB75 pixel clock

// ── HC-12 433 MHz Wireless Transceiver (ESP32 Hardware Serial 2) ─────────────
// The HC-12 module provides transparent bidirectional serial at 9600 baud.
// Serial2 (UART2) is used so that USB (Serial0) remains available for the
// RPi USB link and Serial1 is kept free.
//
// Wiring:
//   HC-12 VCC → 3.3 V
//   HC-12 GND → GND
//   HC-12 TXD → GPIO16  (ESP32 RX2)   HC-12 transmits → ESP32 receives
//   HC-12 RXD → GPIO17  (ESP32 TX2)   HC-12 receives  → ESP32 transmits
//   HC-12 SET → GPIO22  Active-LOW:   LOW = AT-command mode
//                                      HIGH (or floating) = transparent mode
#define PIN_HC12_RX   16   // ESP32 RX2  ← HC-12 TXD
#define PIN_HC12_TX   17   // ESP32 TX2  → HC-12 RXD
#define PIN_HC12_SET  22   // Active-LOW AT-command mode pin

#endif // PINS_CONFIG_H
