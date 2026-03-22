#ifndef P10_LED_CONFIG_H
#define P10_LED_CONFIG_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include "PINS_CONFIG.h"

// =======================================================================
// P10 Full-Color Panel  |  32x16 px, 1/4-scan, HUB75, SMD3535
//
// PHYSICAL CHAIN (left → right, viewed from front):
//
//   ESP32 ──► [PANEL A IN]──[PANEL A OUT]──►[PANEL B IN]──[PANEL B OUT]──►[PANEL C IN]
//
//   PANEL A  │  PANEL B  │  PANEL C
//   [LEFT]   │  [MIDDLE] │  [RIGHT]
//   90° CW   │  90° CCW  │  90° CW
//
// Each physical 32×16 panel stands upright after rotation → 16×32 logical.
// Three panels side-by-side → 48×32 total logical canvas.
//
// "64×8 DMA TRICK": each panel is declared as 64-wide × 8-tall so the
// library is forced into 1/4-scan multiplexing.  The CustomMatrix mapper
// converts logical (x,y) → correct DMA address in three steps:
//   A) rotate  B) 1/4-scan remap  C) chain offset
// =======================================================================
#define PANEL_RES_X     64   // 1/4-scan trick: 32 physical cols -> 64 DMA cols
#define PANEL_RES_Y     8    // 1/4-scan trick: 16 physical rows -> 8 DMA rows
#define PANEL_CHAIN     3    // Number of P10 panels chained

#define P10_LOGICAL_W   48   // Logical width  (PANEL_CHAIN * 16, rotated)
#define P10_LOGICAL_H   32   // Logical height (32 rows after 90-deg rotation)

// HUB75 Pin Definitions — numeric values are centralised in PINS_CONFIG.h.
// These aliases preserve backward compatibility with P10_LED_CONFIG.cpp.
#define P_R1  PIN_P10_R1
#define P_G1  PIN_P10_G1
#define P_B1  PIN_P10_B1
#define P_R2  PIN_P10_R2
#define P_G2  PIN_P10_G2
#define P_B2  PIN_P10_B2
#define P_A   PIN_P10_A
#define P_B   PIN_P10_B
#define P_C   PIN_P10_C
#define P_LAT PIN_P10_LAT
#define P_OE  PIN_P10_OE
#define P_CLK PIN_P10_CLK

// Custom Adafruit_GFX subclass that remaps logical coordinates to the
// physical DMA buffer via S-shape rotation + 1/4-scan addressing.
class CustomMatrix : public Adafruit_GFX {
public:
    MatrixPanel_I2S_DMA *dma;

    explicit CustomMatrix(MatrixPanel_I2S_DMA *d)
        : Adafruit_GFX(P10_LOGICAL_W, P10_LOGICAL_H), dma(d) {}

    void drawPixel(int16_t x, int16_t y, uint16_t color) override;
};

extern MatrixPanel_I2S_DMA *dma_display;
extern CustomMatrix         *display;

void setupP10();
void displayTextP10(const char *text);
void displayVehicleData(float speed, float distance, bool safe);

#endif // P10_LED_CONFIG_H
