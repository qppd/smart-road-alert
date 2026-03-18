#include "P10_LED_CONFIG.h"

MatrixPanel_I2S_DMA *dma_display = nullptr;
CustomMatrix         *display     = nullptr;

// =======================================================================
// CustomMatrix::drawPixel
//
// Maps logical (x, y) on the 48×32 canvas to the physical DMA address
// in the 192×8 buffer for three chained P10 panels.
//
// Physical orientation (viewed from front):
//
//   Logical x:  0──────15 | 16─────31 | 32─────47
//               PANEL A   │  PANEL B  │  PANEL C
//               90° CW    │  90° CCW  │  90° CW
//               IN=bottom │  IN=top   │  IN=bottom
//               OUT=top   │  OUT=bot  │  OUT=top
//
// Pipeline:
//   STEP A – rotation remap (logical 48×32 → physical 32×16 per panel)
//   STEP B – 1/4-scan fix   (physical 32×16 → DMA 64×8 per panel)
//   STEP C – chain offset   (DMA x += panel_index × 64)
// =======================================================================
void CustomMatrix::drawPixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= P10_LOGICAL_W || y < 0 || y >= P10_LOGICAL_H) return;

    // ------------------------------------------------------------------
    // STEP A: Rotation remap
    //
    // 90° CW  formula: px = 31 - ly,  py = lx
    // 90° CCW formula: px = ly,        py = 15 - lx
    //   where lx = local logical col (0-15), ly = logical row (0-31)
    // ------------------------------------------------------------------
    int16_t px_local, py_local, panel_index;

    if (x < 16) {
        // PANEL A – 90° CW (IN connector at bottom, OUT at top)
        px_local    = 31 - y;        // physical col  (0-31)
        py_local    = x;             // physical row  (0-15)
        panel_index = 0;
    } else if (x < 32) {
        // PANEL B – 90° CCW (IN connector at top, OUT at bottom — S-chain flip)
        px_local    = y;             // physical col  (0-31)
        py_local    = 15 - (x - 16); // physical row  (15→0, reversed)
        panel_index = 1;
    } else {
        // PANEL C – 90° CW (IN connector at bottom, OUT at top — same as A)
        px_local    = 31 - y;        // physical col  (0-31)
        py_local    = x - 32;        // physical row  (0-15)
        panel_index = 2;
    }

    // ------------------------------------------------------------------
    // STEP B: P10 1/4-Scan Hardware Fix (physical 32x16 -> DMA 64x8)
    //
    // The DMA library drives the panel as if it were 64-wide x 8-tall.
    // Physical rows  0-7  map to DMA cols  0-31, DMA rows 0-7.
    // Physical rows 8-15  map to DMA cols 32-63, DMA rows 0-7.
    // (Rows overlap in the DMA Y axis, forcing 1/4-scan multiplexing.)
    // ------------------------------------------------------------------
    int16_t dma_x_local, dma_y_local;
    if (py_local < 8) {
        dma_x_local = px_local;
        dma_y_local = py_local;
    } else {
        dma_x_local = px_local + 32;  // bottom 8 rows -> right half of DMA row
        dma_y_local = py_local - 8;   // overlap Y to engage 1/4-scan
    }

    // ------------------------------------------------------------------
    // STEP C: Chain Panel Offset
    // Each panel occupies 64 columns in the DMA buffer.
    // ------------------------------------------------------------------
    int16_t dma_x_global = dma_x_local + (panel_index * 64);

    dma->drawPixel(dma_x_global, dma_y_local, color);
}

// =======================================================================
// setupP10
// =======================================================================
void setupP10() {
    HUB75_I2S_CFG mxconfig(PANEL_RES_X, PANEL_RES_Y, PANEL_CHAIN);

    // Full-color HUB75 pin assignment
    mxconfig.gpio.r1  = P_R1;
    mxconfig.gpio.g1  = P_G1;
    mxconfig.gpio.b1  = P_B1;
    mxconfig.gpio.r2  = P_R2;
    mxconfig.gpio.g2  = P_G2;
    mxconfig.gpio.b2  = P_B2;
    mxconfig.gpio.a   = P_A;
    mxconfig.gpio.b   = P_B;
    mxconfig.gpio.c   = P_C;
    mxconfig.gpio.lat = P_LAT;
    mxconfig.gpio.oe  = P_OE;
    mxconfig.gpio.clk = P_CLK;

    // P10-specific timing fixes required for correct 1/4-scan operation
    mxconfig.clkphase      = false;
    mxconfig.latch_blanking = 4;

    dma_display = new MatrixPanel_I2S_DMA(mxconfig);
    dma_display->begin();
    dma_display->setBrightness8(90);
    dma_display->clearScreen();

    display = new CustomMatrix(dma_display);
    display->setTextWrap(false);
}

// =======================================================================
// displayTextP10 – show a single line of white text on black background
// =======================================================================
void displayTextP10(const char *text) {
    if (!display) return;
    display->fillScreen(0);
    display->setTextSize(1);
    display->setTextColor(dma_display->color565(255, 255, 255));
    display->setCursor(0, 12);
    display->print(text);
}

// =======================================================================
// displayVehicleData – structured road-alert layout (48x32 canvas)
//
//  ┌────────────────────────────────────────────────┐
//  │ SPD        │  42km/h   │  GO  │   <- columns   │
//  │ DIST       │  12.3m    │      │                 │
//  └────────────────────────────────────────────────┘
//
// The wider 48-pixel canvas allows a 3-column layout:
//   col 0-15  : label (SPD / DIST)
//   col 16-35 : value (speed / distance)
//   col 36-47 : status (GO / STP)
// =======================================================================
void displayVehicleData(float speed, float distance, bool safe) {
    if (!display) return;

    display->fillScreen(0);

    // Border
    display->drawRect(0, 0, P10_LOGICAL_W, P10_LOGICAL_H,
                      dma_display->color565(60, 60, 60));

    // --- Column 1: Labels ---
    display->setTextSize(1);
    display->setTextColor(dma_display->color565(255, 80, 0));
    display->setCursor(2, 8);
    display->print("SPD");

    display->setTextColor(dma_display->color565(0, 200, 255));
    display->setCursor(2, 18);
    display->print("DST");

    // --- Column 2: Values ---
    char buf[16];
    display->setTextColor(dma_display->color565(255, 220, 0));
    display->setCursor(20, 8);
    snprintf(buf, sizeof(buf), "%.0fkm/h", speed);
    display->print(buf);

    display->setTextColor(dma_display->color565(0, 200, 255));
    display->setCursor(20, 18);
    snprintf(buf, sizeof(buf), "%.1fm", distance);
    display->print(buf);

    // --- Column 3: Status (x=37 centers text in the last 12px zone) ---
    if (safe) {
        display->setTextColor(dma_display->color565(0, 255, 0));
        display->setCursor(37, 12);
        display->print("GO");
    } else {
        display->setTextColor(dma_display->color565(255, 0, 0));
        display->setCursor(37, 12);
        display->print("STP");
    }
}
