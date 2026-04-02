#include "P10_LED_CONFIG.h"

MatrixPanel_I2S_DMA *dma_display = nullptr;
CustomMatrix         *display     = nullptr;

// =======================================================================
// CustomMatrix::drawPixel
//
// Maps logical (x, y) on the 96x16 canvas to the physical DMA address
// in the 192x8 buffer for three chained P10 panels.
//
// Stock panel orientation is used (no 90-degree panel rotation).
//
// Pipeline:
//   STEP A - panel split      (logical 96x16 -> physical 32x16 per panel)
//   STEP B - 1/4-scan fix     (physical 32x16 -> DMA 64x8 per panel)
//   STEP C - chain offset     (DMA x += panel_index x 64)
// =======================================================================
void CustomMatrix::drawPixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= P10_LOGICAL_W || y < 0 || y >= P10_LOGICAL_H) return;

    // ------------------------------------------------------------------
    // STEP A: Stock panel split (no rotation)
    // ------------------------------------------------------------------
    int16_t px_local, py_local, panel_index;
    panel_index = x / 32;
    px_local = x % 32;
    py_local = y;

    // ------------------------------------------------------------------
    // STEP B: P10 1/4-scan remap (physical 32x16 -> DMA 64x8)
    // ------------------------------------------------------------------
    int16_t dma_x_local, dma_y_local;
    if (py_local < 8) {
        dma_x_local = px_local;
        dma_y_local = py_local;
    } else {
        dma_x_local = px_local + 32;
        dma_y_local = py_local - 8;
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
    mxconfig.gpio.d   = -1;
    mxconfig.gpio.e   = -1;
    mxconfig.gpio.lat = P_LAT;
    mxconfig.gpio.oe  = P_OE;
    mxconfig.gpio.clk = P_CLK;

    // P10-specific timing fixes required for correct 1/4-scan operation
    mxconfig.driver = HUB75_I2S_CFG::FM6124;
    mxconfig.line_decoder = HUB75_I2S_CFG::TYPE138;
    mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_10M;
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
// displayVehicleData – compact layout for 96x16 stock-orientation canvas
//
//  [SPD 42km/h] [DST 12.3m] [GO/STP]
// =======================================================================
void displayVehicleData(float speed, float distance, bool safe) {
    if (!display) return;

    display->fillScreen(0);

    // Border
    display->drawRect(0, 0, P10_LOGICAL_W, P10_LOGICAL_H,
                      dma_display->color565(60, 60, 60));

    // --- Segment 1: Speed ---
    display->setTextSize(1);
    display->setTextColor(dma_display->color565(255, 80, 0));
    display->setCursor(2, 4);
    display->print("SPD");

    // --- Segment 2: Distance ---
    char buf[16];
    display->setTextColor(dma_display->color565(255, 220, 0));
    display->setCursor(22, 4);
    snprintf(buf, sizeof(buf), "%.0fkm/h", speed);
    display->print(buf);

    display->setTextColor(dma_display->color565(0, 200, 255));
    display->setCursor(52, 4);
    display->print("DST");

    display->setCursor(70, 4);
    snprintf(buf, sizeof(buf), "%.1fm", distance);
    display->print(buf);

    // --- Segment 3: Status ---
    if (safe) {
        display->setTextColor(dma_display->color565(0, 255, 0));
        display->setCursor(88, 4);
        display->print("GO");
    } else {
        display->setTextColor(dma_display->color565(255, 0, 0));
        display->setCursor(86, 4);
        display->print("STP");
    }
}
