// =============================================================================
// P10Led.ino  –  Standalone P10 LED test sketch for ESP32
// No external .h/.cpp files needed. Flash and run directly.
//
// Hardware: 3x P10 Full-Color HUB75 32x16 1/4-scan panels chained:
//   ESP32 ──► [PANEL A IN]──[A OUT]──►[PANEL B IN]──[B OUT]──►[PANEL C IN]
//
//   PANEL A [LEFT]   90° CW  (IN=bottom, OUT=top)
//   PANEL B [MIDDLE] 90° CCW (IN=top,    OUT=bottom)  ← S-flip
//   PANEL C [RIGHT]  90° CW  (IN=bottom, OUT=top)
//
// Logical canvas: 48 wide × 32 tall
// =============================================================================

#include <Adafruit_GFX.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

// -----------------------------------------------------------------------------
// HARDWARE CONFIGURATION
// "64×8 DMA trick": declare each panel as 64-wide × 8-tall to force the
// library into the 1/4-scan multiplexing mode required by P10 hardware.
// -----------------------------------------------------------------------------
#define PANEL_RES_X   64    // DMA width per panel  (32 physical cols → 64)
#define PANEL_RES_Y    8    // DMA height per panel (16 physical rows →  8)
#define PANEL_CHAIN    3    // Number of chained panels

#define LOGICAL_W     48    // Logical canvas width  (3 panels × 16 cols)
#define LOGICAL_H     32    // Logical canvas height (32 rows after rotation)

// HUB75 Pin Definitions
#define P_R1   4
#define P_G1   5
#define P_B1  15
#define P_R2  25
#define P_G2  18
#define P_B2  26
#define P_A   27
#define P_B   19
#define P_C   14
#define P_LAT 23
#define P_OE  13
#define P_CLK 21

// -----------------------------------------------------------------------------
// CustomMatrix – Adafruit_GFX subclass that remaps logical (x,y) to the
// correct DMA address through the 3-step pipeline:
//   A) Per-panel rotation  B) 1/4-scan remap  C) Chain offset
// -----------------------------------------------------------------------------
class CustomMatrix : public Adafruit_GFX {
public:
    MatrixPanel_I2S_DMA *dma;

    explicit CustomMatrix(MatrixPanel_I2S_DMA *d)
        : Adafruit_GFX(LOGICAL_W, LOGICAL_H), dma(d) {}

    void drawPixel(int16_t x, int16_t y, uint16_t color) override {
        if (x < 0 || x >= LOGICAL_W || y < 0 || y >= LOGICAL_H) return;

        // ── STEP A: Rotation remap ──────────────────────────────────────
        // 90° CW  → px = 31 - y,  py = lx        (Panel A & C)
        // 90° CCW → px = y,        py = 15 - lx   (Panel B, S-flip)
        int16_t px_local, py_local, panel_index;

        if (x < 16) {
            // PANEL A – 90° CW
            px_local    = 31 - y;
            py_local    = x;
            panel_index = 0;
        } else if (x < 32) {
            // PANEL B – 90° CCW (S-chain flip)
            px_local    = y;
            py_local    = 15 - (x - 16);
            panel_index = 1;
        } else {
            // PANEL C – 90° CW (same as A)
            px_local    = 31 - y;
            py_local    = x - 32;
            panel_index = 2;
        }

        // ── STEP B: 1/4-scan fix (32×16 physical → 64×8 DMA) ───────────
        // Rows  0-7  → DMA cols  0-31, DMA rows 0-7
        // Rows 8-15  → DMA cols 32-63, DMA rows 0-7  (overlap forces 1/4-scan)
        int16_t dma_x, dma_y;
        if (py_local < 8) {
            dma_x = px_local;
            dma_y = py_local;
        } else {
            dma_x = px_local + 32;
            dma_y = py_local - 8;
        }

        // ── STEP C: Chain panel offset (each panel = 64 DMA columns) ────
        dma_x += panel_index * 64;

        dma->drawPixel(dma_x, dma_y, color);
    }
};

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------
MatrixPanel_I2S_DMA *dma_display = nullptr;
CustomMatrix        *matrix      = nullptr;

// -----------------------------------------------------------------------------
// Helper: colour shorthand
// -----------------------------------------------------------------------------
inline uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    return dma_display->color565(r, g, b);
}

// -----------------------------------------------------------------------------
// Test scenes
// -----------------------------------------------------------------------------

// 1. Solid colour fill – confirms all panels light up
void testSolidColors() {
    uint16_t colors[] = {
        rgb(255,   0,   0),   // red
        rgb(  0, 255,   0),   // green
        rgb(  0,   0, 255),   // blue
        rgb(255, 255,   0),   // yellow
        rgb(  0, 255, 255),   // cyan
        rgb(255,   0, 255),   // magenta
        rgb(255, 255, 255),   // white
    };
    for (uint16_t c : colors) {
        matrix->fillScreen(c);
        delay(600);
    }
    matrix->fillScreen(0);
}

// 2. Border rect – confirms edges and corners are correctly mapped
void testBorder() {
    matrix->fillScreen(0);
    matrix->drawRect(0, 0, LOGICAL_W, LOGICAL_H, rgb(255, 0, 0));
    // Vertical centre line to show panel A|B and B|C seams
    matrix->drawFastVLine(15, 0, LOGICAL_H, rgb(0, 255, 0));
    matrix->drawFastVLine(16, 0, LOGICAL_H, rgb(0, 255, 0));
    matrix->drawFastVLine(31, 0, LOGICAL_H, rgb(0, 0, 255));
    matrix->drawFastVLine(32, 0, LOGICAL_H, rgb(0, 0, 255));
    delay(2000);
}

// 3. Horizontal gradient – each column a different shade to verify ordering
void testGradient() {
    matrix->fillScreen(0);
    for (int16_t x = 0; x < LOGICAL_W; x++) {
        uint8_t shade = map(x, 0, LOGICAL_W - 1, 10, 255);
        uint16_t col = rgb(shade, 0, 255 - shade);
        matrix->drawFastVLine(x, 0, LOGICAL_H, col);
    }
    delay(2000);
}

// 4. Pixel walk across every row – quick scan test
void testPixelWalk() {
    matrix->fillScreen(0);
    for (int16_t y = 0; y < LOGICAL_H; y++) {
        for (int16_t x = 0; x < LOGICAL_W; x++) {
            matrix->drawPixel(x, y, rgb(0, 255, 80));
            delay(8);
            matrix->drawPixel(x, y, 0);
        }
    }
}

// 5. Simulated vehicle-data display (same layout as the real firmware)
//    SPD label | speed value | GO/STP status
void testVehicleDisplay(float speed, float distance, bool safe) {
    matrix->fillScreen(0);

    matrix->drawRect(0, 0, LOGICAL_W, LOGICAL_H, rgb(60, 60, 60));

    // Labels
    matrix->setTextSize(1);
    matrix->setTextColor(rgb(255, 80, 0));
    matrix->setCursor(2, 8);
    matrix->print("SPD");

    matrix->setTextColor(rgb(0, 200, 255));
    matrix->setCursor(2, 18);
    matrix->print("DST");

    // Values
    char buf[16];
    matrix->setTextColor(rgb(255, 220, 0));
    matrix->setCursor(20, 8);
    snprintf(buf, sizeof(buf), "%.0fkm/h", speed);
    matrix->print(buf);

    matrix->setTextColor(rgb(0, 200, 255));
    matrix->setCursor(20, 18);
    snprintf(buf, sizeof(buf), "%.1fm", distance);
    matrix->print(buf);

    // Status
    if (safe) {
        matrix->setTextColor(rgb(0, 255, 0));
        matrix->setCursor(37, 12);
        matrix->print("GO");
    } else {
        matrix->setTextColor(rgb(255, 0, 0));
        matrix->setCursor(37, 12);
        matrix->print("STP");
    }
}

// 6. Scrolling text across the full 48-wide canvas
void testScrollText(const char *msg, uint16_t color) {
    int16_t textW = strlen(msg) * 6;  // each char is ~6px wide at size 1
    matrix->setTextSize(1);
    matrix->setTextColor(color);
    for (int16_t xPos = LOGICAL_W; xPos > -textW; xPos--) {
        matrix->fillScreen(0);
        matrix->setCursor(xPos, 12);
        matrix->print(msg);
        delay(40);
    }
}

// -----------------------------------------------------------------------------
// setup / loop
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial.println("P10 LED test starting...");

    HUB75_I2S_CFG cfg(PANEL_RES_X, PANEL_RES_Y, PANEL_CHAIN);
    cfg.gpio.r1  = P_R1;  cfg.gpio.g1  = P_G1;  cfg.gpio.b1  = P_B1;
    cfg.gpio.r2  = P_R2;  cfg.gpio.g2  = P_G2;  cfg.gpio.b2  = P_B2;
    cfg.gpio.a   = P_A;   cfg.gpio.b   = P_B;   cfg.gpio.c   = P_C;
    cfg.gpio.lat = P_LAT; cfg.gpio.oe  = P_OE;  cfg.gpio.clk = P_CLK;

    // P10-specific timing — required for correct 1/4-scan operation
    cfg.clkphase       = false;
    cfg.latch_blanking = 4;

    dma_display = new MatrixPanel_I2S_DMA(cfg);
    dma_display->begin();
    dma_display->setBrightness8(90);
    dma_display->clearScreen();

    matrix = new CustomMatrix(dma_display);
    matrix->setTextWrap(false);

    Serial.println("Setup done.");
}

void loop() {
    Serial.println("== Solid color fill ==");
    testSolidColors();
    delay(300);

    Serial.println("== Border + seam lines ==");
    testBorder();
    delay(300);

    Serial.println("== Horizontal gradient ==");
    testGradient();
    delay(300);

    Serial.println("== Pixel walk ==");
    testPixelWalk();
    delay(300);

    Serial.println("== Vehicle display: SAFE ==");
    testVehicleDisplay(42.0f, 12.3f, true);
    delay(2500);

    Serial.println("== Vehicle display: DANGER ==");
    testVehicleDisplay(80.0f, 3.1f, false);
    delay(2500);

    Serial.println("== Scroll text ==");
    testScrollText("SMART ROAD ALERT", rgb(255, 200, 0));
    delay(300);
}
