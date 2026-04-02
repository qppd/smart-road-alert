// =============================================================================
// P10Led.ino - Standalone P10 LED test sketch for ESP32
// No external .h/.cpp files needed. Flash and run directly.
//
// Hardware: 3x P10 Full-Color HUB75 32x16 1/4-scan panels chained:
//   ESP32 -> [PANEL A IN]--[A OUT]-->[PANEL B IN]--[B OUT]-->[PANEL C IN]
//
// The ESP32-HUB75-MatrixPanel-I2S-DMA core is designed for HALF-scan DMA output.
// For QUARTER-scan P10 panels, this sketch uses a virtual display remap class
// that transforms logical coordinates before calling dma->drawPixel().
//
// Logical canvas: 96 wide x 16 tall (stock panel orientation)
// =============================================================================

#include <Adafruit_GFX.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

#define PANEL_RES_X   64    
#define PANEL_RES_Y    8   
#define PANEL_CHAIN    3   

#define LOGICAL_W     96
#define LOGICAL_H     16

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

class VirtualQuarterScanDisplay : public Adafruit_GFX {
public:
    MatrixPanel_I2S_DMA *dma;

    explicit VirtualQuarterScanDisplay(MatrixPanel_I2S_DMA *d)
        : Adafruit_GFX(LOGICAL_W, LOGICAL_H), dma(d) {}

    void drawPixel(int16_t x, int16_t y, uint16_t color) override {
        if (x < 0 || x >= LOGICAL_W || y < 0 || y >= LOGICAL_H) return;

        int16_t px_local, py_local, panel_index;
        panel_index = x / 32;
        px_local = x % 32;
        py_local = y;

   
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


MatrixPanel_I2S_DMA *dma_display = nullptr;
VirtualQuarterScanDisplay *matrix = nullptr;


inline uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    return dma_display->color565(r, g, b);
}



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

void testBorder() {
    matrix->fillScreen(0);
    matrix->drawRect(0, 0, LOGICAL_W, LOGICAL_H, rgb(255, 0, 0));
    // Vertical seam lines for A|B and B|C panel boundaries
    matrix->drawFastVLine(31, 0, LOGICAL_H, rgb(0, 255, 0));
    matrix->drawFastVLine(32, 0, LOGICAL_H, rgb(0, 255, 0));
    matrix->drawFastVLine(63, 0, LOGICAL_H, rgb(0, 0, 255));
    matrix->drawFastVLine(64, 0, LOGICAL_H, rgb(0, 0, 255));
    delay(2000);
}


void testGradient() {
    matrix->fillScreen(0);
    for (int16_t x = 0; x < LOGICAL_W; x++) {
        uint8_t shade = map(x, 0, LOGICAL_W - 1, 10, 255);
        uint16_t col = rgb(shade, 0, 255 - shade);
        matrix->drawFastVLine(x, 0, LOGICAL_H, col);
    }
    delay(2000);
}


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

void testVehicleDisplay(float speed, float distance, bool safe) {
    matrix->fillScreen(0);

    matrix->drawRect(0, 0, LOGICAL_W, LOGICAL_H, rgb(60, 60, 60));

    matrix->setTextSize(1);
    matrix->setTextColor(rgb(255, 80, 0));
    matrix->setCursor(2, 4);
    matrix->print("SPD");

    char buf[16];
    matrix->setTextColor(rgb(255, 220, 0));
    matrix->setCursor(22, 4);
    snprintf(buf, sizeof(buf), "%.0fkm/h", speed);
    matrix->print(buf);

    matrix->setTextColor(rgb(0, 200, 255));
    matrix->setCursor(52, 4);
    matrix->print("DST");

    matrix->setCursor(70, 4);
    snprintf(buf, sizeof(buf), "%.1fm", distance);
    matrix->print(buf);

    if (safe) {
        matrix->setTextColor(rgb(0, 255, 0));
        matrix->setCursor(88, 4);
        matrix->print("GO");
    } else {
        matrix->setTextColor(rgb(255, 0, 0));
        matrix->setCursor(86, 4);
        matrix->print("STP");
    }
}


void testScrollText(const char *msg, uint16_t color) {
    int16_t textW = strlen(msg) * 6; 
    matrix->setTextSize(1);
    matrix->setTextColor(color);
    for (int16_t xPos = LOGICAL_W; xPos > -textW; xPos--) {
        matrix->fillScreen(0);
        matrix->setCursor(xPos, 4);
        matrix->print(msg);
        delay(40);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("P10 LED test starting...");

    HUB75_I2S_CFG cfg(PANEL_RES_X, PANEL_RES_Y, PANEL_CHAIN);
    cfg.gpio.r1  = P_R1;  cfg.gpio.g1  = P_G1;  cfg.gpio.b1  = P_B1;
    cfg.gpio.r2  = P_R2;  cfg.gpio.g2  = P_G2;  cfg.gpio.b2  = P_B2;
    cfg.gpio.a   = P_A;   cfg.gpio.b   = P_B;   cfg.gpio.c   = P_C;
    cfg.gpio.d   = -1;    cfg.gpio.e   = -1;
    cfg.gpio.lat = P_LAT; cfg.gpio.oe  = P_OE;  cfg.gpio.clk = P_CLK;

    // Verified config for this 32x16 1/4-scan P10 variant (74HC138D + 6124DJ).
    cfg.driver         = HUB75_I2S_CFG::FM6126A;
    cfg.line_decoder   = HUB75_I2S_CFG::TYPE138;
    cfg.i2sspeed       = HUB75_I2S_CFG::HZ_10M;
    cfg.clkphase       = false;
    cfg.latch_blanking = 4;

    dma_display = new MatrixPanel_I2S_DMA(cfg);
    dma_display->begin();
    dma_display->setBrightness8(90);
    dma_display->clearScreen();

    matrix = new VirtualQuarterScanDisplay(dma_display);
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