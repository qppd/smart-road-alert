/*************************************************************************
   Quarter-scan HUB75 panel example for ESP32
   Smooth red-fill animation across the panel without quadrant jumps
*************************************************************************/

#include "ESP32-VirtualMatrixPanel-I2S-DMA.h"

// Custom class derived from VirtualMatrixPanel
class CustomPxBasePanel : public VirtualMatrixPanel {
public:
    using VirtualMatrixPanel::VirtualMatrixPanel; // inherit constructor(s)
protected:
    VirtualCoords getCoords(int16_t x, int16_t y); // custom mapping
};

// Custom pixel mapping for quarter-scan panels
inline VirtualCoords CustomPxBasePanel::getCoords(int16_t x, int16_t y) {
    coords = VirtualMatrixPanel::getCoords(x, y); // base mapping

    if (coords.x == -1 || coords.y == -1)
        return coords;

    uint8_t pxbase = panelResX;

    if (panelResY == 32) { // 32px high panel
        if ((coords.y & 8) == 0)
            coords.x += ((coords.x / pxbase) + 1) * pxbase;
        else
            coords.x += (coords.x / pxbase) * pxbase;
        coords.y = (coords.y >> 4) * 8 + (coords.y & 0b00000111);
    }
    else if (panelResY == 16) { // 16px high panel
        if ((coords.y & 4) == 0)
            coords.x += ((coords.x / pxbase) + 1) * pxbase;
        else
            coords.x += (coords.x / pxbase) * pxbase;
        coords.y = (coords.y >> 3) * 4 + (coords.y & 0b00000011);
    }
    else { // other heights
        uint8_t half_height = panelResY / 2;
        if ((coords.y % half_height) < half_height / 2)
            coords.x += (coords.x / pxbase + 1) * pxbase;
        else
            coords.x += (coords.x / pxbase) * pxbase;

        coords.y = (coords.y / half_height) * (half_height / 2) + (coords.y % (half_height / 2));
    }

    return coords;
}

// Panel configuration
#define PANEL_RES_X 32
#define PANEL_RES_Y 16

#define NUM_ROWS 1
#define NUM_COLS 1

#define SERPENT true
#define TOPDOWN false
#define VIRTUAL_MATRIX_CHAIN_TYPE CHAIN_TOP_LEFT_DOWN_ZZ

// Pins
#define R1_PIN 4
#define G1_PIN 5
#define B1_PIN 15
#define R2_PIN 25
#define G2_PIN 18
#define B2_PIN 26
#define A_PIN 27
#define B_PIN 19
#define C_PIN 14
#define D_PIN 12
#define E_PIN -1
#define LAT_PIN 23
#define OE_PIN 13
#define CLK_PIN 21

// Matrix objects
MatrixPanel_I2S_DMA *dma_display = nullptr;
CustomPxBasePanel *FourScanPanel = nullptr;

void setup() {
    Serial.begin(115200);

    HUB75_I2S_CFG::i2s_pins _pins = { R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN,
                                      A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN };

    HUB75_I2S_CFG mxconfig(
        PANEL_RES_X * 2,      // DMA width (half-scan)
        PANEL_RES_Y / 2,      // DMA height (half-scan)
        NUM_ROWS * NUM_COLS,
        _pins
    );

    mxconfig.clkphase = false;         // adjust if columns shifted
    mxconfig.driver = HUB75_I2S_CFG::MBI5124; // driver type

    // Create DMA display object
    dma_display = new MatrixPanel_I2S_DMA(mxconfig);
    dma_display->setBrightness8(192); // ~75% brightness

    if (!dma_display->begin()) {
        Serial.println("****** !KABOOM! I2S memory allocation failed ***********");
        while (1);
    }

    dma_display->clearScreen();
    delay(500);

    // Create virtual panel object
    FourScanPanel = new CustomPxBasePanel(*dma_display, NUM_ROWS, NUM_COLS,
                                          PANEL_RES_X, PANEL_RES_Y,
                                          VIRTUAL_MATRIX_CHAIN_TYPE);
    //FourScanPanel->setPhysicalPanelScanRate(FOUR_SCAN_16PX_HIGH);
}
void loop() {

 for (int i = FourScanPanel->height() - 1; i >= 0; i--)
{
  for (int j = FourScanPanel->width() - 1; j >= 0; j--)
  {
    FourScanPanel->drawPixel(j, i, FourScanPanel->color565(255, 0, 0));
    delay(30);
  }
}

  delay(2000);
  dma_display->clearScreen();

} // end loop