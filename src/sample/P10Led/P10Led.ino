
/*************************************************************************
  Quarter-scan HUB75 panel example for ESP32
  Smooth red-fill animation across the panel without quadrant jumps
/*************************************************************************/

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

  // mapper for panels with any other heights
  else {
    uint8_t half_height = panelResY / 2;
   
    if ((coords.y  % half_height ) < half_height/2)
    {
     coords.x += (coords.x / pxbase + 1) * pxbase;
    }
    else
    {
     coords.x += (coords.x / pxbase) * pxbase; // 2nd, 4th 'block' of 8 rows of pixels, offset by panel width in DMA buffer
     }
     
     coords.y = (coords.y / half_height ) * (half_height/2) + (coords.y % (half_height/2));

  }
  return coords;
}

// Panel configuration
#define PANEL_RES_X 32 // Number of pixels wide of each INDIVIDUAL panel module. 
#define PANEL_RES_Y 16 // Number of pixels tall of each INDIVIDUAL panel module.

// Use a single panel for tests
#define NUM_ROWS 1 // Number of rows of chained INDIVIDUAL PANELS
#define NUM_COLS 1 // Number of INDIVIDUAL PANELS per ROW

// Chain settings, do not cnahge
#define SERPENT true
#define TOPDOWN false
#define VIRTUAL_MATRIX_CHAIN_TYPE CHAIN_BOTTOM_RIGHT_UP

// placeholder for the matrix object
MatrixPanel_I2S_DMA *dma_display = nullptr;

// placeholder for the virtual display object
CustomPxBasePanel   *FourScanPanel = nullptr;

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

/******************************************************************************
   Setup!
 ******************************************************************************/
void setup()
{
  HUB75_I2S_CFG::i2s_pins _pins={R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN};
  HUB75_I2S_CFG mxconfig(
    PANEL_RES_X * 2,            // DO NOT CHANGE THIS
    PANEL_RES_Y / 2,            // DO NOT CHANGE THIS
    NUM_ROWS * NUM_COLS         // DO NOT CHANGE THIS
    ,_pins            // Uncomment to enable custom pins
  );

  // Change this if you see pixels showing up shifted wrongly by one column the left or right.
  mxconfig.clkphase = false; 
  
  // Uncomment this to use a TYPE595 decoder like DP32020/SM5368/TC75xx
  //mxconfig.line_decoder = HUB75_I2S_CFG::TYPE595;

  // Driver
  mxconfig.driver = HUB75_I2S_CFG::MBI5124;

  // OK, now we can create our matrix object
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);

  // let's adjust default brightness to about 75%
  dma_display->setBrightness8(1);    // range is 0-255, 0 - 0%, 255 - 100%

  // Allocate memory and start DMA display
  if ( not dma_display->begin() )
    Serial.println("****** !KABOOM! I2S memory allocation failed ***********");


  dma_display->clearScreen();
  delay(500);

  // create FourScanPanellay object based on our newly created dma_display object
  FourScanPanel = new CustomPxBasePanel ((*dma_display), NUM_ROWS, NUM_COLS, PANEL_RES_X, PANEL_RES_Y,  VIRTUAL_MATRIX_CHAIN_TYPE);

}


void loop() {
  for (int i = 0; i < FourScanPanel->height(); i++)
  {
    for (int j = 0; j < FourScanPanel->width(); j++)
    {
      FourScanPanel->drawPixel(j, i, FourScanPanel->color565(255, 0, 0));
      delay(5);
    }
  }
  delay(2000);
  dma_display->clearScreen();
} // end loop

