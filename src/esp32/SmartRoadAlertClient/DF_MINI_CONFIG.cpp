#include "DF_MINI_CONFIG.h"
#include <DFRobotDFPlayerMini.h>

// Using HardwareSerial2 with ESP32 GPIO16 (RX) and GPIO17 (TX)
DFRobotDFPlayerMini dfPlayer;

void setupDFMini() {
    Serial2.begin(9600, SERIAL_8N1, 16, 17); // RX=GPIO16, TX=GPIO17
    if (!dfPlayer.begin(Serial2)) {
        // Handle error
        return;
    }
    dfPlayer.volume(20); // Set volume (0~30)
}

void playDFMiniTrack(uint16_t track) {
    dfPlayer.play(track); // Play specific track
}

void stopDFMini() {
    dfPlayer.stop();
}
