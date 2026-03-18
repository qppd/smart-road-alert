#include "DF_MINI_CONFIG.h"
#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>

SoftwareSerial dfSerial(26, 27); // RX, TX (adjust as needed)
DFRobotDFPlayerMini dfPlayer;

void setupDFMini() {
    dfSerial.begin(9600);
    if (!dfPlayer.begin(dfSerial)) {
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
