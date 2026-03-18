#include "SERIAL_CONFIG.h"

#include "P10_LED_CONFIG.h"
#include "DF_MINI_CONFIG.h"

static const unsigned long SENSOR_POLL_INTERVAL_MS  = 500UL;
static const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 500UL;

static unsigned long s_last_sensor_poll_ms   = 0;
static unsigned long s_last_display_ms       = 0;

// Latest vehicle state, updated every sensor poll and reflected on the display
static float  s_speed    = 0.0f;
static float  s_distance = 0.0f;
static bool   s_safe     = true;

static void handle_incoming_message(const SerialMessage &msg);
static void poll_and_send_sensor_data(void);

void setup() {
    serial_init();
    setupP10();
    setupDFMini();
    // Example: display welcome message and play a track
    displayTextP10("Welcome!");
    playDFMiniTrack(1); // Play track 1 (ensure MP3 files are loaded on SD card)
}

void loop() {
    serial_update();
    if (serial_has_message()) {
        SerialMessage msg = serial_get_message();
        handle_incoming_message(msg);
    }
    if (serial_is_connected()) {
        poll_and_send_sensor_data();
    }

    // Refresh the P10 display at the same rate as sensor polling
    unsigned long now = millis();
    if (now - s_last_display_ms >= DISPLAY_UPDATE_INTERVAL_MS) {
        s_last_display_ms = now;
        displayVehicleData(s_speed, s_distance, s_safe);
    }
}

static void handle_incoming_message(const SerialMessage &msg) {
    
    if (!msg.valid || msg.raw[0] == '\0') {
        return;
    }

    if (strstr(msg.raw, "\"cmd\":\"ping\"") != nullptr) {
        serial_send("{\"type\":\"pong\"}");
        return;
    }

   
    if (strstr(msg.raw, "\"cmd\":\"status\"") != nullptr) {
        serial_send("{\"type\":\"status\",\"state\":\"ok\"}");
        return;
    }

    serial_send("{\"type\":\"error\",\"msg\":\"unknown command\"}");
}


static void poll_and_send_sensor_data(void) {
    unsigned long now = millis();

    // Rate-limit without blocking
    if (now - s_last_sensor_poll_ms < SENSOR_POLL_INTERVAL_MS) {
        return;
    }
    s_last_sensor_poll_ms = now;

    s_speed    = 42.0f + static_cast<float>(now % 10);
    s_distance = 12.3f + static_cast<float>(now % 5);
    s_safe     = (s_speed < 60.0f);

    // Build JSON payload
    char payload[SERIAL_TX_BUFFER_SIZE];
    snprintf(
        payload,
        sizeof(payload),
        "{\"type\":\"vehicle\",\"speed\":%.1f,\"distance\":%.1f}",
        s_speed,
        s_distance
    );

    serial_send(payload);
}

