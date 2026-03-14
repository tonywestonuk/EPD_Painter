// Shutdown confirmation example — EPD_Painter + Adafruit GFX
//
// Normal operation:
//   Displays "Press reset to shutdown".
//
// After a reset (on battery):
//   Shutdown handling kicks in. A 5-second countdown is shown.
//   The BOOT button (GPIO 0) can be pressed at any point to cancel.
//   If the countdown reaches zero, the device shuts down normally.

// Choose your board.
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include "EPD_Painter_Adafruit.h"
#include "EPD_Painter_presets.h"
#include "epd_painter_shutdown.h"

#define BOOT_PIN          0   // BOOT button, active LOW
#define COUNTDOWN_SECS    5

EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);

// ---------------------------------------------------------------------------
// Draw text horizontally centred at a given y position.
// ---------------------------------------------------------------------------
void drawCentred(const char* text, int y, uint8_t size, uint8_t colour = 3) {
    int x = (epd.width() - (int)strlen(text) * size * 6) / 2;
    epd.setTextSize(size);
    epd.setTextColor(colour);
    epd.setCursor(x, y);
    epd.print(text);
}

// ---------------------------------------------------------------------------
// Normal idle screen.
// ---------------------------------------------------------------------------
void drawMainScreen() {
    epd.fillScreen(0);
    drawCentred("Press reset to shutdown", epd.height() / 2 - 20, 4);
    epd.paint();
}

// ---------------------------------------------------------------------------
// Draw one frame of the countdown screen.
// ---------------------------------------------------------------------------
void drawCountdownScreen(int seconds) {
    char num[4];
    snprintf(num, sizeof(num), "%d", seconds);

    epd.fillScreen(0);

    drawCentred("Shutting down in",       130, 4);

    // Large countdown number
    int numX = (epd.width() - (int)strlen(num) * 10 * 6) / 2;
    epd.setTextSize(10);
    epd.setTextColor(3);
    epd.setCursor(numX, 220);
    epd.print(num);

    drawCentred("Press BOOT button to cancel", 430, 3, 2);

    epd.paint();
}

// ---------------------------------------------------------------------------
// Run the countdown. Returns true if the user cancelled, false if it expired.
// ---------------------------------------------------------------------------
bool runCountdown() {
    pinMode(BOOT_PIN, INPUT_PULLUP);

    // Clear the shutdown image left on the panel before drawing countdown UI.
    epd.clear();

    for (int s = COUNTDOWN_SECS; s > 0; s--) {
        drawCountdownScreen(s);

        unsigned long t = millis();
        while (millis() - t < 1000) {
            if (digitalRead(BOOT_PIN) == LOW) {
                return true;   // cancelled
            }
            delay(20);
        }
    }
    return false;  // countdown expired
}

// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    epd.setAutoShutdown(false);

    if (!epd.begin()) {
        Serial.println("EPD init failed");
        while (1);
    }

    if (epd.shutdown()->isPending()) {
        // A reset was pressed while running on battery — show the countdown.
        bool cancelled = runCountdown();

        if (cancelled) {
            // User pressed BOOT — abort shutdown, re-arm for next reset.
            epd.shutdown()->cancel();

            epd.fillScreen(0);
            drawCentred("Shutdown cancelled.", epd.height() / 2 - 30, 4);
            drawCentred("Press reset to shut down.", epd.height() / 2 + 30, 3, 2);
            epd.paint();

        } else {
            // Countdown reached zero — proceed with shutdown.
            // proceed() paints the shutdown image and powers off.
            // If USB keeps the device alive it will restart into normal operation.
            epd.shutdown()->proceed();
        }

    } else {
        // Normal startup — draw the idle screen.
        epd.clear();
        drawMainScreen();
    }
}

// ---------------------------------------------------------------------------
void loop() {
    delay(100);
}
