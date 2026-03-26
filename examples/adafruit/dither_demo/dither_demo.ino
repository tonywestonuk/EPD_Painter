// Choose your board.
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#define EPD_PAINTER_ENABLE_AUTO_SHUTDOWN 1
// Optional library flag used by the examples.
// Set to 1 to enable the reset-twice auto-shutdown helper and shutdown image flow.
// Leave it undefined or set it to 0 in normal projects unless you explicitly want this behaviour.

#include "EPD_Painter_presets.h"
#include "EPD_Painter_Adafruit.h"

#define BOOT_BTN 0   // GPIO 0 — BOOT button, active LOW

EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);

static int currentPage = 1;

// ---------------------------------------------------------------------------
// drawRadialGradient()
// ---------------------------------------------------------------------------
static void drawRadialGradient(EPD_PainterAdafruit& gfx,
                                int cx, int cy, int radius,
                                uint8_t centreGrey, uint8_t edgeGrey) {
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            float dist = sqrtf((float)(dx * dx + dy * dy));
            if (dist > radius) continue;
            float t = dist / radius;
            uint8_t grey = (uint8_t)(centreGrey + t * (edgeGrey - centreGrey));
            gfx.drawPixel(cx + dx, cy + dy, grey);
        }
    }
}

// ---------------------------------------------------------------------------
// drawPage1() — dither demo
// ---------------------------------------------------------------------------
static void drawPage1(EPD_PainterAdafruit& epd) {
    const int W      = epd.width();
    const int H      = epd.height();
    const int margin = 20;

    epd.fillScreen(255);

    epd.setTextColor(0);
    epd.setTextSize(3);
    epd.setCursor(margin, margin);
    epd.print("EPD Dither Demo  8bpp -> 2bpp (Floyd-Steinberg)");

    // Full-width horizontal gradient bar
    const int barX = margin;
    const int barY = 58;
    const int barW = W - margin * 2;
    const int barH = 55;

    for (int x = 0; x < barW; x++) {
        uint8_t grey = (uint8_t)((x * 255) / (barW - 1));
        epd.drawFastVLine(barX + x, barY, barH, grey);
    }

    epd.setTextSize(1);
    const int ticks[] = { 0, 64, 128, 192, 255 };
    for (int i = 0; i < 5; i++) {
        int tx = barX + (ticks[i] * (barW - 1)) / 255;
        epd.drawFastVLine(tx, barY + barH, 4, 0);
        char buf[5];
        snprintf(buf, sizeof(buf), "%d", ticks[i]);
        epd.setCursor(tx - (i == 4 ? 12 : 0), barY + barH + 5);
        epd.print(buf);
    }

    // 32-patch swatch grid
    const int cols      = 16;
    const int rows      = 2;
    const int swatchW   = (W - margin * 2) / cols;
    const int swatchH   = 80;
    const int swatchGap = 14;
    const int gridY     = 132;

    epd.setTextSize(1);
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            int     idx  = row * cols + col;
            uint8_t grey = (uint8_t)((idx * 255) / 31);
            int sx = margin + col * swatchW;
            int sy = gridY  + row * (swatchH + swatchGap + 4);
            epd.fillRect(sx, sy, swatchW - 2, swatchH, grey);
            epd.drawRect(sx, sy, swatchW - 2, swatchH, 0);
            char buf[5];
            snprintf(buf, sizeof(buf), "%3d", grey);
            epd.setTextColor(0);
            epd.setCursor(sx + 2, sy + swatchH + 2);
            epd.print(buf);
        }
    }

    // Radial gradient circles
    const int circleY = 390;
    const int radius  = 60;
    const int spacing = (W - margin * 2) / 4;

    drawRadialGradient(epd, margin + spacing * 0 + radius, circleY, radius, 0,   255);
    drawRadialGradient(epd, margin + spacing * 1 + radius, circleY, radius, 100, 255);
    drawRadialGradient(epd, margin + spacing * 2 + radius, circleY, radius, 255, 0);
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            float dist = sqrtf((float)(dx * dx + dy * dy));
            if (dist > radius) continue;
            float   v    = (sinf(dist * 0.25f) + 1.0f) * 0.5f;
            uint8_t grey = (uint8_t)(v * 255.0f);
            int cx = margin + spacing * 3 + radius;
            epd.drawPixel(cx + dx, circleY + dy, grey);
        }
    }

    epd.setTextSize(1);
    epd.setTextColor(0);
    const char* clabels[] = { "0->255", "100->255", "255->0", "sine" };
    for (int i = 0; i < 4; i++) {
        int cx = margin + spacing * i + radius;
        epd.setCursor(cx - 12, circleY + radius + 4);
        epd.print(clabels[i]);
    }

    epd.setTextSize(1);
    epd.setTextColor(0);
    epd.setCursor(margin, H - 14);
    epd.print("0=white  1=lt grey  2=dk grey  3=black   -- dither() converts 8bpp to 2bpp in-place before paint()   [BOOT = next page]");
}

// ---------------------------------------------------------------------------
// drawPage2() — waveform grey calibration
//
// Two large squares, each split horizontally:
//   Square 1 (lt-grey 0x01)
//     Top    : solid fill 8bpp=170  (exact quantisation level, zero dither error)
//     Bottom : 1-in-3 diagonal pixel pattern  → 33.3 % black, 66.7 % white
//
//   Square 2 (dk-grey 0x02)
//     Top    : solid fill 8bpp=85   (exact quantisation level, zero dither error)
//     Bottom : 2-in-3 diagonal pixel pattern  → 66.7 % black, 33.3 % white
//
// If the waveform for a grey level is correctly tuned the two halves of each
// square will appear perceptually identical.
//
// Pattern: black when  (x + y*2) % 3 == 0
//   Gives exactly 1/3 black density; the y*2 offset shifts each row by 1 so
//   adjacent rows are not aligned (avoids visible vertical striping).
// ---------------------------------------------------------------------------
static void drawPage2(EPD_PainterAdafruit& epd) {
    const int W = epd.width();
    const int H = epd.height();

    epd.fillScreen(255);

    // Title
    epd.setTextColor(0);
    epd.setTextSize(2);
    epd.setCursor(10, 8);
    epd.print("Grey Waveform Calibration");
    epd.setTextSize(1);
    epd.setCursor(10, 32);
    epd.print("Top half = solid waveform grey.  Bottom half = dithered B+W at matching density.  Match = correct waveform.   [BOOT = prev page]");

    // Layout
    const int labelColW = 68;
    const int sqY       = 50;
    const int footerH   = 30;
    const int sqH       = H - sqY - footerH;
    const int halfH     = sqH / 2;
    const int gap       = 30;
    const int sqW       = (W - labelColW - gap) / 2;
    const int s1x       = labelColW;
    const int s2x       = labelColW + sqW + gap;
    const int midY      = sqY + halfH;

    // Left bracket annotations
    epd.setTextSize(1);
    epd.setTextColor(0);

    epd.drawFastVLine(labelColW - 4, sqY,   halfH, 0);
    epd.drawFastHLine(labelColW - 8, sqY,   4, 0);
    epd.drawFastHLine(labelColW - 8, midY - 1, 4, 0);

    epd.drawFastVLine(labelColW - 4, midY,  halfH, 0);
    epd.drawFastHLine(labelColW - 8, midY,  4, 0);
    epd.drawFastHLine(labelColW - 8, sqY + sqH - 1, 4, 0);

    epd.setCursor(2, sqY + halfH / 2 - 12);
    epd.print("Solid");
    epd.setCursor(2, sqY + halfH / 2 - 4);
    epd.print("grey");

    epd.setCursor(2, midY + halfH / 2 - 12);
    epd.print("Dithrd");
    epd.setCursor(2, midY + halfH / 2 - 4);
    epd.print("B+W");

    // --- Square 1: lt-grey ---
    epd.setTextSize(2);
    epd.setTextColor(0);
    epd.setCursor(s1x + sqW / 2 - 60, sqY - 18);
    epd.print("lt-grey 0x01");

    epd.fillRect(s1x, sqY, sqW, halfH, 170);

    for (int y = midY; y < sqY + sqH; y++)
        for (int x = s1x; x < s1x + sqW; x++)
            epd.drawPixel(x, y, ((x + y * 2) % 3 == 0) ? 0 : 255);

    epd.drawRect(s1x, sqY, sqW, sqH, 0);
    epd.drawFastHLine(s1x, midY, sqW, 0);

    epd.setTextSize(1);
    epd.setTextColor(0);
    epd.setCursor(s1x + 4, sqY + 4);
    epd.print("8bpp = 170  (solid waveform grey)");
    epd.setCursor(s1x + 4, midY + 4);
    epd.print("33.3% black + 66.7% white pixels");

    // --- Square 2: dk-grey ---
    epd.setTextSize(2);
    epd.setTextColor(0);
    epd.setCursor(s2x + sqW / 2 - 60, sqY - 18);
    epd.print("dk-grey 0x02");

    epd.fillRect(s2x, sqY, sqW, halfH, 85);

    for (int y = midY; y < sqY + sqH; y++)
        for (int x = s2x; x < s2x + sqW; x++)
            epd.drawPixel(x, y, ((x + y * 2) % 3 != 0) ? 0 : 255);

    epd.drawRect(s2x, sqY, sqW, sqH, 0);
    epd.drawFastHLine(s2x, midY, sqW, 0);

    epd.setTextSize(1);
    epd.setTextColor(255);
    epd.setCursor(s2x + 4, sqY + 4);
    epd.print("8bpp = 85   (solid waveform grey)");
    epd.setTextColor(0);
    epd.setCursor(s2x + 4, midY + 4);
    epd.print("66.7% black + 33.3% white pixels");

    // Footer
    epd.setTextSize(1);
    epd.setTextColor(0);
    epd.setCursor(10, H - 18);
    epd.print("0=white  1=lt-grey(8bpp=170)  2=dk-grey(8bpp=85)  3=black(0)   |   pattern: (x + y*2) % 3 == 0  -> 1/3 black, diagonal offset");
}

// ---------------------------------------------------------------------------
// showPage() — draw, dither, paint the requested page number
// ---------------------------------------------------------------------------
static void showPage(int page) {
    epd.clear();
    epd.clear();

    if (page == 1) {
        Serial.println("Drawing page 1...");
        drawPage1(epd);
    } else {
        Serial.println("Drawing page 2...");
        drawPage2(epd);
    }
    Serial.println("Dithering...");
    epd.dither();
    Serial.println("Painting...");
    epd.paint();
    Serial.println("Done.");
}

void setup() {
    Serial.begin(115200);
    pinMode(BOOT_BTN, INPUT_PULLUP);

    if (!epd.begin()) {
        Serial.println("EPD init failed");
        while (1);
    }
    //epd.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
    //epd.clear();
    //epd.clear();
    //epd.clear();


    showPage(currentPage);
}

void loop() {
#ifdef EPD_PAINTER_PRESET_M5PAPER_S3
    // M5PaperS3 has no user-accessible BOOT button — advance automatically
    delay(4000);
    currentPage = (currentPage == 1) ? 2 : 1;
    showPage(currentPage);
#else
    // Wait for BOOT button press (active LOW), with debounce
    if (digitalRead(BOOT_BTN) == LOW) {
        delay(50);
        if (digitalRead(BOOT_BTN) == LOW) {
            currentPage = (currentPage == 1) ? 2 : 1;
            showPage(currentPage);
            while (digitalRead(BOOT_BTN) == LOW)
                delay(10);
            delay(50);
        }
    }
#endif
}
