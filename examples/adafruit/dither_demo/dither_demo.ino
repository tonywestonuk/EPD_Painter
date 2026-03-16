// Choose your board.
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include "EPD_Painter_Adafruit.h"
#include "EPD_Painter_presets.h"

EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);

// ---------------------------------------------------------------------------
// drawRadialGradient()
//
// Fills a circle region with a smooth radial gradient: centre=grey, edge=white.
// Each pixel value is set proportional to its distance from the centre.
// ---------------------------------------------------------------------------
static void drawRadialGradient(EPD_PainterAdafruit& gfx,
                                int cx, int cy, int radius,
                                uint8_t centreGrey, uint8_t edgeGrey) {
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            float dist = sqrtf((float)(dx * dx + dy * dy));
            if (dist > radius) continue;
            float t = dist / radius;  // 0.0 at centre, 1.0 at edge
            uint8_t grey = (uint8_t)(centreGrey + t * (edgeGrey - centreGrey));
            gfx.drawPixel(cx + dx, cy + dy, grey);
        }
    }
}

void setup() {
    Serial.begin(115200);
    if (!epd.begin()) {
        Serial.println("EPD init failed");
        while (1);
    }
    epd.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
    epd.clear();

    const int W      = epd.width();   // 960
    const int H      = epd.height();  // 540
    const int margin = 20;

    // White background
    epd.fillScreen(255);

    // -------------------------------------------------------------------------
    // Title
    // -------------------------------------------------------------------------
    epd.setTextColor(0);
    epd.setTextSize(3);
    epd.setCursor(margin, margin);
    epd.print("EPD Dither Demo  8bpp -> 2bpp (Floyd-Steinberg)");

    // -------------------------------------------------------------------------
    // Full-width horizontal gradient bar  (0 = black on left, 255 = white right)
    // -------------------------------------------------------------------------
    const int barX = margin;
    const int barY = 58;
    const int barW = W - margin * 2;
    const int barH = 55;

    for (int x = 0; x < barW; x++) {
        uint8_t grey = (uint8_t)((x * 255) / (barW - 1));
        epd.drawFastVLine(barX + x, barY, barH, grey);
    }

    // Tick marks and labels at 0 / 64 / 128 / 192 / 255
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

    // -------------------------------------------------------------------------
    // 32-patch swatch grid  (16 columns x 2 rows)
    // Values evenly spread: 0, 8, 16 ... 248  (row 1) then 8, 24... (row 2)
    // -------------------------------------------------------------------------
    const int cols      = 16;
    const int rows      = 2;
    const int swatchW   = (W - margin * 2) / cols;   // 57 px
    const int swatchH   = 80;
    const int swatchGap = 14;   // space below each swatch for the value label
    const int gridY     = 132;

    epd.setTextSize(1);

    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            int   idx  = row * cols + col;
            // 32 evenly spaced values across 0-255
            uint8_t grey = (uint8_t)((idx * 255) / 31);

            int sx = margin + col * swatchW;
            int sy = gridY  + row * (swatchH + swatchGap + 4);

            // Swatch fill
            epd.fillRect(sx, sy, swatchW - 2, swatchH, grey);

            // Thin border so white swatches are visible
            epd.drawRect(sx, sy, swatchW - 2, swatchH, 0);

            // Value label below
            char buf[5];
            snprintf(buf, sizeof(buf), "%3d", grey);
            epd.setTextColor(0);
            epd.setCursor(sx + 2, sy + swatchH + 2);
            epd.print(buf);
        }
    }

    // -------------------------------------------------------------------------
    // Radial gradient circles — shows how smooth gradients dither
    // -------------------------------------------------------------------------
    const int circleY  = 390;
    const int radius   = 60;
    const int spacing  = (W - margin * 2) / 4;

    // Black core -> white edge
    drawRadialGradient(epd, margin + spacing * 0 + radius, circleY, radius, 0,   255);
    // Mid-grey core -> white edge
    drawRadialGradient(epd, margin + spacing * 1 + radius, circleY, radius, 100, 255);
    // White core -> black edge
    drawRadialGradient(epd, margin + spacing * 2 + radius, circleY, radius, 255, 0);
    // Alternating mid-grey bands (sine)
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            float dist = sqrtf((float)(dx * dx + dy * dy));
            if (dist > radius) continue;
            float v = (sinf(dist * 0.25f) + 1.0f) * 0.5f;  // 0..1
            uint8_t grey = (uint8_t)(v * 255.0f);
            int cx = margin + spacing * 3 + radius;
            epd.drawPixel(cx + dx, circleY + dy, grey);
        }
    }

    // Circle labels
    epd.setTextSize(1);
    epd.setTextColor(0);
    const char* clabels[] = { "0->255", "100->255", "255->0", "sine" };
    for (int i = 0; i < 4; i++) {
        int cx = margin + spacing * i + radius;
        epd.setCursor(cx - 12, circleY + radius + 4);
        epd.print(clabels[i]);
    }

    // -------------------------------------------------------------------------
    // Footer
    // -------------------------------------------------------------------------
    epd.setTextSize(1);
    epd.setTextColor(0);
    epd.setCursor(margin, H - 14);
    epd.print("0=white  1=lt grey  2=dk grey  3=black   -- dither() converts 8bpp to 2bpp in-place before paint()");

    // -------------------------------------------------------------------------
    // Dither and paint
    // -------------------------------------------------------------------------
    Serial.println("Dithering...");
    epd.dither();
    Serial.println("Painting...");
    epd.paint();
    Serial.println("Done.");
}

void loop() {
    delay(5000);
}
