// Diagonal-line test pattern — verifies row-end padding on new panels.
//
// Draws 1px 45° diagonal lines 12px apart across the full panel. On a
// healthy setup the lines run evenly spaced all the way to the right-hand
// edge. If the panel's source-driver shift chain needs more flush clocks
// than Config::row_pad_bytes provides, the image repeats in a narrow band
// at the right edge: ghost lines appear interleaved with the real ones.
// See How_It_Works.md §7 "Row-End Padding".

// Choose your board.
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_H752
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include "EPD_Painter_Adafruit.h"

EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);

void setup() {
  Serial.begin(115200);
  if (!epd.begin()) {
    Serial.println("EPD init failed");
    while (1) delay(1000);
  }
  epd.clear();

  const int w = epd.width();    // 960
  const int h = epd.height();   // 540

  epd.fillScreen(0);

  // 45° diagonals ("\" direction), one pixel wide, 12px apart.
  // Start off-screen left so the whole panel is covered.
  for (int x = -h; x < w; x += 12) {
    epd.drawLine(x, 0, x + h, h, 3);
  }

  epd.paint();
  Serial.println("Pattern painted — check the right edge for ghost lines");
}

void loop() {
  delay(1000);
}
