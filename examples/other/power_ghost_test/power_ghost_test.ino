// ============================================================================
// POWER GHOST TEST — minimal repro for the idle-power-cycle ghosts.
//
// Cycle: draw a filled square -> paint -> wait 6.5 s (the idle guard powers
// the panel down at 5 s, so every wait spans exactly one power-off and the
// next paint spans one power-on) -> erase the square -> paint -> wait.
//
// If the ghosts are real they appear within a few cycles as square-shaped
// residue/displacement that ordinary paints never clean. Serial prints a
// cycle counter alongside the library's power ON/OFF lines so the moment
// of appearance can be correlated. No WiFi, no touch, no app framework —
// just paint, idle, power cycle.
// ============================================================================

// Choose your board (or leave all commented for auto-probe).
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include "EPD_Painter.h"
#include "EPD_Painter_presets.h"

EPD_Painter epd(EPD_PAINTER_PRESET);

static uint8_t *fb;
static int W, H;
static uint32_t cycle = 0;

static void square(uint8_t level) {
  const int S = 220;
  const int x0 = (W - S) / 2, y0 = (H - S) / 2;
  for (int y = y0; y < y0 + S; y++)
    memset(fb + (size_t)y * W + x0, level, S);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("[ghost] boot, reset reason %d\n", (int)esp_reset_reason());

  epd.setAutoShutdown(false);          // rig board: no shutdown-on-reset
  if (!epd.begin()) {
    Serial.println("EPD init failed");
    while (1) delay(1000);
  }
  epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
  epd.setIdleTimeout(5);               // aggressive: every wait = one cycle

  W = epd.getConfig().width;
  H = epd.getConfig().height;
  fb = (uint8_t *)heap_caps_malloc((size_t)W * H, MALLOC_CAP_SPIRAM);
  if (!fb) { Serial.println("fb alloc failed"); while (1) delay(1000); }

  epd.clear();
  memset(fb, 0, (size_t)W * H);
  Serial.println("[ghost] running: square / 6.5 s / erase / 6.5 s ...");
}

void loop() {
  cycle++;

  square(3);                           // black square
  epd.paint(fb);
  Serial.printf("[ghost] cycle %lu: square painted\n", (unsigned long)cycle);
  delay(6500);                         // spans one power-off

  square(0);                           // erase it
  epd.paint(fb);
  Serial.printf("[ghost] cycle %lu: square erased\n", (unsigned long)cycle);
  delay(6500);                         // spans another power-off
}
