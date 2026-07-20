// 16-grey test card — decision engine phase C gate (see DECISION_ENGINE.md).
//
// Drives the panel with 16 native grey levels via setGreyLevels(16). The
// canvas is plain 8bpp holding level codes 0..15 (0 = white … 15 = black);
// no dithering anywhere — every patch is a flat native level, which is the
// point: the gate is 16 *distinguishable* steps on glass.
//
// Serial commands:
//   b  staircase — 16 vertical bars, white to black, numbered by ticks
//   w  wedge     — continuous ramp quantised to nearest level (shows the
//                  band boundaries and checks the scale is monotonic)
//   i  inverse staircase — black to white (exercises remove-decisions:
//                  every bar boundary is a grey-to-grey transition)
//   c  clear     — full hard clear
//
// Trains are the phase C placeholder formula library; expect the levels to
// be distinguishable and monotonic but not evenly spaced — even spacing is
// phase D's scanner calibration.

// Choose your board (or leave all commented for auto-probe).
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_H752
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include "EPD_Painter.h"
#include "EPD_Painter_presets.h"
#include "tuned_trains_lilygo_t5s3.h"

EPD_Painter epd(EPD_PAINTER_PRESET);

static uint8_t *fb;
static int W, H;

// Paint twice: erased grey-to-grey pixels are redrawn on the second call
// (the engine's two-step transition — see DECISION_ENGINE.md).
static void show() {
  epd.paint(fb);
  epd.paint(fb);
}

static void drawStaircase(bool inverse) {
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      int level = (x * 16) / W;
      if (inverse) level = 15 - level;
      fb[(size_t)y * W + x] = (uint8_t)level;
    }
  // Tick marks: a black notch at every bar boundary, top and bottom.
  for (int b = 1; b < 16; b++) {
    const int x0 = (b * W) / 16;
    for (int y = 0; y < 20; y++)
      for (int x = x0 - 1; x <= x0; x++) {
        fb[(size_t)y * W + x] = 15;
        fb[(size_t)(H - 1 - y) * W + x] = 15;
      }
  }
}

// Match card (the dithertune method): top half = reference patches built
// from ONLY full white and full black pixels, Bayer-dithered to density
// g/15 — the optical truth for each level. Bottom half = the same 16
// levels as flat native waveform greys. A flatbed scan compares each pair
// directly; scanner gamma and lighting cancel because both halves share
// the glass. Trains are tuned over serial ('t') until the halves match.
static const uint8_t bayer8[8][8] = {
  {  0, 32,  8, 40,  2, 34, 10, 42 },
  { 48, 16, 56, 24, 50, 18, 58, 26 },
  { 12, 44,  4, 36, 14, 46,  6, 38 },
  { 60, 28, 52, 20, 62, 30, 54, 22 },
  {  3, 35, 11, 43,  1, 33,  9, 41 },
  { 51, 19, 59, 27, 49, 17, 57, 25 },
  { 15, 47,  7, 39, 13, 45,  5, 37 },
  { 63, 31, 55, 23, 61, 29, 53, 21 }
};

static void drawMatchCard() {
  const int mid = H / 2;
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      const int g = (x * 16) / W;
      uint8_t v;
      if (y < mid - 3) {
        const int thr = (g * 64 + 7) / 15;         // black-pixel density g/15
        v = (bayer8[y & 7][x & 7] < thr) ? 15 : 0;
      } else if (y < mid + 3) {
        v = 15;                                    // divider
      } else {
        v = (uint8_t)g;                            // native waveform level
      }
      fb[(size_t)y * W + x] = v;
    }
}

static void drawWedge() {
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      // 0..255 ramp, nearest-level quantise (no dither — native levels only)
      const int v = (x * 255) / (W - 1);
      fb[(size_t)y * W + x] = (uint8_t)((v * 15 + 127) / 255);
    }
}

void setup() {
  Serial.begin(115200);

  if (!epd.begin()) {
    Serial.println("EPD init failed");
    while (1) delay(1000);
  }
  epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
  if (!epd.setGreyLevels(16)) {
    Serial.println("setGreyLevels(16) failed");
    while (1) delay(1000);
  }
  // Scanner-tuned apply trains (LilyGo T5 S3 GPS, NORMAL). On other boards
  // these are approximate until the match-card loop is run for that panel.
  loadTunedTrains(epd);

  W = epd.getConfig().width;
  H = epd.getConfig().height;
  fb = (uint8_t *)heap_caps_malloc((size_t)W * H, MALLOC_CAP_SPIRAM);
  if (!fb) {
    Serial.println("framebuffer alloc failed");
    while (1) delay(1000);
  }

  epd.clear();
  drawStaircase(false);
  show();
  Serial.println("[grey16] staircase up. Commands: b=staircase w=wedge i=inverse c=clear");
}

// Calibration probe: replace the apply trains with a pure darken ladder —
// level g gets g darken passes, no whiten trick (levels 14/15 clamp to 13,
// so their bars duplicate level 13's). One scan of this staircase is the
// panel's raw dose-response curve at 13 points.
static void loadPureDarkenLadder() {
  uint8_t t[13];
  for (int g = 1; g <= 15; g++) {
    memset(t, 0, sizeof(t));
    const int n = g <= 13 ? g : 13;
    for (int p = 0; p < n; p++) t[p] = 1;
    epd.setDecisionTrain((uint8_t)((g << 1) | 0), t);
  }
}

void loop() {
  if (!Serial.available()) { delay(20); return; }
  switch (Serial.read()) {
    case 'b': drawStaircase(false); show(); Serial.println("[grey16] staircase");         break;
    case 'w': drawWedge();          show(); Serial.println("[grey16] wedge");             break;
    case 'i': drawStaircase(true);  show(); Serial.println("[grey16] inverse staircase"); break;
    case 'c': epd.clear();                  Serial.println("[grey16] cleared");           break;
    case 'p':
      loadPureDarkenLadder();
      epd.clear();
      drawStaircase(false); show();
      Serial.println("[grey16] pure-darken dose probe (bars 1..13 = 1..13 passes)");
      break;
    case 'm': drawMatchCard(); show(); Serial.println("[grey16] match card"); break;
    case 't': {
      // t <id> <13 codes 0-3>   e.g.  t 4 1112000000000
      const int id = Serial.parseInt();
      uint8_t t[13]; int got = 0;
      const unsigned long t0 = millis();
      while (got < 13 && millis() - t0 < 2000) {
        const int ch = Serial.read();
        if (ch < 0) { delay(2); continue; }
        if (ch >= '0' && ch <= '3') t[got++] = (uint8_t)(ch - '0');
        else if (ch == '\n') break;
      }
      if (got == 13) {
        epd.setDecisionTrain((uint8_t)id, t);
        Serial.printf("[grey16] train %d set\n", id);
      } else {
        Serial.println("[grey16] bad train line");
      }
    } break;
    case 'H':
      epd.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
      Serial.println("[grey16] QUALITY_HIGH");
      break;
    case 'N':
      epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
      Serial.println("[grey16] QUALITY_NORMAL");
      break;
    default: break;
  }
}
