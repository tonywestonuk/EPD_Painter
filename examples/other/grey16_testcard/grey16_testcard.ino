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
//   p  pure-darken dose probe        m  match card (dither references)
//   t <id> <13 codes>  upload a train over serial
//   g <level> <cycles>  DC ghost test: cycle paint/erase on left half
//   u <level>  uniform probe grey    F/M  formula / charge-matched removes
//   H/N  quality high / normal
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

// Performance test: 16 bouncing patches, one per grey level (level 0 kept
// visible by its black border), single paint() per frame. paint() returns
// when the paint task picks the frame up, so rendering the next frame
// overlaps driving the current one — the loop measures true engine
// throughput. Patch motion exercises apply, remove AND grey-to-grey
// (overlap) decisions every frame.
static void perfTest(int frames) {
  struct Patch { int x, y, vx, vy; };
  static Patch p[16];
  const int PW = 110, PH = 110;
  for (int i = 0; i < 16; i++) {
    p[i].x = (i % 4) * (W - PW) / 3;
    p[i].y = (i / 4) * (H - PH) / 3;
    p[i].vx = ((i % 5) + 4) * ((i & 1) ? -1 : 1);
    p[i].vy = (((i * 3) % 5) + 4) * ((i & 2) ? -1 : 1);
  }
  epd.clear();
  const uint32_t t0 = millis();
  for (int f = 0; f < frames; f++) {
    for (int i = 0; i < 16; i++) {
      p[i].x += p[i].vx; p[i].y += p[i].vy;
      if (p[i].x < 0)      { p[i].x = 0;      p[i].vx = -p[i].vx; }
      if (p[i].y < 0)      { p[i].y = 0;      p[i].vy = -p[i].vy; }
      if (p[i].x > W - PW) { p[i].x = W - PW; p[i].vx = -p[i].vx; }
      if (p[i].y > H - PH) { p[i].y = H - PH; p[i].vy = -p[i].vy; }
    }
    memset(fb, 0, (size_t)W * H);
    for (int i = 0; i < 16; i++) {
      for (int y = 0; y < PH; y++) {
        uint8_t *row = fb + (size_t)(p[i].y + y) * W + p[i].x;
        if (y < 3 || y >= PH - 3) memset(row, 15, PW);
        else {
          row[0] = row[1] = row[2] = 15;
          row[PW - 3] = row[PW - 2] = row[PW - 1] = 15;
          memset(row + 3, i, PW - 6);
        }
      }
    }
    epd.paint(fb);
    if ((f + 1) % 20 == 0)
      Serial.printf("[grey16] perf %d/%d: %.2f fps\n", f + 1, frames,
                    (f + 1) * 1000.0f / (millis() - t0));
  }
  const uint32_t dt = millis() - t0;
  Serial.printf("[grey16] perf done: %d frames, %lu ms, %.2f fps (%.0f ms/frame)\n",
                frames, (unsigned long)dt, frames * 1000.0f / dt,
                (float)dt / frames);
}

// DC-balance ghost test (phase D). Cycle paint/erase of a level on the
// LEFT half only; the right half stays virgin. If removes are charge-
// matched the two halves' ledgers are identical afterwards, and a uniform
// probe grey ('u') renders seamlessly across the midline. An unbalanced
// remove leaves the cycled half charge-shifted — the probe shows a ghost
// boundary at x = W/2.
static void ghostCycles(int level, int cycles) {
  epd.clear();
  for (int n = 0; n < cycles; n++) {
    for (int y = 0; y < H; y++)
      for (int x = 0; x < W; x++)
        fb[(size_t)y * W + x] = (x < W / 2) ? (uint8_t)level : 0;
    show();
    memset(fb, 0, (size_t)W * H);
    show();
    Serial.printf("[grey16] ghost cycle %d/%d\n", n + 1, cycles);
  }
}

// Control arm: the phase C formula removes (pure whiten runs, full+3 with
// margin — optically fine, NOT charge-matched). Lets a ghost run measure
// the imbalance the tuned removes are supposed to eliminate.
static void loadFormulaRemoves() {
  uint8_t t[13];
  for (int g = 1; g <= 15; g++) {
    const int dose = (g * 2 * 13 + 7) / 15;
    int wh = dose / 2 + 3;
    if (wh > 13) wh = 13;
    memset(t, 0, sizeof(t));
    for (int p = 0; p < wh; p++) t[p] = 2;
    epd.setDecisionTrain((uint8_t)((g << 1) | 1), t);
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
    case 'x': {
      // x <frames>   bouncing-patch performance test
      int frames = Serial.parseInt();
      if (frames < 1) frames = 60;
      if (frames > 2000) frames = 2000;
      perfTest(frames);
    } break;
    case 'g': {
      // g <level> <cycles>   paint/erase LEFT half N times, end white
      const int level  = Serial.parseInt();
      const int cycles = Serial.parseInt();
      if (level >= 1 && level <= 15 && cycles >= 1 && cycles <= 200) {
        ghostCycles(level, cycles);
        Serial.printf("[grey16] ghost done: L%d x%d, screen white\n", level, cycles);
      } else Serial.println("[grey16] bad ghost args");
    } break;
    case 'u': {
      // u <level>   uniform full-screen probe grey
      const int level = Serial.parseInt();
      if (level >= 0 && level <= 15) {
        memset(fb, (uint8_t)level, (size_t)W * H);
        show();
        Serial.printf("[grey16] uniform L%d\n", level);
      } else Serial.println("[grey16] bad level");
    } break;
    case 'F':
      loadFormulaRemoves();
      Serial.println("[grey16] FORMULA removes loaded (unbalanced control)");
      break;
    case 'M':
      loadTunedTrains(epd);
      Serial.println("[grey16] charge-matched trains loaded");
      break;
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
