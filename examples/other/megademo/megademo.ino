// ============================================================================
// EPD_PAINTER MEGADEMO — an Amiga-style demo that IS the sales pitch.
//
// Six screens, about two minutes, looping. Every effect is also a feature
// demonstration:
//
//   1 INTRO    copper bars + bouncing logo + sine scroller  (FAST, ~17 fps)
//   2 BOING    the checkered ball, of course                (delta engine)
//   3 GREYS    live switch to 16 native greys, plasma       (decision engine)
//   4 STARS    3D starfield                                 (SIMD + DMA)
//   5 MEGA     giant wave text scroller: the feature list   (FAST again)
//   6 CREDITS  16-grey radial glow + greetz                 (DC balanced)
//
// Works on the M5PaperS3 and LilyGo T5 S3 GPS (auto-probe); loads each
// board's scanner-calibrated 16-grey trains for screens 3 and 6.
// ============================================================================

// Choose your board (or leave all commented for auto-probe).
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include "EPD_Painter.h"
#include "EPD_Painter_presets.h"
#include "font8x8_basic.h"
// Scanner-calibrated 16-grey trains live with the testcard example.
#include "../grey16_testcard/tuned_trains_lilygo_t5s3.h"
#include "../grey16_testcard/tuned_trains_m5papers3.h"

EPD_Painter epd(EPD_PAINTER_PRESET);

static uint8_t *fb;
static int W, H;
static uint8_t sinT[256];         // 0..255 sine table

static void loadBoardTrains() {
  if (epd.getConfig().pin_syspwr >= 0) loadTunedTrainsM5PaperS3(epd);
  else                                 loadTunedTrains(epd);
}

// ---------------------------------------------------------------------------
// Mode helpers. Switching bit depth mid-demo is itself the demo: the same
// canvas drives 4-level FAST motion and 16-grey NORMAL stills.
// ---------------------------------------------------------------------------
static void modeFast4() {          // colours 0..3, ~17 fps full motion
  epd.setGreyLevels(4);
  epd.setQuality(EPD_Painter::Quality::QUALITY_FAST);
}
static void modeGrey16() {         // colours 0..15, ~4 fps, calibrated
  epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
  epd.setGreyLevels(16);
  loadBoardTrains();
}

// ---------------------------------------------------------------------------
// Text: scaled 8x8 font, transparent background.
// ---------------------------------------------------------------------------
static void drawGlyph(char ch, int x, int y, int sc, uint8_t col) {
  if (ch < 0) return;
  const uint8_t *g = (const uint8_t *)font8x8_basic[(int)ch];
  for (int gy = 0; gy < 8; gy++) {
    const uint8_t bits = g[gy];
    if (!bits) continue;
    for (int gx = 0; gx < 8; gx++) {
      if (!(bits & (1 << gx))) continue;
      const int px = x + gx * sc, py = y + gy * sc;
      for (int sy = 0; sy < sc; sy++) {
        if (py + sy < 0 || py + sy >= H) continue;
        uint8_t *row = fb + (size_t)(py + sy) * W;
        for (int sx = 0; sx < sc; sx++) {
          const int xx = px + sx;
          if (xx >= 0 && xx < W) row[xx] = col;
        }
      }
    }
  }
}

static void drawText(const char *s, int x, int y, int sc, uint8_t col) {
  for (; *s; s++, x += 8 * sc) drawGlyph(*s, x, y, sc, col);
}

static int textWidth(const char *s, int sc) { return strlen(s) * 8 * sc; }

static void drawTextCentred(const char *s, int y, int sc, uint8_t col) {
  drawText(s, (W - textWidth(s, sc)) / 2, y, sc, col);
}

// ---------------------------------------------------------------------------
// Screen 1 — INTRO. Copper bars, drop-shadow logo on a bounce, sine scroller.
// ---------------------------------------------------------------------------
static const char SCROLL1[] =
  "   *** EPD_PAINTER MEGADEMO ***   THE E-PAPER ENGINE THAT MOVES   "
  "...   FULL MOTION ON GLASS AT 17 FPS   ...   NO FLASHING   ...   "
  "NO GHOSTS   ...   KEEP WATCHING FOR 16 REAL GREYS   ...      ";

static void screenIntro(uint32_t ms) {
  modeFast4();
  epd.clear();
  const uint32_t t0 = millis();
  int sx = W;                       // scroller x
  for (uint32_t f = 0; millis() - t0 < ms; f++) {
    // copper bars: horizontal grey bands drifting down
    for (int y = 0; y < H; y++) {
      const uint8_t v = sinT[((y * 2) - f * 6) & 255];
      const uint8_t band = (v > 208) ? 2 : (v > 176) ? 1 : 0;
      memset(fb + (size_t)y * W, band, W);
    }
    // bouncing logo with a shadow pass
    const int by = 120 + (abs((int)(f * 7 % 220) - 110) * 100) / 110 - 50;
    drawTextCentred("EPD_PAINTER", by + 6, 10, 2);
    drawTextCentred("EPD_PAINTER", by, 10, 3);
    drawTextCentred("M E G A D E M O", by + 100, 4, 3);
    // sine scroller along the bottom
    int x = sx;
    for (const char *p = SCROLL1; *p && x < W; p++, x += 32) {
      if (x > -32)
        drawGlyph(*p, x, H - 90 + (sinT[(x + f * 5) & 255] - 128) / 4, 4, 3);
    }
    sx -= 12;
    if (sx < -(int)textWidth(SCROLL1, 4)) sx = W;
    epd.paint(fb);
  }
}

// ---------------------------------------------------------------------------
// Screen 2 — BOING. The ball. A grid. An fps counter earning its keep.
// ---------------------------------------------------------------------------
static uint8_t *ballLat, *ballLon;   // per-pixel sphere coords, 255 = outside
static const int BALLR = 110;           // ball radius

static void ballInit() {
  const int D = BALLR * 2;
  ballLat = (uint8_t *)heap_caps_malloc(D * D, MALLOC_CAP_SPIRAM);
  ballLon = (uint8_t *)heap_caps_malloc(D * D, MALLOC_CAP_SPIRAM);
  for (int dy = -BALLR; dy < BALLR; dy++)
    for (int dx = -BALLR; dx < BALLR; dx++) {
      const int i = (dy + BALLR) * D + (dx + BALLR);
      const float r2 = (float)(dx * dx + dy * dy) / (BALLR * BALLR);
      if (r2 >= 1.0f) { ballLat[i] = 255; continue; }
      const float z = sqrtf(1.0f - r2);
      ballLat[i] = (uint8_t)((asinf((float)dy / BALLR) / M_PI + 0.5f) * 127);
      ballLon[i] = (uint8_t)((atan2f((float)dx / BALLR, z) / M_PI + 0.5f) * 127);
    }
}

static void screenBoing(uint32_t ms) {
  modeFast4();
  epd.clear();
  if (!ballLat) ballInit();
  const uint32_t t0 = millis();
  uint32_t frames = 0, lastT = t0;
  float bx = W / 2, by = H / 2, vx = 11, vy = 9;
  char fpsTxt[24] = "-- FPS";
  for (uint32_t f = 0; millis() - t0 < ms; f++, frames++) {
    // background: sparse grid (the classic purple grid, in spirit)
    memset(fb, 0, (size_t)W * H);
    for (int y = 0; y < H; y += 60)
      memset(fb + (size_t)y * W, 1, W);
    for (int x = 0; x < W; x += 60)
      for (int y = 0; y < H; y++) fb[(size_t)y * W + x] = 1;
    // ball
    bx += vx; by += vy;
    if (bx < BALLR)     { bx = BALLR;     vx = -vx; }
    if (bx > W - BALLR) { bx = W - BALLR; vx = -vx; }
    if (by < BALLR)     { by = BALLR;     vy = -vy; }
    if (by > H - BALLR) { by = H - BALLR; vy = -vy; }
    const int cx = (int)bx, cy = (int)by, D = BALLR * 2;
    const uint8_t rot = (uint8_t)(f * 3);
    for (int dy = -BALLR; dy < BALLR; dy++) {
      uint8_t *row = fb + (size_t)(cy + dy) * W;
      const uint8_t *latR = ballLat + (dy + BALLR) * D;
      const uint8_t *lonR = ballLon + (dy + BALLR) * D;
      for (int dx = -BALLR; dx < BALLR; dx++) {
        const uint8_t lat = latR[dx + BALLR];
        if (lat == 255) continue;
        const uint8_t cell = ((lat >> 4) ^ ((uint8_t)(lonR[dx + BALLR] + rot) >> 4)) & 1;
        row[cx + dx] = cell ? 3 : 0;
      }
    }
    // captions
    drawText("DELTA ENGINE:", 20, 20, 3, 3);
    drawText("ONLY CHANGED PIXELS HIT THE GLASS", 20, 50, 2, 3);
    if (millis() - lastT > 2000) {
      snprintf(fpsTxt, sizeof(fpsTxt), "%lu FPS",
               (unsigned long)(frames * 1000 / (millis() - t0)));
      lastT = millis();
    }
    drawText(fpsTxt, W - 180, 20, 3, 3);
    epd.paint(fb);
  }
}

// ---------------------------------------------------------------------------
// Screen 3 — GREYS. Live switch to the decision engine: 16-grey plasma.
// ---------------------------------------------------------------------------
static void screenGreys(uint32_t ms) {
  modeGrey16();
  epd.clear();
  const uint32_t t0 = millis();
  for (uint32_t f = 0; millis() - t0 < ms; f++) {
    const int t = f * 9;
    for (int y = 0; y < H; y++) {
      uint8_t *row = fb + (size_t)y * W;
      const uint8_t sy = sinT[((y * 3) / 2 - t) & 255];
      for (int x = 0; x < W; x++) {
        const int v = sinT[(x + t) & 255] + sy +
                      sinT[((x + y) + t * 2) & 255] / 2;
        row[x] = (uint8_t)((v * 15) / 638);
      }
    }
    // swatch strip: the 16 levels, labelled
    for (int b = 0; b < 16; b++) {
      const int x0 = (b * W) / 16, x1 = ((b + 1) * W) / 16;
      for (int y = H - 70; y < H - 20; y++)
        memset(fb + (size_t)y * W + x0, b, x1 - x0);
    }
    drawTextCentred("16 NATIVE GREYS", 30, 6, 15);
    drawTextCentred("SCANNER-CALIBRATED WAVEFORMS", 90, 3, 15);
    drawTextCentred("THIS MODE SWITCH HAPPENED LIVE", 130, 2, 0);
    epd.paint(fb);
  }
}

// ---------------------------------------------------------------------------
// Screen 4 — STARS. Starfield + the engine-room name-dropping.
// ---------------------------------------------------------------------------
static void screenStars(uint32_t ms) {
  modeFast4();
  epd.clear();
  struct Star { int16_t x, y; uint16_t z; };
  static Star st[220];
  for (int i = 0; i < 220; i++)
    st[i] = { (int16_t)random(-500, 500), (int16_t)random(-500, 500),
              (uint16_t)random(64, 1024) };
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    memset(fb, 3, (size_t)W * H);          // space is black
    for (int i = 0; i < 220; i++) {
      st[i].z -= 14;
      if (st[i].z < 32) st[i].z = 1024;
      const int sx = W / 2 + st[i].x * 256 / st[i].z;
      const int sy = H / 2 + st[i].y * 256 / st[i].z;
      const int s = st[i].z < 200 ? 3 : st[i].z < 500 ? 2 : 1;
      for (int dy = 0; dy < s; dy++)
        for (int dx = 0; dx < s; dx++)
          if (sx + dx >= 0 && sx + dx < W && sy + dy >= 0 && sy + dy < H)
            fb[(size_t)(sy + dy) * W + sx + dx] = st[i].z < 400 ? 0 : 1;
    }
    drawTextCentred("XTENSA SIMD ASSEMBLY", 60, 4, 0);
    drawTextCentred("LCD_CAM DMA + DOUBLE-BUFFERED ROWS", 120, 2, 1);
    drawTextCentred("518KB CANVAS COMPACTED EVERY FRAME", 150, 2, 1);
    epd.paint(fb);
  }
}

// ---------------------------------------------------------------------------
// Screen 5 — MEGA. Giant wave scroller: why you should use it.
// ---------------------------------------------------------------------------
static const char SCROLL2[] =
  "      WHY EPD_PAINTER?   ...   ADAFRUIT GFX AND LVGL, READY TO GO   ...   "
  "BAD APPLE AT 20 FPS   ...   M5PAPERS3 + LILYGO T5S3, AUTO-PROBED   ...   "
  "CHARGE-MATCHED INK KEEPS YOUR PANEL YOUNG   ...   "
  "ONE LIBRARY, FOUR SPEEDS, SIXTEEN GREYS   ...   GET IT ON GITHUB      ";

static void screenMega(uint32_t ms) {
  modeFast4();
  epd.clear();
  const uint32_t t0 = millis();
  int sx = W;
  for (uint32_t f = 0; millis() - t0 < ms; f++) {
    // checker backdrop, gently scrolling
    for (int y = 0; y < H; y++) {
      uint8_t *row = fb + (size_t)y * W;
      const int cy = ((y + f * 2) / 48) & 1;
      for (int x = 0; x < W; x += 48)
        memset(row + x, ((x / 48 + cy) & 1) ? 1 : 0, min(48, W - x));
    }
    int x = sx;
    for (const char *p = SCROLL2; *p && x < W; p++, x += 64) {
      if (x > -64) {
        const int yy = H / 2 - 64 + (sinT[(x / 2 + f * 6) & 255] - 128);
        drawGlyph(*p, x + 5, yy + 5, 8, 1);   // shadow
        drawGlyph(*p, x, yy, 8, 3);
      }
    }
    sx -= 16;
    if (sx < -(int)textWidth(SCROLL2, 8)) sx = W;
    epd.paint(fb);
  }
}

// ---------------------------------------------------------------------------
// Screen 6 — CREDITS. 16-grey radial glow, greetz, DC-balance flex.
// ---------------------------------------------------------------------------
static void screenCredits(uint32_t ms) {
  modeGrey16();
  epd.clear();
  for (int y = 0; y < H; y++) {
    uint8_t *row = fb + (size_t)y * W;
    for (int x = 0; x < W; x++) {
      const int dx = x - W / 2, dy = (y - H / 2) * 16 / 9;
      const int d = (int)sqrtf((float)(dx * dx + dy * dy));
      int v = d / 38;
      row[x] = (uint8_t)(v > 15 ? 15 : v);
    }
  }
  drawTextCentred("EPD_PAINTER", 90, 8, 15);
  drawTextCentred("CODE + WAVEFORM PHYSICS: TONY WESTON", 200, 2, 15);
  drawTextCentred("CALIBRATION RIG: A FLATBED SCANNER", 230, 2, 15);
  drawTextCentred("AND A VERY PATIENT AI", 260, 2, 15);
  drawTextCentred("GREETZ: M5STACK * LILYGO * DEMOSCENE ETERNAL", 310, 2, 15);
  drawTextCentred("EVERY PIXEL DC-BALANCED. YOUR PANEL SAYS THANKS.", 360, 2, 15);
  drawTextCentred("GITHUB.COM/TONYWESTONUK/EPD_PAINTER", 420, 3, 0);
  epd.paint(fb);
  epd.paint(fb);                    // stills get the two-step mop-up
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) delay(50);
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  epd.setAutoShutdown(false);
  if (!epd.begin()) {
    Serial.println("EPD init failed");
    while (1) delay(1000);
  }
  W = epd.getConfig().width;
  H = epd.getConfig().height;
  fb = (uint8_t *)heap_caps_malloc((size_t)W * H, MALLOC_CAP_SPIRAM);
  if (!fb) {
    Serial.println("framebuffer alloc failed");
    while (1) delay(1000);
  }
  for (int i = 0; i < 256; i++)
    sinT[i] = (uint8_t)(128 + 127 * sinf(i * M_PI / 128));
  Serial.println("[megademo] here we go");
}

void loop() {
  screenIntro(22000);
  screenBoing(22000);
  screenGreys(24000);
  screenStars(15000);
  screenMega(24000);
  screenCredits(13000);
}
