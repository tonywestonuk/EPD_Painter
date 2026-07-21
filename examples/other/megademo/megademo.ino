// ============================================================================
// EPD_PAINTER MEGADEMO — an Amiga-style demo that IS the sales pitch.
//
// Six screens, about two minutes, looping. Every effect is also a feature
// demonstration:
//
//   1 INTRO    shadowed bouncing logo + sine scroller  (direct grey-to-grey)
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
#include "../grey16_testcard/direct_trains_m5papers3.h"
#include "../grey16_testcard/direct_trains_lilygo_t5s3.h"

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
static void modeFast4Direct() {    // colours 0..3, ~17 fps + direct grey-to-grey
  epd.setGreyLevels(4);
  epd.setQuality(EPD_Painter::Quality::QUALITY_FAST);
  epd.setDirectTransitions(true);
  // Direct trains are per-board and per-quality (both boards tuned).
  if (epd.getConfig().pin_syspwr >= 0) loadDirectTrainsM5PaperS3Fast(epd);
  else                                 loadDirectTrainsLilygoFast(epd);
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
  Serial.printf("[demo] screenIntro t=%lu\n", millis());
  modeFast4Direct();
  epd.clear();
  const uint32_t t0 = millis();
  int sx = W;                       // scroller x
  for (uint32_t f = 0; millis() - t0 < ms; f++) {
    // Copper bars AND the drop-shadow are BACK — both were casualties of
    // the artifact hunt. Black letters ploughing into their own grey
    // shadow was a grey->grey two-step (erase to white this paint,
    // redraw next), punching white holes through descending strokes —
    // Tony spotted it. Now every grey-to-grey pixel rides a tuned FAST
    // direct train in one paint, and the drifting bars are a full-width
    // sheet of grey-to-grey transitions every frame: this screen IS the
    // engine's stress gate.
    for (int y = 0; y < H; y++) {
      const uint8_t v = sinT[((y * 2) - f * 6) & 255];
      const uint8_t band = (v > 208) ? 2 : (v > 176) ? 1 : 0;
      memset(fb + (size_t)y * W, band, W);
    }
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
// Screen 2 — BOING. The ball, promoted to the decision engine: true 3D in
// a perspective room, 16-grey shaded checker sphere, gravity bounce and a
// floor shadow that softens with height. NORMAL mode, ~4 fps — e-paper
// doing something no e-paper demo has business doing.
// ---------------------------------------------------------------------------
static uint8_t *ballLat, *ballLon, *ballShade;  // sphere LUTs, 255 = outside
static const int BALLR = 110;                   // LUT radius (native size)

static void ballInit() {
  const int D = BALLR * 2;
  ballLat   = (uint8_t *)heap_caps_malloc(D * D, MALLOC_CAP_SPIRAM);
  ballLon   = (uint8_t *)heap_caps_malloc(D * D, MALLOC_CAP_SPIRAM);
  ballShade = (uint8_t *)heap_caps_malloc(D * D, MALLOC_CAP_SPIRAM);
  for (int dy = -BALLR; dy < BALLR; dy++)
    for (int dx = -BALLR; dx < BALLR; dx++) {
      const int i = (dy + BALLR) * D + (dx + BALLR);
      const float r2 = (float)(dx * dx + dy * dy) / (BALLR * BALLR);
      if (r2 >= 1.0f) { ballLat[i] = 255; continue; }
      const float z  = sqrtf(1.0f - r2);
      const float nx = (float)dx / BALLR, ny = (float)dy / BALLR;
      ballLat[i] = (uint8_t)((asinf(ny) / M_PI + 0.5f) * 127);
      ballLon[i] = (uint8_t)((atan2f(nx, z) / M_PI + 0.5f) * 127);
      float lit = -0.45f * nx - 0.60f * ny + 0.66f * z;   // light up-left-front
      if (lit < 0) lit = 0;
      ballShade[i] = (uint8_t)(lit * 255.0f);
    }
}

static void screenBoing(uint32_t ms) {
  Serial.printf("[demo] screenBoing t=%lu\n", millis());
  modeGrey16();
  epd.clear();
  epd.clear();   // twice: lift the FAST screen's ghost before grey work
  if (!ballLat) ballInit();
  const int HOR = H / 2 - 60;                 // horizon
  const int NEARY = H - 26;                   // nearest floor edge
  auto groundY = [&](float z) { return (int)(NEARY - z * (NEARY - (HOR + 18))); };
  float wx = -0.4f, vx = 0.13f;               // world x, -1..1
  float hgt = 240, vh = 0;                    // height above floor (px)
  float zp = 0.1f, vz = 0.06f;                // depth, 0 near .. 1 far
  const uint32_t t0 = millis();
  uint32_t frames = 0;
  char fpsTxt[20] = "";
  while (millis() - t0 < ms) {
    frames++;
    memset(fb, 0, (size_t)W * H);
    // floor: depth rulings pack toward the horizon...
    for (int i = 0; i <= 8; i++) {
      const int y = groundY(sqrtf(i / 8.0f));
      memset(fb + (size_t)y * W, 5, W);
      memset(fb + (size_t)(y + 1) * W, 5, W);
    }
    // ...and converging rails toward the vanishing point
    for (int i = -6; i <= 6; i++) {
      const int xn = W / 2 + i * 130;         // spacing at the near edge
      for (int y = HOR + 18; y <= NEARY; y++) {
        const float f = (float)(y - HOR) / (NEARY - HOR);
        const int xx = W / 2 + (int)((xn - W / 2) * f);
        if (xx >= 0 && xx < W) fb[(size_t)y * W + xx] = 5;
      }
    }
    memset(fb + (size_t)HOR * W, 5, W);       // horizon line
    // physics
    wx += vx; if (wx < -1) { wx = -1; vx = -vx; } if (wx > 1) { wx = 1; vx = -vx; }
    zp += vz; if (zp < 0)  { zp = 0;  vz = -vz; } if (zp > 1) { zp = 1; vz = -vz; }
    vh -= 12; hgt += vh;
    if (hgt < 0) { hgt = 0; vh = 64; }        // steady boing
    const float s = 1.0f - 0.52f * zp;        // perspective scale
    const int r  = (int)(96 * s);
    const int bx = W / 2 + (int)(wx * (W / 2 - 160) * (1.0f - 0.30f * zp));
    const int gy = groundY(zp);
    const int cy = gy - r - (int)(hgt * s);
    // shadow: bigger and lighter the higher the ball
    {
      const int rx = r - 8 + (int)(hgt * 0.06f);
      const int ry = rx / 3;
      const uint8_t lvl = (uint8_t)(9 - min(5, (int)(hgt / 55)));
      for (int dy = -ry; dy <= ry; dy++) {
        const int y = gy - 6 + dy;
        if (y < 0 || y >= H) continue;
        const int hw = (int)(rx * sqrtf(1.0f - (float)(dy * dy) / (ry * ry)));
        uint8_t *row = fb + (size_t)y * W;
        for (int dx = -hw; dx <= hw; dx++) {
          const int xx = bx + dx;
          if (xx >= 0 && xx < W) row[xx] = lvl;
        }
      }
    }
    // ball: sample the native-size LUTs at the perspective radius
    const uint8_t rot = (uint8_t)(frames * 6);
    const int D = BALLR * 2;
    for (int sy = -r; sy <= r; sy++) {
      const int y = cy + sy;
      if (y < 0 || y >= H) continue;
      uint8_t *row = fb + (size_t)y * W;
      const int ly = (sy * BALLR) / r + BALLR;
      const uint8_t *latR = ballLat   + ly * D;
      const uint8_t *lonR = ballLon   + ly * D;
      const uint8_t *shdR = ballShade + ly * D;
      for (int sx = -r; sx <= r; sx++) {
        const int xx = bx + sx;
        if (xx < 0 || xx >= W) continue;
        const int lx = (sx * BALLR) / r + BALLR;
        const uint8_t lat = latR[lx];
        if (lat == 255) continue;
        const uint8_t shade = shdR[lx];
        const uint8_t cell = ((lat >> 4) ^ (uint8_t)((uint8_t)(lonR[lx] + rot) >> 4)) & 1;
        row[xx] = cell ? (uint8_t)(15 - (shade * 6) / 256)   // dark squares  9..15
                       : (uint8_t)(8 - (shade * 6) / 256);   // light squares 2..8
      }
    }
    drawTextCentred("THE BALL. NOW IN 16 GREYS.", 16, 4, 15);
    drawTextCentred("TRUE 3D * NORMAL MODE * DECISION ENGINE", 58, 2, 10);
    if (frames % 8 == 0)
      snprintf(fpsTxt, sizeof(fpsTxt), "%.1f FPS",
               frames * 1000.0f / (millis() - t0 + 1));
    drawText(fpsTxt, 16, H - 30, 2, 10);
    epd.paint(fb);
  }
}

// ---------------------------------------------------------------------------
// Screen 3 — GREYS. Live switch to the decision engine: 16 bouncing balls,
// one per native grey level, each with a black ring so white plays too.
// ---------------------------------------------------------------------------
static void screenGreys(uint32_t ms) {
  Serial.printf("[demo] screenGreys t=%lu\n", millis());
  modeGrey16();
  epd.clear();
  const int R = 52;
  static int16_t hw[2 * 52 + 1], hwi[2 * 52 + 1];   // circle half-widths
  for (int dy = -R; dy <= R; dy++) {
    hw[dy + R] = (int16_t)sqrtf((float)(R * R - dy * dy));
    const int ri = R - 4;                            // ring thickness 4
    hwi[dy + R] = (abs(dy) <= ri)
                    ? (int16_t)sqrtf((float)(ri * ri - dy * dy)) : -1;
  }
  struct Ball { int x, y, vx, vy; };
  Ball b[16];
  for (int i = 0; i < 16; i++) {
    b[i].x = R + ((i % 4) * (W - 2 * R)) / 3;
    b[i].y = R + ((i / 4) * (H - 2 * R)) / 3;
    b[i].vx = ((i % 5) + 10) * ((i & 1) ? -1 : 1);
    b[i].vy = (((i * 3) % 5) + 10) * ((i & 2) ? -1 : 1);
  }
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    memset(fb, 0, (size_t)W * H);
    for (int i = 0; i < 16; i++) {
      b[i].x += b[i].vx; b[i].y += b[i].vy;
      if (b[i].x < R)     { b[i].x = R;     b[i].vx = -b[i].vx; }
      if (b[i].x > W - R) { b[i].x = W - R; b[i].vx = -b[i].vx; }
      if (b[i].y < R)     { b[i].y = R;     b[i].vy = -b[i].vy; }
      if (b[i].y > H - R) { b[i].y = H - R; b[i].vy = -b[i].vy; }
      for (int dy = -R; dy <= R; dy++) {
        uint8_t *row = fb + (size_t)(b[i].y + dy) * W + b[i].x;
        const int w = hw[dy + R], wi = hwi[dy + R];
        if (wi < 0) {                                 // ring-only rows
          for (int dx = -w; dx <= w; dx++) row[dx] = 15;
        } else {
          for (int dx = -w;  dx < -wi; dx++) row[dx] = 15;
          for (int dx = -wi; dx <= wi; dx++) row[dx] = (uint8_t)i;
          for (int dx = wi + 1; dx <= w; dx++) row[dx] = 15;
        }
      }
    }
    drawTextCentred("16 NATIVE GREYS", 20, 6, 15);
    drawTextCentred("SCANNER-CALIBRATED WAVEFORMS", 80, 3, 15);
    drawTextCentred("THIS MODE SWITCH HAPPENED LIVE", H - 40, 2, 15);
    epd.paint(fb);
  }
}

// ---------------------------------------------------------------------------
// Screen 4 — STARS. Starfield + the engine-room name-dropping.
// ---------------------------------------------------------------------------
static void screenStars(uint32_t ms) {
  Serial.printf("[demo] screenStars t=%lu\n", millis());
  modeFast4();
  epd.clear();
  // Black stars on WHITE space — trail physics: a re-driven pixel lands
  // deeper than a once-driven one (fresh-response), so dark grounds show
  // trails that no amount of pre-saturation cures (the extra depth
  // relaxes over seconds and the contrast re-emerges). White is the one
  // state that cannot show this: overdriven white saturates — you cannot
  // get whiter than white — so re-whitened trail pixels vanish into the
  // ground by physics, not by tuning. A couple of style flashes first.
  for (int i = 0; i < 2; i++) {
    memset(fb, 3, (size_t)W * H); epd.paint(fb);
    memset(fb, 0, (size_t)W * H); epd.paint(fb);
  }
  struct Star { int16_t x, y; uint16_t z; };
  static Star st[220];
  for (int i = 0; i < 220; i++)
    st[i] = { (int16_t)random(-500, 500), (int16_t)random(-500, 500),
              (uint16_t)random(64, 1024) };
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    memset(fb, 0, (size_t)W * H);          // white space: trails clamp away
    for (int i = 0; i < 220; i++) {
      st[i].z -= 14;
      if (st[i].z < 32) st[i].z = 1024;
      const int sx = W / 2 + st[i].x * 256 / st[i].z;
      const int sy = H / 2 + st[i].y * 256 / st[i].z;
      for (int dy = 0; dy < 10; dy++)      // big black 10x10 stars
        for (int dx = 0; dx < 10; dx++)
          if (sx + dx >= 0 && sx + dx < W && sy + dy >= 0 && sy + dy < H)
            fb[(size_t)(sy + dy) * W + sx + dx] = 3;
    }
    // caption band: white text on black (static, so it never re-drives)
    for (int y = 42; y < 180; y++)
      memset(fb + (size_t)y * W, 3, W);
    drawTextCentred("XTENSA SIMD ASSEMBLY", 58, 4, 0);
    drawTextCentred("LCD_CAM DMA + DOUBLE-BUFFERED ROWS", 118, 2, 0);
    drawTextCentred("518KB CANVAS COMPACTED EVERY FRAME", 148, 2, 0);
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
  Serial.printf("[demo] screenMega t=%lu\n", millis());
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
  Serial.printf("[demo] screenCredits t=%lu\n", millis());
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
  // white halo so the black title reads on the dark gradient
  for (int oy = -4; oy <= 4; oy += 4)
    for (int ox = -4; ox <= 4; ox += 4)
      if (ox || oy)
        drawText("EPD_PAINTER", (W - textWidth("EPD_PAINTER", 8)) / 2 + ox,
                 90 + oy, 8, 0);
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
