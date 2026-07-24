// ============================================================================
// EPD_Painter DASHBOARD — a NORMAL-quality 4-level car console.
//
// The static instrument faces — dial rings, ticks, numerals, KM/H / GEAR /
// RPM labels, the centre-circle outlines, the indicator dot — are painted
// ONCE as a protected NORMAL template (setTemplate): crisp, fully
// saturated, and untouchable. The needles, seven-segment readouts, turn
// signals, fuel gauge and trip meter animate in the template's white
// areas, with the direct grey-to-grey trains loaded so state changes
// (grey arrow -> black arrow) complete in one paint.
//
// YOU drive it: hold the BOOT button (GPIO 0) to accelerate — the revs
// climb, the box shifts up on the limiter; let go and it engine-brakes
// back down through the gears to a stop (hazards on).
// ============================================================================

// Choose your board (or leave all commented for auto-probe).
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include <math.h>
#include "EPD_Painter.h"
#include "EPD_Painter_presets.h"
#include "../megademo/font8x8_basic.h"
#include "../grey16_testcard/direct_trains_m5papers3.h"
#include "../grey16_testcard/direct_trains_lilygo_t5s3.h"

EPD_Painter epd(EPD_PAINTER_PRESET);

static const int PIN_THROTTLE = 0;   // the BOOT button

static uint8_t *fb;
static int W, H;

// ---- geometry -------------------------------------------------------------
static const int LCX = 215, RCX = 745, CY = 270;   // dial centres
static const int R_RING0 = 190, R_RING1 = 200;     // outer ring band
static const int R_TICK1 = 186, R_TICKMAJ = 168, R_TICKMIN = 177;
static const int R_NUM   = 148;                    // numeral centres
static const int R_CIRC  = 86;                     // centre circle
static const int R_HUB   = 30, R_NEEDLE = 136;     // needle span

// scale value -> screen angle (degrees, y-down: 0 = right, 90 = down).
// 270-degree sweep with the gap at the bottom: 135 (down-left) .. 405.
static inline float angOf(float v, float vmax) { return 135.0f + v * 270.0f / vmax; }

// ---- primitives -----------------------------------------------------------
static inline void px(int x, int y, uint8_t c) {
  if (x >= 0 && x < W && y >= 0 && y < H) fb[(size_t)y * W + x] = c;
}
static void rect(int x, int y, int w, int h, uint8_t c) {
  for (int yy = y; yy < y + h; yy++)
    for (int xx = x; xx < x + w; xx++) px(xx, yy, c);
}
static void frameRect(int x, int y, int w, int h, int t, uint8_t c) {
  rect(x, y, w, t, c); rect(x, y + h - t, w, t, c);
  rect(x, y, t, h, c); rect(x + w - t, y, t, h, c);
}
static void glyph(char ch, int x, int y, int sc, uint8_t col) {
  if (ch < 0) return;
  const uint8_t *g = (const uint8_t *)font8x8_basic[(int)ch];
  for (int gy = 0; gy < 8; gy++)
    for (int gx = 0; gx < 8; gx++)
      if (g[gy] & (1 << gx)) rect(x + gx * sc, y + gy * sc, sc, sc, col);
}
static void text(const char *s, int x, int y, int sc, uint8_t col) {
  for (; *s; s++, x += 8 * sc) glyph(*s, x, y, sc, col);
}
static void textCentred(const char *s, int cx, int y, int sc, uint8_t col) {
  text(s, cx - (int)strlen(s) * 8 * sc / 2, y, sc, col);
}
// radial bar: from r0 to r1 at angle deg, thick pixels wide
static void radial(int cx, int cy, float deg, int r0, int r1, int thick, uint8_t c) {
  const float a = deg * (float)M_PI / 180.0f;
  const float dx = cosf(a), dy = sinf(a);
  for (int r = r0; r <= r1; r++) {
    const int x = cx + (int)lroundf(dx * r), y = cy + (int)lroundf(dy * r);
    rect(x - thick / 2, y - thick / 2, thick, thick, c);
  }
}
static void disc(int cx, int cy, int r, uint8_t c) {
  for (int y = -r; y <= r; y++)
    for (int x = -r; x <= r; x++)
      if (x * x + y * y <= r * r) px(cx + x, cy + y, c);
}
static void ringBand(int cx, int cy, int r0, int r1, bool gap, uint8_t c) {
  for (int y = -r1; y <= r1; y++)
    for (int x = -r1; x <= r1; x++) {
      const int d2 = x * x + y * y;
      if (d2 < r0 * r0 || d2 > r1 * r1) continue;
      if (gap) {
        float ang = atan2f((float)y, (float)x) * 180.0f / (float)M_PI;
        if (ang < 0) ang += 360.0f;
        if (ang > 48.0f && ang < 132.0f) continue;   // open at the bottom
      }
      px(cx + x, cy + y, c);
    }
}

// ---- seven-segment digits -------------------------------------------------
//   a         segment bits: gfedcba
//  f b
//   g
//  e c
//   d
static const uint8_t SEG[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66,
                                 0x6D, 0x7D, 0x07, 0x7F, 0x6F };
static void seg7(int x, int y, int w, int h, int t, int d, uint8_t col) {
  const uint8_t s = SEG[d % 10];
  const int m = y + h / 2;
  if (s & 0x01) rect(x + t, y, w - 2 * t, t, col);              // a
  if (s & 0x02) rect(x + w - t, y + t, t, h / 2 - t, col);      // b
  if (s & 0x04) rect(x + w - t, m, t, h / 2 - t, col);          // c
  if (s & 0x08) rect(x + t, y + h - t, w - 2 * t, t, col);      // d
  if (s & 0x10) rect(x, m, t, h / 2 - t, col);                  // e
  if (s & 0x20) rect(x, y + t, t, h / 2 - t, col);              // f
  if (s & 0x40) rect(x + t, m - t / 2, w - 2 * t, t, col);      // g
}

// ---- the template: everything printed on the instrument face --------------
static void buildFace() {
  memset(fb, 0, (size_t)W * H);

  for (int side = 0; side < 2; side++) {
    const int cx = side ? RCX : LCX;
    ringBand(cx, CY, R_RING0, R_RING1, true, 3);       // outer ring
    ringBand(cx, CY, R_CIRC - 3, R_CIRC, false, 3);    // centre circle
  }
  // speedo scale: 0..240, minors every 10, majors + numerals every 20
  for (int v = 0; v <= 240; v += 10) {
    const float a = angOf((float)v, 240);
    if (v % 20 == 0) {
      radial(LCX, CY, a, R_TICKMAJ, R_TICK1, 4, 3);
      if (v) {
        char buf[4]; snprintf(buf, sizeof buf, "%d", v);
        const float rad = a * (float)M_PI / 180.0f;
        textCentred(buf, LCX + (int)(cosf(rad) * R_NUM),
                    CY + (int)(sinf(rad) * R_NUM) - 8, 2, 3);
      }
    } else {
      radial(LCX, CY, a, R_TICKMIN, R_TICK1, 2, 3);
    }
  }
  // tacho scale: 0..9 x1000 rpm, minors every 500
  for (int v = 0; v <= 90; v += 5) {
    const float a = angOf((float)v, 90);
    if (v % 10 == 0) {
      radial(RCX, CY, a, R_TICKMAJ, R_TICK1, 4, 3);
      if (v) {
        char buf[3]; snprintf(buf, sizeof buf, "%d", v / 10);
        const float rad = a * (float)M_PI / 180.0f;
        textCentred(buf, RCX + (int)(cosf(rad) * R_NUM),
                    CY + (int)(sinf(rad) * R_NUM) - 12, 3, 3);
      }
    } else {
      radial(RCX, CY, a, R_TICKMIN, R_TICK1, 2, 3);
    }
  }
  textCentred("KM/H", LCX, CY + 34, 2, 3);
  textCentred("GEAR", RCX, CY + 34, 2, 3);
  textCentred("RPM",   RCX, CY + 116, 2, 3);
  textCentred("X1000", RCX, CY + 136, 2, 3);
  disc(915, 45, 11, 3);                                // top-right indicator
}

// ---- the animated layer ---------------------------------------------------
// sim state: the throttle is the BOOT button.
static float spd = 0, rpm = 800, trip = 22.1f;
static int   gear = 1, fuel = 4;
static bool  throttle = false;
static uint32_t frames = 0;
static const float RPM_PER_KMH[6] = { 110, 75, 54, 41, 33, 27 };

static void simStep() {
  throttle = (digitalRead(PIN_THROTTLE) == LOW);
  if (throttle) {
    spd += (gear < 3) ? 2.6f : 1.6f;           // foot down
    if (spd > 240) spd = 240;
  } else {
    spd -= (spd > 60) ? 2.2f : 1.2f;           // engine braking
    if (spd < 0) spd = 0;
  }
  rpm = (spd < 1) ? 800 : spd * RPM_PER_KMH[gear - 1];
  if (rpm > 6400 && gear < 6) gear++;
  if (rpm < 1700 && gear > 1) gear--;
  rpm = (spd < 1) ? 800 : spd * RPM_PER_KMH[gear - 1];
  trip += spd / 25000.0f;
  if (frames % 900 == 899 && fuel > 1) fuel--;   // slow burn
  frames++;
}

static void drawArrow(int cx, int y, bool left, uint8_t col) {
  for (int i = 0; i < 22; i++) {               // head
    const int xx = left ? cx - 22 + i : cx + 22 - i;
    rect(xx, y + 22 - i, 1, 2 * i, col);
  }
  rect(left ? cx : cx - 18, y + 12, 18, 20, col);   // tail
}

static void renderFrame() {
  memset(fb, 0, (size_t)W * H);

  // needles (the template's hub disc and scale are out of reach)
  radial(LCX, CY, angOf(spd, 240), R_HUB, R_NEEDLE, 7, 3);
  radial(RCX, CY, angOf(rpm / 100.0f, 90), R_HUB, R_NEEDLE, 7, 3);
  disc(LCX, CY, R_HUB - 4, 3);
  disc(RCX, CY, R_HUB - 4, 3);

  // digital speed, right-aligned three digits
  const int v = (int)(spd + 0.5f);
  const int dx0 = LCX - 62;
  if (v > 99) seg7(dx0,      CY - 78, 34, 62, 8, v / 100, 3);
  if (v > 9)  seg7(dx0 + 44, CY - 78, 34, 62, 8, (v / 10) % 10, 3);
  seg7(dx0 + 88, CY - 78, 34, 62, 8, v % 10, 3);

  // gear digit
  seg7(RCX - 20, CY - 82, 40, 68, 9, gear, 3);

  // ---- centre stack ----
  // turn signals: hazards while stopped, right blinker under power
  const bool blink = (frames >> 2) & 1;
  const bool stopped = spd < 1 && !throttle;
  const uint8_t lcol = (stopped && blink) ? 3 : 1;
  const uint8_t rcol = ((stopped || throttle) && blink) ? 3 : 1;
  drawArrow(447, 38, true, lcol);
  drawArrow(513, 38, false, rcol);

  // fuel gauge: black bar, lit blocks solid white, spent blocks outlined
  rect(425, 110, 110, 30, 3);
  for (int i = 0; i < 6; i++) {
    const int bx = 432 + i * 17;
    if (i < fuel) rect(bx, 117, 13, 16, 0);
    else          frameRect(bx, 117, 13, 16, 2, 0);
  }
  // fuel pump icon in its own box
  rect(425, 150, 110, 100, 3);
  rect(455, 165, 34, 60, 0);          // body
  rect(461, 172, 22, 18, 3);          // window
  rect(449, 228, 60, 8, 0);           // base
  rect(493, 170, 8, 40, 0);           // hose
  rect(493, 165, 16, 8, 0);           // nozzle
  // trip meter: white seven-seg on black
  rect(425, 330, 110, 74, 3);
  const int t10 = (int)(trip * 10.0f + 0.5f);
  seg7(433, 344, 24, 46, 6, (t10 / 100) % 10, 0);
  seg7(463, 344, 24, 46, 6, (t10 / 10) % 10, 0);
  rect(493, 384, 6, 6, 0);            // decimal point
  seg7(505, 344, 24, 46, 6, t10 % 10, 0);
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  if (!epd.begin()) {
    Serial.println("EPD init failed");
    while (1) delay(1000);
  }
  epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
  // Direct grey-to-grey trains: arrow state changes (grey <-> black)
  // complete in one paint instead of blinking through white.
  epd.setDirectTransitions(true);
  if (epd.getConfig().pin_syspwr >= 0) loadDirectTrainsM5PaperS3(epd);
  else                                 loadDirectTrainsLilygo(epd);

  W = epd.getConfig().width;
  H = epd.getConfig().height;
  fb = (uint8_t *)heap_caps_malloc((size_t)W * H, MALLOC_CAP_SPIRAM);
  if (!fb) { Serial.println("fb alloc failed"); while (1) delay(1000); }

  pinMode(PIN_THROTTLE, INPUT_PULLUP);

  epd.clear();
  buildFace();
  epd.setTemplate(fb, EPD_Painter::Quality::QUALITY_NORMAL);
  Serial.println("[dash] face template up — hold BOOT to accelerate");
}

void loop() {
  simStep();
  renderFrame();
  epd.paint(fb);
  if (frames % 20 == 0)
    Serial.printf("[dash] throttle=%d spd=%.0f rpm=%.0f gear=%d\n",
                  (int)throttle, spd, rpm, gear);
}
