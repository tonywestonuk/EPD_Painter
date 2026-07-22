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
//   d  4-level direct grey-to-grey mode (seed trains)   D  transition card
//   j <from> <to> <13 codes 0-2>  upload a direct train
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
#include "tuned_trains_m5papers3.h"
#include "tuned_trains_h716.h"
#include "direct_trains_m5papers3.h"
#include "direct_trains_lilygo_t5s3.h"

EPD_Painter epd(EPD_PAINTER_PRESET);

static uint8_t *fb;
static int W, H;
static bool mode16 = true;   // '4'/'6' switch between 4-level and 16-grey

// Pick the scanner-tuned set for the board the AUTO probe found: the
// M5PaperS3 preset is the one with a power-latch pin (pin_syspwr).
static void loadBoardTrains() {
  if (epd.getConfig().shift.driver == EPD_Painter::Shift::H716 &&
      epd.getConfig().shift.data >= 0)   loadTunedTrainsH716(epd);
  else if (epd.getConfig().pin_syspwr >= 0) loadTunedTrainsM5PaperS3(epd);
  else                                      loadTunedTrains(epd);
}

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
  delay(300);
  // Rig diagnostics: why did we boot? (1 power-on, 3 SW, 4 panic,
  // 5/6/7/9 watchdogs, 15 brownout)
  Serial.printf("[grey16] boot, reset reason %d\n", (int)esp_reset_reason());

  // Calibration rig: never shutdown-on-reset — the flasher's hard reset
  // would power the board off after every upload (on the M5PaperS3 the
  // USB check can't see USB: it reads the LilyGo's BQ25896).
  epd.setAutoShutdown(false);

  if (!epd.begin()) {
    Serial.println("EPD init failed");
    while (1) delay(1000);
  }
  epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
  if (!epd.setGreyLevels(16)) {
    Serial.println("setGreyLevels(16) failed");
    while (1) delay(1000);
  }
  // Scanner-tuned trains for this board (LilyGo T5 S3 GPS or M5PaperS3).
  loadBoardTrains();

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
// ncol = palette size (levels spread evenly 0..maxlv); maxlv < 15 keeps
// the longest train in play short, so the pass loop truncates and the
// frame gets cheaper — the decision engine's cost scales with the content.
// In 4-level mode the palette is the engine's own 0..3.
struct Patch { int x, y, vx, vy; };
static Patch patches[16];
static uint8_t patchLv[16], patchBorder;
static const int PW = 110, PH = 110;

static void patchesInit(int ncol, int maxlv) {
  if (mode16) {
    for (int i = 0; i < 16; i++)
      patchLv[i] = (uint8_t)(((i % ncol) * maxlv) / (ncol - 1));
    patchBorder = (uint8_t)maxlv;
  } else {
    for (int i = 0; i < 16; i++) patchLv[i] = (uint8_t)(i % 4);
    patchBorder = 3;
  }
  for (int i = 0; i < 16; i++) {
    patches[i].x = (i % 4) * (W - PW) / 3;
    patches[i].y = (i / 4) * (H - PH) / 3;
    patches[i].vx = ((i % 5) + 4) * ((i & 1) ? -1 : 1);
    patches[i].vy = (((i * 3) % 5) + 4) * ((i & 2) ? -1 : 1);
  }
}

static void patchesRender() {
  for (int i = 0; i < 16; i++) {
    Patch &q = patches[i];
    q.x += q.vx; q.y += q.vy;
    if (q.x < 0)      { q.x = 0;      q.vx = -q.vx; }
    if (q.y < 0)      { q.y = 0;      q.vy = -q.vy; }
    if (q.x > W - PW) { q.x = W - PW; q.vx = -q.vx; }
    if (q.y > H - PH) { q.y = H - PH; q.vy = -q.vy; }
  }
  memset(fb, 0, (size_t)W * H);
  for (int i = 0; i < 16; i++) {
    for (int y = 0; y < PH; y++) {
      uint8_t *row = fb + (size_t)(patches[i].y + y) * W + patches[i].x;
      if (y < 3 || y >= PH - 3) memset(row, patchBorder, PW);
      else {
        row[0] = row[1] = row[2] = patchBorder;
        row[PW - 3] = row[PW - 2] = row[PW - 1] = patchBorder;
        memset(row + 3, patchLv[i], PW - 6);
      }
    }
  }
}

static void perfTest(int frames, int ncol, int maxlv) {
  patchesInit(ncol, maxlv);
  epd.clear();
  const uint32_t t0 = millis();
  for (int f = 0; f < frames; f++) {
    patchesRender();
    epd.paint(fb);
    if ((f + 1) % 20 == 0)
      Serial.printf("[grey16] perf %d/%d: %.2f fps\n", f + 1, frames,
                    (f + 1) * 1000.0f / (millis() - t0));
  }
  const uint32_t dt = millis() - t0;
  Serial.printf("[grey16] perf done (%s, %d colours, max level %d): "
                "%d frames, %lu ms, %.2f fps (%.0f ms/frame)\n",
                mode16 ? "16-grey" : "4-level", mode16 ? ncol : 4,
                mode16 ? maxlv : 3, frames, (unsigned long)dt,
                frames * 1000.0f / dt, (float)dt / frames);
}

// paintLater throughput. paintLater() frameskips — a submit loop faster
// than the panel just overwrites the pending buffer and only the latest
// render is painted — so the effective rate is the paint task's completed
// drive cycles (paintsCompleted()) over wall time, never the submit rate.
static void perfLater(int secs, int ncol, int maxlv) {
  patchesInit(ncol, maxlv);
  epd.clear();
  const uint32_t p0 = epd.paintsCompleted();
  const uint32_t t0 = millis();
  uint32_t submitted = 0;
  while (millis() - t0 < (uint32_t)secs * 1000) {
    patchesRender();
    epd.paintLater(fb);
    submitted++;
  }
  while (!epd.paintIdle()) delay(5);       // drain pending work + mop-up
  const uint32_t dt = millis() - t0;
  const uint32_t painted = epd.paintsCompleted() - p0;
  Serial.printf("[grey16] paintLater perf (%s, %d colours, max level %d): "
                "%lu submitted, %lu painted in %lu ms = %.2f painted fps "
                "(submit loop %.1f fps, %lu frames skipped)\n",
                mode16 ? "16-grey" : "4-level", mode16 ? ncol : 4,
                mode16 ? maxlv : 3, (unsigned long)submitted,
                (unsigned long)painted, (unsigned long)dt,
                painted * 1000.0f / dt, submitted * 1000.0f / dt,
                (unsigned long)(submitted - painted));
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

// ---- direct grey-to-grey transitions (4-level; DECISION_ENGINE.md) --------
// The transition card: 9 columns grouped by target level, reference first.
// Phase A paints every column's FROM level (references stay white), phase B
// repaints with the TO levels in ONE paint — transition columns go through
// the direct trains, references are plain applies from white. The scan
// compares each transition column's landing against its group's reference.
// An unloaded/failed pair shows as a white column (two-step fallback,
// deliberately not mopped up by a second paint).
static const uint8_t cardFrom[9] = { 0, 2, 3,   0, 1, 3,   0, 1, 2 };
static const uint8_t cardTo[9]   = { 1, 1, 1,   2, 2, 2,   3, 3, 3 };

static void cardFill(const uint8_t *lv) {
  for (int i = 0; i < 9; i++) {
    const int x0 = (i * W) / 9, x1 = ((i + 1) * W) / 9;
    for (int y = 0; y < H; y++)
      memset(fb + (size_t)y * W + x0, lv[i], x1 - x0);
  }
}

// Mixed-path DC ghost test: cycle the LEFT half through the grey-to-grey
// web — white>1>2>3>2>1>white then white>1>3>1>white — every direct train
// plus apply(1)/remove(1) fires each cycle, single paint per step. If the
// direct nets are honest (Q(to)-Q(from)) both path ledgers sum to zero and
// a uniform probe ('u') afterwards shows no midline step.
static void halfFill(uint8_t lv) {
  for (int y = 0; y < H; y++) {
    memset(fb + (size_t)y * W, lv, W / 2);
    memset(fb + (size_t)y * W + W / 2, 0, W - W / 2);
  }
}

static bool directOn = false;

static void directGhost(int cycles) {
  static const uint8_t seqA[] = { 1, 2, 3, 2, 1, 0 };
  static const uint8_t seqB[] = { 1, 3, 1, 0 };
  epd.clear();
  for (int n = 0; n < cycles; n++) {
    // Legacy control arm ('e' toggles the engine off): the same visual
    // sequence through the two-step — each step needs the second paint to
    // redraw its erased grey-to-grey pixels.
    for (unsigned k = 0; k < sizeof(seqA); k++) {
      halfFill(seqA[k]); epd.paint(fb);
      if (!directOn) epd.paint(fb);
    }
    for (unsigned k = 0; k < sizeof(seqB); k++) {
      halfFill(seqB[k]); epd.paint(fb);
      if (!directOn) epd.paint(fb);
    }
    Serial.printf("[direct] ghost cycle %d/%d\n", n + 1, cycles);
  }
  while (!epd.paintIdle()) delay(5);
}

static void transitionCard() {
  if (mode16) { Serial.println("[direct] 4-level mode required ('d' first)"); return; }
  epd.clear();
  cardFill(cardFrom);
  epd.paint(fb); epd.paint(fb);            // two-step allowed here: all applies
  while (!epd.paintIdle()) delay(5);
  delay(300);
  cardFill(cardTo);
  epd.paint(fb);                           // ONE paint — the direct engine's gate
  while (!epd.paintIdle()) delay(5);
  Serial.println("[direct] transition card done (cols: ref,2>1,3>1 | ref,1>2,3>2 | ref,1>3,2>3)");
}

void loop() {
  if (!Serial.available()) { delay(20); return; }
  switch (Serial.read()) {
    case 'z': epd.debugPinBench(); break;
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
      // x <frames> [ncol] [maxlv]   bouncing-patch performance test
      int frames = Serial.parseInt();
      int ncol   = Serial.parseInt();
      int maxlv  = Serial.parseInt();
      if (frames < 1) frames = 60;
      if (frames > 2000) frames = 2000;
      if (ncol < 2 || ncol > 16) ncol = 16;
      if (maxlv < 1 || maxlv > 15) maxlv = 15;
      perfTest(frames, ncol, maxlv);
    } break;
    case 'y': {
      // y <seconds> [ncol] [maxlv]   paintLater throughput test
      int secs  = Serial.parseInt();
      int ncol  = Serial.parseInt();
      int maxlv = Serial.parseInt();
      if (secs < 1) secs = 15;
      if (secs > 120) secs = 120;
      if (ncol < 2 || ncol > 16) ncol = 16;
      if (maxlv < 1 || maxlv > 15) maxlv = 15;
      perfLater(secs, ncol, maxlv);
    } break;
    case '4':
      if (epd.setGreyLevels(4)) { mode16 = false; Serial.println("[grey16] 4-level mode"); }
      else Serial.println("[grey16] setGreyLevels(4) failed");
      break;
    case '6':
      if (epd.setGreyLevels(16)) {
        mode16 = true;
        loadBoardTrains();
        Serial.println("[grey16] 16-grey mode");
      } else Serial.println("[grey16] setGreyLevels(16) failed");
      break;
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
      loadBoardTrains();
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
    case 'd':
      // direct grey-to-grey mode: 4-level + direct engine + seed trains
      if (!epd.setGreyLevels(4)) { Serial.println("[direct] setGreyLevels(4) failed"); break; }
      mode16 = false;
      if (!epd.setDirectTransitions(true)) { Serial.println("[direct] enable failed"); break; }
      directOn = true;
      {
        // Trains are per-board and per-quality: pick both.
        const bool m5 = epd.getConfig().pin_syspwr >= 0;
        const bool fast =
            epd.getConfig().quality == EPD_Painter::Quality::QUALITY_FAST;
        if (m5 && fast)       loadDirectTrainsM5PaperS3Fast(epd);
        else if (m5)          loadDirectTrainsM5PaperS3(epd);
        else if (fast)        loadDirectTrainsLilygoFast(epd);
        else                  loadDirectTrainsLilygo(epd);
        Serial.printf("[direct] 4-level direct mode, %s %s trains loaded\n",
                      m5 ? "M5PaperS3" : "LilyGo", fast ? "FAST" : "NORMAL");
      }
      break;
    case 'e':
      // A/B lever: toggle the direct engine (trains stay loaded)
      directOn = !directOn;
      epd.setDirectTransitions(directOn);
      Serial.printf("[direct] engine %s\n", directOn ? "ON" : "OFF (legacy two-step)");
      break;
    case 'D': transitionCard(); break;
    case 'G': {
      // G <cycles>   mixed-path direct ghost test (4-level direct mode)
      int cycles = Serial.parseInt();
      if (cycles < 1) cycles = 20;
      if (cycles > 200) cycles = 200;
      if (mode16) { Serial.println("[direct] 4-level mode required ('d' first)"); break; }
      directGhost(cycles);
      Serial.printf("[direct] ghost done: %d cycles, screen white\n", cycles);
    } break;
    case 'j': {
      // j <from> <to> <up to 26 codes 0-2, newline-terminated>
      // Trains longer than the quality's 13 passes extend the frame —
      // deep lightening transitions need erase + re-darken room.
      const int f = Serial.parseInt(), t = Serial.parseInt();
      uint8_t tr[26]; int got = 0;
      const unsigned long j0 = millis();
      while (got < 26 && millis() - j0 < 2000) {
        const int ch = Serial.read();
        if (ch < 0) { delay(2); continue; }
        if (ch >= '0' && ch <= '2') tr[got++] = (uint8_t)(ch - '0');
        else if (ch == '\n') break;
      }
      if (got >= 1 && f >= 1 && f <= 3 && t >= 1 && t <= 3 && f != t) {
        epd.setDirectTrain((uint8_t)f, (uint8_t)t, tr, got);
        Serial.printf("[direct] train %d->%d set (%d passes)\n", f, t, got);
      } else Serial.println("[direct] bad train line");
    } break;
    case 'H':
      epd.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
      Serial.println("[grey16] QUALITY_HIGH");
      break;
    case 'f':
      epd.setQuality(EPD_Painter::Quality::QUALITY_FAST);
      Serial.println("[grey16] QUALITY_FAST (4-level mode only)");
      break;
    case 'N':
      epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
      Serial.println("[grey16] QUALITY_NORMAL");
      break;
    default: break;
  }
}
