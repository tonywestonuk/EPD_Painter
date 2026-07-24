// =============================================================================
//  Maze 3D  —  a first-person maze game for EPD_Painter + Adafruit GFX
//
//  A randomly generated maze, seen corridor-by-corridor in one-point
//  perspective (3D Monster Maze school, not a Doom clone): you turn and
//  step one cell at a time, and every move is one crisp line-art render,
//  which is exactly the workload the delta engine likes best.
//
//  The goal: three locked doors bar the way to a stone exit door.  Three
//  brass keys open them; three stone tablets each bear one digit of the
//  exit door's code.  Everything is placed so the maze is always solvable
//  (each key lies in the region its door closes off).  Fog-of-war automap
//  top-right.  Escape, and a new maze is one tap away.
//
//  Controls (touch, GT911 — https://github.com/tonywestonuk/gt911-arduino):
//    left / right third of the screen ... turn
//    centre ............................ step forward (or unlock / use door)
//    centre, near the bottom ........... step back
//    keypad / win screen ............... tap what you see
//  Serial fallback: a/d turn, w forward, s back, digits on the keypad,
//  x cancel, n new maze.
//
//  All game logic and rendering live in maze_core.h, templated on the GFX
//  type — it also compiles on a desktop with a stub canvas for previewing.
// =============================================================================

// Choose your hardware (or leave all commented for auto-detect):
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include <Wire.h>
#include "EPD_Painter_Adafruit.h"
#include "EPD_Painter_presets.h"
#include <gt911_lite.h>
#include "esp_system.h"
#include "maze_core.h"

// The game is 1-bit-and-greys line art; FAST paints a move in ~50ms and
// suits the pace. Every DEGHOST_MOVES renders the panel gets a full clear
// to sweep ghosting (it re-renders straight after, so it's just a blink).
static const EPD_Painter::Quality PLAY_QUALITY = EPD_Painter::Quality::QUALITY_NORMAL;
#define DEGHOST_MOVES 20

static EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);
static GT911_Lite touch;
static MzGame game;
static uint16_t renders_since_clear = 0;

// ---- touch (same pattern as the other examples: the display driver owns
// the I2C bus; orientation comes from the controller's config registers) ----
static uint16_t tp_xmax = MZ_VW, tp_ymax = MZ_VH;
static bool     tp_swap = false, touch_ok = false;

static void touchInit() {
  TwoWire *bus = epd.getConfig().i2c.wire;
  if (!bus) return;
  touch.begin(bus);
  for (uint8_t addr : { (uint8_t)0x5D, (uint8_t)0x14 }) {
    bus->beginTransmission(addr);
    bus->write(0x81); bus->write(0x46);
    if (bus->endTransmission(false) != 0) continue;
    if (bus->requestFrom(addr, (uint8_t)4) != 4) continue;
    uint16_t xm = bus->read(); xm |= bus->read() << 8;
    uint16_t ym = bus->read(); ym |= bus->read() << 8;
    if (xm == 0 || ym == 0 || xm == 0xFFFF) break;
    tp_xmax = xm; tp_ymax = ym;
    tp_swap = (xm < ym);
    touch_ok = true;
    Serial.printf("[maze] touch 0x%02x, range %ux%u%s\n", addr, xm, ym,
                  tp_swap ? " (swapped)" : "");
    return;
  }
  Serial.println("[maze] no touch controller — serial controls only");
}

static bool touchTapped(int &px, int &py) {
  if (!touch_ok) return false;
  static bool was_down = false;
  touch.read();
  bool down = touch.isTouched;
  bool tap = down && !was_down;
  was_down = down;
  if (!tap) return false;
  uint16_t rx = touch.x, ry = touch.y;
  if (tp_swap) {
    px = (int)ry * MZ_VW / tp_ymax;
    py = (int)(tp_xmax - rx) * MZ_VH / tp_xmax;
  } else {
    px = (int)rx * MZ_VW / tp_xmax;
    py = (int)ry * MZ_VH / tp_ymax;
  }
  return true;
}

// ---- render + paint ---------------------------------------------------------
static void present() {
  //if (++renders_since_clear >= DEGHOST_MOVES) {
  //  renders_since_clear = 0;
  //  epd.clear();
  //}
  mzRender(epd, game);
  epd.paint();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  if (!epd.begin()) {
    Serial.println("[maze] EPD init failed");
    for (;;) delay(1000);
  }
  epd.setQuality(PLAY_QUALITY);
  touchInit();
  epd.clear();

  mzNewGame(game, esp_random());
  Serial.println("[maze] a/d turn, w forward, s back; find the keys!");
  present();
}

void loop() {
  int act = ACT_NONE;

  int px, py;
  if (touchTapped(px, py))
    act = mzTouchAction(game, px, py);

  if (act == ACT_NONE && Serial.available()) {
    int c = Serial.read();
    switch (c) {
      case 'a': act = ACT_TURN_L; break;
      case 'd': act = ACT_TURN_R; break;
      case 'w': act = ACT_FWD;    break;
      case 's': act = ACT_BACK;   break;
      case 'x': act = ACT_KP_CANCEL; break;
      case 'n': act = ACT_NEW_GAME;  break;
      default:
        if (c >= '0' && c <= '9' && game.state == MZ_KEYPAD)
          act = ACT_DIGIT_0 + (c - '0');
    }
  }

  if (act != ACT_NONE && mzAction(game, act)) {
    if (game.state == MZ_WON) renders_since_clear = DEGHOST_MOVES;  // clean canvas for the banner
    present();
  }
  delay(25);
}
