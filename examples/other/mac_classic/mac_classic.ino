// =============================================================================
//  Mac Classic  —  Macintosh 128K/Plus emulator on e-paper, for EPD_Painter
//
//  Runs Matt Evans's umac emulator (Musashi 68000 core) with the panel as
//  the Mac's screen.  By default the Mac runs at a native 960x540 — umac
//  patches the ROM's video parameters, and Mac OS happily drives the big
//  screen — filling the panel 1:1.  (Set 512x342 in src/umac/umac_cfg.h
//  for the authentic size, centred with a bezel.)  The Mac desktop is an
//  ideal workload for the delta-update engine: mostly static, so only the
//  pixels that change get driven and the UI feels snappy.
//
//  SD card layout (see README.md for where to get these):
//      /mac/rom.bin    Mac Plus v3 ROM, 128KB, checksum 4D1F8172
//      /mac/disk.img   raw disk image (System 3.2 for a 128K Mac; the
//                      default 4MB machine runs System 6 / 7 happily)
//
//  Input:
//    - Touch (GT911) is the mouse: the cursor jumps to your finger and
//      the button presses.  Tap twice for a double-click, hold and move
//      to drag.  Requires Tony Weston's "GT911 Lite" library:
//      https://github.com/tonywestonuk/gt911-arduino
//    - Serial is the keyboard: characters you type are pressed on the Mac.
//      Ctrl-<letter> sends Command-<letter> (Ctrl-S = Cmd-S etc), so use a
//      real terminal (screen/minicom) rather than a line-based monitor for
//      shortcuts.  ESC toggles touch debug output.
//
//  The emulated Mac's disk writes go straight back to /mac/disk.img, so it
//  behaves like a real machine (keep a backup of the image).
// =============================================================================

// Choose your hardware (or leave all commented for auto-detect):
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <EPD_Painter.h>
#include <SD_MMC.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <gt911_lite.h>
#include "esp_heap_caps.h"

extern "C" {
#include "src/umac/umac.h"
#include "src/umac/rom.h"
#include "src/umac/keymap.h"
}

static const char *ROM_PATH  = "/mac/rom.bin";
static const char *DISK_PATH = "/mac/disk.img";

// The Mac screen is pure black-and-white so the calibrated greys of
// QUALITY_NORMAL buy nothing here — FAST gives the snappiest desktop.
static const EPD_Painter::Quality UI_QUALITY = EPD_Painter::Quality::QUALITY_NORMAL;

// How often the panel is allowed to repaint.  Shorter feels livelier but
// steals more panel bandwidth from big updates like window drags.
#define PAINT_INTERVAL_MS   80

// Ghost-busting: every this-many ms, if the screen has changed since the
// last clear, do a full panel clear and repaint.  0 disables.
#define CLEAR_INTERVAL_MS   30000

// Touch mapping tweaks, applied after the automatic orientation detection.
// If the cursor mirrors your finger, flip the offending axis.
#define TOUCH_FLIP_X  0
#define TOUCH_FLIP_Y  0

static EPD_Painter display(EPD_PAINTER_PRESET);
static GT911_Lite  touch;

// -----------------------------------------------------------------------------
// Geometry: the Mac's 1bpp framebuffer centred in the 960x540 2bpp panel.
// The Mac's resolution is DISP_WIDTH x DISP_HEIGHT from src/umac/umac_cfg.h,
// drawn at MAC_SCALE.  The default 480x270 at scale 2 fills the panel with
// 2x2 pixels — readable and touchable.  Vertical doubling is free: the
// panel buffer holds one row per Mac row and paintPacked's line_repeat
// drives each onto two panel rows, so only width is expanded here.
// -----------------------------------------------------------------------------
#define MAC_SCALE    2                        // 1 = native pixels, 2 = 2x2
#define MAC_W        DISP_WIDTH
#define MAC_H        DISP_HEIGHT
#define MAC_FB_BYTES (MAC_W * MAC_H / 8)
#define PANEL_W      960
#define PANEL_H      540
#define PANEL_ROW_B  (PANEL_W / 4)            // 2bpp packed
#define BUF_ROWS     (PANEL_H / MAC_SCALE)    // panel_buf height (pre-repeat)
#define MAC_X0       ((PANEL_W - MAC_W * MAC_SCALE) / 2)   // panel px
#define MAC_Y0       ((PANEL_H - MAC_H * MAC_SCALE) / 2)   // panel px

static uint8_t *mac_ram;                      // emulated RAM (PSRAM)
static uint8_t *mac_rom;                      // patched ROM  (PSRAM)
static uint8_t *panel_buf;                    // full-panel packed frame
static uint8_t *fb_shadow;                    // last-painted Mac fb, for change detection

// 1bpp -> 2bpp expansion: one source byte (8 px, MSB leftmost, 1=black)
// becomes two packed bytes (4 px each, first pixel in the MSBs, 3=black),
// or four packed bytes when each pixel is doubled horizontally.
static uint8_t lut_hi[256], lut_lo[256];
static uint8_t lut_2x[256][4];

static File disk;
static bool disk_readonly = false;

static void die(const char *msg) {
  Serial.printf("[mac] FATAL: %s\n", msg);
  for (;;) delay(1000);
}

static uint8_t *allocBuf(uint32_t size, uint32_t caps, const char *what) {
  uint8_t *p = (uint8_t *)heap_caps_malloc(size, caps);
  if (!p) {
    Serial.printf("[mac] %s: %lu bytes\n", what, (unsigned long)size);
    die("allocation failed");
  }
  return p;
}

// -----------------------------------------------------------------------------
// SD wiring — same per-board table as the bad_apple example
// -----------------------------------------------------------------------------
struct SdPins {
  int  cs, sclk, mosi, miso;
  bool sdmmc;
  int  other_cs;
};
static bool sdPinsForBoard(SdPins &p);   // explicit: keeps the Arduino
                                         // prototype generator at bay

static bool sdPinsForBoard(SdPins &p) {
  const auto &cfg = display.getConfig();
  if (cfg.i2c.sda == 39 && cfg.i2c.scl == 40) {        // LilyGo T5 S3 GPS / Pro
    p = { 12, 14, 13, 21, false, 46 };                 // SPI; deselect LoRa (46)
    return true;
  }
  if (cfg.i2c.sda == 41 && cfg.i2c.scl == 42) {        // M5PaperS3
    p = { 47, 39, 38, 40, true, -1 };
    return true;
  }
  return false;
}

static File sdOpen(const char *path, const char *mode) {
  const auto &cfg = display.getConfig();
  bool sdmmc = (cfg.i2c.sda == 41);
  return sdmmc ? SD_MMC.open(path, mode) : SD.open(path, mode);
}

// -----------------------------------------------------------------------------
// Disc: the emulated Mac's block device, backed directly by the SD file.
// umac calls these from disc driver traps; offsets/lengths are 512-aligned.
// -----------------------------------------------------------------------------
static int disk_read(void *ctx, uint8_t *data, unsigned int offset, unsigned int len) {
  (void)ctx;
  if (!disk.seek(offset)) return -1;
  return disk.read(data, len) == (int)len ? 0 : -1;
}

static int disk_write(void *ctx, uint8_t *data, unsigned int offset, unsigned int len) {
  (void)ctx;
  if (!disk.seek(offset)) return -1;
  if (disk.write(data, len) != len) return -1;
  disk.flush();
  return 0;
}

// -----------------------------------------------------------------------------
// Display: convert the Mac's 1bpp framebuffer into the panel frame and paint.
// Runs as its own task on core 0 so paintPacked()'s blocking never stalls
// the 68000 (which owns core 1).
// -----------------------------------------------------------------------------
static void buildLut() {
  for (int v = 0; v < 256; v++) {
    uint8_t hi = 0, lo = 0;
    for (int b = 0; b < 4; b++) {
      if (v & (0x80 >> b))     hi |= 0xC0 >> (b * 2);
      if (v & (0x08 >> b))     lo |= 0xC0 >> (b * 2);
    }
    lut_hi[v] = hi;
    lut_lo[v] = lo;
    // Doubled: each 2-bit pair of the source becomes one packed byte
    // (2 Mac px -> 4 panel px).
    for (int p = 0; p < 4; p++) {
      int pair = (v >> (6 - p * 2)) & 3;
      lut_2x[v][p] = (uint8_t)(((pair & 2) ? 0xF0 : 0) | ((pair & 1) ? 0x0F : 0));
    }
  }
}

static void drawBezel() {
  // A black frame around the Mac screen area, in panel-buffer coordinates
  // (rows are pre-line_repeat).  Full-panel configurations have no margin
  // and get no frame.
  const int sy = MAC_Y0 / MAC_SCALE;             // buffer-row margin
  if (MAC_X0 < 8 || sy < 4) return;
  const int bx0 = (MAC_X0 - 8) / 4, bx1 = (MAC_X0 + MAC_W * MAC_SCALE) / 4 + 1;
  for (int y = sy - 4; y < sy + MAC_H + 4; y++) {
    uint8_t *row = panel_buf + y * PANEL_ROW_B;
    if (y < sy || y >= sy + MAC_H) {
      memset(row + bx0, 0xFF, bx1 - bx0 + 1);
    } else {
      row[bx0] = 0xFF; row[bx0 + 1] = 0xFF;
      row[bx1 - 1] = 0xFF; row[bx1] = 0xFF;
    }
  }
}

static void convertFb(const uint8_t *fb) {
  for (int y = 0; y < MAC_H; y++) {
    const uint8_t *src = fb + y * (MAC_W / 8);
    uint8_t *dst = panel_buf + (MAC_Y0 / MAC_SCALE + y) * PANEL_ROW_B + MAC_X0 / 4;
    for (int i = 0; i < MAC_W / 8; i++) {
#if MAC_SCALE == 2
      const uint8_t *q = lut_2x[src[i]];
      *dst++ = q[0]; *dst++ = q[1]; *dst++ = q[2]; *dst++ = q[3];
#else
      *dst++ = lut_hi[src[i]];
      *dst++ = lut_lo[src[i]];
#endif
    }
  }
}

static volatile uint32_t stat_paints = 0;

static void displayTask(void *arg) {
  (void)arg;
  const uint8_t *fb = mac_ram + umac_get_fb_offset();
  uint32_t last_clear_ms = millis();
  bool     dirty_since_clear = false;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(PAINT_INTERVAL_MS));
    if (memcmp(fb_shadow, fb, MAC_FB_BYTES) != 0) {
      memcpy(fb_shadow, fb, MAC_FB_BYTES);
      convertFb(fb_shadow);
      display.paintPacked(panel_buf, MAC_SCALE);
      stat_paints++;
      dirty_since_clear = true;
    }
#if CLEAR_INTERVAL_MS
    // Periodic deghost: only worth the flash if the panel has actually
    // been driven since the last one — a static desktop stays quiet.
    if (millis() - last_clear_ms >= CLEAR_INTERVAL_MS) {
      if (dirty_since_clear) {
        display.clear();
        memcpy(fb_shadow, fb, MAC_FB_BYTES);
        convertFb(fb_shadow);
        display.paintPacked(panel_buf, MAC_SCALE);
        dirty_since_clear = false;
      }
      last_clear_ms = millis();
    }
#endif
  }
}

// -----------------------------------------------------------------------------
// Touch -> mouse.  umac's own mouse path is quadrature-accurate but tops
// out around 200px/s — fine for a real mouse, glacial for a finger.  So
// the cursor is warped directly by writing the Mac's low-memory globals
// (MTemp $828, RawMouse $82C, then CrsrNew $8CE = CrsrCouple $8CF), the
// same absolute-pointer technique Basilisk II uses.  The button still
// goes through umac (it's a VIA line); it presses one tick after the
// warp so the click lands where you touched.
// -----------------------------------------------------------------------------
static uint16_t tp_xmax = PANEL_W, tp_ymax = PANEL_H;
static bool     tp_swap = false;
static bool     touch_ok = false, touch_dbg = false;

static void touchInit() {
  // The display driver runs its own TwoWire instance on the board's I2C
  // pins (the global Wire is never begun) — the touch controller lives
  // on that same bus.
  TwoWire *bus = display.getConfig().i2c.wire;
  if (!bus) {
    Serial.println("[mac] no I2C bus — serial keyboard only");
    return;
  }
  touch.begin(bus);
  // Read the controller's configured output range (regs 0x8146-0x8149) to
  // learn its orientation; probe both possible I2C addresses.
  for (uint8_t addr : { (uint8_t)0x5D, (uint8_t)0x14 }) {
    bus->beginTransmission(addr);
    bus->write(0x81); bus->write(0x46);
    if (bus->endTransmission(false) != 0) continue;
    if (bus->requestFrom(addr, (uint8_t)4) != 4) continue;
    uint16_t xm = bus->read(); xm |= bus->read() << 8;
    uint16_t ym = bus->read(); ym |= bus->read() << 8;
    if (xm == 0 || ym == 0 || xm == 0xFFFF) break;
    tp_xmax = xm; tp_ymax = ym;
    tp_swap = (xm < ym);            // controller mounted portrait
    touch_ok = true;
    Serial.printf("[mac] touch 0x%02x, range %ux%u%s\n", addr, xm, ym,
                  tp_swap ? " (swapped)" : "");
    return;
  }
  Serial.println("[mac] no touch controller found — serial keyboard only");
}

static void mapTouch(uint16_t rx, uint16_t ry, int &px, int &py) {
  if (tp_swap) {
    px = (int)ry * PANEL_W / tp_ymax;
    py = (int)(tp_xmax - rx) * PANEL_H / tp_xmax;
  } else {
    px = (int)rx * PANEL_W / tp_xmax;
    py = (int)ry * PANEL_H / tp_ymax;
  }
#if TOUCH_FLIP_X
  px = PANEL_W - 1 - px;
#endif
#if TOUCH_FLIP_Y
  py = PANEL_H - 1 - py;
#endif
}

static void macWr16(uint32_t addr, uint16_t v) {
  mac_ram[addr]     = v >> 8;
  mac_ram[addr + 1] = v & 0xFF;
}

static void macWarpCursor(int x, int y) {
  macWr16(0x828, y);  macWr16(0x82A, x);   // MTemp    (v, h)
  macWr16(0x82C, y);  macWr16(0x82E, x);   // RawMouse (v, h)
  mac_ram[0x8CE] = mac_ram[0x8CF];         // CrsrNew = CrsrCouple
}

static bool t_active = false;

static void touchTick() {
  if (!touch_ok) return;
  touch.read();
  if (touch.isTouched) {
    int px, py;
    mapTouch(touch.x, touch.y, px, py);
    int tx = constrain((px - MAC_X0) / MAC_SCALE, 0, MAC_W - 1);
    int ty = constrain((py - MAC_Y0) / MAC_SCALE, 0, MAC_H - 1);
    if (touch_dbg)
      Serial.printf("[touch] raw %u,%u -> panel %d,%d -> mac %d,%d\n",
                    touch.x, touch.y, px, py, tx, ty);
    macWarpCursor(tx, ty);
    umac_mouse(0, 0, t_active ? 1 : 0);      // press one tick after the warp
    t_active = true;
  } else if (t_active) {
    umac_mouse(0, 0, 0);
    t_active = false;
  }
}

// -----------------------------------------------------------------------------
// Serial -> keyboard.  umac holds a single pending key event, and real Mac
// keyboards were polled, so events are queued here and released with a gap.
// -----------------------------------------------------------------------------
struct KeyEvt { uint8_t mkc; uint8_t down; };
static KeyEvt   kq[128];
static uint8_t  kq_head = 0, kq_tail = 0;
static uint32_t kq_last_ms = 0;

static void kqPush(uint8_t mkc, uint8_t down) {
  uint8_t next = (kq_head + 1) % 128;
  if (next == kq_tail) return;                // full: drop
  kq[kq_head] = { mkc, down };
  kq_head = next;
}

static void kqPushStroke(uint8_t mkc, uint8_t modifier) {
  if (modifier) kqPush(modifier, 1);
  kqPush(mkc, 1);
  kqPush(mkc, 0);
  if (modifier) kqPush(modifier, 0);
}

// Ascii -> Mac keycode.  0xFF = no key.  Alphabetical MKC order for a-z.
static const uint8_t mkc_alpha[26] = {
  MKC_A, MKC_B, MKC_C, MKC_D, MKC_E, MKC_F, MKC_G, MKC_H, MKC_I,
  MKC_J, MKC_K, MKC_L, MKC_M, MKC_N, MKC_O, MKC_P, MKC_Q, MKC_R,
  MKC_S, MKC_T, MKC_U, MKC_V, MKC_W, MKC_X, MKC_Y, MKC_Z
};
static const uint8_t mkc_digit[10] = {
  MKC_0, MKC_1, MKC_2, MKC_3, MKC_4, MKC_5, MKC_6, MKC_7, MKC_8, MKC_9
};

static bool asciiToKey(char c, uint8_t &mkc, bool &shift) {
  shift = false;
  if (c >= 'a' && c <= 'z') { mkc = mkc_alpha[c - 'a']; return true; }
  if (c >= 'A' && c <= 'Z') { mkc = mkc_alpha[c - 'A']; shift = true; return true; }
  if (c >= '0' && c <= '9') { mkc = mkc_digit[c - '0']; return true; }
  switch (c) {
    case ' ':  mkc = MKC_Space;     return true;
    case '\r': case '\n': mkc = MKC_Return; return true;
    case '\t': mkc = MKC_Tab;       return true;
    case 0x08: case 0x7F: mkc = MKC_BackSpace; return true;
    case '-':  mkc = MKC_Minus;     return true;
    case '=':  mkc = MKC_Equal;     return true;
    case '[':  mkc = MKC_LeftBracket;  return true;
    case ']':  mkc = MKC_RightBracket; return true;
    case '\\': mkc = MKC_BackSlash; return true;
    case ';':  mkc = MKC_SemiColon; return true;
    case '\'': mkc = MKC_SingleQuote; return true;
    case ',':  mkc = MKC_Comma;     return true;
    case '.':  mkc = MKC_Period;    return true;
    case '/':  mkc = MKC_Slash;     return true;
    case '`':  mkc = MKC_Grave;     return true;
  }
  shift = true;
  switch (c) {
    case '!': mkc = MKC_1; return true;
    case '@': mkc = MKC_2; return true;
    case '#': mkc = MKC_3; return true;
    case '$': mkc = MKC_4; return true;
    case '%': mkc = MKC_5; return true;
    case '^': mkc = MKC_6; return true;
    case '&': mkc = MKC_7; return true;
    case '*': mkc = MKC_8; return true;
    case '(': mkc = MKC_9; return true;
    case ')': mkc = MKC_0; return true;
    case '_': mkc = MKC_Minus; return true;
    case '+': mkc = MKC_Equal; return true;
    case '{': mkc = MKC_LeftBracket;  return true;
    case '}': mkc = MKC_RightBracket; return true;
    case '|': mkc = MKC_BackSlash; return true;
    case ':': mkc = MKC_SemiColon; return true;
    case '"': mkc = MKC_SingleQuote; return true;
    case '<': mkc = MKC_Comma; return true;
    case '>': mkc = MKC_Period; return true;
    case '?': mkc = MKC_Slash; return true;
    case '~': mkc = MKC_Grave; return true;
  }
  return false;
}

static void serialTick() {
  while (Serial.available()) {
    int c = Serial.read();
    if (c == 0x1B) {                          // ESC: toggle touch debug
      touch_dbg = !touch_dbg;
      Serial.printf("[mac] touch debug %s\n", touch_dbg ? "on" : "off");
      continue;
    }
    if (c >= 0x01 && c <= 0x1A && c != '\r' && c != '\n' && c != '\t' && c != 0x08) {
      kqPushStroke(mkc_alpha[c - 1], MKC_Command);   // Ctrl-X -> Cmd-X
      continue;
    }
    uint8_t mkc; bool shift;
    if (asciiToKey((char)c, mkc, shift))
      kqPushStroke(mkc, shift ? MKC_Shift : 0);
  }
  // Release at most one queued event every 30ms of Mac time.
  if (kq_tail != kq_head && millis() - kq_last_ms >= 30) {
    kq_last_ms = millis();
    KeyEvt &e = kq[kq_tail];
    kq_tail = (kq_tail + 1) % 128;
    umac_kbd_event((e.mkc << 1) | 1, e.down);
  }
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  if (!display.begin()) die("display.begin() failed");
  display.setQuality(UI_QUALITY);

  SdPins sd;
  if (!sdPinsForBoard(sd)) die("no SD pin map for this board — set pins manually");
  pinMode(sd.cs, OUTPUT);
  digitalWrite(sd.cs, HIGH);
  if (sd.other_cs >= 0) {
    pinMode(sd.other_cs, OUTPUT);
    digitalWrite(sd.other_cs, HIGH);
  }
  if (sd.sdmmc) {
    SD_MMC.setPins(sd.sclk, sd.mosi, sd.miso);
    if (!SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT))
      die("SD_MMC.begin() failed — card inserted?");
  } else {
    SPI.begin(sd.sclk, sd.miso, sd.mosi, sd.cs);
    if (!SD.begin(sd.cs, SPI, 25000000)) die("SD.begin() failed — card inserted?");
  }

  // ---- ROM ----
  File romf = sdOpen(ROM_PATH, FILE_READ);
  if (!romf) die("could not open /mac/rom.bin");
  if (romf.size() != ROM_SIZE) die("rom.bin must be exactly 128KB (Mac Plus v3)");
  mac_rom = allocBuf(ROM_SIZE, MALLOC_CAP_SPIRAM, "ROM");
  if (romf.read(mac_rom, ROM_SIZE) != ROM_SIZE) die("ROM read failed");
  romf.close();
  if (rom_patch(mac_rom))
    die("unsupported ROM — need the Mac Plus v3 ROM, checksum 4D1F8172");

  // ---- Disk ----
  disk = sdOpen(DISK_PATH, "r+");
  if (!disk) {
    disk = sdOpen(DISK_PATH, FILE_READ);
    disk_readonly = true;
  }
  if (!disk) die("could not open /mac/disk.img");
  Serial.printf("[mac] disk %lu KB%s\n", (unsigned long)(disk.size() / 1024),
                disk_readonly ? " (read-only)" : "");

  // ---- Emulated machine ----
  mac_ram   = allocBuf(RAM_SIZE, MALLOC_CAP_SPIRAM, "Mac RAM");
  panel_buf = allocBuf(BUF_ROWS * PANEL_ROW_B, MALLOC_CAP_SPIRAM, "panel buffer");
  fb_shadow = (uint8_t *)heap_caps_malloc(MAC_FB_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!fb_shadow)   // internal is faster for the change-detect memcmp, but not vital
    fb_shadow = allocBuf(MAC_FB_BYTES, MALLOC_CAP_SPIRAM, "fb shadow");
  memset(mac_ram, 0, RAM_SIZE);
  memset(panel_buf, 0x00, BUF_ROWS * PANEL_ROW_B);   // white
  memset(fb_shadow, 0xAA, MAC_FB_BYTES);            // != fb, forces first paint
  buildLut();
  drawBezel();

  disc_descr_t discs[DISC_NUM_DRIVES] = {};
  discs[0].base      = NULL;                        // no in-RAM image:
  discs[0].size      = disk.size();                 //  stream via callbacks
  discs[0].read_only = disk_readonly;
  discs[0].op_read   = disk_read;
  discs[0].op_write  = disk_write;

  umac_init(mac_ram, mac_rom, discs);
  touchInit();

  display.clear();

  Serial.printf("[mac] Macintosh %uKB, ROM ok, here we go\n", UMAC_MEMSIZE);
  Serial.println("[mac] touch = mouse, serial = keyboard, Ctrl-X = Cmd-X, ESC = touch debug");

  xTaskCreatePinnedToCore(displayTask, "mac_disp", 4000, NULL, 2, NULL, 0);
}

// -----------------------------------------------------------------------------
// Main loop (core 1): run the 68000 paced to real time, tick the 60Hz/1Hz
// events off the wall clock, and feed input.
// -----------------------------------------------------------------------------
void loop() {
  static uint32_t next_slice_us = micros();
  static uint32_t last_vsync_us = 0, last_1hz_ms = 0, last_stats_ms = 0;
  static uint32_t stat_slices = 0, stat_window_ms = 0;

  // 68000 time: umac_loop() executes 5ms of CPU.  Run when the wall clock
  // has caught up; if we're badly behind (SD stall etc), drop the debt.
  uint32_t now = micros();
  if ((int32_t)(now - next_slice_us) >= 0) {
    umac_loop();
    next_slice_us += 5000;
    stat_slices++;
    if ((int32_t)(now - next_slice_us) > 100000) next_slice_us = now;
  }

  now = micros();
  if (now - last_vsync_us >= 16667) {
    last_vsync_us = now;
    umac_vsync_event();
    touchTick();
    serialTick();
  }

  uint32_t now_ms = millis();
  if (now_ms - last_1hz_ms >= 1000) {
    last_1hz_ms = now_ms;
    umac_1hz_event();
  }

  if (now_ms - last_stats_ms >= 5000) {
    if (last_stats_ms) {
      uint32_t win = now_ms - last_stats_ms;
      Serial.printf("[mac] speed %lu%%, %lu paints\n",
                    (unsigned long)(stat_slices * 5 * 100 / win),
                    (unsigned long)stat_paints);
    }
    stat_slices = 0;
    stat_paints = 0;
    last_stats_ms = now_ms;
  }
}
