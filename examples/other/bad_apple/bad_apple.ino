// =============================================================================
//  Bad Apple  —  .epv video player for EPD_Painter / ESP32-S3
//
//  Streams pre-packed native frames from SD straight into paintPacked().
//  There is no pixel processing on the ESP32 at all: the .epv file (made
//  with video2epv.py in this folder) already holds frames in the driver's
//  packed 2bpp format — 4 pixels/byte, first pixel in the MSBs, 0=white
//  3=black — so each frame is one SD read and one paintPacked() call.
//
//  Videos live in a /videos folder in the root of the SD card.  One video
//  plays immediately; more than one brings up a picker on the panel —
//  tap a row to play it (GT911 touch, both supported boards have one),
//  or send its number over serial.  Tapping the screen during playback
//  returns to the picker.  A lone /badapple.epv in the SD root still
//  plays, for cards made for older versions of this example.
//
//  Line-doubled files (converter --line-double) store half-height frames;
//  paintPacked(buf, 2) drives each stored row onto two panel rows, halving
//  the SD bandwidth. With a 480-line source there is no resolution loss.
//
//  Frames are PackBits RLE by default (~10-20x smaller): SD bandwidth drops
//  to a few hundred KB/s and the access pattern is purely sequential. Every
//  frame is played — if the panel can't keep up, the video simply runs
//  slower and resumes normal tempo when it can.
//
//  The pipeline overlaps naturally: paintPacked() hands the frame to the
//  background paint task and returns, so the next frame's SD read happens
//  while the panel is still being driven.
//
//  Setup (full instructions in README.md): convert videos, copy them into
//  a "videos" folder on the SD card, pick your board below, flash.
//
//      python3 video2epv.py your_video.mp4 video/your_video.epv --fps 20
// =============================================================================

// Choose your hardware (or leave all commented for auto-detect):
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <EPD_Painter.h>
#include <SD_MMC.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <gt911_lite.h>          // https://github.com/tonywestonuk/gt911-arduino
#include "esp_heap_caps.h"
#include "font8x8_basic.h"       // public-domain IBM VGA glyphs, for the picker

static const char *VIDEO_DIR   = "/videos";
static const char *LEGACY_PATH = "/badapple.epv";   // pre-picker cards

// FAST keeps up with 15 fps; try QUALITY_NORMAL for calibrated greys if the
// frame rate holds up on your card.
static const EPD_Painter::Quality PLAY_QUALITY = EPD_Painter::Quality::QUALITY_FAST;


static EPD_Painter display(EPD_PAINTER_PRESET);
static GT911_Lite  touch;

// -----------------------------------------------------------------------------
// SD wiring — selected at runtime from the detected board (keyed off the I2C
// bus, which is unique per preset). Override here if yours differ.
//
// M5PaperS3 has a dedicated slot: native 1-bit SDMMC through the GPIO matrix
// (same three wires as SPI but the real SD protocol, ~2x the throughput).
// The LilyGo T5 S3 shares its SPI bus with the LoRa radio, so per LilyGo's
// own examples it must use SPI mode with every chip-select parked high.
// -----------------------------------------------------------------------------
struct SdPins {
  int  cs, sclk, mosi, miso;
  bool sdmmc;          // true: native 1-bit SD protocol, false: SPI mode
  int  other_cs;       // another device on a shared bus to deselect (-1: none)
};
static bool sdPinsForBoard(SdPins &p);   // explicit prototype for the builder

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

static bool use_sdmmc = false;

static File sdOpen(const char *path) {
  return use_sdmmc ? SD_MMC.open(path, FILE_READ) : SD.open(path, FILE_READ);
}

// -----------------------------------------------------------------------------
// .epv container
// -----------------------------------------------------------------------------
struct __attribute__((packed)) EpvHeader {
  char     magic[4];      // "EPV1"
  uint16_t width;
  uint16_t height;
  uint16_t fps;
  uint16_t flags;         // bit0: line-doubled, bit1: RLE
  uint32_t frame_count;
};

static File      video;
static EpvHeader hdr;
static int       line_repeat = 1;
static bool      rle         = false;
static uint32_t  frame_size  = 0;

static uint8_t  *frame_buf   = nullptr;   // full-panel sized; also the picker's canvas
static uint8_t  *comp_buf    = nullptr;   // RLE payload staging
static uint32_t  full_frame  = 0;         // panel W*H/4
static uint32_t  comp_cap    = 0;
static uint32_t  frame_idx   = 0;
static uint32_t  start_ms    = 0;

// video list
#define MAX_VIDEOS 8
struct VideoEntry {
  char     name[40];      // filename within /videos
  uint16_t fps;
  uint32_t seconds;
};
static VideoEntry vids[MAX_VIDEOS];
static int  n_videos  = 0;
static int  cur_video = -1;
static bool legacy    = false;    // playing /badapple.epv, no picker

// stats
static uint32_t  stat_frames = 0, stat_reads = 0, stat_read_us = 0,
                 stat_paint_us = 0, stat_bytes = 0, stat_ms = 0;

static void die(const char *msg) {
  Serial.printf("[bad_apple] FATAL: %s\n", msg);
  for (;;) delay(1000);
}

// Internal DMA-capable RAM lets the SD driver DMA straight into the buffer;
// a PSRAM destination forces slow bounce-buffered reads.
static uint8_t *allocBuf(uint32_t size, const char *what) {
  uint8_t *p = (uint8_t *)heap_caps_aligned_alloc(
      64, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  if (!p) {
    Serial.printf("[bad_apple] %s in PSRAM — internal RAM full, reads will be slower\n", what);
    p = (uint8_t *)heap_caps_aligned_alloc(16, size, MALLOC_CAP_SPIRAM);
  }
  return p;
}

// ---- frame records ----------------------------------------------------------
// Raw files: fixed-size frames back to back. RLE files: uint32 record header
// (bits 0-30 payload length, bit 31 = payload is an uncompressed frame), then
// the payload — PackBits: header byte h < 128 → copy h+1 literal bytes,
// h > 128 → repeat next byte 257-h times, h == 128 → no-op.

static bool readFrame() {
  uint32_t t0 = micros();
  uint32_t len = frame_size;
  bool ok;
  if (!rle) {
    ok = video.read(frame_buf, frame_size) == (int)frame_size;
  } else {
    uint32_t rec;
    if (video.read((uint8_t *)&rec, 4) != 4) return false;
    len = rec & 0x7FFFFFFFu;
    if (rec & 0x80000000u) {
      ok = (len == frame_size) && video.read(frame_buf, frame_size) == (int)frame_size;
    } else {
      if (len > comp_cap || video.read(comp_buf, len) != (int)len) return false;
      uint32_t si = 0, di = 0;
      while (si < len && di < frame_size) {
        uint8_t h = comp_buf[si++];
        if (h < 128) {
          uint32_t c = h + 1;
          if (si + c > len || di + c > frame_size) return false;
          memcpy(frame_buf + di, comp_buf + si, c);
          si += c; di += c;
        } else if (h > 128) {
          uint32_t c = 257 - h;
          if (si >= len || di + c > frame_size) return false;
          memset(frame_buf + di, comp_buf[si++], c);
          di += c;
        }
      }
      ok = (di == frame_size);
    }
  }
  stat_read_us += micros() - t0;
  stat_bytes   += len;
  stat_reads++;
  return ok;
}

// -----------------------------------------------------------------------------
// Touch (GT911) — the display driver runs its own TwoWire on the board's I2C
// pins; the touch controller shares that bus.  Orientation is read from the
// controller's configured output range.
// -----------------------------------------------------------------------------
static uint16_t tp_xmax = 960, tp_ymax = 540;
static bool     tp_swap = false, touch_ok = false;

static void touchInit() {
  TwoWire *bus = display.getConfig().i2c.wire;
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
    Serial.printf("[bad_apple] touch 0x%02x, range %ux%u%s\n", addr, xm, ym,
                  tp_swap ? " (swapped)" : "");
    return;
  }
  Serial.println("[bad_apple] no touch controller — serial control only");
}

// Rising-edge tap detector; fills panel coordinates on a new touch.
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
    px = (int)ry * 960 / tp_ymax;
    py = (int)(tp_xmax - rx) * 540 / tp_xmax;
  } else {
    px = (int)rx * 960 / tp_xmax;
    py = (int)ry * 540 / tp_ymax;
  }
  return true;
}

static void touchDrain() {           // wait for finger-up, so one tap = one act
  if (!touch_ok) return;
  do { touch.read(); delay(20); } while (touch.isTouched);
}

// -----------------------------------------------------------------------------
// Picker rendering — text drawn straight into the packed 2bpp frame buffer
// with an 8x8 font.  No GFX canvas: the example stays paintPacked-pure.
// -----------------------------------------------------------------------------
static int panel_w, panel_h, row_b;

static inline void setPx(int x, int y) {
  if (x < 0 || y < 0 || x >= panel_w || y >= panel_h) return;
  frame_buf[y * row_b + (x >> 2)] |= 0xC0 >> ((x & 3) * 2);   // black
}

static void drawChar(int x, int y, char c, int scale) {
  if (c < 32 || c > 126) c = '?';
  const char *glyph = font8x8_basic[(int)c];
  for (int gy = 0; gy < 8; gy++)
    for (int gx = 0; gx < 8; gx++)
      if ((glyph[gy] >> gx) & 1)                              // LSB = leftmost
        for (int sy = 0; sy < scale; sy++)
          for (int sx = 0; sx < scale; sx++)
            setPx(x + gx * scale + sx, y + gy * scale + sy);
}

static void drawStr(int x, int y, const char *s, int scale) {
  for (; *s; s++, x += 8 * scale) drawChar(x, y, *s, scale);
}

static void fillRow(int y, int h) {
  for (int yy = y; yy < y + h && yy < panel_h; yy++)
    memset(frame_buf + yy * row_b, 0xFF, row_b);
}

#define MENU_TOP     96
#define MENU_ROW_H   52

static void drawMenu() {
  memset(frame_buf, 0x00, full_frame);                        // white page
  drawStr(32, 28, "SELECT VIDEO", 4);
  fillRow(74, 3);
  char line[64];
  for (int i = 0; i < n_videos; i++) {
    int y = MENU_TOP + i * MENU_ROW_H;
    snprintf(line, sizeof(line), "%d  %-24.24s %3u:%02u  %ufps",
             i + 1, vids[i].name,
             (unsigned)(vids[i].seconds / 60), (unsigned)(vids[i].seconds % 60),
             vids[i].fps);
    drawStr(32, y + 10, line, 3);
  }
  drawStr(32, panel_h - 24, "tap a video to play - tap again for this menu", 2);
  display.paintPacked(frame_buf);
}

// -----------------------------------------------------------------------------
// Video list and selection
// -----------------------------------------------------------------------------
static void scanVideos() {
  File dir = use_sdmmc ? SD_MMC.open(VIDEO_DIR) : SD.open(VIDEO_DIR);
  if (!dir || !dir.isDirectory()) return;
  for (File f = dir.openNextFile(); f && n_videos < MAX_VIDEOS; f = dir.openNextFile()) {
    const char *nm = f.name();
    const char *base = strrchr(nm, '/');
    base = base ? base + 1 : nm;
    size_t len = strlen(base);
    if (f.isDirectory() || len < 5 || strcasecmp(base + len - 4, ".epv") != 0 ||
        base[0] == '.') { f.close(); continue; }
    EpvHeader h;
    if (f.read((uint8_t *)&h, sizeof(h)) == sizeof(h) &&
        memcmp(h.magic, "EPV1", 4) == 0 && h.fps > 0) {
      VideoEntry &v = vids[n_videos++];
      snprintf(v.name, sizeof(v.name), "%.*s", (int)(len - 4), base);  // drop .epv
      v.fps     = h.fps;
      v.seconds = h.frame_count / h.fps;
    }
    f.close();
  }
  dir.close();
  // alphabetical, so numbering is stable across boots
  for (int i = 1; i < n_videos; i++)
    for (int j = i; j > 0 && strcasecmp(vids[j - 1].name, vids[j].name) > 0; j--) {
      VideoEntry t = vids[j]; vids[j] = vids[j - 1]; vids[j - 1] = t;
    }
}

static bool openVideo(const char *path) {
  if (video) video.close();
  video = sdOpen(path);
  if (!video) { Serial.printf("[bad_apple] cannot open %s\n", path); return false; }
  if (video.read((uint8_t *)&hdr, sizeof(hdr)) != sizeof(hdr) ||
      memcmp(hdr.magic, "EPV1", 4) != 0) {
    Serial.printf("[bad_apple] %s is not an EPV1 file\n", path);
    return false;
  }
  line_repeat = (hdr.flags & 1) ? 2 : 1;
  rle         = (hdr.flags & 2) != 0;
  if (hdr.width != panel_w || hdr.height * line_repeat != panel_h) {
    Serial.printf("[bad_apple] %s: %ux%u doesn't match this %dx%d panel\n",
                  path, hdr.width, hdr.height, panel_w, panel_h);
    return false;
  }
  frame_size = (uint32_t)hdr.width * hdr.height / 4;
  Serial.printf("[bad_apple] %s: %ux%u%s%s @ %u fps, %u frames (%us)\n",
                path, hdr.width, hdr.height, line_repeat == 2 ? " line-doubled" : "",
                rle ? " RLE" : "", hdr.fps, hdr.frame_count,
                hdr.frame_count / hdr.fps);
  display.clear();                       // clean slate, kill any ghosting
  frame_idx = 0;
  start_ms = stat_ms = millis();
  stat_frames = 0; stat_reads = 0; stat_read_us = 0;
  stat_paint_us = 0; stat_bytes = 0;
  return true;
}

static bool openVideoIndex(int i) {
  char path[64];
  snprintf(path, sizeof(path), "%s/%s.epv", VIDEO_DIR, vids[i].name);
  if (!openVideo(path)) return false;
  cur_video = i;
  return true;
}

// Blocking picker: returns the chosen index (touch or serial digit).
static int runMenu() {
  drawMenu();
  touchDrain();
  while (Serial.available()) Serial.read();
  for (;;) {
    int px, py;
    if (touchTapped(px, py)) {
      int i = (py - MENU_TOP) / MENU_ROW_H;
      if (py >= MENU_TOP && i >= 0 && i < n_videos) { touchDrain(); return i; }
    }
    if (Serial.available()) {
      int c = Serial.read();
      if (c >= '1' && c < '1' + n_videos) return c - '1';
    }
    delay(30);
  }
}

static void chooseAndOpen() {
  for (;;) {
    int sel = (n_videos > 1) ? runMenu() : 0;
    if (openVideoIndex(sel)) return;
    if (n_videos <= 1) die("the only video in /videos is unplayable");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  if (!display.begin()) die("display.begin() failed");
  display.setQuality(PLAY_QUALITY);
  panel_w = display.getConfig().width;
  panel_h = display.getConfig().height;
  row_b   = panel_w / 4;
  full_frame = (uint32_t)panel_w * panel_h / 4;

  SdPins sd;
  if (!sdPinsForBoard(sd)) die("no SD pin map for this board — set pins manually");
  Serial.printf("[bad_apple] SD pins cs=%d sclk=%d mosi=%d miso=%d\n",
                sd.cs, sd.sclk, sd.mosi, sd.miso);

  // Park all chip-selects high before touching the bus (mandatory on the
  // LilyGo, where SD and LoRa share the SPI lines; harmless elsewhere — in
  // SDMMC mode a high CS/DAT3 keeps the card in native SD mode).
  pinMode(sd.cs, OUTPUT);
  digitalWrite(sd.cs, HIGH);
  if (sd.other_cs >= 0) {
    pinMode(sd.other_cs, OUTPUT);
    digitalWrite(sd.other_cs, HIGH);
  }

  use_sdmmc = sd.sdmmc;
  if (sd.sdmmc) {
    SD_MMC.setPins(sd.sclk, sd.mosi, sd.miso);   // CLK, CMD, D0
    if (!SD_MMC.begin("/sdcard", /*1bit*/ true, /*format*/ false, SDMMC_FREQ_DEFAULT))
      die("SD_MMC.begin() failed — card inserted?");
  } else {
    SPI.begin(sd.sclk, sd.miso, sd.mosi, sd.cs);
    if (!SD.begin(sd.cs, SPI, 25000000)) die("SD.begin() failed — card inserted?");
  }

  // Buffers are sized for the panel, not the file, so any .epv (full-height,
  // line-doubled, RLE or raw) fits and the picker can use frame_buf too.
  frame_buf = allocBuf(full_frame, "frame buffer");
  if (!frame_buf) die("frame buffer alloc failed");
  comp_cap = full_frame + full_frame / 128 + 2;   // PackBits worst case
  comp_buf = allocBuf(comp_cap, "RLE buffer");
  if (!comp_buf) die("RLE buffer alloc failed");

  touchInit();
  scanVideos();

  if (n_videos == 0) {
    // no /videos folder (or nothing in it) — try the old fixed path
    legacy = true;
    Serial.printf("[bad_apple] nothing in %s, trying %s\n", VIDEO_DIR, LEGACY_PATH);
    if (!openVideo(LEGACY_PATH))
      die("no videos: put .epv files in a /videos folder on the SD card");
  } else {
    Serial.printf("[bad_apple] %d video%s in %s\n", n_videos,
                  n_videos == 1 ? "" : "s", VIDEO_DIR);
    chooseAndOpen();
  }
}

static bool paused = false;

void loop() {
  // ---- controls: touch tap -> picker; serial 'p' pause, 'm' picker --------
  int px, py;
  if (!legacy && n_videos > 1 && touchTapped(px, py) && millis() - start_ms > 500) {
    chooseAndOpen();
    return;
  }
  if (Serial.available()) {
    int ch = Serial.read();
    if (ch == 'p') {
      paused = !paused;
      Serial.printf("[bad_apple] %s at frame %u\n", paused ? "PAUSED" : "resumed",
                    (unsigned)frame_idx);
      if (!paused)   // re-anchor so playback resumes at tempo, not in a sprint
        start_ms = millis() - (uint32_t)((uint64_t)frame_idx * 1000u / hdr.fps);
    } else if (ch == 'm' && !legacy && n_videos > 1) {
      paused = false;
      chooseAndOpen();
      return;
    }
  }
  if (paused) { delay(50); return; }

  // ---- read next frame (overlaps the previous frame's panel drive) ----
  if (!readFrame()) {
    // end of file — loop the video
    Serial.printf("[bad_apple] loop end: %u frames in %.1fs\n",
                  frame_idx, (millis() - start_ms) / 1000.0f);
    video.seek(sizeof(EpvHeader));
    frame_idx = 0;
    start_ms  = millis();
    display.clear(nullptr, 0, EPD_Painter::ClearMode::SOFT);
    return;
  }

  // ---- pace to the file's frame rate ----
  // If we're behind, re-anchor the schedule instead of remembering the debt:
  // the video just runs slower through heavy scenes and resumes normal tempo
  // afterwards, rather than sprinting to catch up.
  int32_t wait = (int32_t)(start_ms + frame_idx * 1000u / hdr.fps) - (int32_t)millis();
  if (wait > 0) delay(wait);
  else start_ms -= wait;

  uint32_t t0 = micros();
  display.paintPacked(frame_buf, line_repeat);   // blocks ≈ previous paint's remainder
  stat_paint_us += micros() - t0;
  frame_idx++;
  stat_frames++;

  // ---- stats every 5 s ----
  uint32_t now = millis();
  if (now - stat_ms >= 5000) {
    Serial.printf("[bad_apple] %.1f fps painted, read %lu ms (%.2f MB/s), paint %lu ms\n",
                  stat_frames * 1000.0f / (now - stat_ms),
                  stat_read_us / 1000 / (stat_reads ? stat_reads : 1),
                  stat_read_us ? (float)stat_bytes / stat_read_us : 0.0f,
                  stat_paint_us / 1000 / (stat_frames ? stat_frames : 1));
    stat_frames = 0; stat_reads = 0; stat_read_us = 0; stat_paint_us = 0;
    stat_bytes = 0; stat_ms = now;
  }
}
