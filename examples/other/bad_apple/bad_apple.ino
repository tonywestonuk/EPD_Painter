// =============================================================================
//  Bad Apple  —  .epv video player for EPD_Painter / ESP32-S3
//
//  Streams pre-packed native frames from SD straight into paintPacked().
//  There is no pixel processing on the ESP32 at all: the .epv file (made
//  with video2epv.py in this folder) already holds frames in the driver's
//  packed 2bpp format — 4 pixels/byte, first pixel in the MSBs, 0=white
//  3=black — so each frame is one SD read and one paintPacked() call.
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
//  Setup (full instructions in README.md): convert any video, copy the
//  result to the SD card root as /badapple.epv, pick your board below, flash.
//
//      python3 video2epv.py your_video.mp4 badapple.epv --fps 20
// =============================================================================

// Choose your hardware (or leave all commented for auto-detect):
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <EPD_Painter.h>
#include <SD_MMC.h>
#include <SPI.h>
#include <SD.h>
#include "esp_heap_caps.h"

static const char *VIDEO_PATH = "/badapple.epv";

// FAST keeps up with 15 fps; try QUALITY_NORMAL for calibrated greys if the
// frame rate holds up on your card.
static const EPD_Painter::Quality PLAY_QUALITY = EPD_Painter::Quality::QUALITY_FAST;


static EPD_Painter display(EPD_PAINTER_PRESET);

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

// -----------------------------------------------------------------------------
// .epv container
// -----------------------------------------------------------------------------
struct __attribute__((packed)) EpvHeader {
  char     magic[4];      // "EPV1"
  uint16_t width;
  uint16_t height;
  uint16_t fps;
  uint16_t flags;
  uint32_t frame_count;
};

static File      video;
static EpvHeader hdr;
static int       line_repeat = 1;
static bool      rle         = false;
static uint32_t  frame_size  = 0;

static uint8_t  *frame_buf   = nullptr;
static uint8_t  *comp_buf    = nullptr;   // RLE payload staging
static uint32_t  comp_cap    = 0;
static uint32_t  frame_idx   = 0;
static uint32_t  start_ms    = 0;

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

void setup() {
  Serial.begin(115200);
  delay(200);

  if (!display.begin()) die("display.begin() failed");
  display.setQuality(PLAY_QUALITY);

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

  if (sd.sdmmc) {
    SD_MMC.setPins(sd.sclk, sd.mosi, sd.miso);   // CLK, CMD, D0
    if (!SD_MMC.begin("/sdcard", /*1bit*/ true, /*format*/ false, SDMMC_FREQ_DEFAULT))
      die("SD_MMC.begin() failed — card inserted?");
    video = SD_MMC.open(VIDEO_PATH, FILE_READ);
  } else {
    SPI.begin(sd.sclk, sd.miso, sd.mosi, sd.cs);
    if (!SD.begin(sd.cs, SPI, 25000000)) die("SD.begin() failed — card inserted?");
    video = SD.open(VIDEO_PATH, FILE_READ);
  }
  if (!video) die("could not open /badapple.epv");
  if (video.read((uint8_t *)&hdr, sizeof(hdr)) != sizeof(hdr) ||
      memcmp(hdr.magic, "EPV1", 4) != 0)
    die("not an EPV1 file");
  line_repeat = (hdr.flags & 1) ? 2 : 1;
  rle         = (hdr.flags & 2) != 0;
  if (hdr.width != display.getConfig().width ||
      hdr.height * line_repeat != display.getConfig().height)
    die("video dimensions don't match panel");

  frame_size = (uint32_t)hdr.width * hdr.height / 4;
  frame_buf  = allocBuf(frame_size, "frame buffer");
  if (!frame_buf) die("frame buffer alloc failed");
  if (rle) {
    comp_cap = frame_size + frame_size / 128 + 2;   // PackBits worst case
    comp_buf = allocBuf(comp_cap, "RLE buffer");
    if (!comp_buf) die("RLE buffer alloc failed");
  }

  Serial.printf("[bad_apple] %ux%u%s%s @ %u fps, %u frames (%us), %lu B/frame\n",
                hdr.width, hdr.height, line_repeat == 2 ? " line-doubled" : "",
                rle ? " RLE" : "", hdr.fps, hdr.frame_count,
                hdr.frame_count / hdr.fps, (unsigned long)frame_size);

  display.clear();                       // clean slate, kill any ghosting
  start_ms = stat_ms = millis();
}

void loop() {
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
