#include <cstring>
#include "EPD_Painter_Adafruit.h"
#include "EPD_Painter.h"
#include "epd_painter_shutdown.h"

#ifdef ARDUINO
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <esp_heap_caps.h>
#include <LittleFS.h>
#endif

extern "C" void epd_painter_compact_pixels(const uint8_t* input, uint8_t* output, uint32_t size);

static const char* IMG_PATH = "/.epd_painter_shutdown.img";


// ---------------------------------------------------------------------------
// Generate a Mandelbrot image, compact it to 2bpp, and save it to LittleFS.
// LittleFS must already be mounted by the caller.
// ---------------------------------------------------------------------------
void EPD_PainterShutdown::showMand() {
  const auto cfg = _epd->getConfig();
  const uint16_t W = cfg.width;
  const uint16_t H = cfg.height;
  const size_t   pixel_count  = (size_t)W * H;
  const size_t   packed_size  = pixel_count / 4;

  uint8_t* fb = (uint8_t*)heap_caps_malloc(pixel_count, MALLOC_CAP_SPIRAM);
  if (!fb) { Serial.println("showMand: out of PSRAM for framebuffer"); return; }

  struct Location { float cx, cy, half_w; int max_iter; const char* name; };
  static const Location locs[] = {
    { -0.7436439f,  0.1318259f,  0.005f, 256, "Seahorse spiral"          },
    { -1.4011552f,  0.0000000f,  0.004f, 256, "Feigenbaum point"         },
    { -0.7269000f,  0.1889000f,  0.003f, 256, "Deep seahorse filaments"  },
    { -0.1592900f,  1.0317400f,  0.015f, 192, "Top antenna bulb"         },
  };
  const int N = sizeof(locs) / sizeof(locs[0]);
  const int idx = esp_random() % N;
  const Location& loc = locs[idx];

  Serial.printf("showMand: rendering '%s' (cx=%.7f cy=%.7f zoom=%.4f iters=%d)\n",
    loc.name, loc.cx, loc.cy, loc.half_w, loc.max_iter);

  const int   MAX_ITER  = loc.max_iter;
  const float CX_CENTER = loc.cx;
  const float CY_CENTER = loc.cy;
  const float HALF_W    = loc.half_w;
  const float HALF_H    = HALF_W * H / W;
  for (int py = 0; py < H; py++) {
    float cy = CY_CENTER - HALF_H + 2.0f * HALF_H * py / H;
    for (int px = 0; px < W; px++) {
      float cx = CX_CENTER - HALF_W + 2.0f * HALF_W * px / W;
      float zx = 0.0f, zy = 0.0f;
      int iter = 0;
      while (zx*zx + zy*zy < 4.0f && iter < MAX_ITER) {
        float tmp = zx*zx - zy*zy + cx;
        zy = 2.0f*zx*zy + cy;
        zx = tmp;
        iter++;
      }
      fb[py * W + px] = (iter == MAX_ITER) ? 3 : (uint8_t)(iter * 3 / MAX_ITER);
    }
  }

  uint8_t* packed = (uint8_t*)heap_caps_malloc(packed_size, MALLOC_CAP_SPIRAM);
  if (!packed) {
    Serial.println("showMand: out of PSRAM for packed buffer");
    heap_caps_free(fb);
    return;
  }

  Serial.println("showMand: compacting...");
  epd_painter_compact_pixels(fb, packed, pixel_count);
  heap_caps_free(fb);

  Serial.println("showMand: saving to LittleFS...");
  File f = LittleFS.open(IMG_PATH, FILE_WRITE);
  if (f) {
    size_t written = f.write(packed, packed_size);
    f.close();
    Serial.printf("showMand: wrote %u bytes (expected %u)\n", written, packed_size);
  } else {
    Serial.println("showMand: failed to open file for writing");
  }

  heap_caps_free(packed);
}

// ---------------------------------------------------------------------------
// Fallback: generate Mandelbrot and paint directly (no LittleFS required)
// ---------------------------------------------------------------------------
void EPD_PainterShutdown::showMandAndPaint() {
  const auto cfg = _epd->getConfig();
  const uint16_t W = cfg.width;
  const uint16_t H = cfg.height;
  const size_t   pixel_count = (size_t)W * H;
  const size_t   packed_size = pixel_count / 4;

  uint8_t* fb = (uint8_t*)heap_caps_malloc(pixel_count, MALLOC_CAP_SPIRAM);
  if (!fb) return;

  struct Location { float cx, cy, half_w; int max_iter; };
  static const Location locs[] = {
    { -0.7453954f,  0.1125745f,  0.015f,  128 },
    { -0.7436439f,  0.1318259f,  0.008f,  192 },
    {  0.3750001f,  0.0000000f,  0.030f,  128 },
    { -0.1592900f,  1.0317400f,  0.020f,  160 },
    { -1.9999900f,  0.0000000f,  0.010f,  256 },
    { -0.5621950f,  0.6427570f,  0.012f,  192 },
    {  0.0000000f,  0.0000000f,  1.600f,   64 },
    { -0.7269000f,  0.1889000f,  0.004f,  256 },
    { -1.4011552f,  0.0000000f,  0.006f,  192 },
    { -0.7490000f,  0.0560000f,  0.006f,  192 },
  };
  const int N = sizeof(locs) / sizeof(locs[0]);
  const Location& loc = locs[esp_random() % N];

  const float HALF_H = loc.half_w * H / W;
  for (int py = 0; py < H; py++) {
    float cy = loc.cy - HALF_H + 2.0f * HALF_H * py / H;
    for (int px = 0; px < W; px++) {
      float cx = loc.cx - loc.half_w + 2.0f * loc.half_w * px / W;
      float zx = 0.0f, zy = 0.0f;
      int iter = 0;
      while (zx*zx + zy*zy < 4.0f && iter < loc.max_iter) {
        float tmp = zx*zx - zy*zy + cx;
        zy = 2.0f*zx*zy + cy;
        zx = tmp;
        iter++;
      }
      fb[py * W + px] = (iter == loc.max_iter) ? 3 : (uint8_t)(iter * 3 / loc.max_iter);
    }
  }

  uint8_t* packed = (uint8_t*)heap_caps_malloc(packed_size, MALLOC_CAP_SPIRAM);
  if (packed) {
    
    // Clear current image
    // load new image
    epd_painter_compact_pixels(fb, packed, pixel_count);
    heap_caps_free(fb);

    _epd->clear();

    _epd->paintPacked(packed);
    heap_caps_free(packed);
  } else {
    heap_caps_free(fb);
  }
}

// ---------------------------------------------------------------------------
void EPD_PainterShutdown::powerOff() {
  const auto cfg = _epd->getConfig();
  if (cfg.pin_syspwr != -1) {
    pinMode(cfg.pin_syspwr, OUTPUT);
    digitalWrite(cfg.pin_syspwr, HIGH);
    delay(100);
    digitalWrite(cfg.pin_syspwr, LOW);
    Serial.println("System power pin pulsed. If you see this, USB is keeping device alive.");
  } else {
    // BQ25896: set BATFET_DIS (reg 0x09 bit 5) to disconnect battery FET
    const uint8_t BQ25896_ADDR = 0x6B;
    TwoWire* wire = cfg.i2c.wire;
    wire->beginTransmission(BQ25896_ADDR);
    wire->write(0x09);
    wire->endTransmission();
    wire->requestFrom(BQ25896_ADDR, (uint8_t)1);
    uint8_t reg = wire->read();
    wire->beginTransmission(BQ25896_ADDR);
    wire->write(0x09);
    wire->write(reg | (1 << 5));  // BATFET_DIS
    wire->endTransmission();
    Serial.println("BQ25896 BATFET disabled. If you see this, USB is keeping device alive.");
  }
}

// ---------------------------------------------------------------------------
EPD_PainterShutdown::EPD_PainterShutdown(EPD_Painter* p_epd) {
  _epd = p_epd;

  // --- Check shutdown flag BEFORE doing anything else ---
  Preferences prefs;
  prefs.begin("power", false);
  bool shutdownFlag = prefs.getBool("shutdown", false);

  if (shutdownFlag) {
    Serial.println("Shutdown flag detected. Powering off...");
    prefs.putBool("shutdown", false);
    prefs.end();

    const auto cfg = _epd->getConfig();
    const size_t packed_size = (size_t)cfg.width * cfg.height / 4;
    bool fsOk = LittleFS.begin(false) || LittleFS.begin(true);

    if (!fsOk) {
      // No LittleFS partition — generate and paint directly from PSRAM
      Serial.println("LittleFS unavailable — painting Mandelbrot directly");
      showMandAndPaint();
    } else {
      if (!LittleFS.exists(IMG_PATH)) {
        Serial.println("No shutdown image found — generating...");
        showMand();
      }

      uint8_t* packed = (uint8_t*)heap_caps_malloc(packed_size, MALLOC_CAP_SPIRAM);
      if (packed) {
        File f = LittleFS.open(IMG_PATH, FILE_READ);
        if (f) {
          size_t file_size = f.size();
          size_t bytes_read = f.read(packed, packed_size);
          f.close();
          Serial.printf("paintPacked: file=%u expected=%u read=%u\n", file_size, packed_size, bytes_read);
          _epd->setQuality(EPD_Painter::Quality::QUALITY_HIGH);
          _epd->clear();
          _epd->paintPacked(packed);
        } else {
          Serial.println("Failed to open shutdown image");
        }
        heap_caps_free(packed);
      }
      LittleFS.end();
    }

    delay(500);
    powerOff();

    // If still alive (USB connected), restart back into normal operation
    ESP.restart();
    return;
  }

  // --- Flag was false: normal startup, arm flag for next reset ---
  prefs.putBool("shutdown", true);
  prefs.end();
  Serial.println("Running. Reset again to shut down.");

  // DC-balance pass: erase the shutdown image if it exists
  const auto cfg = _epd->getConfig();
  const size_t packed_size = (size_t)cfg.width * cfg.height / 4;
  if (LittleFS.begin(false) && LittleFS.exists(IMG_PATH)) {
    uint8_t* packed = (uint8_t*)heap_caps_malloc(packed_size, MALLOC_CAP_SPIRAM);
    if (packed) {
      File f = LittleFS.open(IMG_PATH, FILE_READ);
      if (f) {
        f.read(packed, packed_size);
        f.close();
        _epd->setQuality(EPD_Painter::Quality::QUALITY_HIGH);
        _epd->unpaintPacked(packed);
        _epd->setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
      }
      heap_caps_free(packed);
    }
    LittleFS.end();
  }
}