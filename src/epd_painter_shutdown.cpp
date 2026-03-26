#include <cstring>
#include "EPD_Painter.h"
#include "epd_painter_shutdown.h"

#ifdef ARDUINO
#include <Preferences.h>
#include <esp_heap_caps.h>
#include <LittleFS.h>
#include <esp_partition.h>
#endif

extern "C" void epd_painter_compact_pixels(const uint8_t* input, uint8_t* output, uint32_t size);

#if defined(ARDUINO) && EPD_PAINTER_ENABLE_AUTO_SHUTDOWN

static const char* IMG_PATH = "/.epd_painter_shutdown.img";

static bool hasShutdownFilesystemPartition() {
  if (esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, nullptr)) {
    return true;
  }
#ifdef ESP_PARTITION_SUBTYPE_DATA_LITTLEFS
  if (esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_LITTLEFS, nullptr)) {
    return true;
  }
#endif
  return false;
}

static bool mountShutdownFs(bool formatOnFail) {
  if (!hasShutdownFilesystemPartition()) {
    Serial.println("Shutdown: no SPIFFS/LittleFS partition found");
    return false;
  }
  return formatOnFail ? LittleFS.begin(true) : LittleFS.begin(false);
}


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
// Returns true if VBUS (USB power) is present, via BQ25896 reg 0x0B.
// Checks VBUS_GD (bit 7) or any VBUS_STAT (bits 6:3) — VBUS_GD alone can
// read 0 even with USB connected depending on adapter type detection.
bool EPD_PainterShutdown::isUsbConnected() {
  const auto cfg = _epd->getConfig();
  TwoWire* wire = cfg.i2c.wire;
  if (!wire) return false;
  const uint8_t BQ25896_ADDR = 0x6B;
  wire->beginTransmission(BQ25896_ADDR);
  wire->write(0x0B);
  if (wire->endTransmission(false) != 0) return false;
  uint8_t n = wire->requestFrom(BQ25896_ADDR, (uint8_t)1);
  if (n == 0 || !wire->available()) return false;
  uint8_t reg = wire->read();
  return (reg & 0xF8) != 0;  // VBUS_GD (bit 7) or VBUS_STAT (bits 6:3)
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

  if (isUsbConnected()) {
    printf("USB Connected - skipping shutdown handling \n");
    return;
  }


  // Shutdown state is stored as a uint8_t:
  //   0 = normal startup
  //   1 = shutdown requested (popup shown if autoShutdown=false, else auto-proceed)
  //   2 = force shutdown (always auto-proceed, no popup)
  Preferences prefs;
  prefs.begin("power", false);
  uint8_t flag = prefs.getUChar("shutdown", 0);


  if (flag == 2) {
    // Force shutdown — clear flag and proceed immediately, no popup.
    prefs.putUChar("shutdown", 0);
    prefs.end();
    Serial.println("Force shutdown.");
    _pending = true;
    proceed();
    return;
  }

  if (flag == 1) {
    // Normal shutdown request — defer to proceed()/cancel().
    prefs.putUChar("shutdown", 0);
    prefs.end();
    _pending = true;
    Serial.println("Shutdown pending. Call proceed() or cancel().");
    return;
  }

  // Normal startup: arm flag for next reset, then DC-balance the screen.
  prefs.putUChar("shutdown", 1);
  prefs.end();
  Serial.println("Running. Reset again to shut down.");

  const auto cfg = _epd->getConfig();
  const size_t packed_size = (size_t)cfg.width * cfg.height / 4;
  if (mountShutdownFs(false)) {
    if(!LittleFS.exists(IMG_PATH)) return;  // no image, skip
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

// ---------------------------------------------------------------------------
void EPD_PainterShutdown::proceed() {
  if (!_pending) return;
  _pending = false;

  if (_pre_shutdown_cb) {
    Serial.println("Shutdown: running pre-shutdown callback...");
    _pre_shutdown_cb();
  }

  Serial.println("Shutdown: proceeding...");

  const auto cfg = _epd->getConfig();
  const size_t packed_size = (size_t)cfg.width * cfg.height / 4;
  bool fsOk = mountShutdownFs(false) || mountShutdownFs(true);

  if (!fsOk) {
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
        size_t bytes_read = f.read(packed, packed_size);
        f.close();
        Serial.printf("Shutdown: read %u bytes\n", bytes_read);
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

  // If still alive (USB connected), restart into normal operation.
  ESP.restart();
}

// ---------------------------------------------------------------------------
void EPD_PainterShutdown::startIdleTimer(uint32_t seconds) {
  cancelIdleTimer();
  _idle_timer = xTimerCreate(
      "epd_idle",
      pdMS_TO_TICKS(seconds * 1000),
      pdFALSE,           // one-shot
      this,              // timer ID = this pointer
      _idle_timer_cb);
  xTimerStart(_idle_timer, 0);
  Serial.printf("Idle timer started: %u seconds\n", seconds);
}

void EPD_PainterShutdown::resetIdleTimer() {
  if (_idle_timer)
    xTimerReset(_idle_timer, 0);
}

void EPD_PainterShutdown::cancelIdleTimer() {
  if (_idle_timer) {
    xTimerStop(_idle_timer, 0);
    xTimerDelete(_idle_timer, 0);
    _idle_timer = nullptr;
  }
}

void EPD_PainterShutdown::_idle_timer_cb(TimerHandle_t h) {
  EPD_PainterShutdown* self = static_cast<EPD_PainterShutdown*>(pvTimerGetTimerID(h));
  Serial.println("Idle timer expired. Shutting down.");
  self->shutdown(true);
}

// ---------------------------------------------------------------------------
void EPD_PainterShutdown::shutdown(bool force) {
  Preferences prefs;
  prefs.begin("power", false);
  prefs.putUChar("shutdown", force ? 2 : 1);
  prefs.end();
  Serial.printf("Shutdown requested (force=%d). Restarting...\n", force);
  delay(100);
  ESP.restart();
}

// ---------------------------------------------------------------------------
void EPD_PainterShutdown::cancel() {
  if (!_pending) return;
  _pending = false;

  // Re-arm the flag so the next reset triggers shutdown again.
  Preferences prefs;
  prefs.begin("power", false);
  prefs.putUChar("shutdown", 1);
  prefs.end();
  Serial.println("Shutdown cancelled. Re-armed for next reset.");
}

#else  // !ARDUINO or shutdown feature disabled

extern "C" void epd_painter_compact_pixels(const uint8_t*, uint8_t*, uint32_t);

EPD_PainterShutdown::EPD_PainterShutdown(EPD_Painter* p_epd) : _epd(p_epd) {}
void EPD_PainterShutdown::showMand() {}
void EPD_PainterShutdown::showMandAndPaint() {}
bool EPD_PainterShutdown::isUsbConnected() { return false; }
void EPD_PainterShutdown::powerOff() {}
void EPD_PainterShutdown::proceed() {}
void EPD_PainterShutdown::cancel() {}
void EPD_PainterShutdown::shutdown(bool) {}
void EPD_PainterShutdown::startIdleTimer(uint32_t) {}
void EPD_PainterShutdown::resetIdleTimer() {}
void EPD_PainterShutdown::cancelIdleTimer() {}
void EPD_PainterShutdown::_idle_timer_cb(TimerHandle_t) {}

#endif // ARDUINO
