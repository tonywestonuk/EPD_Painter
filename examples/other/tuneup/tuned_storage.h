// ============================================================================
// tuned_storage.h — where THIS sketch keeps its tuned tables.
//
// The library (src/EPD_Painter_tuned.h) only knows how to build, validate
// and install the blob; it deliberately has no idea where the bytes live,
// so that including it never drags a filesystem into a project. This file
// is the other half for the tuneup example: about thirty lines of
// LittleFS.
//
// Swap this out to keep the tables somewhere else — an SD card, NVS, or
// pulled from a server at boot. Nothing else in the sketch changes.
// ============================================================================

#pragma once

#include <LittleFS.h>
#include "EPD_Painter_tuned.h"

static const char *TUNED_PATH = "/epd_g16_tuned.bin";

// Read the blob and install it. False (changing nothing) if absent,
// corrupt, or tuned on a different board.
static bool tunedLoad(EPD_Painter &p) {
  File f = LittleFS.open(TUNED_PATH, "r");
  if (!f) return false;
  uint8_t buf[EPD_PainterTuned::BLOB_SIZE];
  const size_t n = f.read(buf, sizeof(buf));
  f.close();
  return EPD_PainterTuned::install(p, buf, n);
}

static bool tunedSave(EPD_Painter &p, uint32_t period_us,
                      const uint8_t apply[16][13], const uint8_t remove[16][13],
                      const uint8_t *level_lum = nullptr) {
  uint8_t buf[EPD_PainterTuned::BLOB_SIZE];
  EPD_PainterTuned::build(p, period_us, apply, remove, level_lum, buf);
  File f = LittleFS.open(TUNED_PATH, "w");
  if (!f) return false;
  const size_t n = f.write(buf, sizeof(buf));
  f.close();
  return n == sizeof(buf);
}

static bool tunedPresent(EPD_Painter &p) {
  File f = LittleFS.open(TUNED_PATH, "r");
  if (!f) return false;
  uint8_t buf[EPD_PainterTuned::BLOB_SIZE];
  const size_t n = f.read(buf, sizeof(buf));
  f.close();
  return EPD_PainterTuned::valid(p, buf, n);
}

static bool tunedErase() { return LittleFS.remove(TUNED_PATH); }
