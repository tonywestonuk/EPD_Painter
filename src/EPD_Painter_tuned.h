#pragma once

// ============================================================================
// EPD_Painter_tuned.h — the on-glass-tuned 16-grey train tables, as a blob.
//
// The tuneup example tunes a board's 16-grey trains against its own panel
// with a flatbed scanner (the same closed optical loop that produced the
// shipped tables in EPD_Painter_trains.h). This header defines how that
// result is packaged, validated and installed.
//
// STORAGE IS NOT OUR BUSINESS. This header deliberately knows nothing
// about LittleFS, SD, NVS or HTTP — it moves bytes in and out of a plain
// buffer and leaves the keeping of them to the sketch. That way including
// it never drags a filesystem into a project that doesn't want one, and
// you are free to hold the tables wherever suits: flash, an SD card, an
// EEPROM, or a download.
//
//   // ---- installing tables you have loaded from somewhere ----
//   uint8_t buf[EPD_PainterTuned::BLOB_SIZE];
//   size_t n = my_read_bytes(buf, sizeof(buf));         // your storage
//   if (EPD_PainterTuned::install(epd.driver(), buf, n))
//     Serial.println("using tuned trains");
//
//   // ---- packaging tables to store ----
//   uint8_t buf[EPD_PainterTuned::BLOB_SIZE];
//   EPD_PainterTuned::build(epd.driver(), period_us, apply, remove, buf);
//   my_write_bytes(buf, sizeof(buf));                   // your storage
//
// See examples/other/tuneup/tuned_storage.h for a ~30-line LittleFS
// implementation of "your storage".
//
// A blob carries the pass period the tables were tuned at, because trains
// are only valid at that period (see Config::g16_pass_us_normal). It is
// keyed to the board's pins and geometry and CRC-guarded, so tables from
// a different board — or a corrupted read — are refused rather than
// painted.
// ============================================================================

#include <stdint.h>
#include <string.h>
#include "EPD_Painter.h"

namespace EPD_PainterTuned {

struct __attribute__((packed)) Grey16Blob {
  uint32_t magic;           // 'EPT2'
  uint32_t board_key;       // hash of pins + geometry — blob is board-specific
  uint32_t pass_us_normal;  // g16 NORMAL pass period the tables were tuned at
  uint8_t  apply[16][13];   // apply trains (level 0 unused)
  uint8_t  remove[16][13];  // charge-matched removes
  // What each level ACTUALLY renders as, measured on the glass at the end
  // of the tune and normalised so level 0 = 255 (paper white) and level 15
  // = 0 (deepest black). The levels are not evenly spaced — on a real
  // panel the light end bunches up — so anything converting a greyscale
  // image should quantise against THIS curve rather than assuming even
  // steps, or it diffuses error into levels that cannot express it.
  // All-zero means "not measured"; treat the levels as even.
  uint8_t  level_lum[16];
  uint32_t crc;             // CRC32 of everything above
};

static const uint32_t MAGIC = 0x45505432;      // 'EPT2'
static const size_t   BLOB_SIZE = sizeof(Grey16Blob);

inline uint32_t crc32(const uint8_t *p, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  while (n--) {
    c ^= *p++;
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1)));
  }
  return c ^ 0xFFFFFFFFu;
}

// Board identity: enough to stop tables tuned on one board being applied
// to another via a copied file or a shared card.
inline uint32_t boardKey(const EPD_Painter::Config &c) {
  uint32_t k = 2166136261u;
  auto mix = [&](int v) { k = (k ^ (uint32_t)(v & 0xFF)) * 16777619u; };
  for (int i = 0; i < 8; i++) mix(c.data_pins[i]);
  mix(c.i2c.sda);
  mix(c.i2c.scl);
  mix(c.width & 0xFF);
  mix(c.width >> 8);
  mix(c.height & 0xFF);
  mix(c.height >> 8);
  return k;
}

// Is this buffer a blob we should trust for this board?
inline bool valid(const EPD_Painter &p, const uint8_t *data, size_t len) {
  if (!data || len != BLOB_SIZE) return false;
  Grey16Blob b;
  memcpy(&b, data, sizeof(b));
  if (b.magic != MAGIC) return false;
  if (b.crc != crc32(data, sizeof(b) - sizeof(b.crc))) return false;
  if (b.board_key != boardKey(p._config)) return false;
  return b.pass_us_normal >= 5000 && b.pass_us_normal <= 60000;
}

// The installed tables live here for the lifetime of the program, because
// Config::trains points at them rather than copying.
inline uint8_t (*installedApply())[13]  { static uint8_t t[16][13]; return t; }
inline uint8_t (*installedRemove())[13] { static uint8_t t[16][13]; return t; }
inline uint8_t *installedLum()          { static uint8_t t[16]; return t; }
inline bool &haveLumFlag()              { static bool f = false;  return f; }

// The measured render level of each grey, 255 = paper white .. 0 = black,
// or nullptr if this board has never been measured. Feed it to any
// greyscale conversion so it quantises against what the panel really
// does — see the level_lum note above.
inline const uint8_t *levelLuminance() {
  return haveLumFlag() ? installedLum() : nullptr;
}

// Install tables from a blob: Config::trains points at them, the tuned
// pass period is restored, and the 16-grey train library is rebuilt.
// Returns false — changing nothing — if the blob is not valid for this
// board, so it is safe to call on any hardware with any buffer.
inline bool install(EPD_Painter &p, const uint8_t *data, size_t len) {
  if (!valid(p, data, len)) return false;
  Grey16Blob b;
  memcpy(&b, data, sizeof(b));
  memcpy(installedApply(),  b.apply,  sizeof(b.apply));
  memcpy(installedRemove(), b.remove, sizeof(b.remove));
  memcpy(installedLum(),    b.level_lum, sizeof(b.level_lum));
  bool any = false;
  for (int i = 0; i < 16; i++) any |= (b.level_lum[i] != 0);
  haveLumFlag() = any;
  p._config.trains.g16_apply  = installedApply();
  p._config.trains.g16_remove = installedRemove();
  p._config.g16_pass_us_normal = (int)b.pass_us_normal;
  p.rebuildDecisionTrains();
  return true;
}

// Package tables into `out` (must be BLOB_SIZE bytes). period_us is the
// g16 NORMAL pass period they were tuned at.
// level_lum may be null when the levels were not measured.
inline size_t build(const EPD_Painter &p, uint32_t period_us,
                    const uint8_t apply[16][13], const uint8_t remove[16][13],
                    const uint8_t *level_lum, uint8_t *out) {
  Grey16Blob b;
  b.magic = MAGIC;
  b.board_key = boardKey(p._config);
  b.pass_us_normal = period_us;
  memcpy(b.apply,  apply,  sizeof(b.apply));
  memcpy(b.remove, remove, sizeof(b.remove));
  if (level_lum) memcpy(b.level_lum, level_lum, sizeof(b.level_lum));
  else           memset(b.level_lum, 0, sizeof(b.level_lum));
  b.crc = crc32((const uint8_t *)&b, sizeof(b) - sizeof(b.crc));
  memcpy(out, &b, sizeof(b));
  return sizeof(b);
}

}  // namespace EPD_PainterTuned
