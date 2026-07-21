// =============================================================================
// epd_painter_decision.cpp — the decision engine's discovery pass (phase B).
//
// C reference implementation; see DECISION_ENGINE.md for the design.
//
// Discovery reads the packed paintbuffer against the packed screenbuffer and
// emits, per line: slot-encoded planes, chunk masks, a todo word of pending
// decisions, and a sweep list for the pass loop. In phase B the batcher is
// the 4-level compatibility mapping — apply-decisions in plane 0 (slot =
// grey value), remove-decisions in plane 1 — which reproduces the July
// dual-plane engine's output byte for byte. The SIMD ink_dual assembly
// remains the default engine; this path is enabled with setDecisionEngine()
// and cross-checked against the assembly with EPD_DECISION_VERIFY.
//
// Pixel model (unchanged from the dual-plane engine):
//   dark  = new    where screen == 0        (drawn onto white ground)
//   light = screen where new != screen      (erased toward white)
//   screen' = new where drawn, 0 where erased, else unchanged
// A grey-to-grey pixel erases this paint and draws the next (two-step);
// collapsing that into one paint is phase C's batcher, not this one.
// =============================================================================

#include "EPD_Painter.h"
#include <string.h>
#include "esp_timer.h"

// Define to run the C discovery and the SIMD assembly side by side each
// paint and report mismatches on serial. The assembly's output drives the
// panel; the C path is compared and discarded. Costs one extra row buffer
// set and roughly doubles discovery time — a correctness harness, not a
// mode to ship with.
// #define EPD_DECISION_VERIFY

extern "C" uint32_t epd_painter_ink_dual(const uint8_t *packed_paintbuffer,
                                         uint8_t *packed_fastbuffer,
                                         uint8_t *packed_lightbuffer,
                                         uint8_t *packed_screenbuffer,
                                         uint32_t length, uint32_t *maskL_out);

// ---- SWAR helpers: 16 pixels (2-bit fields) per 32-bit word --------------

// 01 in every field that is non-zero
static inline uint32_t fieldNonzero(uint32_t v) {
  return (v | (v >> 1)) & 0x55555555u;
}
// spread 01-per-field to 11-per-field
static inline uint32_t fieldSpread(uint32_t m) {
  return m | (m << 1);
}

// ---- one line -------------------------------------------------------------

uint32_t EPD_Painter::_decision_discover_row(const uint8_t *pb_row,
                                             uint8_t *fbD_row, uint8_t *fbL_row,
                                             uint8_t *sb_row,
                                             uint32_t *maskL_out,
                                             uint32_t *todo_out) {
  uint32_t maskD = 0, maskL = 0, todo = 0;
  const int words = packed_row_bytes / 4;          // 16px per word
  const uint32_t *pb = (const uint32_t *)pb_row;
  uint32_t *fbD = (uint32_t *)fbD_row;
  uint32_t *fbL = (uint32_t *)fbL_row;
  uint32_t *sb  = (uint32_t *)sb_row;

  // chunk = 64 px = 4 words = 16 bytes; mask bits count DOWN from bit 31
  // (chunk 0 = bit 31), matching the assembly's rotating-mask convention.
  // The light plane's stores are skipped for empty chunks (PSRAM — idle
  // writes avoided), also matching the assembly: bytes there are stale
  // don't-cares guarded by the mask.
  for (int c = 0; c < words / 4; c++) {
    uint32_t lbuf[4];
    uint32_t anyD = 0, anyL = 0;
    for (int i = 0; i < 4; i++) {
      const int w = c * 4 + i;
      const uint32_t nv = pb[w], sv = sb[w];
      const uint32_t occupied = fieldSpread(fieldNonzero(sv)); // 11 where screen != 0
      const uint32_t changed  = fieldSpread(fieldNonzero(nv ^ sv));
      const uint32_t dark  = nv & ~occupied;                   // drawn onto white
      const uint32_t light = sv & changed;                     // erased toward white
      fbD[w]  = dark;
      lbuf[i] = light;
      sb[w]   = (sv & ~changed) | dark;
      anyD |= dark;
      anyL |= light;

      // todo bits: walk the fields of the (rare) non-zero ink words.
      // id = (level << 1) | dir; dir 0 = apply, 1 = remove.
      if (dark | light) {
        uint32_t d = dark, l = light;
        while (d) { todo |= 1u << ((d & 3) << 1);       d >>= 2; }
        while (l) { todo |= 1u << (((l & 3) << 1) | 1); l >>= 2; }
      }
    }
    const uint32_t bit = 0x80000000u >> c;
    if (anyD) maskD |= bit;
    if (anyL) {
      maskL |= bit;
      fbL[c * 4 + 0] = lbuf[0];
      fbL[c * 4 + 1] = lbuf[1];
      fbL[c * 4 + 2] = lbuf[2];
      fbL[c * 4 + 3] = lbuf[3];
    }
  }
  // the field walk sets bogus bits for zero fields (level 0 has no
  // decisions); clear ids 0 and 1
  todo &= ~3u;

  *maskL_out = maskL;
  *todo_out  = todo;
  return maskD;
}

// ---- whole frame ----------------------------------------------------------

uint32_t EPD_Painter::_decision_discover() {
  uint32_t any_work = 0;

#ifdef EPD_DECISION_VERIFY
  static uint32_t verify_paint = 0;
  uint32_t mm_plane = 0, mm_mask = 0, mm_screen = 0, mm_todo_rows = 0;
  static uint8_t c_fbD[240], c_fbL[240], c_sb[240];
  int64_t t_c = 0, t_asm = 0;
#endif

  for (int row = 0; row < _config.height; row++) {
    const uint8_t *pb_row = packed_paintbuffer  + row * packed_row_bytes;
    uint8_t *fbD_row = packed_fastbuffer   + row * packed_row_bytes;
    uint8_t *fbL_row = packed_lightbuffer  + row * packed_row_bytes;
    uint8_t *sb_row  = packed_screenbuffer + row * packed_row_bytes;

#ifdef EPD_DECISION_VERIFY
    // C discovery on scratch copies; assembly on the real buffers (it is
    // the engine of record while the C path is on probation). Plane
    // scratch is seeded from the real buffers so chunks whose stores are
    // legitimately skipped (stale don't-cares behind the mask) compare
    // equal.
    memcpy(c_sb,  sb_row,  packed_row_bytes);
    memcpy(c_fbD, fbD_row, packed_row_bytes);
    memcpy(c_fbL, fbL_row, packed_row_bytes);
    uint32_t c_maskL, c_todo;
    int64_t t0 = esp_timer_get_time();
    uint32_t c_maskD = _decision_discover_row(pb_row, c_fbD, c_fbL, c_sb,
                                              &c_maskL, &c_todo);
    t_c += esp_timer_get_time() - t0;

    t0 = esp_timer_get_time();
    bitmask[row] = epd_painter_ink_dual(pb_row, fbD_row, fbL_row, sb_row,
                                        packed_row_bytes, &bitmask_light[row]);
    t_asm += esp_timer_get_time() - t0;

    if (memcmp(c_fbD, fbD_row, packed_row_bytes) ||
        memcmp(c_fbL, fbL_row, packed_row_bytes))       mm_plane++;
    if (c_maskD != bitmask[row] || c_maskL != bitmask_light[row]) mm_mask++;
    if (memcmp(c_sb, sb_row, packed_row_bytes))         mm_screen++;
    uint32_t todo = c_todo;
#else
    uint32_t todo;
    bitmask[row] = _decision_discover_row(pb_row, fbD_row, fbL_row, sb_row,
                                          &bitmask_light[row], &todo);
#endif

    dec_todo[row] = todo;
    any_work |= bitmask[row] | bitmask_light[row];
  }

#ifdef EPD_DECISION_VERIFY
  printf("[decision verify] paint %lu: mismatch rows plane=%lu mask=%lu screen=%lu | C %lldus asm %lldus\n",
         (unsigned long)verify_paint++, (unsigned long)mm_plane,
         (unsigned long)mm_mask, (unsigned long)mm_screen,
         (long long)t_c, (long long)t_asm);
  (void)mm_todo_rows;
#endif

  return any_work;
}

// ---- phase B compatibility batcher ---------------------------------------
// apply-decisions -> plane 0, remove-decisions -> plane 1, slot = grey
// value: the slot-encoded planes ARE the dual-engine's ink planes, so this
// mapping reproduces the July engine byte for byte. Shared by both engines
// (after the SIMD ink_dual sweep or after _decision_discover()).

void EPD_Painter::_decision_batch_compat() {
  for (int row = 0; row < _config.height; row++) {
    uint8_t n = 0;
    LineSweep *ls = &dec_sweeps[row * DEC_MAX_SWEEPS];
    if (bitmask[row]) {
      ls[n].plane_row = packed_fastbuffer + row * packed_row_bytes;
      ls[n].mask = bitmask[row];
      ls[n].dec[0] = (1 << 1) | 0;   // slot1 = apply level 1
      ls[n].dec[1] = (2 << 1) | 0;
      ls[n].dec[2] = (3 << 1) | 0;
      n++;
    }
    if (bitmask_light[row]) {
      ls[n].plane_row = packed_lightbuffer + row * packed_row_bytes;
      ls[n].mask = bitmask_light[row];
      ls[n].dec[0] = (1 << 1) | 1;   // slot1 = remove level 1
      ls[n].dec[1] = (2 << 1) | 1;
      ls[n].dec[2] = (3 << 1) | 1;
      n++;
    }
    dec_nsweeps[row] = n;
  }
}

// ===========================================================================
// 16-grey discovery + batcher (phase C)
//
// The 4bpp paintbuffer is compared against the 4bpp screenbuffer and every
// changed pixel yields exactly one decision: occupied pixels erase toward
// white (remove(screen level), redrawn next paint — today's grey-to-grey
// two-step, unchanged), bare pixels take their new level (apply(new)).
// Decisions are batched into sweeps greedily in first-appearance order,
// 3 per sweep, so a line using d distinct decisions costs ceil(d/3)
// sweeps — the cost scales with the content, not the palette. Slot planes
// stay 2bpp regardless of grey depth (they hold slot indices, not levels):
// sweep 0 writes the internal fastbuffer, sweep 1 the lightbuffer, sweeps
// 2..9 the PSRAM spill block. Chunks are floated (zeroed) lazily on the
// first pixel that lands in them; untouched chunks stay outside the mask
// as don't-cares, same convention as the assembly.
// ===========================================================================

uint8_t *EPD_Painter::_dec_plane_row(int sweep, int row) {
  if (sweep == 0) return packed_fastbuffer  + (size_t)row * packed_row_bytes;
  if (sweep == 1) return packed_lightbuffer + (size_t)row * packed_row_bytes;
  return dec_spill +
         ((size_t)(sweep - 2) * _config.height + row) * packed_row_bytes;
}

void EPD_Painter::_decision_discover16_row(int row) {
  const int p4rb = packed_row_bytes * 2;             // 4bpp row bytes
  const uint8_t *pb = packed4_paintbuffer  + (size_t)row * p4rb;
  uint8_t       *sb = packed4_screenbuffer + (size_t)row * p4rb;
  LineSweep *ls = &dec_sweeps16[row * DEC_MAX_SWEEPS16];

  int8_t map[DEC_IDS];                               // id -> (sweep<<2)|slot
  memset(map, -1, sizeof(map));
  uint8_t *plane[DEC_MAX_SWEEPS16];
  int ndec = 0, nsweeps = 0;
  uint32_t todo = 0;

  const int chunks = _config.width / 64;             // 64 px = 32 bytes at 4bpp
  for (int c = 0; c < chunks; c++) {
    const uint32_t *pw = (const uint32_t *)(pb + c * 32);
    const uint32_t *sw = (const uint32_t *)(sb + c * 32);
    uint32_t diff = 0;
    for (int i = 0; i < 8; i++) diff |= pw[i] ^ sw[i];
    if (!diff) continue;
    const uint32_t cbit = 0x80000000u >> c;          // asm mask convention

    for (int b = 0; b < 32; b++) {
      const uint8_t nb = pb[c * 32 + b];
      uint8_t sbyte = sb[c * 32 + b];
      if (nb == sbyte) continue;
      for (int h = 0; h < 2; h++) {                  // h=0: first px, high nibble
        const int shift = h ? 0 : 4;
        const uint8_t nv = (nb    >> shift) & 15;
        const uint8_t sv = (sbyte >> shift) & 15;
        if (nv == sv) continue;
        const uint8_t id = sv ? (uint8_t)((sv << 1) | 1)   // remove(screen)
                              : (uint8_t)(nv << 1);        // apply(new)
        sbyte = sv ? (uint8_t)(sbyte & ~(15 << shift))     // erased -> white
                   : (uint8_t)(sbyte | (nv << shift));     // drawn  -> nv

        int m = map[id];
        if (m < 0) {                                 // first sighting this row
          const int s = ndec / 3, slot = ndec % 3;
          if (slot == 0) {                           // open a new sweep
            plane[s] = _dec_plane_row(s, row);
            ls[s].plane_row = plane[s];
            ls[s].mask = 0;
            ls[s].dec[0] = ls[s].dec[1] = ls[s].dec[2] = 0;
            nsweeps = s + 1;
          }
          ls[s].dec[slot] = id;
          m = map[id] = (int8_t)((s << 2) | (slot + 1));
          ndec++;
          todo |= 1u << id;
        }
        const int s = m >> 2;
        uint8_t *pl = plane[s];
        if (!(ls[s].mask & cbit)) {                  // first touch: float chunk
          memset(pl + c * 16, 0, 16);
          ls[s].mask |= cbit;
        }
        const int x = c * 64 + b * 2 + h;
        pl[x >> 2] |= (uint8_t)((m & 3) << ((3 - (x & 3)) * 2));
      }
      sb[c * 32 + b] = sbyte;
    }
  }
  dec_todo[row]    = todo;
  dec_nsweeps[row] = (uint8_t)nsweeps;
}

uint32_t EPD_Painter::_decision_discover16() {
  uint32_t any_work = 0;
  for (int row = 0; row < _config.height; row++) {
    _decision_discover16_row(row);
    any_work |= dec_nsweeps[row];
  }
  return any_work;
}

// ===========================================================================
// 4-level direct-transition discovery
// (see DECISION_ENGINE.md "direct grey-to-grey transitions")
//
// The same greedy-sweep walk as the 16-grey discovery, but over the 2bpp
// buffers and with the (from, to) generalization the engine was built for:
// a changed occupied pixel whose pair train is loaded takes ONE direct
// decision — id 16 | (from << 2) | to — and the screen state lands on the
// new value immediately, no two-step through white. Pairs without a loaded
// train (and plain erases to white) emit remove(screen) exactly as before,
// so behavior degrades gracefully to the legacy engine per pair. Up to 12
// distinct decisions can share a row (3 applies + 3 removes + 6 directs) =
// 4 sweeps: fastbuffer, lightbuffer, then the 2 direct spill planes.
// ===========================================================================

uint8_t *EPD_Painter::_dec_plane_row_dir(int sweep, int row) {
  if (sweep == 0) return packed_fastbuffer  + (size_t)row * packed_row_bytes;
  if (sweep == 1) return packed_lightbuffer + (size_t)row * packed_row_bytes;
  return dec_spill_dir +
         ((size_t)(sweep - 2) * _config.height + row) * packed_row_bytes;
}

void EPD_Painter::_decision_discover_direct_row(int row) {
  const uint8_t *pb = packed_paintbuffer  + (size_t)row * packed_row_bytes;
  uint8_t       *sb = packed_screenbuffer + (size_t)row * packed_row_bytes;
  LineSweep *ls = &dec_sweeps[row * DEC_MAX_SWEEPS];

  int8_t map[DEC_IDS];                               // id -> (sweep<<2)|slot
  memset(map, -1, sizeof(map));
  uint8_t *plane[DEC_MAX_SWEEPS];
  int ndec = 0, nsweeps = 0;
  uint32_t todo = 0;

  const int chunks = _config.width / 64;             // 64 px = 16 bytes at 2bpp
  for (int c = 0; c < chunks; c++) {
    const uint32_t *pw = (const uint32_t *)(pb + c * 16);
    const uint32_t *sw = (const uint32_t *)(sb + c * 16);
    uint32_t diff = 0;
    for (int i = 0; i < 4; i++) diff |= pw[i] ^ sw[i];
    if (!diff) continue;
    const uint32_t cbit = 0x80000000u >> c;          // asm mask convention

    for (int b = 0; b < 16; b++) {
      const uint8_t nb = pb[c * 16 + b];
      uint8_t sbyte = sb[c * 16 + b];
      if (nb == sbyte) continue;
      for (int q = 0; q < 4; q++) {                  // q=0: leftmost px, high bits
        const int shift = (3 - q) * 2;
        const uint8_t nv = (nb    >> shift) & 3;
        const uint8_t sv = (sbyte >> shift) & 3;
        if (nv == sv) continue;
        uint8_t id;
        if (sv == 0) {                               // bare ground: plain apply
          id = (uint8_t)(nv << 1);
          sbyte |= (uint8_t)(nv << shift);
        } else if (nv != 0 && (_dir_loaded & (1u << ((sv << 2) | nv)))) {
          id = directId(sv, nv);                     // tuned direct transition
          sbyte = (uint8_t)((sbyte & ~(3u << shift)) | (nv << shift));
        } else {                                     // erase toward white (the
          id = (uint8_t)((sv << 1) | 1);             // two-step for unloaded
          sbyte = (uint8_t)(sbyte & ~(3u << shift)); // pairs — redrawn next paint)
        }

        int m = map[id];
        if (m < 0) {                                 // first sighting this row
          const int s = ndec / 3, slot = ndec % 3;
          if (slot == 0) {                           // open a new sweep
            plane[s] = _dec_plane_row_dir(s, row);
            ls[s].plane_row = plane[s];
            ls[s].mask = 0;
            ls[s].dec[0] = ls[s].dec[1] = ls[s].dec[2] = 0;
            nsweeps = s + 1;
          }
          ls[s].dec[slot] = id;
          m = map[id] = (int8_t)((s << 2) | (slot + 1));
          ndec++;
          todo |= 1u << id;
        }
        const int s = m >> 2;
        uint8_t *pl = plane[s];
        if (!(ls[s].mask & cbit)) {                  // first touch: float chunk
          memset(pl + c * 16, 0, 16);
          ls[s].mask |= cbit;
        }
        pl[c * 16 + b] |= (uint8_t)((m & 3) << shift);
      }
      sb[c * 16 + b] = sbyte;
    }
  }
  dec_todo[row]    = todo;
  dec_nsweeps[row] = (uint8_t)nsweeps;
}

uint32_t EPD_Painter::_decision_discover_direct() {
  uint32_t any_work = 0;
  for (int row = 0; row < _config.height; row++) {
    _decision_discover_direct_row(row);
    any_work |= dec_nsweeps[row];
  }
  return any_work;
}

// ---- placeholder train library (phase C) ----------------------------------
// Formula trains for NORMAL/HIGH's 13 passes; phase D replaces these with
// scanner-calibrated tables. Dose is counted in half-pass units: a darken
// pass is 2 units, and an odd dose gets one extra darken pass immediately
// half-undone by a whiten in the following pass — the pixel-cap retention
// physics gives roughly a half-step net. 15 levels map onto doses
// round(g * 26 / 15) = 2..26, all distinct, so every level lands on its own
// dose even though there are more levels than passes. Removes whiten from
// pass 0 for long enough to erase their level with margin; overdriving
// toward white is self-limiting (it is how clear() works).

void EPD_Painter::_grey16_build_trains() {
  memset(dec_trains16, 0, sizeof(dec_trains16));
  for (int g = 1; g <= 15; g++) {
    uint8_t *ap = dec_trains16[(g << 1) | 0];
    uint8_t *rm = dec_trains16[(g << 1) | 1];

    const int dose = (g * 2 * DEC_WF_LEN16 + 7) / 15;   // half-pass units
    const int full = dose / 2;
    for (int p = 0; p < full; p++) ap[p] = 1;           // darken
    if ((dose & 1) && full + 1 < DEC_WF_LEN16) {
      ap[full]     = 1;                                 // extra darken…
      ap[full + 1] = 2;                                 // …half taken back
    }

    int wh = full + 3;                                  // erase with margin
    if (wh > DEC_WF_LEN16) wh = DEC_WF_LEN16;
    for (int p = 0; p < wh; p++) rm[p] = 2;             // whiten
  }
}
