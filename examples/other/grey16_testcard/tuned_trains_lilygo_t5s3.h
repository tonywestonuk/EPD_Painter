// Scanner-tuned 16-grey apply trains — LilyGo T5 S3 GPS, QUALITY_NORMAL (13 passes).
// Tuned 20 July 2026 with the match-card method: each level's flat native
// patch optically matched (flatbed scan) against a reference patch built
// from only full-white and full-black pixels Bayer-dithered to density
// g/15. Tuned UNDER THE CONSTANT PASS PERIOD (15 ms NORMAL) — phase D's
// dose-stabilisation fix — and cross-verified: strictly monotonic on both
// the match card (max err 4.2 scan units) and the full-height staircase,
// the content transfer test the pre-fix timing failed.
//
// Codes per pass: 0 float, 1 darken, 2 whiten.
//
// What the glass taught us (vs the formula library's assumptions):
//  - A whiten pass is STRONG: after a short darken run one whiten takes
//    back 20-30 units, near saturation ~10 — never the "half step" the
//    formula assumed.
//  - The dose response saturates hard: darken passes 8..13 buy ~3 units
//    total. Deep greys cannot be spaced by run length alone.
//  - Fine steps come from RE-DARKENING after a whiten: patterns like
//    1,1,2,1,1 climb from the lifted grey in small fresh-response steps.
//  - The response drifts a few units over a long drive session; these
//    values are a good NORMAL-quality set, not a final calibration
//    (that is phase D, per board, temperature-banded).
//
// Level 0 is white (no train).

#pragma once
#include <stdint.h>

static const uint8_t TUNED16_LILYGO_T5S3_NORMAL[16][13] = {
  /* 0 (white) */ {0,0,0,0,0,0,0,0,0,0,0,0,0},
  /* 1  (2,2)  */ {1,1,2,2,0,0,0,0,0,0,0,0,0},
  /* 2  (2,1)  */ {1,1,2,0,0,0,0,0,0,0,0,0,0},
  /* 3  (3,2)  */ {1,1,1,2,2,0,0,0,0,0,0,0,0},
  /* 4 (3,3,1) */ {1,1,1,2,2,2,1,0,0,0,0,0,0},
  /* 5  (4,2)  */ {1,1,1,1,2,2,0,0,0,0,0,0,0},
  /* 6 (2,2,2) */ {1,1,2,2,1,1,0,0,0,0,0,0,0},
  /* 7 (2,1,2) */ {1,1,2,1,1,0,0,0,0,0,0,0,0},
  /* 8 (3,2,2) */ {1,1,1,2,2,1,1,0,0,0,0,0,0},
  /* 9  (5,1)  */ {1,1,1,1,1,2,0,0,0,0,0,0,0},
  /* 10(7,2,1) */ {1,1,1,1,1,1,1,2,2,1,0,0,0},
  /* 11 (7,1)  */ {1,1,1,1,1,1,1,2,0,0,0,0,0},
  /* 12(5,1,1) */ {1,1,1,1,1,2,1,0,0,0,0,0,0},
  /* 13 (11,1) */ {1,1,1,1,1,1,1,1,1,1,1,2,0},
  /* 14 (8,0)  */ {1,1,1,1,1,1,1,1,0,0,0,0,0},
  /* 15 (13,0) */ {1,1,1,1,1,1,1,1,1,1,1,1,1},
};

// Charge-matched remove trains (phase D). The DC constraint: under the
// constant pass period every pass carries equal dose, so a train's net
// charge is simply darkens - whitens. A paint+unpaint cycle returns the
// pixel's charge ledger to zero only if the remove's net whiten count
// equals the apply's net darken count — the old formula removes overdrove
// white by 3..8 passes per cycle (optically invisible, electrically not).
//
// Each remove must satisfy BOTH constraints: land optically white AND
// net whitens = the apply's net (+n column above). Scanner-tuned 21 July
// 2026 with the remove-ladder method (paint staircase, erase to white,
// scan residuals against a hard-clear reference). What the glass taught
// us: appended charge-neutral (1,2) scrub pairs erase NOTHING — the
// darken undoes its whiten's gain almost exactly. The shape that works
// is k darkens FIRST (deepening toward saturation, optically cheap),
// then all net+k whitens firing from a dark state where whitens are
// strong — the classic drive-to-opposite-rail activation, at zero extra
// net charge. All 15 levels land within 1.5 scan units of clear-white.
//
// Apply nets: L1:0 L2:+1 L3:+1 L4:+1 L5:+2 L6:+2 L7:+3 L8:+3 L9:+4
//             L10:+6 L11:+6 L12:+5 L13:+10 L14:+8 L15:+13
static const uint8_t TUNED16_LILYGO_T5S3_NORMAL_REMOVE[16][13] = {
  /* 0  net  0 */ {0,0,0,0,0,0,0,0,0,0,0,0,0},   // never removed
  /* 1  net  0 */ {1,2,1,2,0,0,0,0,0,0,0,0,0},   // scrub pairs (light grey)
  /* 2  net -1 */ {1,1,2,2,2,0,0,0,0,0,0,0,0},
  /* 3  net -1 */ {1,1,1,1,2,2,2,2,2,0,0,0,0},
  /* 4  net -1 */ {1,1,1,1,2,2,2,2,2,0,0,0,0},
  /* 5  net -2 */ {1,1,1,2,2,2,2,2,0,0,0,0,0},
  /* 6  net -2 */ {1,1,1,2,2,2,2,2,0,0,0,0,0},
  /* 7  net -3 */ {1,1,2,2,2,2,2,0,0,0,0,0,0},
  /* 8  net -3 */ {1,1,2,2,2,2,2,0,0,0,0,0,0},
  /* 9  net -4 */ {1,1,2,2,2,2,2,2,0,0,0,0,0},
  /* 10 net -6 */ {2,2,2,2,2,2,0,0,0,0,0,0,0},  // free budget suffices
  /* 11 net -6 */ {2,2,2,2,2,2,0,0,0,0,0,0,0},
  /* 12 net -5 */ {1,1,2,2,2,2,2,2,2,0,0,0,0},
  /* 13 net -10*/ {2,2,2,2,2,2,2,2,2,2,0,0,0},
  /* 14 net -8 */ {2,2,2,2,2,2,2,2,0,0,0,0,0},
  /* 15 net -13*/ {2,2,2,2,2,2,2,2,2,2,2,2,2},  // was already balanced
};

// Load the tuned trains into the driver: apply ids (level << 1) and
// charge-matched remove ids ((level << 1) | 1).
template <typename EPD>
static void loadTunedTrains(EPD &epd) {
  for (int g = 1; g <= 15; g++) {
    epd.setDecisionTrain((uint8_t)(g << 1), TUNED16_LILYGO_T5S3_NORMAL[g]);
    epd.setDecisionTrain((uint8_t)((g << 1) | 1),
                         TUNED16_LILYGO_T5S3_NORMAL_REMOVE[g]);
  }
}
