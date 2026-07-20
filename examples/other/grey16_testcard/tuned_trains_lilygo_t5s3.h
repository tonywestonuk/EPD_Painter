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
// Level 0 is white (no train). Remove trains stay on the library default.

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

// Load the tuned apply trains (id = level << 1) into the driver.
template <typename EPD>
static void loadTunedTrains(EPD &epd) {
  for (int g = 1; g <= 15; g++)
    epd.setDecisionTrain((uint8_t)(g << 1), TUNED16_LILYGO_T5S3_NORMAL[g]);
}
