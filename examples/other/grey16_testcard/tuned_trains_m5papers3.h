// Scanner-tuned 16-grey trains — M5PaperS3, QUALITY_NORMAL (13 passes).
// Tuned 21 July 2026 with the match-card method under the constant pass
// period (15 ms NORMAL), seeded from the LilyGo T5 S3 GPS set — the two
// panels share the same glass physics family, and 10 of 15 levels passed
// unchanged on the first scan. Codes per pass: 0 float, 1 darken, 2 whiten.
//
// Divergences from the LilyGo that the glass insisted on: L1 is a single
// darken (this panel's first-pass response is stronger), L7 deepened to
// (3,3,2), L12 collapsed to five pure darkens, L14 gained a pass.
//
// Level 0 is white (no train).

#pragma once
#include <stdint.h>

static const uint8_t TUNED16_M5PAPERS3_NORMAL[16][13] = {
  /* 0 (white) */ {0,0,0,0,0,0,0,0,0,0,0,0,0},
  /* 1  (1,0)  */ {1,0,0,0,0,0,0,0,0,0,0,0,0},
  /* 2  (2,1)  */ {1,1,2,0,0,0,0,0,0,0,0,0,0},
  /* 3  (3,2)  */ {1,1,1,2,2,0,0,0,0,0,0,0,0},
  /* 4 (3,3,1) */ {1,1,1,2,2,2,1,0,0,0,0,0,0},
  /* 5  (4,2)  */ {1,1,1,1,2,2,0,0,0,0,0,0,0},
  /* 6 (2,2,2) */ {1,1,2,2,1,1,0,0,0,0,0,0,0},
  /* 7 (3,3,2) */ {1,1,1,2,2,2,1,1,0,0,0,0,0},
  /* 8 (3,2,2) */ {1,1,1,2,2,1,1,0,0,0,0,0,0},
  /* 9  (5,1)  */ {1,1,1,1,1,2,0,0,0,0,0,0,0},
  /* 10(7,2,1) */ {1,1,1,1,1,1,1,2,2,1,0,0,0},
  /* 11 (7,1)  */ {1,1,1,1,1,1,1,2,0,0,0,0,0},
  /* 12 (5,0)  */ {1,1,1,1,1,0,0,0,0,0,0,0,0},
  /* 13 (11,1) */ {1,1,1,1,1,1,1,1,1,1,1,2,0},
  /* 14 (9,0)  */ {1,1,1,1,1,1,1,1,1,0,0,0,0},
  /* 15 (13,0) */ {1,1,1,1,1,1,1,1,1,1,1,1,1},
};

// Charge-matched remove trains (see the LilyGo header for the method and
// the darkens-first shape rationale). Net whitens per level equal the
// apply's net darkens: L1:1 L2:1 L3:1 L4:1 L5:2 L6:2 L7:2 L8:3 L9:4
// L10:6 L11:6 L12:5 L13:10 L14:9 L15:13.
static const uint8_t TUNED16_M5PAPERS3_NORMAL_REMOVE[16][13] = {
  /* 0  */ {0,0,0,0,0,0,0,0,0,0,0,0,0},   // never removed
  /* 1  */ {2,0,0,0,0,0,0,0,0,0,0,0,0},
  /* 2  */ {1,1,2,2,2,0,0,0,0,0,0,0,0},
  /* 3  */ {1,1,1,1,2,2,2,2,2,0,0,0,0},
  /* 4  */ {1,1,1,1,1,2,2,2,2,2,2,0,0},
  /* 5  */ {1,1,1,2,2,2,2,2,0,0,0,0,0},
  /* 6  */ {1,1,1,1,2,2,2,2,2,2,0,0,0},
  /* 7  */ {1,1,1,1,2,2,2,2,2,2,0,0,0},
  /* 8  */ {1,1,1,2,2,2,2,2,2,0,0,0,0},
  /* 9  */ {1,1,2,2,2,2,2,2,0,0,0,0,0},
  /* 10 */ {2,2,2,2,2,2,0,0,0,0,0,0,0},
  /* 11 */ {2,2,2,2,2,2,0,0,0,0,0,0,0},
  /* 12 */ {1,1,2,2,2,2,2,2,2,0,0,0,0},
  /* 13 */ {2,2,2,2,2,2,2,2,2,2,0,0,0},
  /* 14 */ {2,2,2,2,2,2,2,2,2,0,0,0,0},
  /* 15 */ {2,2,2,2,2,2,2,2,2,2,2,2,2},
};

// Load the tuned trains: apply ids (level << 1), remove ids ((level<<1)|1).
template <typename EPD>
static void loadTunedTrainsM5PaperS3(EPD &epd) {
  for (int g = 1; g <= 15; g++) {
    epd.setDecisionTrain((uint8_t)(g << 1), TUNED16_M5PAPERS3_NORMAL[g]);
    epd.setDecisionTrain((uint8_t)((g << 1) | 1),
                         TUNED16_M5PAPERS3_NORMAL_REMOVE[g]);
  }
}
