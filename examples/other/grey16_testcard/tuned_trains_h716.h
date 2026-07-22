// Scanner-tuned 16-grey trains — LilyGo T5 4.7" EPD47 H716, QUALITY_NORMAL,
// 20 ms pass period (g16_pass_us_normal = 20000), tuned 22 Jul 2026 with
// the match-card method, all levels within ~±2.5 scan units.
//
// Board-specific physics (vs M5PaperS3/LilyGo GPS lessons):
//  - whiten quantum ~20 units; fine steps come from darkens UNDER the
//    trailing whitens: ~-5/darken at w=2, ~-10/darken at w=1.
//  - a darken AFTER the whitens rides fresh response at ~-28 (full
//    strength, NOT the 1/3 trim of the M5PaperS3); second one ~-26
//    after deep whitens, ~-8 after shallow.
//
// REMOVE trains scanner-tuned same session (white-landing residuals
// within ~±1.5, L15 lands fresh-white +6.6 which decays). Shape lesson:
// '12' scrub pairs are NOT optically neutral here — each pair nets ~-8
// darker (boosted darken vs ~+20 whiten), so removes use the dw shape:
// k darkens first (saturating regime, cheap), then net+k whitens firing
// together from the dark state.

#pragma once
#include <stdint.h>

static const uint8_t TUNED16_H716_NORMAL[16][13] = {
  /*  0 (white) */ {0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  1 */ {1,1,2,2,0,0,0,0,0,0,0,0,0},  // err -0.2
  /*  2 */ {1,0,0,0,0,0,0,0,0,0,0,0,0},  // err -1.1
  /*  3 */ {1,1,1,2,2,0,0,0,0,0,0,0,0},  // err +0.1
  /*  4 */ {1,1,2,0,0,0,0,0,0,0,0,0,0},  // err +1.0
  /*  5 */ {1,1,1,1,1,2,2,0,0,0,0,0,0},  // err +1.5
  /*  6 */ {1,1,1,1,1,1,1,2,2,0,0,0,0},  // err +1.6
  /*  7 */ {1,1,1,1,1,1,1,1,1,1,2,2,0},  // err +0.2
  /*  8 */ {1,1,1,1,1,2,2,1,0,0,0,0,0},  // err -2.4
  /*  9 */ {1,1,1,1,1,2,0,0,0,0,0,0,0},  // err -0.1
  /* 10 */ {1,1,1,1,1,1,2,0,0,0,0,0,0},  // err +1.6
  /* 11 */ {1,1,1,1,1,1,1,1,2,0,0,0,0},  // err +0.6
  /* 12 */ {1,1,1,1,0,0,0,0,0,0,0,0,0},  // err -1.4
  /* 13 */ {1,1,1,1,1,0,0,0,0,0,0,0,0},  // err -1.1
  /* 14 */ {1,1,1,1,1,1,0,0,0,0,0,0,0},  // err +1.1
  /* 15 */ {1,1,1,1,1,1,1,1,1,1,1,1,1},
};

// net charge per level (darkens - whitens) — the DC constraint removes honour
// L1:0, L2:1, L3:1, L4:1, L5:3, L6:5, L7:8, L8:4, L9:4, L10:5, L11:7, L12:4, L13:5, L14:6, L15:13
static const uint8_t TUNED16_H716_NORMAL_REMOVE[16][13] = {
  /*  0 (white) */ {0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  1 */ {1,1,1,1,1,2,2,2,2,2,0,0,0},
  /*  2 */ {1,1,1,1,2,2,2,2,2,0,0,0,0},
  /*  3 */ {1,1,1,1,2,2,2,2,2,0,0,0,0},
  /*  4 */ {1,1,1,2,2,2,2,0,0,0,0,0,0},
  /*  5 */ {1,1,2,2,2,2,2,0,0,0,0,0,0},
  /*  6 */ {1,1,2,2,2,2,2,2,2,0,0,0,0},
  /*  7 */ {2,2,2,2,2,2,2,2,1,2,0,0,0},
  /*  8 */ {1,1,2,2,2,2,2,2,0,0,0,0,0},
  /*  9 */ {1,1,2,2,2,2,2,2,0,0,0,0,0},
  /* 10 */ {1,1,2,2,2,2,2,2,2,0,0,0,0},
  /* 11 */ {2,2,2,2,2,2,2,0,0,0,0,0,0},
  /* 12 */ {1,1,2,2,2,2,2,2,0,0,0,0,0},
  /* 13 */ {1,1,2,2,2,2,2,2,2,0,0,0,0},
  /* 14 */ {2,2,2,2,2,2,0,0,0,0,0,0,0},
  /* 15 */ {2,2,2,2,2,2,2,2,2,2,2,2,2},
};

// Load the tuned trains into the driver: apply ids (level << 1) and
// charge-matched remove ids ((level << 1) | 1).
template <typename EPD>
static void loadTunedTrainsH716(EPD &epd) {
  for (int g = 1; g <= 15; g++) {
    epd.setDecisionTrain((uint8_t)(g << 1), TUNED16_H716_NORMAL[g]);
    epd.setDecisionTrain((uint8_t)((g << 1) | 1),
                         TUNED16_H716_NORMAL_REMOVE[g]);
  }
}
