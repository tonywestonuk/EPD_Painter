// Direct grey-to-grey transition trains — M5PaperS3, QUALITY_NORMAL.
// Scanner-tuned 21 July 2026 with the transition-card method (testcard
// 'D'): each pair painted from its FROM level in one paint and matched
// against a reference patch painted from white. All six landed within
// +3.5/-1.8 scan units of reference. See DECISION_ENGINE.md "direct
// grey-to-grey transitions".
//
// 2bpp levels: 1 = light grey, 2 = dark grey, 3 = black. Codes per pass:
// 0 float, 1 darken, 2 whiten.
//
// DC constraint (by construction, per train): net darkens = Q(to) - Q(from),
// with Q(g) = the calibrated apply train's net darkens from the preset's
// NORMAL tables — Q(1) = +4, Q(2) = +6, Q(3) = +13. This makes the charge
// ledger path-independent: white -> a -> b -> white nets zero on any route.
//
// Shape lessons the glass taught (mirrors of the 16-grey findings):
// - Trailing whitens poison a landing (each erases 20-30 units): darkening
//   directs put their charge-balancing whitens FIRST.
// - Darkens after deep whitening hit the fresh-response boost — 2>1 needs
//   a tiny darkens-first erase (1d,3w), not a whiten-then-redarken.
// - A darken followed by one whiten survives at ~1/3 strength: the
//   d,w,d,w,d tail on 3>1 is a fine-trim knob.
// - 3>2 cannot fit its -7 net AND its optical target in 13 passes (max
//   trailing darkens = 3 stops a full step short); its 17-pass train
//   (12 whitens, 5 fresh darkens) needs the pass-count extension —
//   frames only pay the extra passes when the train is in play.

#pragma once
#include <stdint.h>

// Indexed by [from][to], zero-padded to the 26-pass maximum; unused
// entries all-float (never loaded).
static const uint8_t DIRECT_M5PAPERS3_NORMAL[4][4][26] = {
  { {0}, {0}, {0}, {0} },
  { /* from 1 */
    {0},
    {0},
    /* 1->2 net +2 (13) */ {2,2,2,1,1,1,1,1},
    /* 1->3 net +9 (13) */ {2,2,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 2 */
    {0},
    /* 2->1 net -2 (4)  */ {1,2,2,2},
    {0},
    /* 2->3 net +7 (11) */ {2,2,1,1,1,1,1,1,1,1,1},
  },
  { /* from 3 */
    {0},
    /* 3->1 net -9 (15) */ {2,2,2,2,2,2,2,2,2,2,1,2,1,2,1},
    /* 3->2 net -7 (17) */ {2,2,2,2,2,2,2,2,2,2,2,2,1,1,1,1,1},
    {0},
  },
};

template <typename EPD>
static void loadDirectTrainsM5PaperS3(EPD &epd) {
  for (int f = 1; f <= 3; f++)
    for (int t = 1; t <= 3; t++)
      if (f != t) epd.setDirectTrain((uint8_t)f, (uint8_t)t,
                                     DIRECT_M5PAPERS3_NORMAL[f][t], 26);
}
