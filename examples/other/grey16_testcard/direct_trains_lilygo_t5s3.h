// Direct grey-to-grey transition trains — LilyGo T5 S3 GPS.
// Scanner-tuned 21 July 2026, seeded from the M5PaperS3 finals (the
// panels share a physics family; four of six FAST trains transferred
// UNCHANGED). NORMAL all within +-4.0 of reference, FAST within +-3.3.
// This panel's re-darkens after whitening bite weaker than the
// M5Paper's, so most trains carry more charge-neutral drive mass. See
// DECISION_ENGINE.md and direct_trains_m5papers3.h for the method.
//
// NORMAL potentials (net darkens of the preset's apply tables):
// Q(1) = +4, Q(2) = +7, Q(3) = +13 -> nets +3, +9, +6, -3, -9, -6.
// FAST potentials are identical to the M5PaperS3's (+4, +5, +7), so the
// FAST seeds start as its finals unchanged.

#pragma once
#include <stdint.h>

static const uint8_t DIRECT_LILYGO_T5S3_NORMAL[4][4][26] = {
  { {0}, {0}, {0}, {0} },
  { /* from 1 */
    {0},
    {0},
    /* 1->2 net +3 (9)  */ {2,2,2,1,1,1,1,1,1},
    /* 1->3 net +9 (15) */ {2,2,2,1,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 2 */
    {0},
    /* 2->1 net -3 (7)  */ {2,2,2,2,2,1,1},
    {0},
    /* 2->3 net +6 (18) */ {2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 3 */
    {0},
    /* 3->1 net -9 (15) */ {2,2,2,2,2,2,2,2,2,2,2,2,1,1,1},
    /* 3->2 net -6 (18) */ {2,2,2,2,2,2,2,2,2,2,2,1,2,1,1,1,1,1},
    {0},
  },
};

static const uint8_t DIRECT_LILYGO_T5S3_FAST[4][4][26] = {
  { {0}, {0}, {0}, {0} },
  { /* from 1 */
    {0},
    {0},
    /* 1->2 net +1 (9)  */ {2,2,2,2,1,1,1,1,1},
    /* 1->3 net +3 (13) */ {2,2,2,2,2,1,1,1,1,1,1,1,1},
  },
  { /* from 2 */
    {0},
    /* 2->1 net -1 (1)  */ {2},
    {0},
    /* 2->3 net +2 (14) */ {2,2,2,2,2,2,1,1,1,1,1,1,1,1},
  },
  { /* from 3 */
    {0},
    /* 3->1 net -3 (5)  */ {2,2,2,2,1},
    /* 3->2 net -2 (8)  */ {2,2,2,2,2,1,1,1},
    {0},
  },
};

template <typename EPD>
static void loadDirectTrainsLilygo(EPD &epd) {
  for (int f = 1; f <= 3; f++)
    for (int t = 1; t <= 3; t++)
      if (f != t) epd.setDirectTrain((uint8_t)f, (uint8_t)t,
                                     DIRECT_LILYGO_T5S3_NORMAL[f][t], 26);
}

template <typename EPD>
static void loadDirectTrainsLilygoFast(EPD &epd) {
  for (int f = 1; f <= 3; f++)
    for (int t = 1; t <= 3; t++)
      if (f != t) epd.setDirectTrain((uint8_t)f, (uint8_t)t,
                                     DIRECT_LILYGO_T5S3_FAST[f][t], 26);
}
