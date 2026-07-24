// Direct grey-to-grey transition trains — LilyGo T5 4.7" EPD47 H716.
// Scanner-tuned 22 Jul 2026 (transition card vs white-painted refs):
// NORMAL all within +-3.1, ghost gate 3.3; FAST within +-4.0, ghost
// gate 2.6 (20 mixed-path cycles, immediate scan).
//
// This glass at FAST: whitens are the strong pulses and darkens weak
// (leading-whiten + long darken-run shapes); at NORMAL, darkens after
// deep whitens ride full fresh-response boost, so lightening pairs
// split or advance their re-darkens to tame the landing.
//
// Potentials (net darkens of the calibrated apply tables):
// NORMAL Q=(3,6,13) -> nets +3,+10,+7,-3,-10,-7
// FAST   Q=(3,5,7)  -> nets +2,+4,+2,-2,-4,-2

#pragma once
#include <stdint.h>

static const uint8_t DIRECT_H716_NORMAL[4][4][26] = {
  { {0}, {0}, {0}, {0} },
  { /* from 1 */
    {0},
    {0},
    /* 1->2 net +3 (9) */ {2,2,2,1,1,1,1,1,1},
    /* 1->3 net +10 (14) */ {2,2,1,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 2 */
    {0},
    /* 2->1 net -3 (7) */ {2,2,2,2,1,2,1},
    {0},
    /* 2->3 net +7 (15) */ {2,2,2,2,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 3 */
    {0},
    /* 3->1 net -10 (16) */ {2,2,2,2,2,2,2,2,2,2,2,2,1,2,1,1},
    /* 3->2 net -7 (19) */ {2,2,2,2,2,2,2,2,2,2,2,2,2,1,1,1,1,1,1},
    {0},
  },
};

static const uint8_t DIRECT_H716_FAST[4][4][26] = {
  { {0}, {0}, {0}, {0} },
  { /* from 1 */
    {0},
    {0},
    /* 1->2 net +2 (10) */ {2,2,2,2,1,1,1,1,1,1},
    /* 1->3 net +4 (20) */ {2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 2 */
    {0},
    /* 2->1 net -2 (4) */ {2,2,2,1},
    {0},
    /* 2->3 net +2 (22) */ {2,2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 3 */
    {0},
    /* 3->1 net -4 (6) */ {2,2,2,2,1,2},
    /* 3->2 net -2 (8) */ {2,2,2,2,2,1,1,1},
    {0},
  },
};

template <typename EPD>
static void loadDirectTrainsH716(EPD &epd) {
  for (int f = 1; f <= 3; f++)
    for (int t = 1; t <= 3; t++)
      if (f != t) epd.setDirectTrain((uint8_t)f, (uint8_t)t,
                                     DIRECT_H716_NORMAL[f][t], 26);
}

template <typename EPD>
static void loadDirectTrainsH716Fast(EPD &epd) {
  for (int f = 1; f <= 3; f++)
    for (int t = 1; t <= 3; t++)
      if (f != t) epd.setDirectTrain((uint8_t)f, (uint8_t)t,
                                     DIRECT_H716_FAST[f][t], 26);
}
