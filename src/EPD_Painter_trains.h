// Scanner-tuned decision-engine train tables, per board (phase D — see
// DECISION_ENGINE.md). Each calibrated preset in EPD_Painter_presets.h
// points its Config::trains at the tables below; boards without a tuned
// set leave the pointers null and fall back to the formula library
// (16-grey) and the two-step transition path (direct).
//
// 16-grey tables: [16][13] drive codes per level (0 float, 1 darken,
// 2 whiten), apply + charge-matched remove, indexed by level 0..15
// (level 0 = white, never driven). Tuned with the match-card method:
// each level's flat native patch optically matched (flatbed scan)
// against a Bayer-dithered black/white reference of density g/15, under
// the preset's constant 16-grey pass period — the tables are only valid
// AT that period (Config::g16_pass_us_normal).
//
// Direct tables: [from][to][26] trains for the 4-level direct
// grey-to-grey engine, NORMAL and FAST sets. DC constraint by
// construction: net darkens = Q(to) - Q(from) with Q(g) the net darkens
// of the quality's apply train, making the charge ledger
// path-independent on any route.

#ifndef EPD_PAINTER_TRAINS_H
#define EPD_PAINTER_TRAINS_H

#include <stdint.h>

// ===========================================================================
// M5PaperS3
// ===========================================================================
#if defined(EPD_PAINTER_PRESET_M5PAPER_S3) || defined(EPD_PAINTER_PRESET_AUTO)

// Tuned 21 July 2026, seeded from the LilyGo T5 S3 GPS set — the two
// panels share the same glass physics family, and 10 of 15 levels passed
// unchanged on the first scan. Divergences the glass insisted on: L1 is
// a single darken (this panel's first-pass response is stronger), L7
// deepened to (3,3,2), L12 collapsed to five pure darkens, L14 gained a
// pass.
inline const uint8_t TUNED16_M5PAPERS3_NORMAL[16][13] = {
  /*  0 (white) */ {0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  1  (1,0)  */ {1,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  2  (2,1)  */ {1,1,2,0,0,0,0,0,0,0,0,0,0},
  /*  3  (3,2)  */ {1,1,1,2,2,0,0,0,0,0,0,0,0},
  /*  4 (3,3,1) */ {1,1,1,2,2,2,1,0,0,0,0,0,0},
  /*  5  (4,2)  */ {1,1,1,1,2,2,0,0,0,0,0,0,0},
  /*  6 (2,2,2) */ {1,1,2,2,1,1,0,0,0,0,0,0,0},
  /*  7 (3,3,2) */ {1,1,1,2,2,2,1,1,0,0,0,0,0},
  /*  8 (3,2,2) */ {1,1,1,2,2,1,1,0,0,0,0,0,0},
  /*  9  (5,1)  */ {1,1,1,1,1,2,0,0,0,0,0,0,0},
  /* 10 (7,2,1) */ {1,1,1,1,1,1,1,2,2,1,0,0,0},
  /* 11  (7,1)  */ {1,1,1,1,1,1,1,2,0,0,0,0,0},
  /* 12  (5,0)  */ {1,1,1,1,1,0,0,0,0,0,0,0,0},
  /* 13 (11,1)  */ {1,1,1,1,1,1,1,1,1,1,1,2,0},
  /* 14  (9,0)  */ {1,1,1,1,1,1,1,1,1,0,0,0,0},
  /* 15 (13,0)  */ {1,1,1,1,1,1,1,1,1,1,1,1,1},
};

// Charge-matched removes (see the LilyGo tables for the method and the
// darkens-first shape rationale). Net whitens per level equal the
// apply's net darkens: L1:1 L2:1 L3:1 L4:1 L5:2 L6:2 L7:2 L8:3 L9:4
// L10:6 L11:6 L12:5 L13:10 L14:9 L15:13.
inline const uint8_t TUNED16_M5PAPERS3_NORMAL_REMOVE[16][13] = {
  /*  0 */ {0,0,0,0,0,0,0,0,0,0,0,0,0},   // never removed
  /*  1 */ {2,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  2 */ {1,1,2,2,2,0,0,0,0,0,0,0,0},
  /*  3 */ {1,1,1,1,2,2,2,2,2,0,0,0,0},
  /*  4 */ {1,1,1,1,1,2,2,2,2,2,2,0,0},
  /*  5 */ {1,1,1,2,2,2,2,2,0,0,0,0,0},
  /*  6 */ {1,1,1,1,2,2,2,2,2,2,0,0,0},
  /*  7 */ {1,1,1,1,2,2,2,2,2,2,0,0,0},
  /*  8 */ {1,1,1,2,2,2,2,2,2,0,0,0,0},
  /*  9 */ {1,1,2,2,2,2,2,2,0,0,0,0,0},
  /* 10 */ {2,2,2,2,2,2,0,0,0,0,0,0,0},
  /* 11 */ {2,2,2,2,2,2,0,0,0,0,0,0,0},
  /* 12 */ {1,1,2,2,2,2,2,2,2,0,0,0,0},
  /* 13 */ {2,2,2,2,2,2,2,2,2,2,0,0,0},
  /* 14 */ {2,2,2,2,2,2,2,2,2,0,0,0,0},
  /* 15 */ {2,2,2,2,2,2,2,2,2,2,2,2,2},
};

// Direct trains, tuned 21 July 2026 with the transition-card method
// (testcard 'D'): each pair painted from its FROM level in one paint and
// matched against a reference patch painted from white; all six landed
// within +3.5/-1.8 scan units. Shape lessons: trailing whitens poison a
// landing (darkening directs put their charge-balancing whitens FIRST);
// darkens after deep whitening hit the fresh-response boost; a d,w pair
// survives at ~1/3 strength (the d,w,d,w,d tail on 3>1 is a fine-trim
// knob). 3>2 cannot fit its -7 net AND its optical target in 13 passes;
// its 17-pass train needs the pass-count extension — frames only pay the
// extra passes when the train is in play.
// NORMAL potentials Q=(4,6,13).
inline const uint8_t DIRECT_M5PAPERS3_NORMAL[4][4][26] = {
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

// FAST set (tuned 21 July 2026, same card method, all six pairs within
// ±3.0). FAST potentials Q=(4,5,7), so the nets are small — but FAST
// darken passes are weak (no inter-pass delay), so the darkening directs
// need leading whitens plus a LONG charge-neutral darken run to actually
// move the glass. Five of six trains run past FAST's 7 passes (extension
// passes are nearly free undelayed). The lightening directs needed
// almost nothing: whitens bite hard even short.
inline const uint8_t DIRECT_M5PAPERS3_FAST[4][4][26] = {
  { {0}, {0}, {0}, {0} },
  { /* from 1 */
    {0},
    {0},
    /* 1->2 net +1 (9)  */ {2,2,2,2,1,1,1,1,1},
    /* 1->3 net +3 (9)  */ {2,2,2,1,1,1,1,1,1},
  },
  { /* from 2 */
    {0},
    /* 2->1 net -1 (1)  */ {2},
    {0},
    /* 2->3 net +2 (10) */ {2,2,2,2,1,1,1,1,1,1},
  },
  { /* from 3 */
    {0},
    /* 3->1 net -3 (5)  */ {2,2,2,2,1},
    /* 3->2 net -2 (8)  */ {2,2,2,2,2,1,1,1},
    {0},
  },
};

#endif // M5PaperS3

// ===========================================================================
// LilyGo T5 S3 GPS
// ===========================================================================
#if defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS) || defined(EPD_PAINTER_PRESET_AUTO)

// Tuned 20 July 2026 UNDER THE CONSTANT PASS PERIOD (15 ms NORMAL) and
// cross-verified: strictly monotonic on both the match card (max err 4.2
// scan units) and the full-height staircase. What the glass taught us
// (vs the formula library's assumptions):
//  - A whiten pass is STRONG: after a short darken run one whiten takes
//    back 20-30 units, near saturation ~10 — never a "half step".
//  - The dose response saturates hard: darken passes 8..13 buy ~3 units
//    total. Deep greys cannot be spaced by run length alone.
//  - Fine steps come from RE-DARKENING after a whiten: patterns like
//    1,1,2,1,1 climb from the lifted grey in small fresh-response steps.
inline const uint8_t TUNED16_LILYGO_T5S3_NORMAL[16][13] = {
  /*  0 (white) */ {0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  1  (2,2)  */ {1,1,2,2,0,0,0,0,0,0,0,0,0},
  /*  2  (2,1)  */ {1,1,2,0,0,0,0,0,0,0,0,0,0},
  /*  3  (3,2)  */ {1,1,1,2,2,0,0,0,0,0,0,0,0},
  /*  4 (3,3,1) */ {1,1,1,2,2,2,1,0,0,0,0,0,0},
  /*  5  (4,2)  */ {1,1,1,1,2,2,0,0,0,0,0,0,0},
  /*  6 (2,2,2) */ {1,1,2,2,1,1,0,0,0,0,0,0,0},
  /*  7 (2,1,2) */ {1,1,2,1,1,0,0,0,0,0,0,0,0},
  /*  8 (3,2,2) */ {1,1,1,2,2,1,1,0,0,0,0,0,0},
  /*  9  (5,1)  */ {1,1,1,1,1,2,0,0,0,0,0,0,0},
  /* 10 (7,2,1) */ {1,1,1,1,1,1,1,2,2,1,0,0,0},
  /* 11  (7,1)  */ {1,1,1,1,1,1,1,2,0,0,0,0,0},
  /* 12 (5,1,1) */ {1,1,1,1,1,2,1,0,0,0,0,0,0},
  /* 13 (11,1)  */ {1,1,1,1,1,1,1,1,1,1,1,2,0},
  /* 14  (8,0)  */ {1,1,1,1,1,1,1,1,0,0,0,0,0},
  /* 15 (13,0)  */ {1,1,1,1,1,1,1,1,1,1,1,1,1},
};

// Charge-matched removes (phase D). The DC constraint: under the
// constant pass period every pass carries equal dose, so a train's net
// charge is simply darkens - whitens; a paint+unpaint cycle returns the
// pixel's ledger to zero only if the remove's net whitens equal the
// apply's net darkens. Scanner-tuned 21 July 2026 with the remove-ladder
// method. What the glass taught us: appended charge-neutral (1,2) scrub
// pairs erase NOTHING — the shape that works is k darkens FIRST
// (deepening toward saturation, optically cheap), then all net+k whitens
// firing from a dark state where whitens are strong. All 15 levels land
// within 1.5 scan units of clear-white.
// Apply nets: L1:0 L2:+1 L3:+1 L4:+1 L5:+2 L6:+2 L7:+3 L8:+3 L9:+4
//             L10:+6 L11:+6 L12:+5 L13:+10 L14:+8 L15:+13
inline const uint8_t TUNED16_LILYGO_T5S3_NORMAL_REMOVE[16][13] = {
  /*  0  net  0 */ {0,0,0,0,0,0,0,0,0,0,0,0,0},   // never removed
  /*  1  net  0 */ {1,2,1,2,0,0,0,0,0,0,0,0,0},   // scrub pairs (light grey)
  /*  2  net -1 */ {1,1,2,2,2,0,0,0,0,0,0,0,0},
  /*  3  net -1 */ {1,1,1,1,2,2,2,2,2,0,0,0,0},
  /*  4  net -1 */ {1,1,1,1,2,2,2,2,2,0,0,0,0},
  /*  5  net -2 */ {1,1,1,2,2,2,2,2,0,0,0,0,0},
  /*  6  net -2 */ {1,1,1,2,2,2,2,2,0,0,0,0,0},
  /*  7  net -3 */ {1,1,2,2,2,2,2,0,0,0,0,0,0},
  /*  8  net -3 */ {1,1,2,2,2,2,2,0,0,0,0,0,0},
  /*  9  net -4 */ {1,1,2,2,2,2,2,2,0,0,0,0,0},
  /* 10  net -6 */ {2,2,2,2,2,2,0,0,0,0,0,0,0},  // free budget suffices
  /* 11  net -6 */ {2,2,2,2,2,2,0,0,0,0,0,0,0},
  /* 12  net -5 */ {1,1,2,2,2,2,2,2,2,0,0,0,0},
  /* 13  net -10*/ {2,2,2,2,2,2,2,2,2,2,0,0,0},
  /* 14  net -8 */ {2,2,2,2,2,2,2,2,0,0,0,0,0},
  /* 15  net -13*/ {2,2,2,2,2,2,2,2,2,2,2,2,2},  // was already balanced
};

// Direct trains, tuned 21 July 2026, seeded from the M5PaperS3 finals
// (the panels share a physics family; four of six FAST trains
// transferred UNCHANGED). NORMAL all within ±4.0 of reference, FAST
// within ±3.3. This panel's re-darkens after whitening bite weaker than
// the M5Paper's, so most trains carry more charge-neutral drive mass.
// NORMAL potentials Q=(4,7,13); FAST potentials identical to the
// M5PaperS3's (4,5,7).
inline const uint8_t DIRECT_LILYGO_T5S3_NORMAL[4][4][26] = {
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

inline const uint8_t DIRECT_LILYGO_T5S3_FAST[4][4][26] = {
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

#endif // LilyGo T5 S3 GPS

// ===========================================================================
// LilyGo T5 4.7" EPD47 H716
// ===========================================================================
#if defined(EPD_PAINTER_PRESET_LILYGO_EPD47_H716) || defined(EPD_PAINTER_PRESET_AUTO)

// Tuned 22 July 2026 at the preset's 20 ms NORMAL pass period, all
// levels within ~±2.5 scan units. Board-specific physics (vs the
// M5PaperS3/LilyGo GPS lessons):
//  - whiten quantum ~20 units; fine steps come from darkens UNDER the
//    trailing whitens: ~-5/darken at w=2, ~-10/darken at w=1.
//  - a darken AFTER the whitens rides fresh response at ~-28 (full
//    strength, NOT the 1/3 trim of the M5PaperS3); second one ~-26
//    after deep whitens, ~-8 after shallow.
inline const uint8_t TUNED16_H716_NORMAL[16][13] = {
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

// Removes scanner-tuned same session (white-landing residuals within
// ~±1.5, L15 lands fresh-white +6.6 which decays). Shape lesson: '12'
// scrub pairs are NOT optically neutral here — each pair nets ~-8 darker
// (boosted darken vs ~+20 whiten), so removes use the dw shape: k
// darkens first (saturating regime, cheap), then net+k whitens firing
// together from the dark state.
// Apply nets: L1:0 L2:1 L3:1 L4:1 L5:3 L6:5 L7:8 L8:4 L9:4 L10:5 L11:7
//             L12:4 L13:5 L14:6 L15:13
inline const uint8_t TUNED16_H716_NORMAL_REMOVE[16][13] = {
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

// Direct trains, scanner-tuned 22 July 2026 (transition card vs
// white-painted refs): NORMAL all within ±3.1, ghost gate 3.3; FAST
// within ±4.0, ghost gate 2.6 (20 mixed-path cycles, immediate scan).
// This glass at FAST: whitens are the strong pulses and darkens weak
// (leading-whiten + long darken-run shapes); at NORMAL, darkens after
// deep whitens ride full fresh-response boost, so lightening pairs split
// or advance their re-darkens to tame the landing.
// NORMAL potentials Q=(3,6,13); FAST potentials Q=(3,5,7).
inline const uint8_t DIRECT_H716_NORMAL[4][4][26] = {
  { {0}, {0}, {0}, {0} },
  { /* from 1 */
    {0},
    {0},
    /* 1->2 net +3 (9)  */ {2,2,2,1,1,1,1,1,1},
    /* 1->3 net +10 (14) */ {2,2,1,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 2 */
    {0},
    /* 2->1 net -3 (7)  */ {2,2,2,2,1,2,1},
    {0},
    /* 2->3 net +7 (15) */ {2,2,2,2,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 3 */
    {0},
    /* 3->1 net -10 (16) */ {2,2,2,2,2,2,2,2,2,2,2,2,1,2,1,1},
    /* 3->2 net -7 (19)  */ {2,2,2,2,2,2,2,2,2,2,2,2,2,1,1,1,1,1,1},
    {0},
  },
};

inline const uint8_t DIRECT_H716_FAST[4][4][26] = {
  { {0}, {0}, {0}, {0} },
  { /* from 1 */
    {0},
    {0},
    /* 1->2 net +2 (10) */ {2,2,2,2,1,1,1,1,1,1},
    /* 1->3 net +4 (20) */ {2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 2 */
    {0},
    /* 2->1 net -2 (4)  */ {2,2,2,1},
    {0},
    /* 2->3 net +2 (22) */ {2,2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 3 */
    {0},
    /* 3->1 net -4 (6)  */ {2,2,2,2,1,2},
    /* 3->2 net -2 (8)  */ {2,2,2,2,2,1,1,1},
    {0},
  },
};

#endif // H716

#endif // EPD_PAINTER_TRAINS_H
