# 16-Grey Test Card

The decision engine's phase C gate (see `DECISION_ENGINE.md` in the repo
root): 16 native grey levels on one screen, driven in a single paint cycle
by batching per-line decisions into OR-merged sweeps.

## What it does

- `setGreyLevels(16)` switches the driver to 4bpp state; `paint()` then
  reads the 8bpp canvas as level codes 0..15 (0 = white … 15 = black).
- The staircase paints 16 flat vertical bars — no dithering anywhere; every
  bar is one native level. Tick notches mark the bar boundaries so
  neighbouring levels can be compared straight across the line.
- The wedge is a continuous ramp quantised to nearest level: band edges
  should appear in strictly monotonic order with no reversals.
- The inverse staircase repaints every bar to a different grey, exercising
  the remove-decision paths (grey-to-grey runs as erase-then-redraw across
  the example's two paint() calls).

## What to expect

Phase C ships a *formula* train library — levels should be distinguishable
and monotonic, but not evenly spaced or colour-accurate. Even spacing comes
from the flatbed-scanner calibration in phase D.

Quality must be `QUALITY_NORMAL` or `QUALITY_HIGH`; 16-grey mode refuses
`QUALITY_FAST` (7 undelayed passes cannot resolve 16 doses).

## Serial commands

| Key | Action |
|-----|--------|
| `b` | staircase, white → black |
| `w` | quantised wedge |
| `i` | staircase, black → white |
| `c` | hard clear |
