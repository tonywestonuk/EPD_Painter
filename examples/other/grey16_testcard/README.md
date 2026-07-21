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

The sketch loads the scanner-tuned LilyGo T5 S3 GPS NORMAL train set
(`tuned_trains_lilygo_t5s3.h`): apply trains matched against Bayer-dithered
black/white references, and charge-matched remove trains (unpainting a
level returns its net DC charge to zero — see the phase D notes in
`DECISION_ENGINE.md`). On other boards the levels will be in order but not
colour-accurate until the match-card loop is run for that panel.

Quality must be `QUALITY_NORMAL` or `QUALITY_HIGH`; 16-grey mode refuses
`QUALITY_FAST` (7 undelayed passes cannot resolve 16 doses).

## Serial commands

| Key | Action |
|-----|--------|
| `b` | staircase, white → black |
| `w` | quantised wedge |
| `i` | staircase, black → white |
| `c` | hard clear |
| `p` | pure-darken dose probe (raw response ladder) |
| `m` | match card: dithered references above, native levels below |
| `t <id> <13 codes>` | upload one train (codes 0–3 per pass) |
| `x <frames>` | perf test: 16 bouncing patches, one per level, reports fps |
| `g <level> <cycles>` | DC ghost test: cycle paint/erase on the left half |
| `u <level>` | uniform full-screen level (probe grey / erase) |
| `F` / `M` | load formula removes (unbalanced control) / tuned trains |
| `H` / `N` | quality high / normal |
