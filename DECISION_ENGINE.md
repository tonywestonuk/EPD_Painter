# The Decision Engine

Design note, July 2026. Origin: Tony's generalization of the dual-plane
OR-merge — "if we can OR together the two phases we had before, why not OR
together every waveform period needed to build 16 colours, on one line?"

## The idea in one paragraph

The engine's two "phases" (lighten and darken) are not physics; they are
slot assignment. A convert call broadcasts a 4-entry waveform table into
vector registers and translates a staged 2-bit plane into drive codes; the
2-bit values are just indices into whatever table is loaded. Today those
tables are statically bound — 4 draw-to trajectories in one call, 4
erase-from in the other. Unbind them: let a line's 2-bit values index
*decisions* — (grey level, direction) pairs chosen from a larger calibrated
library — and OR as many convert sweeps as the line's content requires into
the row accumulator before its single latch. Two-colour lines collapse to
one sweep (faster than today); today's 4-level content needs two (no
regression); lines using many of 16 grey levels take more sweeps, each
buying levels the current engine cannot express at all.

## Vocabulary

- **Decision**: one (grey level, direction) pair — "apply light grey",
  "remove dark grey". With 16 levels there are 32 decisions. Each decision
  owns a calibrated **train**: an array of 2-bit drive codes, one per
  temporal pass (0 float, 1 darken, 2 whiten, 3 both). Trains are
  mixed-polarity capable — and the *existing* calibrated tables already are
  (e.g. M5PaperS3 `fast_lighter[0] = {1,3,2,3,2,2,3}`), so the model is a
  widening of current practice, not a departure.
- **Todo word**: 32-bit word per line, one bit per decision, built by the
  discovery pass. Non-zero = line has work; drained at staging.
- **Sweep**: one convert call — a staged 2-bit **slot plane** (+ per-row
  chunk mask) whose values 1..3 name up to 3 decisions, plus those
  decisions' trains. Slot 0 is reserved for float (it is what keeps the
  OR-merge neutral), so a sweep carries 3 decisions, and a line needs
  ceil(decisions / 3) sweeps.

## The paint cycle

1. **Discovery** (fused with today's delta pass; PSRAM read once):
   compare the new frame against the screen state per pixel, derive each
   changed pixel's decision, set its bit in the line's todo word, and write
   slot planes: while the todo word drains, batches of ≤3 decisions get a
   plane in which member pixels hold their slot index and all others hold
   0. Plane 1 of every line lives in internal SRAM (today's fastbuffer,
   unchanged); overflow planes spill to PSRAM (today's lightbuffer
   arrangement, generalized from "exactly one" to "as many as needed").
2. **Temporal passes** (structure unchanged): per pass, per line — zero
   the row accumulator, run the line's k sweeps (first convert overwrites,
   the rest OR), send once. One latch per line per pass; the batching
   never touches the glass, so the charge-retention rule (dose = time
   until the row's next latch) is honoured exactly as today.
   Pass count = length of the longest train in play.

## Why the k-way OR is safe

Each pixel has exactly one decision per paint, so across a line's sweeps
exactly one writes ink for any pixel and the rest write float — the same
pixel-disjointness argument that justified the two-plane merge in July,
stated generally. Per pass the wire never carries anything richer than
2-bit codes; more greys differ only in *which passes* say what.

## Cost model (per line, per pass)

| Content on the line     | Decisions | Sweeps | vs today |
|-------------------------|-----------|--------|----------|
| Nothing changed         | 0         | 0      | same (mask skip) |
| Black-and-white         | ≤3        | 1      | **half** |
| Today's 4-level video   | ≤6        | 2      | same     |
| Rich 16-grey gradient   | up to 32  | ≤11    | new capability |

Cost scales with the decisions actually present per line, not with the
palette. The 8bpp source is read from PSRAM exactly once per paint (at
discovery), as now; per-pass traffic is staged 2bpp planes, as now.

## Memory

- Slot planes: 130 KB internal (plane 1, = today's fastbuffer) + spill
  planes in PSRAM (240 B x lines that need them x depth; worst case all
  lines x 11 planes ~ 1.4 MB PSRAM).
- Todo words: 540 x 4 B ~ 2 KB.
- Screenbuffer: widens 2bpp -> 4bpp to remember 16 levels (~260 KB, PSRAM).
- Waveform library: 32 trains x pass count bytes per quality mode - tiny.

## What stays untouched

The SIMD convert kernels (the 2-bit table lookup *is* the sweep kernel),
the row accumulator + OR + single-latch structure, sendRow timing, DMA,
LCD_CAM, the delta principle, per-row broadcast of tables (already
per-call, so per-line tables cost nothing).

## Scope: where the wins land

- **Video (paintPacked)**: frames arrive pre-packed 2bpp, quantised at
  encode time, dithered across all 4 levels — up to 6 decisions/line, 2
  sweeps, today's price. The player does not get faster; it is already at
  the architecture's happy point.
- **True 1-bit content** (e.g. the Mac emulator): 2 decisions, 1 sweep —
  a real but modest speedup on the conversion half of the frame.
- **16 greys**: NORMAL and HIGH only. FAST's 7 undelayed passes lack the
  dose resolution for 16 honest levels regardless of batching — the last
  stretch past ~10 levels needs mixed-polarity trains plus inter-pass
  settling time, which is part of the dose (see the EPD_Painter2
  measurements). FAST remains the 4-level speed mode. Expected 16-grey
  NORMAL full-screen paint: roughly the 100-150 ms class.

## Roadmap

- **Phase A — plumbing** (this branch): pass loop iterates a generic sweep
  list instead of the hardcoded dark+light pair; list built to reproduce
  today's engine byte-for-byte. Gate: Bad Apple 20.0 fps + visuals
  identical on both boards.
- **Phase B — discovery**: C implementation of discovery/batcher emitting
  todo words + slot planes; compatibility batcher reproduces today's two
  planes for 4-level content. Gate: same as A, via the new path.
- **Phase C — 16 levels**: 4bpp screenbuffer, 16-level quantize of the
  8bpp canvas, k > 2 spill planes, decision-indexed train library
  (placeholder trains interpolated from the 4-level tables). Gate: 16
  distinguishable greys on a test card, mechanism correct, colours
  approximate.

  **DONE, gate passed 20 July 2026 (LilyGo T5 S3 GPS, NORMAL).** setGreyLevels(16)
  + C discovery with a greedy first-appearance batcher (<=3 decisions per
  sweep, up to 10 sweeps/line; sweep 0 = fastbuffer, 1 = lightbuffer, 2..9
  spill to PSRAM). The formula train library flunked its first scan (dark
  half non-monotonic), so phase D's match-card method was pulled forward:
  reference patches of pure black/white pixels Bayer-dithered to density
  g/15 on the top half of the glass, native levels below, closed-loop tuned
  over serial (setDecisionTrain) against flatbed scans. Final card: all 16
  levels strictly monotonic, every native within +/-4.5 scan units of its
  reference. Tuned set in examples/other/grey16_testcard/.

  What the glass taught us: a whiten pass takes back 20-30 units after a
  short darken run (~10 near saturation) — never the formula's assumed half
  step; darken passes 8..13 buy ~3 units total (saturation), so deep greys
  cannot be spaced by run length; the fine knob is re-darkening after a
  whiten (1,1,2,1,1-style patterns), which climbs from the lifted grey in
  small fresh-response steps. Response drifts a few units over a long
  session — final calibration stays phase D.

  **Post-gate finding — dose is content-dependent.** The tuned ladder is
  strictly monotonic on the match card but shows a tie and one inversion
  on the full-height staircase, top and bottom halves alike, warm or
  cold. The remaining variable is frame time: a pass's duration is the
  sum of per-row conversion times, and rows convert k sweeps — so a
  content change that alters sweep counts alters every pixel's retention
  dose (match card: half the rows are 1-sweep; staircase: all rows are
  ~6-sweep; ~20% dose difference). The July retention physics, resurfaced
  at the architecture level. Phase D fix, before per-board calibration:
  pad each pass to a constant period (extend today's fixed inter-pass
  delay to "delay until fixed pass duration") so dose depends only on
  the trains, then tune under that timing.

  **Implemented and verified 20 July 2026.** 16-grey passes pad to a
  constant period (15 ms NORMAL / 19 ms HIGH; measured row loops run
  4.8-7.0 ms depending on content, worst-case ~10.5 ms at 10 sweeps),
  with the quality's settle floor preserved on overrun. Trains re-tuned
  under the new timing (LilyGo T5 S3 GPS, NORMAL): strictly monotonic on
  the match card (max err 4.2) AND on the full-height staircase — the
  cross-content transfer test that failed before the fix. Remaining in
  phase D: remove-train tuning, HIGH quality, temperature bands, the
  M5PaperS3's own table, and a home for per-board 16-grey tables in the
  preset system.
- **Phase D — calibration**: scanner rig + dithertune extended to tune 32
  trains per board (HIGH first). Gate: optical match of all 16 levels
  against dither references.

Erase-then-draw note: a grey-to-grey pixel today needs two paint calls
(erase, then draw). Under the decision engine its remove-bit and add-bit
coexist in the todo word; a batcher that places remove-decisions in
earlier sweeps completes the transition in one paint. The last deferral
in the engine goes away.
