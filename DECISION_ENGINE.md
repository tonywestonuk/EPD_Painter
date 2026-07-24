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

  **Charge-matched removes, 21 July 2026 (LilyGo T5 S3 GPS, NORMAL).**
  The DC audit found paint+unpaint cycles were not balanced: the formula
  removes overdrove white by 3..8 passes per cycle. Under the constant
  pass period charge is simply darkens − whitens per train, so the fix
  is a remove train satisfying TWO constraints: land optically white AND
  net whitens = the apply's net darkens. The imbalance is optically real,
  not just ledger-keeping: a half-screen ghost test (cycle paint/erase of
  L12 on the left half 20x, then paint a uniform L7 probe over the whole
  glass) put a 4-scan-unit seam at the midline with the formula removes —
  over-whitened glass answers the next darken train a full grey step
  stronger (the fresh-response physics again). The ghost decays within
  minutes of a clear, so it is a transient artefact plus a long-term
  panel-health liability, not permanent damage per cycle.

  Tuning used a remove ladder: paint the staircase, erase to white,
  scan residuals per bar against a hard-clear reference (column-wise, so
  illumination cancels). Two shape lessons: appended charge-neutral
  (darken, whiten) scrub pairs erase NOTHING — the darken undoes its
  whiten's gain almost exactly; what works is the darkens FIRST, then
  all whitens — deepen toward saturation where darkens are optically
  cheap, then every whiten fires from a dark state where it is strong
  (drive-to-opposite-rail activation, at zero extra net charge). All 15
  removes now land within 1.5 scan units of clear-white at exactly the
  matched net. Gate: the 20-cycle ghost test dropped from a −4.0 seam
  (formula) to −1.1 (matched), at the measurement noise floor; white
  state lands −0.6. Tuned set in the same header as the applies.

  **M5PaperS3 table, 21 July 2026.** The full loop (pure-darken probe →
  match-card applies → remove ladder → ghost gate) ran on the M5PaperS3,
  seeded from the LilyGo set: 10 of 15 apply levels passed unchanged on
  the first scan — the two panels share a physics family — and the rest
  converged in five iterations (all 15 within ±2.6; strictly monotonic
  staircase). Divergences the glass insisted on: L1 is a single darken
  (stronger first-pass response), L12 five pure darkens, L14 nine.
  Charge-matched removes converged in three ladder iterations; ghost
  gate at the noise floor (white +0.3, probe seam +0.9 after 20 cycles).
  The testcard now carries both boards' tables and picks at runtime
  (the M5PaperS3 preset is the one with a power-latch pin). Boot-image
  DC cycle fixed for 16-grey: EPD_BootCtl forces setGreyLevels(4) both
  sides of the sleep, since paintPacked/unpaintPacked are 2bpp-only.
- **Phase D — calibration**: scanner rig + dithertune extended to tune 32
  trains per board (HIGH first). Gate: optical match of all 16 levels
  against dither references.

Erase-then-draw note: a grey-to-grey pixel today needs two paint calls
(erase, then draw). Under the decision engine its remove-bit and add-bit
coexist in the todo word; a batcher that places remove-decisions in
earlier sweeps completes the transition in one paint. The last deferral
in the engine goes away.

## Direct grey-to-grey transitions (implemented 21 July 2026)

Motivated by the megademo's bouncing-title artifact (Tony's diagnosis:
black text descending into its own grey drop-shadow — every grey-to-grey
pixel two-steps through white, punching visible holes; ascending runs
the same physics at invisible contrast). The gap: the LUTs only knew
trajectories from and to white. The generalization the engine was built
for: **decision = (from, to)**, not (level, direction). Both tiers are
in and gated on the M5PaperS3.

**4-level tier: six tuned direct trains.** `setDirectTransitions(true)`
switches 4-level paints to a greedy-sweep discovery over the 2bpp
buffers (the 16-grey walk's shape); a changed occupied pixel whose
(from, to) pair has a loaded train (`setDirectTrain`) takes one direct
decision — id `16 | (from << 2) | to` — and the screen state lands on
the new value in ONE paint. Unloaded pairs fall back to the legacy
two-step, so the engine comes up one tuned pair at a time. Worst case
12 distinct decisions on a line = 4 sweeps (2 spill slot planes,
~260 KB PSRAM, allocated on first enable).

Direct trains may be LONGER than the quality's 13 passes (up to 26):
the frame extends to the longest direct train in play, content without
deep transitions pays nothing. This exists because 3→2 provably cannot
fit its charge AND its landing in 13 passes: net −7 with d trailing
darkens forces 7+d whitens, capping d at 3 — which stops a full grey
step short of dark grey. Its tuned train is 17 passes (12 whitens, 5
fresh darkens).

Calibration (testcard 'D', the transition-card method): paint every
column's FROM level, repaint TO in a single paint, compare each
transition column's landing against a reference column painted from
white. M5PaperS3 NORMAL set (`src/EPD_Painter_trains.h`): all six
pairs within +3.5/−1.8 scan units of reference. Shape lessons mirror
the remove work: trailing whitens poison landings (darkening directs
put their balancing whitens FIRST); darkens after deep whitening ride
the fresh-response boost (2→1 is a tiny darkens-first erase, 1d,3w);
a darken followed by one whiten survives at ~1/3 strength — the
d,w,d,w,d tail on 3→1 is the fine-trim knob.

**16-grey tier: temporal partition.** 240 directed pairs is too many to
tune, so the same paint concatenates the two tuned halves: a
grey-to-grey pixel takes BOTH remove(from) and apply(to); removes drive
passes 0..R−1 and the apply trains that grey-to-grey pixels use shift
right by R (R = longest remove among the frame's grey-to-grey pixels;
fresh draws onto white keep firing from pass 0). One pixel then sits in
two sweeps with disjoint pass ranges, so the OR-merge stays safe.
Discovery partitions sweeps by direction (applies planes 0..4 —
fastbuffer first, fresh draws stay internal; removes planes 5..9)
because one slot plane cannot encode two decisions for one pixel.
Pass count = R + longest shifted apply; frames without grey-to-grey
keep R = 0 and pay nothing.

**DC balance by construction**: potential Q(g) = the apply train's net
charge (measured per board). Every 4-level direct train carries exactly
Q(to) − Q(from); the 16-grey partition composes −Q(from) + Q(to) from
the already-matched tables. The ledger is path-independent —
white→2→3→white nets zero along any route.

**Gates passed (M5PaperS3, 21 July 2026):** the shadowed bouncing logo
descends with no via-white holes (megademo intro, NORMAL + direct); the
boing ball's grey-on-grey motion completes per paint (Tony: "much
better... better slow correct, than fast wrong"). Mixed-path ghost
(testcard 'G', 20 cycles × 10 transitions through
white→1→2→3→2→1→white and white→1→3→1→white): immediate-probe seam
−6.5 for the direct engine vs −9.3 for the legacy two-step on identical
content — less transient polarization than the status quo, both decay
to noise within minutes, no persistent imbalance.

Remaining in this area: LilyGo direct set (seeds from the M5Paper
finals), FAST-quality direct trains if the intro should return to 17
fps with its shadow (7 undelayed passes may not fit the lightening
trains), and the in-paint transit flicker of deep lightening directs
(physics: a 3→1 train must shed 9 charge units and the glass erases
faster than it discharges, so the pixel overshoots toward white inside
the paint — contained, but visible at NORMAL cadence).
