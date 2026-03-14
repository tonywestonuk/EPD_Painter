# EPD Painter — How It Works

A guide for newcomers to the codebase. This document explains the big picture
first, then drills into each major subsystem.

---

## Table of Contents

1. [What Is an E-Paper Display?](#1-what-is-an-e-paper-display)
2. [Pixel Format — From Canvas to Panel](#2-pixel-format--from-canvas-to-panel)
3. [The Screen Buffer — Tracking Physical State](#3-the-screen-buffer--tracking-physical-state)
4. [Waveforms — How Greyscale Works](#4-waveforms--how-greyscale-works)
5. [The Chunk Bitmask — 64-Pixel Decision Blocks](#5-the-chunk-bitmask--64-pixel-decision-blocks)
6. [ink_on and ink_off — The Change Detection System](#6-ink_on-and-ink_off--the-change-detection-system)
7. [DMA and the LCD_CAM Peripheral](#7-dma-and-the-lcd_cam-peripheral)
8. [sendRow() — The Row Timing Protocol](#8-sendrow--the-row-timing-protocol)
9. [paint() — Full Frame Update Walk-Through](#9-paint--full-frame-update-walk-through)
10. [clear() — Full Panel Erase](#10-clear--full-panel-erase)
11. [Memory Map](#11-memory-map)
12. [File Overview](#12-file-overview)

---

## 1. What Is an E-Paper Display?

E-paper (electrophoretic) displays work by physically moving charged pigment
particles through a fluid. Unlike an LCD, the panel holds its image with **no
power** once set — pixels are mechanically latched.

Each pixel contains both black and white particles:

```
  Positive electrode (top)
  ┌───────────────────────┐
  │                       │
  │  ○ ○ ○ ● ● ● ● ○ ○ ●  │   ← white particles (○) and black particles (●)
  │     fluid medium      │      floating in a microcapsule
  └───────────────────────┘
  Negative electrode (bottom)

  Apply + voltage → black particles rise → pixel appears BLACK
  Apply - voltage → white particles rise → pixel appears WHITE
  No voltage      → particles hold position → image retained
```

Because the particles must physically move, driving the display requires
**multiple passes** of carefully timed voltage pulses — you cannot just write a
pixel value once and have it appear instantly at the right shade.

---

## 2. Pixel Format — From Canvas to Panel

The code uses **three different pixel representations** as data moves through
the pipeline:

### Stage 1 — GFX Canvas (8bpp, 1 byte per pixel)

The Adafruit GFX library works with one byte per pixel. Only the **2 least
significant bits** are meaningful here; the upper 6 bits are ignored.

```
  Byte in canvas:  [ x x x x x x P P ]
                                 └─┘
                             2-bit pixel value
```

| Value | Shade       |
|-------|-------------|
| `00`  | White       |
| `01`  | Light grey  |
| `10`  | Dark grey   |
| `11`  | Black       |

### Stage 2 — Packed Framebuffer (2bpp, 4 pixels per byte)

`epd_painter_compact_pixels()` compresses the canvas by packing 4 pixels into
each byte. This is what lives in `packed_fastbuffer` and `packed_screenbuffer`.

```
  4 canvas bytes:    [00PP] [00PP] [00PP] [00PP]
                       P3     P2     P1     P0

  → 1 packed byte:   [ P3 P2 P1 P0 ]   (MSB first)
```

This 4× compression matters because the 960×540 panel would otherwise need
518,400 bytes per framebuffer. At 2bpp it costs only 129,600 bytes — small
enough to fit in internal RAM for the fast path. The 2bpp format also matches
the ink drive format (described below) which makes the vector code to translate
between them much easier, and hence faster.

### Stage 3 — Ink Drive Format (for DMA output)

`epd_painter_convert_packed_fb_to_ink()` converts each packed byte into the
actual voltage drive pattern the panel expects, according to the current
waveform pass. Each 2bpp pixel becomes **two independent drive bits** — one
for each electrode:

```
  Packed byte:  [ P3 P3 P2 P2 P1 P1 P0 P0 ]
                  └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘
                    │     │     │     └── pixel 0: bit1=positive electrode, bit0=negative
                    │     │     └───────  pixel 1
                    │     └────────────   pixel 2
                    └─────────────────    pixel 3

  Drive codes:  00 = no drive (float)
                01 = drive negative  (whiten)
                10 = drive positive  (darken)
                11 = drive both      (DC balance)
```

---

## 3. The Screen Buffer — Tracking Physical State

The EPD panel has no way to report what is currently shown on screen. The code
maintains `packed_screenbuffer` as a **software model of the physical display**:

```
  packed_screenbuffer   ←→   What is physically on the panel right now
  packed_fastbuffer     ←→   What we WANT the panel to show next
```

The screen buffer starts as all `0x00` (all white) and is updated incrementally
by `ink_on` and `ink_off` as pixels change. It is stored in PSRAM (it only
needs to be read once per frame, so speed is less critical).

**Why is this necessary?**

If the driver doesn't know what colour a pixel currently is, it would need to
start from scratch every update, moving the pixel to full white, then to the
actual colour it needs to be. This causes the panel to flash.

**The screen buffer does not survive a power cycle.** On reboot it is
initialised to all-white, but the physical panel still shows whatever was last
displayed. The `EPD_PainterShutdown` class handles this: on shutdown it saves
a known image (the shutdown image) to LittleFS and paints it to the panel; on
the next boot it unpaints that image — driving all those pixels back toward
white — so that by the time user code starts, the screen buffer and the
physical panel are in agreement. See the Reference Manual for the full
shutdown handling documentation.

---

## 4. Waveforms — How Greyscale Works

Greyscale is achieved by driving each pixel with a **sequence of voltage pulses
across 7 passes**. The accumulated effect of these pulses moves the particles
to a position that reflects the target shade.

Two waveform tables are defined — `lighter_waveform` and `darker_waveform`.
Each has 3 rows (one per pixel value group: black `11`, dark grey `10`, light
grey `01`) and 7 columns (one per pass):

```c
  lighter_waveform[pixel_value][pass]:
               Pass: 0  1  2  3  4  5  6
  11 (black):      { 1, 2, 2, 0, 2, 0, 2 }
  10 (dark grey):  { 2, 2, 2, 2, 2, 3, 3 }
  01 (light grey): { 2, 2, 2, 2, 2, 2, 2 }

  Drive codes:  0 = float    1 = whiten (+)
                2 = darken (-)    3 = both simultaneously
```

### The Printing Press Analogy

This process is conceptually very close to how a traditional printing press
works. A press applies each ink colour in a separate pass — cyan, magenta,
yellow, black — and the final image emerges from their combination across
all passes.

Here, the three "inks" are the drive patterns for each greyscale shade
(black, dark grey, light grey). On each pass, all three are loaded
simultaneously and stamped onto the 64-pixel chunk at once, each one
landing only on the pixels it belongs to:

```
  Traditional press:                  EPD waveform pass:

  Cyan plate   → only cyan pixels     Waveform for 11  → only black pixels
  Magenta plate→ only magenta pixels  Waveform for 10  → only dark grey pixels
  Yellow plate → only yellow pixels   Waveform for 01  → only light grey pixels
        │                                     │
        └── all applied in one roller pass    └── all applied in one 64-pixel chunk
```

For each pass, the waveform bytes for all three pixel groups are broadcast
into Q registers in the assembly routine:

```
  lighter_wf[0] = lighter_waveform[2][pass] * 0x55   // for 01 pixels
  lighter_wf[1] = lighter_waveform[1][pass] * 0x55   // for 10 pixels
  lighter_wf[2] = lighter_waveform[0][pass] * 0x55   // for 11 pixels
```

The `* 0x55` multiplication spreads each 2-bit drive code across all pixel
positions in the byte, so the waveform can be applied with a simple AND/OR
mask against the pixel identity bits.

---

## 5. The Chunk Bitmask — 64-Pixel Decision Blocks

All pixel processing in EPD Painter operates on **64-pixel chunks** (16 packed
bytes), matching the ESP32-S3's 128-bit vector registers. Each row's chunks
are tracked by a 32-bit bitmask stored in the `bitmask[]` array (one entry
per row, MSB = first chunk on the left):

```
  Row of 960 pixels = 240 packed bytes = 15 chunks of 64 pixels

  bitmask[row]:  [1 0 1 1 0 0 0 1 0 0 0 0 0 0 0 x x x x x x x x x x x x x x x x x]
                  ↑               ↑               ↑
               bit 31          bit 24          bit 17    (bits 16–0 unused)

  1 = this chunk is DARKENING (new pixels appearing)
  0 = this chunk is LIGHTENING (pixels being removed or unchanged)
```

**The critical rule:** within any single chunk, ALL pixels must be either
darkening or lightening — never a mix. If a pixel needs to lighten but falls
within a darkening chunk, it is set to zero and deferred to a future frame.
Over successive `paint()` calls, the display converges to the correct image.

This constraint exists because each chunk receives either the darker or lighter
waveform during `epd_painter_convert_packed_fb_to_ink()`. The bitmask tells
the converter which waveform to apply:

```
  epd_painter_convert_packed_fb_to_ink(fb_row, dma_buf, len, darker_wf,  bitmask[row]);
  epd_painter_convert_packed_fb_to_ink(fb_row, dma_buf, len, lighter_wf, ~bitmask[row]);
```

The first call processes chunks flagged as darkening; the second call processes
the complement (lightening chunks). Together they cover every chunk exactly
once, fully populating the DMA output buffer for that row.

The bitmask persists across frames — it is passed back into `ink_on`/`ink_off`
on the next `paint()` call so the system remembers which chunks were active.

---

## 6. ink_on and ink_off — The Change Detection System

These two assembly functions are the heart of the delta-update system. They
compare the desired new frame (`packed_fastbuffer`) against the current
physical state (`packed_screenbuffer`) and work out what needs to be driven.

Both functions are called per row, processing all chunks in that row
sequentially.

### ink_on — Turning Pixels On

```
  For each 64-pixel chunk:

  ┌──────────────────────┬──────────────────────────────────────────┐
  │  Screen buffer       │  Action                                  │
  ├──────────────────────┼──────────────────────────────────────────┤
  │  00  (white)         │  Copy new pixel value to output          │
  │                      │  Mark pixel in screen buffer             │
  ├──────────────────────┼──────────────────────────────────────────┤
  │  01/10/11 (non-white)│  Skip — pixel is already occupied.       │
  │                      │  Zero this pixel in the fastbuffer.      │
  │                      │  ink_off must clear it to white first.   │
  └──────────────────────┴──────────────────────────────────────────┘
```

After masking, if any pixels in the chunk survived (are non-zero), the chunk
is flagged as darkening (bit set in the returned bitmask). The screen buffer
is updated to reflect the newly occupied pixels.

The mask logic in the assembly works by spreading each pixel's bits — if
either bit of a 2bpp pixel in the screen buffer is set, both bits become set,
then the mask is inverted. The result is `11` only for pixels that were `00`:

```
  screen pixel = 01       screen pixel = 00

  spread low→high:  11    spread low→high:  00
  invert:           00    invert:           11
                    ↑                       ↑
               blocked                  allowed
```

### ink_off — Turning Pixels Off

`ink_off` processes chunks that `ink_on` did NOT flag (bitmask bit = 0).
Chunks with bitmask bit = 1 are skipped entirely.

```
  For each unflagged 64-pixel chunk:

  ┌──────────────────────────────────────┬───────────────────────────────────┐
  │  Condition                           │  Action                           │
  ├──────────────────────────────────────┼───────────────────────────────────┤
  │  screen buffer DIFFERS from          │  Write screen buffer value to     │
  │  packed_fastbuffer                   │  fastbuffer (to drive pixel back  │
  │  (pixel needs to change shade        │  toward white).                   │
  │   OR needs to go to white)           │  Clear pixel in screen buffer.    │
  ├──────────────────────────────────────┼───────────────────────────────────┤
  │  screen buffer MATCHES               │  Set fastbuffer to zero           │
  │  packed_fastbuffer                   │  (no drive needed).               │
  └──────────────────────────────────────┴───────────────────────────────────┘
```

XOR is used to find differences. The difference is spread across both bits of
each pixel slot so that any change to either bit flags the whole pixel:

```
  desired = 01,  screen = 10     desired = 10,  screen = 10
  XOR     = 11  → pixel differs  XOR     = 00  → pixel matches
  spread  = 11  → drive it       spread  = 00  → skip it
```

### The Two-Frame Transition

A pixel changing from dark grey (`10`) to light grey (`01`) takes **two frames**:

```
  Frame N:                          Frame N+1:
  ┌──────────────────────────┐      ┌──────────────────────────┐
  │ ink_off detects mismatch │      │ screen buffer now = 00   │
  │ writes 10 to fastbuffer  │  →   │ ink_on copies 01 to      │
  │ clears screen buffer     │      │ fastbuffer & screen buf   │
  │  (screen buf → 00)       │      │  (screen buf → 01)        │
  └──────────────────────────┘      └──────────────────────────┘
     Panel moving toward white         Panel moving toward light grey
```

### Deferred Pixels

If a pixel needs to lighten but falls within a chunk that is darkening
(bitmask bit = 1), `ink_on` zeros that pixel in the fastbuffer and `ink_off`
skips the entire chunk. That pixel is **deferred** — it will be processed on a
future frame when the chunk is no longer darkening. Over successive `paint()`
calls the display converges to the correct image.

---

## 7. DMA and the LCD_CAM Peripheral

The ESP32-S3's LCD_CAM peripheral can clock data out on a parallel bus at high
speed using DMA, without CPU involvement during the transfer. This is how one
row of pixel data is sent to the EPD panel quickly.

### Hardware Setup

```
  ESP32-S3
  ┌─────────────────────────────────────────────────────────┐
  │                                                         │
  │  Internal RAM                                           │
  │  ┌──────────────┐   ┌──────────────────┐                │
  │  │  DMA buffer1 │   │  DMA descriptor1 │                │
  │  │  (row data)  │←──│  .buffer = buf1  │                │
  │  └──────────────┘   │  .suc_eof = 1    │                │
  │                      │  .next = nullptr │                │
  │  ┌──────────────┐   └──────────────────┘                │
  │  │  DMA buffer2 │   ┌──────────────────┐                │
  │  │  (row data)  │←──│  DMA descriptor2 │                │
  │  └──────────────┘   │  .suc_eof = 1    │                │
  │                      │  .next = nullptr │                │
  │                      └──────────────────┘                │
  │         ↑                                               │
  │       GDMA ──────────────────────────────────────────►  │
  │                        LCD_CAM peripheral                │
  │                        (i8080 8-bit mode, 80 MHz PCLK)  │
  └─────────────────────────┬───────────────────────────────┘
                             │  8-bit parallel bus + PCLK
                             ▼
                    EPD data lines (D0–D7 + CL)
```

### Double Buffering

Two DMA buffers are used so the CPU can fill one while the DMA engine is
streaming the other — there is no gap between rows:

```
  Time →

  CPU:  [fill buf1] [fill buf2] [fill buf1] [fill buf2] ...
  DMA:             [send buf1] [send buf2] [send buf1]  ...
```

Each DMA descriptor has `suc_eof = 1` and `.next = nullptr`, so the DMA engine
stops after completing each buffer. `sendRow()` explicitly resets the DMA
channel and points it at the correct descriptor before each transfer, ensuring
the DMA always starts from byte 0 of the intended buffer. This avoids any
synchronisation issues between the DMA engine and the LCD_CAM peripheral.

---

## 8. sendRow() — The Row Timing Protocol

The EPD panel uses a shift-register architecture for row selection. Three
control signals manage the row pointer:

| Signal | Purpose                                           |
|--------|---------------------------------------------------|
| `SPV`  | Frame start — resets row pointer to line 0        |
| `CKV`  | Row clock — falling edge advances the row pointer |
| `LE`   | Latch enable — commits shift register to the row  |

### Timing for the First Row

```
  SPV  ──┐    ┌─────
         └────┘
  CKV  ──┐   ┌─────
         └───┘
          ↑
Row pointer resets to line 0 here
```

### Timing for Subsequent Rows

```
  LE   ──┐       ┌────
         └───────┘
  CKV  ──┐      ┌────
         └──────┘
            ↑
         Row pointer advances here; previous row's data is latched
```

The `gdma_reset()` call between pin transitions serves double duty: it resets
the DMA channel for the next transfer while simultaneously providing the
timing delay required by the CKV/LE signals — eliminating the need for a
separate `delayMicroseconds()` call.

### Full Sequence Per Row

```
  1. Wait for previous DMA transfer to complete (lcd_start == 0)
  2. Issue timing signals (SPV pulse on row 0, LE+CKV pulse otherwise)
     — gdma_reset() is used as the timing delay between pin transitions
  3. Point DMA at the correct descriptor (alternating desc1/desc2)
  4. Start DMA transfer (LCD_CAM clocks data onto D0–D7 at 80 MHz)
  5. On the last row: wait for DMA to finish, then issue final latch pulse
```

---

## 9. paint() — Full Frame Update Walk-Through

This is what happens every time you call `paint()`:

```
  GFX canvas (8bpp, PSRAM)
         │
         │  epd_painter_compact_pixels()
         ▼
  packed_fastbuffer (2bpp, internal RAM)
         │
         │  For each row:
         ├──── ink_on()  ───────────────────────────────────────────┐
         │     compares fastbuffer with packed_screenbuffer          │
         │     pixels on white screen → copied to output            │
         │     pixels on occupied screen → zeroed (deferred)        │
         │     screen buffer updated (new pixels marked)            │
         │     returns bitmask: 1 = darkening chunk                 │
         │                                                          │
         ├──── ink_off() ───────────────────────────────────────────┤
         │     skips chunks with bitmask bit = 1                    │
         │     changed pixels → screen value written to fastbuffer  │
         │     screen buffer cleared for those pixels               │
         │     matching pixels → fastbuffer zeroed                  │
         │                                                          ▼
         │                                             packed_fastbuffer
         │                                             (now contains only
         │                                              pixels that need
         │                                              driving this frame)
         │
         │  For each of 7 waveform passes:
         │    For each row:
         │      convert_packed_fb_to_ink(darker_wf,  bitmask[row])  ← darkening chunks
         │      convert_packed_fb_to_ink(lighter_wf, ~bitmask[row]) ← lightening chunks
         │      sendRow()                                           ← DMA to panel
         │
         └──► Neutralise: send all-zero frame (stops all pixel movement)
```

### Performance

At 80 MHz pixel clock on a 960×540 panel, each row of 240 packed bytes
transfers in ~3µs. With 540 rows × 7 passes plus latch delays, a full
`paint()` completes in approximately **14ms at low quality** — roughly 70fps
equivalent, making this one of the fastest greyscale EPD drivers available.

---

## 10. clear() — Full Panel Erase

`clear()` ignores the framebuffer content and forces all pixels to white,
in several stages:

```
  Stage 1: Set packed_fastbuffer to all 0x00 (all-white target)

  Stage 2: ink_off() on all rows
           → marks all currently inked pixels as needing to clear
           → screen buffer reset to all 0x00

  Stage 3: 7 lighter waveform passes
           → moves inked pixels toward white, discharging DC bias

  Stage 4: Hard erase — 2 phases of 8 passes each:
           0b01010101 (full positive drive, all pixels)
           0b10101010 (full negative drive, all pixels)
           → aggressively resets any stubborn particles

  Stage 5: Neutralise with all-zero frame
```

The alternating hard drive in Stage 4 forces all particles fully to one
extreme and then the other, ensuring the panel is in a known clean state.

Note that `clear()` does not clear the Adafruit GFX canvas buffer, only the
physical screen. The next time `paint()` is called, the canvas contents will
be redrawn on a fresh white background.

---

## 11. Memory Map

```
  ┌─────────────────────────────────────────────────────────┐
  │  PSRAM (external, slower)                               │
  │                                                         │
  │  buffer              960×540 bytes  (GFX canvas, 8bpp)  │
  │  packed_screenbuffer 129,600 bytes  (screen state, 2bpp)│
  │  bitmask[]           540 × 4 bytes  (chunk flags/row)   │
  └─────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────┐
  │  Internal RAM (fast, CPU + DMA accessible)              │
  │                                                         │
  │  packed_fastbuffer   129,600 bytes  (working fb, 2bpp)  │
  │  dma_buffer1         240 bytes      (one packed row)    │
  │  dma_buffer2         240 bytes      (one packed row)    │
  └─────────────────────────────────────────────────────────┘

  packed_row_bytes = width / 4 = 960 / 4 = 240 bytes per row
  packed_size      = 960 × 540 / 4       = 129,600 bytes
```

`packed_fastbuffer` is kept in internal RAM because it is read once per row
per pass — at 7 passes × 540 rows = 3,780 sequential reads per frame. PSRAM
would be a significant bottleneck here. The screen buffer is only read once
per frame (during `ink_on`/`ink_off`) so PSRAM is fine.

---

## 12. File Overview

```
  EPD_Painter.h              Class definition, config struct, buffer pointers
  EPD_Painter.cpp            C++ driver: init, DMA setup, paint(), clear(), sendRow()
  EPD_Painter.S              Xtensa assembly: pixel packing, waveform conversion,
                             ink_on, ink_off
  EPD_Painter_presets.h      Board-specific pin configurations
  epd_painter_powerctl       TPS65185 power management (for boards with this chip)
  epd_painter_shutdown.h/.cpp  Screen-buffer reconciliation across power cycles,
                               reset-to-shutdown trigger, shutdown image
                               storage, idle timer, pre-shutdown callbacks
```

### Key Data Flow Summary

```
  User code
     │
     │  drawPixel(), fillRect(), etc.   (Adafruit GFX API)
     ▼
  buffer[]               8bpp GFX canvas in PSRAM

  paint() called
     │
     │  compact_pixels()
     ▼
  packed_fastbuffer[]    2bpp packed, internal RAM

     │  ink_on() / ink_off()  per row
     │  (delta detection vs packed_screenbuffer)
     │  (bitmask[] tracks darkening vs lightening chunks)
     ▼
  packed_fastbuffer[]    only changed pixels remain

     │  convert_packed_fb_to_ink() × 7 passes
     │  (darker waveform for bitmask=1 chunks,
     │   lighter waveform for bitmask=0 chunks)
     ▼
  dma_buffer1/2[]        ink drive bytes for one row

     │  sendRow() → GDMA reset → LCD_CAM 80MHz → GPIO pins
     ▼
  EPD panel              physical pixels update
```

---

*For questions about the assembly internals, see the comments in `EPD_Painter.S`
which document every register and operation in detail.*
