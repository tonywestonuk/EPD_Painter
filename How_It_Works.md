# EPD Painter — How It Works

A guide for newcomers to the codebase. This document explains the big picture
first, then drills into each major subsystem.

---

## Table of Contents

1. [What Is an E-Paper Display?](#1-what-is-an-e-paper-display)
2. [Pixel Format — From Canvas to Panel](#2-pixel-format--from-canvas-to-panel)
3. [The Screen Buffer — Tracking Physical State](#3-the-screen-buffer--tracking-physical-state)
4. [Waveforms — How Greyscale Works](#4-waveforms--how-greyscale-works)
5. [Interlacing — Why Every Other Row?](#5-interlacing--why-every-other-row)
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
  │                       |
  |  ○ ○ ○ ● ● ● ● ○ ○ ●  │   ← white particles (○) and black particles (●)
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
enough to fit in internal RAM for the fast path.  The 2bpp also matches the Ink drive format (described below) which makes the vector code to translate it into that format much easier, and hence faster.

### Stage 3 — Ink Drive Format (for DMA output)

`epd_painter_convert_packed_fb_to_ink()` converts each packed byte into the
actual voltage drive pattern the panel expects, according to the current
waveform pass. Each 2bpp pixel becomes **two independent drive bits** — one
for each electrode:

```
  Packed byte:  [ P3 P3 P2 P2 P1 P1 P0 P0 ]
                  └-┬-┘ └-┬-┘ └-┬-┘ └-┬-┘
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

If the driver doesn't know what colour pixel, it would need to start from scratch every update, moving the pixel to full white, then
to the actual colour it needs to be. This causes the panel to flash.


---

## 4. Waveforms — How Greyscale Works

Greyscale is achieved by driving each pixel with a **sequence of voltage pulses
across 6 passes**. The accumulated effect of these pulses moves the particles
to a position that reflects the target shade.

Two waveform tables are defined — `lighter_waveform` and `darker_waveform`.
Each has 3 rows (one per pixel value group: black `11`, dark grey `10`, light
grey `01`) and 6 columns (one per pass):

```c
  lighter_waveform[pixel_value][pass]:
               Pass: 0  1  2  2  4  5
  11 (black):      { 1, 2, 2, 2, 3, 0 }
  10 (dark grey):  { 1, 2, 2, 2, 3, 2 }
  01 (light grey): { 2, 2, 2, 2, 2, 2 }

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

For each pass, the waveform bytes for all three pixel groups are packed into a
single `uint32_t` and broadcast across the Q register in the assembly routine:

```
  uint32_t waveform = ( waveform[0][pass] << 16      // for 11 pixels
                      + waveform[1][pass] << 8        // for 10 pixels
                      + waveform[2][pass]      )      // for 01 pixels
```



---

## 5. Interlacing — Why Every Other Row?

The panel is driven in an interlaced fashion: on each frame, **even rows**
receive the `darker` waveform and **odd rows** receive the `lighter` waveform
(then they swap on the next frame via `interlace_period`).

```
  Frame N  (interlace_period = 0):      Frame N+1  (interlace_period = 1):

  Row 0  → DARKER waveform             Row 0  → LIGHTER waveform
  Row 1  → LIGHTER waveform            Row 1  → DARKER waveform
  Row 2  → DARKER waveform             Row 2  → LIGHTER waveform
  Row 3  → LIGHTER waveform            Row 3  → DARKER waveform
  ...                                   ...
```

This serves two purposes:

1. **Reduces flicker** — not all rows receive the same aggressive drive
   simultaneously, so the visual effect is smoother.
2. **Speed** — The user gets to see the image faster. In just 6 passes they would see the image appearing. 

The `ink_on` and `ink_off` functions also respect interlacing — each call
processes only even or only odd rows, controlled by the `interlace_period`
argument. `paint()` calls them with opposite parities so together they cover
every row.

---

## 6. ink_on and ink_off — The Change Detection System

These two assembly functions are the heart of the delta-update system. They
compare the desired new frame (`packed_fastbuffer`) against the current
physical state (`packed_screenbuffer`) and work out what needs to be driven.

### ink_on — Turning Pixels On

```
  For each pixel (in the active interlace rows):

  ┌─────────────────────-┬──────────────────────────────────────────┐
  │  Screen buffer       │  Action                                  │
  ├─────────────────────-┼──────────────────────────────────────────┤
  │  00  (white)         │  Copy new pixel value to output          │
  │                      │  Mark pixel in screen buffer             │
  ├─────────────────────-┼──────────────────────────────────────────┤
  │  01/10/11 (non-white)│  Skip — pixel is already occupied.       │
  │                      │  ink_off must clear it to white first.   │
  └─────────────────────-┴──────────────────────────────────────────┘
```

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

```
  For each pixel (in the active interlace rows):

  ┌──────────────────────────────────────┬───────────────────────────────────┐
  │  Condition                           │  Action                            │
  ├──────────────────────────────────────┼───────────────────────────────────┤
  │  screen buffer DIFFERS from          │  Write screen buffer value to      │
  │  packed_fastbuffer                   │  output (drive pixel toward white) │
  │  (pixel needs to change shade        │  Clear pixel in screen buffer      │
  │   OR needs to go to white)           │  (mark as white)                   │
  ├──────────────────────────────────────┼───────────────────────────────────┤
  │  screen buffer MATCHES               │  Skip — pixel is already correct   │
  │  packed_fastbuffer                   │                                    │
  └──────────────────────────────────────┴───────────────────────────────────┘
```

XOR is used to find differences. Again, the difference is spread across both
bits of each pixel slot so that any change to either bit flags the whole pixel:

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
  │ writes 10 to output      │  →   │ ink_on copies 01 to      │
  │ clears screen buffer     │      │ output and screen buffer  │
  │  (screen buf → 00)       │      │  (screen buf → 01)        │
  └──────────────────────────┘      └──────────────────────────┘
     Panel moving toward white         Panel moving toward light grey
```

---

## 7. DMA and the LCD_CAM Peripheral

The ESP32-S3's LCD_CAM peripheral can clock data out on a parallel bus at high
speed using DMA, without CPU involvement during the transfer. This is how one
row of pixel data is sent to the EPD panel quickly.

### Hardware Setup

```
  ESP32-S3
  ┌─────────────────────────────────────────────┐
  │                                             │
  │  PSRAM / Internal RAM                       │
  │  ┌──────────────┐   ┌──────────────────┐    │
  │  │  DMA buffer1 │   │  DMA descriptor1 │    │
  │  │  (row data)  │←──│  .buffer = buf1  │    │
  │  └──────────────┘   │  .next   = desc2 │    │
  │                      └────────┬─────────┘   │
  │  ┌──────────────┐   ┌────────▼─────────┐    │
  │  │  DMA buffer2 │   │  DMA descriptor2 │    │
  │  │  (row data)  │←──│  .buffer = buf2  │    │
  │  └──────────────┘   │  .next   = desc1 │◄─ -┤ circular chain
  │                      └──────────────────┘   │
  │         ↑                                   │
  │       GDMA ──────────────────────────────►  │
  │                        LCD_CAM peripheral   │
  │                        (i8080 8-bit mode)   │
  └─────────────────────────┬───────────────────┘
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

  sendRow() swaps which buffer is "current" on each call.
  The DMA descriptor chain loops buf1→buf2→buf1 automatically.
```

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
  SPV  ──┐           ┌──────────────────
         └───────────┘
  CKV  ──────┐   ┌─────────────────────
             └───┘
              ↑
         Row pointer resets to line 0 here
```

### Timing for Subsequent Rows

```
  LE   ──┐       ┌────
         └───────┘
  CKV  ──┐       ┌────
         └───────┘
              5µs hold  (for now...may increase to improve quality)
              ↑
         Row pointer advances here; previous row's data is latched
```

### Full Sequence Per Row

```
  1. Swap DMA buffer (CPU fills the one DMA just finished with)
  2. Wait for previous DMA transfer to complete (if active)
  3. Issue timing signals (SPV pulse on row 0, LE+CKV pulse otherwise)
  4. Start DMA transfer (LCD_CAM clocks data onto D0–D7)
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
         ├──── ink_on()  ────────────────────────────────────────────────┐
         │     (even rows)   compares with packed_screenbuffer            │
         │                   pixels: white→grey/black copied to output   │
         │                   screen buffer updated (pixel now inked)      │
         │                                                                │
         ├──── ink_off() ─────────────────────────────────────────────── ┤
         │     (odd rows)    compares with packed_screenbuffer            │
         │                   pixels: changed/clearing → screen value      │
         │                   copied to output; screen buffer cleared      │
         │                   (pixel back to white)                        │
         │                                                                ▼
         │                                                  packed_fastbuffer
         │                                                  (now contains only
         │                                                   pixels that need
         │                                                   driving this frame)
         │
         │  For each of 6 passes:
         │    For each row:
         │      epd_painter_convert_packed_fb_to_ink()  ← applies waveform
         │      sendRow()                               ← DMA to panel
         │
         └──► Neutralise: send 0x00 frame (stops all pixel movement)

  interlace_period flips for next frame.
```

---

## 10. clear() — Full Panel Erase

`clear()` ignores the framebuffer content and forces all pixels to white,
in several stages:

```
  Stage 1: Set packed_fastbuffer to all 0x00 (all-white target)

  Stage 2: ink_off() × 2 (both parities)
           → marks all currently inked pixels as needing to clear
           → screen buffer reset to all 0x00

  Stage 3: 6 lighter waveform passes
           → gradually drives all pixels toward white

  Stage 4: Hard erase — 4 passes each of:
           0b01010101 (full positive drive, all pixels)
           0b10101010 (full negative drive, all pixels)
           → aggressively resets any stubborn particles

  Stage 5: Neutralise with 0x00 frame
```

The alternating hard drive in Stage 4 forces all particles fully to one
extreme and then the other, ensuring the panel is in a known clean state.

---

## 11. Memory Map

```
  ┌─────────────────────────────────────────────────────────┐
  │  PSRAM (external, slower)                               │
  │                                                         │
  │  buffer              960×540 bytes  (GFX canvas, 8bpp)  │
  │  packed_screenbuffer 129,600 bytes  (screen state, 2bpp)│
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
per pass — at 6 passes × 540 rows = 3,240 sequential reads per frame. PSRAM
would be a significant bottleneck here. The screen buffer is only read once
per frame (during `ink_on`/`ink_off`) so PSRAM is fine.

---

## 12. File Overview

```
  EPD_Painter.h      Class definition, pin constants, buffer pointers
  EPD_Painter.cpp    C++ driver: init, DMA setup, paint(), clear(), sendRow()
  EPD_Painter.S      Xtensa assembly: pixel packing, waveform conversion,
                     ink_on, ink_off, interleaved copy
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

     │  ink_on() / ink_off()
     │  (delta detection vs packed_screenbuffer)
     ▼
  packed_fastbuffer[]    only changed pixels remain

     │  convert_packed_fb_to_ink() × 6 passes
     ▼
  dma_buffer1/2[]        ink drive bytes for one row

     │  sendRow() → LCD_CAM DMA → GPIO pins
     ▼
  EPD panel              physical pixels update
```

---

*For questions about the assembly internals, see the comments in `EPD_Painter.S`
which document every register and operation in detail.*
