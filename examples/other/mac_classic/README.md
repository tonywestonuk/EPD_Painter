# Mac Classic — a Macintosh on e-paper

Runs an early Apple Macintosh on the M5PaperS3 or LilyGo T5 S3, with the
e-paper panel as its screen, the touch panel as its mouse and the serial
port as its keyboard. Boots to the Finder; MacPaint on a 1-bit e-paper
screen is exactly as fitting as you'd hope.

By default the Mac runs at a native **960x540**, filling the panel 1:1 —
umac patches the ROM's video parameters and Mac OS drives the big screen
as if you'd bought a period monitor for it. A handful of very early apps
(MacPaint among them, ironically) assume the original 512x342 geometry;
set that in `src/umac/umac_cfg.h` and the sketch centres it with a bezel
instead.

The emulator is [umac](https://github.com/evansm7/umac) by Matt Evans (the
engine behind MicroMac, which ran a Mac on a Raspberry Pi Pico), built on
the [Musashi](https://github.com/kstenerud/Musashi) 68000 core. Both are
MIT licensed and vendored under `src/umac/` — see `src/umac/VENDORED.md`.
This sketch is just the glue: EPD_Painter display, SD-backed disk, GT911
mouse, serial keyboard.

The Mac desktop is an ideal workload for EPD_Painter's delta engine —
mostly static screen, small changes — so only the pixels that change get
driven and the UI feels remarkably alive for e-paper.

## Requirements

- **EPD_Painter** (this library)
- **[GT911 Lite](https://github.com/tonywestonuk/gt911-arduino)** for touch
- An SD card, and two files you must source yourself:

### 1. `/mac/rom.bin` — the Mac Plus v3 ROM

128KB, version `4D1F8172` (the "Mac Plus v3" / 512Ke ROM — the only
version umac's patches support). Apple's ROMs are still Apple's: dump it
from a machine you own, or make your own peace with the usual archives.
If the file is any other size or version the sketch will tell you and halt.

### 2. `/mac/disk.img` — a bootable disk image

A raw image (first two bytes `LK`), any reasonable size — a 400K/800K
floppy image or a several-MB hard-disk-style image both work. The default
4MB machine happily runs System 6; System 3.2 suits a smaller build.
WinWorld carries period System releases; use Mini vMac or Basilisk II on
your desktop to prepare an image with apps on it (MacPaint, MacWrite,
MacDraw...).

If an image has a DiskCopy 4.2 header (84 bytes) rather than starting
with `LK`, strip it:

```
dd if=system.image of=disk.img bs=84 skip=1
```

**Writes are real**: the emulated Mac writes straight back to
`/mac/disk.img`, so your work survives a reboot — and so does anything
you break. Keep a backup copy of the image. If the image can't be opened
for writing it falls back to read-only (the Mac sees a locked disk).

## Build and run

Open `mac_classic.ino`, pick your board preset at the top (or leave
auto-detect), compile and flash. Board/SD wiring is identical to the
`bad_apple` example: native SDMMC on the M5PaperS3, shared-bus SPI on the
LilyGo.

On boot you should see the disk-question-mark, then the happy Mac, then
the Finder — all in glorious 1-bit.

## Using it

**Mouse**: touch the screen. The cursor jumps to your finger (via the
Mac's own low-memory cursor globals — the same trick Basilisk II uses
for absolute pointers) and the button presses a tick later, so a tap
clicks exactly where you touched. Tap twice for a double-click; hold
and move to drag. If the cursor mirrors your finger, adjust
`TOUCH_FLIP_X/Y` at the top of the sketch (press ESC over serial to see
raw coordinates while you calibrate).

**Keyboard**: characters typed over serial (115200 baud) are pressed on
the Mac. `Ctrl-<letter>` sends `Command-<letter>` — Ctrl-S is Cmd-S, and
so on — which needs a real terminal (`screen /dev/cu.usbmodem... 115200`
or similar) rather than a line-buffered serial monitor. There are no
arrow keys; neither did the Mac 128K's keyboard.

A stats line prints every 5 seconds: `speed 100%` means the 68000 is
running at true 8MHz pace; paints counts panel updates.

## Tweakables

- `UMAC_MEMSIZE` in `src/umac/umac_cfg.h` — emulated RAM in KB. Default
  4096 (a 4MB Mac Plus-class machine, System 6/7 capable). Use 128 for a
  faithful Mac 128K (System 3.2 at most, and use a floppy-sized image).
- `DISP_WIDTH`/`DISP_HEIGHT` in `src/umac/umac_cfg.h` — the Mac's screen
  resolution. 960x540 (default) fills the panel; 512x342 is the original,
  shown centred. Width must be a multiple of 32, and width x height / 8
  must stay under 64KB (960x540 uses 64,800 of the 65,536).
- `UI_QUALITY` — QUALITY_FAST by default. The Mac screen is pure black
  and white, so the calibrated greys of NORMAL buy nothing except slower
  updates; FAST is the right choice here.
- `PAINT_INTERVAL_MS` — minimum time between panel updates. Lower is
  livelier, higher leaves more bandwidth for large updates.
- `CLEAR_INTERVAL_MS` — every 30s (default), if the screen changed since
  the last clear, the panel gets a full clear-and-repaint to sweep away
  accumulated ghosting. A static desktop never flashes. 0 disables.

## How it fits together

- The 68000 runs on core 1, paced to real time in 5ms slices (the ESP32-S3
  has speed to spare; the throttle is what keeps games playable).
- A small task on core 0 watches the Mac's framebuffer at
  `PAINT_INTERVAL_MS`; when it changes, a 256-entry LUT expands 1bpp to
  the panel's packed 2bpp format and `paintPacked()` does the rest —
  the delta engine only drives pixels that actually changed.
- The disk is streamed block-by-block from SD via umac's paravirtual
  driver callbacks — no image in RAM, so image size is unconstrained.
- Touch is absolute but the Mac's mouse is relative, so the sketch reads
  the cursor position the Mac believes (low-memory global `RawMouse`)
  and feeds it deltas until the two agree.
