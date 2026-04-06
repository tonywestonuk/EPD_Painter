# EPD_Painter Reference Manual

A high-performance e-paper display driver for ESP32-S3 boards (M5PaperS3, LilyGo T5 S3 GPS, LilyGo T5 S3 H752).

---

## Quick Setup

### 1. Choose how the board is selected

You can select a board explicitly with a `#define`, or let the library fall back to AUTO mode when no preset is defined.

Optional explicit presets:

```cpp
#define EPD_PAINTER_PRESET_M5PAPER_S3
// or
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
// or
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_H752
```

If none of those is defined, `EPD_PAINTER_PRESET_AUTO` is enabled automatically and `begin()` probes the supported boards at runtime via I2C.

### 2. Choose a binding

| Binding | Header | Use when |
|---|---|---|
| Adafruit GFX | `EPD_Painter_Adafruit.h` | You want `print()`, `drawBitmap()`, shapes, fonts |
| LVGL v9 | `EPD_Painter_LVGL.h` | You want a full UI widget library |
| Raw driver | `EPD_Painter.h` | You manage your own 8bpp pixel buffer |

---

## Core API

### `begin()`

Allocates buffers, initialises the LCD_CAM DMA hardware, and starts the background paint task on core 0. Power control hardware (TPS65185 PMIC, PCA9555 IO expander, or 74HCT4094D shift register) is initialised automatically based on the preset. I2C is initialised internally — you do not need to call `Wire.begin()`.

Returns `true` on success.

```cpp
if (!display.begin()) {
    Serial.println("EPD init failed");
    while (1);
}
```

When AUTO mode is active, `begin()` also performs board detection and returns `false` if no supported board is found.

If you need access to the I2C bus (e.g. to talk to other devices on the same bus), retrieve the `Wire` instance via `getConfig()`:

```cpp
TwoWire* wire = display.getConfig().i2c.wire;
wire->beginTransmission(0x51);  // talk to another device on the bus
// ...
```

---

### `paint()`

Pushes the current framebuffer to the physical display using **delta update** — only pixels that changed since the last `paint()` are driven.

Blocks until the background paint task picks up the buffer. The task then runs the full waveform sequence on core 0 while your code continues on core 1.

```cpp
display.paint();
```

Panel power is managed automatically. The panel powers on when painting starts and powers off approximately 5 seconds after the last paint call with no further activity.

---

### `paintLater()`

Non-blocking version of `paint()`. Compacts the framebuffer and hands it to the background paint task, then returns immediately. The panel refresh happens on core 0.

```cpp
display.paintLater();   // returns immediately; panel updates in background
// ...do other work here...
```

> **Note:** The framebuffer can be modified freely while a background refresh is in progress — the driver always moves towards the latest framebuffer state. In the LVGL binding, `paintLater()` is called automatically inside the flush callback.

---

### `paintPacked()`

Like `paint()` but accepts a pre-packed 2bpp buffer directly, skipping the 8bpp→2bpp compaction step. Useful when you already have data in the driver's internal format (e.g. loaded from storage).

Blocks until the paint task picks it up.

```cpp
display.paintPacked(packed_buf);   // packed_buf: 2bpp, 4 pixels per byte
```

---

### `unpaintPacked()`

DC-balance pass. Tells the driver that the screen currently shows the content of `packed`, then drives a blank (all-zero) frame — giving every pixel that was darkened a matching lightening pulse.

Used during startup to reconcile the screen state after a graceful shutdown: the shutdown image that was on screen gets driven back toward white before normal use begins.

```cpp
display.unpaintPacked(shutdown_packed_buf);
```

---

### `clear()`

**Drives the physical panel to full white.** This is a hardware-level operation that applies a full blanking waveform — it removes ghosting by actively driving every pixel. It does **not** clear your framebuffer.

```cpp
display.clear();   // full panel — HARD mode by default
```

`clear()` accepts optional parameters:

```cpp
// Clear specific regions only
EPD_Painter::Rect rects[] = { {x, y, w, h} };
display.clear(rects, 1);

// SOFT mode: 2 phases instead of 4 — faster, less thorough
display.clear(nullptr, 0, EPD_Painter::ClearMode::SOFT);
```

| `ClearMode` | Phases | Use when |
|---|---|---|
| `HARD` (default) | 4 (6+2+4+8 passes) | Full ghosting removal, maximum quality |
| `SOFT` | 2 (2+2 passes) | Quick refresh between similar content |

**The standard ghosting-removal sequence:**
```cpp
display.clear();   // blanks the physical panel
display.paint();   // pushes current framebuffer onto the clean panel
```

> Use `clear()` sparingly — it is a full-panel waveform sequence and slower than a delta `paint()`.

---

### `clearDirtyAreas()`

Convenience function: compacts the framebuffer, computes dirty rectangles, then calls `clear()` on only those areas.

```cpp
display.clearDirtyAreas();                                       // default tolerance=0, SOFT mode
display.clearDirtyAreas(tolerance, EPD_Painter::ClearMode::HARD);
```

Useful before a `paint()` when you want to ghost-clear only the changed parts of the screen rather than the full panel.

---

### `computeDirtyRects()`

Scans the packed screenbuffer (current display state) against the packed paintbuffer (next frame) and returns an array of dirty rectangles — regions where pixels have changed.

```cpp
EPD_Painter::Rect rects[32];
int n = display.computeDirtyRects(rects, 32, tolerance);
```

`tolerance` controls how aggressively clean rows are absorbed into a rectangle to avoid fragmentation:
- `0` — only consecutive dirty rows are merged (tightest, most rectangles)
- Large value — merges across clean rows (fewer, larger rectangles)
- `width * height` — collapses to a single full-screen rect

Returns the number of rectangles written to `out_rects`.

---

### `fxClear()`

Animated clear effect. A thick bar sweeps across the panel from top to bottom — a black leading edge followed by a white trailing edge — leaving the panel blank. Provides a visual clearing animation instead of an abrupt white flash.

```cpp
display.fxClear();
```

`fxClear()` does not alter the screenbuffer state. Only pixels that were already white in the screenbuffer are actively driven. Non-white pixels are left for the next `paint()` to handle via normal delta update.

---

### `clearBuffers()`

Zeroes all internal packed pixel buffers (fastbuffer, screenbuffer, paintbuffer). Resets the DC-balance baseline so that on the next boot, `unpaintPacked()` works from a clean all-white starting point.

Call this after painting a shutdown image and before cutting power:

```cpp
display.clearBuffers();
// ...then power off
```

---

### `packBuffer()`

Packs an 8bpp framebuffer into the driver's 2bpp format, respecting the current rotation setting. Returns a PSRAM-allocated buffer that the caller must free with `heap_caps_free()`. Returns `nullptr` on allocation failure.

This is the bridge between Adafruit GFX drawing and `EPD_BootCtl::IImageProvider` — it lets you draw a shutdown image with the normal GFX API and hand it to the boot control system in the correct packed format.

```cpp
// Adafruit binding — packs the internal framebuffer
uint8_t* packed = display.packBuffer();
// ... use packed ...
heap_caps_free(packed);

// Raw driver — pack your own buffer
uint8_t* packed = driver.packBuffer(my_8bpp_buffer);
```

---

### `dither()`

Floyd-Steinberg error-diffusion dither. Converts an 8bpp greyscale framebuffer (0=black … 255=white) **in-place** to the driver's 4-level encoding (0=white, 1=light grey, 2=dark grey, 3=black).

```cpp
// Adafruit binding — dithers the internal framebuffer
display.dither();

// Raw — dither your own buffer
EPD_Painter::dither(my_buffer, width, height);
```

Useful when rendering photographic content or smooth gradients into the 4-shade palette.

---

### `setQuality()`

Adjusts waveform latch timing, trading refresh speed for image quality.

| Constant | Description |
|---|---|
| `EPD_Painter::Quality::QUALITY_HIGH` | Longest latch delay — deepest blacks, slowest |
| `EPD_Painter::Quality::QUALITY_NORMAL` | Default — good balance for most use cases |
| `EPD_Painter::Quality::QUALITY_FAST` | Shortest latch delay — fastest refresh, lighter blacks |

```cpp
display.setQuality(EPD_Painter::Quality::QUALITY_FAST);
display.paint();   // quick update

display.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
display.paint();   // high quality render
```

Quality can be changed at any time between `paint()` calls.

---

## Rotation / Portrait Mode

The driver supports two orientations:

| `Rotation` | Canvas | Physical panel |
|---|---|---|
| `ROTATION_0` (default) | 960×540 landscape | 960×540 |
| `ROTATION_CW` | 540×960 portrait canvas | 960×540 (rotated 90° CW) |

Set via the config before constructing the display object:

```cpp
EPD_PainterAdafruit display(EPD_PAINTER_PRESET.withRotation(EPD_Painter::Rotation::ROTATION_CW));
```

In portrait mode, the GFX canvas dimensions are swapped (width↔height). Drawing coordinates map to the rotated physical panel. The compaction step performs the rotation in a single cache-friendly pass — no performance penalty compared to landscape.

---

## Colour Depth — Why Only Four Shades?

EPD_Painter deliberately limits the display to four shades of grey (2 bits per pixel). This is a performance decision, not a hardware limitation.

E-paper panels are driven by waveforms where each period is encoded as 2 bits. Four shades maps directly onto this — one 2bpp pixel value equals one 2-bit waveform period. The conversion is a handful of boolean operations, and the ESP32-S3 SIMD assembly handles 64 pixels at a time in a single 128-bit vector operation.

More grey levels would break this clean 1:1 mapping, requiring significantly more processing per pixel and making the pipeline too slow for real-time updates.

| Value | Shade |
|---|---|
| `0` | White |
| `1` | Light grey |
| `2` | Dark grey |
| `3` | Black |

---

## Automatic Power Management

Panel power (high-voltage supply) is managed automatically — you do not call `powerOn()` or `powerOff()` directly.

The `PanelPowerGuard` mechanism works as follows:
- Power turns **on** automatically at the start of any `paint()`, `clear()`, or `fxClear()` call
- A background task decrements an internal counter every second
- Power turns **off** approximately 5 seconds after the last painting activity

This means the panel is only energised while actively refreshing, and powers down automatically during idle periods. No explicit power management is needed in user code.

---

## Shutdown Handling

### Why an EPD driver needs shutdown management

An e-paper display retains its image indefinitely with no power. The driver maintains `packed_screenbuffer` — a software model of exactly what is physically shown — which is lost when the device powers off. On the next boot the software assumes the screen is blank (all white), but the panel still shows whatever it last displayed. Without reconciliation:

- `paint()` computes wrong deltas — it thinks pixels are white when they aren't
- Images build up as ghost layers rather than replacing each other cleanly
- EPD panels accumulate DC bias over time from particles held in one direction, causing permanent ghosting

`EPD_BootCtl` handles this automatically.

```
  Power-off                             Power-on (next boot)
  ──────────────────────────────────    ──────────────────────────────────────
  1. Paint shutdown image to panel      1. Load shutdown image from NVS
  2. Store packed image data in NVS     2. unpaintPacked() drives it off screen
  3. Power off hardware                    (screenbuffer reconciled → all white)
                                        3. Normal operation begins
```

---

### How the reset-to-shutdown mechanism works

Shutdown is triggered by pressing the hardware reset button while the device is running on battery. `EPD_BootCtl` uses NVS to track state:

- `flag = false` — device was at rest; this is a normal boot
- `flag = true` — device was running; this reset is a shutdown request

The device **arms itself on every normal boot**, so a single reset press is all that's needed.

```
  Normal boot (flag = false):
    Write flag = true  (arm — device is now running)
    If shutdown image exists in NVS:
      unpaintPacked() — drive it off the panel (DC balance)
    Normal operation proceeds

  User presses reset (flag = true):
    Write flag = false  (clear — next boot will be normal)
    shutdownPending() returns true
    If autoShutdown = true:  shutdown() called automatically [[noreturn]]
    If autoShutdown = false: deferred to user code
```

If `shutdown()` is called and the hardware power-off fails (e.g. USB is connected), `ESP.restart()` is called automatically and the device boots normally.

---

### USB power behaviour

When powered via USB, the shutdown state machine is bypassed entirely. `EPD_BootCtl` detects VBUS presence via the BQ25896 charger IC and skips all NVS flag logic. Pressing reset while on USB always produces a clean normal boot — the expected behaviour during development.

---

### Default behaviour (`setAutoShutdown(true)`)

With default settings, `begin()` handles everything transparently:

```cpp
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#include "EPD_Painter_Adafruit.h"
#include "LittleFS.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

void setup() {
    Serial.begin(115200);
    LittleFS.begin(true);

    if (!display.begin()) {      // DC-balance and shutdown handled inside begin()
        Serial.println("init failed");
        while (1);
    }

    display.clear();
    display.setTextColor(3);
    display.setCursor(40, 260);
    display.print("Press reset to power off.");
    display.paint();
}

void loop() {}
```

On a normal boot the DC-balance unpaint runs silently and the app starts. The device is now armed — pressing reset triggers shutdown automatically.

---

### Intercepting shutdown — showing a confirmation screen

Set `setAutoShutdown(false)` before `begin()`. Then create your own `EPD_BootCtl` instance in `setup()` or `loop()` and check `shutdownPending()`:

```cpp
#include "epd_painter_bootctl.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

void setup() {
    Serial.begin(115200);

    display.setAutoShutdown(false);   // handle shutdown yourself

    if (!display.begin()) {
        Serial.println("init failed");
        while (1);
    }

    EPD_BootCtl boot(display.driver());

    if (boot.shutdownPending()) {
        // Show confirmation UI, wait for user input...
        // To proceed:  boot.shutdown();      // [[noreturn]]
        // To cancel:   boot.cancelShutdown();
    }

    drawMainScreen();
    display.paint();
}
```

---

### Custom shutdown image — drawn with Adafruit GFX

The cleanest way to create a shutdown image is to draw it with the normal Adafruit GFX API and use `packBuffer()` to convert it to the packed format `EPD_BootCtl` expects.

Implement `EPD_BootCtl::IImageProvider` and draw inside `getBootImage()`:

```cpp
#include "epd_painter_bootctl.h"

class GFXShutdownImage : public EPD_BootCtl::IImageProvider {
public:
    GFXShutdownImage(EPD_PainterAdafruit& display) : _display(display) {}

    uint8_t* getBootImage(uint16_t w, uint16_t h) override {
        // Draw the shutdown screen using normal Adafruit GFX calls
        _display.fillScreen(0);
        _display.setTextColor(3);
        _display.setTextSize(4);
        _display.setCursor(240, 240);
        _display.print("Powered off");

        // Pack to 2bpp — EPD_BootCtl takes ownership and frees this buffer
        return _display.packBuffer();
    }

private:
    EPD_PainterAdafruit& _display;
};
```

Usage in `setup()`:

```cpp
EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

void setup() {
    display.setAutoShutdown(false);   // handle shutdown manually
    display.begin();

    GFXShutdownImage shutdownImage(display);
    EPD_BootCtl boot(display.driver(), shutdownImage);

    if (boot.shutdownPending()) {
        boot.shutdown();   // [[noreturn]] — draws image, stores to NVS, powers off
    }

    // Normal startup — continue drawing your application UI
    drawMainScreen();
    display.paint();
}
```

`getBootImage()` is called twice by `EPD_BootCtl` — once on shutdown (to paint the image to the panel) and once on the next startup (to unpaint it for DC balance). The drawing code runs both times, so the image is reconstructed from scratch each boot rather than stored in RAM across resets.

### Custom shutdown image — from raw data

If you prefer to supply pre-packed binary data (e.g. loaded from LittleFS or generated offline with ImageMagick), implement `getBootImage()` to return that buffer directly:

```cpp
class StoredShutdownImage : public EPD_BootCtl::IImageProvider {
public:
    uint8_t* getBootImage(uint16_t w, uint16_t h) override {
        size_t sz = (size_t)w * h / 4;
        uint8_t* buf = (uint8_t*)heap_caps_malloc(sz, MALLOC_CAP_SPIRAM);
        // Load from LittleFS, flash, etc.
        File f = LittleFS.open("/shutdown.img", "r");
        if (f) f.read(buf, sz);
        return buf;   // EPD_BootCtl frees this buffer
    }
};
```

The buffer format is raw 2bpp packed pixels — 4 pixels per byte, MSB-first, row by row:

```
Bits 7-6 → pixel 0 (leftmost)
Bits 5-4 → pixel 1
Bits 3-2 → pixel 2
Bits 1-0 → pixel 3
```

File size for 960×540: `960 × 540 / 4 = 129,600 bytes`.

To generate from an image using ImageMagick:

```bash
convert input.png -resize 960x540! -colorspace Gray \
  -posterize 4 -negate -depth 2 -type Grayscale \
  gray:shutdown.img
```

---

### `EPD_BootCtl` API summary

| Method | Description |
|---|---|
| `EPD_BootCtl(epd)` | Construct with default fractal image; reads NVS flag, runs DC-balance unpaint on normal boot |
| `EPD_BootCtl(epd, provider)` | Construct with custom `IImageProvider` |
| `shutdownPending()` | Returns `true` if a shutdown was requested on the previous reset |
| `cancelShutdown()` | Dismiss the pending shutdown — sets `shutdownPending()` to false. Next reset will be pending again |
| `shutdown()` | Paint the shutdown image, store to NVS, power off. `[[noreturn]]` |

---

## Adafruit GFX Binding

Wraps `EPD_PainterAdafruit`, which extends `GFXcanvas8`. All standard Adafruit GFX drawing functions are available — `print()`, `drawLine()`, `drawBitmap()`, `fillRect()`, `setFont()`, etc.

Pixel values are 8bpp, but only the two least-significant bits are used, giving four shades:

| Value | Shade |
|---|---|
| `0` | White |
| `1` | Light grey |
| `2` | Dark grey |
| `3` | Black |

`fillRect()` is overridden to use `memset` directly on the 8bpp buffer rather than the Adafruit GFX pixel-by-pixel loop — significantly faster for large fills.

### Complete example — M5PaperS3

```cpp
#define EPD_PAINTER_PRESET_M5PAPER_S3
#include "EPD_Painter_Adafruit.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

void setup() {
    Serial.begin(115200);

    if (!display.begin()) {
        Serial.println("Display init failed");
        while (1);
    }

    display.fillScreen(0);               // white background
    display.setTextColor(3);             // black text
    display.setTextSize(3);
    display.setCursor(40, 40);
    display.print("Hello, EPD!");

    display.drawRect(20, 20, 400, 80, 3);
    display.paint();
}

void loop() {}
```

### Complete example — LilyGo T5 S3 GPS

```cpp
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#include "EPD_Painter_Adafruit.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

void setup() {
    Serial.begin(115200);

    if (!display.begin()) {
        Serial.println("Display init failed");
        while (1);
    }

    display.clear();

    display.fillScreen(0);
    display.setTextColor(3);
    display.setTextSize(2);
    display.setCursor(10, 10);
    display.print("LilyGo T5 S3 GPS");

    display.paint();
}

void loop() {}
```

### Updating content in a loop

```cpp
int counter = 0;

void loop() {
    display.fillScreen(0);
    display.setTextColor(3);
    display.setTextSize(4);
    display.setCursor(100, 220);
    display.print(counter++);

    display.paint();   // delta update — only changed pixels are driven
    delay(500);
}
```

### Removing ghosting periodically

```cpp
void loop() {
    static int refreshCount = 0;

    if (refreshCount++ % 20 == 0) {
        display.clear();   // full clear every 20 frames
    }

    display.setTextColor(3);
    display.setCursor(50, 50);
    display.print("Frame: ");
    display.print(refreshCount);

    display.paint();
}
```

### Portrait mode

```cpp
#define EPD_PAINTER_PRESET_M5PAPER_S3
#include "EPD_Painter_Adafruit.h"

// Canvas is 540 wide × 960 tall; rotated 90° CW to the physical 960×540 panel
EPD_PainterAdafruit display(EPD_PAINTER_PRESET.withRotation(EPD_Painter::Rotation::ROTATION_CW));

void setup() {
    display.begin();
    display.fillScreen(0);
    display.setTextColor(3);
    display.setTextSize(3);
    display.setCursor(20, 20);
    display.print("Portrait");   // draws in portrait coordinates
    display.paint();
}
```

---

## LVGL v9 Binding

`EPD_PainterLVGL` registers the EPD as an LVGL display. LVGL renders directly into the shared framebuffer; the flush callback calls `paintLater()` automatically. You do not call `paint()` yourself.

**Requirements:**
- `LV_COLOR_DEPTH 8` in `lv_conf.h`
- LVGL v9

### Colour constants

Use the provided constants instead of `lv_color_black()` / `lv_color_white()`. LVGL renders RGB332; the EPD driver reads the two LSBs of the blue channel.

```cpp
EPD_PainterLVGL::WHITE     // 0 — white
EPD_PainterLVGL::LT_GREY   // 1 — light grey
EPD_PainterLVGL::DK_GREY   // 2 — dark grey
EPD_PainterLVGL::BLACK     // 3 — black
```

### Complete example — M5PaperS3 with LVGL

```cpp
#define EPD_PAINTER_PRESET_M5PAPER_S3
#include "EPD_Painter_LVGL.h"

EPD_PainterLVGL display(EPD_PAINTER_PRESET);

static uint32_t my_tick_cb(void) {
    return millis();
}

void setup() {
    Serial.begin(115200);

    lv_init();
    lv_tick_set_cb(my_tick_cb);

    if (!display.begin()) {
        Serial.println("Display init failed");
        while (1);
    }

    display.clear();

    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello LVGL!");
    lv_obj_set_style_text_color(label, EPD_PainterLVGL::BLACK, 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void loop() {
    lv_timer_handler();   // LVGL drives paint() internally via flush callback
    delay(5);
}
```

---

## `setQuality()` examples

```cpp
// Fast scrolling / animation
display.setQuality(EPD_Painter::Quality::QUALITY_FAST);
for (int i = 0; i < 100; i++) {
    display.fillScreen(0);
    display.setCursor(i * 4, 200);
    display.print(">>>");
    display.paint();
}

// Final high-quality render
display.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
display.fillScreen(0);
display.setCursor(40, 200);
display.print("Done.");
display.paint();
```

---

## Summary

| Function | Blocks? | What it does |
|---|---|---|
| `begin()` | yes | Allocate buffers, init hardware, start background paint task |
| `paint()` | until task picks up | Delta-update; background task runs the full waveform sequence |
| `paintLater()` | no | Trigger a background delta-update; returns immediately |
| `paintPacked()` | until task picks up | Paint from pre-packed 2bpp buffer |
| `unpaintPacked()` | until task picks up | DC-balance: drive packed content back toward white |
| `clear()` | yes | Drive physical panel to full white; optional rects + ClearMode |
| `clearDirtyAreas()` | yes | Compact framebuffer, compute dirty rects, clear only those |
| `computeDirtyRects()` | no | Return dirty rectangles between screen state and new framebuffer |
| `fxClear()` | yes | Animated sweep-bar clear effect |
| `clearBuffers()` | no | Zero all packed buffers — reset DC-balance baseline |
| `packBuffer()` | no | Pack 8bpp framebuffer to 2bpp — for use with `EPD_BootCtl::IImageProvider` |
| `dither()` | no | Floyd-Steinberg dither 8bpp greyscale buffer in-place to 4 levels |
| `setQuality(q)` | — | Set waveform timing for next paint |
| `setAutoShutdown(bool)` | — | If `false`, `EPD_BootCtl` must be managed manually |

**Ghosting removal recipe:**
```cpp
display.clear();       // blank the physical panel
display.paint();       // repaint current framebuffer onto clean panel
```
