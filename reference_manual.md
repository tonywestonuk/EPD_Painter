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
display.clearDirtyAreas();                                        // default tolerance=0, SOFT mode
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

This is the bridge between Adafruit GFX drawing and `EPD_BootCtl` — it lets you draw a shutdown image with the normal GFX API and hand it to the boot control system in the correct packed format.

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

Quality can be changed at any time between `paint()` calls.

---

## Rotation / Portrait Mode

The driver supports two orientations:

| `Rotation` | Canvas | Physical panel |
|---|---|---|
| `ROTATION_0` (default) | 960×540 landscape | 960×540 |
| `ROTATION_CW` | 540×960 portrait canvas | 960×540 (rotated 90° CW) |

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

- Power turns **on** automatically at the start of any `paint()`, `clear()`, or `fxClear()` call
- Power turns **off** approximately 5 seconds after the last painting activity

No explicit power management is needed in user code.

---

## Shutdown Handling

### Why an EPD driver needs shutdown management

An e-paper display retains its image indefinitely with no power. The driver maintains a software model of exactly what is physically shown — which is lost when the device powers off. On the next boot the software assumes the screen is blank (all white), but the panel still shows whatever it last displayed. Without reconciliation:

- `paint()` computes wrong deltas — it thinks pixels are white when they aren't
- Images build up as ghost layers rather than replacing each other cleanly
- EPD panels accumulate DC bias over time from particles held in one direction, causing permanent ghosting

`EPD_BootCtl` handles this automatically.

```
  Power-off                             Power-on (next boot)
  ──────────────────────────────────    ──────────────────────────────────────
  1. Paint shutdown image to panel      1. Load shutdown image from NVS
  2. Store packed image data in NVS     2. Drive it back toward white (DC balance)
  3. Power off hardware                 3. Normal operation begins
```

---

### How the reset-to-shutdown mechanism works

Shutdown is triggered by pressing the hardware reset button while the device is running on battery. The device **arms itself on every normal boot**, so a single reset press is all that's needed.

When powered via USB, shutdown is bypassed entirely — pressing reset always does a clean normal boot. This is the expected behaviour during development.

---

## Examples

---

### Hello World — M5PaperS3

```cpp
// hello_world.ino
// Displays "Hello, EPD!" on the M5PaperS3.
// Change the #define to match your board.

#define EPD_PAINTER_PRESET_M5PAPER_S3
// #define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
// #define EPD_PAINTER_PRESET_LILYGO_T5_S3_H752
#include "EPD_Painter_Adafruit.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

void setup() {
    Serial.begin(115200);

    if (!display.begin()) {
        Serial.println("Display init failed — check board selection");
        while (1);
    }

    display.fillScreen(0);          // 0 = white background
    display.setTextColor(3);        // 3 = black text
    display.setTextSize(3);
    display.setCursor(40, 40);
    display.print("Hello, EPD!");

    display.drawRect(20, 20, 400, 80, 3);   // draw a black rectangle

    display.paint();                // push to display
}

void loop() {
    // nothing — static display
}
```

---

### Counter — updating the display in a loop

```cpp
// counter.ino
// Counts upward and updates the display every 500ms.
// Only changed pixels are redrawn each frame.

#define EPD_PAINTER_PRESET_M5PAPER_S3
#include "EPD_Painter_Adafruit.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

int counter = 0;

void setup() {
    Serial.begin(115200);

    if (!display.begin()) {
        Serial.println("Display init failed");
        while (1);
    }

    display.fillScreen(0);
    display.paint();
}

void loop() {
    display.fillScreen(0);          // clear framebuffer to white
    display.setTextColor(3);
    display.setTextSize(6);
    display.setCursor(100, 220);
    display.print(counter++);

    display.paint();                // only changed pixels are driven
    delay(500);
}
```

---

### Removing ghosting periodically

After many updates, faint ghost images can accumulate. Clear the panel every N frames to reset it.

```cpp
// ghosting_removal.ino
// Clears the panel every 20 frames to remove accumulated ghosting.

#define EPD_PAINTER_PRESET_M5PAPER_S3
#include "EPD_Painter_Adafruit.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

int frameCount = 0;

void setup() {
    Serial.begin(115200);

    if (!display.begin()) {
        Serial.println("Display init failed");
        while (1);
    }
}

void loop() {
    // Every 20 frames, do a full hardware clear to remove ghosting
    if (frameCount % 20 == 0) {
        display.clear();            // drives the physical panel to white
    }

    display.fillScreen(0);
    display.setTextColor(3);
    display.setTextSize(3);
    display.setCursor(40, 240);
    display.print("Frame: ");
    display.print(frameCount);

    display.paint();
    frameCount++;
    delay(200);
}
```

---

### Quality settings — fast animation then sharp final frame

```cpp
// quality_demo.ino
// Animates text scrolling across the screen using FAST quality (lighter blacks
// but higher speed), then renders a sharp final frame at HIGH quality.

#define EPD_PAINTER_PRESET_M5PAPER_S3
#include "EPD_Painter_Adafruit.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

void setup() {
    Serial.begin(115200);

    if (!display.begin()) {
        Serial.println("Display init failed");
        while (1);
    }

    // Animate with FAST quality — quickest refresh, slightly lighter blacks
    display.setQuality(EPD_Painter::Quality::QUALITY_FAST);

    for (int x = 0; x < 800; x += 8) {
        display.fillScreen(0);
        display.setTextColor(3);
        display.setTextSize(4);
        display.setCursor(x, 240);
        display.print(">>>");
        display.paint();
    }

    // Final frame at HIGH quality — deepest blacks, slowest
    display.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
    display.fillScreen(0);
    display.setTextColor(3);
    display.setTextSize(4);
    display.setCursor(300, 240);
    display.print("Done!");
    display.paint();
}

void loop() {
    // nothing
}
```

---

### Portrait mode

```cpp
// portrait.ino
// Draws in portrait orientation (540 wide x 960 tall canvas).
// The driver rotates the output to fit the physical 960x540 landscape panel.

#define EPD_PAINTER_PRESET_M5PAPER_S3
#include "EPD_Painter_Adafruit.h"

// withRotation() returns a modified copy of the preset — no other changes needed
EPD_PainterAdafruit display(EPD_PAINTER_PRESET.withRotation(EPD_Painter::Rotation::ROTATION_CW));

void setup() {
    Serial.begin(115200);

    if (!display.begin()) {
        Serial.println("Display init failed");
        while (1);
    }

    // Canvas is now 540 wide x 960 tall
    display.fillScreen(0);
    display.setTextColor(3);
    display.setTextSize(3);
    display.setCursor(20, 20);
    display.print("Portrait");
    display.setCursor(20, 80);
    display.print("540 x 960");

    display.paint();
}

void loop() {
    // nothing
}
```

---

### LVGL Hello World

```cpp
// lvgl_hello.ino
// Minimal LVGL setup. Requires: LVGL v9, LV_COLOR_DEPTH 8 in lv_conf.h

#define EPD_PAINTER_PRESET_M5PAPER_S3
#include "EPD_Painter_LVGL.h"

EPD_PainterLVGL display(EPD_PAINTER_PRESET);

void setup() {
    Serial.begin(115200);

    lv_init();
    lv_tick_set_cb([]() -> uint32_t { return millis(); });

    if (!display.begin()) {
        Serial.println("Display init failed");
        while (1);
    }

    display.clear();    // remove any ghosting before first render

    // Create a centred label using LVGL
    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello LVGL!");
    lv_obj_set_style_text_color(label, EPD_PainterLVGL::BLACK, 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void loop() {
    lv_timer_handler();     // LVGL calls paint() automatically — don't call it yourself
    delay(5);
}
```

Use the provided colour constants — standard LVGL colour functions will not produce the correct shades:

```cpp
EPD_PainterLVGL::WHITE     // 0 — white
EPD_PainterLVGL::LT_GREY   // 1 — light grey
EPD_PainterLVGL::DK_GREY   // 2 — dark grey
EPD_PainterLVGL::BLACK     // 3 — black
```

---

### Shutdown — automatic (simplest)

With default settings, pressing reset while running on battery triggers a shutdown sequence automatically. No extra code is needed.

```cpp
// shutdown_auto.ino
// Press reset while on battery to power off gracefully.
// On the next boot, the display is cleaned up automatically before the app starts.

#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#include "EPD_Painter_Adafruit.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

void setup() {
    Serial.begin(115200);

    // begin() handles everything: DC-balance on startup, shutdown on reset
    if (!display.begin()) {
        Serial.println("Display init failed");
        while (1);
    }

    display.clear();
    display.fillScreen(0);
    display.setTextColor(3);
    display.setTextSize(3);
    display.setCursor(40, 260);
    display.print("Press reset to power off.");
    display.paint();
}

void loop() {
    // nothing
}
```

The device arms itself on every normal boot. Pressing reset once triggers shutdown — a built-in fractal image is painted to the screen, pixel state is saved, and the device powers off. On the next boot the fractal is undrawn (DC balance) and the app starts normally.

---

### Shutdown — with a custom "Powered off" screen drawn using Adafruit GFX

If you want to control what appears on the screen when the device powers off, draw it yourself using the normal Adafruit GFX functions.

The boilerplate below is the connection between your drawing code and the shutdown system. **Copy it exactly and only change the drawing inside `drawShutdownScreen()`.**

```cpp
// shutdown_custom_image.ino
// Shows a custom "Powered off" screen when the device shuts down.
// Only edit the drawShutdownScreen() function — leave everything else as-is.

#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#include "EPD_Painter_Adafruit.h"
#include "epd_painter_bootctl.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

// -----------------------------------------------------------------------
// Edit this function to design your shutdown screen.
// Use any normal Adafruit GFX drawing calls.
// -----------------------------------------------------------------------
void drawShutdownScreen() {
    display.fillScreen(0);              // white background
    display.setTextColor(3);            // black text
    display.setTextSize(5);
    display.setCursor(220, 220);
    display.print("Powered off");
    display.setTextSize(2);
    display.setCursor(300, 320);
    display.print("Press reset to wake");
}

// -----------------------------------------------------------------------
// Boilerplate — copy this block exactly, do not change it
// -----------------------------------------------------------------------
class ShutdownImage : public EPD_BootCtl::IImageProvider {
    uint8_t* getBootImage(uint16_t w, uint16_t h) override {
        drawShutdownScreen();
        return display.packBuffer();    // converts drawing to packed format
    }
} shutdownImage;
// -----------------------------------------------------------------------

void setup() {
    Serial.begin(115200);

    display.setAutoShutdown(false);     // we handle shutdown ourselves below
    if (!display.begin()) {
        Serial.println("Display init failed");
        while (1);
    }

    EPD_BootCtl boot(display.driver(), shutdownImage);

    if (boot.shutdownPending()) {
        boot.shutdown();    // shows the shutdown screen and powers off — never returns
    }

    // Normal startup continues here
    display.clear();
    display.fillScreen(0);
    display.setTextColor(3);
    display.setTextSize(3);
    display.setCursor(40, 260);
    display.print("Press reset to power off.");
    display.paint();
}

void loop() {
    // nothing
}
```

> `drawShutdownScreen()` is called twice internally — once when shutting down (to paint it) and once on the next startup (to drive it back toward white for DC balance). Both times it is reconstructed from your drawing code, so nothing needs to persist across resets.

---

### Shutdown — with a confirmation dialog

Show a "Power off?" prompt and let the user cancel by pressing a button before committing to shutdown.

```cpp
// shutdown_confirm.ino
// Shows a confirmation dialog when reset is pressed.
// Press the BOOT button (GPIO 0) to cancel; do nothing to confirm after 5 seconds.

#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#include "EPD_Painter_Adafruit.h"
#include "epd_painter_bootctl.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

void drawMainScreen() {
    display.fillScreen(0);
    display.setTextColor(3);
    display.setTextSize(3);
    display.setCursor(40, 260);
    display.print("Press reset to power off.");
}

void showConfirmDialog() {
    // Draw a popup over the current screen content
    display.fillRect(240, 160, 480, 220, 0);        // white box
    display.drawRect(240, 160, 480, 220, 3);        // black border
    display.setTextColor(3);
    display.setTextSize(3);
    display.setCursor(310, 200);
    display.print("Power off?");
    display.setTextSize(2);
    display.setCursor(270, 270);
    display.print("Confirm: do nothing (5s)");
    display.setCursor(270, 310);
    display.print("Cancel:  press BOOT button");
    display.paint();
}

void setup() {
    Serial.begin(115200);
    pinMode(0, INPUT_PULLUP);           // BOOT button on GPIO 0

    display.setAutoShutdown(false);
    if (!display.begin()) {
        Serial.println("Display init failed");
        while (1);
    }

    EPD_BootCtl boot(display.driver());

    if (boot.shutdownPending()) {
        drawMainScreen();
        showConfirmDialog();

        // Wait up to 5 seconds for the user to cancel
        unsigned long deadline = millis() + 5000;
        while (millis() < deadline) {
            if (digitalRead(0) == LOW) {
                // BOOT button pressed — cancel shutdown, resume normally
                boot.cancelShutdown();
                break;
            }
            delay(50);
        }

        if (boot.shutdownPending()) {
            boot.shutdown();            // confirmed — powers off, never returns
        }

        // Cancelled — redraw the main screen
        display.fillScreen(0);
        drawMainScreen();
        display.paint();
        return;
    }

    // Normal startup
    drawMainScreen();
    display.paint();
}

void loop() {
    // nothing
}
```

---

## `EPD_BootCtl` API summary

| Method | Description |
|---|---|
| `EPD_BootCtl(epd)` | Construct with default fractal image; reads NVS flag, runs DC-balance on normal boot |
| `EPD_BootCtl(epd, provider)` | Construct with custom `IImageProvider` for a drawn shutdown screen |
| `shutdownPending()` | Returns `true` if reset was pressed while running on battery |
| `cancelShutdown()` | Dismiss the pending shutdown — next reset will be pending again |
| `shutdown()` | Paint the shutdown image, store to NVS, power off — never returns |

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
| `packBuffer()` | no | Pack 8bpp framebuffer to 2bpp — for use with custom shutdown images |
| `dither()` | no | Floyd-Steinberg dither 8bpp greyscale buffer in-place to 4 levels |
| `setQuality(q)` | — | Set waveform timing for next paint |
| `setAutoShutdown(bool)` | — | If `false`, `EPD_BootCtl` must be managed manually |

**Ghosting removal recipe:**
```cpp
display.clear();       // blank the physical panel
display.paint();       // repaint current framebuffer onto clean panel
```
