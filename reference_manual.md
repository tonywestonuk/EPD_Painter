# EPD_Painter Reference Manual

A high-performance e-paper display driver for ESP32-S3 boards (M5PaperS3, LilyGo T5 S3 GPS).

---

## Quick Setup

### 1. Choose a board preset

Add one of these `#define`s before your includes:

```cpp
#define EPD_PAINTER_PRESET_M5PAPER_S3
// or
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
```

### 2. Choose a binding

| Binding | Header | Use when |
|---|---|---|
| Adafruit GFX | `EPD_Painter_Adafruit.h` | You want `print()`, `drawBitmap()`, shapes, fonts |
| LVGL v9 | `EPD_Painter_LVGL.h` | You want a full UI widget library |
| Raw driver | `EPD_Painter.h` | You manage your own 8bpp pixel buffer |

---

## Core API

### `begin()`

Allocates buffers, initialises the LCD_CAM DMA hardware, and powers on the PMIC. I2C is initialised internally — you do not need to call `Wire.begin()`.

Returns `true` on success.

```cpp
if (!display.begin()) {
    Serial.println("EPD init failed");
    while (1);
}
```

If you need access to the I2C bus (e.g. to talk to other devices on the same bus), retrieve the `Wire` instance via `getConfig()`:

```cpp
TwoWire* wire = display.getConfig().i2c.wire;
wire->beginTransmission(0x51);  // talk to another device on the bus
// ...
```

---

### `paint()`

Pushes the current framebuffer to the physical display. Uses **delta update** — only pixels that changed since the last `paint()` are driven.

`paint()` runs the first pass and returns. A second pass then runs automatically in the background to handle the case where a previously lightened pixel needs to be darkened — full coverage is achieved without blocking your code.

```cpp
display.paint();   // runs first pass, returns; second pass runs in background
```

---

### `paintLater()`

Non-blocking version of `paint()`. Hands the framebuffer to the background paint task and returns immediately. The panel refresh happens on the second core.

Use this when you want to continue work on the main core (e.g. updating sensor readings) while the display refreshes.

```cpp
display.paintLater();     // triggers a background refresh, returns immediately
// ...do other work here...
```

> **Note:** The framebuffer can be modified freely while a background refresh is in progress. The driver is designed this way — the panel always moves towards the latest framebuffer state, even if it changed mid-refresh. In the LVGL binding, `paintLater()` is called automatically inside the flush callback — you do not need to call it yourself.

---

### `clear()`

**Drives the physical panel to full white.** This is a hardware-level screen clear, not a framebuffer fill. It removes ghosting by applying a full blanking waveform to every pixel on the panel.

`clear()` does **not** clear your framebuffer. If you draw nothing before calling `paint()` after a `clear()`, you will push whatever is still in the framebuffer.

**The standard way to remove ghosting and get a clean redraw:**

```cpp
display.clear();   // blanks the physical panel
display.paint();   // pushes the current framebuffer onto the clean panel
```

> Use `clear()` sparingly. It is a full-panel refresh and takes longer than a delta `paint()`. Call it when ghosting has accumulated or before displaying something completely new.

---

### `setQuality()`

Adjusts the waveform latch timing, trading refresh speed for image quality.

| Constant | Description |
|---|---|
| `EPD_Painter::Quality::QUALITY_HIGH` | Longest latch delay — deepest blacks, slowest |
| `EPD_Painter::Quality::QUALITY_NORMAL` | Default. Good balance for most use cases |
| `EPD_Painter::Quality::QUALITY_FAST` | Shortest latch delay — fastest refresh, lighter blacks |

```cpp
display.setQuality(EPD_Painter::Quality::QUALITY_FAST);
display.paint();  // quick update
display.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
display.paint();  // high quality render
```

> Quality can be changed at any time between `paint()` calls. It takes effect on the next `paint()`.

---

## Colour Depth — Why Only Four Shades?

EPD_Painter deliberately limits the display to four shades of grey (2 bits per pixel). This is a performance decision, not a hardware limitation.

E-paper panels are driven by multi-step waveforms — each shade transition requires a sequence of voltage pulses applied over several passes. Supporting more grey levels (e.g. 16 or 32) means running proportionally more waveform passes per frame, which multiplies refresh time. At four shades the driver can complete a full-screen delta update at ~70fps equivalent, with the second pass running in the background.

The 2bpp pixel format also has a direct impact on memory bandwidth and the efficiency of the ESP32-S3 SIMD assembly that packs pixels: 64 pixels fit in a single 128-bit vector operation, keeping the pipeline fast enough to sustain real-time updates.

In short: four shades gives you a display that feels responsive and interactive. More shades would mean slower, flashier refreshes more typical of traditional e-paper use.

---

## Shutdown Image

When the board powers off, EPD_Painter displays a static image from LittleFS:

```
/.epd_painter_shutdown.img
```

If this file is not present, a Mandelbrot fractal is generated and shown as a fallback.

### File format

The file is raw 2bpp packed pixel data — the same format used internally by the driver. Each byte holds **4 pixels**, packed from MSB to LSB:

```
Bits 7-6 → pixel 0
Bits 5-4 → pixel 1
Bits 3-2 → pixel 2
Bits 1-0 → pixel 3
```

Pixel values:

| Bits | Shade |
|---|---|
| `00` | White |
| `01` | Light grey |
| `10` | Dark grey |
| `11` | Black |

File size for a 960×540 display: `960 × 540 / 4 = 129,600 bytes`.

Pixels are stored row by row, left to right, top to bottom.

### Generating a shutdown image

Convert any image to the correct format with ImageMagick — quantise to 4 grey levels and pack to 2bpp:

```bash
convert input.png -resize 960x540! -colorspace Gray \
  -posterize 4 -negate -depth 2 -type Grayscale \
  gray:shutdown.img
```

Then upload `shutdown.img` to LittleFS at the path `/.epd_painter_shutdown.img` using the Arduino LittleFS upload tool or your preferred method.

---

## Adafruit GFX Binding

Wraps `EPD_PainterAdafruit`, which extends `GFXcanvas8`. All standard Adafruit GFX drawing functions are available — `print()`, `drawLine()`, `drawBitmap()`, `fillRect()`, `setFont()`, etc.

Pixel values are 8bpp, but only the two most significant bits are used, giving four shades. Use the logical values `0`–`3`:

| Value | Shade |
|---|---|
| `0` | White |
| `1` | Light grey |
| `2` | Dark grey |
| `3` | Black |

### Complete example — M5PaperS3

```cpp
#define EPD_PAINTER_PRESET_M5PAPER_S3
#include "EPD_Painter_presets.h"
#include "EPD_Painter_Adafruit.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

void setup() {
    Serial.begin(115200);

    if (!display.begin()) {
        Serial.println("Display init failed");
        while (1);
    }

    // Draw
    display.fillScreen(0);               // white background
    display.setTextColor(3);             // black text
    display.setTextSize(3);
    display.setCursor(40, 40);
    display.print("Hello, EPD!");

    display.drawRect(20, 20, 400, 80, 3);

    // Push to panel — second pass runs automatically in background
    display.paint();
}

void loop() {}
```

### Complete example — LilyGo T5 S3 GPS

```cpp
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#include "EPD_Painter_presets.h"
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
    display.fillScreen(0);              // clear framebuffer to white
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
        // Full clear every 20 frames to eliminate ghosting,
        // then repaint the current framebuffer onto the clean panel
        display.clear();
    }

    display.setTextColor(3);
    display.setCursor(50, 50);
    display.print("Frame: ");
    display.print(refreshCount);

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
#include "EPD_Painter_presets.h"
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

    // Optional: clear ghosting on startup before first render
    display.clear();

    // Create a label
    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello LVGL!");
    lv_obj_set_style_text_color(label, EPD_PainterLVGL::BLACK, 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void loop() {
    lv_timer_handler();  // LVGL drives paint() internally via flush callback
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
| `begin()` | yes | Allocate buffers, init hardware |
| `paint()` | first pass | Delta-update; first pass blocks, second pass runs in background |
| `paintLater()` | no | Trigger a background delta-update; returns immediately |
| `clear()` | yes | Drive the **physical panel** to full white, remove ghosting |
| `setQuality(q)` | — | Set waveform timing for next paint |

**Ghosting removal recipe:**
```cpp
display.clear();       // blank the physical panel
display.paint();       // repaint current framebuffer onto clean panel
```
