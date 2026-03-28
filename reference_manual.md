# EPD_Painter Reference Manual

A high-performance e-paper display driver for ESP32-S3 boards (M5PaperS3, LilyGo T5 S3 GPS, Lilygo T5 S3 H752 model).

---

## Quick Setup

### 1. Choose a board preset

Add one of these `#define`s before your includes:

```cpp
#define EPD_PAINTER_PRESET_M5PAPER_S3
// or
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
// or
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_H752
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

E-paper panels are driven by waveforms where each period is encoded as 2 bits. Four shades maps directly onto this — one 2bpp pixel value equals one 2-bit waveform period. The conversion is a handful of boolean operations, and the ESP32-S3 SIMD assembly handles 64 pixels at a time in a single 128-bit vector operation.

More grey levels would break this clean 1:1 mapping. Converting, say, 4bpp or 8bpp pixel values into 2-bit waveform periods requires significantly more memory manipulation per pixel, which would make the pipeline too slow for real-time updates.

In short: four shades is the sweet spot where the pixel format and the waveform encoding are the same thing — and that is what makes full screen updates at 20fps update rate possible.

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

---

## Shutdown Handling

### Why an EPD driver needs shutdown management

An e-paper display is different from every other display technology in one
critical way: **the image is always active, even when the device has no power.**

Pigment particles are mechanical — once moved into position by a voltage pulse,
they stay there indefinitely with no power required. This is what gives EPD its
ultra-low standby power. But it creates a problem for software.

The driver maintains `packed_screenbuffer` — a software model of exactly what
is physically shown on the panel at every pixel. This buffer lives in RAM and
is lost when the device powers off. On the next boot the software assumes the
screen is blank (all white), but the panel physically still shows whatever it
last displayed. If nothing is done to reconcile this:

- `paint()` computes wrong deltas — it thinks pixels are white when they aren't
- Images build up as "ghost" layers rather than replacing each other cleanly
- The display gradually degrades

Beyond the software state problem, EPD panels also accumulate **DC bias** over
time. Leaving particles driven hard in one direction — as happens when a dark
image sits on screen for extended periods — causes ion migration within the
fluid. Over time this manifests as permanent ghosting and reduced contrast.
Proper shutdown and startup procedures prevent this by driving pixels to a
known neutral state.

EPD_Painter handles all of this automatically through the `EPD_PainterShutdown`
class, created by `begin()`.

```
  Power-off                             Power-on (next boot)
  ──────────────────────────────────    ──────────────────────────────────────
  1. Paint shutdown image to panel      1. Load shutdown image from LittleFS
  2. Save packed image to LittleFS      2. unpaintPacked() drives it off screen
  3. Power off hardware                    (screenbuffer reconciled → all white)
                                        3. Normal operation begins
```

The shutdown image serves two purposes: it gives the user a clean resting
image (rather than a frozen UI), and its known pixel values let the driver
reconcile the screenbuffer on the next startup by unpainting it — i.e.
driving every pixel away from the shutdown state and back toward white.

---

### How the reset-to-shutdown mechanism works

Shutdown is triggered by pressing the hardware reset button **once** while the
device is running on battery. The driver uses NVS (non-volatile storage) to
track state across resets:

```
  NVS key "shutdown" values:
    0 = unarmed — device has not been running yet (e.g. fresh after shutdown)
    1 = armed   — device has been running; next reset will trigger shutdown
    2 = force   — always proceed immediately, no popup
```

The key insight is that the device **arms itself on every normal boot**. So by
the time the user presses reset, the flag is already set to `1` — a single
press is all that's needed.

```
  Normal boot (key = 0):
  ┌─────────────────────────────────────────────────────┐
  │  Read key = 0                                       │
  │  Write key = 1  (arm — device is now running)       │
  │  If shutdown image exists in LittleFS:              │
  │    unpaintPacked() — drive it off the panel         │
  │    screenbuffer is now reconciled to all-white      │
  │  Normal operation proceeds                          │
  └─────────────────────────────────────────────────────┘

  User presses reset (key = 1):
  ┌─────────────────────────────────────────────────────┐
  │  Read key = 1                                       │
  │  Write key = 0  (clear flag)                        │
  │  isPending() = true                                 │
  │  If autoShutdown = true:  proceed() called          │
  │  If autoShutdown = false: deferred to user code     │
  └─────────────────────────────────────────────────────┘
```

After `proceed()` completes and the device powers off, the flag is `0`. The
next power-on is a normal boot that re-arms to `1` — the cycle repeats.

If `proceed()` is called and the hardware power-off fails (e.g. USB was
connected after the reset), `ESP.restart()` is called automatically. The
device then boots cleanly as a normal startup (key = 0).

---

### USB power behaviour

When the device is powered via USB, the shutdown state machine is bypassed
entirely. `EPD_PainterShutdown` detects VBUS presence via the BQ25896 charger
IC (I2C register 0x0B, VBUS_STAT bits) and returns from the constructor
immediately, leaving `isPending()` false and no flag written to NVS.

This means pressing reset while USB is connected always produces a clean normal
boot — which is exactly the right behaviour during development.

---

### Default behaviour (autoShutdown = true)

With the default settings, `begin()` handles everything transparently:

```cpp
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#include "EPD_Painter_presets.h"
#include "EPD_Painter_Adafruit.h"
#include "LittleFS.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

void setup() {
    Serial.begin(115200);
    LittleFS.begin(true);

    if (!display.begin()) {      // shutdown is handled inside begin() automatically
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

On a normal boot the DC-balance runs silently and the app starts. The device
is now armed — pressing reset will trigger shutdown.

---

### Intercepting shutdown — showing a confirmation screen

Set `setAutoShutdown(false)` before `begin()` so that `proceed()` is not called
automatically. Then check `isPending()` in `setup()` or `loop()` and show your
own UI before deciding whether to proceed or cancel.

A complete working example is provided at
`examples/adafruit/shutdown_confirmation/`. It shows a 5-second countdown on
the EPD with a prompt to press the BOOT button (GPIO 0) to cancel. If the
countdown expires the device shuts down; if the button is pressed, shutdown is
cancelled and the main screen is restored.

The key structure:

```cpp
EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

void setup() {
    Serial.begin(115200);
    LittleFS.begin(true);

    display.setAutoShutdown(false);   // handle shutdown yourself

    if (!display.begin()) {
        Serial.println("init failed");
        while (1);
    }

    display.clear();
    drawMainScreen();
    display.paint();
}

static bool shutdownPopupShown = false;

void loop() {
    if (display.shutdown()->isPending() && !shutdownPopupShown) {
        shutdownPopupShown = true;
        showShutdownConfirmation();   // draw your popup
        display.paint();
    }

    // Later — user presses a button or taps the touchscreen:
    // display.shutdown()->proceed();   // confirm — show shutdown image, power off
    // display.shutdown()->cancel();    // cancel — re-arm the flag, resume normal use
}

void showShutdownConfirmation() {
    display.fillRect(280, 190, 400, 160, 1);   // popup background
    display.drawRect(280, 190, 400, 160, 3);
    display.setTextColor(3);
    display.setTextSize(2);
    display.setCursor(340, 220);
    display.print("Power off?");
    display.setCursor(310, 260);
    display.print("[Reset] confirm  [Touch] cancel");
}
```

> **`cancel()` re-arms the flag.** After cancellation, pressing reset again
> will trigger `isPending()` once more. This matches the expected behaviour —
> the user still has a way to shut down after cancelling.

---

### Running code before power-off — pre-shutdown callback

Register a callback that runs at the start of every `proceed()` call, before
the shutdown image is painted and the device powers off. Use this to send a
final network message, flush data to storage, or log a shutdown event.

```cpp
display.shutdown()->setPreShutdownCallback([]() {
    Serial.println("Shutting down — sending goodbye packet...");
    network.sendShutdownNotice();
    delay(200);   // allow time for transmission
});
```

The callback runs regardless of whether shutdown was triggered by a double
reset, an idle timer expiry, or a direct call to `shutdown()`.

---

### Idle auto-off timer

Start a countdown on boot. Any user activity resets the timer. When the timer
expires the device shuts down automatically.

```cpp
void setup() {
    // ...
    display.begin();

    // Power off after 30 minutes of inactivity
    display.shutdown()->startIdleTimer(30 * 60);
}

void loop() {
    if (userTouchedScreen() || buttonPressed()) {
        display.shutdown()->resetIdleTimer();   // restart the countdown
    }
}
```

The timer calls `shutdown(true)` on expiry — force mode, no popup, immediate
proceed. Cancel it entirely if your application manages its own sleep logic:

```cpp
display.shutdown()->cancelIdleTimer();
```

---

### Triggering shutdown from code

Call `shutdown()` directly to write the NVS flag and restart into the shutdown
sequence. This is the same path used by the idle timer.

```cpp
// Normal path — if autoShutdown=false, isPending() becomes true on the
// next boot and your popup can still intervene.
display.shutdown()->shutdown();

// Force path — always proceeds immediately, no popup.
display.shutdown()->shutdown(true);
```

---

### LVGL — shutdown with a popup widget

In an LVGL application, the recommended pattern is to create the shutdown popup
as a hidden LVGL object and make it visible when `isPending()` is true. The
flush callback handles painting automatically.

```cpp
EPD_PainterLVGL display(EPD_PAINTER_PRESET);
static lv_obj_t *shutdown_popup = nullptr;

void create_shutdown_popup() {
    shutdown_popup = lv_obj_create(lv_layer_top());
    lv_obj_set_size(shutdown_popup, 360, 160);
    lv_obj_align(shutdown_popup, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(shutdown_popup, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *label = lv_label_create(shutdown_popup);
    lv_label_set_text(label, "Power off?");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 16);

    lv_obj_t *btn_yes = lv_button_create(shutdown_popup);
    lv_obj_set_size(btn_yes, 120, 48);
    lv_obj_align(btn_yes, LV_ALIGN_BOTTOM_LEFT, 16, -16);
    lv_obj_t *lbl_yes = lv_label_create(btn_yes);
    lv_label_set_text(lbl_yes, "Power off");
    lv_obj_add_event_cb(btn_yes, [](lv_event_t*) {
        display.shutdown()->proceed();
    }, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *btn_no = lv_button_create(shutdown_popup);
    lv_obj_set_size(btn_no, 120, 48);
    lv_obj_align(btn_no, LV_ALIGN_BOTTOM_RIGHT, -16, -16);
    lv_obj_t *lbl_no = lv_label_create(btn_no);
    lv_label_set_text(lbl_no, "Cancel");
    lv_obj_add_event_cb(btn_no, [](lv_event_t*) {
        display.shutdown()->cancel();
        lv_obj_add_flag(shutdown_popup, LV_OBJ_FLAG_HIDDEN);
    }, LV_EVENT_CLICKED, nullptr);
}

void setup() {
    lv_init();
    lv_tick_set_cb([]() -> uint32_t { return millis(); });

    display.setAutoShutdown(false);
    display.begin();
    create_shutdown_popup();

    display.shutdown()->startIdleTimer(60 * 60);   // 1 hour idle timeout
}

void loop() {
    if (display.shutdown()->isPending()) {
        lv_obj_remove_flag(shutdown_popup, LV_OBJ_FLAG_HIDDEN);
    }
    lv_timer_handler();
    delay(5);
}
```

---

### EPD_PainterShutdown API summary

| Method | Description |
|---|---|
| `isPending()` | Returns `true` if a shutdown was requested on the previous reset |
| `proceed()` | Paint shutdown image, power off. Does nothing if `!isPending()` |
| `cancel()` | Abort pending shutdown and re-arm the flag. Does nothing if `!isPending()` |
| `shutdown(force)` | Write NVS flag and restart. `force=true` bypasses popups |
| `setPreShutdownCallback(fn)` | Register a `void()` callback run before every `proceed()` |
| `startIdleTimer(seconds)` | Start a one-shot idle auto-off countdown |
| `resetIdleTimer()` | Restart the idle countdown from zero |
| `cancelIdleTimer()` | Stop and destroy the idle timer |

---

## Summary

| Function | Blocks? | What it does |
|---|---|---|
| `begin()` | yes | Allocate buffers, init hardware, create shutdown handler |
| `paint()` | first pass | Delta-update; first pass blocks, second pass runs in background |
| `paintLater()` | no | Trigger a background delta-update; returns immediately |
| `clear()` | yes | Drive the **physical panel** to full white, remove ghosting |
| `setQuality(q)` | — | Set waveform timing for next paint |
| `setAutoShutdown(bool)` | — | If `false`, `isPending()` must be checked manually in `loop()` |
| `shutdown()` | — | Returns the `EPD_PainterShutdown*` instance created by `begin()` |

**Ghosting removal recipe:**
```cpp
display.clear();       // blank the physical panel
display.paint();       // repaint current framebuffer onto clean panel
```
