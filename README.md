![EPD Painter logo](img/epdpainter_logo.png)

# EPD_Painter

A high-performance e-paper display driver for ESP32-S3, targeting the **M5Stack M5PaperS3** and **LilyGo T5 S3 GPS**. Achieves ~20fps equivalent update rates (FAST mode) through a combination of ESP32-S3 vector (SIMD) assembly, a custom LCD_CAM DMA pipeline, and a delta-update pixel system.

---

## What makes this different

Most ESP32-S3 e-paper drivers use the standard `esp_lcd_panel_io_i80` interface for pushing pixels. That interface has significant overhead and is the main bottleneck for high-speed EPD updates. EPD_Painter bypasses it entirely and drives the LCD_CAM peripheral directly — an approach inspired by [this Adafruit blog post](https://blog.adafruit.com/2022/06/21/esp32uesday-more-s3-lcd-peripheral-hacking-with-code/).

On top of that:

- **ESP32-S3 vector (SIMD) assembly** — pixel packing, waveform conversion, and delta detection are done 64 pixels at a time using 128-bit vector registers
- **Delta updates** — only pixels that have changed since the last frame are driven.
- **Dual-core pipeline** — the waveform rendering runs in the background core while your code continues on the main core.
- **4-shade greyscale** (white, light grey, dark grey, black) — the 2bpp pixel format maps directly onto the EPD waveform encoding with minimal conversion, keeping the pipeline fast

---

## Supported boards

| Board | Resolution | Preset |
|---|---|---|
| M5Stack M5PaperS3 | 960×540 | `EPD_PAINTER_PRESET_M5PAPER_S3` |
| LilyGo T5 S3 GPS | 960×540 | `EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS` |

---

## Installation

1. Clone this repository into your Arduino `libraries` folder:
   ```
   git clone https://github.com/your-repo/EPD_Painter ~/Arduino/libraries/EPD_Painter
   ```
2. Install the **Adafruit GFX Library** (required) via Arduino Library Manager
3. Install **LVGL v9** (optional — needed for LVGL examples) via Arduino Library Manager
4. Select your board in Arduino IDE: M5Stack M5PaperS3 or LilyGo T5 S3 GPS (ESP32-S3 with PSRAM)
5. Add one `#define` before your includes to select your board preset (see Quick Start below)

---

## Quick start

```cpp
// Uncomment the define that matches your board.
//#define EPD_PAINTER_PRESET_M5PAPER_S3
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_H752
#include "EPD_Painter_presets.h"
#include "EPD_Painter_Adafruit.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

void setup() {
    display.begin();

    display.fillScreen(0);      // white background
    display.setTextColor(3);    // black text
    display.setTextSize(3);
    display.setCursor(40, 40);
    display.print("Hello, EPD!");

    display.paint();
}

void loop() {}
```

---

## Bindings

| Binding | Header | Description |
|---|---|---|
| Adafruit GFX | `EPD_Painter_Adafruit.h` | Full Adafruit GFX API — `print()`, `drawBitmap()`, shapes, custom fonts |
| LVGL v9 | `EPD_Painter_LVGL.h` | Complete LVGL widget library; flush callback wired automatically |
| Raw driver | `EPD_Painter.h` | Bring your own 8bpp framebuffer |

---

## Core API

| Function | Description |
|---|---|
| `begin()` | Allocate buffers, init hardware and I2C |
| `paint()` | Delta-update the panel; first pass blocks, second pass runs in background |
| `paintLater()` | Fully non-blocking paint — hands off to background task immediately |
| `clear()` | Drive the **physical panel** to full white; removes ghosting |
| `setQuality(q)` | `QUALITY_HIGH` / `QUALITY_NORMAL` / `QUALITY_FAST` — trades refresh speed for black depth |

**Ghosting removal:**
```cpp
display.clear();   // blank the physical panel
display.paint();   // repaint current framebuffer onto the clean panel
```

See [reference_manual.md](reference_manual.md) for full documentation with copy-paste examples.

---

## Colour depth

EPD_Painter uses 4 shades of grey (2 bits per pixel). This is a deliberate performance choice — more grey levels require proportionally more waveform passes per frame, multiplying refresh time. At 4 shades the driver keeps updates fast enough to feel interactive.

| Value | Shade |
|---|---|
| `0` | White |
| `1` | Light grey |
| `2` | Dark grey |
| `3` | Black |

For the LVGL binding, use the provided colour constants (`EPD_PainterLVGL::WHITE`, `::LT_GREY`, `::DK_GREY`, `::BLACK`) rather than standard LVGL colour functions.

---

## Shutdown handling.

EPD_Painter checks a shutdown flag on initalisation. When the device is reset (either by software, or pressing the reset button), if this is set to shutdown, it will begin a shutdown sequence:
- The Existing image is erased. (Image data stored in PSRAM data survives a reset)
- The screen is cleared
- an shutdown image '/.epd_painter_shutdown.img' is loaded from little_fs, and output to the EPD.
- Device is powered off.

If the flag is set to startup normally, then:
- The shutdown image is first loaded from little_fs and is 'unpainted' from the EPD screen. This keeps DC balance by sending the reverse pulses to the screen that were sent, when the device was first turned off.
- The screen is cleared.
- Device continues to boot normally.

If the file is not present, a Mandelbrot fractal is generated as a fallback. The file format is raw 2bpp packed pixels — 4 pixels per byte, `00`=white through `11`=black — matching the driver's internal format.

You can create your own shutdown screen by using ImageMagick, as follows. Once created upload it to your device's little_fs partition.

```bash
convert input.png -resize 960x540! -colorspace Gray \
  -posterize 4 -negate -depth 2 -type Grayscale \
  gray:.epd_painter_shutdown.img
```

---

## Examples

| Example | Binding | Description |
|---|---|---|
| `adafruit/bounce` | Adafruit GFX | Bouncing ball animation |
| `adafruit/breakout` | Adafruit GFX | A breakout game animation |
| `adafruit/circles` | Adafruit GFX | Animated circles |
| `adafruit/page_text` | Adafruit GFX | Text rendering |
| `lvgl/lvgl_hello_world` | LVGL | Minimal LVGL setup |
| `lvgl/lvgl_gps_clock` | LVGL | GPS-synced clock (LilyGo) |
| `lvgl/lvgl_gps_receiver` | LVGL | Live GPS data display (LilyGo) |
| `lvgl/lvgl_lighting_control` | LVGL | UI control panel demo |
| `other/elevated` | Raw | ESP32 port of the Elevated 4K intro |
| `other/drift` | Raw | Julia set morphing animation |
| `other/gps_raw_serial` | Raw | Raw NMEA output from u-blox GPS (LilyGo) |
| `other/lilygo_t5_shutdown` | Raw | Graceful shutdown with LittleFS image |
| `map_viewer` | Raw | Live OpenStreetMap tile viewer via GPS |

---

## Architecture overview

```
 ┌─────────────────────────┐
 │  8bpp GFX canvas (PSRAM)│  ← Adafruit GFX / LVGL draws here
 └────────────┬────────────┘
              │ SIMD pixel packing (assembly)
 ┌────────────▼────────────┐
 │ 2bpp fastbuffer (IRAM)  │  ← 64-pixel chunks, bitmask delta detection
 └────────────┬────────────┘
              │ waveform lookup + DMA
 ┌────────────▼────────────┐
 │   LCD_CAM DMA → panel   │  ← row-by-row, SPV/CKV/LE timing signals
 └─────────────────────────┘
```

Memory usage on a 960×540 display:

| Buffer | Location | Size |
|---|---|---|
| 8bpp GFX canvas | PSRAM | ~518 KB |
| 2bpp fastbuffer | Internal RAM | ~130 KB |
| 2bpp screenbuffer | PSRAM | ~130 KB |
| DMA row buffers | Internal RAM | 480 B (double-buffered) |
