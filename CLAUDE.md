# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

EPD_Painter is a high-performance e-paper (EPD) display driver for ESP32-S3 boards, targeting the **M5PaperS3** and **LilyGo T5 S3 GPS**. It achieves ~70fps equivalent update rates via ESP32-S3 vector (SIMD) assembly, custom LCD_CAM DMA, and a delta-update pixel pipeline.

## Build System

This is an **Arduino library** (and project). Build and flash via **Arduino IDE**:
- Open `src/src.ino` as the entry point
- Required library: **Adafruit GFX Library**
- Optional library: **LVGL v9** (for LVGL-based examples)
- Board: M5Stack M5PaperS3 or LilyGo T5 S3 GPS (ESP32-S3 with PSRAM)

There is no Makefile, CMake, or CLI build system — Arduino IDE is the primary build tool.

## Architecture

### Three-Stage Pixel Pipeline

1. **8bpp GFX canvas** (PSRAM) — drawn to via Adafruit GFX or LVGL
2. **2bpp packed fastbuffer** (internal RAM) — 64-pixel chunks with bitmasks
3. **Ink drive format** — sent row-by-row over LCD_CAM DMA to the EPD panel

### Key Source Files

- `src/EPD_Painter.h/.cpp` — Core driver: hardware init, LCD_CAM DMA setup, `paint()`, `clear()`, `sendRow()` row timing (SPV/CKV/LE signals)
- `src/EPD_Painter.S` — **Xtensa assembly**: all pixel packing, waveform conversion, `ink_on`/`ink_off` delta detection, 64-pixel SIMD chunks
- `src/EPD_Painter_Adafruit.h` — Wrapper exposing Adafruit GFX API; owns 8bpp framebuffer in PSRAM
- `src/EPD_Painter_LVGL.h` — LVGL v9 integration: flush callback + paint timer
- `src/EPD_Painter_presets.h` — Board pin/I2C presets (`EPD_PAINTER_PRESET_M5PAPER_S3`, `EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS`)
- `src/epd_painter_powerctl.h/.cpp` — TPS65185 PMIC + PCA9555 IO expander control over I2C
- `src/build_opt.h` — HAL macros for Arduino vs ESP-IDF compatibility

### Board Presets

Enable via `#define` before including the library:
```cpp
#define EPD_PAINTER_PRESET_M5PAPER_S3        // 960×540, pins 6,14,7,12,9,11,8,10
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS  // 960×540, pins 5,6,7,15,16,17,18,8
```

### Delta Update System

`packed_screenbuffer` (PSRAM) tracks the physical display state. Each `paint()` call compares the new framebuffer against this state and only drives pixels that changed. Pixels needing lightening within a darkening chunk are deferred to a subsequent pass — a single `paint()` call is sufficient for complete coverage.

### Waveform System

7-pass waveforms (`lighter_waveform`, `darker_waveform` tables in `.S`) map 2bpp pixel values (00/01/10/11) to voltage drive sequences. Quality levels `QUALITY_HIGH / QUALITY_NORMAL / QUALITY_FAST` adjust latch delays.

### Memory Layout

| Buffer | Location | Size |
|---|---|---|
| 8bpp GFX canvas | PSRAM | ~518 KB |
| 2bpp fastbuffer | Internal RAM | ~130 KB |
| 2bpp screenbuffer | PSRAM | ~130 KB |
| DMA row buffers | Internal RAM | 480 B (double-buffered) |

## Hardware: LilyGo T5 S3 GPS (H752-1)

- **GPS**: u-blox M10Q, UART1 RX=44 TX=43, baud 9600 (cold boot default)
- **I2C**: SDA=39, SCL=40
- **Devices**: 0x20 (PCA9555), 0x51 (PCF8563 RTC), 0x55 (BQ27220 PMU), 0x5D (GT911 touch), 0x6B (BQ25896 charger)
- **GPS power**: PCA9555 port 0 bit 0 HIGH (`REG_CONFIG_P0=0xFE`, `REG_OUTPUT_P0=0xFF`)

## Hardware: LilyGo T5 S3 (H752)

- **I2C**: SDA=5, SCL=6
- **Devices**: 0x51 (PCF8563 RTC), 0x55 (BQ27220 PMU), 0x5D (GT911 touch), 0x6B (BQ25896 charger)

## Technical Reference

`How_It_Works.md` contains deep technical documentation covering e-paper physics, the full pixel pipeline, DMA architecture, sendRow() timing, assembly internals, and memory layout. Read this before modifying core driver code.
