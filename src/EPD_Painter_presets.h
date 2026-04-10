#ifndef EPD_PAINTER_DEVICES_H
#define EPD_PAINTER_DEVICES_H

#include "EPD_Painter.h"

#if defined(EPD_PAINTER_PRESET_LILYGO_EPD47_H716) || defined(EPD_PAINTER_PRESET_AUTO)
// -----------------------------------------------------------------------
// LilyGo T5 4.7" EPD47 H716 (ESP32-S3 variant, capacitive touch)
//   74HCT4094D shift register with H716-specific bit layout:
//     QP0=EP_LE, QP1=PWR_DIS, QP2=POS_PWR, QP3=NEG_PWR,
//     QP4=EP_STV, QP5=SCAN_DIR, QP6=EP_MODE, QP7=EP_OE
//   CKV, STH, CKH and the 8-bit data bus remain direct GPIOs.
// -----------------------------------------------------------------------
    inline EPD_Painter::Config EPD_LILYGO_EPD47_H716_PRESET = {
        .width    = 960,
        .height   = 540,
        .pin_pwr  = -1,             // shift register (power managed by H716 power driver)
        .pin_sph  = 40,             // STH  — direct GPIO
        .pin_oe   = -1,             // shift register QP7
        .pin_cl   = 41,             // CKH  — direct GPIO
        .pin_spv  = EPD_SR_PIN(4),  // EP_STV — shift register QP4
        .pin_ckv  = 38,             // CKV   — direct GPIO
        .pin_le   = EPD_SR_PIN(0),  // EP_LE — shift register QP0
        .quality  = EPD_Painter::Quality::QUALITY_NORMAL,
        .data_pins = { 8, 1, 2, 3, 4, 5, 6, 7 },  // D0–D7
        .i2c = { .sda = 18, .scl = 17, .freq = 100000 },
        .power = { .pca_addr = -1, .tps_addr = -1 },
        .waveforms = {
            .fast_lighter   = { { 1, 3, 2, 3, 2, 2, 3 },
                                { 3, 2, 3, 2, 2, 3, 2 },
                                { 2, 2, 2, 2, 2, 2, 2 } },
            .fast_darker    = { { 3, 1, 3, 2, 1, 1, 3 },
                                { 1, 3, 1, 3, 1, 1, 3 },
                                { 1, 1, 1, 1, 1, 1, 1 } },
            .normal_lighter = { { 1, 1, 1, 1, 2, 3, 3, 2, 2, 2, 2, 2, 2 },
                                { 2, 1, 2, 2, 1, 2, 2, 2, 0, 0, 2, 2, 3 },
                                { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
            .normal_darker  = { { 1, 2, 1, 3, 1, 3, 1, 2, 2, 1, 2, 1, 1 },
                                { 1, 1, 1, 2, 2, 3, 1, 1, 3, 2, 1, 1, 3 },
                                { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
            .high_lighter   = { { 1, 3, 1, 1, 1, 2, 1, 2, 2, 2, 2, 2, 2 },
                                { 1, 1, 2, 2, 1, 2, 2, 2, 2, 1, 2, 2, 2 },
                                { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
            .high_darker    = { { 1, 3, 1, 1, 1, 2, 2, 2, 1, 2, 2, 1, 1 },
                                { 1, 1, 1, 1, 2, 2, 1, 1, 2, 1, 2, 1, 1 },
                                { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
        },
        .shift = { .data = 13, .clk = 12, .strobe = 0, .le_time = 0,
                   .driver = EPD_Painter::Shift::H716 },
    };
#endif

#if defined(EPD_PAINTER_PRESET_M5PAPER_S3) || defined(EPD_PAINTER_PRESET_AUTO)
    inline EPD_Painter::Config EPD_M5PAPER_S3_PRESET = {
        .width    = 960,
        .height   = 540,
        .pin_pwr    = 46,
        .pin_syspwr = 44,
        .pin_sph  = 13,
        .pin_oe   = 45,
        .pin_cl   = 16,
        .pin_spv  = 17,
        .pin_ckv  = 18,
        .pin_le   = 15,
        .quality  = EPD_Painter::Quality::QUALITY_NORMAL,
        .data_pins = { 6, 14, 7, 12, 9, 11, 8, 10 },
        .i2c = {
            .sda = 41,
            .scl = 42,
            .freq = 100000
        },
        .waveforms = {
            .fast_lighter   = { { 1, 2, 2, 2, 2, 2, 3 },
                                { 3, 2, 2, 2, 2, 2, 3 },
                                { 2, 2, 2, 2, 2, 2, 2 } },
            .fast_darker    = { { 1, 1, 3, 3, 1, 3, 1 },
                                { 3, 1, 1, 1, 1, 1, 3 },
                                { 1, 1, 1, 1, 1, 1, 1 } },
            .normal_lighter = { { 1, 3, 1, 1, 2, 2, 1, 2, 2, 2, 2, 2, 2 },
                                { 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
                                { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
            .normal_darker  = { { 1, 1, 1, 3, 1, 1, 1, 1, 2, 1, 2, 2, 2 },
                                { 1, 3, 1, 3, 1, 2, 2, 3, 1, 1, 1, 1, 1 },
                                { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
            .high_lighter   = { { 1, 3, 1, 1, 1, 2, 1, 2, 2, 2, 2, 2, 2 },
                                { 1, 3, 3, 1, 3, 2, 2, 2, 2, 2, 2, 2, 2 },
                                { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
            .high_darker    = { { 1, 3, 1, 1, 2, 2, 2, 1, 2, 1, 1, 2, 1 },
                                { 3, 1, 1, 1, 2, 1, 1, 1, 1, 2, 1, 1, 2 },
                                { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
        },
    };
#endif
#if defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS) || defined(EPD_PAINTER_PRESET_AUTO)
    inline EPD_Painter::Config EPD_LILYGO_T5_S3_GPS_PRESET = {
        .width    = 960,
        .height   = 540,
        .pin_pwr  = -1,  // managed via TPS65185 over I2C (powerctl)
        .pin_sph  = 41,
        .pin_oe   = -1,  // managed via PCA9555 over I2C (powerctl)
        .pin_cl   = 4,
        .pin_spv  = 45,
        .pin_ckv  = 48,
        .pin_le   = 42,
        .quality  = EPD_Painter::Quality::QUALITY_NORMAL,
        .data_pins = { 5,6,7,15,16,17,18,8 },
        .i2c = {
            .sda = 39,
            .scl = 40,
            .freq = 100000
        },
        .power = {
            .pca_addr = 0x20,
            .tps_addr = 0x68,
        },
        .waveforms = {
            .fast_lighter   = { { 1, 2, 2, 2, 2, 2, 3 },
                                { 3, 2, 2, 2, 2, 2, 3 },
                                { 2, 2, 2, 2, 2, 2, 2 } },
            .fast_darker    = { { 1, 1, 3, 3, 1, 3, 1 },
                                { 1, 3, 1, 1, 1, 1, 3 },
                                { 1, 1, 1, 1, 1, 1, 1 } },
            .normal_lighter = { { 1, 1, 1, 1, 2, 2, 3, 2, 2, 2, 2, 2, 2 },
                                { 1, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3 },
                                { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
            .normal_darker  = { { 1, 2, 1, 1, 1, 3, 1, 2, 2, 1, 2, 1, 1 },
                                { 1, 1, 1, 2, 2, 3, 1, 1, 3, 1, 3, 1, 1 },
                                { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
            .high_lighter   = { { 1, 3, 1, 1, 1, 2, 1, 2, 2, 2, 2, 2, 2 },
                                { 1, 1, 3, 1, 3, 2, 2, 2, 2, 2, 2, 2, 2 },
                                { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
            .high_darker    = { { 1, 3, 1, 1, 1, 2, 2, 2, 1, 2, 2, 1, 1 },
                                { 1, 1, 1, 1, 2, 2, 1, 1, 2, 1, 2, 1, 1 },
                                { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
        },
    };
#endif
// -----------------------------------------------------------------------
// LilyGo T5 S3 H752 (older variant with 74HCT4094D shift register)
//   74HCT4094D outputs used by the panel driver:
//     QP0 -> EP_LE, QP4 -> EP_STV, QP5 -> PWR_EN, QP6 -> EP_MODE, QP7 -> EP_OE
//   CKV/CKH/STH and the 8-bit data bus remain direct GPIOs.
// -----------------------------------------------------------------------
#if defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_H752) || defined(EPD_PAINTER_PRESET_AUTO)
    inline EPD_Painter::Config EPD_LILYGO_T5_S3_H752_PRESET = {
        .width    = 960,
        .height   = 540,
        .pin_pwr  = -1,            // shift register (power managed by shiftctl->powerOn/Off)
        .pin_sph  = 9,             // STH  — direct GPIO
        .pin_oe   = -1,            // shift register (power managed by shiftctl->powerOn/Off)
        .pin_cl   = 10,            // CKH  — direct GPIO
        .pin_spv  = EPD_SR_PIN(4), // EP_STV / SPV — shift register QP4
        .pin_ckv  = 39,            // CKV  — direct GPIO
        .pin_le   = EPD_SR_PIN(0), // EP_LE — shift register QP0
        .quality  = EPD_Painter::Quality::QUALITY_NORMAL,
        .data_pins = { 11, 12, 13, 14, 21, 47, 45, 38 },
        .i2c = { .sda = 6, .scl = 5, .freq = 100000 },   // I2C bus exposed for peripherals on H752
        .power = { .pca_addr = -1, .tps_addr = -1 },
        .waveforms = {
            .fast_lighter   = { { 1, 3, 2, 3, 2, 2, 3 },
                                { 3, 2, 3, 2, 2, 3, 2 },
                                { 2, 2, 2, 2, 2, 2, 2 } },
            .fast_darker    = { { 3, 1, 3, 2, 1, 1, 3 },
                                { 1, 3, 1, 3, 1, 1, 3 },
                                { 1, 1, 1, 1, 1, 1, 1 } },
            .normal_lighter = { { 1, 1, 1, 1, 2, 3, 3, 2, 2, 2, 2, 2, 2 },
                                { 2, 1, 2, 2, 1, 2, 2, 2, 0, 0, 2, 2, 3 },
                                { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
            .normal_darker  = { { 1, 2, 1, 3, 1, 3, 1, 2, 2, 1, 2, 1, 1 },
                                { 1, 1, 1, 2, 2, 3, 1, 1, 3, 2, 1, 1, 3 },
                                { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
            .high_lighter   = { { 1, 3, 1, 1, 1, 2, 1, 2, 2, 2, 2, 2, 2 },
                                { 1, 1, 2, 2, 1, 2, 2, 2, 2, 1, 2, 2, 2 },
                                { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
            .high_darker    = { { 1, 3, 1, 1, 1, 2, 2, 2, 1, 2, 2, 1, 1 },
                                { 1, 1, 1, 1, 2, 2, 1, 1, 2, 1, 2, 1, 1 },
                                { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
        },
        .shift = { .data = 2, .clk = 42, .strobe = 1, .le_time = 0 },
    };

#endif

#if defined(EPD_PAINTER_PRESET_M5PAPER_S3)
    inline EPD_Painter::Config& EPD_PAINTER_PRESET = EPD_M5PAPER_S3_PRESET;
#elif defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS)
    inline EPD_Painter::Config& EPD_PAINTER_PRESET = EPD_LILYGO_T5_S3_GPS_PRESET;
#elif defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_H752)
    inline EPD_Painter::Config& EPD_PAINTER_PRESET = EPD_LILYGO_T5_S3_H752_PRESET;
#elif defined(EPD_PAINTER_PRESET_LILYGO_EPD47_H716)
    inline EPD_Painter::Config& EPD_PAINTER_PRESET = EPD_LILYGO_EPD47_H716_PRESET;
#elif defined(EPD_PAINTER_PRESET_AUTO)
// Auto-select preset based on target board (via preprocessor macros defined by the build system)
    inline EPD_Painter::Config EPD_PAINTER_PRESET = {
    .width    = 960,
    .height   = 540,
    .pin_pwr  = -1,
    .pin_sph  = -1,
    .pin_oe   = -1,
    .pin_cl   = -1,
    .pin_spv  = -1,
    .pin_ckv  = -1,
    .pin_le   = -1,
    .quality  = EPD_Painter::Quality::QUALITY_NORMAL,
    .data_pins = { -1, -1, -1, -1, -1, -1, -1, -1 },
    .waveforms = { },
};

inline EPD_Painter::ProbeSettings Probe[] = {
    // Boards with unique I2C addresses go first to minimise failed-probe noise.
    { &EPD_LILYGO_T5_S3_GPS_PRESET,      39, 40, 0x20, false }, // PCA9555  — unique addr
    { &EPD_LILYGO_EPD47_H716_PRESET,     18, 17, 0x5D, false }, // GT911    — unique addr
    // Boards sharing 0x51 (BM8563 RTC) are distinguished by their SCL/SDA pins.
    { &EPD_LILYGO_T5_S3_H752_PRESET,      6,  5, 0x51, false }, // BM8563 on bus 6/5
    { &EPD_M5PAPER_S3_PRESET,            41, 42, 0x51, false }, // BM8563 on bus 41/42
    // Add new board probes here, ensuring they have unique I2C addresses and pins
};
#else
    #error "No EPD_PAINTER_PRESET defined; please define one of the presets or set EPD_PAINTER_PRESET_AUTO"
#endif

#endif
