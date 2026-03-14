#ifndef EPD_PAINTER_DEVICES_H
#define EPD_PAINTER_DEVICES_H

#include "EPD_Painter.h"

#if defined(EPD_PAINTER_PRESET_M5PAPER_S3)
    static EPD_Painter::Config EPD_PAINTER_PRESET = {
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
    };

#elif defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS)
    static EPD_Painter::Config EPD_PAINTER_PRESET = {
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
            .vcom_mv = -1000
        }
    };

#else
#error "No EPD_Painter device selected."
#endif


#endif