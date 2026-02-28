#ifndef EPD_PAINTER_DEVICES_H
#define EPD_PAINTER_DEVICES_H

#include "EPD_Painter.h"

namespace EPD_PAPER_DEVICE {

    static EPD_Painter::Config M5PAPER_S3 = {
        .width    = 960,
        .height   = 540,
        .pin_pwr  = 46,
        .pin_sph  = 13,
        .pin_oe   = 45,
        .pin_cl   = 16,
        .pin_spv  = 17,
        .pin_ckv  = 18,
        .pin_le   = 15,
        .data_pins = { 6, 14, 7, 12, 9, 11, 8, 10 }
    };

    static EPD_Painter::Config LILYGO_T5_S3_GPS = {
        .width    = 960,
        .height   = 540,
        .pin_sph  = 41,
        .pin_cl   = 4,
        .pin_spv  = 45,
        .pin_ckv  = 48,
        .pin_le   = 42,
        .data_pins = { 5,6,7,15,16,17,18,8 },
        .i2c = {
            .sda = 39,
            .scl = 40,
            .freq = 100000
        },
        .power = {
            .pca_addr = 0x20,
            .tps_addr = 0x68,
            .vcom_mv = -1400
        }
    };

}

#endif