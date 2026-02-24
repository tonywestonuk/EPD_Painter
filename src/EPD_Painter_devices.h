#ifndef EPD_PAINTER_DEVICES_H
#define EPD_PAINTER_DEVICES_H

#include "EPD_Painter.h"

// -------------------------------------------------------
// M5Stack Paper S3
// -------------------------------------------------------
static const EPD_Painter::Config epd_paper_dev_m5stacks3 = {
    .pin_pwr  = 46,
    .pin_sph  = 13,
    .pin_oe   = 45,
    .pin_cl   = 16,
    .pin_spv  = 17,
    .pin_ckv  = 18,
    .pin_le   = 15,
    .data_pins = { 6, 14, 7, 12, 9, 11, 8, 10 },
    .width    = 960,
    .height   = 540
};

#endif