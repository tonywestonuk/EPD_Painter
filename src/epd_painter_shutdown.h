#pragma once

#include "EPD_Painter.h"

// =============================================================================
// EPD_PainterShutdown
//
// Reset-button toggle shutdown handler.
//
// Called once from EPD_Painter::begin() after the paint task is running.
//
//   Reset 1 (flag absent/false):  arm the flag, return — normal boot.
//   Reset 2 (flag present/true):  clear flag, show "Powered Off", power off.
//     If USB keeps the device alive after powerOff(), ESP.restart() is called
//     so the device reboots into normal operation rather than hanging.
//
// Board-specific power-off:
//   EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS  — PPM.shutdown() via BQ25896
//                                          (requires #define XPOWERS_CHIP_BQ25896
//                                           and XPowersLib before including this)
//   EPD_PAINTER_PRESET_M5PAPER_S3        — GPIO 44 HIGH→LOW pulse
//
// NVS namespace: "epd", key: "shutdown"
// =============================================================================
class EPD_PainterShutdown {
public:
    EPD_PainterShutdown(EPD_Painter* epd);
private:
    EPD_Painter* _epd;
    void showMand();
    void showMandAndPaint();
    void powerOff();
};
