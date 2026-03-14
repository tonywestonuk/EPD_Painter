#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <functional>
#include "EPD_Painter.h"

// =============================================================================
// EPD_PainterShutdown
//
// Reset-button toggle shutdown handler.
//
//   Reset 1 (flag absent/false):  arm the flag, DC-balance the screen,
//                                  return — normal boot.
//   Reset 2 (flag present/true):  clear flag, set isPending() true, return.
//                                  Call proceed() to show shutdown image and
//                                  power off, or cancel() to abort.
//
// Typical usage:
//   In setup():  EPD_PainterShutdown sd(&display.driver());
//   In loop():   if (sd.isPending() && !popup) show_shutdown_popup();
//                // popup calls sd.proceed() or sd.cancel()
//
// Board-specific power-off:
//   EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS  — BQ25896 BATFET disable via I2C
//   EPD_PAINTER_PRESET_M5PAPER_S3        — GPIO 44 HIGH->LOW pulse
//
// NVS namespace: "power", key: "shutdown"
// =============================================================================
class EPD_PainterShutdown {
public:
    EPD_PainterShutdown(EPD_Painter* epd);

    // True if a shutdown was requested on the previous reset.
    // Check this in loop() and call proceed() or cancel().
    bool isPending() const { return _pending; }

    // Run the shutdown sequence: show image, power off.
    // Does nothing if !isPending().
    void proceed();

    // Abort the pending shutdown. Re-arms the flag so the next reset
    // will trigger shutdown again. Does nothing if !isPending().
    void cancel();

    // Trigger a shutdown from code. Writes the shutdown flag and restarts.
    //   force=false  — normal path: popup shown if autoShutdown=false,
    //                  otherwise auto-proceeds (same as pressing reset).
    //   force=true   — always bypasses any popup and shuts down immediately.
    void shutdown(bool force = false);

    // Register a callback invoked at the start of every proceed() call,
    // before the shutdown image is shown or the device powers off.
    // Use this to send network messages, flush data, etc.
    void setPreShutdownCallback(std::function<void()> cb) { _pre_shutdown_cb = cb; }

    // Idle auto-off timer. Starts a one-shot countdown; on expiry calls
    // shutdown(true). Call resetIdleTimer() on any user activity to restart
    // the countdown. Call cancelIdleTimer() to stop it entirely.
    void startIdleTimer(uint32_t seconds = 10);
    void resetIdleTimer();
    void cancelIdleTimer();

private:
    EPD_Painter*           _epd;
    bool                   _pending          = false;
    TimerHandle_t          _idle_timer       = nullptr;
    std::function<void()>  _pre_shutdown_cb;

    static void _idle_timer_cb(TimerHandle_t h);

    void showMand();
    void showMandAndPaint();
    void powerOff();
    bool isUsbConnected();
};
