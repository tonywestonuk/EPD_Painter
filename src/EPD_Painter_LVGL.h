#pragma once

#include <string.h>
#include <lvgl.h>
#include <esp_heap_caps.h>
#include "EPD_Painter.h"
#include "esp_timer.h"


// =============================================================================
// EPD_PainterLVGL
//
// Owns the 8bpp framebuffer, registers it as the LVGL draw buffer, and drives
// the eInk panel via an EPD_Painter instance.
//
// Ownership model:
//   - This class allocates the framebuffer in begin() (PSRAM, 16-byte aligned)
//     and frees it in the destructor.
//   - EPD_Painter is constructed internally and borrows the same pointer via
//     paint(buffer) — matching the pattern used by EPD_PainterAdafruit.
//   - LVGL renders directly into the same buffer — no copying on flush.
//
// Requires:
//   - LV_COLOR_DEPTH 8 in lv_conf.h
//   - LVGL v9
//
// Usage:
//   EPD_Painter::Config config = { ... };
//   EPD_PainterLVGL lvgl(config);
//
//   lv_init();
//   lv_tick_set_cb(my_tick);
//   lvgl.begin();
//
//   // Create widgets normally, then in loop():
//   lv_timer_handler();
// =============================================================================
class EPD_PainterLVGL {
public:

    // EPD shade constants — use these instead of lv_color_black()/white().
    // LVGL renders to 8-bit RGB332; the EPD driver reads the 2 LSBs of each
    // byte as the pixel value (0=white … 3=black). The blue channel in RGB332
    // supplies those 2 bits, so we drive it with multiples of 85 (0x55).
    inline static const lv_color_t WHITE   = lv_color_make(0, 0,   0);   // blue>>6 = 0
    inline static const lv_color_t LT_GREY = lv_color_make(0, 0,  85);   // blue>>6 = 1
    inline static const lv_color_t DK_GREY = lv_color_make(0, 0, 170);   // blue>>6 = 2
    inline static const lv_color_t BLACK   = lv_color_make(0, 0, 255);   // blue>>6 = 3

    explicit EPD_PainterLVGL(const EPD_Painter::Config &config)
        : _painter(config)
    {}

    ~EPD_PainterLVGL() {
        heap_caps_free(_framebuffer);
        _framebuffer = nullptr;
    }

    // -------------------------------------------------------------------------
    // begin() — allocates framebuffer, inits EPD_Painter, registers LVGL display.
    // Call after lv_init() and lv_tick_set_cb(), before creating any widgets.
    // -------------------------------------------------------------------------
    bool begin() {
        const size_t buf_size = (size_t)_painter.getConfig().width * (size_t)_painter.getConfig().height;

        _framebuffer = static_cast<uint8_t *>(
            heap_caps_aligned_alloc(16, buf_size, MALLOC_CAP_SPIRAM));

        if (!_framebuffer) return false;
        memset(_framebuffer, 0x00, buf_size);

        if (!_painter.begin()) return false;

        _painter.setInterlaceMode(true);

        // Register with LVGL — FULL mode means flush_cb is called once per
        // frame with the complete buffer, ideal for eInk.
        _disp = lv_display_create(_painter.getConfig().width, _painter.getConfig().height);
        lv_display_set_buffers(
            _disp,
            _framebuffer,
            nullptr,
            (uint32_t)buf_size,
            LV_DISPLAY_RENDER_MODE_FULL
        );
        lv_display_set_flush_cb(_disp, _flush_cb);
        lv_display_set_user_data(_disp, this);


        return true;
    }

    // -------------------------------------------------------------------------
    // end()
    // -------------------------------------------------------------------------
    bool end() { return _painter.end(); }
    void clear() { 
        _painter.clear(); 
        
    }

    // -------------------------------------------------------------------------
    // Quality
    // -------------------------------------------------------------------------
    void setQuality(EPD_Painter::Quality q) { _painter.setQuality(q); }

    // -------------------------------------------------------------------------
    // Config accessor — mirrors EPD_PainterAdafruit
    // -------------------------------------------------------------------------
    EPD_Painter::Config getConfig() { return _painter.getConfig(); }

    // -------------------------------------------------------------------------
    // Access to the underlying driver if needed
    // -------------------------------------------------------------------------
    EPD_Painter  &driver()  { return _painter; }
    lv_display_t *display() { return _disp; }

    // -------------------------------------------------------------------------
    // setPaintPasses() — how many times paint() is called per flush.
    // 1 = single pass (default); 2 = two sequential paint passes per frame.
    // -------------------------------------------------------------------------
    void setPaintPasses(uint8_t n) { _paint_passes = (n > 0) ? n : 1; }

private:
    uint8_t             *_framebuffer    = nullptr;   // owned here
    EPD_Painter          _painter;                    // borrows _framebuffer
    lv_display_t        *_disp           = nullptr;
    lv_timer_t          *_paint_timer    = nullptr;

    uint8_t             *_pending_px_map = nullptr;   // set by flush_cb
    bool                 _flush_pending  = false;     // task consumes this
    uint8_t              _paint_pass     = 0;         // passes completed
    uint8_t              _paint_passes   = 2;         // passes required

    // -------------------------------------------------------------------------
    // LVGL flush callback — stores the buffer pointer and returns immediately.
    // The actual EPD paint happens in _paint_task_cb.
    // -------------------------------------------------------------------------
    static void _flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
        auto *self = static_cast<EPD_PainterLVGL *>(lv_display_get_user_data(disp));
        self->_painter.paintLater(px_map);
        lv_display_flush_ready(disp);
    }

  
};