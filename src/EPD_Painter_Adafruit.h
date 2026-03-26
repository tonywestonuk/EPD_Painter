#include <string.h>
#pragma once

#include <Adafruit_GFX.h>
#include <esp_heap_caps.h>
#include "EPD_Painter.h"

// =============================================================================
// EPD_PainterAdafruit
//
// Owns the 8bpp framebuffer, exposes the Adafruit GFX drawing API, and drives
// the eInk panel via an EPD_Painter instance.
//
// Ownership model:
//   - This class allocates the framebuffer in its constructor (PSRAM, 16-byte
//     aligned) and frees it in the destructor.
//   - EPD_Painter is constructed internally and borrows the same pointer.
//   - GFXcanvas8 also draws into the same buffer — no copying on paint().
//
// Usage:
//   EPD_Painter::Config config = { ... };
//   EPD_PainterAdafruit gfx(config);
//
//   gfx.begin();
//   gfx.fillScreen(0xFF);
//   gfx.setCursor(10, 10);
//   gfx.print("Hello!");
//   gfx.paint();
// =============================================================================
class EPD_PainterAdafruit : public GFXcanvas8 {
public:

    // -------------------------------------------------------------------------
    // Constructor — allocates the framebuffer and wires everything together.
    // -------------------------------------------------------------------------
    explicit EPD_PainterAdafruit(const EPD_Painter::Config &config, bool portrait = false)
        : GFXcanvas8(
              // Swap canvas dimensions for portrait (CW rotation) so the GFX
              // drawing space matches the logical orientation.
              (config.rotation == EPD_Painter::Rotation::ROTATION_CW || portrait) ? config.height : config.width,
              (config.rotation == EPD_Painter::Rotation::ROTATION_CW || portrait) ? config.width  : config.height,
              false),
          _config(config),
          _painter(config, portrait)
    {
        if (portrait) _config.rotation = EPD_Painter::Rotation::ROTATION_CW;
    }

    // -------------------------------------------------------------------------
    // Destructor — frees the framebuffer.
    // GFXcanvas8 must not free it too, so we null the pointer before it runs.
    // -------------------------------------------------------------------------
    ~EPD_PainterAdafruit() {
        this->buffer = nullptr;         // disown before ~GFXcanvas8 runs
        heap_caps_free(_framebuffer);
        _framebuffer = nullptr;
    }

    // -------------------------------------------------------------------------
    // begin() — allocates framebuffer, inits EPD_Painter hardware.
    // Call after Serial / I2C setup, before any drawing.
    // -------------------------------------------------------------------------
    bool begin() {
        bool r = _painter.begin();
        _painter.setInterlaceMode(true);
        _config = _painter.getConfig();

        const size_t buf_size = (size_t)_config.width * (size_t)_config.height;

        buffer = static_cast<uint8_t *>(
            heap_caps_aligned_alloc(16, buf_size, MALLOC_CAP_SPIRAM));
        
        memset(buffer,0x00,_config.width * _config.height);

        return r;
    }

    // -------------------------------------------------------------------------
    // end() — mirrors EPD_Painter::end()
    // -------------------------------------------------------------------------
    bool end() { return _painter.end(); }

    // -------------------------------------------------------------------------
    // Rendering
    // -------------------------------------------------------------------------
    void paint()   { _painter.paint(buffer); }
    void clear()   { _painter.clear(); }
    void fxClear() { _painter.fxClear(); }
    void dither()  { EPD_Painter::dither(buffer, _config.width, _config.height); }

    // -------------------------------------------------------------------------
    // Quality
    // -------------------------------------------------------------------------
    void setQuality(EPD_Painter::Quality q) { _painter.setQuality(q); }
    
    EPD_Painter::Config getConfig(){ return _painter.getConfig(); }
    

    // -------------------------------------------------------------------------
    // Access to the underlying driver if needed
    // -------------------------------------------------------------------------
    EPD_Painter      &driver()   { return _painter; }

    // Shutdown — call setAutoShutdown(false) BEFORE begin() to intercept
    // the shutdown yourself. Then check shutdown()->isPending() in loop().
    void              setAutoShutdown(bool v) { _painter.setAutoShutdown(v); }
    EPD_PainterShutdown *shutdown()           { return _painter.shutdown(); }

private:
    EPD_Painter::Config  _config;
    uint8_t             *_framebuffer = nullptr;    // owned here
    EPD_Painter          _painter;                  // borrows _framebuffer
};
