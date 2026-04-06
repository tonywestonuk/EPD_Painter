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
class EPD_PainterAdafruit : public GFXcanvas8
{
public:
    // -------------------------------------------------------------------------
    // Constructor — allocates the framebuffer and wires everything together.
    // -------------------------------------------------------------------------
    explicit EPD_PainterAdafruit(const EPD_Painter::Config &config, bool portrait = false)
        : GFXcanvas8(
              // Swap canvas dimensions for portrait (CW rotation) so the GFX
              // drawing space matches the logical orientation.
              (config.rotation == EPD_Painter::Rotation::ROTATION_CW || portrait) ? config.height : config.width,
              (config.rotation == EPD_Painter::Rotation::ROTATION_CW || portrait) ? config.width : config.height,
              false), _painter(config, portrait) {}
   
    // -------------------------------------------------------------------------
    // Destructor — frees the framebuffer.
    // GFXcanvas8 must not free it too, so we null the pointer before it runs.
    // -------------------------------------------------------------------------
    ~EPD_PainterAdafruit()
    {
        this->buffer = nullptr; // disown before ~GFXcanvas8 runs
        heap_caps_free(_framebuffer);
        _framebuffer = nullptr;
    }

    // -------------------------------------------------------------------------
    // begin() — allocates framebuffer, inits EPD_Painter hardware.
    // Call after Serial / I2C setup, before any drawing.
    // -------------------------------------------------------------------------
    bool begin()
    {
        const size_t buf_size = (size_t)_painter._config.width * (size_t)_painter._config.height;

        buffer = static_cast<uint8_t *>(
            heap_caps_aligned_alloc(16, buf_size, MALLOC_CAP_SPIRAM));

        memset(buffer, 0x00, _painter._config.width * _painter._config.height);

        _painter.setInterlaceMode(true);

        return _painter.begin();
    }

    // -------------------------------------------------------------------------
    // end() — mirrors EPD_Painter::end()
    // -------------------------------------------------------------------------
    bool end() { return _painter.end(); }

    // -------------------------------------------------------------------------
    // Rendering
    // -------------------------------------------------------------------------
    void paint() { _painter.paint(buffer); }
    void paintLater() { _painter.paintLater(buffer); }

    void clear(const EPD_Painter::Rect *rects = nullptr, int num_rects = 0, EPD_Painter::ClearMode mode = EPD_Painter::ClearMode::HARD) { _painter.clear(rects, num_rects, mode); }
    int computeDirtyRects(EPD_Painter::Rect *out, int max, int tolerance = 0) const { return _painter.computeDirtyRects(out, max, tolerance); }
    void clearDirtyAreas(int tolerance = 0, EPD_Painter::ClearMode mode = EPD_Painter::ClearMode::SOFT) { _painter.clearDirtyAreas(buffer, tolerance, mode); }
    void fxClear() { _painter.fxClear(); }
    void dither() { EPD_Painter::dither(buffer, _painter._config.width, _painter._config.height); }

    // Pack the current framebuffer to 2bpp. Returns a PSRAM-allocated buffer
    // the caller must free with heap_caps_free(). Returns nullptr on failure.
    // Use this to supply packed image data to EPD_BootCtl::IImageProvider.
    uint8_t* packBuffer() { return _painter.packBuffer(buffer); }

    // -------------------------------------------------------------------------
    // Quality
    // -------------------------------------------------------------------------
    void setQuality(EPD_Painter::Quality q) { _painter.setQuality(q); }

    const EPD_Painter::Config& getConfig() { return _painter.getConfig(); }
    const EPD_Painter::Config* getPreset() const { return _painter.getPreset(); }

    // -------------------------------------------------------------------------
    // Access to the underlying driver if needed
    // -------------------------------------------------------------------------
    EPD_Painter &driver() { return _painter; }

    // Direct framebuffer access — 8bpp, one byte per pixel, width*height bytes.
    uint8_t* getBuffer() { return buffer; }

    // Override fillRect to use memset directly on the 8bpp buffer instead of the
    // Adafruit GFX pixel-by-pixel loop.  Falls back to the base class if a
    // rotation is active (rare — MagiTrac runs landscape with rotation=0).
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override {
        if (rotation != 0) {
            Adafruit_GFX::fillRect(x, y, w, h, color);
            return;
        }
        // Clamp to buffer bounds
        if (x < 0) { w += x; x = 0; }
        if (y < 0) { h += y; y = 0; }
        if (x + w > WIDTH)  w = WIDTH  - x;
        if (y + h > HEIGHT) h = HEIGHT - y;
        if (w <= 0 || h <= 0) return;

        uint8_t c = static_cast<uint8_t>(color);
        if (x == 0 && w == WIDTH) {
            // Full-width span is contiguous — single memset.
            memset(buffer + static_cast<int32_t>(y) * WIDTH, c,
                   static_cast<int32_t>(w) * h);
        } else {
            for (int16_t r = 0; r < h; r++)
                memset(buffer + static_cast<int32_t>(y + r) * WIDTH + x, c, w);
        }
    }

    // Shutdown — call setAutoShutdown(false) BEFORE begin() to intercept
    // the shutdown yourself. Then check shutdown()->isPending() in loop().
    void setAutoShutdown(bool v) { _painter.setAutoShutdown(v); }
    EPD_PainterShutdown *shutdown() { return _painter.shutdown(); }

private:
    uint8_t *_framebuffer = nullptr; // owned here
    EPD_Painter _painter;            // borrows _framebuffer
};
