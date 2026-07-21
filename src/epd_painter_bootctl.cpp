#include "epd_painter_bootctl.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_sleep.h"
#include "esp_heap_caps.h"
#include <stdio.h>

// Static default fractal instance.
EPD_BootCtl::FractalImage EPD_BootCtl::fractal;

// =============================================================================
// Constructors
// =============================================================================

EPD_BootCtl::EPD_BootCtl(EPD_Painter& epd)
    : _epd(epd), _provider(fractal) { _init(); }

EPD_BootCtl::EPD_BootCtl(EPD_Painter& epd, IImageProvider& provider)
    : _epd(epd), _provider(provider) { _init(); }

void EPD_BootCtl::_init() {
    if (_isUsbConnected()) return;

    if (!_readFlag()) {
        _pending = true;
        return;
    }

    // Was sleeping — regenerate the image and unpaint it for DC balance.
    _writeFlag(false);
    uint8_t* packed = _getImage();
    if (packed) {
        _epd.setGreyLevels(4);   // unpaintPacked is 2bpp-only (no-op at boot,
                                 // defensive for manually-constructed bootctl)
        _epd.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
        _epd.unpaintPacked(packed);
        _epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
        heap_caps_free(packed);
    }
}

// =============================================================================
// Shutdown
// =============================================================================

void EPD_BootCtl::shutdown() {
    _paintAndPowerOff();
}

void EPD_BootCtl::_paintAndPowerOff() {
    _writeFlag(true);

    uint8_t* packed = _getImage();
    // The boot-image DC cycle is a 2bpp operation (paintPacked /
    // unpaintPacked refuse in 16-grey mode). If the app was running 16
    // greys, drop back to 4 levels first — setGreyLevels migrates the
    // screen state, so the clear below still erases what is on the glass.
    _epd.setGreyLevels(4);
    _epd.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
    _epd.clear();
    if (packed) {
        _epd.paintPacked(packed);
        heap_caps_free(packed);
    }
    delay(500);
    _epd.clearBuffers();
    _powerOff();  // [[noreturn]]
}

uint8_t* EPD_BootCtl::_getImage() const {
    const EPD_Painter::Config& cfg = _epd.getConfig();
    return _provider.getBootImage(cfg.width, cfg.height);
}

// =============================================================================
// FractalImage — built-in IImageProvider implementation.
// Multi-scale XOR: ((x>>2)^(x>>4)^(x>>6)^yterm) & 3
// Each grey level appears ~25% of pixels — ideal for DC balance.
// =============================================================================

uint8_t* EPD_BootCtl::FractalImage::getBootImage(uint16_t W, uint16_t H) {
    const size_t packed_size = (size_t)W * H / 4;

    uint8_t* packed = (uint8_t*)heap_caps_malloc(packed_size, MALLOC_CAP_SPIRAM);
    if (!packed) {
        printf("[BOOT] fractal: PSRAM alloc failed (%u bytes)\n", (unsigned)packed_size);
        return nullptr;
    }

    for (int py = 0; py < H; py++) {
        uint8_t* row   = packed + py * (W / 4);
        uint8_t  yterm = (uint8_t)((py >> 2) ^ (py >> 4) ^ (py >> 6));
        for (int px = 0; px < W; px += 4) {
            auto pixel = [yterm](int x) -> uint8_t {
                return ((x >> 2) ^ (x >> 4) ^ (x >> 6) ^ yterm) & 3;
            };
            row[px >> 2] = (uint8_t)(
                pixel(px    ) << 6 |
                pixel(px + 1) << 4 |
                pixel(px + 2) << 2 |
                pixel(px + 3)
            );
        }
    }
    return packed;
}

// =============================================================================
// Power off
// =============================================================================

void EPD_BootCtl::_powerOff() {
//    return;  // TEMP (Tony): rig work — never power the board down
    const EPD_Painter::Config& cfg = _epd.getConfig();

    if (cfg.pin_syspwr >= 0) {
        pinMode(cfg.pin_syspwr, OUTPUT);
        digitalWrite(cfg.pin_syspwr, HIGH);
        delay(100);
        digitalWrite(cfg.pin_syspwr, LOW);
        printf("[BOOT] pin_syspwr pulsed. If still running, USB is keeping device alive.\n");
    } else {
#ifdef ARDUINO
        TwoWire* wire = cfg.i2c.wire;
        if (wire) {
            const uint8_t BQ_ADDR = 0x6B;
            wire->beginTransmission(BQ_ADDR);
            wire->write(0x09);
            wire->endTransmission();
            wire->requestFrom(BQ_ADDR, (uint8_t)1);
            uint8_t reg = wire->available() ? wire->read() : 0;
            wire->beginTransmission(BQ_ADDR);
            wire->write(0x09);
            wire->write(reg | (1 << 5));  // BATFET_DIS
            wire->endTransmission();
            printf("[BOOT] BQ25896 BATFET disabled. If still running, USB is keeping device alive.\n");
        }
#endif
    }

    esp_deep_sleep_start();
    while (true) {}
}

// =============================================================================
// USB detection
// =============================================================================

bool EPD_BootCtl::_isUsbConnected() const {
#ifdef ARDUINO
    const EPD_Painter::Config& cfg = _epd.getConfig();
    TwoWire* wire = cfg.i2c.wire;
    if (!wire) return false;
    const uint8_t BQ_ADDR = 0x6B;
    wire->beginTransmission(BQ_ADDR);
    wire->write(0x0B);
    if (wire->endTransmission(false) != 0) return false;
    if (wire->requestFrom(BQ_ADDR, (uint8_t)1) == 0) return false;
    uint8_t reg = wire->read();
    return (reg & 0xF8) != 0;
#else
    return false;
#endif
}

// =============================================================================
// NVS flag — only stores running/sleeping state, nothing else
// =============================================================================

static void _nvsInit() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
}

bool EPD_BootCtl::_readFlag() const {
    _nvsInit();
    nvs_handle_t h;
    uint8_t val = 0;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, NVS_KEY_FLAG, &val);
        nvs_close(h);
    }
    return val != 0;
}

void EPD_BootCtl::_writeFlag(bool v) const {
    _nvsInit();
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, NVS_KEY_FLAG, v ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}
