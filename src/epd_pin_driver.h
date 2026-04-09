#pragma once
#include <stdint.h>
#include <soc/gpio_reg.h>

#ifndef IRAM_ATTR
  #define IRAM_ATTR
#endif

// =============================================================================
// EPD_PinDriver — abstract interface for a single EPD control pin.
// =============================================================================
class EPD_PinDriver {
public:
    virtual void set(bool high) = 0;
    virtual ~EPD_PinDriver() = default;
};

// =============================================================================
// EPD_GpioPin — direct GPIO register write, IRAM-safe.
// =============================================================================
class EPD_GpioPin : public EPD_PinDriver {
public:
    EPD_GpioPin() : _pin(0) {}
    explicit EPD_GpioPin(uint8_t pin) : _pin(pin) {}
    EPD_GpioPin& operator=(const EPD_GpioPin&) = default;

    void set(bool high) override {
        if (high) {
            if (_pin < 32) REG_WRITE(GPIO_OUT_W1TS_REG,  1UL << _pin);
            else           REG_WRITE(GPIO_OUT1_W1TS_REG, 1UL << (_pin - 32));
        } else {
            if (_pin < 32) REG_WRITE(GPIO_OUT_W1TC_REG,  1UL << _pin);
            else           REG_WRITE(GPIO_OUT1_W1TC_REG, 1UL << (_pin - 32));
        }
    }

private:
    uint8_t _pin;
};

// =============================================================================
// EPD_ISRController — interface for shift-register bit-level control.
// =============================================================================
class EPD_ISRController {
public:
    virtual void IRAM_ATTR sr_set_bit(uint8_t index, bool val) = 0;
    virtual ~EPD_ISRController() = default;
};

// =============================================================================
// EPD_SRMasks — fast-GPIO bit masks for epd_sr_push_fast().
//
// DATA, CLK, STR are assigned to fast-GPIO output bits 0, 1, 2.
// EE.WR_MASK_GPIO_OUT operates on this 8-bit port; physical pins are routed
// to it via the GPIO Matrix (core-specific signals configured at runtime).
// =============================================================================
struct EPD_SRMasks {
    uint32_t data_mask;      // (1 << 0)
    uint32_t clk_mask;       // (1 << 1)
    uint32_t str_mask;       // (1 << 2)
    uint32_t clkdata_mask;   // (1 << 0) | (1 << 1)
};

// Assembly implementation — defined in EPD_Painter.S, placed in IRAM.
extern "C" void IRAM_ATTR epd_sr_push_fast(uint8_t byte, const EPD_SRMasks* masks);

// =============================================================================
// EPD_ShiftReg — 74HCT4094D with internal 8-bit state tracking.
//
// push() uses EE.WR_MASK_GPIO_OUT via epd_sr_push_fast(). Physical pins are
// routed from the executing core's fast-GPIO signals on demand — so the same
// code works whether called from the user task (core 1) or the paint task
// (core 0). Re-routing via the GPIO Matrix is performed automatically on the
// first push() after a core change; it costs ~3 register writes and is rare.
//
// Works for any physical pin number (fast-GPIO is not limited to pins < 32).
// le_time: optional hold delay (µs) after bit 0 (EP_LE) goes high.
// =============================================================================
class EPD_ShiftReg : public EPD_ISRController {
public:
    EPD_ShiftReg() = default;
    EPD_ShiftReg(uint8_t data, uint8_t clk, uint8_t str, int le_time = 0);
    EPD_ShiftReg& operator=(const EPD_ShiftReg&) = default;

    void IRAM_ATTR sr_set_bit(uint8_t index, bool val) override;
    void IRAM_ATTR push(uint8_t byte);
    void reset();

private:
    EPD_SRMasks _masks    = {};
    uint8_t     _state    = 0;
    int         _le_time  = 0;
    uint8_t     _data_pin = 0;
    uint8_t     _clk_pin  = 0;
    uint8_t     _str_pin  = 0;
    int8_t      _fast_gpio_core = -1;  // -1 = not yet routed

    void _reconfigure_fast_gpio(int core);
};

// =============================================================================
// EPD_SRPin — routes a logical pin through any EPD_ISRController.
// =============================================================================
class EPD_SRPin : public EPD_PinDriver {
public:
    EPD_SRPin(EPD_ISRController* sr, uint8_t index)
        : _sr(sr), _index(index) {}

    void set(bool high) override {
        _sr->sr_set_bit(_index, high);
    }

private:
    EPD_ISRController* _sr;
    uint8_t _index;
};

// =============================================================================
// EPD_PowerDriver — abstract interface for board-level power sequencing.
// SR-based boards override isrController() to expose their EPD_ShiftReg.
// =============================================================================
class EPD_PowerDriver {
public:
    virtual bool powerOn() = 0;
    virtual void powerOff() = 0;
    virtual EPD_ISRController* isrController() { return nullptr; }
    virtual ~EPD_PowerDriver() = default;
};
