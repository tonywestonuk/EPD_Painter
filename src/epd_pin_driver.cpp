#include "epd_pin_driver.h"
#include "build_opt.h"
#include <string.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

EPD_ShiftReg::EPD_ShiftReg(uint8_t data, uint8_t clk, uint8_t str, int le_time)
    : _le_time(le_time), _data_pin(data), _clk_pin(clk), _str_pin(str)
{
    // Fixed bit assignments in the 8-bit fast-GPIO port.
    // Physical pins are routed to these bits via the GPIO Matrix at runtime.
    _masks.data_mask    = (1u << 0);
    _masks.clk_mask     = (1u << 1);
    _masks.str_mask     = (1u << 2);
    _masks.clkdata_mask = (1u << 0) | (1u << 1);
}

// Route physical pins from the fast-GPIO signals for the given core.
// Core 0 uses PRO_ALONEGPIO_OUT0~2 (indices 221-223).
// Core 1 uses CORE1_GPIO_OUT0~2   (indices 129-131).
// Called at most once per core change — typically twice per paint cycle.
void EPD_ShiftReg::_reconfigure_fast_gpio(int core) {
    uint8_t data_sig, clk_sig, str_sig;
    if (core == 0) {
        data_sig = PRO_ALONEGPIO_OUT0_IDX;
        clk_sig  = PRO_ALONEGPIO_OUT1_IDX;
        str_sig  = PRO_ALONEGPIO_OUT2_IDX;
    } else {
        data_sig = CORE1_GPIO_OUT0_IDX;
        clk_sig  = CORE1_GPIO_OUT1_IDX;
        str_sig  = CORE1_GPIO_OUT2_IDX;
    }
    esp_rom_gpio_connect_out_signal(_data_pin, data_sig, false, false);
    esp_rom_gpio_connect_out_signal(_clk_pin,  clk_sig,  false, false);
    esp_rom_gpio_connect_out_signal(_str_pin,  str_sig,  false, false);
}

void IRAM_ATTR EPD_ShiftReg::push(uint8_t byte) {
    int core = xPortGetCoreID();
    if (core != _fast_gpio_core) {
        _reconfigure_fast_gpio(core);
        _fast_gpio_core = core;
    }
    epd_sr_push_fast(byte, &_masks);
}

void IRAM_ATTR EPD_ShiftReg::sr_set_bit(uint8_t index, bool val) {
    if (val) _state |=  (1u << index);
    else     _state &= ~(1u << index);
    push(_state);
    // LE (bit 0) going high needs a board-specific hold before the next edge
    if (index == 0 && val && _le_time > 0)
        EPD_DELAY_US(_le_time);
}

void EPD_ShiftReg::reset() {
    _state = 0;
    push(0);
}
