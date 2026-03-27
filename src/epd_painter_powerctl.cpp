#include "epd_painter_powerctl.h"
#include <stdio.h>

#ifndef ARDUINO
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
#endif

epd_painter_powerctl::epd_painter_powerctl() {
  _pca_out[0] = 0x00;
  _pca_out[1] = 0x00;
  _pca_cfg[0] = 0xFF;
  _pca_cfg[1] = 0xFF;
}

bool epd_painter_powerctl::begin(EPD_Painter::Config cfg) {
  config = cfg;

  if (config.power.pca_addr != -1) {
    // PCA9555 I2C expander (GPS board)
    for (int pin = 8; pin <= 13; ++pin) {
      if (!pcaPinMode(pin, PCA_OUTPUT)) {
        printf("[PWRCTL] Failed setting PCA pin %d OUTPUT\n", pin);
        return false;
      }
    }
    if (!pcaPinMode(14, PCA_INPUT))  { printf("[PWRCTL] PCA pin 14 INPUT fail\n"); return false; }
    if (!pcaPinMode(15, PCA_INPUT))  { printf("[PWRCTL] PCA pin 15 INPUT fail\n"); return false; }
    printf("[PWRCTL] PCA init OK. CFG1=0x%02X OUT1=0x%02X\n", _pca_cfg[1], _pca_out[1]);

  } else if (config.shift.data >= 0) {
    // 74HCT4094D shift register (H752 board)
    // GPIO42 is JTAG MTMS — must be reset before use
    gpio_reset_pin((gpio_num_t)config.shift.clk);
    EPD_PIN_OUTPUT(config.shift.data);
    EPD_PIN_OUTPUT(config.shift.clk);
    EPD_PIN_OUTPUT(config.shift.str);
    EPD_PIN_LOW(config.shift.str);
    // Push safe initial state: power_disable=1, everything else off
    _sr = ShiftState{};  // defaults: power_disable=true, rest false
    sr_push_slow();
    printf("[PWRCTL] Shift-reg init OK (DATA=%d CLK=%d STR=%d)\n",
           config.shift.data, config.shift.clk, config.shift.str);
  }

  return true;
}

// ---------------------------------------------------------------------------
// Shift-register push — slow version (uses digitalWrite, not IRAM-safe)
// ---------------------------------------------------------------------------
void epd_painter_powerctl::sr_push_slow() {
  EPD_PIN_LOW(config.shift.str);
  // Push MSB first: Q7..Q0
  auto pushBit = [&](bool b) {
    EPD_PIN_LOW(config.shift.clk);
    if (b) EPD_PIN_HIGH(config.shift.data); else EPD_PIN_LOW(config.shift.data);
    EPD_PIN_HIGH(config.shift.clk);
  };
  pushBit(_sr.ep_output_enable);   // Q7
  pushBit(_sr.ep_mode);             // Q6
  pushBit(_sr.ep_scan_direction);   // Q5
  pushBit(_sr.ep_stv);              // Q4
  pushBit(_sr.neg_power_enable);    // Q3
  pushBit(_sr.pos_power_enable);    // Q2
  pushBit(_sr.power_disable);       // Q1
  pushBit(_sr.ep_latch_enable);     // Q0
  EPD_PIN_HIGH(config.shift.str);
}

// ---------------------------------------------------------------------------
// Hot-path controls (called from sendRow)
// ---------------------------------------------------------------------------
void IRAM_ATTR epd_painter_powerctl::sr_set_le(bool val) {
  _sr.ep_latch_enable = val;
  sr_push_slow();
}

void IRAM_ATTR epd_painter_powerctl::sr_set_stv(bool val) {
  _sr.ep_stv = val;
  sr_push_slow();
}

// ---------------------------------------------------------------------------
// Power-on sequence
// ---------------------------------------------------------------------------
bool epd_painter_powerctl::powerOn() {
  printf("[PWRCTL] Power-on...\n");

  if (config.power.pca_addr != -1) {
    // GPS board — PCA9555 + TPS65185
    if (!pcaWrite(PIN_OE, true))     return false;
    if (!pcaWrite(PIN_MODE, true))   return false;
    if (!pcaWrite(PIN_WAKEUP, true)) return false;
    if (!pcaWrite(PIN_PWRUP, true))  return false;
    if (!pcaWrite(PIN_VCOM, true))   return false;

    EPD_DELAY_MS(3);
    printf("[PWRCTL] Waiting for PWR_GOOD...");
    int timeout = 0;
    bool good = false;
    while (timeout < 400) {
      if (!pcaRead(PIN_PWRGOOD, good)) { printf(" read error\n"); return false; }
      if (good) break;
      EPD_DELAY_MS(1);
      ++timeout;
    }
    if (!good) { printf(" TIMEOUT\n"); return false; }
    printf(" OK (%d ms)\n", timeout);

    if (!tpsWrite(TPS_UPSEQ0, 0xE1)) return false;
    if (!tpsWrite(TPS_UPSEQ1, 0xAA)) return false;
    if (!tpsWrite(TPS_ENABLE, 0x3F)) return false;

  // Dont set vcomm... this chip should already konow it. 
  //setVcomMv(config.power.vcom_mv);

    uint8_t v1 = 0, v2 = 0;
    tpsRead(TPS_VCOM1, v1);
    tpsRead(TPS_VCOM2, v2);
    printf("[PWRCTL] TPS VCOM = %d mV\n", -(((int)(v2 & 0x01) << 8 | v1) * 10));

    printf("[PWRCTL] Waiting for TPS PG...");
    timeout = 0;
    uint8_t pg = 0;
    while (timeout < 400) {
      if (!tpsRead(TPS_PG, pg)) { printf(" read error\n"); return false; }
      if ((pg & 0xFA) == 0xFA) break;
      EPD_DELAY_MS(1);
      ++timeout;
    }
    printf(" PG=0x%02X (%d ms) %s\n", pg, timeout, ((pg & 0xFA) == 0xFA) ? "OK" : "TIMEOUT");
    return (pg & 0xFA) == 0xFA;

  } else if (config.shift.data >= 0) {
    // H752 — 74HCT4094D shift register sequence
    _sr.ep_scan_direction = true;
    _sr.power_disable = false;
    sr_push_slow();
    EPD_DELAY_US(100);

    _sr.neg_power_enable = true;
    _sr.pos_power_enable = true;
    sr_push_slow();
    EPD_DELAY_US(100);

    _sr.ep_stv            = true;
    _sr.ep_output_enable  = true;
    _sr.ep_mode           = true;
    sr_push_slow();

    printf("[PWRCTL] Shift-reg power-on OK\n");
    return true;
  }

  return false;
}

// ---------------------------------------------------------------------------
// Power-off sequence
// ---------------------------------------------------------------------------
void epd_painter_powerctl::powerOff() {
  printf("[PWRCTL] Power-off...\n");

  if (config.power.pca_addr != -1) {
    pcaWrite(PIN_OE, false);
    pcaWrite(PIN_MODE, false);
    pcaWrite(PIN_PWRUP, false);
    pcaWrite(PIN_VCOM, false);
    EPD_DELAY_MS(1);
    pcaWrite(PIN_WAKEUP, false);

  } else if (config.shift.data >= 0) {
    _sr.pos_power_enable = false;
    _sr.neg_power_enable = false;
    sr_push_slow();
    EPD_DELAY_US(100);

    _sr.ep_stv           = false;
    _sr.ep_output_enable = false;
    _sr.ep_mode          = false;
    _sr.ep_latch_enable  = false;
    _sr.power_disable    = true;
    sr_push_slow();
  }
}

// ---------------------------------------------------------------------------
bool epd_painter_powerctl::isPwrGood() {
  if (config.power.pca_addr != -1) {
    bool val = false;
    if (!pcaRead(PIN_PWRGOOD, val)) return false;
    return val;
  }
  return true;  // shift-register board: assume good
}

uint8_t epd_painter_powerctl::readTpsPg() {
  if (config.power.tps_addr != -1) {
    uint8_t val = 0;
    tpsRead(TPS_PG, val);
    return val;
  }
  return 0xFF;
}

uint8_t epd_painter_powerctl::readPcaPort(uint8_t port) {
  if (config.power.pca_addr != -1 && port <= 1) {
    uint8_t val = 0;
    pcaReadReg(port, val);
    return val;
  }
  return 0;
}

void epd_painter_powerctl::setVcomMv(int vcom_mv) {
  int mag = vcom_mv < 0 ? -vcom_mv : vcom_mv;
  mag /= 10;
  tpsWrite16(TPS_VCOM1, (uint8_t)(mag & 0xFF), (uint8_t)((mag >> 8) & 0xFF));
}

// ---------------------------------------------------------------------------
// PCA9555 low-level
// ---------------------------------------------------------------------------
bool epd_painter_powerctl::pcaWriteReg(uint8_t reg, uint8_t val) {
#ifdef ARDUINO
  config.i2c.wire->beginTransmission(config.power.pca_addr);
  config.i2c.wire->write(reg);
  config.i2c.wire->write(val);
  return (config.i2c.wire->endTransmission() == 0);
#else
  return false;
#endif
}

bool epd_painter_powerctl::pcaReadReg(uint8_t reg, uint8_t& val) {
#ifdef ARDUINO
  config.i2c.wire->beginTransmission(config.power.pca_addr);
  config.i2c.wire->write(reg);
  if (config.i2c.wire->endTransmission(false) != 0) return false;
  int n = config.i2c.wire->requestFrom(config.power.pca_addr, 1);
  if (n != 1 || !config.i2c.wire->available()) return false;
  val = config.i2c.wire->read();
  return true;
#else
  return false;
#endif
}

bool epd_painter_powerctl::pcaPinMode(uint8_t pin, uint8_t mode) {
  uint8_t port = pin / 8, bit = pin % 8;
  if (port > 1) return false;
  if (mode == PCA_INPUT) _pca_cfg[port] |=  (1 << bit);
  else                   _pca_cfg[port] &= ~(1 << bit);
  return pcaWriteReg(6 + port, _pca_cfg[port]);
}

bool epd_painter_powerctl::pcaWrite(uint8_t pin, bool val) {
  uint8_t port = pin / 8, bit = pin % 8;
  if (port > 1) return false;
  if (val) _pca_out[port] |=  (1 << bit);
  else     _pca_out[port] &= ~(1 << bit);
  return pcaWriteReg(2 + port, _pca_out[port]);
}

bool epd_painter_powerctl::pcaRead(uint8_t pin, bool& val) {
  uint8_t port = pin / 8, bit = pin % 8, reg_val = 0;
  if (port > 1) return false;
  if (!pcaReadReg(port, reg_val)) return false;
  val = ((reg_val >> bit) & 0x01) != 0;
  return true;
}

// ---------------------------------------------------------------------------
// TPS65185 low-level
// ---------------------------------------------------------------------------
bool epd_painter_powerctl::tpsWrite(uint8_t reg, uint8_t val) {
#ifdef ARDUINO
  config.i2c.wire->beginTransmission(config.power.tps_addr);
  config.i2c.wire->write(reg);
  config.i2c.wire->write(val);
  return (config.i2c.wire->endTransmission() == 0);
#else
  return false;
#endif
}

bool epd_painter_powerctl::tpsWrite16(uint8_t reg, uint8_t lo, uint8_t hi) {
#ifdef ARDUINO
  config.i2c.wire->beginTransmission(config.power.tps_addr);
  config.i2c.wire->write(reg);
  config.i2c.wire->write(lo);
  config.i2c.wire->write(hi);
  return (config.i2c.wire->endTransmission() == 0);
#else
  return false;
#endif
}

bool epd_painter_powerctl::tpsRead(uint8_t reg, uint8_t& val) {
#ifdef ARDUINO
  config.i2c.wire->beginTransmission(config.power.tps_addr);
  config.i2c.wire->write(reg);
  if (config.i2c.wire->endTransmission(false) != 0) return false;
  int n = config.i2c.wire->requestFrom(config.power.tps_addr, 1);
  if (n != 1 || !config.i2c.wire->available()) return false;
  val = config.i2c.wire->read();
  return true;
#else
  return false;
#endif
}
