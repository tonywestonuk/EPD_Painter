#include "epd_painter_powerctl.h"
#include <stdio.h>
#include <hal/gpio_hal.h>

#ifndef ARDUINO
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
#endif

// Routine power-cycle trace. With idle power-off (default 5s) powerOn/powerOff
// run constantly, so the per-cycle messages are compiled out unless
// EPD_DEBUG_REGISTERS is defined. Errors always print.
//#define EPD_DEBUG_REGISTERS
#ifdef EPD_DEBUG_REGISTERS
#define EPD_DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define EPD_DEBUG_PRINT(...)
#endif

epd_painter_powerctl::epd_painter_powerctl() {
  _pca_out[0] = 0x00;
  _pca_out[1] = 0x00;
  _pca_cfg[0] = 0xFF;
  _pca_cfg[1] = 0xFF;
}

bool epd_painter_powerctl::begin(EPD_Painter::Config cfg) {

  config = cfg;
  if (!_mtx) _mtx = xSemaphoreCreateMutex();

  // I2C not used in ESP-IDF builds

  // ---- Configure PCA9535: pins 8-13 outputs, 14-15 inputs ----
    for (int pin = 8; pin <= 13; ++pin) {
      if (!pcaPinMode(pin, PCA_OUTPUT)) {
        printf("[PWRCTL] Failed setting PCA pin %d OUTPUT\n", pin);
        return false;
      }
    }
  if (!pcaPinMode(14, PCA_INPUT)) {
    printf("[PWRCTL] Failed setting PCA pin 14 INPUT\n");
    return false;
  }
  if (!pcaPinMode(15, PCA_INPUT)) {
    printf("[PWRCTL] Failed setting PCA pin 15 INPUT\n");
    return false;
  }

  printf("[PWRCTL] PCA init OK. CFG1=0x%02X OUT1=0x%02X\n",
         _pca_cfg[1], _pca_out[1]);
  return true;
}

bool epd_painter_powerctl::powerOn() {
  if (_mtx) xSemaphoreTake(_mtx, portMAX_DELAY);
  bool ok = powerOnLocked();
  if (_mtx) xSemaphoreGive(_mtx);
  return ok;
}

bool epd_painter_powerctl::powerOnLocked() {
  EPD_DEBUG_PRINT("[PWRCTL] Power-on sequence... \n");

  if (!pcaWrite(PIN_OE, true))     return false;
  if (!pcaWrite(PIN_MODE, true))   return false;
  if (!pcaWrite(PIN_WAKEUP, true)) return false;
  if (!pcaWrite(PIN_PWRUP, true))  return false;
  if (!pcaWrite(PIN_VCOM, true))   return false;

  EPD_DELAY_MS(3);

  EPD_DEBUG_PRINT("[PWRCTL] Waiting for PWR_GOOD...");
  int timeout = 0;
  bool good = false;
  while (timeout < 400) {
    if (!pcaRead(PIN_PWRGOOD, good)) {
      printf("[PWRCTL] PWR_GOOD read error\n");
      return false;
    }
    if (good) break;
    EPD_DELAY_MS(1);
    ++timeout;
  }
  if (!good) {
    printf("[PWRCTL] PWR_GOOD timeout\n");
    return false;
  }
  EPD_DEBUG_PRINT(" OK (%d ms)\n", timeout);

  if (!tpsWrite(TPS_UPSEQ0, 0xE1)) return false;
  if (!tpsWrite(TPS_UPSEQ1, 0xAA)) return false;
  if (!tpsWrite(TPS_ENABLE, 0x3F)) return false;

  // Dont set vcomm... this chip should already konow it.
  //setVcomMv(config.power.vcom_mv);

  uint8_t v1 = 0, v2 = 0;
  tpsRead(TPS_VCOM1, v1);
  tpsRead(TPS_VCOM2, v2);
  int vcom_mv = -(((int)(v2 & 0x01) << 8 | v1) * 10);
  EPD_DEBUG_PRINT("[PWRCTL] TPS VCOM = %d mV (VCOM1=0x%02X VCOM2=0x%02X)\n", vcom_mv, v1, v2);

  EPD_DEBUG_PRINT("[PWRCTL] Waiting for TPS PG...");
  timeout = 0;
  uint8_t pg = 0;
  while (timeout < 400) {
    if (!tpsRead(TPS_PG, pg)) {
      printf("[PWRCTL] TPS PG read error\n");
      return false;
    }
    if ((pg & 0xFA) == 0xFA) break;
    EPD_DELAY_MS(1);
    ++timeout;
  }

  if ((pg & 0xFA) != 0xFA) {
    printf("[PWRCTL] TPS PG timeout (PG=0x%02X after %d ms)\n", pg, timeout);
    return false;
  }
  EPD_DEBUG_PRINT(" PG=0x%02X (%d ms) OK\n", pg, timeout);
  return true;
}

void epd_painter_powerctl::powerOff() {
  if (_mtx) xSemaphoreTake(_mtx, portMAX_DELAY);
  EPD_DEBUG_PRINT("[PWRCTL] Power-off... \n");

  // Disable the TPS65185 rails (VPOS/VNEG/VGH/VGL/VCOM) first.  Without
  // this the rails stay hot even though the PCA9555 controls go low,
  // which lets charge bleed through the panel and pulls pixels darker
  // over time — newly visible once WiFi started running alongside the
  // display.  Pairs with TPS_ENABLE=0x3F set in powerOn().
  tpsWrite(TPS_ENABLE, 0x00);

  pcaWrite(PIN_OE, false);
  pcaWrite(PIN_MODE, false);
  pcaWrite(PIN_PWRUP, false);
  pcaWrite(PIN_VCOM, false);
  EPD_DELAY_MS(1);
  pcaWrite(PIN_WAKEUP, false);
  if (_mtx) xSemaphoreGive(_mtx);
}

bool epd_painter_powerctl::isPwrGood() {
  bool val = false;
  if (!pcaRead(PIN_PWRGOOD, val)) return false;
  return val;
}

uint8_t epd_painter_powerctl::readTpsPg() {
  uint8_t val = 0;
  tpsRead(TPS_PG, val);
  return val;
}

int epd_painter_powerctl::readTemperatureC() {
  if (_mtx) xSemaphoreTake(_mtx, portMAX_DELAY);
  int r = readTemperatureCLocked();
  if (_mtx) xSemaphoreGive(_mtx);
  return r;
}

int epd_painter_powerctl::readTemperatureCLocked() {
  // The TPS65185 only answers I2C in STANDBY/ACTIVE (WAKEUP high). If it is
  // asleep, raise WAKEUP for the read and drop it again afterwards.
  bool wasAwake = (_pca_out[1] >> (PIN_WAKEUP - 8)) & 0x01;
  if (!wasAwake) {
    if (!pcaWrite(PIN_WAKEUP, true)) return TEMP_UNAVAILABLE;
    EPD_DELAY_MS(4);  // datasheet: I2C ready ~1.8ms after WAKEUP
  }

  int result = TEMP_UNAVAILABLE;
  uint8_t t1 = 0;
  if (tpsRead(TPS_TMST1, t1) && tpsWrite(TPS_TMST1, t1 | 0x80)) {  // READ_THERM
    for (int i = 0; i < 50; ++i) {
      if (!tpsRead(TPS_TMST1, t1)) break;
      if (t1 & 0x20) {  // CONV_END
        uint8_t v = 0;
        if (tpsRead(TPS_TMST_VALUE, v)) result = (int8_t)v;
        break;
      }
      EPD_DELAY_MS(1);
    }
  }

  if (!wasAwake) pcaWrite(PIN_WAKEUP, false);
  return result;
}

uint8_t epd_painter_powerctl::readPcaPort(uint8_t port) {
  uint8_t val = 0;
  if (port > 1) return 0;
  pcaReadReg(port, val);
  return val;
}


// This is a bad function.  
// The spec sheet does not match this...in parrticular, the hi register has a set of control bits.
// DO NOT USE
void epd_painter_powerctl::setVcomMv(int vcom_mv) {
  int mag = vcom_mv < 0 ? -vcom_mv : vcom_mv;
  mag /= 10;
  uint8_t lo = static_cast<uint8_t>(mag & 0xFF);
  uint8_t hi = static_cast<uint8_t>((mag >> 8) & 0xFF);
  tpsWrite16(TPS_VCOM1, lo, hi);
}

// ---- PCA low-level ----

bool epd_painter_powerctl::pcaWriteReg(uint8_t reg, uint8_t val) {
#ifdef ARDUINO
  config.i2c.wire->beginTransmission(config.power.pca_addr);
  config.i2c.wire->write(reg);
  config.i2c.wire->write(val);
  uint8_t err = config.i2c.wire->endTransmission();
  if (err) printf("[PWRCTL] pcaWriteReg(0x%02X)=I2C err %d (task %s core %d)\n",
                  reg, err, pcTaskGetName(nullptr), xPortGetCoreID());
  return (err == 0);
#else
  return false;  // I2C not used in ESP-IDF builds
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
  return false;  // I2C not used in ESP-IDF builds
#endif
}

bool epd_painter_powerctl::pcaPinMode(uint8_t pin, uint8_t mode) {
  uint8_t port = pin / 8;
  uint8_t bit  = pin % 8;
  if (port > 1) return false;

  if (mode == PCA_INPUT) {
    _pca_cfg[port] |= (1 << bit);
  } else {
    _pca_cfg[port] &= ~(1 << bit);
  }
  return pcaWriteReg(6 + port, _pca_cfg[port]);
}

bool epd_painter_powerctl::pcaWrite(uint8_t pin, bool val) {
  uint8_t port = pin / 8;
  uint8_t bit  = pin % 8;
  if (port > 1) return false;

  if (val) {
    _pca_out[port] |= (1 << bit);
  } else {
    _pca_out[port] &= ~(1 << bit);
  }
  return pcaWriteReg(2 + port, _pca_out[port]);
}

bool epd_painter_powerctl::pcaRead(uint8_t pin, bool& val) {
  uint8_t port    = pin / 8;
  uint8_t bit     = pin % 8;
  uint8_t reg_val = 0;
  if (port > 1) return false;
  if (!pcaReadReg(port, reg_val)) return false;
  val = ((reg_val >> bit) & 0x01) != 0;
  return true;
}

// ---- TPS low-level ----

bool epd_painter_powerctl::tpsWrite(uint8_t reg, uint8_t val) {
#ifdef ARDUINO
  config.i2c.wire->beginTransmission(config.power.tps_addr);
  config.i2c.wire->write(reg);
  config.i2c.wire->write(val);
  return (config.i2c.wire->endTransmission() == 0);
#else
  return false;  // I2C not used in ESP-IDF builds
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
  return false;  // I2C not used in ESP-IDF builds
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
  return false;  // I2C not used in ESP-IDF builds
#endif
}

bool epd_painter_powerctl_74HCT4094D::begin(EPD_Painter::Config& cfg) {
  gpio_reset_pin((gpio_num_t)cfg.shift.clk);
  EPD_PIN_OUTPUT(cfg.shift.data);
  EPD_PIN_OUTPUT(cfg.shift.clk);
  EPD_PIN_OUTPUT(cfg.shift.strobe);
  EPD_PIN_LOW(cfg.shift.strobe);
  _sr_hw = EPD_ShiftReg(cfg.shift.data, cfg.shift.clk, cfg.shift.strobe, cfg.shift.le_time);
  _sr_hw.reset();
  EPD_DEBUG_PRINT("[PWRCTL] H752 shift-reg init OK (DATA=%d CLK=%d STR=%d)\n",
                  cfg.shift.data, cfg.shift.clk, cfg.shift.strobe);
  return true;
}

bool epd_painter_powerctl_74HCT4094D::powerOn() {
  EPD_DEBUG_PRINT("[PWRCTL] H752 power-on...\n");
  // Ensure all drive signals are low before enabling power.
  _pin_le.set(false);
  _pin_stv.set(false);
  _pin_oe.set(false);
  _pin_mode.set(false);
  // Bring panel rails up, then enable driver outputs.
  _pin_pwr.set(true);
  EPD_DELAY_US(200);
  _pin_mode.set(true);
  _pin_oe.set(true);
  return true;
}

void epd_painter_powerctl_74HCT4094D::powerOff() {
  EPD_DEBUG_PRINT("[PWRCTL] H752 power-off...\n");
  // Disable panel drive before removing power.
  _pin_le.set(false);
  _pin_stv.set(false);
  _pin_mode.set(false);
  _pin_oe.set(false);
  EPD_DELAY_US(100);
  _pin_pwr.set(false);
}

// =============================================================================
// EPD_H716PowerDriver implementation
// =============================================================================

bool EPD_H716PowerDriver::begin(const EPD_Painter::Shift& shift) {
  gpio_reset_pin((gpio_num_t)shift.clk);
  EPD_PIN_OUTPUT(shift.data);
  EPD_PIN_OUTPUT(shift.clk);
  EPD_PIN_OUTPUT(shift.strobe);
  EPD_PIN_LOW(shift.strobe);
  _sr_hw = EPD_ShiftReg(shift.data, shift.clk, shift.strobe, shift.le_time);
  _sr_hw.reset();
  // Safe startup state: power disabled, scan direction set.
  _pin_pwr_dis.set(true);
  _pin_scan_dir.set(true);
  return true;
}

bool EPD_H716PowerDriver::powerOn() {
  // Initial state: all drive outputs low, power disabled.
  _pin_le.set(false);
  _pin_stv.set(false);
  _pin_oe.set(false);
  _pin_mode.set(false);
  _pin_pos_pwr.set(false);
  _pin_neg_pwr.set(false);
  _pin_scan_dir.set(true);
  _pin_pwr_dis.set(false);   // clear power-disable → rails start ramping
  EPD_DELAY_US(100);

  _pin_neg_pwr.set(true);    // negative supply on
  EPD_DELAY_US(500);

  _pin_pos_pwr.set(true);    // positive supply on
  EPD_DELAY_US(100);

  _pin_stv.set(true);        // STV high — panel scan ready
  _pin_mode.set(true);
  _pin_oe.set(true);
  return true;
}

void EPD_H716PowerDriver::powerOff() {
  _pin_pos_pwr.set(false);
  EPD_DELAY_US(10);

  _pin_neg_pwr.set(false);
  EPD_DELAY_US(100);

  _pin_mode.set(false);
  _pin_oe.set(false);
  _pin_stv.set(false);
  _pin_pwr_dis.set(true);    // re-engage power-disable
}
