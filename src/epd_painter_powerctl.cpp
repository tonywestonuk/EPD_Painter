#include "epd_painter_powerctl.h"
#include <stdio.h>

#ifndef ARDUINO
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "driver/i2c_master.h"
#endif

epd_painter_powerctl::epd_painter_powerctl() {
  _pca_out[0] = 0x00;
  _pca_out[1] = 0x00;
  _pca_cfg[0] = 0xFF;
  _pca_cfg[1] = 0xFF;
}

bool epd_painter_powerctl::begin(EPD_Painter::Config cfg) {

  config = cfg;

#ifndef ARDUINO
  // ---- Create per-device I2C handles from the shared bus ----
  i2c_device_config_t pca_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address  = (uint16_t)config.power.pca_addr,
    .scl_speed_hz    = config.i2c.freq,
  };
  if (i2c_master_bus_add_device(config.i2c.i2c_bus, &pca_dev_cfg, &_pca_dev) != ESP_OK) {
    printf("[PWRCTL] Failed to add PCA I2C device\n");
    return false;
  }

  i2c_device_config_t tps_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address  = (uint16_t)config.power.tps_addr,
    .scl_speed_hz    = config.i2c.freq,
  };
  if (i2c_master_bus_add_device(config.i2c.i2c_bus, &tps_dev_cfg, &_tps_dev) != ESP_OK) {
    printf("[PWRCTL] Failed to add TPS I2C device\n");
    return false;
  }
#endif

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
  printf("[PWRCTL] Power-on sequence... \n");

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
    if (!pcaRead(PIN_PWRGOOD, good)) {
      printf(" read error \n");
      return false;
    }
    if (good) break;
    EPD_DELAY_MS(1);
    ++timeout;
  }
  if (!good) {
    printf(" TIMEOUT \n");
    return false;
  }
  printf(" OK (%d ms)\n", timeout);

  if (!tpsWrite(TPS_UPSEQ0, 0xE1)) return false;
  if (!tpsWrite(TPS_UPSEQ1, 0xAA)) return false;
  if (!tpsWrite(TPS_ENABLE, 0x3F)) return false;

  // Dont set vcomm... this chip should already konow it. 
  //setVcomMv(config.power.vcom_mv);

  uint8_t val = 0;
  tpsRead(TPS_VCOM1, val);
  
  printf("[PWRCTL] Reading TPS VCOM1... %d \n", val);
  tpsRead(TPS_VCOM2, val);
  printf("[PWRCTL] Reading TPS VCOM2... %d \n", val);



  printf("[PWRCTL] Waiting for TPS PG...");
  timeout = 0;
  uint8_t pg = 0;
  while (timeout < 400) {
    if (!tpsRead(TPS_PG, pg)) {
      printf(" read error \n");
      return false;
    }
    if ((pg & 0xFA) == 0xFA) break;
    EPD_DELAY_MS(1);
    ++timeout;
  }

  printf(" PG=0x%02X (%d ms) %s\n",
         pg, timeout, ((pg & 0xFA) == 0xFA) ? "OK" : "TIMEOUT");

  return ((pg & 0xFA) == 0xFA);
}

void epd_painter_powerctl::powerOff() {
  printf("[PWRCTL] Power-off... \n");

  pcaWrite(PIN_OE, false);
  pcaWrite(PIN_MODE, false);
  pcaWrite(PIN_PWRUP, false);
  pcaWrite(PIN_VCOM, false);
  EPD_DELAY_MS(1);
  pcaWrite(PIN_WAKEUP, false);
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
  return (config.i2c.wire->endTransmission() == 0);
#else
  uint8_t buf[2] = { reg, val };
  return i2c_master_transmit(_pca_dev, buf, 2, pdMS_TO_TICKS(100)) == ESP_OK;
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
  return i2c_master_transmit_receive(_pca_dev, &reg, 1, &val, 1, pdMS_TO_TICKS(100)) == ESP_OK;
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
  uint8_t buf[2] = { reg, val };
  return i2c_master_transmit(_tps_dev, buf, 2, pdMS_TO_TICKS(100)) == ESP_OK;
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
  uint8_t buf[3] = { reg, lo, hi };
  return i2c_master_transmit(_tps_dev, buf, 3, pdMS_TO_TICKS(100)) == ESP_OK;
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
  return i2c_master_transmit_receive(_tps_dev, &reg, 1, &val, 1, pdMS_TO_TICKS(100)) == ESP_OK;
#endif
}
