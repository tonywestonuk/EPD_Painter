#include "epd_painter_powerctl.h"

epd_painter_powerctl::epd_painter_powerctl() {
  _pca_out[0] = 0x00;
  _pca_out[1] = 0x00;
  _pca_cfg[0] = 0xFF;
  _pca_cfg[1] = 0xFF;
}

bool epd_painter_powerctl::begin(EPD_Painter::Config cfg) {

  config=cfg;
  // Pins 8..13 outputs, 14..15 inputs
  for (int pin = 8; pin <= 13; ++pin) {
    if (!pcaPinMode(pin, OUTPUT)) {
      Serial.printf("[PWRCTL] Failed setting PCA pin %d OUTPUT\n", pin);
      return false;
    }
  }
  if (!pcaPinMode(14, INPUT)) {
    Serial.println("[PWRCTL] Failed setting PCA pin 14 INPUT");
    return false;
  }
  if (!pcaPinMode(15, INPUT)) {
    Serial.println("[PWRCTL] Failed setting PCA pin 15 INPUT");
    return false;
  }

  Serial.printf("[PWRCTL] PCA init OK. CFG1=0x%02X OUT1=0x%02X\n",
                _pca_cfg[1], _pca_out[1]);
  return true;
}

bool epd_painter_powerctl::powerOn() {
  Serial.println("[PWRCTL] Power-on sequence...");

  // 1. OE on, MODE on
  if (!pcaWrite(PIN_OE, true)) return false;
  if (!pcaWrite(PIN_MODE, true)) return false;

  // 2. WAKEUP on
  if (!pcaWrite(PIN_WAKEUP, true)) return false;

  // 3. PWRUP on
  if (!pcaWrite(PIN_PWRUP, true)) return false;

  // 4. VCOM control on
  if (!pcaWrite(PIN_VCOM, true)) return false;

  delay(3);

  // 5. Wait for expander PWRGOOD
  Serial.print("[PWRCTL] Waiting for PWR_GOOD...");
  int timeout = 0;
  bool good = false;
  while (timeout < 400) {
    if (!pcaRead(PIN_PWRGOOD, good)) {
      Serial.println(" read error");
      return false;
    }
    if (good) break;
    delay(1);
    ++timeout;
  }
  if (!good) {
    Serial.println(" TIMEOUT");
    return false;
  }
  Serial.printf(" OK (%d ms)\n", timeout);

  // 6. Configure TPS
  if (!tpsWrite(TPS_UPSEQ0, 0xE1)) return false;
  if (!tpsWrite(TPS_UPSEQ1, 0xAA)) return false;
  if (!tpsWrite(TPS_ENABLE, 0x3F)) return false;

  // 7. Set VCOM
  setVcomMv(config.power.vcom_mv);

  // 8. Wait for TPS power good bits
  Serial.print("[PWRCTL] Waiting for TPS PG...");
  timeout = 0;
  uint8_t pg = 0;
  while (timeout < 400) {
    if (!tpsRead(TPS_PG, pg)) {
      Serial.println(" read error");
      return false;
    }
    if ((pg & 0xFA) == 0xFA) break;
    delay(1);
    ++timeout;
  }

  Serial.printf(" PG=0x%02X (%d ms) %s\n",
                pg, timeout, ((pg & 0xFA) == 0xFA) ? "OK" : "TIMEOUT");

  return ((pg & 0xFA) == 0xFA);
}


void epd_painter_powerctl::powerOff() {
  Serial.println("[PWRCTL] Power-off...");

  pcaWrite(PIN_OE, false);
  pcaWrite(PIN_MODE, false);
  pcaWrite(PIN_PWRUP, false);
  pcaWrite(PIN_VCOM, false);
  delay(1);
  pcaWrite(PIN_WAKEUP, false);
}

bool epd_painter_powerctl::isPwrGood() {
  bool val = false;
  if (!pcaRead(PIN_PWRGOOD, val)) {
    return false;
  }
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

void epd_painter_powerctl::setVcomMv(int vcom_mv) {
  int mag = abs(vcom_mv) / 10;
  uint8_t lo = static_cast<uint8_t>(mag & 0xFF);
  uint8_t hi = static_cast<uint8_t>((mag >> 8) & 0xFF);

  tpsWrite16(TPS_VCOM1, lo, hi);
}

// ---------------- PCA low-level ----------------

bool epd_painter_powerctl::pcaWriteReg(uint8_t reg, uint8_t val) {
  config.i2c.wire->beginTransmission(config.power.pca_addr);
  config.i2c.wire->write(reg);
  config.i2c.wire->write(val);
  return (config.i2c.wire->endTransmission() == 0);
}

bool epd_painter_powerctl::pcaReadReg(uint8_t reg, uint8_t& val) {
  config.i2c.wire->beginTransmission(config.power.pca_addr);
  config.i2c.wire->write(reg);
  if (config.i2c.wire->endTransmission(false) != 0) {
    return false;
  }

  int n = config.i2c.wire->requestFrom(config.power.pca_addr, 1);
  if (n != 1 || !config.i2c.wire->available()) {
    return false;
  }

  val = config.i2c.wire->read();
  return true;
}

bool epd_painter_powerctl::pcaPinMode(uint8_t pin, uint8_t mode) {
  uint8_t port = pin / 8;
  uint8_t bit = pin % 8;

  if (port > 1) return false;

  if (mode == INPUT) {
    _pca_cfg[port] |= (1 << bit);
  } else {
    _pca_cfg[port] &= ~(1 << bit);
  }

  return pcaWriteReg(6 + port, _pca_cfg[port]);
}

bool epd_painter_powerctl::pcaWrite(uint8_t pin, bool val) {
  uint8_t port = pin / 8;
  uint8_t bit = pin % 8;

  if (port > 1) return false;

  if (val) {
    _pca_out[port] |= (1 << bit);
  } else {
    _pca_out[port] &= ~(1 << bit);
  }

  return pcaWriteReg(2 + port, _pca_out[port]);
}

bool epd_painter_powerctl::pcaRead(uint8_t pin, bool& val) {
  uint8_t port = pin / 8;
  uint8_t bit = pin % 8;
  uint8_t reg_val = 0;

  if (port > 1) return false;
  if (!pcaReadReg(port, reg_val)) return false;

  val = ((reg_val >> bit) & 0x01) != 0;
  return true;
}

// ---------------- TPS low-level ----------------

bool epd_painter_powerctl::tpsWrite(uint8_t reg, uint8_t val) {
  config.i2c.wire->beginTransmission(config.power.tps_addr);
  config.i2c.wire->write(reg);
  config.i2c.wire->write(val);
  return (config.i2c.wire->endTransmission() == 0);
}

bool epd_painter_powerctl::tpsWrite16(uint8_t reg, uint8_t lo, uint8_t hi) {
  config.i2c.wire->beginTransmission(config.power.tps_addr);
  config.i2c.wire->write(reg);
  config.i2c.wire->write(lo);
  config.i2c.wire->write(hi);
  return (config.i2c.wire->endTransmission() == 0);
}

bool epd_painter_powerctl::tpsRead(uint8_t reg, uint8_t& val) {
  config.i2c.wire->beginTransmission(config.power.tps_addr);
  config.i2c.wire->write(reg);
  if (config.i2c.wire->endTransmission(false) != 0) {
    return false;
  }

  int n = config.i2c.wire->requestFrom(config.power.tps_addr, 1);
  if (n != 1 || !config.i2c.wire->available()) {
    return false;
  }

  val = config.i2c.wire->read();
  return true;
}