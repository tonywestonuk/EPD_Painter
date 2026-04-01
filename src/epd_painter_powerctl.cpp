#include "epd_painter_powerctl.h"
#include <stdio.h>
#include <hal/gpio_hal.h>

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

  uint8_t v1 = 0, v2 = 0;
  tpsRead(TPS_VCOM1, v1);
  tpsRead(TPS_VCOM2, v2);
  int vcom_mv = -(((int)(v2 & 0x01) << 8 | v1) * 10);
  printf("[PWRCTL] TPS VCOM = %d mV (VCOM1=0x%02X VCOM2=0x%02X)\n", vcom_mv, v1, v2);

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

//#define EPD_DEBUG_REGISTERS
#ifdef EPD_DEBUG_REGISTERS
#define EPD_DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define EPD_DEBUG_PRINT(...)
#endif

epd_painter_powerctl_74HCT4094D::epd_painter_powerctl_74HCT4094D() {}

bool epd_painter_powerctl_74HCT4094D::begin(EPD_Painter::Config& cfg) {
  config = &cfg;
  gpio_reset_pin((gpio_num_t)config->shift.clk);
  EPD_PIN_OUTPUT(config->shift.data);
  EPD_PIN_OUTPUT(config->shift.clk);
  EPD_PIN_OUTPUT(config->shift.strobe);
  EPD_PIN_LOW(config->shift.strobe);
  _sr = ShiftState{};
  sr_push_bits();
  EPD_DEBUG_PRINT("[PWRCTL] Shift-reg init OK (DATA=%d CLK=%d STR=%d)\n",
                  config->shift.data, config->shift.clk, config->shift.strobe);
  return true;
}

// ---------------------------------------------------------------------------
// IRAM-safe GPIO helpers (direct register writes, no flash access)
// ---------------------------------------------------------------------------
static IRAM_ATTR inline void iram_pin_set(int pin) {
    if (pin < 32) REG_WRITE(GPIO_OUT_W1TS_REG,  1UL << pin);
    else          REG_WRITE(GPIO_OUT1_W1TS_REG, 1UL << (pin - 32));
}
static IRAM_ATTR inline void iram_pin_clr(int pin) {
    if (pin < 32) REG_WRITE(GPIO_OUT_W1TC_REG,  1UL << pin);
    else          REG_WRITE(GPIO_OUT1_W1TC_REG, 1UL << (pin - 32));
}

void epd_painter_powerctl_74HCT4094D::sr_push_bits() {
// ---------------------------------------------------------------------------
// 74HCT4094D timing requirements at 5V (3.3V operation is slightly slower):
//   tsu  (DATA setup before CLK rising) = 20ns min
//   th   (DATA hold after  CLK rising)  = 5ns  min
//   tw   (CLK pulse width high/low)     = 15ns min
//
// At 240 MHz, one REG_WRITE or NOP = ~4 ns.
// ---------------------------------------------------------------------------
#define SR_NOP6  __asm volatile("nop;nop;nop;nop;nop;nop;" ::: "memory")
#define SR_NOP2  __asm volatile("nop;nop;" ::: "memory")

  const int data = config->shift.data;
  const int clk  = config->shift.clk;
  const int str  = config->shift.strobe;

  uint8_t byte =
    (_sr.ep_output_enable  ? 0x80 : 0) | // QP7 -> EP_OE
    (_sr.ep_mode           ? 0x40 : 0) | // QP6 -> EP_MODE
    (_sr.power_enable      ? 0x20 : 0) | // QP5 -> PWR_EN
    (_sr.ep_stv            ? 0x10 : 0) | // QP4 -> EP_STV
    (_sr.q3_unused         ? 0x08 : 0) | // QP3
    (_sr.q2_unused         ? 0x04 : 0) | // QP2
    (_sr.q1_unused         ? 0x02 : 0) | // QP1
    (_sr.ep_latch_enable   ? 0x01 : 0);  // QP0 -> EP_LE

  iram_pin_clr(str);
  for (int i = 7; i >= 0; i--) {
    iram_pin_clr(clk);           // CLK low  (~4 ns)
    SR_NOP2;                     // hold low (~9 ns)
    if ((byte >> i) & 1) iram_pin_set(data);
    else                  iram_pin_clr(data);
    SR_NOP6;                     // DATA setup time ~25 ns  (> 20 ns required)
    iram_pin_set(clk);           // CLK rising edge — latch bit
    SR_NOP2;                     // DATA hold time   ~9 ns  (> 5 ns required)
  }
  iram_pin_clr(clk);
  iram_pin_set(str);             // STR rising edge — latch byte to outputs

}

void IRAM_ATTR epd_painter_powerctl_74HCT4094D::sr_set_le(bool val) {
  _sr.ep_latch_enable = val;
  sr_push_bits();
  if(val && config->shift.le_time > 0) EPD_DELAY_US(config->shift.le_time);
}

void IRAM_ATTR epd_painter_powerctl_74HCT4094D::sr_set_stv(bool val) {
  _sr.ep_stv = val;
  sr_push_bits();
}

bool epd_painter_powerctl_74HCT4094D::powerOn() {
  EPD_DEBUG_PRINT("[PWRCTL] Shift-reg power-on...\n");
  _sr.ep_latch_enable = false;
  _sr.ep_stv = false;
  _sr.ep_output_enable = false;
  _sr.ep_mode = false;
  sr_push_bits();

  // H752 schematic:
  //   QP0=EP_LE, QP4=EP_STV, QP5=PWR_EN, QP6=EP_MODE, QP7=EP_OE.
  // Bring panel rails up first, then enable the driver outputs.
  _sr.power_enable = true;
  sr_push_bits();
  EPD_DELAY_US(200);

  _sr.ep_mode = true;
  _sr.ep_output_enable = true;
  sr_push_bits();
  return true;
}

void epd_painter_powerctl_74HCT4094D::powerOff() {
  EPD_DEBUG_PRINT("[PWRCTL] Shift-reg power-off...\n");
  // Disable panel drive before removing power.
  _sr.ep_latch_enable = false;
  _sr.ep_stv = false;
  _sr.ep_mode = false;
  _sr.ep_output_enable = false;
  sr_push_bits();
  EPD_DELAY_US(100);

  _sr.power_enable = false;
  sr_push_bits();
}
