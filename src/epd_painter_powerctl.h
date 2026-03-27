#pragma once

#include "build_opt.h"
#include <EPD_Painter.h>
class epd_painter_powerctl {
public:
  epd_painter_powerctl();

  bool begin(EPD_Painter::Config config);

  bool powerOn();
  void powerOff();

  bool isPwrGood();
  uint8_t readTpsPg();
  uint8_t readPcaPort(uint8_t port);

  void setVcomMv(int vcom_mv);

  // ---- IRAM-safe shift-register control (H752 only) ----
  // Called from sendRow() hot path; use direct GPIO register writes.
  void IRAM_ATTR sr_set_le(bool val);   // toggle ep_latch_enable bit
  void IRAM_ATTR sr_set_stv(bool val);  // toggle ep_stv bit

private:
  EPD_Painter::Config config;

  // ---- Shift-register state (74HCT4094D, H752) ----
  // Bit layout pushed MSB-first (Q7..Q0):
  //   Q7=ep_output_enable, Q6=ep_mode, Q5=ep_scan_direction, Q4=ep_stv,
  //   Q3=neg_power_enable, Q2=pos_power_enable, Q1=power_disable, Q0=ep_latch_enable
  struct ShiftState {
    bool power_disable     = true;
    bool pos_power_enable  = false;
    bool neg_power_enable  = false;
    bool ep_scan_direction = true;
    bool ep_output_enable  = false;
    bool ep_mode           = false;
    bool ep_stv            = false;
    bool ep_latch_enable   = false;
  } _sr;

  // Push _sr to the 74HCT4094D via bit-banging (slow, uses digitalWrite)
  void sr_push_slow();

  // ---- PCA9535 cached state ----
  uint8_t _pca_out[2];
  uint8_t _pca_cfg[2];

  // ---- PCA9535 logical pin mapping ----
  static constexpr uint8_t PIN_OE      = 8;   // port1 bit0
  static constexpr uint8_t PIN_MODE    = 9;   // port1 bit1
  static constexpr uint8_t PIN_PWRUP   = 11;  // port1 bit3
  static constexpr uint8_t PIN_VCOM    = 12;  // port1 bit4
  static constexpr uint8_t PIN_WAKEUP  = 13;  // port1 bit5
  static constexpr uint8_t PIN_PWRGOOD = 14;  // port1 bit6 input
  static constexpr uint8_t PIN_INT     = 15;  // port1 bit7 input

  // ---- PCA pin direction constants (I2C expander, not real GPIO) ----
  static constexpr uint8_t PCA_OUTPUT = 0;
  static constexpr uint8_t PCA_INPUT  = 1;

  // ---- TPS65185 registers ----
  static constexpr uint8_t TPS_ENABLE = 0x01;
  static constexpr uint8_t TPS_VCOM1  = 0x03;
  static constexpr uint8_t TPS_VCOM2  = 0x04;
  static constexpr uint8_t TPS_UPSEQ0 = 0x09;
  static constexpr uint8_t TPS_UPSEQ1 = 0x0A;
  static constexpr uint8_t TPS_PG     = 0x0F;

  // ---- PCA low-level ----
  bool pcaWriteReg(uint8_t reg, uint8_t val);
  bool pcaReadReg(uint8_t reg, uint8_t& val);
  bool pcaPinMode(uint8_t pin, uint8_t mode);
  bool pcaWrite(uint8_t pin, bool val);
  bool pcaRead(uint8_t pin, bool& val);

  // ---- TPS low-level ----
  bool tpsWrite(uint8_t reg, uint8_t val);
  bool tpsWrite16(uint8_t reg, uint8_t lo, uint8_t hi);
  bool tpsRead(uint8_t reg, uint8_t& val);
};
