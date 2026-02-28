#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <EPD_Painter.h>

class epd_painter_powerctl {
public:
  // Constructor
  epd_painter_powerctl();

  // Init only the I2C side and expander direction cache
  bool begin(EPD_Painter::Config config);

  // Main power sequencing
  bool powerOn();
  void powerOff();

  // Diagnostics
  bool isPwrGood();
  uint8_t readTpsPg();
  uint8_t readPcaPort(uint8_t port);

  // Optional direct helpers if you want them elsewhere
  void setVcomMv(int vcom_mv);

private:
  EPD_Painter::Config config;

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