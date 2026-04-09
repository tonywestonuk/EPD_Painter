#pragma once

#include "build_opt.h"
#include <EPD_Painter.h>
#include "epd_pin_driver.h"

// =============================================================================
// epd_painter_powerctl — TPS65185 PMIC + PCA9535 I/O expander (M5PaperS3).
// =============================================================================
class epd_painter_powerctl : public EPD_PowerDriver {
public:
  epd_painter_powerctl();

  bool begin(EPD_Painter::Config config);

  bool powerOn();
  void powerOff();

  bool isPwrGood();
  uint8_t readTpsPg();
  uint8_t readPcaPort(uint8_t port);

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

// =============================================================================
// EPD_GpioPowerDriver — direct GPIO power control (e.g. M5PaperS3).
// =============================================================================
class EPD_GpioPowerDriver : public EPD_PowerDriver {
public:
    EPD_GpioPowerDriver(int8_t pin_oe, int8_t pin_pwr)
        : _pin_oe(pin_oe), _pin_pwr(pin_pwr) {}

    bool powerOn() override {
        EPD_PIN_HIGH(_pin_oe);
        EPD_DELAY_US(100);
        EPD_PIN_HIGH(_pin_pwr);
        EPD_DELAY_US(100);
        return true;
    }

    void powerOff() override {
        EPD_PIN_LOW(_pin_oe);
        EPD_DELAY_US(100);
        EPD_PIN_LOW(_pin_pwr);
        EPD_DELAY_US(100);
    }

private:
    int8_t _pin_oe;
    int8_t _pin_pwr;
};

// =============================================================================
// epd_painter_powerctl_74HCT4094D — shift-register power driver for H752.
//
//   QP0 = EP_LE   QP4 = EP_STV   QP5 = PWR_EN   QP6 = EP_MODE   QP7 = EP_OE
//   QP1-3 unused
// =============================================================================
class epd_painter_powerctl_74HCT4094D : public EPD_PowerDriver {
public:
  bool begin(EPD_Painter::Config& config);
  bool powerOn() override;
  void powerOff() override;
  EPD_ISRController* isrController() override { return &_sr_hw; }

private:
  EPD_ShiftReg _sr_hw;

  // Named pins for power sequencing — all route through _sr_hw
  EPD_SRPin _pin_le   {&_sr_hw, 0};  // QP0 -> EP_LE
  EPD_SRPin _pin_stv  {&_sr_hw, 4};  // QP4 -> EP_STV
  EPD_SRPin _pin_pwr  {&_sr_hw, 5};  // QP5 -> PWR_EN
  EPD_SRPin _pin_mode {&_sr_hw, 6};  // QP6 -> EP_MODE
  EPD_SRPin _pin_oe   {&_sr_hw, 7};  // QP7 -> EP_OE
};

// =============================================================================
// EPD_H716PowerDriver — shift-register power driver for LilyGo EPD47 H716.
//
//   QP0 = EP_LE    QP1 = PWR_DIS   QP2 = POS_PWR   QP3 = NEG_PWR
//   QP4 = EP_STV   QP5 = SCAN_DIR  QP6 = EP_MODE   QP7 = EP_OE
//
// Power sequence: clear PWR_DIS → NEG_PWR → POS_PWR
// (mirrors the epd_poweron() sequence in the upstream epdiy/EPD47 driver).
// =============================================================================
class EPD_H716PowerDriver : public EPD_PowerDriver {
public:
  bool begin(const EPD_Painter::Shift& shift);
  bool powerOn() override;
  void powerOff() override;
  EPD_ISRController* isrController() override { return &_sr_hw; }

private:
  EPD_ShiftReg _sr_hw;

  // Named pins for power sequencing — all route through _sr_hw
  EPD_SRPin _pin_le       {&_sr_hw, 0};  // QP0 -> EP_LE
  EPD_SRPin _pin_pwr_dis  {&_sr_hw, 1};  // QP1 -> PWR_DIS (active-high)
  EPD_SRPin _pin_pos_pwr  {&_sr_hw, 2};  // QP2 -> POS_PWR
  EPD_SRPin _pin_neg_pwr  {&_sr_hw, 3};  // QP3 -> NEG_PWR
  EPD_SRPin _pin_stv      {&_sr_hw, 4};  // QP4 -> EP_STV
  EPD_SRPin _pin_scan_dir {&_sr_hw, 5};  // QP5 -> SCAN_DIR
  EPD_SRPin _pin_mode     {&_sr_hw, 6};  // QP6 -> EP_MODE
  EPD_SRPin _pin_oe       {&_sr_hw, 7};  // QP7 -> EP_OE
};
