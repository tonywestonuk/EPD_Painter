/*
 * =============================================================================
 * EPD_painter.h - T5 E-Paper S3 Pro EPD Power Control
 * =============================================================================
 *
 * Arduino-compatible driver for the Lilygo T5 E-Paper S3 Pro board.
 * Controls the e-paper power subsystem the same way EPDIY v7 does:
 *
 *   - PCA9535PW (I2C GPIO expander, addr 0x20) controls the TPS65185
 *     control pins: WAKEUP, PWRUP, VCOM_CTRL, plus EP_OE and EP_MODE
 *   - TPS65185 (E-ink PMIC, addr 0x68) generates ±15V source, ±22V gate,
 *     and VCOM voltages for the e-paper panel
 *
 * Board: Lilygo T5 E-Paper S3 Pro (H752-01)
 * MCU:   ESP32-S3-WROOM-1
 * Panel: ED047TC1 (4.7", 960x540, 16 gray)
 */

#ifndef EPD_POWER_H
#define EPD_POWER_H

#include <Arduino.h>
#include <Wire.h>

// =============================================================================
// I2C Configuration
// =============================================================================
#define EPD_DEFAULT_SDA       39
#define EPD_DEFAULT_SCL       40
#define EPD_DEFAULT_I2C_FREQ  400000  // 400kHz

// =============================================================================
// I2C Device Addresses
// =============================================================================
#define PCA9535_ADDR      0x20   // PCA9535PW IO expander
#define TPS65185_ADDR     0x68   // TPS65185 E-ink PMIC

// =============================================================================
// PCA9535 Register Definitions
// =============================================================================
#define PCA9535_REG_INPUT_0       0x00
#define PCA9535_REG_INPUT_1       0x01
#define PCA9535_REG_OUTPUT_0      0x02
#define PCA9535_REG_OUTPUT_1      0x03
#define PCA9535_REG_POLARITY_0    0x04
#define PCA9535_REG_POLARITY_1    0x05
#define PCA9535_REG_CONFIG_0      0x06  // 1=input, 0=output
#define PCA9535_REG_CONFIG_1      0x07

// =============================================================================
// PCA9535 Port 1 Pin Assignments (from Lilygo T5 Pro schematic & EPDIY)
// =============================================================================
#define CFG_PIN_EP_OE        (1 << 0)  // IO1.0 - EP Output Enable (source driver)
#define CFG_PIN_EP_MODE      (1 << 1)  // IO1.1 - EP Mode selection (gate driver)
#define CFG_PIN_BUTTON       (1 << 2)  // IO1.2 - Button input
#define CFG_PIN_PWRUP        (1 << 3)  // IO1.3 - TPS65185 PWRUP control
#define CFG_PIN_VCOM_CTRL    (1 << 4)  // IO1.4 - VCOM control
#define CFG_PIN_WAKEUP       (1 << 5)  // IO1.5 - TPS65185 WAKEUP control
#define CFG_PIN_PWRGOOD      (1 << 6)  // IO1.6 - TPS65185 Power Good (INPUT)
#define CFG_PIN_INT          (1 << 7)  // IO1.7 - TPS65185 Interrupt (INPUT)

// Port 1 direction mask: PWRGOOD, INT, BUTTON are inputs; rest outputs
#define PORT1_DIR_MASK  (CFG_PIN_PWRGOOD | CFG_PIN_INT | CFG_PIN_BUTTON)  // 0xC4
#define PORT0_DIR_MASK  0xFF  // All inputs

// =============================================================================
// TPS65185 Register Definitions (from TI datasheet)
// =============================================================================
#define TPS_REG_TMST_VALUE    0x00
#define TPS_REG_ENABLE        0x01
#define TPS_REG_VADJ          0x02
#define TPS_REG_VCOM1         0x03
#define TPS_REG_VCOM2         0x04
#define TPS_REG_INT_EN1       0x05
#define TPS_REG_INT_EN2       0x06
#define TPS_REG_INT1          0x07
#define TPS_REG_INT2          0x08
#define TPS_REG_UPSEQ0        0x09
#define TPS_REG_UPSEQ1        0x0A
#define TPS_REG_DWNSEQ0       0x0B
#define TPS_REG_DWNSEQ1       0x0C
#define TPS_REG_TMST1         0x0D
#define TPS_REG_TMST2         0x0E
#define TPS_REG_PG            0x0F
#define TPS_REG_REVID         0x10

// TPS_REG_ENABLE bits
#define TPS_ENABLE_ACTIVE     0x80
#define TPS_ENABLE_STANDBY    0x40
#define TPS_ENABLE_V3P3_EN    0x20
#define TPS_ENABLE_VCOM_EN    0x10
#define TPS_ENABLE_VDDH_EN   0x08
#define TPS_ENABLE_VPOS_EN   0x04
#define TPS_ENABLE_VEE_EN    0x02
#define TPS_ENABLE_VNEG_EN   0x01

// TPS_REG_VCOM2 bits
#define TPS_VCOM2_HIZ        0x40
#define TPS_VCOM2_ACQ        0x80

// TPS_REG_INT1 bits
#define TPS_INT1_ACQC        0x02
#define TPS_INT1_PRGC        0x01

// TPS_REG_INT2 bits
#define TPS_INT2_VB_UV       0x80
#define TPS_INT2_VDDH_UV     0x40
#define TPS_INT2_VN_UV       0x20
#define TPS_INT2_VPOS_UV     0x10
#define TPS_INT2_VEE_UV      0x08
#define TPS_INT2_VCOMF       0x04
#define TPS_INT2_VNEG_UV     0x02
#define TPS_INT2_EOC         0x01


class EPD_painter_power {
public:
    /**
     * Construct with optional I2C pin and frequency overrides.
     */
    EPD_painter_power(int sda = EPD_DEFAULT_SDA, int scl = EPD_DEFAULT_SCL,
             uint32_t i2cFreq = EPD_DEFAULT_I2C_FREQ);

    /**
     * Initialize I2C bus, PCA9535, and TPS65185. Call once in setup().
     * @param scanBus  If true, prints an I2C bus scan to Serial.
     * @return true on success.
     */
    bool begin(bool scanBus = true);

    /**
     * Power ON the e-paper display rails and enable VCOM.
     * Must be called before writing to the display.
     */
    bool powerOn();

    /**
     * Power OFF the e-paper display rails.
     * Sequences rails down properly via TPS65185.
     */
    bool powerOff();

    /**
     * Full shutdown: power off + de-assert WAKEUP → TPS65185 SLEEP.
     * Use before ESP32 deep sleep for minimum current draw.
     */
    void sleep();

    /**
     * Wake from sleep state. Re-asserts WAKEUP and verifies TPS65185.
     * @return true if TPS65185 responds after wake.
     */
    bool wake();

    /**
     * Check if power rails are currently on.
     */
    bool isPowerOn() const { return _powerOn; }

    // ---- VCOM Control ----

    /**
     * Set VCOM voltage in millivolts (as a positive number).
     * Example: setVCOM(1500) sets VCOM to -1.500V.
     * Range: 0–5110 mV in 10mV steps.
     */
    bool setVCOM(uint16_t millivolts);

    /**
     * Read current VCOM setting in millivolts.
     * @return VCOM in mV, or -1 on error.
     */
    int getVCOM();

    // ---- Temperature ----

    /**
     * Read temperature from the TPS65185 internal thermistor.
     * @return Temperature in °C, or -999 on error.
     */
    int readTemperature();

    // ---- EP_MODE Control ----

    /**
     * Set EP_MODE pin (gate driver mode selection).
     */
    void setMode(bool enable);

    // ---- Diagnostics ----

    /**
     * Print full power subsystem status to Serial.
     */
    void printStatus();

    /**
     * Scan I2C bus and print found devices to Serial.
     */
    void scanI2C();
    bool    pca9535SetPin(uint8_t pins);
    bool    pca9535ClearPin(uint8_t pins);
private:
    // I2C configuration
    int      _sda;
    int      _scl;
    uint32_t _i2cFreq;

    // PCA9535 output shadow registers
    uint8_t _port0Output;
    uint8_t _port1Output;

    // State
    bool _powerOn;

    // ---- Low-level I2C helpers ----
    bool    i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t value);
    int16_t i2cReadReg(uint8_t addr, uint8_t reg);

    // ---- PCA9535 helpers ----
    bool    pca9535Init();

    int16_t pca9535ReadInput(uint8_t port);

    // ---- TPS65185 helpers ----
    bool    tpsWriteReg(uint8_t reg, uint8_t value);
    int16_t tpsReadReg(uint8_t reg);
    bool    tpsCheckID();
    void    tpsClearInterrupts();
    bool    tpsConfigureSequencing();
    uint8_t tpsReadPowerGood();
};

#endif // EPD_POWER_H