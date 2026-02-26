/*
 * =============================================================================
 * EPDPower.cpp - T5 E-Paper S3 Pro EPD Power Control
 * =============================================================================
 *
 * Power-on sequence (as per EPDIY / TPS65185 datasheet):
 *   1. Set PCA9535 pin directions (outputs for control, inputs for status)
 *   2. Assert WAKEUP high on PCA9535 → TPS65185 enters STANDBY
 *   3. Configure TPS65185 registers (VCOM, power sequencing, etc.) via I2C
 *   4. Assert PWRUP high on PCA9535 → TPS65185 enters ACTIVE, rails come up
 *   5. Wait for PWRGOOD signal on PCA9535 input
 *   6. Enable VCOM by setting VCOM_CTRL
 *
 * Power-off sequence:
 *   1. Disable VCOM (clear VCOM_CTRL)
 *   2. De-assert PWRUP → TPS65185 sequences rails down
 *   3. Wait for power-down complete
 *   4. De-assert WAKEUP → TPS65185 enters SLEEP
 */

#include "EPD_painter_power.h"

// =============================================================================
// Constructor
// =============================================================================

EPD_painter_power::EPD_painter_power(int sda, int scl, uint32_t i2cFreq)
    : _sda(sda)
    , _scl(scl)
    , _i2cFreq(i2cFreq)
    , _port0Output(0x00)
    , _port1Output(0x00)
    , _powerOn(false)
{
}

// =============================================================================
// Low-level I2C helpers
// =============================================================================

bool EPD_painter_power::i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        Serial.printf("[I2C] Write error: addr=0x%02X reg=0x%02X err=%d\n", addr, reg, err);
        return false;
    }
    return true;
}

int16_t EPD_painter_power::i2cReadReg(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    uint8_t err = Wire.endTransmission(false);  // repeated start
    if (err != 0) {
        Serial.printf("[I2C] Read setup error: addr=0x%02X reg=0x%02X err=%d\n", addr, reg, err);
        return -1;
    }
    Wire.requestFrom(addr, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    Serial.printf("[I2C] No data from addr=0x%02X reg=0x%02X\n", addr, reg);
    return -1;
}

// =============================================================================
// PCA9535 GPIO Expander
// =============================================================================

bool EPD_painter_power::pca9535Init() {
    Serial.println("[PCA9535] Initializing...");

    // Port 0: all inputs
    if (!i2cWriteReg(PCA9535_ADDR, PCA9535_REG_CONFIG_0, PORT0_DIR_MASK)) return false;

    // Port 1: PWRGOOD, INT, BUTTON as inputs; rest as outputs
    if (!i2cWriteReg(PCA9535_ADDR, PCA9535_REG_CONFIG_1, PORT1_DIR_MASK)) return false;

    // Initialize all outputs LOW
    _port0Output = 0x00;
    _port1Output = 0x00;
    if (!i2cWriteReg(PCA9535_ADDR, PCA9535_REG_OUTPUT_0, _port0Output)) return false;
    if (!i2cWriteReg(PCA9535_ADDR, PCA9535_REG_OUTPUT_1, _port1Output)) return false;

    Serial.println("[PCA9535] Init OK");
    return true;
}

bool EPD_painter_power::pca9535SetPin(uint8_t pins) {
    _port1Output |= pins;
    return i2cWriteReg(PCA9535_ADDR, PCA9535_REG_OUTPUT_1, _port1Output);
}

bool EPD_painter_power::pca9535ClearPin(uint8_t pins) {
    _port1Output &= ~pins;
    return i2cWriteReg(PCA9535_ADDR, PCA9535_REG_OUTPUT_1, _port1Output);
}

int16_t EPD_painter_power::pca9535ReadInput(uint8_t port) {
    uint8_t reg = (port == 0) ? PCA9535_REG_INPUT_0 : PCA9535_REG_INPUT_1;
    return i2cReadReg(PCA9535_ADDR, reg);
}

// =============================================================================
// TPS65185 PMIC
// =============================================================================

bool EPD_painter_power::tpsWriteReg(uint8_t reg, uint8_t value) {
    return i2cWriteReg(TPS65185_ADDR, reg, value);
}

int16_t EPD_painter_power::tpsReadReg(uint8_t reg) {
    return i2cReadReg(TPS65185_ADDR, reg);
}

bool EPD_painter_power::tpsCheckID() {
    int16_t rev = tpsReadReg(TPS_REG_REVID);
    if (rev < 0) {
        Serial.println("[TPS65185] ERROR: Cannot communicate with TPS65185!");
        return false;
    }
    Serial.printf("[TPS65185] Revision ID: 0x%02X\n", (uint8_t)rev);
    return true;
}

void EPD_painter_power::tpsClearInterrupts() {
    int16_t int1 = tpsReadReg(TPS_REG_INT1);
    int16_t int2 = tpsReadReg(TPS_REG_INT2);
    if (int1 >= 0 && int2 >= 0) {
        if (int1 || int2) {
            Serial.printf("[TPS65185] Cleared interrupts: INT1=0x%02X INT2=0x%02X\n",
                          (uint8_t)int1, (uint8_t)int2);
        }
    }
}

bool EPD_painter_power::tpsConfigureSequencing() {
    Serial.println("[TPS65185] Configuring power sequencing...");

    // EPDIY default power-up sequence
    if (!tpsWriteReg(TPS_REG_UPSEQ0, 0xE4)) return false;
    if (!tpsWriteReg(TPS_REG_UPSEQ1, 0x55)) return false;

    // Power-down sequence
    if (!tpsWriteReg(TPS_REG_DWNSEQ0, 0x1E)) return false;
    if (!tpsWriteReg(TPS_REG_DWNSEQ1, 0xE0)) return false;

    Serial.println("[TPS65185] Sequencing configured OK");
    return true;
}

uint8_t EPD_painter_power::tpsReadPowerGood() {
    int16_t pg = tpsReadReg(TPS_REG_PG);
    return (pg >= 0) ? (uint8_t)pg : 0;
}

// =============================================================================
// Public API
// =============================================================================

bool EPD_painter_power::begin(bool scanBus) {
    Serial.println("\n====================================");
    Serial.println(" EPD Power Init (EPDIY-style)");
    Serial.println("====================================\n");

    // Initialize I2C bus
    Wire.begin(_sda, _scl, _i2cFreq);
    Serial.printf("[I2C] Initialized on SDA=%d, SCL=%d @ %d Hz\n", _sda, _scl, _i2cFreq);

    if (scanBus) {
        scanI2C();
    }

    // 1. Initialize PCA9535 GPIO expander
    if (!pca9535Init()) {
        Serial.println("[EPD] FATAL: PCA9535 init failed!");
        return false;
    }

    // 2. All control pins LOW
    _port1Output = 0x00;
    i2cWriteReg(PCA9535_ADDR, PCA9535_REG_OUTPUT_1, _port1Output);
    delay(10);

    // 3. Assert WAKEUP → TPS65185 SLEEP → STANDBY
    Serial.println("[EPD] Asserting WAKEUP → TPS65185 STANDBY");
    pca9535SetPin(CFG_PIN_WAKEUP);
    delay(10);

    // 4. Verify TPS65185 communication
    if (!tpsCheckID()) {
        Serial.println("[EPD] FATAL: TPS65185 not responding!");
        return false;
    }

    // 5. Clear stale interrupts
    tpsClearInterrupts();

    // 6. Configure power sequencing
    tpsConfigureSequencing();

    // 7. Read temperature (informational)
    int temp = readTemperature();
    if (temp != -999) {
        Serial.printf("[EPD] Panel temperature: %d°C\n", temp);
    }

    Serial.println("[EPD] Power subsystem initialized OK\n");
    return true;
}

bool EPD_painter_power::powerOn() {
    if (_powerOn) {
        Serial.println("[EPD] Power already ON");
        return true;
    }

    Serial.println("[EPD] === POWER ON ===");

    // Ensure WAKEUP is asserted
    pca9535SetPin(CFG_PIN_WAKEUP);
    delay(5);

    // Clear pending interrupts
    tpsClearInterrupts();

    // Assert PWRUP → TPS65185 STANDBY → ACTIVE
    Serial.println("[EPD] Asserting PWRUP → TPS65185 ACTIVE");
    pca9535SetPin(CFG_PIN_PWRUP);

    // Wait for PWRGOOD
    Serial.print("[EPD] Waiting for PWRGOOD...");
    int tries = 0;
    const int maxTries = 500;
    while (tries < maxTries) {
        int16_t port1Input = pca9535ReadInput(1);
        if (port1Input < 0) {
            Serial.println(" I2C ERROR!");
            return false;
        }
        if (port1Input & CFG_PIN_PWRGOOD) {
            Serial.printf(" OK (after %d ms)\n", tries);
            break;
        }
        delay(1);
        tries++;
    }
    if (tries >= maxTries) {
        Serial.println(" TIMEOUT! Power good not received.");
        uint8_t pg = tpsReadPowerGood();
        Serial.printf("[EPD] TPS65185 PG register: 0x%02X\n", pg);
        tpsClearInterrupts();
        pca9535ClearPin(CFG_PIN_PWRUP);
        return false;
    }

    delay(5);  // Rails stabilize

    // Enable VCOM
    Serial.println("[EPD] Enabling VCOM");
    pca9535SetPin(CFG_PIN_VCOM_CTRL);
    delay(5);

    // Enable EP_OE (source driver output enable)
    Serial.println("[EPD] Enabling EP_OE (source driver)");
    pca9535SetPin(CFG_PIN_EP_OE);

    uint8_t pg = tpsReadPowerGood();
    Serial.printf("[EPD] Power Good Status: 0x%02X\n", pg);

    _powerOn = true;
    Serial.println("[EPD] === POWER ON COMPLETE ===\n");
    return true;
}

bool EPD_painter_power::powerOff() {
    if (!_powerOn) {
        Serial.println("[EPD] Power already OFF");
        return true;
    }

    Serial.println("[EPD] === POWER OFF ===");

    // Disable EP_OE first (stop driving panel)
    pca9535ClearPin(CFG_PIN_EP_OE);
    delay(1);

    // Disable VCOM
    Serial.println("[EPD] Disabling VCOM");
    pca9535ClearPin(CFG_PIN_VCOM_CTRL);
    delay(1);

    // De-assert PWRUP → power-down sequence
    Serial.println("[EPD] De-asserting PWRUP → power-down sequence");
    pca9535ClearPin(CFG_PIN_PWRUP);

    // Wait for PWRGOOD to go low
    Serial.print("[EPD] Waiting for power-down...");
    int tries = 0;
    while (tries < 500) {
        int16_t port1Input = pca9535ReadInput(1);
        if (port1Input < 0) break;
        if (!(port1Input & CFG_PIN_PWRGOOD)) {
            Serial.printf(" OK (after %d ms)\n", tries);
            break;
        }
        delay(1);
        tries++;
    }
    if (tries >= 500) {
        Serial.println(" TIMEOUT (forced)");
    }

    // TPS65185 generates a late interrupt — wait and clear
    delay(500);
    pca9535ReadInput(0);
    pca9535ReadInput(1);
    tpsClearInterrupts();

    _powerOn = false;
    Serial.println("[EPD] === POWER OFF COMPLETE ===\n");
    return true;
}

void EPD_painter_power::sleep() {
    Serial.println("[EPD] Entering deep power sleep...");

    if (_powerOn) {
        powerOff();
    }

    // De-assert WAKEUP → TPS65185 SLEEP (lowest power)
    pca9535ClearPin(CFG_PIN_WAKEUP);

    _port1Output = 0x00;
    i2cWriteReg(PCA9535_ADDR, PCA9535_REG_OUTPUT_1, _port1Output);

    Serial.println("[EPD] TPS65185 in SLEEP mode");
}

bool EPD_painter_power::wake() {
    pca9535SetPin(CFG_PIN_WAKEUP);
    delay(10);
    return tpsCheckID();
}

// =============================================================================
// VCOM Control
// =============================================================================

bool EPD_painter_power::setVCOM(uint16_t millivolts) {
    uint16_t vcomReg = millivolts / 10;
    if (vcomReg > 511) vcomReg = 511;

    uint8_t vcom1 = vcomReg & 0xFF;

    int16_t vcom2Current = tpsReadReg(TPS_REG_VCOM2);
    if (vcom2Current < 0) vcom2Current = 0;
    uint8_t vcom2 = (vcom2Current & 0xFE) | ((vcomReg >> 8) & 0x01);

    Serial.printf("[TPS65185] Setting VCOM: -%d.%02dV (reg=%d)\n",
                  millivolts / 1000, (millivolts % 1000) / 10, vcomReg);

    if (!tpsWriteReg(TPS_REG_VCOM1, vcom1)) return false;
    if (!tpsWriteReg(TPS_REG_VCOM2, vcom2)) return false;
    return true;
}

int EPD_painter_power::getVCOM() {
    int16_t v1 = tpsReadReg(TPS_REG_VCOM1);
    int16_t v2 = tpsReadReg(TPS_REG_VCOM2);
    if (v1 < 0 || v2 < 0) return -1;

    uint16_t vcomReg = ((v2 & 0x01) << 8) | (v1 & 0xFF);
    return vcomReg * 10;
}

// =============================================================================
// Temperature
// =============================================================================

int EPD_painter_power::readTemperature() {
    int16_t tmst2 = tpsReadReg(TPS_REG_TMST2);
    if (tmst2 < 0) return -999;

    tpsWriteReg(TPS_REG_TMST2, tmst2 | 0x80);
    delay(10);

    int16_t temp = tpsReadReg(TPS_REG_TMST_VALUE);
    if (temp < 0) return -999;

    return (int8_t)temp;  // Signed 8-bit
}

// =============================================================================
// EP_MODE
// =============================================================================

void EPD_painter_power::setMode(bool enable) {
    if (enable) {
        pca9535SetPin(CFG_PIN_EP_MODE);
    } else {
        pca9535ClearPin(CFG_PIN_EP_MODE);
    }
}

// =============================================================================
// Diagnostics
// =============================================================================

void EPD_painter_power::scanI2C() {
    Serial.println("[I2C] Scanning...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  Found device at 0x%02X", addr);
            if (addr == PCA9535_ADDR)   Serial.print(" (PCA9535)");
            if (addr == TPS65185_ADDR)  Serial.print(" (TPS65185)");
            if (addr == 0x5D)           Serial.print(" (GT911 Touch)");
            if (addr == 0x51)           Serial.print(" (PCF85063 RTC)");
            if (addr == 0x6B)           Serial.print(" (BQ25896 Charger)");
            if (addr == 0x55)           Serial.print(" (BQ27220 Fuel Gauge)");
            Serial.println();
        }
    }
    Serial.println();
}

void EPD_painter_power::printStatus() {
    Serial.println("\n--- EPD Power Status ---");

    int16_t in0 = pca9535ReadInput(0);
    int16_t in1 = pca9535ReadInput(1);
    Serial.printf("PCA9535 Port0 Input:  0x%02X\n", (in0 >= 0) ? (uint8_t)in0 : 0);
    Serial.printf("PCA9535 Port1 Input:  0x%02X\n", (in1 >= 0) ? (uint8_t)in1 : 0);
    Serial.printf("PCA9535 Port1 Output: 0x%02X\n", _port1Output);

    if (in1 >= 0) {
        Serial.printf("  PWRGOOD:  %s\n", (in1 & CFG_PIN_PWRGOOD) ? "HIGH (good)" : "LOW");
        Serial.printf("  INT:      %s\n", (in1 & CFG_PIN_INT)     ? "HIGH (no int)" : "LOW (interrupt!)");
        Serial.printf("  BUTTON:   %s\n", (in1 & CFG_PIN_BUTTON)  ? "HIGH" : "LOW (pressed)");
    }
    Serial.printf("  WAKEUP:   %s\n", (_port1Output & CFG_PIN_WAKEUP)    ? "HIGH" : "LOW");
    Serial.printf("  PWRUP:    %s\n", (_port1Output & CFG_PIN_PWRUP)     ? "HIGH" : "LOW");
    Serial.printf("  VCOM_CTRL:%s\n", (_port1Output & CFG_PIN_VCOM_CTRL) ? "HIGH" : "LOW");
    Serial.printf("  EP_OE:    %s\n", (_port1Output & CFG_PIN_EP_OE)     ? "HIGH" : "LOW");
    Serial.printf("  EP_MODE:  %s\n", (_port1Output & CFG_PIN_EP_MODE)   ? "HIGH" : "LOW");

    if (_port1Output & CFG_PIN_WAKEUP) {
        int16_t rev    = tpsReadReg(TPS_REG_REVID);
        int16_t enable = tpsReadReg(TPS_REG_ENABLE);
        int16_t pg     = tpsReadReg(TPS_REG_PG);
        int16_t vadj   = tpsReadReg(TPS_REG_VADJ);

        Serial.printf("TPS65185 REVID:  0x%02X\n",  (rev >= 0)    ? (uint8_t)rev : 0);
        Serial.printf("TPS65185 ENABLE: 0x%02X\n",   (enable >= 0) ? (uint8_t)enable : 0);
        Serial.printf("TPS65185 PG:     0x%02X\n",   (pg >= 0)     ? (uint8_t)pg : 0);
        Serial.printf("TPS65185 VADJ:   0x%02X\n",   (vadj >= 0)   ? (uint8_t)vadj : 0);

        int vcom = getVCOM();
        if (vcom >= 0) {
            Serial.printf("TPS65185 VCOM:   -%d.%02dV\n", vcom / 1000, (vcom % 1000) / 10);
        }

        int temp = readTemperature();
        if (temp != -999) {
            Serial.printf("TPS65185 Temp:   %d°C\n", temp);
        }
    }

    Serial.println("------------------------\n");
}