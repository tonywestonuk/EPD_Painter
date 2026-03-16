// tps65185_vcom_cal.ino
//
// LilyGo T5 S3 GPS — TPS65185 VCOM calibration tool.
//
// Standalone example: all TPS65185 I2C access is implemented here.
// No VCOM-specific code is required in EPD_Painter core.
//
// Useful for restoring a correct VCOM value after another EPD driver has
// overwritten the TPS65185 NVM with an incorrect value.
//
// Performs 5 iterative measurements starting from 0 mV. Each pass uses the
// previous result as the VCOM pre-charge, converging on the true panel voltage.
// Hold BOOT (pin 0) for 5 seconds to write the final value to NVM.
//
// Board:   LilyGo T5 S3 GPS (ESP32-S3)
// Library: EPD_Painter + Adafruit GFX

#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#include <Arduino.h>
#include <esp_heap_caps.h>
#include "EPD_Painter_Adafruit.h"
#include "EPD_Painter_presets.h"

EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);

// ── TPS65185 I2C constants ───────────────────────────────────────────────────

static const uint8_t TPS_ADDR  = 0x68;

static const uint8_t TPS_VCOM1 = 0x03;
static const uint8_t TPS_VCOM2 = 0x04;
static const uint8_t TPS_INT1  = 0x07;

// VCOM2 bit masks  (datasheet s8.6.5, reset = 0x04)
static const uint8_t VCOM2_ACQ  = 0x80;
static const uint8_t VCOM2_PROG = 0x40;
static const uint8_t VCOM2_HIZ  = 0x20;
static const uint8_t VCOM2_AVG1 = 0x10;
static const uint8_t VCOM2_AVG0 = 0x08;
static const uint8_t VCOM2_MSB  = 0x01;

static const int     NUM_PASSES = 20;

// Null waveform packed buffer (2bpp, all zeros)
static uint8_t     *_nullbuf = nullptr;
static const size_t PACKED_SIZE = (960 * 540) / 4;

static TwoWire *_wire = nullptr;

// ── TPS65185 low-level I2C ───────────────────────────────────────────────────

static bool tpsRead(uint8_t reg, uint8_t &val) {
    _wire->beginTransmission(TPS_ADDR);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom(TPS_ADDR, (uint8_t)1) != 1) return false;
    val = _wire->read();
    return true;
}

static bool tpsWrite(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(TPS_ADDR);
    _wire->write(reg);
    _wire->write(val);
    return (_wire->endTransmission() == 0);
}

// ── VCOM helpers ─────────────────────────────────────────────────────────────

static int vcomReadMv() {
    uint8_t v1 = 0, v2 = 0;
    if (!tpsRead(TPS_VCOM1, v1) || !tpsRead(TPS_VCOM2, v2)) return 1;
    int raw = ((int)(v2 & VCOM2_MSB) << 8) | v1;
    return -(raw * 10);
}

static bool vcomWriteMv(int mv) {
    if (mv > 0)    mv = 0;
    if (mv < -5110) mv = -5110;
    int raw    = (-mv) / 10;
    uint8_t lo  = (uint8_t)(raw & 0xFF);
    uint8_t msb = (uint8_t)((raw >> 8) & 0x01);
    // AVG=3 (8x), preserve MSB only
    uint8_t v2 = (VCOM2_AVG1 | VCOM2_AVG0) | msb;
    return tpsWrite(TPS_VCOM1, lo) && tpsWrite(TPS_VCOM2, v2);
}

// Program current VCOM registers to NVM.
static bool vcomProgramNvm() {
    uint8_t v2 = 0;
    if (!tpsRead(TPS_VCOM2, v2)) return false;
    if (!tpsWrite(TPS_VCOM2, (v2 & (VCOM2_AVG1 | VCOM2_AVG0 | VCOM2_MSB)) | VCOM2_PROG)) return false;
    for (int i = 0; i < 200; i++) {
        delay(1);
        uint8_t poll = 0;
        if (!tpsRead(TPS_VCOM2, poll)) return true;  // NACK = chip entered STANDBY = done
        if (!(poll & VCOM2_PROG)) return true;
    }
    return true;
}

// Single kick-back measurement pass.
// Sets VCOM to startMv, drives the null waveform to pre-charge the panel,
// then measures with HiZ + ACQ (AVG=8x).
static bool vcomMeasureOnce(int startMv, int &resultMv) {
    if (!_nullbuf) return false;

    // Write startMv as the VCOM pre-charge voltage (AVG=8x, no control bits set).
    if (!vcomWriteMv(startMv)) return false;

    // Drive null waveform — panel charges to startMv.
    epd.driver().paintPacked(_nullbuf);
    delay(300);

    // Clear stale interrupt flags.
    uint8_t dummy = 0;
    tpsRead(TPS_INT1, dummy);

    // Read current v2 to preserve AVG bits.
    uint8_t v2 = 0;
    if (!tpsRead(TPS_VCOM2, v2)) return false;

    // Set HiZ — VCOM pin floats from startMv baseline.
    if (!tpsWrite(TPS_VCOM2, (v2 & (VCOM2_AVG1 | VCOM2_AVG0 | VCOM2_MSB)) | VCOM2_HIZ)) return false;

    // Drive null waveform — kick-back voltage appears on floating VCOM pin.
    epd.driver().paintPacked(_nullbuf);
    delay(20);

    // Start acquisition.
    if (!tpsWrite(TPS_VCOM2, (v2 & (VCOM2_AVG1 | VCOM2_AVG0 | VCOM2_MSB)) | VCOM2_HIZ | VCOM2_ACQ)) return false;

    // Poll until ACQ auto-clears (600 ms timeout).
    bool done = false;
    for (int i = 0; i < 1000; i++) {
        delay(1);
        uint8_t v2poll = 0;
        if (!tpsRead(TPS_VCOM2, v2poll)) break;
        if (!(v2poll & VCOM2_ACQ)) { done = true; break; }
    }

    // Reconnect VCOM amplifier.
    tpsRead(TPS_VCOM2, v2);
    tpsWrite(TPS_VCOM2, v2 & ~VCOM2_HIZ);

    if (!done) return false;
    resultMv = vcomReadMv();
    return true;
}

// ── Display ───────────────────────────────────────────────────────────────────

// ── Globals ───────────────────────────────────────────────────────────────────

static int  _storedMv                 = 0;
static int  _measurements[NUM_PASSES] = {0};
static int  _convergedMv              = 0;  // rounded to nearest 10 mV — actual NVM write value
static bool _calOk                    = false;

// ── Display ───────────────────────────────────────────────────────────────────

static void drawTitle() {
    epd.setTextColor(3);
    epd.setTextSize(4);
    epd.setCursor(10, 10);
    epd.print("TPS65185 VCOM Calibration");
    epd.drawLine(10, 54, 950, 54, 3);
}

static void showCountdown(int n) {
    epd.fillScreen(0);
    drawTitle();
    epd.setTextColor(3);
    epd.setTextSize(4);
    epd.setCursor(10, 100);
    epd.print("Running calibration in...");
    int16_t x1, y1; uint16_t tw, th;
    String ns = String(n);
    epd.getTextBounds(ns, 0, 0, &x1, &y1, &tw, &th);
    epd.setCursor((epd.width() - tw) / 2, 200);
    epd.print(ns);
    epd.paint();
}

static void showMeasuringPass(int pass) {
    epd.fillScreen(0);
    drawTitle();
    epd.setTextColor(3);
    epd.setTextSize(4);
    epd.setCursor(10, 180);
    epd.print("Measuring... pass " + String(pass) + " of " + String(NUM_PASSES));
    epd.paint();
}

// results: array of NUM_PASSES values (0 = not yet measured).
// storedMv: value loaded from NVM at startup.
// countdown: -1=prompt, 0=writing, 1-5=countdown, -2=saved.
static void showResults(int storedMv, const int *results, int numDone,
                        bool calOk, int countdown = -1) {
    epd.fillScreen(0);
    drawTitle();

    epd.setTextColor(3);

    // NVM stored value
    epd.setTextSize(3);
    epd.setCursor(10, 62);
    epd.print("NVM (stored): " + String(storedMv) + " mV");
    epd.drawLine(10, 94, 950, 94, 3);

    // Two columns of 10 passes at size 3 (24px tall, 28px row spacing).
    // Left column: passes 1-10   Right column: passes 11-20
    static const int COL_X[2]  = { 10, 490 };
    static const int ROW_Y0    = 102;
    static const int ROW_STEP  = 28;

    epd.setTextSize(3);
    for (int i = 0; i < numDone; i++) {
        int col = i / 10;
        int row = i % 10;
        epd.setCursor(COL_X[col], ROW_Y0 + row * ROW_STEP);
        epd.print("P" + String(i + 1) + ": " + String(results[i]) + " mV");
    }

    // Footer — sits below the two columns (10 rows * 28px = 280px + ROW_Y0 = 382)
    epd.drawLine(10, 390, 950, 390, 3);
    epd.setTextSize(3);
    epd.setCursor(10, 400);
    if (!calOk) {
        epd.print("Measurement failed. Reset to retry.");
    } else if (numDone < NUM_PASSES) {
        // still measuring — no footer prompt yet
    } else if (countdown == -2) {
        epd.print("Saved " + String(_convergedMv) + " mV to NVM.  Reset to run again.");
    } else if (countdown == 0) {
        epd.print("Writing " + String(_convergedMv) + " mV to NVM...");
    } else if (countdown >= 1) {
        epd.print("Saving " + String(_convergedMv) + " mV in " + String(countdown) + "...");
    } else {
        epd.print("Hold BOOT 5s to save " + String(_convergedMv) + " mV to NVM.");
    }

    epd.paint();
}

// ── Globals ───────────────────────────────────────────────────────────────────


// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    pinMode(BOOT_PIN, INPUT_PULLUP);

    if (!epd.begin()) {
        Serial.println("FATAL: epd.begin() failed.");
        while (1) delay(1000);
    }

    _wire = epd.getConfig().i2c.wire;

    _nullbuf = (uint8_t*)heap_caps_calloc(1, PACKED_SIZE, MALLOC_CAP_SPIRAM);
    if (!_nullbuf) {
        Serial.println("FATAL: failed to allocate null waveform buffer.");
        while (1) delay(1000);
    }

    epd.clear();

    _storedMv = vcomReadMv();
    Serial.printf("NVM VCOM: %d mV\n", _storedMv);

    // Countdown
    for (int i = 5; i >= 1; i--) {
        showCountdown(i);
        delay(1000);
    }

    // Binary chop over [-5110, 0] mV.
    // Pre-charge at midpoint, measure, then compare result to mid:
    //   result > mid → true value is above mid → raise low
    //   result < mid → true value is below mid → lower high
    // 20 passes reduces a 5110 mV range to <0.01 mV — converges in ~10 passes.
    int low = -5110, high = 0;
    _calOk = true;
    for (int pass = 0; pass < NUM_PASSES; pass++) {
        int mid = (low + high) / 2;
        showMeasuringPass(pass + 1);
        int result = 0;
        if (!vcomMeasureOnce(mid, result)) {
            _calOk = false;
            Serial.printf("Pass %d failed.\n", pass + 1);
            break;
        }
        _measurements[pass] = result;
        if (result > mid) low  = mid;
        else              high = mid;
        Serial.printf("Pass %d: pre=%d  result=%d  range=[%d, %d]\n",
                      pass + 1, mid, result, low, high);
        showResults(_storedMv, _measurements, pass + 1, _calOk);
    }

    heap_caps_free(_nullbuf);
    _nullbuf = nullptr;

    // Write the converged midpoint into the registers ready for NVM programming.
    // (low+high)/2 is more accurate than the last individual measurement.
    if (_calOk) {
        int converged    = (low + high) / 2;
        _convergedMv     = -((-converged) / 10 * 10);  // truncate to 10 mV resolution
        vcomWriteMv(_convergedMv);
        Serial.printf("Converged: %d mV  →  will write: %d mV\n", converged, _convergedMv);
    }

    // Ensure final screen shows the BOOT prompt
    showResults(_storedMv, _measurements, _calOk ? NUM_PASSES : 0, _calOk);
}

void loop() {
    if (!_calOk) return;

    static int           lastCountdown = -1;
    static unsigned long holdStart     = 0;
    static bool          saved         = false;

    if (saved) return;

    bool pressed = (digitalRead(BOOT_PIN) == LOW);

    if (!pressed) {
        holdStart = 0;
        if (lastCountdown != -1) {
            lastCountdown = -1;
            showResults(_storedMv, _measurements, NUM_PASSES, _calOk, -1);
        }
        return;
    }

    if (holdStart == 0) holdStart = millis();

    unsigned long held      = millis() - holdStart;
    int           countdown = 5 - (int)(held / 1000);

    if (held >= 5000) {
        saved = true;
        showResults(_storedMv, _measurements, NUM_PASSES, _calOk, 0);
        bool ok = vcomProgramNvm();
        Serial.printf("NVM write: %s  read back: %d mV\n", ok ? "OK" : "FAILED", vcomReadMv());
        showResults(_storedMv, _measurements, NUM_PASSES, _calOk, -2);
        return;
    }

    if (countdown != lastCountdown) {
        lastCountdown = countdown;
        showResults(_storedMv, _measurements, NUM_PASSES, _calOk, countdown);
    }
}
