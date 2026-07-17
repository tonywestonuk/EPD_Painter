// wavecal — waveform-calibration servant for the scanner rig.
//
// Sits on serial waiting for commands; renders grey patch charts so a flatbed
// scanner (panel face-down) can measure the real reflectance of each grey
// level. Waveform rows can be replaced over serial and repainted WITHOUT
// reflashing — paint() reads _config.waveforms live on every pass.
//
// Protocol (newline-terminated ASCII, responses end with a line "DONE", "OK",
// "ERR ..." or a data line):
//   T                     -> "TEMP <c>"  panel temp from TPS65185 thermistor
//   Q F|N|H               -> set quality FAST/NORMAL/HIGH
//   W <TBL> <row> <v...>  -> replace one waveform row. TBL: FL FD NL ND HL HD
//                            (fast/normal/high, lighter/darker). 7 values for
//                            F*, 13 for the rest. Values 0..3.
//   G                     -> dump all waveform tables
//   P                     -> hard clear, draw patch chart, paint, "DONE T=<c>"
//   U                     -> paint all-white (round trip; exercises lighter)
//   K                     -> paint all-black (panel exercise / cold wake-up)
//   C                     -> hard clear
//   B                     -> clearBuffers (reset DC-balance baseline)
//   X                     -> TPS65185 diag: reg dump + traced conversion
//   J                     -> BQ27220 fuel-gauge temperature (independent
//                            cross-check; the battery chills with the panel)

#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS

#include <Arduino.h>
#include "EPD_Painter_Adafruit.h"

EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);

// Patch chart geometry (landscape 960x540, 8bpp values: low 2 bits are the
// level — 0=white 1=light grey 2=dark grey 3=black).
static const int FID = 40;                    // fiducial square size
static const int FID_INSET = 20;
static const int PATCH_W = 160, PATCH_H = 280, PATCH_Y = 130;
static const int PATCH_X[4] = { 100, 300, 500, 700 };
static const uint8_t PATCH_VAL[4] = { 3, 2, 1, 0 };  // black,dk,lt,white L->R

static void drawChart() {
  epd.fillScreen(0);  // white
  // Corner fiducials
  epd.fillRect(FID_INSET, FID_INSET, FID, FID, 3);
  epd.fillRect(960 - FID_INSET - FID, FID_INSET, FID, FID, 3);
  epd.fillRect(FID_INSET, 540 - FID_INSET - FID, FID, FID, 3);
  epd.fillRect(960 - FID_INSET - FID, 540 - FID_INSET - FID, FID, FID, 3);
  for (int i = 0; i < 4; i++)
    epd.fillRect(PATCH_X[i], PATCH_Y, PATCH_W, PATCH_H, PATCH_VAL[i]);
}

static uint8_t* wfRow(const char* tbl, int row, int& len) {
  EPD_Painter::Waveforms& w = epd.driver()._config.waveforms;
  len = 13;
  if (!strcmp(tbl, "FL")) { len = 7; return w.fast_lighter[row]; }
  if (!strcmp(tbl, "FD")) { len = 7; return w.fast_darker[row]; }
  if (!strcmp(tbl, "NL")) return w.normal_lighter[row];
  if (!strcmp(tbl, "ND")) return w.normal_darker[row];
  if (!strcmp(tbl, "HL")) return w.high_lighter[row];
  if (!strcmp(tbl, "HD")) return w.high_darker[row];
  return nullptr;
}

static void dumpTable(const char* name, const uint8_t* t, int rows, int len) {
  for (int r = 0; r < rows; r++) {
    Serial.printf("%s %d:", name, r);
    for (int i = 0; i < len; i++) Serial.printf(" %d", t[r * len + i]);
    Serial.println();
  }
}

static bool i2cRegRead(uint8_t addr, uint8_t reg, uint8_t* buf, int n) {
  TwoWire* w = epd.driver()._config.i2c.wire;
  w->beginTransmission(addr);
  w->write(reg);
  if (w->endTransmission(false) != 0) return false;
  if (w->requestFrom((int)addr, n) != n) return false;
  for (int i = 0; i < n; i++) buf[i] = w->read();
  return true;
}

static bool i2cRegWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  TwoWire* w = epd.driver()._config.i2c.wire;
  w->beginTransmission(addr);
  w->write(reg);
  w->write(val);
  return w->endTransmission() == 0;
}

static void tpsDiag() {
  const uint8_t TPS = 0x68, PCA = 0x20;
  // Wake the TPS via PCA9555 port-1 WAKEUP (bit5) and remember prior state.
  uint8_t out1 = 0;
  if (!i2cRegRead(PCA, 0x03, &out1, 1)) { Serial.println("ERR pca read"); return; }
  bool wasAwake = out1 & 0x20;
  if (!wasAwake) {
    i2cRegWrite(PCA, 0x03, out1 | 0x20);
    delay(5);
  }
  Serial.print("REGS");
  for (uint8_t r = 0; r <= 0x10; r++) {
    uint8_t v = 0xEE;
    i2cRegRead(TPS, r, &v, 1);
    Serial.printf(" %02X=%02X", r, v);
  }
  Serial.println();
  uint8_t t1 = 0;
  i2cRegRead(TPS, 0x0D, &t1, 1);
  Serial.printf("TMST1 before=0x%02X\n", t1);
  i2cRegWrite(TPS, 0x0D, t1 | 0x80);  // READ_THERM
  for (int i = 0; i < 20; i++) {
    uint8_t v = 0, tv = 0;
    i2cRegRead(TPS, 0x0D, &v, 1);
    i2cRegRead(TPS, 0x00, &tv, 1);
    Serial.printf("  t+%dms TMST1=0x%02X TMST_VALUE=%d\n", i, v, (int8_t)tv);
    if (v & 0x20) break;
    delay(1);
  }
  if (!wasAwake) i2cRegWrite(PCA, 0x03, out1);
  Serial.println("DONE");
}

static void bqTemp() {
  uint8_t b[2];
  if (i2cRegRead(0x55, 0x06, b, 2)) {  // Temperature(), 0.1 K units, LE
    int raw = b[0] | (b[1] << 8);
    Serial.printf("BQTEMP %.1f C (raw %d)\n", raw / 10.0 - 273.15, raw);
  } else {
    Serial.println("ERR bq27220 read");
  }
}

static char line[160];
static int linePos = 0;

static void handle(char* cmd) {
  // tokenize
  char* tok[20];
  int ntok = 0;
  for (char* p = strtok(cmd, " \r\n"); p && ntok < 20; p = strtok(nullptr, " \r\n"))
    tok[ntok++] = p;
  if (!ntok) return;

  if (!strcmp(tok[0], "T")) {
    Serial.printf("TEMP %d\n", epd.readPanelTemperatureC());

  } else if (!strcmp(tok[0], "Q") && ntok == 2) {
    switch (tok[1][0]) {
      case 'F': epd.setQuality(EPD_Painter::Quality::QUALITY_FAST);   break;
      case 'N': epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL); break;
      case 'H': epd.setQuality(EPD_Painter::Quality::QUALITY_HIGH);   break;
      default: Serial.println("ERR quality"); return;
    }
    Serial.println("OK");

  } else if (!strcmp(tok[0], "W") && ntok >= 3) {
    int row = atoi(tok[2]);
    int len;
    uint8_t* dst = (row >= 0 && row < 3) ? wfRow(tok[1], row, len) : nullptr;
    if (!dst) { Serial.println("ERR table/row"); return; }
    if (ntok != 3 + len) { Serial.printf("ERR need %d values\n", len); return; }
    for (int i = 0; i < len; i++) {
      int v = atoi(tok[3 + i]);
      if (v < 0 || v > 3) { Serial.println("ERR value"); return; }
      dst[i] = (uint8_t)v;
    }
    Serial.println("OK");

  } else if (!strcmp(tok[0], "G")) {
    EPD_Painter::Waveforms& w = epd.driver()._config.waveforms;
    dumpTable("FL", &w.fast_lighter[0][0], 3, 7);
    dumpTable("FD", &w.fast_darker[0][0], 3, 7);
    dumpTable("NL", &w.normal_lighter[0][0], 3, 13);
    dumpTable("ND", &w.normal_darker[0][0], 3, 13);
    dumpTable("HL", &w.high_lighter[0][0], 3, 13);
    dumpTable("HD", &w.high_darker[0][0], 3, 13);
    Serial.println("DONE");

  } else if (!strcmp(tok[0], "P")) {
    epd.clear();  // hard clear -> known white state
    drawChart();
    epd.paint();
    delay(2500);  // paint() returns before the sweeps finish — let them
    Serial.printf("DONE T=%d\n", epd.readPanelTemperatureC());

  } else if (!strcmp(tok[0], "U")) {
    epd.fillScreen(0);
    epd.paint();
    delay(2500);
    Serial.printf("DONE T=%d\n", epd.readPanelTemperatureC());

  } else if (!strcmp(tok[0], "K")) {
    epd.fillScreen(3);
    epd.paint();
    delay(2500);
    Serial.printf("DONE T=%d\n", epd.readPanelTemperatureC());

  } else if (!strcmp(tok[0], "C")) {
    epd.clear();
    Serial.println("DONE");

  } else if (!strcmp(tok[0], "B")) {
    epd.driver().clearBuffers();
    Serial.println("OK");

  } else if (!strcmp(tok[0], "X")) {
    tpsDiag();

  } else if (!strcmp(tok[0], "J")) {
    bqTemp();

  } else {
    Serial.println("ERR unknown");
  }
}

void setup() {
  Serial.begin(115200);
  // Auto-shutdown would power-cycle between commands; harmless, keep default.
  if (!epd.begin()) {
    while (1) { Serial.println("ERR begin failed"); delay(2000); }
  }
  epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
  delay(100);
  Serial.printf("READY T=%d\n", epd.readPanelTemperatureC());
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line[linePos] = 0;
      linePos = 0;
      handle(line);
    } else if (linePos < (int)sizeof(line) - 1) {
      line[linePos++] = c;
    }
  }
  delay(5);
}
