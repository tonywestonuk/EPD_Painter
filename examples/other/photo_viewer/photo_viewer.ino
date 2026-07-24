// ============================================================================
// EPD_Painter PHOTO VIEWER — JPEGs from an SD card, in 16 native greys.
//
// Put photographs in a /photos folder on an SD card and this shows them,
// one per tap. They are rendered in the panel's 16 grey levels using the
// board's own scanner-tuned trains and, crucially, its MEASURED level
// curve — see "why the curve matters" below.
//
//   Left half of the screen  ... previous photo
//   Right half               ... next photo
//   Serial: n / p / space    ... next / previous / next
//   Serial: i                ... what the current photo is doing
//
// WHY THE CURVE MATTERS
// A panel's 16 greys are NOT evenly spaced. On the LilyGo T5 S3 the
// measured curve runs
//   255 253 231 219 205 190 172 135 126 115 61 55 47 35 21 0
// so levels 0 and 1 differ by 2 units while levels 9 and 10 differ by 54.
// A dither that assumes even steps therefore pushes error into levels that
// cannot express it, and the result bands badly. Quantising against the
// measured curve instead is what makes photographs look right.
//
// The curve is produced by the tuneup example and stored alongside the
// tuned trains; this sketch just loads it. Without it (an untuned board)
// the viewer falls back to assuming even levels, which still works but
// will band on smooth gradients.
//
// Requires: Adafruit GFX, JPEGDEC, gt911-arduino (touch).
// ============================================================================

// Choose your board (or leave all commented for auto-probe).
//#define EPD_PAINTER_PRESET_M5PAPER_S3
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <SD_MMC.h>
#include <LittleFS.h>
#include <JPEGDEC.h>
#include <esp_heap_caps.h>
#include "EPD_Painter_Adafruit.h"
#include "EPD_Painter_tuned.h"
#include <gt911_lite.h>

static EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);

static const char *PHOTO_DIR = "/photos";
static const char *TUNED_PATH = "/epd_g16_tuned.bin";

// Decode no larger than this before scaling down — keeps the intermediate
// image inside PSRAM whatever the camera produced.
static const int MAX_DECODE_W = 1920;

static int  panelW = 0, panelH = 0;
static bool useSdmmc = false;

// ============================================================================
// SD pins per board (the map proven in the bad_apple example)
// ============================================================================
struct SdPins {
  int  cs, sclk, mosi, miso;
  bool sdmmc;          // true: native 1-bit SD protocol, false: SPI mode
  int  other_cs;       // another device on a shared bus to deselect (-1: none)
};

static bool sdPinsForBoard(SdPins &p) {
  const auto &cfg = epd.getConfig();
  if (cfg.i2c.sda == 39 && cfg.i2c.scl == 40) {        // LilyGo T5 S3 GPS / Pro
    p = { 12, 14, 13, 21, false, 46 };                 // SPI; deselect LoRa (46)
    return true;
  }
  if (cfg.i2c.sda == 41 && cfg.i2c.scl == 42) {        // M5PaperS3
    p = { 47, 39, 38, 40, true, -1 };
    return true;
  }
  if (cfg.i2c.sda == 18 && cfg.i2c.scl == 17) {        // LilyGo EPD47 H716
    p = { 42, 11, 15, 16, false, -1 };
    return true;
  }
  if (cfg.i2c.sda == 6 && cfg.i2c.scl == 5) {          // LilyGo T5 S3 H752
    p = { 42, 11, 15, 16, false, -1 };
    return true;
  }
  return false;
}

static File openPath(const char *path) {
  return useSdmmc ? SD_MMC.open(path, FILE_READ) : SD.open(path, FILE_READ);
}

// ============================================================================
// Touch — same GT911 pattern as the other examples.
//
// NOTE the poll interval below: the driver only trusts a press it sees on
// two frames within 40 ms, deliberately, because EPD refresh transients
// produce isolated frames. Poll any slower and every real tap is thrown
// away as a phantom.
// ============================================================================
static GT911_Lite touch;
static uint16_t tpXmax = 960, tpYmax = 540;
static bool tpSwap = false, touchOk = false, touchWasDown = false;

static void touchInit() {
  TwoWire *bus = epd.getConfig().i2c.wire;
  if (!bus) return;
  bus->setTimeOut(50);
  touch.begin(bus);
  for (uint8_t addr : { (uint8_t)0x5D, (uint8_t)0x14 }) {
    bus->beginTransmission(addr);
    bus->write(0x81); bus->write(0x46);
    if (bus->endTransmission(false) != 0) continue;
    if (bus->requestFrom(addr, (uint8_t)4) != 4) continue;
    uint16_t xm = bus->read(); xm |= bus->read() << 8;
    uint16_t ym = bus->read(); ym |= bus->read() << 8;
    if (xm == 0 || ym == 0 || xm == 0xFFFF) break;
    tpXmax = xm; tpYmax = ym;
    tpSwap = (xm < ym);
    touchOk = true;
    Serial.printf("[viewer] touch 0x%02x, range %ux%u%s\n", addr, xm, ym,
                  tpSwap ? " (swapped)" : "");
    return;
  }
  Serial.println("[viewer] no touch controller - serial control only");
}

static bool touchTapped(int &px, int &py) {
  if (!touchOk) return false;
  touch.read();
  const bool down = touch.isTouched;
  const bool tap = down && !touchWasDown;
  touchWasDown = down;
  if (!tap) return false;
  const uint16_t rx = touch.x, ry = touch.y;
  if (tpSwap) {
    px = (int)ry * panelW / tpYmax;
    py = (int)(tpXmax - rx) * panelH / tpXmax;
  } else {
    px = (int)rx * panelW / tpXmax;
    py = (int)ry * panelH / tpYmax;
  }
  return true;
}

// ============================================================================
// JPEG -> greyscale
// ============================================================================
static JPEGDEC   jpeg;
static uint8_t  *fileBuf = nullptr;    // the compressed file
static uint8_t  *grayBuf = nullptr;    // decoded greyscale
static int       grayW = 0, grayH = 0;

static int jpegDrawCb(JPEGDRAW *d) {
  const uint8_t *src = (const uint8_t *)d->pPixels;
  for (int y = 0; y < d->iHeight; y++) {
    const int dy = d->y + y;
    if (dy < 0 || dy >= grayH) continue;
    uint8_t *row = grayBuf + (size_t)dy * grayW;
    for (int x = 0; x < d->iWidth; x++) {
      const int dx = d->x + x;
      if (dx >= 0 && dx < grayW) row[dx] = src[(size_t)y * d->iWidth + x];
    }
  }
  return 1;
}

// ============================================================================
// Floyd-Steinberg onto the panel's real levels
//
// The quantiser searches the MEASURED luminance of each level rather than
// assuming an even ramp, and diffuses the true residual. That is what
// keeps gradients smooth on a panel whose levels bunch together at one
// end. src is 8bpp grey (255 = white) at src_w x src_h; it is scaled to
// fit the canvas, centred, and dithered in one pass.
// ============================================================================
static void renderDithered(const uint8_t *src, int sw, int sh) {
  const uint8_t *lum = EPD_PainterTuned::levelLuminance();
  uint8_t even[16];
  if (!lum) {
    for (int i = 0; i < 16; i++) even[i] = (uint8_t)(255 - i * 255 / 15);
    lum = even;
  }

  uint8_t *fb = epd.getBuffer();
  memset(fb, 0, (size_t)panelW * panelH);          // white surround

  // fit, preserving aspect
  const float s = min((float)panelW / sw, (float)panelH / sh);
  const int dw = max(1, (int)(sw * s)), dh = max(1, (int)(sh * s));
  const int ox = (panelW - dw) / 2, oy = (panelH - dh) / 2;

  static int16_t errCur[2048], errNext[2048];
  memset(errCur, 0, sizeof(int16_t) * dw);
  for (int y = 0; y < dh; y++) {
    memset(errNext, 0, sizeof(int16_t) * dw);
    const uint8_t *srow = src + (size_t)((int)(y / s)) * sw;
    uint8_t *drow = fb + (size_t)(oy + y) * panelW + ox;
    for (int x = 0; x < dw; x++) {
      int v = (int)srow[(int)(x / s)] + errCur[x];
      if (v < 0) v = 0; else if (v > 255) v = 255;
      int lv = 0, best = 1000;
      for (int i = 0; i < 16; i++) {
        const int d = abs((int)lum[i] - v);
        if (d < best) { best = d; lv = i; }
      }
      const int err = v - (int)lum[lv];
      drow[x] = (uint8_t)lv;
      if (x + 1 < dw) errCur[x + 1]  += (int16_t)(err * 7 / 16);
      if (x > 0)      errNext[x - 1] += (int16_t)(err * 3 / 16);
      errNext[x]                     += (int16_t)(err * 5 / 16);
      if (x + 1 < dw) errNext[x + 1] += (int16_t)(err * 1 / 16);
    }
    memcpy(errCur, errNext, sizeof(int16_t) * dw);
  }
}

// ============================================================================
// Photo list
// ============================================================================
static const int MAX_PHOTOS = 200;
static String photos[MAX_PHOTOS];
static int    nPhotos = 0, current = 0;

static bool isJpeg(const char *name) {
  const char *dot = strrchr(name, '.');
  if (!dot) return false;
  return !strcasecmp(dot, ".jpg") || !strcasecmp(dot, ".jpeg");
}

static void scanPhotos() {
  nPhotos = 0;
  File dir = openPath(PHOTO_DIR);
  if (!dir || !dir.isDirectory()) {
    Serial.printf("[viewer] no %s folder on the card\n", PHOTO_DIR);
    return;
  }
  for (File f = dir.openNextFile(); f && nPhotos < MAX_PHOTOS; f = dir.openNextFile()) {
    if (!f.isDirectory() && isJpeg(f.name())) {
      String p = String(PHOTO_DIR) + "/" + f.name();
      photos[nPhotos++] = p;
    }
    f.close();
  }
  dir.close();
  // simple alphabetical order so the sequence is predictable
  for (int i = 0; i < nPhotos; i++)
    for (int j = i + 1; j < nPhotos; j++)
      if (photos[j] < photos[i]) { String t = photos[i]; photos[i] = photos[j]; photos[j] = t; }
  Serial.printf("[viewer] %d photo(s) in %s\n", nPhotos, PHOTO_DIR);
}

// ============================================================================
static void showMessage(const char *l1, const char *l2) {
  epd.fillScreen(0);
  epd.setTextColor(15);
  epd.setTextSize(3);
  epd.setCursor(40, panelH / 2 - 40);
  epd.print(l1);
  if (l2) {
    epd.setTextSize(2);
    epd.setCursor(40, panelH / 2 + 10);
    epd.print(l2);
  }
  epd.paint();
  while (!epd.driver().paintIdle()) delay(10);
}

static bool showPhoto(int idx) {
  if (idx < 0 || idx >= nPhotos) return false;
  const uint32_t t0 = millis();
  Serial.printf("[viewer] %d/%d  %s\n", idx + 1, nPhotos, photos[idx].c_str());

  File f = openPath(photos[idx].c_str());
  if (!f) { Serial.println("[viewer]   open failed"); return false; }
  const size_t len = f.size();
  if (fileBuf) heap_caps_free(fileBuf);
  fileBuf = (uint8_t *)heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
  if (!fileBuf) { f.close(); Serial.println("[viewer]   no PSRAM for file"); return false; }
  f.read(fileBuf, len);
  f.close();

  if (!jpeg.openRAM(fileBuf, (int)len, jpegDrawCb)) {
    Serial.printf("[viewer]   not a readable JPEG (%d)\n", jpeg.getLastError());
    return false;
  }
  jpeg.setPixelType(EIGHT_BIT_GRAYSCALE);

  // Decode at the largest built-in scale that stays within our buffer —
  // a phone photo is far larger than the panel, and decoding it full size
  // would need more PSRAM than the board has.
  int w = jpeg.getWidth(), h = jpeg.getHeight(), opt = 0;
  static const int opts[] = { 0, JPEG_SCALE_HALF, JPEG_SCALE_QUARTER, JPEG_SCALE_EIGHTH };
  for (int shift = 0; shift <= 3; shift++) {
    if ((w >> shift) <= MAX_DECODE_W || shift == 3) {
      opt = opts[shift]; w >>= shift; h >>= shift;
      break;
    }
  }
  grayW = w; grayH = h;
  if (grayBuf) heap_caps_free(grayBuf);
  grayBuf = (uint8_t *)heap_caps_malloc((size_t)grayW * grayH, MALLOC_CAP_SPIRAM);
  if (!grayBuf) { jpeg.close(); Serial.println("[viewer]   no PSRAM for image"); return false; }
  memset(grayBuf, 255, (size_t)grayW * grayH);

  const bool ok = jpeg.decode(0, 0, opt);
  jpeg.close();
  if (!ok) { Serial.printf("[viewer]   decode failed (%d)\n", jpeg.getLastError()); return false; }

  renderDithered(grayBuf, grayW, grayH);
  epd.paint();
  while (!epd.driver().paintIdle()) delay(10);
  Serial.printf("[viewer]   %dx%d -> panel in %lu ms\n", grayW, grayH, millis() - t0);
  return true;
}

static void step(int dir) {
  if (!nPhotos) return;
  int tries = nPhotos;
  while (tries--) {
    current = (current + dir + nPhotos) % nPhotos;
    if (showPhoto(current)) return;      // skip anything that will not decode
  }
}

// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Rig-friendly: a reset should not power the board off mid-slideshow.
  epd.setAutoShutdown(false);

  if (!epd.begin()) { Serial.println("[viewer] display begin() failed"); for (;;) delay(1000); }
  panelW = epd.width();
  panelH = epd.height();

  // The panel still holds the previous image after a reset while the
  // driver assumes it is blank, so start from a real clear.
  epd.clear();
  while (!epd.driver().paintIdle()) delay(10);

  // Tuned trains + the measured level curve, if this board has been
  // through the tuneup example. Safe to call on an untuned board.
  bool tuned = false;
  if (LittleFS.begin(true)) {
    File f = LittleFS.open(TUNED_PATH, "r");
    if (f) {
      uint8_t buf[EPD_PainterTuned::BLOB_SIZE];
      const size_t n = f.read(buf, sizeof(buf));
      f.close();
      tuned = EPD_PainterTuned::install(epd.driver(), buf, n);
    }
  }
  const uint8_t *curve = EPD_PainterTuned::levelLuminance();
  Serial.printf("[viewer] trains: %s, level curve: %s\n",
                tuned ? "flash-tuned" : "preset",
                curve ? "measured" : "assumed even");
  if (curve) {
    Serial.print("[viewer] curve:");
    for (int i = 0; i < 16; i++) Serial.printf(" %d", curve[i]);
    Serial.println();
  }

  epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
  if (!epd.driver().setGreyLevels(16)) {
    Serial.println("[viewer] setGreyLevels(16) refused");
    for (;;) delay(1000);
  }
  touchInit();

  SdPins sd;
  if (!sdPinsForBoard(sd)) {
    showMessage("NO SD PIN MAP", "This board is not in sdPinsForBoard()");
    for (;;) delay(1000);
  }
  pinMode(sd.cs, OUTPUT);
  digitalWrite(sd.cs, HIGH);
  if (sd.other_cs >= 0) { pinMode(sd.other_cs, OUTPUT); digitalWrite(sd.other_cs, HIGH); }

  useSdmmc = sd.sdmmc;
  bool sdOk;
  if (sd.sdmmc) {
    SD_MMC.setPins(sd.sclk, sd.mosi, sd.miso);
    sdOk = SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT);
  } else {
    SPI.begin(sd.sclk, sd.miso, sd.mosi, sd.cs);
    sdOk = SD.begin(sd.cs, SPI, 25000000);
  }
  if (!sdOk) {
    showMessage("NO SD CARD", "Insert a card with a /photos folder, then reset");
    for (;;) delay(1000);
  }

  scanPhotos();
  if (!nPhotos) {
    showMessage("NO PHOTOS", "Put .jpg files in /photos on the SD card");
    for (;;) delay(1000);
  }
  Serial.println("[viewer] tap right = next, left = previous (serial: n p space i)");
  showPhoto(current);
}

void loop() {
  int px, py;
  if (touchTapped(px, py)) {
    step(px >= panelW / 2 ? +1 : -1);
    return;
  }
  if (Serial.available()) {
    const int c = Serial.read();
    if (c == 'n' || c == ' ') step(+1);
    else if (c == 'p')        step(-1);
    else if (c == 'i')
      Serial.printf("[viewer] %d/%d %s\n", current + 1, nPhotos,
                    nPhotos ? photos[current].c_str() : "-");
    return;
  }
  delay(8);      // fast enough for the touch driver's confirmation window
}
