// Choose your board.
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_H752
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include "EPD_Painter_Adafruit.h"

#define BOOT_BTN 0   // GPIO 0 — BOOT button, active LOW
#define NUM_PAGES 3

EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);

static int currentPage = 1;
static String serialBuffer = "";

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
struct Page2Layout;
static void showPage(int page);

// ---------------------------------------------------------------------------
// Serial command protocol for waveform calibration tool
// ---------------------------------------------------------------------------
static void parseWaveformRow(const char* data, uint8_t* dest, int len) {
    int idx = 0;
    const char* p = data;
    while (*p && idx < len) {
        while (*p == ' ') p++;
        if (*p >= '0' && *p <= '3') {
            dest[idx++] = *p - '0';
        }
        p++;
    }
}

static void parseWaveformCommand(String& cmd) {
    // Format: WF:FAST_LIGHTER:v,v,v,...|v,v,v,...|v,v,v,...
    int firstColon = cmd.indexOf(':', 3);
    if (firstColon < 0) { Serial.println("ERR:BAD_FORMAT"); return; }

    String name = cmd.substring(3, firstColon);
    String data = cmd.substring(firstColon + 1);

    // Split rows by '|'
    int pipe1 = data.indexOf('|');
    int pipe2 = data.indexOf('|', pipe1 + 1);
    if (pipe1 < 0 || pipe2 < 0) { Serial.println("ERR:BAD_ROWS"); return; }

    String row0 = data.substring(0, pipe1);
    String row1 = data.substring(pipe1 + 1, pipe2);
    String row2 = data.substring(pipe2 + 1);

    EPD_Painter::Waveforms& wf = epd.driver()._config.waveforms;

    if (name == "FAST_LIGHTER") {
        parseWaveformRow(row0.c_str(), wf.fast_lighter[0], 7);
        parseWaveformRow(row1.c_str(), wf.fast_lighter[1], 7);
        parseWaveformRow(row2.c_str(), wf.fast_lighter[2], 7);
    } else if (name == "FAST_DARKER") {
        parseWaveformRow(row0.c_str(), wf.fast_darker[0], 7);
        parseWaveformRow(row1.c_str(), wf.fast_darker[1], 7);
        parseWaveformRow(row2.c_str(), wf.fast_darker[2], 7);
    } else if (name == "NORMAL_LIGHTER") {
        parseWaveformRow(row0.c_str(), wf.normal_lighter[0], 13);
        parseWaveformRow(row1.c_str(), wf.normal_lighter[1], 13);
        parseWaveformRow(row2.c_str(), wf.normal_lighter[2], 13);
    } else if (name == "NORMAL_DARKER") {
        parseWaveformRow(row0.c_str(), wf.normal_darker[0], 13);
        parseWaveformRow(row1.c_str(), wf.normal_darker[1], 13);
        parseWaveformRow(row2.c_str(), wf.normal_darker[2], 13);
    } else if (name == "HIGH_LIGHTER") {
        parseWaveformRow(row0.c_str(), wf.high_lighter[0], 13);
        parseWaveformRow(row1.c_str(), wf.high_lighter[1], 13);
        parseWaveformRow(row2.c_str(), wf.high_lighter[2], 13);
    } else if (name == "HIGH_DARKER") {
        parseWaveformRow(row0.c_str(), wf.high_darker[0], 13);
        parseWaveformRow(row1.c_str(), wf.high_darker[1], 13);
        parseWaveformRow(row2.c_str(), wf.high_darker[2], 13);
    } else {
        Serial.println("ERR:UNKNOWN_WF");
        return;
    }
    Serial.println("OK");
}

static void sendWaveformRow(const uint8_t* row, int len) {
    for (int i = 0; i < len; i++) {
        if (i > 0) Serial.print(',');
        Serial.print(row[i]);
    }
}

static void sendWaveformTable(const char* name, const uint8_t row0[], const uint8_t row1[], const uint8_t row2[], int len) {
    Serial.print(name);
    Serial.print(':');
    sendWaveformRow(row0, len);
    Serial.print('|');
    sendWaveformRow(row1, len);
    Serial.print('|');
    sendWaveformRow(row2, len);
    Serial.println();
}

static void sendAllWaveforms() {
    EPD_Painter::Waveforms& wf = epd.driver()._config.waveforms;
    Serial.println("WF_START");
    sendWaveformTable("FAST_LIGHTER",   wf.fast_lighter[0],   wf.fast_lighter[1],   wf.fast_lighter[2],   7);
    sendWaveformTable("FAST_DARKER",    wf.fast_darker[0],    wf.fast_darker[1],    wf.fast_darker[2],    7);
    sendWaveformTable("NORMAL_LIGHTER", wf.normal_lighter[0], wf.normal_lighter[1], wf.normal_lighter[2], 13);
    sendWaveformTable("NORMAL_DARKER",  wf.normal_darker[0],  wf.normal_darker[1],  wf.normal_darker[2],  13);
    sendWaveformTable("HIGH_LIGHTER",   wf.high_lighter[0],   wf.high_lighter[1],   wf.high_lighter[2],   13);
    sendWaveformTable("HIGH_DARKER",    wf.high_darker[0],    wf.high_darker[1],    wf.high_darker[2],    13);
    Serial.println("WF_END");
}

static void processSerialCommand(String& cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.startsWith("WF:")) {
        parseWaveformCommand(cmd);
    } else if (cmd == "GET_WF") {
        sendAllWaveforms();
    } else if (cmd.startsWith("PAGE:")) {
        int page = cmd.substring(5).toInt();
        if (page >= 1 && page <= NUM_PAGES) {
            currentPage = page;
            showPage(currentPage);
        } else {
            Serial.println("ERR:BAD_PAGE");
        }
    } else if (cmd == "CLEAR") {
        epd.clear();
        epd.clear();
        epd.clear();
        Serial.println("OK");
    } else if (cmd == "UPDATE") {
        showPage(currentPage);
    } else if (cmd.startsWith("QUALITY:")) {
        String q = cmd.substring(8);
        q.trim();
        if (q == "FAST") {
            epd.setQuality(EPD_Painter::Quality::QUALITY_FAST);
            Serial.println("OK");
        } else if (q == "NORMAL") {
            epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
            Serial.println("OK");
        } else if (q == "HIGH") {
            epd.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
            Serial.println("OK");
        } else {
            Serial.println("ERR:BAD_QUALITY");
        }
    } else {
        Serial.print("ERR:UNKNOWN_CMD:");
        Serial.println(cmd);
    }
}

static void handleSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialBuffer.length() > 0) {
                processSerialCommand(serialBuffer);
                serialBuffer = "";
            }
        } else {
            if (serialBuffer.length() < 512) {
                serialBuffer += c;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// drawRadialGradient()
// ---------------------------------------------------------------------------
static void drawRadialGradient(EPD_PainterAdafruit& gfx,
                                int cx, int cy, int radius,
                                uint8_t centreGrey, uint8_t edgeGrey) {
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            float dist = sqrtf((float)(dx * dx + dy * dy));
            if (dist > radius) continue;
            float t = dist / radius;
            uint8_t grey = (uint8_t)(centreGrey + t * (edgeGrey - centreGrey));
            gfx.drawPixel(cx + dx, cy + dy, grey);
        }
    }
}

// ---------------------------------------------------------------------------
// drawPage1() — dither demo
// ---------------------------------------------------------------------------
static void drawPage1(EPD_PainterAdafruit& epd) {
    const int W      = epd.width();
    const int H      = epd.height();
    const int margin = 20;

    epd.fillScreen(255);

    epd.setTextColor(0);
    epd.setTextSize(3);
    epd.setCursor(margin, margin);
    epd.print("EPD Dither Demo  8bpp -> 2bpp (Floyd-Steinberg)");

    // Full-width horizontal gradient bar
    const int barX = margin;
    const int barY = 58;
    const int barW = W - margin * 2;
    const int barH = 55;

    for (int x = 0; x < barW; x++) {
        uint8_t grey = (uint8_t)((x * 255) / (barW - 1));
        epd.drawFastVLine(barX + x, barY, barH, grey);
    }

    epd.setTextSize(1);
    const int ticks[] = { 0, 64, 128, 192, 255 };
    for (int i = 0; i < 5; i++) {
        int tx = barX + (ticks[i] * (barW - 1)) / 255;
        epd.drawFastVLine(tx, barY + barH, 4, 0);
        char buf[5];
        snprintf(buf, sizeof(buf), "%d", ticks[i]);
        epd.setCursor(tx - (i == 4 ? 12 : 0), barY + barH + 5);
        epd.print(buf);
    }

    // 32-patch swatch grid
    const int cols      = 16;
    const int rows      = 2;
    const int swatchW   = (W - margin * 2) / cols;
    const int swatchH   = 80;
    const int swatchGap = 14;
    const int gridY     = 132;

    epd.setTextSize(1);
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            int     idx  = row * cols + col;
            uint8_t grey = (uint8_t)((idx * 255) / 31);
            int sx = margin + col * swatchW;
            int sy = gridY  + row * (swatchH + swatchGap + 4);
            epd.fillRect(sx, sy, swatchW - 2, swatchH, grey);
            epd.drawRect(sx, sy, swatchW - 2, swatchH, 0);
            char buf[5];
            snprintf(buf, sizeof(buf), "%3d", grey);
            epd.setTextColor(0);
            epd.setCursor(sx + 2, sy + swatchH + 2);
            epd.print(buf);
        }
    }

    // Radial gradient circles
    const int circleY = 390;
    const int radius  = 60;
    const int spacing = (W - margin * 2) / 4;

    drawRadialGradient(epd, margin + spacing * 0 + radius, circleY, radius, 0,   255);
    drawRadialGradient(epd, margin + spacing * 1 + radius, circleY, radius, 100, 255);
    drawRadialGradient(epd, margin + spacing * 2 + radius, circleY, radius, 255, 0);
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            float dist = sqrtf((float)(dx * dx + dy * dy));
            if (dist > radius) continue;
            float   v    = (sinf(dist * 0.25f) + 1.0f) * 0.5f;
            uint8_t grey = (uint8_t)(v * 255.0f);
            int cx = margin + spacing * 3 + radius;
            epd.drawPixel(cx + dx, circleY + dy, grey);
        }
    }

    epd.setTextSize(1);
    epd.setTextColor(0);
    const char* clabels[] = { "0->255", "100->255", "255->0", "sine" };
    for (int i = 0; i < 4; i++) {
        int cx = margin + spacing * i + radius;
        epd.setCursor(cx - 12, circleY + radius + 4);
        epd.print(clabels[i]);
    }

    epd.setTextSize(1);
    epd.setTextColor(0);
    epd.setCursor(margin, H - 14);
    epd.print("0=white  1=lt grey  2=dk grey  3=black   -- dither() converts 8bpp to 2bpp in-place before paint()   [BOOT = next page]");
}

// ---------------------------------------------------------------------------
// Page 2 layout constants (shared by drawPage2 and drawPage3)
// ---------------------------------------------------------------------------
struct Page2Layout {
    int W, H;
    int labelColW, sqY, footerH, sqH, halfH, gap, sqW;
    int s1x, s2x, midY;

    Page2Layout(int w, int h) : W(w), H(h) {
        labelColW = 68;
        sqY       = 50;
        footerH   = 30;
        sqH       = H - sqY - footerH;
        halfH     = sqH / 2;
        gap       = 30;
        sqW       = (W - labelColW - gap) / 2;
        s1x       = labelColW;
        s2x       = labelColW + sqW + gap;
        midY      = sqY + halfH;
    }
};

// ---------------------------------------------------------------------------
// drawPage2Dithered() — draw the dithered halves and borders (shared)
// ---------------------------------------------------------------------------
static void drawPage2Dithered(EPD_PainterAdafruit& epd, const Page2Layout& L) {
    // Left bracket annotations
    epd.setTextSize(1);
    epd.setTextColor(0);

    epd.drawFastVLine(L.labelColW - 4, L.sqY,   L.halfH, 0);
    epd.drawFastHLine(L.labelColW - 8, L.sqY,   4, 0);
    epd.drawFastHLine(L.labelColW - 8, L.midY - 1, 4, 0);

    epd.drawFastVLine(L.labelColW - 4, L.midY,  L.halfH, 0);
    epd.drawFastHLine(L.labelColW - 8, L.midY,  4, 0);
    epd.drawFastHLine(L.labelColW - 8, L.sqY + L.sqH - 1, 4, 0);

    epd.setCursor(2, L.sqY + L.halfH / 2 - 12);
    epd.print("Solid");
    epd.setCursor(2, L.sqY + L.halfH / 2 - 4);
    epd.print("grey");

    epd.setCursor(2, L.midY + L.halfH / 2 - 12);
    epd.print("Dithrd");
    epd.setCursor(2, L.midY + L.halfH / 2 - 4);
    epd.print("B+W");

    // --- Dithered halves ---
    // Square 1: 1-in-3 pattern (33% black)
    for (int y = L.midY; y < L.sqY + L.sqH; y++)
        for (int x = L.s1x; x < L.s1x + L.sqW; x++)
            epd.drawPixel(x, y, ((x + y * 2) % 3 == 0) ? 0 : 255);

    // Square 2: 2-in-3 pattern (67% black)
    for (int y = L.midY; y < L.sqY + L.sqH; y++)
        for (int x = L.s2x; x < L.s2x + L.sqW; x++)
            epd.drawPixel(x, y, ((x + y * 2) % 3 != 0) ? 0 : 255);

    // --- Borders ---
    epd.drawRect(L.s1x, L.sqY, L.sqW, L.sqH, 0);
    epd.drawFastHLine(L.s1x, L.midY, L.sqW, 0);
    epd.drawRect(L.s2x, L.sqY, L.sqW, L.sqH, 0);
    epd.drawFastHLine(L.s2x, L.midY, L.sqW, 0);

    // --- Dithered half labels ---
    epd.setTextSize(1);
    epd.setTextColor(0);
    epd.setCursor(L.s1x + 4, L.midY + 4);
    epd.print("33.3% black + 66.7% white pixels");
    epd.setCursor(L.s2x + 4, L.midY + 4);
    epd.print("66.7% black + 33.3% white pixels");
}

// ---------------------------------------------------------------------------
// drawPage2() — darker waveform grey calibration
// ---------------------------------------------------------------------------
static void drawPage2(EPD_PainterAdafruit& epd) {
    const int W = epd.width();
    const int H = epd.height();

    epd.fillScreen(255);

    // Title
    epd.setTextColor(0);
    epd.setTextSize(2);
    epd.setCursor(10, 8);
    epd.print("Darker Waveform Calibration");
    epd.setTextSize(1);
    epd.setCursor(10, 32);
    epd.print("Top half = solid waveform grey.  Bottom half = dithered B+W at matching density.  Match = correct waveform.   [BOOT = next]");

    Page2Layout L(W, H);
    drawPage2Dithered(epd, L);

    // --- Solid fills (grey) ---
    epd.setTextSize(2);
    epd.setTextColor(0);
    epd.setCursor(L.s1x + L.sqW / 2 - 60, L.sqY - 18);
    epd.print("lt-grey 0x01");
    epd.fillRect(L.s1x + 1, L.sqY + 1, L.sqW - 2, L.halfH - 1, 170);

    epd.setTextSize(2);
    epd.setTextColor(0);
    epd.setCursor(L.s2x + L.sqW / 2 - 60, L.sqY - 18);
    epd.print("dk-grey 0x02");
    epd.fillRect(L.s2x + 1, L.sqY + 1, L.sqW - 2, L.halfH - 1, 85);

    // Solid half labels
    epd.setTextSize(1);
    epd.setTextColor(0);
    epd.setCursor(L.s1x + 4, L.sqY + 4);
    epd.print("8bpp = 170  (solid waveform grey)");
    epd.setTextColor(255);
    epd.setCursor(L.s2x + 4, L.sqY + 4);
    epd.print("8bpp = 85   (solid waveform grey)");

    // Footer
    epd.setTextSize(1);
    epd.setTextColor(0);
    epd.setCursor(10, H - 18);
    epd.print("0=white  1=lt-grey(8bpp=170)  2=dk-grey(8bpp=85)  3=black(0)   |   pattern: (x + y*2) % 3 == 0  -> 1/3 black, diagonal offset");
}

// ---------------------------------------------------------------------------
// drawPage3() — lighter waveform calibration
//
// Same layout as Page 2, but the solid grey halves are WHITE.
// Show Page 2 first (greys on screen), then Page 3: the grey→white
// transition exercises the lighter waveform.  If calibrated correctly,
// no ghosting will remain in the solid areas.
// ---------------------------------------------------------------------------
static void drawPage3(EPD_PainterAdafruit& epd) {

    int trd = epd.height() / 3;
    int H = epd.height();
    int W = epd.width();
    epd.fillRect(0, 0, W, H, 1);
    epd.paint();
    epd.clear();

    delay(500);
    epd.fillRect(0, 0, W-200, trd, 1);
    epd.fillRect(0, trd, W-200, trd, 2);
    epd.fillRect(0, 2*trd, W-200, trd, 3);
    epd.paint();

    delay(500);
    epd.fillRect(W-400, 0, 400, epd.height(), 0);

    epd.setTextSize(3);
    epd.setCursor(10, 8);
    epd.print("Lighter Waveform Calibration");
    epd.paint();
}

// ---------------------------------------------------------------------------
// showPage() — draw, dither, paint the requested page number
// ---------------------------------------------------------------------------
static void showPage(int page) {
    epd.clear();
    epd.clear();
    epd.clear();

    switch (page) {
        case 1:
            Serial.println("Drawing page 1...");
            drawPage1(epd);
            break;
        case 2:
            Serial.println("Drawing page 2...");
            drawPage2(epd);
            break;
        case 3:
            Serial.println("Drawing page 3...");
            drawPage3(epd);
            return; // dither and paint are handled inside drawPage3() 
                    // to ensure the timing is correct for the lighter 
                    // waveform test — we don't want the white rectangles 
                    // to be composited into the framebuffer before paint() 
                    // reads it, or it would defeat the purpose of the test.
            break;
        default:
            Serial.println("ERR:BAD_PAGE");
            return;
    }
    Serial.println("Dithering...");
    epd.dither();
    Serial.println("Painting...");
    epd.paint();
    Serial.println("Done.");
}

void setup() {
    Serial.begin(115200);
    pinMode(BOOT_BTN, INPUT_PULLUP);

    if (!epd.begin()) {
        Serial.println("EPD init failed");
        while (1);
    }

#if defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_H752)
    #define BACKLIGHT_PIN 40
#elif defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS)
    #define BACKLIGHT_PIN 11
#endif
    #define TFT_BRIGHT_Bits 8
    #define TFT_BRIGHT_FREQ 5000

#if defined(BACKLIGHT_PIN)
    pinMode(BACKLIGHT_PIN, OUTPUT);
    //digitalWrite(BACKLIGHT_PIN, HIGH);
    ledcAttach(BACKLIGHT_PIN, TFT_BRIGHT_FREQ, TFT_BRIGHT_Bits);
    ledcWrite(BACKLIGHT_PIN, 0);
#endif

    Serial.println("EPD_Painter Dither Demo + Waveform Calibration");
    Serial.println("Serial commands: PAGE:n  CLEAR  UPDATE  QUALITY:x  WF:name:data  GET_WF");

    showPage(currentPage);
}

void loop() {
    // Handle serial commands from calibration tool
    handleSerial();

#ifdef EPD_PAINTER_PRESET_M5PAPER_S3
    // M5PaperS3 has no user-accessible BOOT button — advance automatically
    delay(4000);
    currentPage = (currentPage % NUM_PAGES) + 1;
    showPage(currentPage);
#else
    // Wait for BOOT button press (active LOW), with debounce
    if (digitalRead(BOOT_BTN) == LOW) {
        delay(50);
        if (digitalRead(BOOT_BTN) == LOW) {
            currentPage = (currentPage % NUM_PAGES) + 1;
            showPage(currentPage);
            while (digitalRead(BOOT_BTN) == LOW)
                delay(10);
            delay(50);
        }
    }
    delay(10); // avoid watchdog trigger
#endif
}
