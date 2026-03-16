// lora_receiver.ino
//
// LoRa packet receiver for LilyGo T5 S3 GPS.
// Listens for incoming LoRa packets and scrolls received text onto the
// e-paper display using the Adafruit GFX API.
//
// Hardware: SX1262 LoRa module (onboard), shared power rail with GPS.
// Power is enabled via the PCA9555 / XL9555 IO expander (IO0 bit 0 HIGH).
//
// Required libraries:
//   - RadioLib  (https://github.com/jgromes/RadioLib)
//   - Adafruit GFX Library
//   - EPD_Painter

#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "EPD_Painter_Adafruit.h"
#include "EPD_Painter_presets.h"

// ── SPI / LoRa pin definitions ────────────────────────────────────────────────
#define LORA_SCLK   14
#define LORA_MISO   21
#define LORA_MOSI   13
#define LORA_CS     46
#define LORA_IRQ    10
#define LORA_RST     1
#define LORA_BUSY   47

#define SD_CS       12   // pull HIGH before SPI init to keep SD deselected

// ── Radio settings — must match the transmitter ───────────────────────────────
#define LORA_FREQUENCY      868.0   // MHz
#define LORA_BANDWIDTH      125.0   // kHz
#define LORA_SF              12     // spreading factor
#define LORA_CR               8     // coding rate denominator (5 → 4/5)

// ── IO expander (PCA9555 / XL9555 at 0x20) ───────────────────────────────────
// Port 0 bit 0 HIGH = LoRa + GPS power rail enabled
#define IO_ADDR         0x20
#define REG_CONFIG_P0   0x06
#define REG_OUTPUT_P0   0x02

// ── Terminal display layout ───────────────────────────────────────────────────
#define TERM_FONT_SIZE   2
#define CHAR_H           8    // base font height at size 1
#define LINE_H          (CHAR_H * TERM_FONT_SIZE + 4)
#define MARGIN_X         6
#define MARGIN_Y         6
#define MAX_LINES       60    // ring-buffer depth

// ── Globals ───────────────────────────────────────────────────────────────────
EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);

SPIClass loraSPI(HSPI);
SX1262   radio = new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_BUSY, loraSPI);

volatile bool packetReady = false;

static String termLines[MAX_LINES];
static int    lineHead    = 0;   // index of oldest line
static int    lineCount   = 0;
static int    visibleRows = 0;

// ── ISR ───────────────────────────────────────────────────────────────────────
void IRAM_ATTR onPacket() {
    packetReady = true;
}

// ── IO expander helpers ───────────────────────────────────────────────────────
static void io_write(TwoWire *wire, uint8_t reg, uint8_t val) {
    wire->beginTransmission(IO_ADDR);
    wire->write(reg);
    wire->write(val);
    wire->endTransmission();
}

static void lora_power_on(TwoWire *wire) {
    io_write(wire, REG_CONFIG_P0, 0xFE);  // bit 0 = output
    io_write(wire, REG_OUTPUT_P0, 0xFF);  // bit 0 HIGH → power on
}

// ── Terminal helpers ──────────────────────────────────────────────────────────
static void pushLine(const String &text) {
    if (lineCount < MAX_LINES) {
        termLines[(lineHead + lineCount) % MAX_LINES] = text;
        lineCount++;
    } else {
        termLines[lineHead] = text;
        lineHead = (lineHead + 1) % MAX_LINES;
    }
}

static void renderTerminal() {
    epd.fillScreen(0);
    epd.setTextColor(3);
    epd.setTextSize(TERM_FONT_SIZE);
    epd.setTextWrap(false);

    int start = (lineCount > visibleRows) ? (lineCount - visibleRows) : 0;
    for (int i = 0; i < visibleRows && (start + i) < lineCount; i++) {
        int idx = (lineHead + start + i) % MAX_LINES;
        epd.setCursor(MARGIN_X, MARGIN_Y + i * LINE_H);
        epd.print(termLines[idx]);
    }

    epd.paint();
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // Deselect all SPI slaves before starting SPI
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);

    // Initialise display (also brings up I2C)
    if (!epd.begin()) {
        Serial.println("EPD init failed");
        while (1);
    }
    epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
    epd.clear();

    visibleRows = (epd.height() - MARGIN_Y * 2) / LINE_H;
    if (visibleRows > MAX_LINES) visibleRows = MAX_LINES;

    pushLine("LoRa Receiver");
    pushLine("Powering radio...");
    renderTerminal();

    // Power on LoRa (shared rail with GPS) via IO expander
    TwoWire *wire = epd.getConfig().i2c.wire;
    lora_power_on(wire);
    delay(1500);  // wait for power rail to stabilise

    // Initialise SPI for LoRa
    loraSPI.begin(LORA_SCLK, LORA_MISO, LORA_MOSI, LORA_CS);

    // Initialise SX1262
    pushLine("Init SX1262...");
    renderTerminal();

    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        pushLine("Radio fail: " + String(state));
        renderTerminal();
        while (1) delay(1000);
    }

    // Board-specific hardware requirements
    radio.setTCXO(2.4);          // TCXO reference voltage for this module
    radio.setDio2AsRfSwitch();   // DIO2 drives the onboard RF switch

    // Apply channel settings
    radio.setFrequency(LORA_FREQUENCY);
    radio.setBandwidth(LORA_BANDWIDTH);
    radio.setSpreadingFactor(LORA_SF);
    radio.setCodingRate(LORA_CR);
    radio.setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE);
    radio.setCRC(2);

    // Register interrupt and start listening
    radio.setPacketReceivedAction(onPacket);
    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        pushLine("Listen fail: " + String(state));
        renderTerminal();
        while (1) delay(1000);
    }

    pushLine("Listening " + String(LORA_FREQUENCY, 1) + " MHz  SF" +
             String(LORA_SF) + "  BW" + String((int)LORA_BANDWIDTH) + " kHz");
    pushLine("Waiting for packets...");
    renderTerminal();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    if (!packetReady) return;
    packetReady = false;

    String payload;
    int state = radio.readData(payload);

    if (state == RADIOLIB_ERR_NONE) {
        pushLine(payload);
        pushLine("  RSSI " + String(radio.getRSSI(), 1) +
                 " dBm  SNR " + String(radio.getSNR(), 1) + " dB");
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        pushLine("[CRC error]");
    } else {
        pushLine("[RX error: " + String(state) + "]");
    }

    renderTerminal();

    // Re-arm for next packet
    radio.startReceive();
}
