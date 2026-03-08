// gps_raw_serial.ino
//
// Minimal GPS test for LilyGo T5 S3 GPS board.
// No libraries — just raw Arduino Wire + HardwareSerial.
//
// The GPS module (u-blox M10Q, 9600 baud) is behind a PCA9555 IO expander.
// Port 0 bit 0 must be driven HIGH to enable the GPS power rail.
// Everything received is echoed straight to the USB Serial console.

#include <Wire.h>

// ── Pin / address constants ───────────────────────────────────────────────────
#define I2C_SDA            39
#define I2C_SCL            40
#define IO_EXPANDER_ADDR   0x20

// PCA9555 registers
#define REG_OUTPUT_P0      0x02
#define REG_CONFIG_P0      0x06   // 0=output, 1=input

#define GPS_RX             44
#define GPS_TX             43
#define GPS_BAUD           9600

// ── Globals ───────────────────────────────────────────────────────────────────
HardwareSerial gpsSerial(1);   // UART1

// ── IO expander ───────────────────────────────────────────────────────────────
static bool io_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(IO_EXPANDER_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static void gps_power_on() {
    io_write(REG_CONFIG_P0, 0xFE);   // port 0 bit 0 = output
    io_write(REG_OUTPUT_P0, 0xFF);   // port 0 bit 0 HIGH = GPS power on
}

// ── Arduino entry points ──────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    delay(200);

    Serial.println("=== GPS raw serial test ===");

    Wire.begin(I2C_SDA, I2C_SCL);
    gps_power_on();

    delay(2000);   // wait for module to boot

    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

    Serial.println("Streaming NMEA — waiting for data...");
    Serial.println("-------------------------------------------");
}

static uint32_t last_heartbeat = 0;
static uint32_t bytes_received = 0;

void loop() {
    while (gpsSerial.available()) {
        Serial.write(gpsSerial.read());
        bytes_received++;
        last_heartbeat = millis();
    }

    if (millis() - last_heartbeat >= 5000) {
        last_heartbeat = millis();
        Serial.printf("[%lus] waiting — %lu bytes received so far\n",
                      millis() / 1000, bytes_received);
    }
}
