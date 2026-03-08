/**
 * @file    lilygo_shutdown.ino
 * @brief   Power off a LILYGO T5-S3 GPS using the XPowers library (BQ25896)
 *
 * Demonstrates how to share the I2C bus with EPD_Painter's driver.
 * EPD_Painter takes ownership of the Wire instance during initialisation,
 * so a reference is retrieved via epd.getConfig() and passed directly to
 * PPM.init() — no separate Wire.begin() call required.
 *
 * Press the 'Boot' button (GPIO 0) to trigger a clean shutdown via PPM.shutdown().
 *
 * Dependencies:
 *   - EPD_Painter   (with LILYGO_T5_S3_GPS preset)
 *   - XPowersLib    (XPOWERS_CHIP_BQ25896)
 */
 

#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS

#include "EPD_Painter_Adafruit.h"
#include "EPD_Painter_presets.h"

#define XPOWERS_CHIP_BQ25896
#include <XPowersLib.h>


EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);
XPowersPPM PPM;


void setup() {
  Serial.begin(115200);
  delay(1000);

epd.begin();

const auto& cfg = epd.getConfig();

Serial.println(cfg.i2c.wire==nullptr);

// This config can be found in EPD_Painter_presets
bool result = PPM.init(*cfg.i2c.wire, cfg.i2c.sda, cfg.i2c.scl, BQ25896_SLAVE_ADDRESS);
 if (!result) {
     Serial.println("PPM is not online...");
    while(true);
 }

  pinMode(0, INPUT);

  epd.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
  epd.clear();
  epd.clear();

  epd.setTextColor(3);
  epd.setTextSize(5);

  epd.println("Press 'Boot' button.");
  epd.println("to turn off");
  epd.paint();
}

void loop() {

 if (digitalRead(0) == 0) {
   epd.fillScreen(0);
   epd.setCursor(2, 2);
   epd.print("Disconnecting battery in 3 seconds....");
   epd.paint();
   epd.paint();

   delay(3000);
   epd.clear();
   PPM.shutdown();

   Serial.println("Battery now disconnected. If you're seeing this, its because its connected to USB.");

 }

}
