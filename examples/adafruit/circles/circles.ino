// Choose your board.
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_M5PAPER_S3


#include <Arduino.h>
#define EPD_PAINTER_ENABLE_AUTO_SHUTDOWN 1
// Optional library flag used by the examples.
// Set to 1 to enable the reset-twice auto-shutdown helper and shutdown image flow.
// Leave it undefined or set it to 0 in normal projects unless you explicitly want this behaviour.

#include "EPD_Painter_presets.h"
#include "EPD_Painter_Adafruit.h"


EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);

void setup() {
if (!epd.begin()) {
    Serial.println("EPD init failed");
    while (1);
  }
  epd.clear();
  epd.clear();

}

void loop() {
  epd.fillCircle(random(epd.width()), random(epd.height()), random(300), random(4) );
  epd.paint();

}
