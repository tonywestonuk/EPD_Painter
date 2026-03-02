// Choose your board.
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#define EPD_PAINTER_PRESET_M5PAPER_S3


#include <Arduino.h>
#include "EPD_Painter.h"
#include "EPD_Painter_presets.h"


EPD_Painter epd(EPD_PAINTER_PRESET);

void setup() {

Serial.begin(115200);
delay(1000);


if (!epd.begin()) {
    Serial.println("EPD init failed");
    while (1);
  }
  
  epd.paint();
  
}

void loop() {
 // epd.paint();

}
