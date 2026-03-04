#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
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
for (int i=0; i<5; i++){
    epd.clear();
}

epd.fillRect(0, 100, 512, 200, 3);

epd.paint();

epd.fillScreen(0);


epd.paint();

}

void loop() {

}