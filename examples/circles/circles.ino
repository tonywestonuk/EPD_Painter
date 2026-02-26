#include <Arduino.h>
#include "EPD_Painter.h"
#include "EPD_Painter_devices.h"

EPD_Painter epd(EPD_PAPER_DEVICE::M5PAPER_S3);

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
