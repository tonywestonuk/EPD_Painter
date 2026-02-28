#include <Arduino.h>
#include "EPD_Painter.h"
#include "EPD_Painter_devices.h"

// Pick one.
//EPD_Painter epd(EPD_PAPER_DEVICE::M5STACK_PAPERS3);
EPD_Painter epd(EPD_PAPER_DEVICE::LILYGO_T5_S3_GPS);

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
