#include "EPD_Painter.h"
#include <Fonts/FreeMonoBoldOblique12pt7b.h>

EPD_Painter epd;

void setup() {
  Serial.begin(115200);
  delay(1000);
  if (!epd.begin()) {
    Serial.println("EPD init failed");
    while (1);
  }

  // Set the font
  epd.setTextSize(5);
  epd.setFont(&FreeMonoBoldOblique12pt7b);
  epd.setCursor(0, 100);
  epd.setTextColor(3);

  for (int i=0; i<2; i++){
    epd.clear();
  }
    // Write hello world.
  epd.print("Hello World");


  //epd.fillCircle(200, 200,50,3);
  //epd.paint();

  // for (int i=100; i<400; i+=2){
  //   epd.drawLine(200, i, 300, i, 3);
  // }

  // for (int i=101; i<300; i+=2){
  //   epd.drawLine(400, i, 500, i, 3);
  // }

}
// int x[] = {200, 400, 100};
// int y[] = {200, 100, 350};
// int dx[] = {33, -25, 18};
// int dy[] = {30, 22, -35};

void loop() {
  epd.fillCircle(random(epd.width()), random(epd.height()), random(200), random(4));
//  long start = esp_timer_get_time();
  epd.paint();
  //Serial.println(esp_timer_get_time()-start);
}