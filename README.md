# EPD_Painter
Fast m5paperS3 epaper driver

Currently just a Proof of concept Arduino project.

Download, open in arduino IDE.

You need to add the Adafruit GFX Library, to get things going.  It might not be the fastest out there, but its rock solid.

This should work straight away on an M5PaperS3,  for other epaper devices that use an ESP32-S3, update the pins in EPD_Painter.h

The main differences between this and, I think, all other ESP32-S3 epaper drivers:
* Uses ESP32-S3 Vector code to calculate 64 pixels at a time.
* Uses a different way of using DMA.  The standard ESP32-S3 I80 drivers (using esp_lcd_panel_io_i80_config_t and esp_lcd_panel_io_tx_color), are terrible, and the biggest bottleneck for high performance.  After battling hard trying to get it to perform, I found this adafruit blog.  https://blog.adafruit.com/2022/06/21/esp32uesday-more-s3-lcd-peripheral-hacking-with-code/  -  I ripped it out, and used the LCD_CAM driver instead.  Far far faster.
* Just 4 colours (2 bit).  I don't see this changing soon, due to the way the vector code shift bits...   epaper receives commands in 2 bits per pixel,  and 4 colours are 2 bits per pixel... to convert between requires a few boolean operations...and incredibly quick.  There maybe ways though ....maybe do multiple passes with different waveform arrays? but, this is far future.

# todo
* Make this a propper library
* Currently requires 2 paints to paint every line complete.  Add an interlace mode, which if turned off, will automatically do 2 paints 
* Screen is balanced when in use, but when turned off, the memory clears and when turned on, it doesn't know what pixels were dark/white in order to dc balance.  What would be perfect is a 'turn off' method that will clear the screen, and then turn off the M5PaperS3.... Or, even better, display an off image, before turning off. which is used to DC balance when turned back on.
* Add a quality setting, that adjusts the delay when each line is latched. It appears that a longer delay = darker image, at a cost of slower update.
  
