# EPD_Painter
Fast m5paperS3 epaper driver

Currently just a Proof of concept Arduino project.

Download, open in arduino IDE.

This should work straight away on an M5PaperS3,  for other epaper devices that use an ESP32-S3, update the pins in EPD_Painter.h

# todo
* Make this a propper library
* Currently requires 2 paints to paint every line complete.  Add an interlace mode, which if turned off, will automatically do 2 paints 
* Screen is balanced when in use, but when turned off, the memory clears and when turned on, it doesn't know what pixels were dark/white in order to dc balance.  What would be perfect is a 'turn off' method that will clear the screen, and then turn off the M5PaperS3.... Or, even better, display an off image, before turning off. which is used to DC balance when turned back on.
  
