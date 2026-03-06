// Choose your board.
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include <lvgl.h>
#include "EPD_Painter_presets.h"
#include "EPD_Painter_LVGL.h"

EPD_PainterLVGL display(EPD_PAINTER_PRESET);

static uint32_t my_tick_cb() {
    return millis();
}

void setup() {
    Serial.begin(115200);

    lv_init();
    lv_tick_set_cb(my_tick_cb);

    if (!display.begin()) {
        Serial.println("Display init failed!");
        while (1) delay(1000);
    }

    display.clear();
    display.clear();

    lv_obj_set_style_bg_color(lv_screen_active(), EPD_PainterLVGL::WHITE, 0);

    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello World!");
    lv_obj_set_style_text_color(label, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_center(label);
}

void loop() {
    lv_timer_handler();
    delay(5);
}
