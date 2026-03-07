// Choose your board.
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include <lvgl.h>
#include "EPD_Painter_presets.h"
#include "EPD_Painter_LVGL.h"
#include <TAMC_GT911.h>

EPD_PainterLVGL display(EPD_PAINTER_PRESET);
TAMC_GT911 tc(-1, EPD_PAINTER_PRESET.width, EPD_PAINTER_PRESET.height);

static uint32_t my_tick_cb() {
    return millis();
}


static lv_obj_t *slider1_label;
static lv_obj_t *slider2_label;

static void slider1_event_cb(lv_event_t *e) {
    int val = (int)lv_slider_get_value(lv_event_get_target_obj(e));
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d%%", val);
    lv_label_set_text(slider1_label, buf);
//    Serial.printf("Slider 1: %d\n", val);
}

static void slider2_event_cb(lv_event_t *e) {
    int val = (int)lv_slider_get_value(lv_event_get_target_obj(e));
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d%%", val);
    lv_label_set_text(slider2_label, buf);
//    Serial.printf("Slider 2: %d\n", val);
}

static void switch_event_cb(lv_event_t *e) {
    lv_obj_t *sw = lv_event_get_target_obj(e);
    bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);
 //   Serial.printf("Switch %d: %s\n", (int)(intptr_t)lv_event_get_user_data(e), on ? "ON" : "OFF");
}

static void button_event_cb(lv_event_t *e) {
 //   Serial.printf("Button %d pressed\n", (int)(intptr_t)lv_event_get_user_data(e));
}
static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    // data->point.x = 480;
    // data->point.y = 270;
    // data->state   = touchHeld ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;

    // touchCount -= 1;
    // if (touchCount == 0) {
    //     touchHeld   = !touchHeld;
    //     data->state = touchHeld ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    //     Serial.println(touchHeld ? "Sending PRESSED" : "Sending RELEASED");
    //     touchCount  = 100;
    // }

    tc.read();
    data->point.x = tc.x;
    data->point.y = tc.y;
    data->state   = tc.down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

void setup() {
    Serial.begin(115200);

    lv_init();
    lv_tick_set_cb(my_tick_cb);

    if (!display.begin()) {
        Serial.println("Display init failed!");
        while (1) delay(1000);
    }

   if (display.getConfig().i2c.wire == nullptr) {                                                                                                  
        Serial.println("TAMC_GT911: Wire is null, aborting begin()");                                                         
        return;
   } else {
        Serial.println("wire is not null");
    }  
    Serial.println("got there");

    tc.setRotation(ROTATION_RIGHT);
    tc.begin(display.getConfig().i2c.wire);
    Serial.println("got here");

    display.clear();

    lv_obj_set_style_bg_color(lv_screen_active(), EPD_PainterLVGL::WHITE, 0);

    // Title
    lv_obj_t *title = lv_label_create(lv_screen_active());
    lv_label_set_text(title, "LVGL Touch Demo");
    lv_obj_set_style_text_color(title, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_48, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // ── Sliders ──────────────────────────────────────────────────────────────
    auto make_slider = [](lv_obj_t *parent, int y_ofs, int init_val,
                          lv_event_cb_t cb, lv_obj_t **out_label) {
        lv_obj_t *s = lv_slider_create(parent);
        lv_obj_set_size(s, 700, 40);
        lv_slider_set_range(s, 0, 100);
        lv_slider_set_value(s, init_val, LV_ANIM_OFF);
        lv_obj_align(s, LV_ALIGN_TOP_MID, -60, y_ofs);
        lv_obj_set_style_bg_color(s, EPD_PainterLVGL::LT_GREY, LV_PART_MAIN);
        lv_obj_set_style_border_color(s, EPD_PainterLVGL::BLACK, LV_PART_MAIN);
        lv_obj_set_style_border_width(s, 2, LV_PART_MAIN);
        lv_obj_set_style_bg_color(s, EPD_PainterLVGL::DK_GREY, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(s, EPD_PainterLVGL::BLACK, LV_PART_KNOB);
        lv_obj_add_event_cb(s, cb, LV_EVENT_VALUE_CHANGED, nullptr);

        *out_label = lv_label_create(parent);
        char buf[8];
        lv_snprintf(buf, sizeof(buf), "%d%%", init_val);
        lv_label_set_text(*out_label, buf);
        lv_obj_set_style_text_color(*out_label, EPD_PainterLVGL::BLACK, 0);
        lv_obj_set_style_text_font(*out_label, &lv_font_montserrat_48, 0);
        lv_obj_align_to(*out_label, s, LV_ALIGN_OUT_RIGHT_MID, 20, 0);
    };

    make_slider(lv_screen_active(), 90,  50, slider1_event_cb, &slider1_label);
    make_slider(lv_screen_active(), 150, 25, slider2_event_cb, &slider2_label);

    // ── Switches ─────────────────────────────────────────────────────────────
    auto make_switch = [](lv_obj_t *parent, lv_align_t align, int x_ofs, int y_ofs, int id) {
        lv_obj_t *sw = lv_switch_create(parent);
        lv_obj_set_size(sw, 200, 100);
        lv_obj_align(sw, align, x_ofs, y_ofs);
        lv_obj_set_style_bg_color(sw, EPD_PainterLVGL::LT_GREY, LV_PART_MAIN);
        lv_obj_set_style_border_color(sw, EPD_PainterLVGL::BLACK, LV_PART_MAIN);
        lv_obj_set_style_border_width(sw, 2, LV_PART_MAIN);
        lv_obj_set_style_bg_color(sw, EPD_PainterLVGL::DK_GREY, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(sw, EPD_PainterLVGL::BLACK, LV_PART_KNOB);
        lv_obj_add_event_cb(sw, switch_event_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)id);
    };

    make_switch(lv_screen_active(), LV_ALIGN_TOP_LEFT,  80,  280, 1);
    make_switch(lv_screen_active(), LV_ALIGN_TOP_RIGHT, -80, 280, 2);

    // ── Buttons ───────────────────────────────────────────────────────────────
    const char *btn_labels[] = { "Button 1", "Button 2", "Button 3" };
    for (int i = 0; i < 3; i++) {
        lv_obj_t *btn = lv_button_create(lv_screen_active());
        lv_obj_set_size(btn, 260, 100);
        lv_obj_align(btn, LV_ALIGN_TOP_MID, (i - 1) * 300, 410);
        lv_obj_set_style_bg_color(btn, EPD_PainterLVGL::LT_GREY, 0);
        lv_obj_set_style_bg_color(btn, EPD_PainterLVGL::DK_GREY, LV_STATE_PRESSED);
        lv_obj_set_style_radius(btn, 8, 0);
        lv_obj_add_event_cb(btn, button_event_cb, LV_EVENT_CLICKED, (void*)(intptr_t)(i + 1));

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, btn_labels[i]);
        lv_obj_set_style_text_color(lbl, EPD_PainterLVGL::BLACK, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_48, 0);
        lv_obj_set_style_outline_width(lbl, 1, 0);
        lv_obj_set_style_outline_color(lbl, EPD_PainterLVGL::BLACK, 0);
        lv_obj_center(lbl);
    }

    // Register touch input device.
    lv_indev_t *touch = lv_indev_create();
    lv_indev_set_type(touch, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touch, touch_read_cb);
}

void loop() {
    lv_timer_handler();
    delay(5);
}