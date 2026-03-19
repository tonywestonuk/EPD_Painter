// LVGL portrait demo — EPD_Painter with portrait=true (90° CW rotation).
//
// Four pages, swipe left/right between them via lv_tileview.
// Logical canvas: 540 wide × 960 tall (portrait).
// Physical panel: 960 wide × 540 tall (landscape).
// compact_pixels_rotated_cw() handles the rotation during compaction.

// Choose your board.
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include <lvgl.h>
#include "EPD_Painter_presets.h"
#include "EPD_Painter_LVGL.h"
#include <TAMC_GT911.h>

EPD_PainterLVGL display(EPD_PAINTER_PRESET, true);

// Touch controller — ROTATION_INVERTED maps correctly for CW display rotation.
TAMC_GT911 tc(-1, EPD_PAINTER_PRESET.width, EPD_PAINTER_PRESET.height);

LV_FONT_DECLARE(Montserrat_Bold);

static uint32_t my_tick_cb() { return millis(); }

// ---------------------------------------------------------------------------
// Widget state
// ---------------------------------------------------------------------------
static lv_obj_t *slider1_label;
static lv_obj_t *slider2_label;

static void slider1_event_cb(lv_event_t *e) {
    int val = (int)lv_slider_get_value(lv_event_get_target_obj(e));
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d%%", val);
    lv_label_set_text(slider1_label, buf);
}

static void slider2_event_cb(lv_event_t *e) {
    int val = (int)lv_slider_get_value(lv_event_get_target_obj(e));
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d%%", val);
    lv_label_set_text(slider2_label, buf);
}

static void switch_event_cb(lv_event_t *e)   { /* handle switch toggle  */ }
static void button_event_cb(lv_event_t *e)   { /* handle button press   */ }
static void checkbox_event_cb(lv_event_t *e) { /* handle checkbox change */ }

static uint32_t last_activity_ms = 0;
static bool     idle_cleared     = false;

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    tc.read();
    // ROTATION_INVERTED in TAMC_GT911 maps physical touch coordinates to
    // portrait logical space correctly for CW display rotation on LilyGo T5.
    data->point.x = tc.x;
    data->point.y = tc.y;
    data->state   = tc.down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;

    if (tc.down) {
        last_activity_ms = millis();
        idle_cleared     = false;
    }
}

// ---------------------------------------------------------------------------
// Layout helpers
// ---------------------------------------------------------------------------

static void make_page_header(lv_obj_t *parent, const char *title_text) {
    lv_obj_t *title = lv_label_create(parent);
    lv_label_set_text(title, title_text);
    lv_obj_set_style_text_color(title, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(title, &Montserrat_Bold, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);

    lv_obj_t *div = lv_obj_create(parent);
    lv_obj_set_size(div, 480, 2);
    lv_obj_align(div, LV_ALIGN_TOP_MID, 0, 80);
    lv_obj_set_style_bg_color(div, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_border_width(div, 0, 0);
}

// Page-indicator dots at the bottom; filled dot marks the current page.
static void make_page_dots(lv_obj_t *parent, int current_page, int total_pages) {
    const int dot_size = 14;
    const int dot_gap  = 28;   // centre-to-centre spacing

    for (int i = 0; i < total_pages; i++) {
        int x_ofs = -(total_pages - 1) * dot_gap / 2 + i * dot_gap;

        lv_obj_t *dot = lv_obj_create(parent);
        lv_obj_set_size(dot, dot_size, dot_size);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_color(dot, EPD_PainterLVGL::BLACK, 0);
        lv_obj_set_style_border_width(dot, 2, 0);
        lv_obj_set_style_pad_all(dot, 0, 0);
        lv_obj_set_style_bg_color(dot,
            (i == current_page) ? EPD_PainterLVGL::BLACK : EPD_PainterLVGL::WHITE, 0);
        lv_obj_align(dot, LV_ALIGN_BOTTOM_MID, x_ofs, -30);
    }
}

// A labelled slider; value label sits to the right of the track.
static void make_slider(lv_obj_t *parent, int y_ofs, int init_val,
                        lv_event_cb_t cb, lv_obj_t **out_label)
{
    lv_obj_t *s = lv_slider_create(parent);
    lv_obj_set_size(s, 380, 36);
    lv_slider_set_range(s, 0, 100);
    lv_slider_set_value(s, init_val, LV_ANIM_OFF);
    lv_obj_align(s, LV_ALIGN_TOP_MID, -40, y_ofs);
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
    lv_obj_set_style_text_font(*out_label, &Montserrat_Bold, 0);
    lv_obj_align_to(*out_label, s, LV_ALIGN_OUT_RIGHT_MID, 16, 0);
}

static void make_switch(lv_obj_t *parent, lv_align_t align, int x_ofs, int y_ofs, int id) {
    lv_obj_t *sw = lv_switch_create(parent);
    lv_obj_set_size(sw, 180, 90);
    lv_obj_align(sw, align, x_ofs, y_ofs);
    lv_obj_set_style_bg_color(sw, EPD_PainterLVGL::LT_GREY, LV_PART_MAIN);
    lv_obj_set_style_border_color(sw, EPD_PainterLVGL::BLACK, LV_PART_MAIN);
    lv_obj_set_style_border_width(sw, 2, LV_PART_MAIN);
    lv_obj_set_style_bg_color(sw, EPD_PainterLVGL::DK_GREY, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(sw, EPD_PainterLVGL::BLACK, LV_PART_KNOB);
    lv_obj_add_event_cb(sw, switch_event_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)id);
}

static void make_button(lv_obj_t *parent, const char *label_text, int y_ofs, int id) {
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_size(btn, 440, 90);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, y_ofs);
    lv_obj_set_style_bg_color(btn, EPD_PainterLVGL::WHITE, 0);
    lv_obj_set_style_bg_color(btn, EPD_PainterLVGL::LT_GREY, LV_STATE_PRESSED);
    lv_obj_set_style_radius(btn, 8, 0);
    lv_obj_add_event_cb(btn, button_event_cb, LV_EVENT_CLICKED, (void*)(intptr_t)id);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, label_text);
    lv_obj_set_style_text_color(lbl, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(lbl, &Montserrat_Bold, 0);
    lv_obj_center(lbl);
}

static void make_checkbox(lv_obj_t *parent, const char *text, int y_ofs,
                          bool checked, int id)
{
    lv_obj_t *cb = lv_checkbox_create(parent);
    lv_checkbox_set_text(cb, text);
    lv_obj_align(cb, LV_ALIGN_TOP_LEFT, 40, y_ofs);
    lv_obj_set_style_text_color(cb, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(cb, &Montserrat_Bold, 0);
    lv_obj_set_style_bg_color(cb, EPD_PainterLVGL::WHITE, LV_PART_INDICATOR);
    lv_obj_set_style_border_color(cb, EPD_PainterLVGL::BLACK, LV_PART_INDICATOR);
    lv_obj_set_style_border_width(cb, 2, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(cb, EPD_PainterLVGL::DK_GREY,
                              LV_PART_INDICATOR | LV_STATE_CHECKED);
    if (checked) lv_obj_add_state(cb, LV_STATE_CHECKED);
    lv_obj_add_event_cb(cb, checkbox_event_cb, LV_EVENT_VALUE_CHANGED,
                        (void*)(intptr_t)id);
}

static void make_bar(lv_obj_t *parent, int y_ofs, int val) {
    lv_obj_t *bar = lv_bar_create(parent);
    lv_obj_set_size(bar, 440, 36);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, y_ofs);
    lv_bar_set_range(bar, 0, 100);
    lv_bar_set_value(bar, val, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar, EPD_PainterLVGL::LT_GREY, LV_PART_MAIN);
    lv_obj_set_style_border_color(bar, EPD_PainterLVGL::BLACK, LV_PART_MAIN);
    lv_obj_set_style_border_width(bar, 2, LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, EPD_PainterLVGL::DK_GREY, LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar, 4, LV_PART_MAIN);
    lv_obj_set_style_radius(bar, 4, LV_PART_INDICATOR);
}

// Helper for the text pages — a body-text label with wrapping.
static lv_obj_t *make_body_text(lv_obj_t *parent, const char *text, int y_ofs) {
    lv_obj_t *lbl = lv_label_create(parent);
    lv_label_set_text(lbl, text);
    lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl, 480);
    lv_obj_set_style_text_color(lbl, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(lbl, &Montserrat_Bold, 0);
    lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, y_ofs);
    return lbl;
}

// ---------------------------------------------------------------------------
// Page builders
// ---------------------------------------------------------------------------

static void build_page1(lv_obj_t *tile) {
    lv_obj_set_style_bg_color(tile, EPD_PainterLVGL::WHITE, 0);

    make_page_header(tile, "Controls");

    // -- Sliders -------------------------------------------------------------
    lv_obj_t *s1_heading = lv_label_create(tile);
    lv_label_set_text(s1_heading, "Slider 1");
    lv_obj_set_style_text_color(s1_heading, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(s1_heading, &Montserrat_Bold, 0);
    lv_obj_align(s1_heading, LV_ALIGN_TOP_LEFT, 30, 110);

    make_slider(tile, 150, 50, slider1_event_cb, &slider1_label);

    lv_obj_t *s2_heading = lv_label_create(tile);
    lv_label_set_text(s2_heading, "Slider 2");
    lv_obj_set_style_text_color(s2_heading, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(s2_heading, &Montserrat_Bold, 0);
    lv_obj_align(s2_heading, LV_ALIGN_TOP_LEFT, 30, 220);

    make_slider(tile, 260, 25, slider2_event_cb, &slider2_label);

    // -- Switches ------------------------------------------------------------
    lv_obj_t *sw_heading = lv_label_create(tile);
    lv_label_set_text(sw_heading, "Switches");
    lv_obj_set_style_text_color(sw_heading, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(sw_heading, &Montserrat_Bold, 0);
    lv_obj_align(sw_heading, LV_ALIGN_TOP_LEFT, 30, 330);

    make_switch(tile, LV_ALIGN_TOP_LEFT,  50, 375, 1);
    make_switch(tile, LV_ALIGN_TOP_RIGHT, -50, 375, 2);

    // -- Buttons -------------------------------------------------------------
    lv_obj_t *btn_heading = lv_label_create(tile);
    lv_label_set_text(btn_heading, "Buttons");
    lv_obj_set_style_text_color(btn_heading, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(btn_heading, &Montserrat_Bold, 0);
    lv_obj_align(btn_heading, LV_ALIGN_TOP_LEFT, 30, 500);

    make_button(tile, "Button 1", 545, 1);
    make_button(tile, "Button 2", 655, 2);
    make_button(tile, "Button 3", 765, 3);

    make_page_dots(tile, 0, 4);
}

static void build_page2(lv_obj_t *tile) {
    lv_obj_set_style_bg_color(tile, EPD_PainterLVGL::WHITE, 0);

    make_page_header(tile, "Settings");

    // -- Status box ----------------------------------------------------------
    lv_obj_t *box = lv_obj_create(tile);
    lv_obj_set_size(box, 460, 140);
    lv_obj_align(box, LV_ALIGN_TOP_MID, 0, 110);
    lv_obj_set_style_bg_color(box, EPD_PainterLVGL::WHITE, 0);
    lv_obj_set_style_border_color(box, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_border_width(box, 2, 0);
    lv_obj_set_style_radius(box, 8, 0);

    lv_obj_t *box_text = lv_label_create(box);
    lv_label_set_text(box_text, "Status: Online\nSignal: Good\nBattery: 87%");
    lv_obj_set_style_text_color(box_text, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(box_text, &Montserrat_Bold, 0);
    lv_obj_align(box_text, LV_ALIGN_TOP_LEFT, 12, 12);

    // -- Checkboxes ----------------------------------------------------------
    lv_obj_t *cb_heading = lv_label_create(tile);
    lv_label_set_text(cb_heading, "Options");
    lv_obj_set_style_text_color(cb_heading, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(cb_heading, &Montserrat_Bold, 0);
    lv_obj_align(cb_heading, LV_ALIGN_TOP_LEFT, 30, 295);

    make_checkbox(tile, "Enable notifications", 345, true,  1);
    make_checkbox(tile, "Auto-refresh",         425, false, 2);
    make_checkbox(tile, "Dark mode",            505, false, 3);

    // -- Progress bars -------------------------------------------------------
    lv_obj_t *pb_heading = lv_label_create(tile);
    lv_label_set_text(pb_heading, "Progress");
    lv_obj_set_style_text_color(pb_heading, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(pb_heading, &Montserrat_Bold, 0);
    lv_obj_align(pb_heading, LV_ALIGN_TOP_LEFT, 30, 605);

    make_bar(tile, 655, 72);
    make_bar(tile, 715, 45);

    make_page_dots(tile, 1, 4);
}

static void build_page3(lv_obj_t *tile) {
    lv_obj_set_style_bg_color(tile, EPD_PainterLVGL::WHITE, 0);

    make_page_header(tile, "E-Paper Displays");

    make_body_text(tile,
        "E-paper pixels are tiny charged"
        " particles. Voltage flips them"
        " black or white. No backlight.",
        100);

    make_body_text(tile,
        "Once set, no power holds the"
        " image. Switch off the display"
        " and the picture stays forever.",
        310);

    make_body_text(tile,
        "Battery life is extraordinary."
        " Power is only used during an"
        " update, not to hold the image.",
        510);

    make_body_text(tile,
        "Readable in direct sunlight."
        " EPDs reflect light like ink"
        " on paper, with no glare.",
        710);

    make_page_dots(tile, 2, 4);
}

static void build_page4(lv_obj_t *tile) {
    lv_obj_set_style_bg_color(tile, EPD_PainterLVGL::WHITE, 0);

    make_page_header(tile, "EPD_Painter");

    make_body_text(tile,
        "A high-performance e-paper driver"
        " for the ESP32-S3. Targets the"
        " M5PaperS3 and LilyGo T5 S3.",
        100);

    make_body_text(tile,
        "Xtensa SIMD assembly and LCD_CAM"
        " DMA push updates at up to 20fps"
        " equivalent - far faster than"
        " any stock EPD driver.",
        310);

    make_body_text(tile,
        "Only changed pixels are driven."
        " A delta pipeline compares each"
        " frame against the screen buffer"
        " and moves only what changed.",
        530);

    make_body_text(tile,
        "LVGL v9 and Adafruit GFX both"
        " supported. Portrait mode, touch,"
        " shutdown, and power management"
        " all built in.",
        750);

    make_page_dots(tile, 3, 4);
}

// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    lv_init();
    lv_tick_set_cb(my_tick_cb);

    if (!display.begin()) {
        Serial.println("Display init failed!");
        while (1) delay(1000);
    }

    if (display.getConfig().i2c.wire == nullptr) {
        Serial.println("TAMC_GT911: Wire is null, skipping touch init");
    } else {
        tc.setRotation(ROTATION_INVERTED);
        tc.begin(display.getConfig().i2c.wire);
    }

    display.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
    display.clear();

    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, EPD_PainterLVGL::WHITE, 0);

    // -- Tileview — four tiles side by side, swipe left/right ---------------
    lv_obj_t *tv = lv_tileview_create(scr);
    lv_obj_set_size(tv, 540, 960);
    lv_obj_set_pos(tv, 0, 0);
    lv_obj_set_style_bg_color(tv, EPD_PainterLVGL::WHITE, 0);

    lv_obj_t *tile1 = lv_tileview_add_tile(tv, 0, 0, LV_DIR_RIGHT);
    lv_obj_t *tile2 = lv_tileview_add_tile(tv, 1, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));
    lv_obj_t *tile3 = lv_tileview_add_tile(tv, 2, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));
    lv_obj_t *tile4 = lv_tileview_add_tile(tv, 3, 0, LV_DIR_LEFT);

    build_page1(tile1);
    build_page2(tile2);
    build_page3(tile3);
    build_page4(tile4);

    // -- Touch input ---------------------------------------------------------
    lv_indev_t *touch = lv_indev_create();
    lv_indev_set_type(touch, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touch, touch_read_cb);

    last_activity_ms = millis();
}

void loop() {
    lv_timer_handler();
    delay(5);

    if (!idle_cleared && (millis() - last_activity_ms) > 3000) {
        idle_cleared = true;
        display.fxClear();
    }
}
