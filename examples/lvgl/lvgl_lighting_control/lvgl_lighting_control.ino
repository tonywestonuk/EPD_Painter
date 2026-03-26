// Choose your board.
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include <lvgl.h>
#define EPD_PAINTER_ENABLE_AUTO_SHUTDOWN 1
// Optional library flag used by the examples.
// Set to 1 to enable the reset-twice auto-shutdown helper and shutdown image flow.
// Leave it undefined or set it to 0 in normal projects unless you explicitly want this behaviour.

#include "EPD_Painter_presets.h"
#include "EPD_Painter_LVGL.h"

EPD_PainterLVGL display(EPD_PAINTER_PRESET);

static uint32_t my_tick_cb() { return millis(); }

// ── Room data ─────────────────────────────────────────────────────────────────

struct Room {
    const char *name;
    bool        on;
    int         brightness;  // 0–100
    float       temp;        // current °C
    float       setpoint;    // target °C
    float       kwh;         // today's usage
};

static Room rooms[] = {
    { "Living Room", true,  80,  21.5f, 22.0f, 1.8f },
    { "Kitchen",     true,  60,  20.0f, 20.0f, 0.9f },
    { "Bedroom",     false, 30,  18.5f, 19.0f, 0.3f },
    { "Bathroom",    true,  100, 22.0f, 22.0f, 0.5f },
    { "Study",       false, 50,  17.5f, 20.0f, 1.2f },
    { "Garden",      false, 20,  12.0f,  0.0f, 0.1f },
};
static const int ROOM_COUNT = 6;
static const int CLIM_COUNT = 4;  // first 4 rooms have climate control

// ── Screen cycling ────────────────────────────────────────────────────────────

static lv_obj_t *screens[3];
static int       current_screen = 0;
static uint32_t  last_switch_ms = 0;
static const uint32_t SWITCH_INTERVAL_MS = 2000;

// ── Shared layout constants ───────────────────────────────────────────────────

#define HEADER_H  56
#define MARGIN    14
#define GAP       12
#define CARD_PAD  14

// ── Shared helper: black header bar ──────────────────────────────────────────

static void make_header(lv_obj_t *scr, const char *title, const char *subtitle) {
    lv_obj_t *hdr = lv_obj_create(scr);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_size(hdr, 960, HEADER_H);
    lv_obj_set_style_bg_color(hdr, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_set_style_pad_hor(hdr, 18, 0);
    lv_obj_set_style_pad_ver(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *t = lv_label_create(hdr);
    lv_label_set_text(t, title);
    lv_obj_set_style_text_color(t, EPD_PainterLVGL::WHITE, 0);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_28, 0);
    lv_obj_align(t, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *s = lv_label_create(hdr);
    lv_label_set_text(s, subtitle);
    lv_obj_set_style_text_color(s, EPD_PainterLVGL::DK_GREY, 0);
    lv_obj_set_style_text_font(s, &lv_font_montserrat_28, 0);
    lv_obj_align(s, LV_ALIGN_RIGHT_MID, 0, 0);
}

// ── Screen 1: Lighting Control ────────────────────────────────────────────────

static void build_lighting_screen(lv_obj_t *scr) {
    const int COLS   = 3, ROWS = 2;
    const int CARD_W = (960 - 2 * MARGIN - (COLS - 1) * GAP) / COLS;
    const int CARD_H = (540 - HEADER_H - (ROWS + 1) * MARGIN - (ROWS - 1) * GAP) / ROWS;

    lv_obj_set_style_bg_color(scr, EPD_PainterLVGL::LT_GREY, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    int active = 0;
    for (int i = 0; i < ROOM_COUNT; i++) if (rooms[i].on) active++;
    char sub[32];
    snprintf(sub, sizeof(sub), "%d of %d on", active, ROOM_COUNT);
    make_header(scr, "Lighting Control", sub);

    for (int i = 0; i < ROOM_COUNT; i++) {
        int col = i % COLS;
        int row = i / COLS;
        int x   = MARGIN + col * (CARD_W + GAP);
        int y   = HEADER_H + MARGIN + row * (CARD_H + GAP);
        bool on = rooms[i].on;

        lv_color_t fg  = on ? EPD_PainterLVGL::WHITE   : EPD_PainterLVGL::BLACK;
        lv_color_t dim = on ? EPD_PainterLVGL::DK_GREY : EPD_PainterLVGL::DK_GREY;
        lv_color_t bg  = on ? EPD_PainterLVGL::BLACK   : EPD_PainterLVGL::WHITE;

        lv_obj_t *card = lv_obj_create(scr);
        lv_obj_set_pos(card, x, y);
        lv_obj_set_size(card, CARD_W, CARD_H);
        lv_obj_set_style_bg_color(card, bg, 0);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(card, EPD_PainterLVGL::DK_GREY, 0);
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_set_style_radius(card, 8, 0);
        lv_obj_set_style_pad_all(card, CARD_PAD, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *name_lbl = lv_label_create(card);
        lv_label_set_text(name_lbl, rooms[i].name);
        lv_obj_set_style_text_color(name_lbl, fg, 0);
        lv_obj_set_style_text_font(name_lbl, &lv_font_montserrat_28, 0);
        lv_obj_align(name_lbl, LV_ALIGN_TOP_LEFT, 0, 0);

        // ON/OFF badge
        lv_obj_t *badge = lv_obj_create(card);
        lv_obj_set_size(badge, 80, 42);
        lv_obj_set_style_bg_color(badge, on ? EPD_PainterLVGL::DK_GREY : EPD_PainterLVGL::LT_GREY, 0);
        lv_obj_set_style_bg_opa(badge, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(badge, 0, 0);
        lv_obj_set_style_radius(badge, 21, 0);
        lv_obj_set_style_pad_all(badge, 0, 0);
        lv_obj_align(badge, LV_ALIGN_TOP_RIGHT, 0, 0);
        lv_obj_clear_flag(badge, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *badge_lbl = lv_label_create(badge);
        lv_label_set_text(badge_lbl, on ? "ON" : "OFF");
        lv_obj_set_style_text_color(badge_lbl, fg, 0);
        lv_obj_set_style_text_font(badge_lbl, &lv_font_montserrat_28, 0);
        lv_obj_center(badge_lbl);

        // Brightness %
        char brt_str[8];
        snprintf(brt_str, sizeof(brt_str), "%d%%", rooms[i].brightness);
        lv_obj_t *brt_lbl = lv_label_create(card);
        lv_label_set_text(brt_lbl, brt_str);
        lv_obj_set_style_text_color(brt_lbl, dim, 0);
        lv_obj_set_style_text_font(brt_lbl, &lv_font_montserrat_28, 0);
        lv_obj_align(brt_lbl, LV_ALIGN_BOTTOM_MID, 0, -(42 + 8 + 16 + 6));

        // Brightness bar
        lv_obj_t *bar = lv_bar_create(card);
        lv_obj_set_size(bar, lv_pct(100), 16);
        lv_bar_set_range(bar, 0, 100);
        lv_bar_set_value(bar, on ? rooms[i].brightness : 0, LV_ANIM_OFF);
        lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, -(42 + 8));
        lv_obj_set_style_bg_color(bar, dim, 0);
        lv_obj_set_style_bg_color(bar, fg, LV_PART_INDICATOR);
        lv_obj_set_style_radius(bar, 4, 0);
        lv_obj_set_style_radius(bar, 4, LV_PART_INDICATOR);
        lv_obj_set_style_border_width(bar, 0, 0);

        // Toggle button
        lv_obj_t *btn = lv_obj_create(card);
        lv_obj_set_size(btn, lv_pct(100), 42);
        lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_color(btn, on ? EPD_PainterLVGL::DK_GREY : EPD_PainterLVGL::BLACK, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_set_style_pad_all(btn, 0, 0);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *btn_lbl = lv_label_create(btn);
        lv_label_set_text(btn_lbl, on ? "Turn Off" : "Turn On");
        lv_obj_set_style_text_color(btn_lbl, EPD_PainterLVGL::WHITE, 0);
        lv_obj_set_style_text_font(btn_lbl, &lv_font_montserrat_28, 0);
        lv_obj_center(btn_lbl);
    }
}

// ── Screen 2: Climate Control ─────────────────────────────────────────────────

static void build_climate_screen(lv_obj_t *scr) {
    const int COLS   = 2, ROWS = 2;
    const int CARD_W = (960 - 2 * MARGIN - (COLS - 1) * GAP) / COLS;
    const int CARD_H = (540 - HEADER_H - (ROWS + 1) * MARGIN - (ROWS - 1) * GAP) / ROWS;

    lv_obj_set_style_bg_color(scr, EPD_PainterLVGL::LT_GREY, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    float avg = 0;
    for (int i = 0; i < CLIM_COUNT; i++) avg += rooms[i].temp;
    avg /= CLIM_COUNT;
    char sub[32];
    snprintf(sub, sizeof(sub), "Avg %.1f C", avg);
    make_header(scr, "Climate Control", sub);

    for (int i = 0; i < CLIM_COUNT; i++) {
        int col = i % COLS;
        int row = i / COLS;
        int x   = MARGIN + col * (CARD_W + GAP);
        int y   = HEADER_H + MARGIN + row * (CARD_H + GAP);
        bool heating = rooms[i].temp < rooms[i].setpoint;

        lv_obj_t *card = lv_obj_create(scr);
        lv_obj_set_pos(card, x, y);
        lv_obj_set_size(card, CARD_W, CARD_H);
        lv_obj_set_style_bg_color(card, EPD_PainterLVGL::WHITE, 0);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(card, EPD_PainterLVGL::DK_GREY, 0);
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_set_style_radius(card, 8, 0);
        lv_obj_set_style_pad_all(card, CARD_PAD, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *name_lbl = lv_label_create(card);
        lv_label_set_text(name_lbl, rooms[i].name);
        lv_obj_set_style_text_color(name_lbl, EPD_PainterLVGL::BLACK, 0);
        lv_obj_set_style_text_font(name_lbl, &lv_font_montserrat_28, 0);
        lv_obj_align(name_lbl, LV_ALIGN_TOP_LEFT, 0, 0);

        // HEATING / IDLE badge
        lv_obj_t *badge = lv_obj_create(card);
        lv_obj_set_size(badge, 140, 42);
        lv_obj_set_style_bg_color(badge, heating ? EPD_PainterLVGL::BLACK : EPD_PainterLVGL::LT_GREY, 0);
        lv_obj_set_style_bg_opa(badge, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(badge, 0, 0);
        lv_obj_set_style_radius(badge, 21, 0);
        lv_obj_set_style_pad_all(badge, 0, 0);
        lv_obj_align(badge, LV_ALIGN_TOP_RIGHT, 0, 0);
        lv_obj_clear_flag(badge, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *badge_lbl = lv_label_create(badge);
        lv_label_set_text(badge_lbl, heating ? "HEATING" : "IDLE");
        lv_obj_set_style_text_color(badge_lbl, heating ? EPD_PainterLVGL::WHITE : EPD_PainterLVGL::DK_GREY, 0);
        lv_obj_set_style_text_font(badge_lbl, &lv_font_montserrat_28, 0);
        lv_obj_center(badge_lbl);

        // Current temperature (large)
        char temp_str[8];
        snprintf(temp_str, sizeof(temp_str), "%.1f", rooms[i].temp);
        lv_obj_t *temp_lbl = lv_label_create(card);
        lv_label_set_text(temp_lbl, temp_str);
        lv_obj_set_style_text_color(temp_lbl, EPD_PainterLVGL::BLACK, 0);
        lv_obj_set_style_text_font(temp_lbl, &lv_font_montserrat_48, 0);
        lv_obj_align(temp_lbl, LV_ALIGN_CENTER, -20, 0);

        lv_obj_t *unit_lbl = lv_label_create(card);
        lv_label_set_text(unit_lbl, "C");
        lv_obj_set_style_text_color(unit_lbl, EPD_PainterLVGL::DK_GREY, 0);
        lv_obj_set_style_text_font(unit_lbl, &lv_font_montserrat_28, 0);
        lv_obj_align_to(unit_lbl, temp_lbl, LV_ALIGN_OUT_RIGHT_TOP, 6, 8);

        // Setpoint
        char set_str[20];
        snprintf(set_str, sizeof(set_str), "Set point  %.1f C", rooms[i].setpoint);
        lv_obj_t *set_lbl = lv_label_create(card);
        lv_label_set_text(set_lbl, set_str);
        lv_obj_set_style_text_color(set_lbl, EPD_PainterLVGL::DK_GREY, 0);
        lv_obj_set_style_text_font(set_lbl, &lv_font_montserrat_28, 0);
        lv_obj_align(set_lbl, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    }
}

// ── Screen 3: Energy Monitor ──────────────────────────────────────────────────

static void build_energy_screen(lv_obj_t *scr) {
    lv_obj_set_style_bg_color(scr, EPD_PainterLVGL::WHITE, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    float total = 0, max_kwh = 0;
    for (int i = 0; i < ROOM_COUNT; i++) {
        total += rooms[i].kwh;
        if (rooms[i].kwh > max_kwh) max_kwh = rooms[i].kwh;
    }
    char sub[40];
    snprintf(sub, sizeof(sub), "Total  %.1f kWh today", total);
    make_header(scr, "Energy Monitor", sub);

    const int ROW_H   = 64;
    const int ROW_GAP = 9;
    const int START_Y = HEADER_H + MARGIN;
    const int LABEL_W = 200;
    const int VALUE_W = 130;
    const int BAR_X   = MARGIN + LABEL_W + 10;
    const int BAR_W   = 960 - BAR_X - VALUE_W - MARGIN;

    for (int i = 0; i < ROOM_COUNT; i++) {
        int y = START_Y + i * (ROW_H + ROW_GAP);

        // Alternating row background
        lv_obj_t *row_bg = lv_obj_create(scr);
        lv_obj_set_pos(row_bg, MARGIN, y);
        lv_obj_set_size(row_bg, 960 - 2 * MARGIN, ROW_H);
        lv_obj_set_style_bg_color(row_bg, (i % 2 == 0) ? EPD_PainterLVGL::WHITE : EPD_PainterLVGL::LT_GREY, 0);
        lv_obj_set_style_bg_opa(row_bg, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(row_bg, 0, 0);
        lv_obj_set_style_radius(row_bg, 4, 0);
        lv_obj_set_style_pad_all(row_bg, 0, 0);
        lv_obj_clear_flag(row_bg, LV_OBJ_FLAG_SCROLLABLE);

        // Room name
        lv_obj_t *name_lbl = lv_label_create(scr);
        lv_label_set_text(name_lbl, rooms[i].name);
        lv_obj_set_pos(name_lbl, MARGIN + 10, y + (ROW_H - 36) / 2);
        lv_obj_set_style_text_color(name_lbl, EPD_PainterLVGL::BLACK, 0);
        lv_obj_set_style_text_font(name_lbl, &lv_font_montserrat_28, 0);

        // Bar
        int bar_val = (max_kwh > 0) ? (int)(rooms[i].kwh / max_kwh * 100) : 0;
        lv_obj_t *bar = lv_bar_create(scr);
        lv_obj_set_pos(bar, BAR_X, y + (ROW_H - 24) / 2);
        lv_obj_set_size(bar, BAR_W, 24);
        lv_bar_set_range(bar, 0, 100);
        lv_bar_set_value(bar, bar_val, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(bar, EPD_PainterLVGL::LT_GREY, 0);
        lv_obj_set_style_bg_color(bar, EPD_PainterLVGL::BLACK, LV_PART_INDICATOR);
        lv_obj_set_style_radius(bar, 4, 0);
        lv_obj_set_style_radius(bar, 4, LV_PART_INDICATOR);
        lv_obj_set_style_border_width(bar, 0, 0);

        // kWh value
        char val_str[12];
        snprintf(val_str, sizeof(val_str), "%.1f kWh", rooms[i].kwh);
        lv_obj_t *val_lbl = lv_label_create(scr);
        lv_label_set_text(val_lbl, val_str);
        lv_obj_set_pos(val_lbl, BAR_X + BAR_W + 12, y + (ROW_H - 36) / 2);
        lv_obj_set_style_text_color(val_lbl, EPD_PainterLVGL::DK_GREY, 0);
        lv_obj_set_style_text_font(val_lbl, &lv_font_montserrat_28, 0);
    }
}

// ── Entry points ──────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    lv_init();
    lv_tick_set_cb(my_tick_cb);

    if (!display.begin()) {
        Serial.println("Display init failed!");
        while (1) delay(1000);
    }

    display.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
    display.clear();

    screens[0] = lv_obj_create(NULL);
    screens[1] = lv_obj_create(NULL);
    screens[2] = lv_obj_create(NULL);

    build_lighting_screen(screens[0]);
    build_climate_screen(screens[1]);
    build_energy_screen(screens[2]);

    lv_screen_load(screens[0]);
    last_switch_ms = millis();
}

void loop() {
    if (millis() - last_switch_ms >= SWITCH_INTERVAL_MS) {
        current_screen = (current_screen + 1) % 3;
        lv_screen_load(screens[current_screen]);
        last_switch_ms = millis();
    }
    lv_timer_handler();
    delay(5);
}
