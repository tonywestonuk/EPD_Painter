// Pixel Post Controller – effect/channel selector UI for pixel LED systems.
// Board: LilyGo T5 S3 GPS

#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include <lvgl.h>
#include "EPD_Painter_presets.h"
#include "EPD_Painter_LVGL.h"
#include "epd_painter_shutdown.h"  // needed for EPD_PainterShutdown* type
#include <gt911_lite.h>
#include <I2C_BM8563.h>
#include "PixelPostNetwork.h"
#include "battery.h"

// HMAC key – must match the key on all pixel post receivers.
static const uint8_t HMAC_KEY[32] = {
  0x0a, 0x8d, 0x29, 0x33, 0xb0, 0x89, 0x36, 0x30,
  0xf4, 0xfc, 0xe9, 0x5c, 0x29, 0xed, 0xc0, 0xf9,
  0xd5, 0xd8, 0x0e, 0x6e, 0xca, 0xe0, 0x10, 0x3d,
  0xeb, 0x20, 0x6b, 0xb6, 0xa6, 0x08, 0x60, 0x1e
};
static PixelPostNetwork net(HMAC_KEY);

EPD_PainterLVGL display(EPD_PAINTER_PRESET);
GT911_Lite tc;
static I2C_BM8563 *rtc = nullptr;
static EPD_PainterShutdown *psd = nullptr;

static uint32_t my_tick_cb() {
  return millis();
}

// ── Backlight (LilyGo only) ───────────────────────────────────────────────────
#ifdef EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#define BACKLIGHT_PIN       11
#define BACKLIGHT_LEDC_FREQ 5000
#define BACKLIGHT_LEDC_BITS 8
static uint8_t     backlight_brightness = 200;  // 0-255, loaded from prefs
static Preferences bl_prefs;
#endif

#define BACKLIGHT_TIMEOUT_MS 5000
static uint32_t backlight_last_touch_ms = 0;
static bool backlight_on = false;

static void backlight_on_touch() {
  if (!backlight_on) {
    #ifdef BACKLIGHT_PIN
    ledcWrite(BACKLIGHT_PIN, backlight_brightness);
    #endif
    backlight_on = true;
  }
  backlight_last_touch_ms = millis();
}

static void backlight_tick() {
  if (backlight_on && (millis() - backlight_last_touch_ms >= BACKLIGHT_TIMEOUT_MS)) {
    #ifdef BACKLIGHT_PIN
    ledcWrite(BACKLIGHT_PIN, 0);
    #endif
    backlight_on = false;
    esp_wifi_stop();
    Serial.println("sleeping");
    Serial.end();
    esp_light_sleep_start();
    Serial.begin(115200);
    Serial.println("woke up");
    esp_wifi_start();
    net.reinit();
  }
}

// ── Layout ────────────────────────────────────────────────────────────────────

static const int DISP_W = 960;
static const int DISP_H = 540;
static const int MARGIN = 10;
static const int GAP = 10;
static const int HEADER_H = 44;
static const int BOTTOM_H = 82;                            // increased 20% (was 72 → BOT_BTN_H 52 → 62)
static const int MIDDLE_H = DISP_H - HEADER_H - BOTTOM_H;  // 414

// Effect button grid (2 cols × 3 rows)
static const int EFF_COLS = 2;
static const int EFF_ROWS = 3;
static const int EFF_W = 170;
static const int EFF_H = (MIDDLE_H - 2 * MARGIN - (EFF_ROWS - 1) * GAP) / EFF_ROWS;  // 124

// Left panel total width (2 btn cols + gap + outer margins)
static const int LEFT_W = 2 * EFF_W + GAP + 2 * MARGIN;  // 370

// Vertical slider column
static const int SLDR_COL_W = 70;
static const int SLDR_W = 36;

// Preview panel
static const int PREV_X = LEFT_W + SLDR_COL_W;   // 440
static const int PREV_W = DISP_W - PREV_X - 20;  // 500

// Bottom row (7 menu buttons + 1 settings button = 8 total)
static const int BOT_BTNS = 8;
static const int BOT_BTN_W = (DISP_W - 2 * MARGIN - (BOT_BTNS - 1) * GAP) / BOT_BTNS;  // 108
static const int BOT_BTN_H = BOTTOM_H - 2 * MARGIN;                                    // 62



// ── Menu pages ────────────────────────────────────────────────────────────────
// 7 menu buttons, each showing a page of 6 effect buttons.

static const int MENU_COUNT = 7;

static const char *MENU_LABELS[MENU_COUNT] = {
  "FX-1",
  "FX-2",
  "FX-3",
  "FX-4",
  "FX-5",
  "FX-6",
  "FX-7",
};

// 6 effect labels per page.
// Pages 0-2 map to the 18 live effects on the receivers (indices 0-17).
// Pages 3-6 are reserved for future effects.
static const char *PAGE_EFFECTS[MENU_COUNT][6] = {
  { "BLOCK-1", "SINE\nWAVE", "BLOCK-2", "FIRE", "POW!", "COLOR\nWHEEL" },  // 0-5
  { "RAINBOW", "SPARKLE", "STROBE", "METEOR", "SPRINGS", "CIRCLES" },      // 6-11
  { "CHASER", "SOUND", "BLOOD", "UPDATE", "-", "-" },                      // 12-17
  { "FX-4A", "FX-4B", "FX-4C", "FX-4D", "FX-4E", "FX-4F" },                // 18-23
  { "FX-5A", "FX-5B", "FX-5C", "FX-5D", "FX-5E", "FX-5F" },                // 24-29
  { "FX-6A", "FX-6B", "FX-6C", "FX-6D", "FX-6E", "FX-6F" },                // 30-35
  { "FX-7A", "FX-7B", "FX-7C", "FX-7D", "FX-7E", "FX-7F" },                // 36-41
};

// ── State ─────────────────────────────────────────────────────────────────────

static int selected_menu = 0;
static int selected_effect = -1;  // -1 = none
static int brightness = 75;

// ── Widget handles ────────────────────────────────────────────────────────────

static lv_obj_t *effect_btns[6];
static lv_obj_t *effect_lbls[6];
static lv_obj_t *menu_btns[MENU_COUNT];
static lv_obj_t *header_time_lbl = nullptr;
static lv_obj_t *bat_lbl         = nullptr;
static lv_obj_t *chan_btns[3] = { nullptr, nullptr, nullptr };
static lv_obj_t *popup = nullptr;

// ── Helpers ───────────────────────────────────────────────────────────────────

static void apply_btn_style(lv_obj_t *btn, bool selected) {
  lv_obj_set_style_bg_color(btn,
                            selected ? EPD_PainterLVGL::LT_GREY : EPD_PainterLVGL::WHITE, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(btn, EPD_PainterLVGL::DK_GREY, 0);
  lv_obj_set_style_border_width(btn, 2, 0);
  lv_obj_set_style_radius(btn, 10, 0);
  lv_obj_set_style_pad_all(btn, 0, 0);
  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
}

// Create two overlapping labels on a button, offset by 1 px, to simulate bold.
// Child 0 = shadow (offset), child 1 = main label.
static lv_obj_t *make_bold_label(lv_obj_t *btn, const char *text) {
  lv_obj_t *shadow = lv_label_create(btn);
  lv_label_set_text(shadow, text);
  lv_obj_set_style_text_font(shadow, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(shadow, EPD_PainterLVGL::BLACK, 0);
  lv_obj_set_style_text_align(shadow, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(shadow, LV_ALIGN_CENTER, 1, 0);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, text);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(lbl, EPD_PainterLVGL::BLACK, 0);
  lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_center(lbl);
  return lbl;
}

static void load_menu_page(int page) {
  for (int i = 0; i < 6; i++) {
    const char *text = PAGE_EFFECTS[page][i];
    lv_label_set_text(lv_obj_get_child(effect_btns[i], 0), text);  // shadow
    lv_label_set_text(lv_obj_get_child(effect_btns[i], 1), text);  // main
    lv_obj_set_style_bg_color(effect_btns[i], EPD_PainterLVGL::WHITE, 0);
  }
  selected_effect = -1;
}

// ── Callbacks ─────────────────────────────────────────────────────────────────

static void effect_btn_cb(lv_event_t *e) {
  int idx = (int)(intptr_t)lv_event_get_user_data(e);
  selected_effect = idx;
  for (int i = 0; i < 6; i++)
    lv_obj_set_style_bg_color(effect_btns[i],
                              (i == idx) ? EPD_PainterLVGL::LT_GREY : EPD_PainterLVGL::WHITE, 0);
  net.sendEffectSelect((uint8_t)idx, (uint8_t)selected_menu);
}

static void menu_btn_cb(lv_event_t *e) {
  int idx = (int)(intptr_t)lv_event_get_user_data(e);
  selected_menu = idx;
  for (int i = 0; i < MENU_COUNT; i++)
    lv_obj_set_style_bg_color(menu_btns[i],
                              (i == idx) ? EPD_PainterLVGL::LT_GREY : EPD_PainterLVGL::WHITE, 0);
  load_menu_page(idx);
}

static void brightness_cb(lv_event_t *e) {
  brightness = (int)lv_slider_get_value(lv_event_get_target_obj(e));
  bool down = (lv_event_get_code(e) != LV_EVENT_RELEASED);
  net.sendSlider((uint8_t)(brightness * 255 / 100), down);
}

// ── Trackpad (preview panel) touch ────────────────────────────────────────────

static void preview_event_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_indev_t *indev = lv_indev_active();
  if (!indev) return;

  lv_point_t pt;
  lv_indev_get_point(indev, &pt);

  // Scale screen coordinates to 0-255 within the preview panel bounds
  int px = pt.x - PREV_X;
  int py = pt.y - (HEADER_H + MARGIN);
  uint8_t sx = (uint8_t)max(0, min(255, px * 255 / (int)PREV_W));
  uint8_t sy = (uint8_t)max(0, min(255, py * 255 / (MIDDLE_H - 2 * MARGIN)));

  if (code == LV_EVENT_PRESSED) {
    net.sendTapped();
    net.sendMove(sx, sy, true);
  } else if (code == LV_EVENT_PRESSING) {
    net.sendMove(sx, sy, true);
  } else if (code == LV_EVENT_RELEASED) {
    net.sendMove(sx, sy, false);
  }
}

// ── RTC helpers ───────────────────────────────────────────────────────────────

static void sync_epoch_from_rtc() {
  if (!rtc) return;
  I2C_BM8563_DateTypeDef d;
  I2C_BM8563_TimeTypeDef t;
  rtc->getDate(&d);
  rtc->getTime(&t);

  struct tm tm = {};
  tm.tm_year = d.year - 1900;
  tm.tm_mon = d.month;  // 0-indexed
  tm.tm_mday = d.date + 1;
  tm.tm_hour = t.hours;
  tm.tm_min = t.minutes;
  tm.tm_sec = t.seconds;
  tm.tm_isdst = 0;

  net.setEpoch((uint32_t)mktime(&tm));
}

static void update_header_time() {
  if (!header_time_lbl || !rtc) return;
  I2C_BM8563_DateTypeDef d;
  I2C_BM8563_TimeTypeDef t;
  rtc->getDate(&d);
  rtc->getTime(&t);

  char buf[32];
  lv_snprintf(buf, sizeof(buf), "%d:%02d  %d/%d/%d",
              t.hours, t.minutes,
              d.date + 1, d.month + 1, d.year);
  lv_label_set_text(header_time_lbl, buf);
}

// LVGL timer – fires every second to refresh the header clock
static void time_timer_cb(lv_timer_t *) {
  update_header_time();
}

// ── Settings popup ────────────────────────────────────────────────────────────

static void close_popup_cb(lv_event_t *) {
  if (popup) {
    lv_obj_del(popup);
    popup = nullptr;
    chan_btns[0] = chan_btns[1] = chan_btns[2] = nullptr;
    display.clear();
  }
}

static void wifi_chan_cb(lv_event_t *e) {
  uint8_t idx = (uint8_t)(intptr_t)lv_event_get_user_data(e);
  net.setWifiChannel(idx);
  for (int i = 0; i < 3; i++) {
    if (chan_btns[i])
      lv_obj_set_style_bg_color(chan_btns[i],
                                (i == idx) ? EPD_PainterLVGL::LT_GREY : EPD_PainterLVGL::WHITE, 0);
  }
}

#ifdef BACKLIGHT_PIN
static void backlight_brightness_cb(lv_event_t *e) {
  int val = (int)lv_slider_get_value(lv_event_get_target_obj(e));
  backlight_brightness = (uint8_t)(val * 255 / 100);
  if (backlight_on) ledcWrite(BACKLIGHT_PIN, backlight_brightness);
  if (lv_event_get_code(e) == LV_EVENT_RELEASED)
    bl_prefs.putUInt("brightness", backlight_brightness);
}
#endif

static void settings_btn_cb(lv_event_t *) {
  if (popup) return;

  lv_obj_t *scr = lv_screen_active();

  // Dimming overlay
  popup = lv_obj_create(scr);
  lv_obj_set_pos(popup, 0, 0);
  lv_obj_set_size(popup, DISP_W, DISP_H);
  lv_obj_set_style_bg_color(popup, EPD_PainterLVGL::DK_GREY, 0);
  lv_obj_set_style_bg_opa(popup, LV_OPA_50, 0);
  lv_obj_set_style_border_width(popup, 0, 0);
  lv_obj_set_style_radius(popup, 0, 0);
  lv_obj_clear_flag(popup, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(popup, close_popup_cb, LV_EVENT_CLICKED, NULL);

  // Card
  lv_obj_t *card = lv_obj_create(popup);
#ifdef BACKLIGHT_PIN
  lv_obj_set_size(card, 500, 400);
#else
  lv_obj_set_size(card, 500, 280);
#endif
  lv_obj_align(card, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(card, EPD_PainterLVGL::WHITE, 0);
  lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(card, EPD_PainterLVGL::BLACK, 0);
  lv_obj_set_style_border_width(card, 2, 0);
  lv_obj_set_style_radius(card, 10, 0);
  lv_obj_set_style_pad_all(card, 24, 0);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *title = lv_label_create(card);
  lv_label_set_text(title, "Settings");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(title, EPD_PainterLVGL::BLACK, 0);
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_t *chan_lbl = lv_label_create(card);
  lv_label_set_text(chan_lbl, "Wi-Fi Channel");
  lv_obj_set_style_text_font(chan_lbl, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(chan_lbl, EPD_PainterLVGL::DK_GREY, 0);
  lv_obj_align(chan_lbl, LV_ALIGN_TOP_LEFT, 0, 64);

  static const int CHAN_LABELS[3] = { 1, 6, 11 };
  uint8_t cur = net.getWifiChannelIndex();
  for (int i = 0; i < 3; i++) {
    lv_obj_t *btn = lv_obj_create(card);
    lv_obj_set_size(btn, 120, 60);
    lv_obj_set_pos(btn, i * 138, 102);
    lv_obj_set_style_bg_color(btn,
                              (i == (int)cur) ? EPD_PainterLVGL::LT_GREY : EPD_PainterLVGL::WHITE, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(btn, EPD_PainterLVGL::DK_GREY, 0);
    lv_obj_set_style_border_width(btn, 2, 0);
    lv_obj_set_style_radius(btn, 8, 0);
    lv_obj_set_style_pad_all(btn, 0, 0);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(btn, wifi_chan_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    chan_btns[i] = btn;

    lv_obj_t *lbl = lv_label_create(btn);
    char buf[4];
    lv_snprintf(buf, sizeof(buf), "%d", CHAN_LABELS[i]);
    lv_label_set_text(lbl, buf);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(lbl, EPD_PainterLVGL::BLACK, 0);
    lv_obj_center(lbl);
  }

#ifdef BACKLIGHT_PIN
  lv_obj_t *bl_lbl = lv_label_create(card);
  lv_label_set_text(bl_lbl, "Backlight");
  lv_obj_set_style_text_font(bl_lbl, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(bl_lbl, EPD_PainterLVGL::DK_GREY, 0);
  lv_obj_align(bl_lbl, LV_ALIGN_TOP_LEFT, 0, 182);

  lv_obj_t *bl_sldr = lv_slider_create(card);
  lv_obj_set_size(bl_sldr, 452, 52);
  lv_obj_align(bl_sldr, LV_ALIGN_TOP_LEFT, 0, 220);
  lv_slider_set_range(bl_sldr, 5, 100);
  lv_slider_set_value(bl_sldr, (int)(backlight_brightness * 100 / 255), LV_ANIM_OFF);
  lv_obj_set_style_bg_color(bl_sldr, EPD_PainterLVGL::LT_GREY, LV_PART_MAIN);
  lv_obj_set_style_border_color(bl_sldr, EPD_PainterLVGL::DK_GREY, LV_PART_MAIN);
  lv_obj_set_style_border_width(bl_sldr, 2, LV_PART_MAIN);
  lv_obj_set_style_radius(bl_sldr, 6, LV_PART_MAIN);
  lv_obj_set_style_bg_color(bl_sldr, EPD_PainterLVGL::DK_GREY, LV_PART_INDICATOR);
  lv_obj_set_style_radius(bl_sldr, 6, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(bl_sldr, EPD_PainterLVGL::WHITE, LV_PART_KNOB);
  lv_obj_set_style_radius(bl_sldr, 4, LV_PART_KNOB);
  lv_obj_add_event_cb(bl_sldr, backlight_brightness_cb, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(bl_sldr, backlight_brightness_cb, LV_EVENT_RELEASED, NULL);
#endif

  lv_obj_t *close_btn = lv_button_create(card);
  lv_obj_set_size(close_btn, 130, 50);
  lv_obj_align(close_btn, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_style_bg_color(close_btn, EPD_PainterLVGL::BLACK, 0);
  lv_obj_set_style_radius(close_btn, 6, 0);
  lv_obj_add_event_cb(close_btn, close_popup_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *close_lbl = lv_label_create(close_btn);
  lv_label_set_text(close_lbl, "Close");
  lv_obj_set_style_text_font(close_lbl, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(close_lbl, EPD_PainterLVGL::WHITE, 0);
  lv_obj_center(close_lbl);
}

// ── Shutdown popup ────────────────────────────────────────────────────────────

static void shutdown_confirm_cb(lv_event_t *) {
  if (popup) {
    lv_obj_del(popup);
    popup = nullptr;
  }
  psd->proceed();  // show shutdown image and power off
}

static void shutdown_cancel_cb(lv_event_t *) {
  if (popup) {
    lv_obj_del(popup);
    popup = nullptr;
  }
  psd->cancel();  // re-arm for next reset, continue running
  display.clear();
}

static void show_shutdown_popup() {
  if (popup) return;

  lv_obj_t *scr = lv_screen_active();

  // Dimming overlay (taps outside the card cancel shutdown)
  popup = lv_obj_create(scr);
  lv_obj_set_pos(popup, 0, 0);
  lv_obj_set_size(popup, DISP_W, DISP_H);
  lv_obj_set_style_bg_color(popup, EPD_PainterLVGL::DK_GREY, 0);
  lv_obj_set_style_bg_opa(popup, LV_OPA_50, 0);
  lv_obj_set_style_border_width(popup, 0, 0);
  lv_obj_set_style_radius(popup, 0, 0);
  lv_obj_clear_flag(popup, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(popup, shutdown_cancel_cb, LV_EVENT_CLICKED, NULL);

  // Card
  lv_obj_t *card = lv_obj_create(popup);
  lv_obj_set_size(card, 460, 220);
  lv_obj_align(card, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(card, EPD_PainterLVGL::WHITE, 0);
  lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(card, EPD_PainterLVGL::BLACK, 0);
  lv_obj_set_style_border_width(card, 2, 0);
  lv_obj_set_style_radius(card, 10, 0);
  lv_obj_set_style_pad_all(card, 24, 0);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_CLICKABLE);  // don't propagate to overlay

  lv_obj_t *title = lv_label_create(card);
  lv_label_set_text(title, "Shut down?");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(title, EPD_PainterLVGL::BLACK, 0);
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 0);

  // Cancel button
  lv_obj_t *cancel_btn = lv_obj_create(card);
  lv_obj_set_size(cancel_btn, 160, 56);
  lv_obj_align(cancel_btn, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_set_style_bg_color(cancel_btn, EPD_PainterLVGL::WHITE, 0);
  lv_obj_set_style_bg_opa(cancel_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(cancel_btn, EPD_PainterLVGL::DK_GREY, 0);
  lv_obj_set_style_border_width(cancel_btn, 2, 0);
  lv_obj_set_style_radius(cancel_btn, 6, 0);
  lv_obj_set_style_pad_all(cancel_btn, 0, 0);
  lv_obj_clear_flag(cancel_btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(cancel_btn, shutdown_cancel_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *cancel_lbl = lv_label_create(cancel_btn);
  lv_label_set_text(cancel_lbl, "Cancel");
  lv_obj_set_style_text_font(cancel_lbl, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(cancel_lbl, EPD_PainterLVGL::BLACK, 0);
  lv_obj_center(cancel_lbl);

  // Confirm button
  lv_obj_t *confirm_btn = lv_obj_create(card);
  lv_obj_set_size(confirm_btn, 160, 56);
  lv_obj_align(confirm_btn, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_style_bg_color(confirm_btn, EPD_PainterLVGL::BLACK, 0);
  lv_obj_set_style_bg_opa(confirm_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(confirm_btn, 0, 0);
  lv_obj_set_style_radius(confirm_btn, 6, 0);
  lv_obj_set_style_pad_all(confirm_btn, 0, 0);
  lv_obj_clear_flag(confirm_btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(confirm_btn, shutdown_confirm_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *confirm_lbl = lv_label_create(confirm_btn);
  lv_label_set_text(confirm_lbl, "Shut down");
  lv_obj_set_style_text_font(confirm_lbl, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(confirm_lbl, EPD_PainterLVGL::WHITE, 0);
  lv_obj_center(confirm_lbl);
}

// ── Build UI ──────────────────────────────────────────────────────────────────

static void build_ui() {
  lv_obj_t *scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, EPD_PainterLVGL::BLACK, 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // ── Header ────────────────────────────────────────────────────────────────
  lv_obj_t *hdr = lv_obj_create(scr);
  lv_obj_set_pos(hdr, 0, 0);
  lv_obj_set_size(hdr, DISP_W, HEADER_H);
  lv_obj_set_style_bg_color(hdr, EPD_PainterLVGL::WHITE, 0);
  lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(hdr, 0, 0);
  lv_obj_set_style_border_width(hdr, 0, 0);
  lv_obj_set_style_pad_hor(hdr, MARGIN, 0);
  lv_obj_set_style_pad_ver(hdr, 0, 0);
  lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

  // Time + date (left) – updated each second by time_timer_cb
  header_time_lbl = lv_label_create(hdr);
  lv_label_set_text(header_time_lbl, "--:--");
  lv_obj_set_style_text_font(header_time_lbl, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(header_time_lbl, EPD_PainterLVGL::BLACK, 0);
  lv_obj_align(header_time_lbl, LV_ALIGN_LEFT_MID, 0, 0);

  // Battery % + icon (right) – populated by battery_update() / battery_timer_cb()
  bat_lbl = lv_label_create(hdr);
  lv_label_set_text(bat_lbl, "--%  " LV_SYMBOL_BATTERY_FULL);
  lv_obj_set_style_text_font(bat_lbl, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(bat_lbl, EPD_PainterLVGL::BLACK, 0);
  lv_obj_align(bat_lbl, LV_ALIGN_RIGHT_MID, -20, 0);

  // ── Effect buttons (2×3 grid) ─────────────────────────────────────────────
  for (int i = 0; i < 6; i++) {
    int col = i % EFF_COLS;
    int row = i / EFF_COLS;
    int x = MARGIN + col * (EFF_W + GAP);
    int y = HEADER_H + MARGIN + row * (EFF_H + GAP);

    lv_obj_t *btn = lv_obj_create(scr);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, EFF_W, EFF_H);
    apply_btn_style(btn, false);
    lv_obj_add_event_cb(btn, effect_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    effect_btns[i] = btn;

    effect_lbls[i] = make_bold_label(btn, PAGE_EFFECTS[0][i]);
  }

  // ── Vertical brightness slider ─────────────────────────────────────────────
  // Centre between the right edge of the button grid and the left edge of the preview panel.
  int buttons_right = MARGIN + EFF_COLS * EFF_W + (EFF_COLS - 1) * GAP;  // 360
  int sldr_x = buttons_right + (PREV_X - buttons_right - SLDR_W) / 2;    // 382
  int sldr_y = HEADER_H + MARGIN;
  int sldr_h = MIDDLE_H - 2 * MARGIN;

  lv_obj_t *sldr = lv_slider_create(scr);
  lv_obj_set_pos(sldr, sldr_x, sldr_y);
  lv_obj_set_size(sldr, SLDR_W, sldr_h);
  lv_slider_set_range(sldr, 0, 100);
  lv_slider_set_value(sldr, brightness, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(sldr, EPD_PainterLVGL::DK_GREY, LV_PART_MAIN);
  lv_obj_set_style_border_color(sldr, EPD_PainterLVGL::LT_GREY, LV_PART_MAIN);
  lv_obj_set_style_border_width(sldr, 2, LV_PART_MAIN);
  lv_obj_set_style_radius(sldr, 6, LV_PART_MAIN);
  lv_obj_set_style_bg_color(sldr, EPD_PainterLVGL::LT_GREY, LV_PART_INDICATOR);
  lv_obj_set_style_radius(sldr, 6, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(sldr, EPD_PainterLVGL::WHITE, LV_PART_KNOB);
  lv_obj_set_style_radius(sldr, 4, LV_PART_KNOB);
  lv_obj_add_event_cb(sldr, brightness_cb, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(sldr, brightness_cb, LV_EVENT_RELEASED, NULL);

  // ── Preview panel ──────────────────────────────────────────────────────────
  lv_obj_t *preview = lv_obj_create(scr);
  lv_obj_set_pos(preview, PREV_X, HEADER_H + MARGIN);
  lv_obj_set_size(preview, PREV_W, MIDDLE_H - 2 * MARGIN);
  lv_obj_set_style_bg_color(preview, EPD_PainterLVGL::WHITE, 0);
  lv_obj_set_style_bg_opa(preview, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(preview, EPD_PainterLVGL::DK_GREY, 0);
  lv_obj_set_style_border_width(preview, 2, 0);
  lv_obj_set_style_radius(preview, 10, 0);
  lv_obj_set_style_pad_all(preview, 0, 0);
  lv_obj_clear_flag(preview, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(preview, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(preview, preview_event_cb, LV_EVENT_PRESSED, NULL);
  lv_obj_add_event_cb(preview, preview_event_cb, LV_EVENT_PRESSING, NULL);
  lv_obj_add_event_cb(preview, preview_event_cb, LV_EVENT_RELEASED, NULL);

  // ── Bottom menu row ────────────────────────────────────────────────────────
  int bot_y = HEADER_H + MIDDLE_H + MARGIN;

  for (int i = 0; i < MENU_COUNT; i++) {
    int x = MARGIN + i * (BOT_BTN_W + GAP);

    lv_obj_t *btn = lv_obj_create(scr);
    lv_obj_set_pos(btn, x, bot_y);
    lv_obj_set_size(btn, BOT_BTN_W, BOT_BTN_H);
    apply_btn_style(btn, i == selected_menu);
    lv_obj_add_event_cb(btn, menu_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    menu_btns[i] = btn;

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, MENU_LABELS[i]);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(lbl, EPD_PainterLVGL::BLACK, 0);
    lv_obj_center(lbl);
  }

  // ── Settings gear button (8th slot) ───────────────────────────────────────
  int gear_x = MARGIN + MENU_COUNT * (BOT_BTN_W + GAP);

  lv_obj_t *gear_btn = lv_obj_create(scr);
  lv_obj_set_pos(gear_btn, gear_x, bot_y);
  lv_obj_set_size(gear_btn, BOT_BTN_W, BOT_BTN_H);
  apply_btn_style(gear_btn, false);
  lv_obj_add_event_cb(gear_btn, settings_btn_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *gear_lbl = lv_label_create(gear_btn);
  lv_label_set_text(gear_lbl, LV_SYMBOL_SETTINGS);
  lv_obj_set_style_text_font(gear_lbl, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(gear_lbl, EPD_PainterLVGL::BLACK, 0);
  lv_obj_center(gear_lbl);

  lv_screen_load(scr);
}

// ── Touch ─────────────────────────────────────────────────────────────────────

static void touch_read_cb(lv_indev_t *, lv_indev_data_t *data) {
  tc.read();
  if (tc.down) {
  if (psd) psd->resetIdleTimer();
  backlight_on_touch();
  }
 
  data->point.x =  tc.y;
  data->point.y =  display.getConfig().height-tc.x;


  data->state = tc.down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;

}

// ── Setup / loop ──────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  lv_init();
  lv_tick_set_cb(my_tick_cb);

#ifdef EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
  bl_prefs.begin("backlight", false);
  backlight_brightness = (uint8_t)bl_prefs.getUInt("brightness", 200);
  ledcAttach(BACKLIGHT_PIN, BACKLIGHT_LEDC_FREQ, BACKLIGHT_LEDC_BITS);
  gpio_wakeup_enable((gpio_num_t) 3, GPIO_INTR_LOW_LEVEL);  // touch interrupt wakes device
#endif

#ifdef EPD_PAINTER_PRESET_M5PAPER_S3
    gpio_wakeup_enable((gpio_num_t) 48, GPIO_INTR_LOW_LEVEL); // pin 48 = touch interupt. Wakes the device up if touched.
#endif




  backlight_on_touch();

  display.setAutoShutdown(false);  // handle shutdown ourselves with a popup
  if (!display.begin()) {
    Serial.println("Display init failed!");
    while (1) delay(1000);
  }

  if (display.getConfig().i2c.wire != nullptr) {
    tc.begin(display.getConfig().i2c.wire);
    static I2C_BM8563 rtc_obj(I2C_BM8563_DEFAULT_ADDRESS, *display.getConfig().i2c.wire);
    rtc = &rtc_obj;
    rtc->begin();
  }

  esp_sleep_enable_gpio_wakeup();



  psd = display.shutdown();  // use the instance created by begin()
  psd->setPreShutdownCallback([]() {
    net.sendTurnOff();
    delay(200);  // allow the ESP-NOW broadcast to go out before power-off
  });
  psd->startIdleTimer(3600);  // auto power-off after 1 hour idle

  //display.setQuality(EPD_Painter::Quality::QUALITY_HIGH);

  if (!(psd->isPending())) display.clear();

  net.begin();
  sync_epoch_from_rtc();

  build_ui();
  update_header_time();                           // show correct time immediately
  lv_timer_create(time_timer_cb, 1000, nullptr);  // refresh every second

#ifdef EPD_PAINTER_PRESET_M5PAPER_S3
  battery_begin_adc(4, 2.0f);  // GPIO 4, 100k/100k voltage divider
#elif defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS)
  if (display.getConfig().i2c.wire)
    battery_begin_bq25896(display.getConfig().i2c.wire);
#endif
  battery_set_label(bat_lbl);
  battery_update();                                // show real value immediately
  lv_timer_create(battery_timer_cb, 60000, nullptr);  // refresh every 60 s

  lv_indev_t *touch = lv_indev_create();
  lv_indev_set_type(touch, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(touch, touch_read_cb);
}

void loop() {
  if (psd && psd->isPending() && !popup)
    show_shutdown_popup();

  lv_timer_handler();
  backlight_tick();

  delay(5);
}
