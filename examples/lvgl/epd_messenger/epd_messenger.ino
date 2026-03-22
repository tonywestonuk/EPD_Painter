// EPD Messenger – peer-to-peer text messaging over ESP-NOW.
// Flash the same sketch on two devices; they discover each other automatically.
//
// Select your board (comment out the other):
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include <lvgl.h>
#include "EPD_Painter_presets.h"
#include "EPD_Painter_LVGL.h"
#include <gt911_lite.h>
#include "MessengerNetwork.h"

// ── Backlight (LilyGo only, pin 11) ──────────────────────────────────────────
#ifdef EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
  #define BACKLIGHT_PIN        11
  #define BACKLIGHT_TIMEOUT_MS 10000
  static uint32_t backlight_last_touch_ms = 0;
  static bool     backlight_on            = false;

  static void backlight_on_touch() {
      if (!backlight_on) { digitalWrite(BACKLIGHT_PIN, HIGH); backlight_on = true; }
      backlight_last_touch_ms = millis();
  }
  static void backlight_tick() {
      if (backlight_on && millis() - backlight_last_touch_ms >= BACKLIGHT_TIMEOUT_MS) {
          digitalWrite(BACKLIGHT_PIN, LOW); backlight_on = false;
      }
  }
#endif

// ── Globals ───────────────────────────────────────────────────────────────────

static uint32_t my_tick_cb() { return millis(); }

EPD_PainterLVGL display(EPD_PAINTER_PRESET);
GT911_Lite      tc;
static MessengerNetwork net;

// ── Message store ─────────────────────────────────────────────────────────────

static const int MAX_MESSAGES = 30;
static MessengerMessage g_messages[MAX_MESSAGES];
static int g_msg_count = 0;

// Thread-safe pending inbox (written from Wi-Fi task, drained in loop)
static portMUX_TYPE      g_mux        = portMUX_INITIALIZER_UNLOCKED;
static volatile bool     g_has_inbox  = false;
static MessengerMessage  g_inbox;

static void push_message(const char *text, bool is_mine, const uint8_t *mac) {
    if (g_msg_count >= MAX_MESSAGES) {
        memmove(g_messages, g_messages + 1,
                (MAX_MESSAGES - 1) * sizeof(MessengerMessage));
        g_msg_count = MAX_MESSAGES - 1;
    }
    MessengerMessage &m = g_messages[g_msg_count++];
    memcpy(m.sender_mac, mac, 6);
    strncpy(m.text, text, MESSENGER_MAX_TEXT);
    m.text[MESSENGER_MAX_TEXT] = '\0';
    m.is_mine = is_mine;
}

// ── Layout constants ──────────────────────────────────────────────────────────

static const int DISP_W   = 960;
static const int DISP_H   = 540;
static const int HEADER_H = 44;
static const int COMPOSE_H = 52;    // bottom compose bar
static const int MSG_AREA_H = DISP_H - HEADER_H - COMPOSE_H;  // 444

// Compose popup
static const int POPUP_TB_H  = 100;
static const int POPUP_KBD_H = 300;                            // fixed — ~70px per key row
static const int POPUP_KBD_Y = 0;         // 240 — keyboard anchored to bottom
static const int POPUP_TA_H  = 130;  // 188 — text area fills the gap

// ── Widget handles ────────────────────────────────────────────────────────────

static lv_obj_t *msg_container  = nullptr;  // scrollable message list
static lv_obj_t *compose_popup  = nullptr;
static lv_obj_t *compose_ta     = nullptr;
static lv_obj_t *compose_kbd    = nullptr;
static lv_obj_t *hdr_id_lbl     = nullptr;  // own device ID in header

// ── Bubble rendering ──────────────────────────────────────────────────────────

static void rebuild_messages() {
    if (!msg_container) return;
    lv_obj_clean(msg_container);

    if (g_msg_count == 0) {
        lv_obj_t *hint = lv_label_create(msg_container);
        lv_label_set_text(hint, "No messages yet.\nTap  Compose  to write one.");
        lv_obj_set_style_text_font(hint, &lv_font_montserrat_32, 0);
        lv_obj_set_style_text_color(hint, EPD_PainterLVGL::DK_GREY, 0);
        lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_width(hint, DISP_W - 40);
        lv_obj_center(hint);
        return;
    }

    lv_obj_t *last_row = nullptr;

    for (int i = 0; i < g_msg_count; i++) {
        const MessengerMessage &m = g_messages[i];

        // Full-width transparent row – flex child of msg_container
        lv_obj_t *row = lv_obj_create(msg_container);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_hor(row, 10, 0);
        lv_obj_set_style_pad_ver(row, 4, 0);
        lv_obj_set_style_pad_gap(row, 0, 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        // Row is itself a flex container so we can justify the bubble
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row,
            m.is_mine ? LV_FLEX_ALIGN_END : LV_FLEX_ALIGN_START,
            LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

        // Bubble
        lv_obj_t *bubble = lv_obj_create(row);
        lv_obj_set_width(bubble, 650);
        lv_obj_set_height(bubble, LV_SIZE_CONTENT);
        lv_obj_set_style_pad_all(bubble, 14, 0);
        lv_obj_set_style_radius(bubble, 12, 0);
        lv_obj_set_style_border_width(bubble, 2, 0);
        lv_obj_set_style_border_color(bubble, EPD_PainterLVGL::DK_GREY, 0);
        lv_obj_clear_flag(bubble, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_flex_flow(bubble, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(bubble, 4, 0);

        if (m.is_mine) {
            lv_obj_set_style_bg_color(bubble, EPD_PainterLVGL::WHITE, 0);
            lv_obj_set_style_bg_opa(bubble, LV_OPA_COVER, 0);
        } else {
            lv_obj_set_style_bg_color(bubble, EPD_PainterLVGL::WHITE, 0);
            lv_obj_set_style_bg_opa(bubble, LV_OPA_COVER, 0);

            // "From: XX:XX:XX" header in received bubbles
            char from_str[32];
            const uint8_t *mac = m.sender_mac;
            lv_snprintf(from_str, sizeof(from_str),
                        "From: %02X:%02X:%02X", mac[3], mac[4], mac[5]);
            lv_obj_t *from_lbl = lv_label_create(bubble);
            lv_label_set_text(from_lbl, from_str);
            lv_obj_set_width(from_lbl, lv_pct(100));
            lv_obj_set_style_text_font(from_lbl, &lv_font_montserrat_28, 0);
            lv_obj_set_style_text_color(from_lbl, EPD_PainterLVGL::DK_GREY, 0);
        }

        // Message body
        lv_obj_t *txt_lbl = lv_label_create(bubble);
        lv_label_set_text(txt_lbl, m.text);
        lv_obj_set_width(txt_lbl, lv_pct(100));
        lv_obj_set_style_text_font(txt_lbl, &lv_font_montserrat_32, 0);
        lv_obj_set_style_text_color(txt_lbl, EPD_PainterLVGL::BLACK, 0);
        lv_label_set_long_mode(txt_lbl, LV_LABEL_LONG_WRAP);

        last_row = row;
    }

    if (last_row) lv_obj_scroll_to_view(last_row, LV_ANIM_OFF);
}

// ── Compose popup ─────────────────────────────────────────────────────────────

static void close_compose() {
    if (compose_popup) {
        lv_obj_del(compose_popup);
        compose_popup = compose_ta = compose_kbd = nullptr;
    }
}

static void do_send() {
    if (!compose_ta) return;
    const char *txt = lv_textarea_get_text(compose_ta);
    if (strlen(txt) == 0) return;

    uint8_t own_mac[6];
    net.getOwnMac(own_mac);
    push_message(txt, true, own_mac);
    net.send(txt);
    close_compose();
    rebuild_messages();
    display.clear();
}

static void compose_kbd_cb(lv_event_t *) {
    do_send();  // only registered for LV_EVENT_READY (Enter key)
}
static void compose_send_cb(lv_event_t *)   { do_send(); }
static void compose_cancel_cb(lv_event_t *) { close_compose(); }

static void open_compose() {
    if (compose_popup) return;
    display.clear();
    lv_obj_t *scr = lv_screen_active();

    // Full-screen overlay
    compose_popup = lv_obj_create(scr);
    lv_obj_set_pos(compose_popup, 0, 0);
    lv_obj_set_size(compose_popup, DISP_W, DISP_H);
    lv_obj_set_style_bg_color(compose_popup, EPD_PainterLVGL::WHITE, 0);
    lv_obj_set_style_bg_opa(compose_popup, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(compose_popup, 0, 0);
    lv_obj_set_style_radius(compose_popup, 0, 0);
    lv_obj_set_style_pad_all(compose_popup, 0, 0);
    lv_obj_clear_flag(compose_popup, LV_OBJ_FLAG_SCROLLABLE);

    // ── Title bar ─────────────────────────────────────────────────────────────
    lv_obj_t *tb = lv_obj_create(compose_popup);
    lv_obj_set_pos(tb, 0, 0);
    lv_obj_set_size(tb, DISP_W, POPUP_TB_H);
    lv_obj_set_style_bg_color(tb, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_bg_opa(tb, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(tb, 0, 0);
    lv_obj_set_style_radius(tb, 0, 0);
    lv_obj_set_style_pad_hor(tb, 14, 0);
    lv_obj_set_style_pad_ver(tb, 0, 0);
    lv_obj_clear_flag(tb, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title_lbl = lv_label_create(tb);
    lv_label_set_text(title_lbl, "New Message");
    lv_obj_set_style_text_font(title_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(title_lbl, EPD_PainterLVGL::WHITE, 0);
    lv_obj_align(title_lbl, LV_ALIGN_LEFT_MID, 0, 0);

    // Cancel button
    lv_obj_t *cancel_btn = lv_obj_create(tb);
    lv_obj_set_size(cancel_btn, 130, 44);
    lv_obj_align(cancel_btn, LV_ALIGN_RIGHT_MID, -150, 0);
    lv_obj_set_style_bg_color(cancel_btn, EPD_PainterLVGL::DK_GREY, 0);
    lv_obj_set_style_bg_opa(cancel_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(cancel_btn, 0, 0);
    lv_obj_set_style_radius(cancel_btn, 6, 0);
    lv_obj_set_style_pad_all(cancel_btn, 0, 0);
    lv_obj_clear_flag(cancel_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(cancel_btn, compose_cancel_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn);
    lv_label_set_text(cancel_lbl, "Cancel");
    lv_obj_set_style_text_font(cancel_lbl, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(cancel_lbl, EPD_PainterLVGL::WHITE, 0);
    lv_obj_center(cancel_lbl);

    // Send button
    lv_obj_t *send_btn = lv_obj_create(tb);
    lv_obj_set_size(send_btn, 130, 44);
    lv_obj_align(send_btn, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_style_bg_color(send_btn, EPD_PainterLVGL::WHITE, 0);
    lv_obj_set_style_bg_opa(send_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(send_btn, 0, 0);
    lv_obj_set_style_radius(send_btn, 6, 0);
    lv_obj_set_style_pad_all(send_btn, 0, 0);
    lv_obj_clear_flag(send_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(send_btn, compose_send_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *send_lbl = lv_label_create(send_btn);
    lv_label_set_text(send_lbl, "Send");
    lv_obj_set_style_text_font(send_lbl, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(send_lbl, EPD_PainterLVGL::BLACK, 0);
    lv_obj_center(send_lbl);

    // ── Text area ─────────────────────────────────────────────────────────────
    compose_ta = lv_textarea_create(compose_popup);
    lv_obj_set_pos(compose_ta, 10, POPUP_TB_H + 8);
    lv_obj_set_size(compose_ta, DISP_W - 20, POPUP_TA_H);
    lv_textarea_set_max_length(compose_ta, MESSENGER_MAX_TEXT);
    lv_textarea_set_one_line(compose_ta, false);
    lv_obj_set_style_text_font(compose_ta, &lv_font_montserrat_32, 0);
    lv_obj_set_style_border_color(compose_ta, EPD_PainterLVGL::DK_GREY, 0);
    lv_obj_set_style_border_width(compose_ta, 2, 0);
    lv_obj_set_style_radius(compose_ta, 6, 0);
    lv_obj_set_style_bg_color(compose_ta, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_bg_opa(compose_ta, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(compose_ta, EPD_PainterLVGL::WHITE, 0);

    // ── Keyboard ──────────────────────────────────────────────────────────────
    compose_kbd = lv_keyboard_create(compose_popup);
    lv_obj_set_pos(compose_kbd, 0, POPUP_KBD_Y);
    lv_obj_set_size(compose_kbd, DISP_W, POPUP_KBD_H);
    lv_keyboard_set_textarea(compose_kbd, compose_ta);
    // Dark keys for e-paper contrast — font + colour both set on LV_PART_ITEMS
    // so our styles take priority over the default theme
    lv_obj_set_style_bg_color(compose_kbd, EPD_PainterLVGL::WHITE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(compose_kbd, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(compose_kbd, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(compose_kbd, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_row(compose_kbd, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_column(compose_kbd, 4, LV_PART_MAIN);

    lv_obj_set_style_bg_color(compose_kbd, EPD_PainterLVGL::DK_GREY, LV_PART_ITEMS);
    lv_obj_set_style_bg_opa(compose_kbd, LV_OPA_COVER, LV_PART_ITEMS);
    lv_obj_set_style_text_font(compose_kbd, &lv_font_montserrat_32, LV_PART_ITEMS);
    lv_obj_set_style_text_color(compose_kbd, EPD_PainterLVGL::WHITE, LV_PART_ITEMS);
    lv_obj_set_style_border_width(compose_kbd, 0, LV_PART_ITEMS);
    lv_obj_set_style_radius(compose_kbd, 8, LV_PART_ITEMS);

    // Checked state (e.g. ABC / shift toggled)
    lv_obj_set_style_bg_color(compose_kbd, EPD_PainterLVGL::BLACK, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_font(compose_kbd, &lv_font_montserrat_32, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(compose_kbd, EPD_PainterLVGL::WHITE, LV_PART_ITEMS | LV_STATE_CHECKED);

    lv_obj_add_event_cb(compose_kbd, compose_kbd_cb, LV_EVENT_READY, NULL);
    // LV_EVENT_CANCEL is NOT handled here — the keyboard's built-in hide button
    // (LV_SYMBOL_KEYBOARD) fires it, which would wrongly close the compose screen.
    // Use the Cancel button in the title bar instead.
}

// ── Build main UI ─────────────────────────────────────────────────────────────

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
    lv_obj_set_style_pad_hor(hdr, 14, 0);
    lv_obj_set_style_pad_ver(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(hdr);
    lv_label_set_text(title, "EPD Messenger");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(title, EPD_PainterLVGL::BLACK, 0);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 0, 0);

    // Show own device ID (last 3 bytes of MAC) so users can tell devices apart
    hdr_id_lbl = lv_label_create(hdr);
    lv_obj_set_style_text_font(hdr_id_lbl, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(hdr_id_lbl, EPD_PainterLVGL::DK_GREY, 0);
    lv_obj_align(hdr_id_lbl, LV_ALIGN_RIGHT_MID, 0, 0);

    uint8_t mac[6];
    net.getOwnMac(mac);
    char id_str[24];
    lv_snprintf(id_str, sizeof(id_str), "Me: %02X:%02X:%02X", mac[3], mac[4], mac[5]);
    lv_label_set_text(hdr_id_lbl, id_str);

    // ── Message scroll area ───────────────────────────────────────────────────
    msg_container = lv_obj_create(scr);
    lv_obj_set_pos(msg_container, 0, HEADER_H);
    lv_obj_set_size(msg_container, DISP_W, MSG_AREA_H);
    lv_obj_set_style_bg_color(msg_container, EPD_PainterLVGL::WHITE, 0);
    lv_obj_set_style_bg_opa(msg_container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(msg_container, 0, 0);
    lv_obj_set_style_radius(msg_container, 0, 0);
    lv_obj_set_style_pad_all(msg_container, 6, 0);
    lv_obj_set_style_pad_row(msg_container, 8, 0);
    lv_obj_set_scroll_dir(msg_container, LV_DIR_VER);
    lv_obj_set_flex_flow(msg_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(msg_container,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);

    // ── Compose bar (bottom) ──────────────────────────────────────────────────
    int compose_y = HEADER_H + MSG_AREA_H;

    static const int CLEAR_W = 160;

    // Compose button (fills bar except for the Clear button on the right)
    lv_obj_t *compose_bar = lv_obj_create(scr);
    lv_obj_set_pos(compose_bar, 0, compose_y);
    lv_obj_set_size(compose_bar, DISP_W - CLEAR_W, COMPOSE_H);
    lv_obj_set_style_bg_color(compose_bar, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_bg_opa(compose_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(compose_bar, 0, 0);
    lv_obj_set_style_radius(compose_bar, 0, 0);
    lv_obj_set_style_pad_all(compose_bar, 6, 0);
    lv_obj_clear_flag(compose_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(compose_bar, [](lv_event_t *) { open_compose(); },
                        LV_EVENT_CLICKED, NULL);

    lv_obj_t *compose_lbl = lv_label_create(compose_bar);
    lv_label_set_text(compose_lbl, LV_SYMBOL_EDIT "  Compose");
    lv_obj_set_style_text_font(compose_lbl, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(compose_lbl, EPD_PainterLVGL::WHITE, 0);
    lv_obj_center(compose_lbl);

    // Clear button
    lv_obj_t *clear_bar = lv_obj_create(scr);
    lv_obj_set_pos(clear_bar, DISP_W - CLEAR_W, compose_y);
    lv_obj_set_size(clear_bar, CLEAR_W, COMPOSE_H);
    lv_obj_set_style_bg_color(clear_bar, EPD_PainterLVGL::DK_GREY, 0);
    lv_obj_set_style_bg_opa(clear_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(clear_bar, 0, 0);
    lv_obj_set_style_radius(clear_bar, 0, 0);
    lv_obj_set_style_pad_all(clear_bar, 6, 0);
    lv_obj_clear_flag(clear_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(clear_bar, [](lv_event_t *) {
        g_msg_count = 0;
        rebuild_messages();
        display.clear();
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t *clear_lbl = lv_label_create(clear_bar);
    lv_label_set_text(clear_lbl, LV_SYMBOL_TRASH "  Clear");
    lv_obj_set_style_text_font(clear_lbl, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(clear_lbl, EPD_PainterLVGL::WHITE, 0);
    lv_obj_center(clear_lbl);

    lv_screen_load(scr);
    rebuild_messages();
}

// ── Touch ─────────────────────────────────────────────────────────────────────

static void touch_read_cb(lv_indev_t *, lv_indev_data_t *data) {
    tc.read();
#ifdef EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
    if (tc.down) backlight_on_touch();
#endif
  data->point.x =  tc.y;
  data->point.y =  display.getConfig().height-tc.x;
    data->state   = tc.down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

// ── Setup / loop ──────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

#ifdef EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
    pinMode(BACKLIGHT_PIN, OUTPUT);
    digitalWrite(BACKLIGHT_PIN, LOW);
#endif

    lv_init();
    lv_tick_set_cb(my_tick_cb);

    if (!display.begin()) {
        Serial.println("Display init failed!");
        while (1) delay(1000);
    }
    display.setQuality(EPD_Painter::Quality::QUALITY_HIGH);  // snappier for messaging

    if (display.getConfig().i2c.wire != nullptr) {
        tc.begin(display.getConfig().i2c.wire);
    }

    if (!net.begin(1)) {
        Serial.println("Network init failed!");
        while (1) delay(1000);
    }

    // ESP-NOW receive callback — runs from Wi-Fi task, must be thread-safe
    net.onReceive([](const MessengerMessage &m) {
        portENTER_CRITICAL(&g_mux);
        if (!g_has_inbox) {
            g_inbox     = m;
            g_has_inbox = true;
        }
        portEXIT_CRITICAL(&g_mux);
    });

    display.clear();
    build_ui();

    lv_indev_t *touch = lv_indev_create();
    lv_indev_set_type(touch, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touch, touch_read_cb);
}

void loop() {
    // Drain inbox from Wi-Fi task
    MessengerMessage m;
    bool got_msg = false;
    portENTER_CRITICAL(&g_mux);
    if (g_has_inbox) {
        m           = g_inbox;
        g_has_inbox = false;
        got_msg     = true;
    }
    portEXIT_CRITICAL(&g_mux);

    if (got_msg) {
        push_message(m.text, false, m.sender_mac);
        rebuild_messages();
        display.clear();
    }

    lv_timer_handler();
#ifdef EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
    backlight_tick();
#endif
    delay(5);
}
