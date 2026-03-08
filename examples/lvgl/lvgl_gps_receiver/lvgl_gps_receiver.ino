// GPS Receiver GUI – TinyGPSPlus + LVGL display for EPD_Painter.
// Board: LilyGo T5 S3 GPS

#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_M5PAPER_S3

#include <Arduino.h>
#include <lvgl.h>
#include <math.h>
#include <TinyGPSPlus.h>
#include "EPD_Painter_presets.h"
#include "EPD_Painter_LVGL.h"
#include <TAMC_GT911.h>

EPD_PainterLVGL display(EPD_PAINTER_PRESET);
TAMC_GT911      tc(-1, EPD_PAINTER_PRESET.width, EPD_PAINTER_PRESET.height);

static uint32_t my_tick_cb() { return millis(); }

// ── GPS ───────────────────────────────────────────────────────────────────────
#define BOARD_GPS_RXD    44
#define BOARD_GPS_TXD    43
#define GPS_BAUD         9600

// PCA9555 IO expander — port 0 bit 0 HIGH = GPS power on
#define IO_EXPANDER_ADDR 0x20
#define IO_EXP_CONFIG_P0 0x06
#define IO_EXP_OUTPUT_P0 0x02

TinyGPSPlus    gpsParser;
HardwareSerial gpsSerial(1);

static void gps_power_on(TwoWire *wire) {
    wire->beginTransmission(IO_EXPANDER_ADDR);
    wire->write(IO_EXP_CONFIG_P0);
    wire->write(0xFE);
    wire->endTransmission();
    wire->beginTransmission(IO_EXPANDER_ADDR);
    wire->write(IO_EXP_OUTPUT_P0);
    wire->write(0xFF);
    wire->endTransmission();
}

// ── Layout ────────────────────────────────────────────────────────────────────
#define DISP_W   960
#define DISP_H   540
#define HDR_H     60
#define MARGIN    14
#define GAP       12
#define CARD_PAD  16

// ── Math ──────────────────────────────────────────────────────────────────────
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD (M_PI / 180.0)

static float haversine_m(double la1, double lo1, double la2, double lo2) {
    double dlat = (la2-la1)*DEG2RAD, dlon = (lo2-lo1)*DEG2RAD;
    double a = sin(dlat/2)*sin(dlat/2)
             + cos(la1*DEG2RAD)*cos(la2*DEG2RAD)*sin(dlon/2)*sin(dlon/2);
    return (float)(6371000.0 * 2.0 * atan2(sqrt(a), sqrt(1.0-a)));
}

// ── Trip tracking ─────────────────────────────────────────────────────────────
static double   trip_prev_lat = 0.0, trip_prev_lon = 0.0;
static bool     trip_has_prev = false;
static float    trip_km       = 0.0f;
static float    max_speed_kmh = 0.0f;
static bool     trip_started  = false;

// ── Screen ────────────────────────────────────────────────────────────────────
static lv_obj_t *scr       = nullptr;
static uint32_t  last_build = 0;
static const uint32_t BUILD_INTERVAL_MS = 1000;

// ── LVGL helpers ──────────────────────────────────────────────────────────────
static lv_obj_t *make_card(lv_obj_t *p, int x, int y, int w, int h, lv_color_t bg) {
    lv_obj_t *c = lv_obj_create(p);
    lv_obj_set_pos(c, x, y);
    lv_obj_set_size(c, w, h);
    lv_obj_set_style_bg_color(c, bg, 0);
    lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(c, EPD_PainterLVGL::DK_GREY, 0);
    lv_obj_set_style_border_width(c, 2, 0);
    lv_obj_set_style_radius(c, 8, 0);
    lv_obj_set_style_pad_all(c, CARD_PAD, 0);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
    return c;
}
static void mk_key(lv_obj_t *p, const char *t, lv_align_t a, int dx, int dy) {
    lv_obj_t *l = lv_label_create(p);
    lv_label_set_text(l, t);
    lv_obj_set_style_text_color(l, EPD_PainterLVGL::DK_GREY, 0);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_28, 0);
    lv_obj_align(l, a, dx, dy);
}
static void mk_val(lv_obj_t *p, const char *t, lv_align_t a, int dx, int dy) {
    lv_obj_t *l = lv_label_create(p);
    lv_label_set_text(l, t);
    lv_obj_set_style_text_color(l, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_48, 0);
    lv_obj_align(l, a, dx, dy);
}

// ── Compass rose ──────────────────────────────────────────────────────────────
static lv_point_precise_t needle_pts[2];

static void draw_compass(lv_obj_t *parent, int cx, int cy, int r, float hdg) {
    lv_obj_t *ring = lv_obj_create(parent);
    lv_obj_set_pos(ring, cx-r, cy-r);
    lv_obj_set_size(ring, r*2, r*2);
    lv_obj_set_style_radius(ring, r, 0);
    lv_obj_set_style_bg_color(ring, EPD_PainterLVGL::LT_GREY, 0);
    lv_obj_set_style_border_color(ring, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_border_width(ring, 3, 0);
    lv_obj_clear_flag(ring, LV_OBJ_FLAG_SCROLLABLE);

    for (int deg = 0; deg < 360; deg += 45) {
        float rad = (float)(deg * DEG2RAD);
        int   tr  = (deg % 90 == 0) ? r-10 : r-18;
        lv_obj_t *d = lv_obj_create(parent);
        lv_obj_set_pos(d, cx+(int)(tr*sin(rad))-3, cy-(int)(tr*cos(rad))-3);
        lv_obj_set_size(d, 7, 7);
        lv_obj_set_style_radius(d, 3, 0);
        lv_obj_set_style_bg_color(d, EPD_PainterLVGL::DK_GREY, 0);
        lv_obj_set_style_border_width(d, 0, 0);
    }

    const struct { const char *txt; float angle; } CARDS[] = {
        {"N",0.0f},{"E",90.0f},{"S",180.0f},{"W",270.0f}
    };
    for (auto &c : CARDS) {
        float rad = (float)(c.angle * DEG2RAD);
        lv_obj_t *l = lv_label_create(parent);
        lv_label_set_text(l, c.txt);
        lv_obj_set_style_text_color(l, EPD_PainterLVGL::BLACK, 0);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_28, 0);
        lv_obj_set_pos(l, cx+(int)((r-28)*sin(rad))-9, cy-(int)((r-28)*cos(rad))-14);
    }

    float rad = (float)(hdg * DEG2RAD);
    needle_pts[0].x = cx + (int)((r-36)*sin(rad));
    needle_pts[0].y = cy - (int)((r-36)*cos(rad));
    needle_pts[1].x = cx - (int)(30*sin(rad));
    needle_pts[1].y = cy + (int)(30*cos(rad));
    lv_obj_t *needle = lv_line_create(parent);
    lv_line_set_points(needle, needle_pts, 2);
    lv_obj_set_style_line_color(needle, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_line_width(needle, 5, 0);

    lv_obj_t *dot = lv_obj_create(parent);
    lv_obj_set_pos(dot, cx-8, cy-8);
    lv_obj_set_size(dot, 16, 16);
    lv_obj_set_style_radius(dot, 8, 0);
    lv_obj_set_style_bg_color(dot, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_border_width(dot, 0, 0);
}

// ── Build screen ──────────────────────────────────────────────────────────────
static void build_screen() {
    if (scr) lv_obj_del(scr);
    scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, EPD_PainterLVGL::LT_GREY, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    bool hasFix = gpsParser.location.isValid() && gpsParser.location.age() < 3000;

    // ── Header ───────────────────────────────────────────────────────────────
    lv_obj_t *hdr = lv_obj_create(scr);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_size(hdr, DISP_W, HDR_H);
    lv_obj_set_style_bg_color(hdr, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_set_style_pad_hor(hdr, 20, 0);
    lv_obj_set_style_pad_ver(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);
    {
        lv_obj_t *title = lv_label_create(hdr);
        lv_label_set_text(title, hasFix ? "GPS Navigation" : "GPS \xe2\x80\x93 No Fix");
        lv_obj_set_style_text_color(title, EPD_PainterLVGL::WHITE, 0);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
        lv_obj_align(title, LV_ALIGN_LEFT_MID, 0, 0);

        char time_str[12] = "--:--:--";
        if (gpsParser.time.isValid())
            snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
                     gpsParser.time.hour(), gpsParser.time.minute(), gpsParser.time.second());
        lv_obj_t *clk = lv_label_create(hdr);
        lv_label_set_text(clk, time_str);
        lv_obj_set_style_text_color(clk, EPD_PainterLVGL::WHITE, 0);
        lv_obj_set_style_text_font(clk, &lv_font_montserrat_28, 0);
        lv_obj_align(clk, LV_ALIGN_CENTER, 0, 0);

        char sat_str[32];
        int sats = gpsParser.satellites.isValid() ? (int)gpsParser.satellites.value() : 0;
        if (hasFix)
            snprintf(sat_str, sizeof(sat_str), "%d sats  HDOP %.1f",
                     sats, gpsParser.hdop.isValid() ? gpsParser.hdop.hdop() : 0.0f);
        else
            snprintf(sat_str, sizeof(sat_str), "Acquiring...  %d sats", sats);
        lv_obj_t *sats_lbl = lv_label_create(hdr);
        lv_label_set_text(sats_lbl, sat_str);
        lv_obj_set_style_text_color(sats_lbl, EPD_PainterLVGL::DK_GREY, 0);
        lv_obj_set_style_text_font(sats_lbl, &lv_font_montserrat_28, 0);
        lv_obj_align(sats_lbl, LV_ALIGN_RIGHT_MID, 0, 0);
    }

    const int CONTENT_Y = HDR_H + MARGIN;
    const int CONTENT_H = DISP_H - CONTENT_Y - MARGIN;
    const int LEFT_W    = 610;
    const int RIGHT_X   = MARGIN + LEFT_W + GAP;
    const int RIGHT_W   = DISP_W - RIGHT_X - MARGIN;

    // ── Coordinates card ──────────────────────────────────────────────────────
    const int COORD_H = 170;
    lv_obj_t *coord = make_card(scr, MARGIN, CONTENT_Y, LEFT_W, COORD_H, EPD_PainterLVGL::WHITE);
    {
        char lat_str[28] = "---\xc2\xb0 --.----' -";
        char lon_str[28] = "----\xc2\xb0 --.----' -";
        if (hasFix) {
            double la = fabs(gpsParser.location.lat());
            double lo = fabs(gpsParser.location.lng());
            int lad = (int)la, lod = (int)lo;
            snprintf(lat_str, sizeof(lat_str), "%02d\xc2\xb0 %07.4f' %c",
                     lad, (la-lad)*60.0, gpsParser.location.lat() >= 0 ? 'N' : 'S');
            snprintf(lon_str, sizeof(lon_str), "%03d\xc2\xb0 %07.4f' %c",
                     lod, (lo-lod)*60.0, gpsParser.location.lng() >= 0 ? 'E' : 'W');
        }
        mk_key(coord, "LAT", LV_ALIGN_TOP_LEFT, 0, -10);
        mk_val(coord, lat_str, LV_ALIGN_TOP_LEFT, 0, 18);
        mk_key(coord, "LON", LV_ALIGN_TOP_LEFT, 0, 70);
        mk_val(coord, lon_str, LV_ALIGN_TOP_LEFT, 0, 98);
    }

    // ── Stat cards: Speed | Altitude | Heading ────────────────────────────────
    const int STAT_Y = CONTENT_Y + COORD_H + GAP;
    const int STAT_H = CONTENT_H - COORD_H - GAP;
    const int STAT_W = (LEFT_W - 2*GAP) / 3;
    {
        char spd_str[12] = "---";
        if (gpsParser.speed.isValid())
            snprintf(spd_str, sizeof(spd_str), "%.1f", gpsParser.speed.kmph());
        lv_obj_t *c = make_card(scr, MARGIN, STAT_Y, STAT_W, STAT_H, EPD_PainterLVGL::WHITE);
        mk_key(c, "SPEED",  LV_ALIGN_TOP_MID,   0,  0);
        mk_val(c, spd_str,  LV_ALIGN_CENTER,     0, -6);
        mk_key(c, "km/h",   LV_ALIGN_BOTTOM_MID, 0,  0);
    }
    {
        char alt_str[12] = "---";
        if (gpsParser.altitude.isValid())
            snprintf(alt_str, sizeof(alt_str), "%.0f", gpsParser.altitude.meters());
        lv_obj_t *c = make_card(scr, MARGIN + STAT_W + GAP, STAT_Y, STAT_W, STAT_H, EPD_PainterLVGL::WHITE);
        mk_key(c, "ALTITUDE", LV_ALIGN_TOP_MID,   0,  0);
        mk_val(c, alt_str,    LV_ALIGN_CENTER,     0, -6);
        mk_key(c, "m ASL",    LV_ALIGN_BOTTOM_MID, 0,  0);
    }
    {
        char hdg_str[12] = "---";
        if (gpsParser.course.isValid())
            snprintf(hdg_str, sizeof(hdg_str), "%.0f\xc2\xb0", gpsParser.course.deg());
        lv_obj_t *c = make_card(scr, MARGIN + 2*(STAT_W + GAP), STAT_Y, STAT_W, STAT_H, EPD_PainterLVGL::WHITE);
        mk_key(c, "HEADING", LV_ALIGN_TOP_MID,   0,  0);
        mk_val(c, hdg_str,   LV_ALIGN_CENTER,     0, -6);
        mk_key(c, "true",    LV_ALIGN_BOTTOM_MID, 0,  0);
    }

    // ── Compass card ──────────────────────────────────────────────────────────
    lv_obj_t *comp = make_card(scr, RIGHT_X, CONTENT_Y, RIGHT_W, CONTENT_H, EPD_PainterLVGL::WHITE);
    {
        int r    = min((RIGHT_W - 2*CARD_PAD) / 2 - 10, (CONTENT_H - 2*CARD_PAD) / 2 - 20);
        int cx   = (RIGHT_W - 2*CARD_PAD) / 2;
        int cy   = CONTENT_H / 2 - 20;
        float hdg = gpsParser.course.isValid() ? (float)gpsParser.course.deg() : 0.0f;
        draw_compass(comp, cx, cy, r, hdg);

        char hdg_str[12];
        if (gpsParser.course.isValid())
            snprintf(hdg_str, sizeof(hdg_str), "%.0f\xc2\xb0", gpsParser.course.deg());
        else
            snprintf(hdg_str, sizeof(hdg_str), "---");
        mk_val(comp, hdg_str, LV_ALIGN_BOTTOM_MID, 0, 0);
    }

    lv_screen_load(scr);
    last_build = millis();
}

// ── Touch ─────────────────────────────────────────────────────────────────────
static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    tc.read();
    data->point.x = tc.x;
    data->point.y = tc.y;
    data->state   = tc.down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

// ── Setup / loop ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    lv_init();
    lv_tick_set_cb(my_tick_cb);

    if (!display.begin()) {
        Serial.println("Display init failed!");
        while (1) delay(1000);
    }

    if (display.getConfig().i2c.wire != nullptr) {
        tc.setRotation(ROTATION_RIGHT);
        tc.begin(display.getConfig().i2c.wire);
        gps_power_on(display.getConfig().i2c.wire);
        gpsSerial.begin(GPS_BAUD, SERIAL_8N1, BOARD_GPS_RXD, BOARD_GPS_TXD);
    }

    display.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
    display.clear();
    build_screen();

    lv_indev_t *touch = lv_indev_create();
    lv_indev_set_type(touch, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touch, touch_read_cb);
}

void loop() {
    while (gpsSerial.available())
        gpsParser.encode(gpsSerial.read());

    if (gpsParser.location.isUpdated() && gpsParser.location.isValid()) {
        double lat = gpsParser.location.lat();
        double lon = gpsParser.location.lng();

        if (!trip_started) { trip_started = true; }

        if (trip_has_prev)
            trip_km += haversine_m(trip_prev_lat, trip_prev_lon, lat, lon) / 1000.0f;
        trip_prev_lat = lat;
        trip_prev_lon = lon;
        trip_has_prev = true;

        if (gpsParser.speed.isValid()) {
            float spd = (float)gpsParser.speed.kmph();
            if (spd > max_speed_kmh) max_speed_kmh = spd;
        }
    }

    if (millis() - last_build >= BUILD_INTERVAL_MS)
        build_screen();

    lv_timer_handler();
    delay(5);
}
