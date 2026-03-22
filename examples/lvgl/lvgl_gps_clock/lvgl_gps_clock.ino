// GPS Clock – 7-segment style digital clock from GPS time.
// Board: LilyGo T5 S3 GPS

#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS

#include <Arduino.h>
#include <lvgl.h>
#include <TinyGPSPlus.h>
#include "EPD_Painter_presets.h"
#include "EPD_Painter_LVGL.h"
#include <gt911_lite.h>

// Must be declared before Arduino preprocessor inserts function forward-decls
// Each digit is ONE lv_obj_t; the draw callback renders all 7 segments itself.
struct SegDigit {
    lv_obj_t *obj;
    uint8_t   pattern;  // current segment pattern
    uint8_t   seg_t;    // segment thickness for this digit size
};

EPD_PainterLVGL display(EPD_PAINTER_PRESET);
GT911_Lite      tc;

static uint32_t my_tick_cb() { return millis(); }

// ── GPS ───────────────────────────────────────────────────────────────────────
#define BOARD_GPS_RXD    44
#define BOARD_GPS_TXD    43
#define GPS_BAUD         9600
#define IO_EXPANDER_ADDR 0x20
#define IO_EXP_CONFIG_P0 0x06
#define IO_EXP_OUTPUT_P0 0x02

TinyGPSPlus    gpsParser;
HardwareSerial gpsSerial(1);

static void gps_power_on(TwoWire *wire) {
    wire->beginTransmission(IO_EXPANDER_ADDR);
    wire->write(IO_EXP_CONFIG_P0); wire->write(0xFE); wire->endTransmission();
    wire->beginTransmission(IO_EXPANDER_ADDR);
    wire->write(IO_EXP_OUTPUT_P0); wire->write(0xFF); wire->endTransmission();
}

// ── Timezones ─────────────────────────────────────────────────────────────────
struct TZ { const char *label; int offset_min; };
static const TZ TIMEZONES[] = {
    { "UTC+0   London (GMT)",         0    },
    { "UTC+1   Paris / Berlin",       60   },
    { "UTC+2   Athens / Cairo",       120  },
    { "UTC+3   Moscow / Nairobi",     180  },
    { "UTC+4   Dubai",                240  },
    { "UTC+5:30 Mumbai / Delhi",      330  },
    { "UTC+6   Dhaka",                360  },
    { "UTC+7   Bangkok / Jakarta",    420  },
    { "UTC+8   Singapore / Beijing",  480  },
    { "UTC+9   Tokyo / Seoul",        540  },
    { "UTC+10  Sydney (AEST)",        600  },
    { "UTC+12  Auckland",             720  },
    { "UTC-3:30 St. John's",         -210  },
    { "UTC-4   Halifax (AST)",       -240  },
    { "UTC-5   New York (EST)",      -300  },
    { "UTC-6   Chicago (CST)",       -360  },
    { "UTC-7   Denver (MST)",        -420  },
    { "UTC-8   Los Angeles (PST)",   -480  },
    { "UTC-9   Anchorage",           -540  },
    { "UTC-10  Hawaii",              -600  },
};
static const int TZ_COUNT = (int)(sizeof(TIMEZONES) / sizeof(TIMEZONES[0]));
static int tz_idx = 0;

static bool local_time(int &h, int &m, int &s) {
    if (!gpsParser.time.isValid()) return false;
    int total = (int)gpsParser.time.hour() * 60 + (int)gpsParser.time.minute()
                + TIMEZONES[tz_idx].offset_min;
    total = ((total % 1440) + 1440) % 1440;
    h = total / 60; m = total % 60; s = (int)gpsParser.time.second();
    return true;
}

// ── 7-segment patterns ────────────────────────────────────────────────────────
// Bit order: a=0(top) b=1(top-R) c=2(bot-R) d=3(bot) e=4(bot-L) f=5(top-L) g=6(mid)
static const uint8_t SEG_PATTERNS[10] = {
    0x3F, // 0: a b c d e f
    0x06, // 1: b c
    0x5B, // 2: a b d e g
    0x4F, // 3: a b c d g
    0x66, // 4: b c f g
    0x6D, // 5: a c d f g
    0x7D, // 6: a c d e f g
    0x07, // 7: a b c
    0x7F, // 8: all
    0x6F, // 9: a b c d f g
};
#define SEG_DASH 0x40u  // g only (middle bar)
#define SEG_NONE 0x00u

// ── Layout ────────────────────────────────────────────────────────────────────
#define DISP_W     960
#define DISP_H     540

// Digit box
#define DIG_W      108
#define DIG_H      300
#define SEG_T       24   // segment thickness
#define SEG_INSET    0   // chamfered ends create natural gaps; no extra inset needed

// Clock positioning
// Total width: 6 digits + 2 colons(36px each) + 7 gaps(12px)
// = 6*108 + 2*36 + 7*12 = 648+72+84 = 804  → left margin = (960-804)/2 = 78
#define COL_W       36
#define ELEM_GAP    12
#define CLOCK_X    78
#define CLOCK_Y   100   // pushed down to make room for date row above

// Satellite count 7-segment display (top strip, y=4)
#define SAT_DIG_H       44
#define SAT_DIG_W       26
#define SAT_SEG_T        6
#define SAT_ELEM_GAP     4
#define SAT_DIGITS_X   140
#define SAT_ROW_Y       10

// Date 7-segment display (between sats and clock)
// DD.MM.YYYY across the full clock width
#define DATE_DIG_H      50
#define DATE_DIG_W      50
#define DATE_SEG_T       8
#define DATE_ELEM_GAP    5
#define DATE_SEP_SZ     15   // separator dot size between day / month / year
#define DATE_Y          20   // top of date digit row (bottom = 52+64 = 116, gap to clock = 8)
#define DATE_X         400  // left-align with clock

// Coordinate 7-segment display (below clock)
#define COORD_DIGITS     8    // 3 integer + 5 decimal digits per coordinate
#define COORD_DIG_W     26
#define COORD_DIG_H     48
#define COORD_SEG_T      6
#define COORD_ELEM_GAP   4
#define COORD_DOT_SZ     7   // decimal-point dot diameter
#define COORD_LABEL_X   20
#define COORD_DIGITS_X 110   // x where digit[0] starts (after "Lat:" label)
#define COORD_ROW1_Y   (CLOCK_Y + DIG_H + 12)
#define COORD_ROW2_Y   (COORD_ROW1_Y + COORD_DIG_H + 15)

// Pre-compute element X positions
static const int DIG_X[6] = {
    CLOCK_X,                                                      // H1
    CLOCK_X + DIG_W + ELEM_GAP,                                  // H2
    CLOCK_X + 2*DIG_W + 3*ELEM_GAP + COL_W,                     // M1
    CLOCK_X + 3*DIG_W + 4*ELEM_GAP + COL_W,                     // M2
    CLOCK_X + 4*DIG_W + 6*ELEM_GAP + 2*COL_W,                   // S1
    CLOCK_X + 5*DIG_W + 7*ELEM_GAP + 2*COL_W,                   // S2
};
static const int COL_X[2] = {
    CLOCK_X + 2*DIG_W + 2*ELEM_GAP,   // colon between H and M
    CLOCK_X + 4*DIG_W + 5*ELEM_GAP + COL_W,  // colon between M and S
};

// ── Widget storage ────────────────────────────────────────────────────────────
static SegDigit  digits[6];
static lv_obj_t *colon_dot[2][2];
static SegDigit  coord_digits[2][COORD_DIGITS];
static lv_obj_t *coord_dec_dot[2];
static lv_obj_t *coord_dir_lbl[2];
static SegDigit  sat_digits[2];
static SegDigit  date_digits[8];
static lv_obj_t *date_sep_dot[2];
static lv_obj_t *popup = nullptr;
static lv_obj_t *dd_popup_tz = nullptr;
static uint32_t  last_update = 0;

// ── Segment helpers ───────────────────────────────────────────────────────────
static lv_obj_t *make_rect(lv_obj_t *p, int x, int y, int w, int h) {
    lv_obj_t *o = lv_obj_create(p);
    lv_obj_set_pos(o, x, y); lv_obj_set_size(o, w, h);
    lv_obj_set_style_radius(o, 3, 0);
    lv_obj_set_style_border_width(o, 0, 0);
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    return o;
}

// Helpers: draw a hexagonal segment as two triangles + one centre rectangle.
// Each hexagon is the segment's full bounding box with 45° chamfered ends.
static void draw_seg_h(lv_layer_t *layer, lv_color_t col,
                       int32_t x1, int32_t y1, int32_t x2, int32_t y2) {
    int32_t c = (y2 - y1) / 2;  // chamfer = half thickness

    lv_draw_triangle_dsc_t td;
    lv_draw_triangle_dsc_init(&td);
    td.color = col;
    td.opa   = LV_OPA_COVER;

    // Left pointed cap
    td.p[0].x = x1;     td.p[0].y = y1 + c;
    td.p[1].x = x1 + c; td.p[1].y = y1;
    td.p[2].x = x1 + c; td.p[2].y = y2;
    lv_draw_triangle(layer, &td);

    // Right pointed cap
    td.p[0].x = x2 - c; td.p[0].y = y1;
    td.p[1].x = x2;     td.p[1].y = y1 + c;
    td.p[2].x = x2 - c; td.p[2].y = y2;
    lv_draw_triangle(layer, &td);

    // Centre rectangle
    lv_draw_rect_dsc_t rd;
    lv_draw_rect_dsc_init(&rd);
    rd.bg_color     = col;
    rd.bg_opa       = LV_OPA_COVER;
    rd.radius       = 0;
    rd.border_width = 0;
    lv_area_t area  = {x1 + c, y1, x2 - c, y2};
    lv_draw_rect(layer, &rd, &area);
}

static void draw_seg_v(lv_layer_t *layer, lv_color_t col,
                       int32_t x1, int32_t y1, int32_t x2, int32_t y2) {
    int32_t c = (x2 - x1) / 2;  // chamfer = half thickness

    lv_draw_triangle_dsc_t td;
    lv_draw_triangle_dsc_init(&td);
    td.color = col;
    td.opa   = LV_OPA_COVER;

    // Top pointed cap
    td.p[0].x = x1 + c; td.p[0].y = y1;
    td.p[1].x = x2;     td.p[1].y = y1 + c;
    td.p[2].x = x1;     td.p[2].y = y1 + c;
    lv_draw_triangle(layer, &td);

    // Bottom pointed cap
    td.p[0].x = x1;     td.p[0].y = y2 - c;
    td.p[1].x = x2;     td.p[1].y = y2 - c;
    td.p[2].x = x1 + c; td.p[2].y = y2;
    lv_draw_triangle(layer, &td);

    // Centre rectangle
    lv_draw_rect_dsc_t rd;
    lv_draw_rect_dsc_init(&rd);
    rd.bg_color     = col;
    rd.bg_opa       = LV_OPA_COVER;
    rd.radius       = 0;
    rd.border_width = 0;
    lv_area_t area  = {x1, y1 + c, x2, y2 - c};
    lv_draw_rect(layer, &rd, &area);
}

// Unified draw callback: one lv_obj_t covers the full digit bounding box;
// all 7 segments are drawn here, ON=black OFF=white.
static void digit_draw_cb(lv_event_t *e) {
    lv_obj_t  *obj = (lv_obj_t *)lv_event_get_target(e);
    SegDigit  *sd  = (SegDigit *)lv_obj_get_user_data(obj);
    if (!sd) return;

    lv_layer_t *layer = lv_event_get_layer(e);
    lv_area_t   coords;
    lv_obj_get_coords(obj, &coords);

    int32_t ox = coords.x1, oy = coords.y1;
    int32_t W  = coords.x2 - ox;
    int32_t H  = coords.y2 - oy;
    int32_t T  = sd->seg_t;
    int32_t HH = H / 2;
    uint8_t p  = sd->pattern;

#define SEG_COL(bit) (((p >> (bit)) & 1u) ? EPD_PainterLVGL::BLACK : EPD_PainterLVGL::WHITE)
    draw_seg_h(layer, SEG_COL(0), ox+T,   oy,        ox+W-T, oy+T);        // a top
    draw_seg_v(layer, SEG_COL(1), ox+W-T, oy+T,      ox+W,   oy+HH);       // b top-right
    draw_seg_v(layer, SEG_COL(2), ox+W-T, oy+HH,     ox+W,   oy+H-T);      // c bot-right
    draw_seg_h(layer, SEG_COL(3), ox+T,   oy+H-T,    ox+W-T, oy+H);        // d bottom
    draw_seg_v(layer, SEG_COL(4), ox,     oy+HH,     ox+T,   oy+H-T);      // e bot-left
    draw_seg_v(layer, SEG_COL(5), ox,     oy+T,      ox+T,   oy+HH);       // f top-left
    draw_seg_h(layer, SEG_COL(6), ox+T,   oy+HH-T/2, ox+W-T, oy+HH+T/2);  // g middle
#undef SEG_COL
}

// Create a single digit object (ONE lv_obj_t for all 7 segments).
static void create_digit_at(lv_obj_t *parent, int dx, int dy,
                             int W, int H, int T, SegDigit &d) {
    d.seg_t   = (uint8_t)T;
    d.pattern = SEG_NONE;

    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_set_pos(o, dx, dy);
    lv_obj_set_size(o, W, H);
    lv_obj_set_style_bg_opa(o, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(o, 0, 0);
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_set_style_shadow_width(o, 0, 0);
    lv_obj_set_style_outline_width(o, 0, 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_user_data(o, &d);
    lv_obj_add_event_cb(o, digit_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    d.obj = o;
}

static void create_digit(lv_obj_t *parent, int dx, int dy, SegDigit &d) {
    create_digit_at(parent, dx, dy, DIG_W, DIG_H, SEG_T, d);
}

// X position of coord digit i, accounting for extra gap where the decimal dot sits.
static int coord_dig_x(int i) {
    int x = COORD_DIGITS_X + i * (COORD_DIG_W + COORD_ELEM_GAP);
    if (i >= 3) x += COORD_ELEM_GAP + COORD_DOT_SZ;  // skip over decimal dot
    return x;
}

// X position of date digit i: [0,1]=day  [2,3]=month  [4..7]=year
// Extra space is inserted after digit 1 and after digit 3 for the separator dots.
static int date_dig_x(int i) {
    int x = DATE_X + i * (DATE_DIG_W + DATE_ELEM_GAP);
    if (i >= 2) x += DATE_SEP_SZ + DATE_ELEM_GAP;   // skip over day separator
    if (i >= 4) x += DATE_SEP_SZ + DATE_ELEM_GAP;   // skip over month separator
    return x;
}

static void set_digit(SegDigit &d, uint8_t pattern) {
    if (d.pattern != pattern) {
        d.pattern = pattern;
        lv_obj_invalidate(d.obj);
    }
}

static void set_digit_num(SegDigit &d, int v) {
    set_digit(d, (v >= 0 && v <= 9) ? SEG_PATTERNS[v] : SEG_DASH);
}

// ── Settings popup ────────────────────────────────────────────────────────────
static void close_popup(lv_event_t *) {
    if (popup) { lv_obj_del(popup); popup = nullptr; dd_popup_tz = nullptr; }
}

static void popup_tz_changed(lv_event_t *e) {
    if (dd_popup_tz) tz_idx = (int)lv_dropdown_get_selected(dd_popup_tz);
}

static void open_popup(lv_event_t *) {
    if (popup) return;

    lv_obj_t *scr = lv_screen_active();

    // Full-screen dimming overlay
    popup = lv_obj_create(scr);
    lv_obj_set_pos(popup, 0, 0);
    lv_obj_set_size(popup, DISP_W, DISP_H);
    lv_obj_set_style_bg_color(popup, EPD_PainterLVGL::DK_GREY, 0);
    lv_obj_set_style_bg_opa(popup, LV_OPA_50, 0);
    lv_obj_set_style_border_width(popup, 0, 0);
    lv_obj_set_style_radius(popup, 0, 0);
    lv_obj_clear_flag(popup, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(popup, close_popup, LV_EVENT_CLICKED, NULL);

    // Card
    lv_obj_t *card = lv_obj_create(popup);
    lv_obj_set_size(card, 500, 280);
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

    lv_obj_t *tz_lbl = lv_label_create(card);
    lv_label_set_text(tz_lbl, "Timezone");
    lv_obj_set_style_text_font(tz_lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(tz_lbl, EPD_PainterLVGL::DK_GREY, 0);
    lv_obj_align(tz_lbl, LV_ALIGN_TOP_LEFT, 0, 60);

    // Build option string
    static char tz_opts[TZ_COUNT * 42];
    tz_opts[0] = '\0';
    for (int i = 0; i < TZ_COUNT; i++) {
        strncat(tz_opts, TIMEZONES[i].label, sizeof(tz_opts) - strlen(tz_opts) - 2);
        if (i < TZ_COUNT - 1) strncat(tz_opts, "\n", sizeof(tz_opts) - strlen(tz_opts) - 1);
    }

    dd_popup_tz = lv_dropdown_create(card);
    lv_dropdown_set_options(dd_popup_tz, tz_opts);
    lv_dropdown_set_selected(dd_popup_tz, tz_idx);
    lv_obj_set_width(dd_popup_tz, 440);
    lv_obj_align(dd_popup_tz, LV_ALIGN_TOP_LEFT, 0, 96);
    lv_obj_set_style_text_font(dd_popup_tz, &lv_font_montserrat_28, 0);
    lv_obj_add_event_cb(dd_popup_tz, popup_tz_changed, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_t *list = lv_dropdown_get_list(dd_popup_tz);
    lv_obj_set_style_text_font(list, &lv_font_montserrat_28, 0);
    lv_obj_set_style_max_height(list, 280, 0);

    lv_obj_t *close_btn = lv_button_create(card);
    lv_obj_set_size(close_btn, 120, 48);
    lv_obj_align(close_btn, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
    lv_obj_set_style_bg_color(close_btn, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_radius(close_btn, 6, 0);
    lv_obj_add_event_cb(close_btn, close_popup, LV_EVENT_CLICKED, NULL);
    lv_obj_t *close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, "Close");
    lv_obj_set_style_text_font(close_lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(close_lbl, EPD_PainterLVGL::WHITE, 0);
    lv_obj_center(close_lbl);
}

// ── Build static UI ───────────────────────────────────────────────────────────
static void build_ui() {
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, EPD_PainterLVGL::WHITE, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // ── Date row (DD.MM.YYYY) ─────────────────────────────────────────────────
    for (int i = 0; i < 8; i++) {
        create_digit_at(scr, date_dig_x(i), DATE_Y,
                        DATE_DIG_W, DATE_DIG_H, DATE_SEG_T, date_digits[i]);
        set_digit(date_digits[i], SEG_DASH);
    }
    // Separator dots: positioned after digit 1 and digit 3, near the bottom
    {
        int sep_y = DATE_Y + DATE_DIG_H - DATE_SEP_SZ - 4;
        int sep_xs[2] = {
            DATE_X + 2*(DATE_DIG_W + DATE_ELEM_GAP) + DATE_ELEM_GAP,
            date_dig_x(3) + DATE_DIG_W + DATE_ELEM_GAP
        };
        for (int s = 0; s < 2; s++) {
            date_sep_dot[s] = make_rect(scr, sep_xs[s], sep_y, DATE_SEP_SZ, DATE_SEP_SZ);
            lv_obj_set_style_radius(date_sep_dot[s], DATE_SEP_SZ / 2, 0);
            lv_obj_set_style_bg_color(date_sep_dot[s], EPD_PainterLVGL::BLACK, 0);
            lv_obj_set_style_bg_opa(date_sep_dot[s], LV_OPA_COVER, 0);
        }
    }

    // ── 6 digits ─────────────────────────────────────────────────────────────
    for (int i = 0; i < 6; i++)
        create_digit(scr, DIG_X[i], CLOCK_Y, digits[i]);

    // ── 2 colons ──────────────────────────────────────────────────────────────
    int dot_r = 10;
    for (int c = 0; c < 2; c++) {
        int cx = COL_X[c] + COL_W/2 - dot_r;
        colon_dot[c][0] = make_rect(scr, cx, CLOCK_Y + DIG_H/3 - dot_r,   dot_r*2, dot_r*2);
        colon_dot[c][1] = make_rect(scr, cx, CLOCK_Y + 2*DIG_H/3 - dot_r, dot_r*2, dot_r*2);
        for (int d = 0; d < 2; d++) {
            lv_obj_set_style_radius(colon_dot[c][d], dot_r, 0);
            lv_obj_set_style_bg_color(colon_dot[c][d], EPD_PainterLVGL::BLACK, 0);
            lv_obj_set_style_bg_opa(colon_dot[c][d], LV_OPA_COVER, 0);
        }
    }

    // ── Satellite count (top-left) ────────────────────────────────────────────
    lv_obj_t *sat_lbl = lv_label_create(scr);
    lv_label_set_text(sat_lbl, LV_SYMBOL_GPS " SATS");
    lv_obj_set_style_text_font(sat_lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(sat_lbl, EPD_PainterLVGL::DK_GREY, 0);
    lv_obj_set_pos(sat_lbl, 20, SAT_ROW_Y + (SAT_DIG_H - 28) / 2);

    create_digit_at(scr, SAT_DIGITS_X, SAT_ROW_Y,
                    SAT_DIG_W, SAT_DIG_H, SAT_SEG_T, sat_digits[0]);
    create_digit_at(scr, SAT_DIGITS_X + SAT_DIG_W + SAT_ELEM_GAP, SAT_ROW_Y,
                    SAT_DIG_W, SAT_DIG_H, SAT_SEG_T, sat_digits[1]);
    set_digit(sat_digits[0], SEG_DASH);
    set_digit(sat_digits[1], SEG_DASH);

    // ── Coordinate 7-segment rows (below clock) ───────────────────────────────
    static const char *coord_labels[2] = { "Lat:", "Lng:" };
    static const int   coord_row_y[2]  = { COORD_ROW1_Y, COORD_ROW2_Y };

    for (int row = 0; row < 2; row++) {
        int ry = coord_row_y[row];

        // "Lat:" / "Lng:" label, vertically centred against the digit height
        lv_obj_t *lbl = lv_label_create(scr);
        lv_label_set_text(lbl, coord_labels[row]);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(lbl, EPD_PainterLVGL::DK_GREY, 0);
        lv_obj_set_pos(lbl, COORD_LABEL_X, ry + (COORD_DIG_H - 28) / 2);

        // 8 7-segment digit objects
        for (int i = 0; i < COORD_DIGITS; i++) {
            create_digit_at(scr, coord_dig_x(i), ry,
                            COORD_DIG_W, COORD_DIG_H, COORD_SEG_T, coord_digits[row][i]);
            set_digit(coord_digits[row][i], SEG_NONE);
        }

        // Decimal dot between integer and fractional digits
        int dot_x = COORD_DIGITS_X + 3 * (COORD_DIG_W + COORD_ELEM_GAP) + COORD_ELEM_GAP;
        int dot_y = ry + COORD_DIG_H - COORD_DOT_SZ - 3;
        coord_dec_dot[row] = make_rect(scr, dot_x, dot_y, COORD_DOT_SZ, COORD_DOT_SZ);
        lv_obj_set_style_radius(coord_dec_dot[row], COORD_DOT_SZ / 2, 0);
        lv_obj_set_style_bg_color(coord_dec_dot[row], EPD_PainterLVGL::BLACK, 0);
        lv_obj_set_style_bg_opa(coord_dec_dot[row], LV_OPA_COVER, 0);

        // N/S or E/W direction label
        coord_dir_lbl[row] = lv_label_create(scr);
        lv_label_set_text(coord_dir_lbl[row], "");
        lv_obj_set_style_text_font(coord_dir_lbl[row], &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(coord_dir_lbl[row], EPD_PainterLVGL::DK_GREY, 0);
        lv_obj_set_pos(coord_dir_lbl[row],
                       coord_dig_x(COORD_DIGITS - 1) + COORD_DIG_W + 8,
                       ry + (COORD_DIG_H - 28) / 2);
    }

    // ── Settings button (bottom-right) ────────────────────────────────────────
    lv_obj_t *btn = lv_button_create(scr);
    lv_obj_set_size(btn, 160, 44);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_RIGHT, -20, -14);
    lv_obj_set_style_bg_color(btn, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_radius(btn, 6, 0);
    lv_obj_add_event_cb(btn, open_popup, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_lbl = lv_label_create(btn);
    lv_label_set_text(btn_lbl, "Settings");
    lv_obj_set_style_text_font(btn_lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(btn_lbl, EPD_PainterLVGL::WHITE, 0);
    lv_obj_center(btn_lbl);

    lv_screen_load(scr);

    // Initial digit state — all dashes
    for (int i = 0; i < 6; i++) set_digit(digits[i], SEG_DASH);
}

// ── Coordinate helpers ────────────────────────────────────────────────────────
static void update_coord_row(int row, double val, char dir_char) {
    double absval = fabs(val);
    int int_part  = (int)absval;
    int dec_part  = (int)round((absval - int_part) * 100000.0);
    if (dec_part >= 100000) { dec_part -= 100000; int_part++; }

    int d[8] = {
        int_part / 100,
        (int_part / 10) % 10,
        int_part % 10,
        dec_part / 10000,
        (dec_part / 1000) % 10,
        (dec_part / 100)  % 10,
        (dec_part / 10)   % 10,
        dec_part % 10
    };

    // Blank leading zeros in the integer part (always show units digit)
    set_digit(coord_digits[row][0], int_part >= 100 ? SEG_PATTERNS[d[0]] : SEG_NONE);
    set_digit(coord_digits[row][1], int_part >= 10  ? SEG_PATTERNS[d[1]] : SEG_NONE);
    set_digit(coord_digits[row][2], SEG_PATTERNS[d[2]]);
    for (int i = 3; i < COORD_DIGITS; i++)
        set_digit(coord_digits[row][i], SEG_PATTERNS[d[i]]);

    char dir_str[2] = { dir_char, '\0' };
    lv_label_set_text(coord_dir_lbl[row], dir_str);
}

static void clear_coord_row(int row) {
    for (int i = 0; i < COORD_DIGITS; i++) set_digit(coord_digits[row][i], SEG_NONE);
    lv_label_set_text(coord_dir_lbl[row], "");
}

// ── Refresh ───────────────────────────────────────────────────────────────────
static void refresh() {
    int h, m, s;
    bool hasTime = local_time(h, m, s);

    if (hasTime) {
        set_digit_num(digits[0], h / 10);
        set_digit_num(digits[1], h % 10);
        set_digit_num(digits[2], m / 10);
        set_digit_num(digits[3], m % 10);
        set_digit_num(digits[4], s / 10);
        set_digit_num(digits[5], s % 10);
    } else {
        for (int i = 0; i < 6; i++) set_digit(digits[i], SEG_DASH);
    }

    // Date  DD.MM.YYYY
    if (gpsParser.date.isValid()) {
        int dd = (int)gpsParser.date.day();
        int mm = (int)gpsParser.date.month();
        int yy = (int)gpsParser.date.year();
        int d[8] = {
            dd / 10,         dd % 10,
            mm / 10,         mm % 10,
            (yy / 1000) % 10, (yy / 100) % 10, (yy / 10) % 10, yy % 10
        };
        for (int i = 0; i < 8; i++) set_digit(date_digits[i], SEG_PATTERNS[d[i]]);
    } else {
        for (int i = 0; i < 8; i++) set_digit(date_digits[i], SEG_DASH);
    }

    // Coordinates
    bool hasFix = gpsParser.location.isValid() && gpsParser.location.age() < 5000;
    if (hasFix) {
        double la = gpsParser.location.lat();
        double lo = gpsParser.location.lng();
        update_coord_row(0, la, la >= 0 ? 'N' : 'S');
        update_coord_row(1, lo, lo >= 0 ? 'E' : 'W');
    } else {
        clear_coord_row(0);
        clear_coord_row(1);
    }

    // Satellite count
    if (gpsParser.satellites.isValid()) {
        int sats = (int)gpsParser.satellites.value();
        set_digit(sat_digits[0], sats >= 10 ? SEG_PATTERNS[sats / 10] : SEG_NONE);
        set_digit(sat_digits[1], SEG_PATTERNS[sats % 10]);
    } else {
        set_digit(sat_digits[0], SEG_DASH);
        set_digit(sat_digits[1], SEG_DASH);
    }
}

// ── Touch ─────────────────────────────────────────────────────────────────────
static void touch_read_cb(lv_indev_t *, lv_indev_data_t *data) {
    tc.read();
    data->point.x =  tc.y;
    data->point.y =  display.getConfig().height-tc.x;
    
    data->state   = tc.down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

// ── Setup / loop ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    lv_init();
    lv_tick_set_cb(my_tick_cb);

    if (!display.begin()) { Serial.println("Display init failed!"); while (1) delay(1000); }

    if (display.getConfig().i2c.wire != nullptr) {
        tc.begin(display.getConfig().i2c.wire);
        gps_power_on(display.getConfig().i2c.wire);
        gpsSerial.begin(GPS_BAUD, SERIAL_8N1, BOARD_GPS_RXD, BOARD_GPS_TXD);
    }

    display.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
    display.clear();
    build_ui();

    lv_indev_t *touch = lv_indev_create();
    lv_indev_set_type(touch, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touch, touch_read_cb);
}

void loop() {
    while (gpsSerial.available()) gpsParser.encode(gpsSerial.read());

    if (millis() - last_update >= 1000) {
        refresh();
        last_update = millis();
    }

    lv_timer_handler();
    delay(5);
}
