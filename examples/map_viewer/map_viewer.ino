// map_viewer.ino
//
// Downloads a 2×2 grid of OpenStreetMap tiles centred on the GPS position
// and renders them on the e-paper display.
//
// Source grid: 512×512 px (4 tiles of 256×256), scaled to fill 960×540.
// The GPS position is kept near the centre of the grid.
//
// Board: LilyGo T5 S3 GPS

#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <PNGdec.h>
#include <TinyGPSPlus.h>
#include <esp_heap_caps.h>
#include <math.h>
#include "EPD_Painter_presets.h"
#include "EPD_Painter.h"

// ── Demo mode ─────────────────────────────────────────────────────────────────
// When defined, uses a fixed landmark coordinate instead of live GPS.
// Useful for screenshots and forum posts without revealing your location.
#define DEMO_MODE
#define DEMO_LAT  48.858370   // Eiffel Tower, Paris
#define DEMO_LON   2.294481

// ── WiFi ──────────────────────────────────────────────────────────────────────
#define WIFI_SSID  "<YOUR SSPAID>"
#define WIFI_PASS  "<YOUR PASSWORD>"

// ── GPS ───────────────────────────────────────────────────────────────────────
#define GPS_RX           44
#define GPS_TX           43
#define GPS_BAUD         9600
#define IO_EXPANDER_ADDR 0x20
#define IO_EXP_CONFIG_P0 0x06
#define IO_EXP_OUTPUT_P0 0x02

// ── Map ───────────────────────────────────────────────────────────────────────
#define TILE_ZOOM  17
#define TILE_SIZE  256
#define GRID_W     (TILE_SIZE * 2)   // 512
#define GRID_H     (TILE_SIZE * 2)   // 512
#define DISP_W     960
#define DISP_H     540

// ── Globals ───────────────────────────────────────────────────────────────────
EPD_Painter    painter(EPD_PAINTER_PRESET);
TinyGPSPlus    gps;
HardwareSerial gpsSerial(1);

static uint8_t *framebuffer = nullptr;   // 960×540 8bpp  (PSRAM)
static uint8_t *tile_gray   = nullptr;   // 512×512 gray  (PSRAM)

static PNG png;

// Top-left tile of the current 2×2 grid
static int    cur_gtx = -1, cur_gty = -1;

// Lat/lon of the centre of the last painted grid (for distance checking)
static double grid_centre_lat = 0.0, grid_centre_lon = 0.0;

// GPS position at the time the current grid was fetched (for marker placement)
static double map_lat = 0.0, map_lon = 0.0;

#define REFRESH_DISTANCE_M  100   // re-fetch when GPS moves this far from grid centre

// Written before each tile decode so the callback knows where to place pixels
static int png_off_x = 0, png_off_y = 0;

// ── GPS power ─────────────────────────────────────────────────────────────────
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

// ── Tile coordinate math ──────────────────────────────────────────────────────
// Returns the fractional tile coordinate (integer part = tile index,
// fractional part = position within tile).
static void latlon_to_tile_f(double lat, double lon, int zoom,
                              double &tx_f, double &ty_f) {
    int n    = 1 << zoom;
    double r = lat * M_PI / 180.0;
    tx_f = (lon + 180.0) / 360.0 * n;
    ty_f = (1.0 - log(tan(r) + 1.0 / cos(r)) / M_PI) / 2.0 * n;
}

// Convert tile corner back to lat/lon (used to find grid centre)
static void tile_to_latlon(int tx, int ty, int zoom, double &lat, double &lon) {
    int n = 1 << zoom;
    lon = (double)tx / n * 360.0 - 180.0;
    lat = atan(sinh(M_PI * (1.0 - 2.0 * ty / n))) * 180.0 / M_PI;
}

// Haversine distance in metres between two lat/lon points
static float haversine_m(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dlat/2)*sin(dlat/2)
             + cos(lat1*M_PI/180.0)*cos(lat2*M_PI/180.0)*sin(dlon/2)*sin(dlon/2);
    return (float)(6371000.0 * 2.0 * atan2(sqrt(a), sqrt(1.0 - a)));
}

// Picks the 2×2 grid origin so the GPS position lands near the centre.
static void grid_origin(double lat, double lon, int zoom, int &gtx, int &gty) {
    double tx_f, ty_f;
    latlon_to_tile_f(lat, lon, zoom, tx_f, ty_f);
    int tx = (int)tx_f;
    int ty = (int)ty_f;
    // If GPS is in the right half of tile, use this tile as left column; else shift left
    gtx = ((tx_f - tx) >= 0.5) ? tx : tx - 1;
    gty = ((ty_f - ty) >= 0.5) ? ty : ty - 1;
}

// ── Grayscale → 4-level EPD value ─────────────────────────────────────────────
static inline uint8_t gray_to_epd(uint8_t g) {
    if (g < 64)  return 3;
    if (g < 128) return 2;
    if (g < 192) return 1;
    return 0;
}

// ── PNGdec draw callback ──────────────────────────────────────────────────────
static int png_draw_cb(PNGDRAW *pDraw) {
    uint16_t line[TILE_SIZE];
    png.getLineAsRGB565(pDraw, line, PNG_RGB565_LITTLE_ENDIAN, 0xFFFFFFFF);
    int row = png_off_y + pDraw->y;
    for (int x = 0; x < pDraw->iWidth && x < TILE_SIZE; x++) {
        uint16_t rgb = line[x];
        uint8_t r = ((rgb >> 11) & 0x1F) * 255 / 31;
        uint8_t g = ((rgb >>  5) & 0x3F) * 255 / 63;
        uint8_t b = ((rgb >>  0) & 0x1F) * 255 / 31;
        tile_gray[row * GRID_W + png_off_x + x] = (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
    }
    return 1;
}

// ── Download one tile, decode into the correct quadrant of tile_gray[] ────────
static bool fetch_one_tile(int tx, int ty, int zoom, int off_x, int off_y) {
    char url[128];
    snprintf(url, sizeof(url),
             "https://tile.openstreetmap.org/%d/%d/%d.png", zoom, tx, ty);
    Serial.printf("Fetching %s\n", url);

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    http.begin(client, url);
    http.addHeader("User-Agent", "EPD_Painter/1.0 MapViewer");
    int code = http.GET();

    if (code != HTTP_CODE_OK) {
        Serial.printf("HTTP %d\n", code);
        http.end();
        return false;
    }

    int len = http.getSize();
    if (len <= 0 || len > 200000) {
        Serial.printf("Bad payload size: %d\n", len);
        http.end();
        return false;
    }

    uint8_t *buf = (uint8_t *)heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
    if (!buf) {
        Serial.println("OOM for PNG buffer");
        http.end();
        return false;
    }

    WiFiClient *stream = http.getStreamPtr();
    size_t got = 0;
    uint32_t t = millis();
    while (got < (size_t)len && millis() - t < 10000) {
        int avail = stream->available();
        if (avail > 0) {
            got += stream->readBytes(buf + got, min(avail, (int)(len - got)));
            t = millis();
        } else {
            delay(5);
        }
    }
    http.end();

    if (got < (size_t)len) {
        Serial.printf("Short read: %d / %d\n", got, len);
        heap_caps_free(buf);
        return false;
    }

    png_off_x = off_x;
    png_off_y = off_y;

    int rc = png.openRAM(buf, len, png_draw_cb);
    if (rc == PNG_SUCCESS) {
        rc = png.decode(nullptr, 0);
        png.close();
    }
    heap_caps_free(buf);

    if (rc != PNG_SUCCESS) {
        Serial.printf("PNG decode error: %d\n", rc);
        return false;
    }

    return true;
}

// ── Fetch all 4 tiles of the 2×2 grid ────────────────────────────────────────
static bool fetch_grid(int gtx, int gty, int zoom) {
    if (!fetch_one_tile(gtx,   gty,   zoom, 0,         0))         return false;
    if (!fetch_one_tile(gtx+1, gty,   zoom, TILE_SIZE, 0))         return false;
    if (!fetch_one_tile(gtx,   gty+1, zoom, 0,         TILE_SIZE)) return false;
    if (!fetch_one_tile(gtx+1, gty+1, zoom, TILE_SIZE, TILE_SIZE)) return false;
    return true;
}

// ── Scale tile_gray[512×512] → framebuffer[960×540] ──────────────────────────
static void render_to_framebuffer() {
    uint8_t mn = 255, mx = 0;
    for (int i = 0; i < GRID_W * GRID_H; i++) {
        if (tile_gray[i] < mn) mn = tile_gray[i];
        if (tile_gray[i] > mx) mx = tile_gray[i];
    }
    int range = mx - mn;
    if (range < 1) range = 1;
    Serial.printf("Grid gray range: %d–%d\n", mn, mx);

    for (int dy = 0; dy < DISP_H; dy++) {
        int sy = dy * GRID_H / DISP_H;
        for (int dx = 0; dx < DISP_W; dx++) {
            int sx = dx * GRID_W / DISP_W;
            uint8_t s = (uint8_t)(((int)(tile_gray[sy * GRID_W + sx] - mn) * 255) / range);
            float g = powf(s / 255.0f, 5.0f) * 255.0f;
            framebuffer[dy * DISP_W + dx] = gray_to_epd((uint8_t)g);
        }
    }
}

// ── Draw position marker onto framebuffer ─────────────────────────────────────
static void draw_marker() {
    double tx_f, ty_f;
    latlon_to_tile_f(map_lat, map_lon, TILE_ZOOM, tx_f, ty_f);

    // Pixel position within the 512×512 grid
    int gx = (int)((tx_f - cur_gtx) * TILE_SIZE);
    int gy = (int)((ty_f - cur_gty) * TILE_SIZE);

    // Scale to framebuffer coordinates
    int px = gx * DISP_W / GRID_W;
    int py = gy * DISP_H / GRID_H;

    const int OUTER_R = 14;
    const int INNER_R = 7;

    for (int dy = -OUTER_R; dy <= OUTER_R; dy++) {
        for (int dx = -OUTER_R; dx <= OUTER_R; dx++) {
            int fx = px + dx;
            int fy = py + dy;
            if (fx < 0 || fx >= DISP_W || fy < 0 || fy >= DISP_H) continue;
            int d2 = dx*dx + dy*dy;
            if (d2 <= INNER_R * INNER_R)
                framebuffer[fy * DISP_W + fx] = 0;   // white centre
            else if (d2 <= OUTER_R * OUTER_R)
                framebuffer[fy * DISP_W + fx] = 3;   // black ring
        }
    }
}

// ── Update map if moved far enough from grid centre ───────────────────────────
static void update_map() {
#ifndef DEMO_MODE
    if (!gps.location.isValid()) {
        Serial.println("Waiting for GPS fix...");
        return;
    }
#endif

#ifdef DEMO_MODE
    double lat = DEMO_LAT;
    double lon = DEMO_LON;
#else
    double lat = gps.location.lat();
    double lon = gps.location.lng();
#endif

    // First load, or moved more than threshold from the centre of the current grid
    bool needs_update = (cur_gtx == -1);
    if (!needs_update) {
        float dist = haversine_m(lat, lon, grid_centre_lat, grid_centre_lon);
        Serial.printf("Distance from grid centre: %.0f m\n", dist);
        needs_update = (dist > REFRESH_DISTANCE_M);
    }

    if (!needs_update) return;

    int gtx, gty;
    grid_origin(lat, lon, TILE_ZOOM, gtx, gty);

    Serial.printf("Refreshing grid: %d/%d/%d\n", TILE_ZOOM, gtx, gty);
    map_lat = lat;
    map_lon = lon;
    if (fetch_grid(gtx, gty, TILE_ZOOM)) {
        cur_gtx = gtx;
        cur_gty = gty;
        render_to_framebuffer();
        draw_marker();
        painter.paint(framebuffer);
        // Store the lat/lon of the centre corner of the 2×2 grid
        tile_to_latlon(gtx + 1, gty + 1, TILE_ZOOM, grid_centre_lat, grid_centre_lon);
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    if (!painter.begin()) {
        Serial.println("Display init failed!");
        while (1) delay(1000);
    }
    painter.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
    painter.clear();
    painter.clear();

    gps_power_on(painter.getConfig().i2c.wire);

    framebuffer = (uint8_t *)heap_caps_aligned_alloc(16, DISP_W * DISP_H, MALLOC_CAP_SPIRAM);
    tile_gray   = (uint8_t *)heap_caps_malloc(GRID_W * GRID_H, MALLOC_CAP_SPIRAM);
    if (!framebuffer || !tile_gray) {
        Serial.println("PSRAM allocation failed!");
        while (1) delay(1000);
    }
    memset(framebuffer, 0x00, DISP_W * DISP_H);

    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

    Serial.printf("WiFi: connecting to %s...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nConnected: %s\n", WiFi.localIP().toString().c_str());
}

// ── Loop ──────────────────────────────────────────────────────────────────────
static uint32_t last_check = 0;

void loop() {
    while (gpsSerial.available())
        gps.encode(gpsSerial.read());

    if (millis() - last_check >= 5000) {
        last_check = millis();
        update_map();
    }
}
