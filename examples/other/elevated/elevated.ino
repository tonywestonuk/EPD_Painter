// =============================================================================
//  Elevated — ESP32 Port
//
//  Based on "Elevated" by Inigo Quilez (TBC & RGBA)
//  Winner of Breakpoint 2009 4K Intro Competition
//  Original license: CC BY-NC-SA 3.0
//  https://creativecommons.org/licenses/by-nc-sa/3.0/
//
//  Ported to ESP32 C++ software rasteriser for EPD_Painter.
//  Renders at 240x135, upscaled 4x to 960x540 for the e-paper display.
//
//  Speed-ups vs the first version:
//    1. Permutation-table noise   — removes 4 multiplicative hashes per no() call
//    2. Fast fexpf / fpowf        — bit-trick approximation, ~8x faster
//    3. Dual-core rendering       — core 0 renders bottom half in parallel
//    4. PIE-vectorised upscale    — ee.vzip.8 does 16→64 bytes in 11 instructions
// =============================================================================

// Choose your hardware:
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
// #define EPD_PAINTER_PRESET_M5PAPER_S3

// elevated.h must be first — Arduino inserts auto-prototypes after includes,
// so NR, Scene and RenderJob must be visible at that point.
#include "elevated.h"
#include "EPD_Painter_presets.h"
#include <stdint.h>
#include "esp_heap_caps.h"

static EPD_Painter display(EPD_PAINTER_PRESET);

// --- Render resolution ---
static const int RW = 240;
static const int RH = 135;

static uint8_t *render_buf  = nullptr;   // RW*RH  — internal RAM
static uint8_t *display_buf = nullptr;   // 960*540 — PSRAM

// =============================================================================
// Permutation-table noise
//
// The original Elevated samples a 256x256 random float texture for each
// corner of the bilinear cell.  We replace that with a classic Perlin-style
// permutation table: two 8-bit array lookups replace four 32-bit multiplies.
//
// perm[512]  — shuffled 0..255 repeated twice (avoids index masking on add)
// rval[256]  — pre-computed float random values in [0, 1]
// =============================================================================

static uint8_t perm[512];
static float   rval[256];

static void init_noise_table() {
    // Deterministic shuffle using a simple LCG
    uint32_t s = 0xDEADBEEF;
    uint8_t  t[256];
    for (int i = 0; i < 256; i++) t[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        s = s * 1664525u + 1013904223u;
        int j = s >> 24;            // 0..255
        uint8_t tmp = t[i]; t[i] = t[j]; t[j] = tmp;
    }
    for (int i = 0; i < 256; i++) {
        perm[i] = perm[i+256] = t[i];
        s = s * 1664525u + 1013904223u;
        rval[i] = (float)(s >> 16) / 65535.f;
    }
}

// Two table lookups instead of four 32-bit multiplies + XOR mixing
static inline float hash2(int x, int z) {
    return rval[ perm[ (perm[x & 255] + (z & 255)) & 255 ] ];
}

// =============================================================================
// Value noise with derivatives  — equivalent to no() in original Elevated
// =============================================================================

static NR no(float px, float pz) {
    float fx = px - floorf(px);
    float fz = pz - floorf(pz);

    float ux  = fx*fx*fx*(fx*(fx*6.f-15.f)+10.f);
    float uz  = fz*fz*fz*(fz*(fz*6.f-15.f)+10.f);
    float dux = 30.f*fx*fx*(fx*(fx-2.f)+1.f);
    float duz = 30.f*fz*fz*(fz*(fz-2.f)+1.f);

    int ix = (int)floorf(px), iz = (int)floorf(pz);
    float a = hash2(ix,   iz),   b = hash2(ix+1, iz);
    float c = hash2(ix,   iz+1), d = hash2(ix+1, iz+1);
    float ba = b-a, ca = c-a, abcd = a-b-c+d;

    return { a + ba*ux + ca*uz + abcd*ux*uz,
             dux*(ba + abcd*uz),
             duz*(ca + abcd*ux) };
}

// =============================================================================
// Terrain height — derivative-dampened fBm, equivalent to f() in original.
// The march() inner loop uses OCT_MARCH (4) octaves for speed; the final
// shading pass uses more for visual detail.
// =============================================================================

static const int OCT_MARCH  = 3;   // coarser but fast — used during ray march
static const int OCT_NORMAL = 7;   // detail normal for terrain texturing
static const int OCT_SMOOTH = 4;   // smooth normal for lighting / soft shadow

static float terrain(float px, float pz, int octaves) {
    float ddx = 0.f, ddz = 0.f, acc = 0.f, b = 3.f;
    for (int i = 0; i < octaves; i++) {
        NR n = no(0.25f*px, 0.25f*pz);
        ddx += n.dx;  ddz += n.dz;
        acc += (b *= 0.5f) * n.v / (1.f + ddx*ddx + ddz*ddz);
        float npx = 1.6f*px - 1.2f*pz;
        pz        = 1.2f*px + 1.6f*pz;
        px        = npx;
    }
    return acc;
}

// Terrain normal via central differences — equivalent to cn() in original.
// ny_scale: stretched Y before normalising (set to 6 for flat water surface).
static V3 terrain_normal(float px, float pz, float e, int oct, float ts,
                          float ny_scale = 1.f) {
    float h  = terrain(px,   pz,   oct);
    float hx = terrain(px+e, pz,   oct);
    float hz = terrain(px,   pz+e, oct);
    return normalize(V3(ts*(h-hx), e*ny_scale, ts*(h-hz)));
}

// =============================================================================
// Precomputed terrain heightmaps (startup bake)
//
// htc: coarse (OCT_MARCH octaves, 0.5-unit step) replaces terrain() in march
// htf: fine   (OCT_NORMAL octaves, 0.25-unit step) replaces terrain_normal()
// ±100 world-unit coverage handles all camera positions + tmax=60 ray reach.
// Bilinear lookup at runtime eliminates all terrain() calls from inner loops.
// =============================================================================

static const float HT_RANGE = 100.f;

static const float HTC_STEP = 0.5f;
static const int   HTC_N    = 401;     // 2*100/0.5 + 1
static float      *htc      = nullptr; // ~643 KB PSRAM

static const float HTF_STEP = 0.25f;
static const int   HTF_N    = 801;     // 2*100/0.25 + 1
static float      *htf      = nullptr; // ~2.5 MB PSRAM

static inline float ht_bilerp(const float *map, int N, float step,
                               float px, float pz) {
    float fx = (px + HT_RANGE) / step;
    float fz = (pz + HT_RANGE) / step;
    int ix = (int)fx, iz = (int)fz;
    if (ix < 0) ix = 0; else if (ix > N-2) ix = N-2;
    if (iz < 0) iz = 0; else if (iz > N-2) iz = N-2;
    float tx = fx - ix, tz = fz - iz;
    float h00 = map[ iz    * N + ix    ];
    float h10 = map[ iz    * N + ix + 1];
    float h01 = map[(iz+1) * N + ix    ];
    float h11 = map[(iz+1) * N + ix + 1];
    return h00 + (h10-h00)*tx + (h01-h00)*tz + (h11-h10-h01+h00)*tx*tz;
}

// Drop-in replacement for terrain(px, pz, OCT_MARCH) in march inner loop
static inline float terrain_fast(float px, float pz) {
    return ht_bilerp(htc, HTC_N, HTC_STEP, px, pz);
}

// Replaces both terrain_normal() calls per hit pixel (detail + smooth).
// Fixed epsilon = HTF_STEP; one call vs the original 6 terrain() calls.
static V3 terrain_normal_fast(float px, float pz, float ts) {
    float h  = ht_bilerp(htf, HTF_N, HTF_STEP, px,            pz);
    float hx = ht_bilerp(htf, HTF_N, HTF_STEP, px + HTF_STEP, pz);
    float hz = ht_bilerp(htf, HTF_N, HTF_STEP, px,            pz + HTF_STEP);
    return normalize(V3(ts*(h-hx), HTF_STEP, ts*(h-hz)));
}

static void precompute_heightmaps() {
    Serial.printf("[Elevated] Baking terrain (%dx%d coarse + %dx%d fine)...\n",
                  HTC_N, HTC_N, HTF_N, HTF_N);
    uint32_t t0 = millis();

    for (int iz = 0; iz < HTC_N; iz++) {
        float pz = -HT_RANGE + iz * HTC_STEP;
        for (int ix = 0; ix < HTC_N; ix++)
            htc[iz * HTC_N + ix] = terrain(-HT_RANGE + ix * HTC_STEP, pz, OCT_MARCH);
        if ((iz & 15) == 0) vTaskDelay(pdMS_TO_TICKS(1));
    }
    Serial.printf("[Elevated]  coarse: %u ms\n", millis() - t0);

    uint32_t t1 = millis();
    for (int iz = 0; iz < HTF_N; iz++) {
        float pz = -HT_RANGE + iz * HTF_STEP;
        for (int ix = 0; ix < HTF_N; ix++)
            htf[iz * HTF_N + ix] = terrain(-HT_RANGE + ix * HTF_STEP, pz, OCT_NORMAL);
        if ((iz & 7) == 0) vTaskDelay(pdMS_TO_TICKS(1));
    }
    Serial.printf("[Elevated]  fine:   %u ms.  Total bake: %u ms\n",
                  millis() - t1, millis() - t0);
}

// =============================================================================
// Scene factory
// =============================================================================

static float cnoise(float t) {
    int   it = (int)floorf(t);
    float ft = t - (float)it;
    float u  = ft*ft*(3.f - 2.f*ft);
    return lrp(hash2(it, 7), hash2(it+1, 7), u)*2.f - 1.f;
}

static Scene make_scene(float t) {
    Scene s;
    s.time   = t;
    s.ts     = 0.28f;
    s.season = 0.5f + 0.5f*sinf(t*0.022f);

    float sa = t*0.04f + 1.5f;
    s.sun    = normalize(V3(cosf(sa), 0.3125f, sinf(sa)));
    s.water  = 0.20f;

    float ct  = t * 0.25f;
    float spd = 0.07f;
    float cx  = 16.f*cosf(spd*ct      + 3.f*cnoise(ct*0.08f))
              +  8.f*cosf(spd*ct*2.f  + 3.f*cnoise(ct*0.16f));
    float cz  = 16.f*cosf(spd*ct*1.3f + 3.f*cnoise(ct*0.11f))
              +  8.f*cosf(spd*ct*2.6f + 3.f*cnoise(ct*0.22f));

    float gy  = s.ts * terrain(cx, cz, 3);
    s.cam     = V3(cx, gy + 0.45f, cz);
    s.target  = V3(cx + cosf(ct*0.37f)*8.f,
                   gy + 0.10f,
                   cz + sinf(ct*0.31f)*8.f);
    return s;
}

// =============================================================================
// Lighting — equivalent to b() in original
// =============================================================================

static V3 lighting(const Scene& sc, V3 n, V3 d) {
    float sdl = dot(d, sc.sun);
    float ndl = lrp(sdl, dot(n, sc.sun), 0.5f + 0.5f*sc.season);
    V3 amb = V3(0.13f, 0.18f, 0.22f) * (n.y + 0.25f*sat(-ndl) - 0.05f);
    V3 dif = V3(1.4f,  1.0f,  0.7f)  * sat(ndl) * sat(2.f*sdl);
    return amb + dif;
}

// =============================================================================
// Terrain ray march — height-guided stepping
//
// The old fixed-scale dt formula (0.01*t + 0.002) needs ~600 steps to cover
// t=0..100 but we capped at 180, so sky pixels always ran all 180 steps and
// still missed the horizon.
//
// Height-guided stepping: dt = max(h * 0.4, min_dt + scale * t)
//   - When ray is far above terrain (h large) → big step, few iterations
//   - When ray skims the surface (h small)    → small step, accurate hit
// Sky pixels typically converge in ~30-50 steps; terrain hits in ~20-40.
// =============================================================================

static bool march(const Scene& sc, V3 ro, V3 rd,
                  float tmin, float tmax, float& hit_t) {
    float t  = tmin;
    float dt = 0.01f;

    for (int i = 0; i < 120 && t < tmax; i++) {
        float px = ro.x + rd.x*t, pz = ro.z + rd.z*t;
        float h  = (ro.y + rd.y*t) - sc.ts*terrain_fast(px, pz);
        if (h < 0.f) {
            float ta = t - dt, tb = t;
            for (int j = 0; j < 5; j++) {
                float tc = (ta+tb)*0.5f;
                float hc = (ro.y+rd.y*tc) - sc.ts*terrain(ro.x+rd.x*tc, ro.z+rd.z*tc, OCT_MARCH);
                if (hc < 0.f) tb = tc; else ta = tc;
            }
            hit_t = (ta+tb)*0.5f;
            return true;
        }
        // Step proportional to height above terrain; floor grows with distance
        // so very shallow/horizontal rays don't stall with tiny steps.
        dt = h * 0.4f;
        float dt_min = 0.008f + 0.04f*t;
        if (dt < dt_min) dt = dt_min;
        if (dt > 6.0f)   dt = 6.0f;
        t += dt;
    }
    return false;
}

// =============================================================================
// Per-pixel renderer — combined equivalent of m3 + m4 in original.
// fpowf / fexpf replace the standard library versions throughout.
// =============================================================================

static float render_pixel(const Scene& sc,
                           V3 ro, V3 right, V3 up_v, V3 fwd,
                           float fx, float fy, float fov_tan) {
    V3    rd      = normalize(fwd + right*(fx*fov_tan) + up_v*(fy*fov_tan));
    float sun_dot = sat(dot(rd, sc.sun));

    // Sky: dome + horizon + sun disc + clouds
    V3 col = V3(0.55f, 0.65f, 0.75f)
           + V3(0.50f) * fpowf(1.f - sat(rd.y), 8.f)
           + V3(0.40f, 0.30f, 0.10f) * fpowf(sun_dot, 16.f);

    float hit_t;
    bool  hit = march(sc, ro, rd, 0.05f, 60.f, hit_t);

    if (hit) {
        V3    hp    = ro + rd*hit_t;
        float w     = sc.water - hp.y;
        float eff_t = hit_t;

        if (w < 0.f) {
            // ---- Terrain above water ----
            // One precomputed-map normal replaces two terrain_normal() calls.
            // Rock micro-texture (r) and high-freq color sample (h) removed —
            // not distinguishable at 4 grey levels on e-paper.
            V3 n = terrain_normal_fast(hp.x, hp.z, sc.ts);

            V3 tcol     = V3(0.1f + 0.75f*sc.season);
            V3 flat_col = lrp(V3(0.80f, 0.85f, 0.90f),
                              V3(0.45f, 0.45f, 0.20f),
                              sc.season);
            tcol = lrp(tcol, flat_col,
                       sstep(0.5f - 0.8f*n.y, 1.f - 1.1f*n.y, 0.075f));
            V3 slope_col = lrp(V3(0.37f, 0.23f, 0.08f),
                               V3(0.42f, 0.40f, 0.20f), sc.season);
            tcol = lrp(tcol, slope_col,
                       sstep(0.f, 1.f, 50.f*(n.y-1.f) + (0.5f+sc.season)/0.4f));

            col = tcol * lighting(sc, n, n);

        } else {
            // ---- Water surface ----
            if (rd.y < 0.f) {
                float wt = (sc.water - ro.y) / rd.y;
                if (wt > 0.f) {
                    V3 wp = ro + rd*wt;
                    float wx = 512.f*wp.x + sat(w*60.f)*sc.time;
                    float wz = 32.f*wp.z;
                    V3 wn = terrain_normal(wx, wz, 0.001f*wt, 4, sc.ts, 6.f);

                    V3 depth = V3(0.40f,1.0f,1.0f) - V3(0.20f,0.60f,0.40f)*sat(w*16.f);
                    V3 wcol  = V3(0.12f) * depth * (0.3f + 0.7f*sc.season);

                    float fres = fpowf(1.f - sat(-dot(rd, wn)), 4.f);
                    float spec = fpowf(sat(dot(sc.sun, reflect(-rd, wn))), 32.f);
                    wcol = wcol + fres*(spec*V3(0.32f,0.31f,0.30f) + V3(0.10f));

                    col   = wcol;
                    eff_t = wt;
                }
            }
        }

        col = col * (0.7f + 0.3f*sstep(0.f, 1.f, 256.f*fabsf(w)));

        // Fog: extinction + inscattering using fast exp
        col = col * fexpf(-0.042f * eff_t);
        V3 fog = V3(0.52f,0.59f,0.65f)
               + V3(0.60f,0.40f,0.10f)*fpowf(sun_dot, 8.f);
        col = col + V3(1.f - fexpf(-0.1f*eff_t)) * fog;
    }

    // Tone map — gamma 0.45 via fast pow
    col.x = fpowf(sat(col.x), 0.45f);
    col.y = fpowf(sat(col.y), 0.45f);
    col.z = fpowf(sat(col.z), 0.45f);

    return 0.2126f*col.x + 0.7152f*col.y + 0.0722f*col.z;
}

// =============================================================================
// Camera setup
// =============================================================================

static void setup_camera(const Scene& sc,
                          V3& right, V3& up_v, V3& fwd, float& fov_tan) {
    fwd     = normalize(sc.target - sc.cam);
    right   = normalize(cross(fwd, V3(0.f, 1.f, 0.f)));
    up_v    = cross(right, fwd);
    fov_tan = tanf(0.55f);
}

// =============================================================================
// Render a row range into buf (stride = buf_stride bytes per row)
// Shared between both cores.
// =============================================================================

static void render_rows(const Scene* sc,
                         V3 right, V3 up_v, V3 fwd,
                         float fov_tan, float aspect,
                         int y0, int y1,
                         uint8_t* buf, int stride) {
    for (int y = y0; y < y1; y++) {
        float fy = 1.f - 2.f*(y + 0.5f)/RH;
        for (int x = 0; x < RW; x++) {
            float fx  = (2.f*(x + 0.5f)/RW - 1.f) * aspect;
            float lum = render_pixel(*sc, sc->cam, right, up_v, fwd, fx, fy, fov_tan);
            int   v   = (int)(lum * 3.f + 0.5f);
            v = v < 0 ? 0 : v > 3 ? 3 : v;
            buf[y * stride + x] = (uint8_t)(3 - v);
        }
        // Yield for 1 ms per row so the idle task can run and reset the WDT.
        // Works from both the main loopTask and the helper task on core 0.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =============================================================================
// Dual-core rendering
//
// The EPD paint task lives on core 0 (priority 10) and blocks on a semaphore
// most of the time.  We create a persistent helper task on core 0 at priority
// 5; it renders the bottom half of the image while the Arduino loop (core 1)
// renders the top half simultaneously.  The paint task will preempt the helper
// if a new frame is ready to display.
// =============================================================================

static RenderJob g_job;
static TaskHandle_t helper_task_h = nullptr;

static void helper_task(void*) {
    for (;;) {
        xSemaphoreTake(g_job.start_sem, portMAX_DELAY);
        render_rows(g_job.sc,
                    g_job.right, g_job.up_v, g_job.fwd,
                    g_job.fov_tan, g_job.aspect,
                    g_job.row_start, g_job.row_end,
                    g_job.buf, g_job.buf_stride);
        xSemaphoreGive(g_job.done_sem);
    }
}

// =============================================================================
// Upscale render_buf (RW×RH) → display_buf (960×540)
//
// Each source row is written to 4 consecutive display rows.
// Within each row, elevated_upscale_row() (PIE assembly) replicates every
// source byte 4 times, turning 240 bytes → 960 bytes in one pass.
// =============================================================================

static void upscale() {
    for (int sy = 0; sy < RH; sy++) {
        const uint8_t *src  = render_buf + sy * RW;
        // Each source row maps to 4 display rows
        for (int rep = 0; rep < 4; rep++) {
            uint8_t *dst = display_buf + (sy*4 + rep) * 960;
            elevated_upscale_row(src, dst, RW);
        }
    }
}

// =============================================================================
// Arduino entry points
// =============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("[Elevated] Initialising...");

    init_noise_table();

    render_buf  = (uint8_t*)heap_caps_malloc(RW * RH,   MALLOC_CAP_INTERNAL);
    display_buf = (uint8_t*)heap_caps_malloc(960 * 540, MALLOC_CAP_SPIRAM);
    htc = (float*)heap_caps_malloc((size_t)HTC_N * HTC_N * sizeof(float), MALLOC_CAP_SPIRAM);
    htf = (float*)heap_caps_malloc((size_t)HTF_N * HTF_N * sizeof(float), MALLOC_CAP_SPIRAM);

    if (!render_buf || !display_buf || !htc || !htf) {
        Serial.println("[Elevated] FATAL: buffer allocation failed");
        while (1) delay(1000);
    }

    precompute_heightmaps();

    if (!display.begin()) {
        Serial.println("[Elevated] FATAL: display init failed");
        while (1) delay(1000);
    }
    display.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
    display.clear();

    // Create the dual-core render helper on core 0
    g_job.start_sem = xSemaphoreCreateBinary();
    g_job.done_sem  = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(helper_task, "elev_render", 4096, nullptr,
                            5, &helper_task_h, 0);

    Serial.printf("[Elevated] Ready. Rendering %dx%d -> 960x540\n", RW, RH);
}

static float g_time = 0.0f;

void loop() {
    uint32_t t0 = millis();
    Scene sc = make_scene(g_time);

    V3 right, up_v, fwd;
    float fov_tan;
    setup_camera(sc, right, up_v, fwd, fov_tan);

    float aspect = (float)RW / (float)RH;

    Serial.printf("[Elevated] t=%.1fs  season=%.2f  sun=(%.2f,%.2f,%.2f)\n",
                  g_time, sc.season, sc.sun.x, sc.sun.y, sc.sun.z);

    // Kick off helper task (core 0): bottom half
    g_job.sc         = &sc;
    g_job.right      = right;
    g_job.up_v       = up_v;
    g_job.fwd        = fwd;
    g_job.fov_tan    = fov_tan;
    g_job.aspect     = aspect;
    g_job.row_start  = RH / 2;
    g_job.row_end    = RH;
    g_job.buf        = render_buf;
    g_job.buf_stride = RW;
    xSemaphoreGive(g_job.start_sem);

    // Core 1 renders top half concurrently
    render_rows(&sc, right, up_v, fwd, fov_tan, aspect,
                0, RH / 2, render_buf, RW);

    // Wait for core 0 to finish bottom half
    xSemaphoreTake(g_job.done_sem, portMAX_DELAY);

    uint32_t render_ms = millis() - t0;
    Serial.printf("[Elevated] Render: %u ms.  Upscaling + painting...\n", render_ms);

    upscale();
    display.paint(display_buf);


    g_time += 1.5f;
}
