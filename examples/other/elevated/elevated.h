#pragma once
#include <math.h>
#include <stdint.h>
// FreeRTOS is pulled in by Arduino.h in Arduino builds; include explicitly
// only for pure ESP-IDF builds.
#ifndef ARDUINO
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif

// =============================================================================
// Fast exp / pow approximations
//
// Standard expf / powf use Cody-Waite range reduction and take ~25-50 cycles.
// These bit-trick versions take ~3-5 cycles with <0.5% relative error —
// more than sufficient for visual rendering.
//
// Based on Nicol Schraudolph, "A Fast, Compact Approximation of the
// Exponential Function", Neural Computation 11(4), 1999.
// =============================================================================

static inline float fexp2f(float x) {
    // Exploit IEEE 754: exponent bits sit at [30:23], biased by 127.
    // Multiply x by 2^23 and add the bias to set the exponent directly.
    union { float f; int32_t i; } u;
    u.i = (int32_t)(x * 8388608.f + 1065353216.f);   // 8388608=2^23, bias=127*2^23
    return u.f;
}

static inline float fexpf(float x) {
    return fexp2f(x * 1.4426950408f);    // x * log2(e)
}

// powf(base, exp) = 2^(exp * log2(base))
// Extracts log2(base) from the IEEE 754 representation of base.
static inline float fpowf(float base, float exp) {
    union { float f; int32_t i; } u;
    u.f = base;
    float log2_base = (float)(u.i - 1065353216) * (1.0f / 8388608.f);
    return fexp2f(exp * log2_base);
}

// =============================================================================
// Vector math
// =============================================================================

struct V2 {
    float x, y;
    V2()                 : x(0), y(0) {}
    V2(float s)          : x(s), y(s) {}
    V2(float x, float y) : x(x), y(y) {}
    V2 operator+(V2 b)    const { return {x+b.x, y+b.y}; }
    V2 operator*(float s) const { return {x*s,   y*s};   }
};

struct V3 {
    float x, y, z;
    V3()                          : x(0), y(0), z(0) {}
    V3(float s)                   : x(s), y(s), z(s) {}
    V3(float x, float y, float z) : x(x), y(y), z(z) {}
    V3 operator+(V3 b)    const { return {x+b.x,y+b.y,z+b.z}; }
    V3 operator-(V3 b)    const { return {x-b.x,y-b.y,z-b.z}; }
    V3 operator*(V3 b)    const { return {x*b.x,y*b.y,z*b.z}; }
    V3 operator*(float s) const { return {x*s,  y*s,  z*s};   }
    V3 operator-()        const { return {-x,   -y,   -z};    }
};

static inline V3    operator*(float s, V3 a)      { return a * s; }
static inline float dot(V3 a, V3 b)               { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline V3    normalize(V3 a)               { return a * (1.f / sqrtf(a.x*a.x + a.y*a.y + a.z*a.z)); }
static inline V3    cross(V3 a, V3 b)             { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
static inline V3    reflect(V3 e, V3 n)           { return e - n*(2.f*dot(e,n)); }
static inline float sat(float x)                  { return x<0.f?0.f:(x>1.f?1.f:x); }
static inline float lrp(float a, float b, float t){ return a+(b-a)*t; }
static inline V3    lrp(V3 a,    V3 b,    float t){ return a+(b-a)*t; }
static inline float sstep(float lo, float hi, float x) {
    float t = sat((x-lo)/(hi-lo));
    return t*t*(3.f - 2.f*t);
}

// =============================================================================
// Noise result (value + partial derivatives)
// =============================================================================

struct NR { float v, dx, dz; };

// =============================================================================
// Scene parameters
// =============================================================================

struct Scene {
    V3    sun;      // normalised sun direction
    float water;    // water level (world Y)
    float season;   // 0 = winter, 1 = summer
    float ts;       // terrain vertical scale
    V3    cam;      // camera world position
    V3    target;   // look-at point
    float time;
};

// =============================================================================
// Dual-core render task state
// The helper task (core 0) renders rows [row_start, row_end).
// =============================================================================

struct RenderJob {
    const Scene *sc;
    V3           right, up_v, fwd;
    float        fov_tan, aspect;
    int          row_start, row_end;
    uint8_t     *buf;
    int          buf_stride;
    SemaphoreHandle_t start_sem;
    SemaphoreHandle_t done_sem;
};

// =============================================================================
// PIE assembly declaration
// =============================================================================

#ifdef __cplusplus
extern "C" {
#endif

// Expand one src row (src_w bytes) to dst (src_w*4 bytes) via 4x replication.
// src_w must be a multiple of 16.
void elevated_upscale_row(const uint8_t *src, uint8_t *dst, int src_w);

#ifdef __cplusplus
}
#endif
