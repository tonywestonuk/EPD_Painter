#ifdef __ASSEMBLER__
// =============================================================================
// EPD Painter Assembly Functions
//
// Uses ESP32-S3 PIE (128-bit vector) instructions to process pixels 16 bytes
// at a time. The processor handles all pipeline interlocks automatically;
// instructions are written in logical order.
//
// Vector registers q0–q4 are scratch; q5–q7 hold constants loaded in each
// function's prologue.
// =============================================================================

    .text
    .align 4

// =============================================================================
// epd_painter_compact_pixels
//
// Converts a GFX canvas (1 byte per pixel, values 0–3) to a packed 2bpp
// framebuffer (4 pixels per byte, most-significant pixel first).
//
// Parameters:
//   a2 - Input pointer  (1 byte per pixel)
//   a3 - Output pointer (2bpp packed)
//   a4 - Size in bytes  (multiple of 16)
// =============================================================================
    .global epd_painter_compact_pixels
    .type   epd_painter_compact_pixels, @function

epd_painter_compact_pixels:
    entry   a1, 32

    ee.zero.q       q7              // q7 = zero (used as shift source)

    // Broadcast 2-bit pixel mask 0x03 to all lanes of q5
    movi            a10, 0b00000011
    s32i            a10, a1, 0
    ee.vldbc.8      q5, a1

    // SAR_BYTE=1 for ee.src.q (byte shift), SAR=2 for ee.vsl.32 (2-bit shift)
    movi            a10, 1
    wur.sar_byte    a10
    movi            a10, 2
    wsr.sar         a10

    mov             a11, a4         // loop counter

.compact_process_pixels:
    // Load 16 pixels and strip each to its 2-bit value
    ee.vld.128.ip   q0, a2, 16
    ee.andq         q0, q0, q5

    // Fold 16 bytes into 4 by shifting and OR-ing in three passes.
    // Each pass halves the active byte count and doubles the pixels per byte:
    //   pass 1: 16 bytes, 1 pixel/byte  →  8 bytes, 2 pixels/byte
    //   pass 2: 8 bytes,  2 pixels/byte →  4 bytes, 4 pixels/byte (interleaved)
    //   pass 3: consolidates into the low 4 bytes
    ee.src.q        q1, q0, q7      // q1 = q0 >> 8 bytes
    ee.vsl.32       q0, q0          // q0 <<= 2 bits
    ee.orq          q0, q0, q1

    ee.src.q        q1, q1, q7
    ee.vsl.32       q0, q0
    ee.orq          q0, q0, q1

    ee.src.q        q1, q1, q7
    ee.vsl.32       q0, q0
    ee.orq          q0, q0, q1

    // Deinterleave so the 4 packed result bytes are contiguous
    ee.vunzip.8     q0, q1
    ee.vunzip.8     q0, q1

    // Store the 4 output bytes then advance the output pointer
    ee.stxq.32      q0, q7, a3, 0, 0
    addi            a3, a3, 4

    addi            a11, a11, -16
    bnez            a11, .compact_process_pixels

    retw.n


// =============================================================================
// epd_painter_convert_packed_fb_to_ink
//
// Converts one row of packed 2bpp framebuffer data into EPD ink drive bytes
// for one waveform pass. Only chunks whose bit is set in chunk_flags are
// processed; others have their pointers advanced without writing.
//
// Parameters:
//   a2 - Input pointer  (packed 2bpp framebuffer row)
//   a3 - Output pointer (ink drive data)
//   a4 - Length in bytes (multiple of 16)
//   a5 - Pointer to 3-byte waveform entry (one byte per pixel value 1/2/3)
//   a6 - chunk_flags (bit 31 = first chunk; consumed MSB-first each iteration)
// =============================================================================
    .text
    .align 4
    .global epd_painter_convert_packed_fb_to_ink
    .type   epd_painter_convert_packed_fb_to_ink, @function

epd_painter_convert_packed_fb_to_ink:
    entry       a1, 32

    // SAR=1 for ee.vsr.32 (1-bit right shift)
    movi        a7, 1
    wsr.sar     a7

    // Broadcast 0x55 to q7: selects the lo (even) bit of each pixel pair
    movi        a10, 0b01010101
    s32i        a10, a1, 0
    ee.vldbc.8  q7, a1

    // Load the three waveform bytes into broadcast registers q4, q5, q6
    // (q4 = waveform for pixel value 3, q5 = value 2, q6 = value 1)
    ee.vldbc.8.ip   q4, a5, 1
    ee.vldbc.8.ip   q5, a5, 1
    ee.vldbc.8.ip   q6, a5, 1

.convert_loop:
    // Consume the MSB of chunk_flags to decide whether to process this chunk
    extui       a9, a6, 31, 1
    slli        a6, a6, 1

    beqz        a9, .convert_skip

    // --- Active path: process this 16-byte chunk ---
    ee.vld.128.ip   q0, a2, 16

    // Separate the lo bit (bit0) and hi bit (bit1) of each pixel pair
    ee.andq         q3, q0, q7      // q3 = lo bits
    ee.vsr.32       q2, q0          // shift right to bring bit1 down to bit0 position
    ee.andq         q2, q2, q7      // q2 = hi bits

    // Build a presence mask for each pixel value:
    //   q1 = pixels with value 3 (lo=1 AND hi=1)
    //   q0 = pixels with value 2 (lo=0 AND hi=1)
    //   q2 = pixels with value 1 (lo=1 AND hi=0)
    ee.andq         q1, q3, q2
    ee.notq         q0, q3
    ee.andq         q0, q2, q0
    ee.notq         q2, q2
    ee.andq         q2, q3, q2

    // Expand each mask to cover both bit positions of its pixel pair
    ee.vsl.32       q3, q1
    ee.orq          q3, q3, q1

    ee.vsl.32       q1, q0
    ee.orq          q0, q0, q1

    ee.vsl.32       q1, q2
    ee.orq          q2, q2, q1

    // AND each expanded mask with its waveform byte and OR all three together
    ee.andq         q3, q4, q3
    ee.andq         q0, q5, q0
    ee.orq          q3, q3, q0
    ee.andq         q2, q6, q2
    ee.orq          q3, q3, q2

    ee.vst.128.ip   q3, a3, 16
    j               .convert_next

.convert_skip:
    // Chunk not active: advance both pointers without writing
    addi        a2, a2, 16
    addi        a3, a3, 16

.convert_next:
    addi        a4, a4, -16
    bnez        a4, .convert_loop

    retw.n


// =============================================================================
// epd_painter_ink
//
// Delta-detects changed pixels between the new framebuffer and the current
// screen state, writes drive data into both buffers, and returns an updated
// chunk_flags bitmask indicating which chunks need driving.
//
// For each chunk the flags bit determines which waveform is tried first:
//   bit=0 → try on-waveform first  (pixels to lighten)
//   bit=1 → try off-waveform first (pixels to darken)
// If the first waveform produces no driven pixels, the bit is restored and
// the other waveform is tried. If both are empty, q7 (zero) is stored.
//
// Parameters:
//   a2 - packed_fastbuffer row  (new desired pixel state)
//   a3 - packed_screenbuffer row (current physical screen state)
//   a4 - Length in bytes (multiple of 16)
//   a5 - Initial chunk_flags bitmask
//
// Returns (a2): updated chunk_flags
// =============================================================================
    .global epd_painter_ink
    .type   epd_painter_ink, @function
    .text
    .align 4

epd_painter_ink:
    entry           a1, 32

    // SAR=1 for ee.vsl.32 / ee.vsr.32 (1-bit shifts)
    movi            a10, 1
    wsr.sar         a10

    // q6 = 0x55 broadcast (lo bit mask), q5 = 0xAA broadcast (hi bit mask)
    movi            a10, 0b01010101
    s32i            a10, a1, 0
    ee.vldbc.8      q6, a1
    ee.vsl.32       q5, q6

    ee.zero.q       q7              // q7 = zero (used for all-zero chunk stores)

    mov             a11, a5         // working copy of chunk_flags

    // a12 = rotating single-bit mask, starts at bit 31
    movi            a12, 31
    ssl             a12
    movi            a12, 1
    sll             a12, a12

    movi            a10, 1
    wsr.sar         a10

.ink_loop:
    mov             a13, a2         // save current fastbuffer row pointer

    // Test and clear the current chunk's bit in the flags
    and             a10, a11, a12
    xor             a11, a11, a12

    bnez            a10, .ink_off_first

.ink_on:
    // On-waveform path: find pixels that need lightening.
    // Only fastbuffer pixels where the screen is currently zero are driven on.
    ee.vld.128.ip   q2, a2, 16      // q2 = new (fastbuffer)
    ee.vld.128.ip   q3, a3, 16      // q3 = current screen

    // Spread each screen pixel's value across both its bit positions,
    // then invert to get a mask of pixels that are currently zero on screen
    ee.vsl.32       q1, q3
    ee.andq         q1, q1, q5
    ee.orq          q0, q3, q1
    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q6
    ee.orq          q0, q0, q1
    ee.notq         q0, q0          // q0: both bits set wherever screen pixel == 0

    ee.andq         q2, q2, q0      // keep only fastbuffer pixels where screen is zero

    // Check whether any pixels need driving (OR all 4 lanes into a scalar)
    ee.movi.32.a    q2, a10, 0
    ee.movi.32.a    q2, a14, 1
    or              a10, a10, a14
    ee.movi.32.a    q2, a14, 2
    or              a10, a10, a14
    ee.movi.32.a    q2, a14, 3
    or              a10, a10, a14

    beqz            a10, .bit0_on_empty

    // Write on-drive data to fastbuffer; merge into screenbuffer
    ee.vst.128.ip   q2, a13, 0

    addi            a10, a3, -16    // back up to the screenbuffer chunk just loaded
    ee.vld.128.ip   q3, a10, 0
    ee.orq          q3, q3, q2
    ee.vst.128.ip   q3, a10, 0

    j               .ink_next

.bit0_on_empty:
    // On-waveform produced nothing; restore the flag bit and
    // fall through to the off-waveform to handle this chunk instead
    xor             a11, a11, a12

    ee.vld.128.ip   q2, a13, 0
    addi            a10, a3, -16
    ee.vld.128.ip   q3, a10, 0

    ee.xorq         q0, q2, q3

    ee.vsl.32       q1, q0
    ee.andq         q1, q1, q5
    ee.orq          q0, q0, q1
    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q6
    ee.orq          q0, q0, q1

    ee.andq         q4, q3, q0
    ee.vst.128.ip   q4, a13, 0

    ee.notq         q0, q0
    ee.andq         q3, q3, q0
    addi            a10, a3, -16
    ee.vst.128.ip   q3, a10, 0

    j               .ink_next

.ink_off_first:
    // Off-waveform path: find pixels that need darkening.
    // Pixels that differ between new and current screen state are driven off.
    ee.vld.128.ip   q2, a2, 16      // q2 = new (fastbuffer)
    ee.vld.128.ip   q3, a3, 16      // q3 = current screen

    // Spread changed pixels across both bit positions
    ee.xorq         q0, q2, q3
    ee.vsl.32       q1, q0
    ee.andq         q1, q1, q5
    ee.orq          q0, q0, q1
    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q6
    ee.orq          q0, q0, q1

    ee.andq         q4, q3, q0      // screen pixels that differ = need driving off

    // Check whether any pixels need driving
    ee.movi.32.a    q4, a10, 0
    ee.movi.32.a    q4, a14, 1
    or              a10, a10, a14
    ee.movi.32.a    q4, a14, 2
    or              a10, a10, a14
    ee.movi.32.a    q4, a14, 3
    or              a10, a10, a14

    beqz            a10, .bit1_off_empty

    ee.vst.128.ip   q4, a13, 0

    ee.notq         q0, q0
    ee.andq         q3, q3, q0
    addi            a10, a3, -16    // back up to the screenbuffer chunk just loaded
    ee.vst.128.ip   q3, a10, 0

    j               .ink_next

.bit1_off_empty:
    // Off-waveform produced nothing; restore the flag bit and
    // fall through to the on-waveform to handle this chunk instead
    xor             a11, a11, a12

    ee.vld.128.ip   q2, a13, 0
    addi            a10, a3, -16
    ee.vld.128.ip   q3, a10, 0

    ee.vsl.32       q1, q3
    ee.andq         q1, q1, q5
    ee.orq          q0, q3, q1
    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q6
    ee.orq          q0, q0, q1

    ee.notq         q0, q0
    ee.andq         q2, q2, q0

    ee.movi.32.a    q2, a10, 0
    ee.movi.32.a    q2, a14, 1
    or              a10, a10, a14
    ee.movi.32.a    q2, a14, 2
    or              a10, a10, a14
    ee.movi.32.a    q2, a14, 3
    or              a10, a10, a14

    beqz            a10, .bit1_on_empty

    ee.vst.128.ip   q2, a13, 0

    addi            a10, a3, -16    // back up to the screenbuffer chunk just loaded
    ee.vld.128.ip   q3, a10, 0
    ee.orq          q3, q3, q2
    ee.vst.128.ip   q3, a10, 0

    j               .ink_next

.bit1_on_empty:
    // Both waveforms produced nothing for this chunk; write zero so the
    // output word is defined rather than left with stale data
    ee.vst.128.ip   q7, a13, 0

.ink_next:
    srli            a12, a12, 1
    addi            a4, a4, -16
    bnez            a4, .ink_loop

    mov             a2, a11
    retw.n
#endif
