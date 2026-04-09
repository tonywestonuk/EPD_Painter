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
// =============================================================================
// epd_sr_push_fast
//
// Clocks one byte MSB-first into a 74HCT4094D shift register using
// EE.WR_MASK_GPIO_OUT — atomic masked fast-GPIO writes.
//
// Timing: one NOP (≈4 ns at 240 MHz) between each GPIO transition is the
// empirically determined minimum — 0 NOPs fails.  The fast-GPIO interface
// bypasses the AHB bus, so the chip runs well beyond its 20 ns datasheet
// spec.  If reliability issues appear, add a second NOP after each CLK edge.
//
// Parameters:
//   a2 — byte to clock out
//   a3 — pointer to EPD_SRMasks:
//          [+0]  data_mask      (uint32_t)  DATA pin bitmask
//          [+4]  clk_mask       (uint32_t)  CLK pin bitmask
//          [+8]  str_mask       (uint32_t)  STR pin bitmask
//          [+12] clkdata_mask   (uint32_t)  CLK | DATA bitmask
//
// AR register assignments (held for the duration of the function):
//   a4 — data_mask    (value AND mask for DATA-only writes)
//   a5 — clk_mask     (value AND mask for CLK-only writes)
//   a6 — str_mask     (value AND mask for STROBE writes)
//   a7 — clkdata_mask (mask for combined CLK+DATA writes)
//   a8 — 0            (value for driving pins low)
//
// EE.WR_MASK_GPIO_OUT semantics:
//   GPIO_OUT = (GPIO_OUT & ~mask) | (value & mask)
//   Both operands are AR registers.
// =============================================================================
    // -----------------------------------------------------------------------
    // Assembler directives — tell the linker where to place this function.
    // -----------------------------------------------------------------------
    .section .iram1,"ax"    // Place in Internal RAM (IRAM), not flash.
                            // IRAM code runs at full CPU speed with no cache
                            // miss stalls.  Flash-cached code would add ~100 ns
                            // of latency on a cold miss — fatal for tight GPIO
                            // bit-bang timing.
    .align 4                // Align the function to a 4-byte boundary.
                            // Xtensa fetches instructions in 4-byte chunks;
                            // misalignment wastes a fetch cycle on entry.
    .global epd_sr_push_fast  // Make the symbol visible to the C++ linker so
                              // the extern "C" declaration in epd_pin_driver.h
                              // can find it.
    .type   epd_sr_push_fast, @function  // Mark as a function (vs data) for
                                         // debug info and stack unwinding.

epd_sr_push_fast:           // Function entry point label.

    // -----------------------------------------------------------------------
    // Xtensa uses a WINDOWED register file.  'entry' rotates the register
    // window and allocates stack space.
    //
    //   entry a1, 32
    //         ^^  ^^
    //         |   └─ Reserve 32 bytes of stack space (enough for a2-a15 spill
    //         |      if a deeper call needs them, though we make no calls here).
    //         └─ a1 is the stack pointer — entry adjusts it by -32.
    //
    // After 'entry', the CALLER's a2 and a3 are visible to us as our a2/a3.
    // Xtensa's calling convention passes the first two arguments in a2/a3:
    //   a2 = byte to clock out  (uint8_t, zero-extended to 32 bits)
    //   a3 = pointer to EPD_SRMasks struct
    // -----------------------------------------------------------------------
    entry   a1, 32

    // -----------------------------------------------------------------------
    // Load the four mask values from the EPD_SRMasks struct into registers.
    // We keep them in registers for the whole function to avoid re-loading
    // from memory on every bit.
    //
    //   l32i  dst, base, byte_offset
    //         ^^^  ^^^^  ^^^^^^^^^^^
    //         |    |     └─ Byte offset into the struct (0, 4, 8, 12)
    //         |    └─ Base address register (a3 = &_masks)
    //         └─ Destination register
    //
    // The struct layout matches EPD_SRMasks exactly:
    //   offset  0 → data_mask    = (1 << 0) = 0x01
    //   offset  4 → clk_mask     = (1 << 1) = 0x02
    //   offset  8 → str_mask     = (1 << 2) = 0x04
    //   offset 12 → clkdata_mask = (1 << 0)|(1 << 1) = 0x03
    //
    // These are fast-GPIO PORT BIT MASKS, not physical GPIO pin numbers.
    // Physical pins were routed to fast-GPIO bits 0/1/2 via the GPIO Matrix
    // in EPD_ShiftReg::_reconfigure_fast_gpio().
    // -----------------------------------------------------------------------
    l32i    a4, a3, 0    // a4 = data_mask    (bit 0 of the fast-GPIO port = DATA pin)
    l32i    a5, a3, 4    // a5 = clk_mask     (bit 1 of the fast-GPIO port = CLK pin)
    l32i    a6, a3, 8    // a6 = str_mask     (bit 2 of the fast-GPIO port = STR pin)
    l32i    a7, a3, 12   // a7 = clkdata_mask (bits 0+1 — used to touch both DATA and CLK atomically)
    movi    a8, 0        // a8 = 0            (constant zero — used to CLEAR pins)

    // -----------------------------------------------------------------------
    // Pull STROBE low before we start shifting.
    // The 74HCT4094D latches its shift register to the parallel outputs on a
    // RISING STROBE edge.  Holding STR low during shifting prevents any
    // partial byte from being latched mid-transfer.
    //
    //   ee.wr_mask_gpio_out  value_reg, mask_reg
    //
    // Effect: fast_gpio_port = (fast_gpio_port & ~mask) | (value & mask)
    //   mask  = a6 = str_mask = 0x04  → only the STR bit is affected
    //   value = a8 = 0               → STR bit is driven to 0
    // -----------------------------------------------------------------------
    ee.wr_mask_gpio_out  a8, a6     // STR = 0

    // -----------------------------------------------------------------------
    // Clock out 8 bits, MSB (bit 7) first.
    //
    // The 74HCT4094D samples its DS (data serial) input on the RISING edge
    // of CP (clock).  So for each bit we:
    //   1. Set DATA to the bit value while keeping CLK low.
    //   2. Pulse CLK high then low.
    //
    // Each bit follows this structure:
    //
    //   bbci  a2, N, .bN_lo        — "Branch if Bit Clear Immediate"
    //                                 Tests bit N of register a2 (our byte).
    //                                 If the bit is 0, jump to .bN_lo.
    //                                 If the bit is 1, fall through.
    //
    //   [bit=1 path]
    //   ee.wr_mask_gpio_out a4, a7  — value=data_mask, mask=clkdata_mask
    //                                 Sets DATA=1, forces CLK=0 atomically.
    //   j .bN_clk                   — Unconditional jump, skip the bit=0 path.
    //
    //   [bit=0 path]
    //   .bN_lo:
    //   ee.wr_mask_gpio_out a8, a7  — value=0, mask=clkdata_mask
    //                                 Clears DATA=0 and CLK=0 atomically.
    //
    //   [both paths converge here]
    //   .bN_clk:
    //   nop                         — One pipeline bubble (~4 ns) to meet the
    //                                 74HCT4094D's DATA setup time before CLK↑.
    //   ee.wr_mask_gpio_out a5, a5  — value=clk_mask, mask=clk_mask
    //                                 CLK = 1  (rising edge — chip samples DATA here)
    //   nop                         — One bubble (~4 ns) to meet CLK high pulse width.
    //                                 The NEXT bit's DATA write will bring CLK back low.
    // -----------------------------------------------------------------------

    // --- Bit 7 (MSB) ---
    bbci    a2, 7, .b7_lo           // if bit 7 of byte == 0, jump to .b7_lo
    ee.wr_mask_gpio_out  a4, a7     // bit=1: DATA=1, CLK=0
    j       .b7_clk                 // skip the bit=0 path
.b7_lo:
    ee.wr_mask_gpio_out  a8, a7     // bit=0: DATA=0, CLK=0
.b7_clk:
    nop                             // DATA setup hold before CLK rises
    ee.wr_mask_gpio_out  a5, a5     // CLK=1  ← chip latches DATA on this edge

    // --- Bit 6 ---
    bbci    a2, 6, .b6_lo
    ee.wr_mask_gpio_out  a4, a7     // bit=1: DATA=1, CLK=0
    j       .b6_clk
.b6_lo:
    ee.wr_mask_gpio_out  a8, a7     // bit=0: DATA=0, CLK=0
.b6_clk:
    nop
    ee.wr_mask_gpio_out  a5, a5     // CLK=1

    // --- Bit 5 ---
    bbci    a2, 5, .b5_lo
    ee.wr_mask_gpio_out  a4, a7
    j       .b5_clk
.b5_lo:
    ee.wr_mask_gpio_out  a8, a7
.b5_clk:
    nop
    ee.wr_mask_gpio_out  a5, a5

    // --- Bit 4 ---
    bbci    a2, 4, .b4_lo
    ee.wr_mask_gpio_out  a4, a7
    j       .b4_clk
.b4_lo:
    ee.wr_mask_gpio_out  a8, a7
.b4_clk:
    nop
    ee.wr_mask_gpio_out  a5, a5

    // --- Bit 3 ---
    bbci    a2, 3, .b3_lo
    ee.wr_mask_gpio_out  a4, a7
    j       .b3_clk
.b3_lo:
    ee.wr_mask_gpio_out  a8, a7
.b3_clk:
    nop
    ee.wr_mask_gpio_out  a5, a5

    // --- Bit 2 ---
    bbci    a2, 2, .b2_lo
    ee.wr_mask_gpio_out  a4, a7
    j       .b2_clk
.b2_lo:
    ee.wr_mask_gpio_out  a8, a7
.b2_clk:
    nop
    ee.wr_mask_gpio_out  a5, a5

    // --- Bit 1 ---
    bbci    a2, 1, .b1_lo
    ee.wr_mask_gpio_out  a4, a7
    j       .b1_clk
.b1_lo:
    ee.wr_mask_gpio_out  a8, a7
.b1_clk:
    nop
    ee.wr_mask_gpio_out  a5, a5

    // --- Bit 0 (LSB) ---
    bbci    a2, 0, .b0_lo
    ee.wr_mask_gpio_out  a4, a7
    j       .b0_clk
.b0_lo:
    ee.wr_mask_gpio_out  a8, a7
.b0_clk:
    nop
    ee.wr_mask_gpio_out  a5, a5

    // -----------------------------------------------------------------------
    // All 8 bits have been clocked in.  Bring CLK low (it was left high after
    // the last bit's CLK=1), then pulse STROBE high to latch the shift
    // register contents into the 74HCT4094D's parallel output register.
    // The parallel outputs (QP1-QP8) update on the rising STROBE edge.
    // -----------------------------------------------------------------------
    ee.wr_mask_gpio_out  a8, a5     // CLK=0  (idle state between transfers)
    nop                             // CLK low hold time before STR rises
    ee.wr_mask_gpio_out  a6, a6     // STR=1  ← parallel outputs update here

    // -----------------------------------------------------------------------
    // Return to caller.  'retw.n' is the compact (2-byte) windowed return:
    // it restores the register window that 'entry' rotated on the way in,
    // making the caller's registers visible again.
    // -----------------------------------------------------------------------
    retw.n

#endif
