// =============================================================================
// EPD Painter Assembly Functions
// Optimized routines for e-paper display pixel manipulation
//
// PIXEL FORMAT OVERVIEW:
//   The GFX canvas stores pixels as 1 byte each (8bpp).
//   This driver packs them down to 2 bits per pixel (2bpp):
//     00 = white (unpowered)
//     01 = light grey
//     10 = dark grey
//     11 = black
//
//   The "ink" format sent to the EPD encodes each 2bpp pixel as two
//   independent drive bits across two parallel output lines:
//     Bit 1 (high): drives the positive electrode
//     Bit 0 (low):  drives the negative electrode
//   The waveform tables define what drive pattern each pixel value
//   should receive on each of the 6 passes.
//
// SCREEN BUFFER:
//   packed_screenbuffer tracks what is currently physically displayed
//   on the panel (2bpp packed format, same as framebuffer).
//   ink_on / ink_off use this to decide what actually needs to change.
//
// INTERLACING:
//   The EPD panel is driven in an interlaced fashion — on alternating
//   frames, even rows are driven with the "darker" waveform and odd
//   rows with "lighter", then they swap. 
// =============================================================================

    .text
    .align 4

// =============================================================================
// epd_painter_compact_pixels
//
// Converts the raw GFX canvas (1 byte per pixel) into a packed 2bpp
// framebuffer (4 pixels per byte), using only the 2 LSBs of each input byte.
//
// Input byte:  [xxxxxx PP]  (P = the 2 meaningful pixel bits)
// Output byte: [P3P3 P2P2 P1P1 P0P0]  ← 4 pixels packed, MSB first
//
// Processes 16 input bytes → 4 output bytes per loop iteration.
//
// Parameters:
//   a2 - Input pointer  (1 byte per pixel, from GFX canvas)
//   a3 - Output pointer (2 bits per pixel, packed destination)
//   a4 - Size in bytes  (must be a multiple of 16, i.e. width*height)
//
// Registers used:
//   a10 - Scratch / temp value for building constants
//   a11 - Remaining pixel count (counts down in steps of 16)
//   q0  - Holds the 16 input pixels being processed
//   q1  - Shifted copy used during packing
//   q5  - Mask: 0x03 broadcast to all 16 bytes (isolates 2 LSBs)
//   q7  - Zero register (used as shift-in source for ee.src.q)
// =============================================================================
    .global epd_painter_compact_pixels
    .type   epd_painter_compact_pixels, @function

epd_painter_compact_pixels:
    entry   a1, 32

    // -------------------------------------------------------------------------
    // Initialisation
    // -------------------------------------------------------------------------

    // Zero out q7 — used as a source of zero bytes in ee.src.q shifts
    ee.zero.q       q7

    // Build the 2-LSB mask: store 0x03 to stack then broadcast to all
    // 16 bytes of q5, giving q5 = [03 03 03 03 ... 03]
    movi            a10, 0b00000011
    s32i            a10, a1, 0
    ee.vldbc.8      q5, a1

    // Configure the hardware shift registers used by ee.src.q and ee.vsl.32:
    //   SAR_BYTE = 1  → ee.src.q shifts by 1 byte at a time
    //   SAR      = 2  → ee.vsl.32 shifts left by 2 bits within each 32-bit lane
    movi            a10, 1
    wur.sar_byte    a10
    movi            a10, 2
    wsr.sar         a10

    // -------------------------------------------------------------------------
    // Main loop: 16 input bytes → 4 packed output bytes
    // -------------------------------------------------------------------------
    mov             a11, a4             // a11 = total pixel count, counts down

.compact_process_pixels:

    // Load 16 pixels from input (1 byte each) into q0, advance a2 by 16
    ee.vld.128.ip   q0, a2, 16

    // Mask off upper 6 bits of each byte — keep only the 2 pixel bits.
    // q0 = [00PP 00PP 00PP ... 00PP]  (16 bytes)
    ee.andq         q0, q0, q5

    // Now pack 4 pixels into each output byte using a series of
    // shift-and-OR operations. Each step moves the next pixel's bits
    // into position alongside the previous ones.
    //
    // ee.src.q shifts the 128-bit register right by SAR_BYTE (1 byte),
    // filling from q7 (zeros) on the left. Combined with ee.vsl.32
    // (left shift by SAR=2 bits within each 32-bit lane), this slides
    // the accumulated pixel bits left to make room for the next pixel.
    //
    // After 3 iterations, each 32-bit lane holds 4 packed pixels in
    // its most significant byte: [P3P2P1P0 xx xx xx]

    // Step 1: shift q0 right by 1 byte into q1, shift q0 left by 2 bits,
    // then OR together — merges pixels 0 and 1 within each lane
    ee.src.q        q1, q0, q7         // q1 = q0 >> 8 (byte shift, zero fill)
    ee.vsl.32       q0, q0             // q0 <<= 2 bits (per 32-bit lane)
    ee.orq          q0, q0, q1         // q0 |= q1 → pixels 0+1 combined

    // Step 2: merge pixel 2
    ee.vsl.32       q0, q0             // shift accumulated bits left 2 more
    ee.src.q        q1, q1, q7         // shift q1 another byte right
    ee.orq          q0, q0, q1         // merge → pixels 0+1+2 combined

    // Step 3: merge pixel 3
    ee.vsl.32       q0, q0             // shift left 2 more
    ee.src.q        q1, q1, q7         // shift q1 another byte right
    ee.orq          q0, q0, q1         // merge → all 4 pixels combined per lane

    // At this point each 32-bit lane of q0 has 4 packed pixels in its
    // top byte, with garbage in the lower bytes. The two ee.vunzip.8
    // calls reorganise the register so that all 4 useful bytes end up
    // contiguous at the bottom of q0, ready for a 32-bit store.
    ee.vunzip.8     q0, q1
    ee.vunzip.8     q0, q1

    // Store the 4 packed output bytes, then advance output pointer by 4
    ee.stxq.32      q0, q7, a3, 0, 0
    addi            a3, a3, 4

    // Decrement pixel counter by 16 and loop if more pixels remain
    addi            a11, a11, -16
    bnez            a11, .compact_process_pixels

    retw.n



// =============================================================================
// epd_painter_convert_packed_fb_to_ink
//
// Converts packed 2bpp framebuffer into "ink" drive format for one
// waveform pass. Only processes 64-pixel chunks whose corresponding
// bit is set in the chunk_flags parameter.
//
// When writing output, existing non-zero data in the output buffer is
// preserved — only pixels that are currently 00 in the output are written.
// This allows two calls (one for lighten waveforms, one for darken) to
// accumulate into the same output buffer without overwriting each other.
//
// Parameters:
//   a2 - Input pointer  (packed 2bpp framebuffer row)
//   a3 - Output pointer (ink drive data — may contain data from prior call)
//   a4 - Length in bytes (packed row size, multiple of 16)
//   a5 - Pointer to 3-byte waveform entry for this pass
//   a6 - chunk_flags (1 bit per 64-pixel chunk, MSB = first chunk;
//         bit=1 means process this chunk, bit=0 means skip)
//
// Registers used:
//   a7  - Constant 1 (for SAR shift amount)
//   a9  - Scratch for flag testing
//   a10 - Scratch
//   q0  - Input pixel data / existing output for merge
//   q1  - Intermediate / pixel sub-masks
//   q2  - Shifted copy for mask spreading
//   q3  - Waveform output for this chunk
//   q4  - Waveform for pixel value 11 (broadcast)
//   q5  - Waveform for pixel value 10 (broadcast)
//   q6  - Waveform for pixel value 01 (broadcast)
//   q7  - Mask: 0x55 = 01010101 (isolates low bit of each 2bpp pixel)
// =============================================================================
    .global epd_painter_convert_packed_fb_to_ink
    .type   epd_painter_convert_packed_fb_to_ink, @function
    .align 4

epd_painter_convert_packed_fb_to_ink:
    entry           a1, 32

    movi            a7, 1
    wsr.sar         a7

    // Build 0x55 mask broadcast to all bytes of q7
    movi            a10, 0b01010101
    s32i            a10, a1, 0
    ee.vldbc.8      q7, a1

    // Load and broadcast the 3 waveform bytes
    ee.vldbc.8.ip   q4, a5, 1          // waveform for 11 pixels
    ee.vldbc.8.ip   q5, a5, 1          // waveform for 10 pixels
    ee.vldbc.8.ip   q6, a5, 1          // waveform for 01 pixels

.convert_loop:

    // --- Check flag for this 64-pixel chunk ---
    extui           a9, a6, 31, 1
    slli            a6, a6, 1

    beqz            a9, .convert_skip

    // --- Process this chunk ---
    ee.vld.128.ip   q0, a2, 16

    ee.zero.q       q3

    // ----- Handle pixels with value 11 (both bits set) -----
    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q7
    ee.andq         q1, q0, q1
    ee.vsl.32       q2, q1
    ee.orq          q1, q1, q2

    ee.andq         q2, q4, q1
    ee.orq          q3, q3, q2

    // ----- Handle pixels with value 10 -----
    ee.xorq         q0, q0, q1

    ee.notq         q7, q7             // q7 = 0xAA temporarily
    ee.andq         q1, q0, q7
    ee.notq         q7, q7             // restore q7 = 0x55

    ee.vsr.32       q2, q1
    ee.orq          q1, q1, q2

    ee.andq         q2, q5, q1
    ee.orq          q3, q3, q2

    // ----- Handle pixels with value 01 -----
    ee.andq         q1, q0, q7
    ee.vsl.32       q2, q1
    ee.orq          q1, q1, q2

    ee.andq         q2, q6, q1
    ee.orq          q3, q3, q2

    // --- Merge into output buffer without overwriting existing data ---
    ee.vld.128.ip   q0, a3, 0          // q0 = existing output (don't advance yet)

    // Build mask of occupied slots in existing output
    ee.vsl.32       q1, q0
    ee.notq         q7, q7             // q7 = 0xAA temporarily
    ee.andq         q1, q1, q7
    ee.notq         q7, q7             // restore q7 = 0x55
    ee.orq          q1, q0, q1

    ee.vsr.32       q2, q1
    ee.andq         q2, q2, q7
    ee.orq          q1, q1, q2

    // Invert: 11 where output slot is free (00)
    ee.notq         q1, q1

    // Mask new waveform data to free slots only, merge, store
    ee.andq         q3, q3, q1
    ee.orq          q3, q3, q0
    ee.vst.128.ip   q3, a3, 16

    j               .convert_next

.convert_skip:
    // Skip: advance pointers without processing
    addi            a2, a2, 16
    addi            a3, a3, 16

.convert_next:
    addi            a4, a4, -16
    bnez            a4, .convert_loop

    retw.n


// =============================================================================
// epd_painter_ink_on
//
// Determines which pixels need to TURN ON (become grey/black) and records
// them in both the output buffer and the screen buffer.
//
// A pixel is eligible to turn on only if it is currently WHITE (00) in
// the screen buffer. For eligible pixels, the new pixel value from
// packed_fastbuffer is copied to the output and written into the screen
// buffer so future calls know the pixel is no longer white.
//
// Returns a 32-bit flag word: one bit per 64-pixel (16-byte) chunk,
// set to 1 if any non-zero pixel was written to output for that chunk.
// The first chunk processed corresponds to the MSB of the returned word.
//
// Parameters:
//   a2 - packed_fastbuffer (new desired pixel values, 2bpp packed)
//   a3 - packed_screenbuffer (current physical display state, read+write)
//   a4 - Output pointer (pixels to drive in this pass)
//   a5 - Length in bytes (total packed bytes to process, must be multiple of 16)
//
// Returns:
//   a2 - 32-bit flag register (1 bit per 64-pixel chunk, MSB = first chunk)
//
// Registers used:
//   a10 - Scratch
//   a11 - Accumulated flag register
//   q0  - Mask: identifies writable (white) pixels
//   q1  - Temporary / shifted copy
//   q2  - Data loaded from packed_fastbuffer (desired new pixels)
//   q3  - Data loaded from packed_screenbuffer (current screen state)
//   q4  - Output: masked new pixel data to drive
//   q5  - Mask 0xAA = 10101010 (isolates HIGH bit of each 2bpp pixel)
//   q6  - Mask 0x55 = 01010101 (isolates LOW bit of each 2bpp pixel)
// =============================================================================
.global epd_painter_ink_on
    .type   epd_painter_ink_on, @function
    .text
    .align 4

epd_painter_ink_on:
    entry           a1, 32

    movi            a10, 1
    wsr.sar         a10

    // Build q6 = 0x55 (01010101) and q5 = 0xAA (10101010)
    movi            a10, 0b01010101
    s32i            a10, a1, 0
    ee.vldbc.8      q6, a1
    ee.vsl.32       q5, q6

    // Clear flag accumulator
    movi            a11, 0

    // a7 = number of chunks (for final shift)
    srli            a7, a5, 4

.ink_on_loop:

    ee.vld.128.ip   q2, a2, 16
    ee.vld.128.ip   q3, a3, 0

    // Build mask: spread each pixel's bits so any non-zero pixel gets both bits set
    ee.vsl.32       q1, q3
    ee.andq         q1, q1, q5
    ee.orq          q0, q3, q1

    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q6
    ee.orq          q0, q0, q1

    // Invert: 11 only where original pixel was 00 (white/writable)
    ee.notq         q0, q0

    // Mask new pixels to writable positions only
    ee.andq         q4, q2, q0

    // Write drive data to output
    ee.vst.128.ip   q4, a4, 16

    // Update screen buffer with newly written pixels
    ee.orq          q3, q3, q4
    ee.vst.128.ip   q3, a3, 16

    // --- Build flag bit for this 64-pixel chunk ---
    slli            a11, a11, 1

    ee.vst.128.ip   q4, a1, 0
    l32i            a10, a1, 0
    l32i            a9, a1, 4
    or              a10, a10, a9
    l32i            a9, a1, 8
    or              a10, a10, a9
    l32i            a9, a1, 12
    or              a10, a10, a9

    beqz            a10, .ink_on_no_flag
    addi            a11, a11, 1
.ink_on_no_flag:

    addi            a5, a5, -16
    bnez            a5, .ink_on_loop

    // Shift flags so first chunk aligns to bit 31
    // Currently flags occupy the lowest `num_chunks` bits.
    // We need to shift left by (32 - num_chunks).
    movi            a10, 32
    sub             a10, a10, a7        // a10 = 32 - num_chunks
    ssl             a10                 // set shift amount for SLL
    sll             a11, a11            // shift left by (32 - num_chunks)

    mov             a2, a11
    retw.n

// =============================================================================
// epd_painter_ink_off
//
// Determines which pixels need to TURN OFF (be cleared back to white) and
// records them in both the output buffer and the screen buffer.
//
// A pixel needs clearing if the desired framebuffer differs from the current
// screen state — it must pass through white before ink_on can set a new value.
//
// Chunks where ink_on already drove pixels (indicated by the skip_flags
// parameter) are skipped entirely — their pointers are advanced but no
// processing is done.
//
// Parameters:
//   a2 - packed_fastbuffer (new desired pixel values, 2bpp packed)
//   a3 - packed_screenbuffer (current physical display state, read+write)
//   a4 - Output pointer (pixels to drive in this pass)
//   a5 - Length in bytes (total packed bytes to process, must be multiple of 16)
//   a6 - skip_flags (1 bit per 64-pixel chunk, MSB = first chunk;
//         bit=1 means ink_on drove that chunk, so skip it here)
//
// Registers used:
//   a9  - Scratch for flag testing
//   a10 - Scratch
//   q0  - Mask: identifies changed pixels
//   q1  - Temporary / shifted copy
//   q2  - Data loaded from packed_fastbuffer (desired new pixels)
//   q3  - Data loaded from packed_screenbuffer (current screen state)
//   q5  - Mask 0xAA = 10101010 (isolates HIGH bit of each 2bpp pixel)
//   q6  - Mask 0x55 = 01010101 (isolates LOW bit of each 2bpp pixel)
// =============================================================================
    .global epd_painter_ink_off
    .type   epd_painter_ink_off, @function
    .text
    .align 4

epd_painter_ink_off:
    entry           a1, 32

    movi            a10, 1
    wsr.sar         a10

    // Build q6 = 0x55 (01010101) and q5 = 0xAA (10101010)
    movi            a10, 0b01010101
    s32i            a10, a1, 0
    ee.vldbc.8      q6, a1
    ee.vsl.32       q5, q6

.ink_off_loop:

    // --- Check skip flag for this 64-pixel chunk ---
    // Test the MSB of a6; if set, ink_on already handled this chunk
    extui           a9, a6, 31, 1
    slli            a6, a6, 1

    bnez            a9, .ink_off_skip

    // --- Process this chunk ---
    ee.vld.128.ip   q2, a2, 16         // q2 = desired new pixels
    ee.vld.128.ip   q3, a3, 0          // q3 = current screen state

    // Detect changed pixels: XOR gives 1 in any bit position that differs
    ee.xorq         q0, q2, q3

    // Spread the difference across both bits of each 2bpp pixel slot

    // Spread low bit -> high
    ee.vsl.32       q1, q0
    ee.andq         q1, q1, q5
    ee.orq          q0, q0, q1

    // Spread high bit -> low
    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q6
    ee.orq          q0, q0, q1

    // Write screen buffer value to output for changed pixels only
    // (drives pixel back toward white)
    ee.andq         q1, q3, q0
    ee.vst.128.ip   q1, a4, 16

    // Clear changed pixels from screen buffer (mark as white/00)
    ee.notq         q0, q0
    ee.andq         q3, q3, q0
    ee.vst.128.ip   q3, a3, 16

    j               .ink_off_next

.ink_off_skip:
    // Skip this chunk: advance all pointers without processing
    addi            a2, a2, 16
    addi            a3, a3, 16
    addi            a4, a4, 16

.ink_off_next:
    // Loop until all bytes processed
    addi            a5, a5, -16
    bnez            a5, .ink_off_loop

    retw.n

// =============================================================================
// epd_painter_interleaved_copy
//
// Copies every other row from source to destination, skipping alternate rows.
// Used to copy either even or odd rows depending on interlace_period.
//
// Parameters:
//   a2 - Input pointer  (source)
//   a3 - Output pointer (destination)
//   a4 - Width in bytes (packed bytes per row)
//   a5 - Height in rows (total, including skipped rows)
//   a6 - Interlace period: 0 = copy even rows, non-zero = copy odd rows
//
// Registers used:
//   a11 - Byte counter within a row
//   q0  - Data transfer register
// =============================================================================
    .global epd_painter_interleaved_copy
    .type   epd_painter_interleaved_copy, @function
    .align 4

epd_painter_interleaved_copy:
    entry           a1, 32

    // If starting on odd rows, advance both pointers past the first row
    beqz            a6, .interleave_line_loop
    add             a2, a2, a4
    add             a3, a3, a4

.interleave_line_loop:
    mov             a11, a4             // reset byte counter for this row

.interleave_copy_16:
    // Copy 16 bytes at a time using Q register load/store
    ee.vld.128.ip   q0, a2, 16
    ee.vst.128.ip   q0, a3, 16

    addi            a11, a11, -16
    bnez            a11, .interleave_copy_16

    // Skip the next row in both source and destination (interlacing)
    add             a2, a2, a4
    add             a3, a3, a4

    addi            a5, a5, -2          // consumed 2 rows (1 copied + 1 skipped)
    bnez            a5, .interleave_line_loop

    retw.n