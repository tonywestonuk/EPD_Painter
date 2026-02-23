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
//   rows with "lighter", then they swap. This reduces flicker and
//   distributes stress on the panel evenly.
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
// Converts one row of the packed 2bpp framebuffer into the "ink" drive
// format expected by the EPD panel's data lines for one waveform pass.
//
// Each 2bpp pixel value (00, 01, 10, 11) is mapped to a 2-bit drive
// pattern taken from the waveform table. The waveform defines whether
// each pixel should be driven positively, negatively, or left floating
// during this particular pass. This is how greyscale is achieved — the
// ratio of positive to negative drives across 6 passes determines the
// final shade.
//
// The waveform parameter points to 3 consecutive bytes:
//   Byte 0: drive pattern for pixels that are 11 (black / darkest)
//   Byte 1: drive pattern for pixels that are 10 (dark grey)
//   Byte 2: drive pattern for pixels that are 01 (light grey)
//   (pixels that are 00 = white are not driven)
//
// Each waveform byte is broadcast to all 16 positions in a Q register,
// then masked against the pixels that match that value.
//
// Processes 16 packed bytes (= 64 pixels) per loop iteration.
//
// Parameters:
//   a2 - Input pointer  (packed 2bpp framebuffer row)
//   a3 - Output pointer (ink drive data for this pass)
//   a4 - Length in bytes (packed row size, multiple of 16)
//   a5 - Pointer to 3-byte waveform entry for this row/pass
//
// Registers used:
//   a7  - Constant 1 (for SAR shift amount)
//   a10 - Scratch
//   q0  - Input pixel data (loaded from packed framebuffer)
//   q1  - Intermediate / pixel sub-masks
//   q2  - Shifted copy for mask spreading
//   q3  - Output accumulator (OR of all waveform contributions)
//   q4  - Waveform for pixel value 11 (broadcast to all bytes)
//   q5  - Waveform for pixel value 10 (broadcast to all bytes)
//   q6  - Waveform for pixel value 01 (broadcast to all bytes)
//   q7  - Mask: 0x55 = 01010101 (isolates low bit of each 2bpp pixel)
// =============================================================================
    .global epd_painter_convert_packed_fb_to_ink
    .type   epd_painter_convert_packed_fb_to_ink, @function
    .align 4

epd_painter_convert_packed_fb_to_ink:
    entry           a1, 32

    // SAR = 1: ee.vsr.32 / ee.vsl.32 shift by 1 bit within each 32-bit lane.
    // Used below to spread each pixel's low bit into the high bit position
    // (and vice versa) so we can build a full 2-bit mask from a single bit.
    movi            a7, 1
    wsr.sar         a7

    // Build 0x55 mask (01010101) broadcast to all bytes of q7.
    // This isolates the LOW bit of each 2bpp pixel pair in a packed byte.
    movi            a10, 0b01010101
    s32i            a10, a1, 0
    ee.vldbc.8      q7, a1

    // Load the 3 waveform bytes and broadcast each to a full Q register.
    // a5 advances by 1 byte each time (.ip = increment pointer).
    //   q4 = waveform for 11 pixels, broadcast to all 16 bytes
    //   q5 = waveform for 10 pixels, broadcast to all 16 bytes
    //   q6 = waveform for 01 pixels, broadcast to all 16 bytes
    ee.vldbc.8.ip   q4, a5, 1
    ee.vldbc.8.ip   q5, a5, 1
    ee.vldbc.8.ip   q6, a5, 1

    // Loop: process 16 packed bytes (64 pixels) per iteration
    srli            a4, a4, 4           // a4 = number of 16-byte chunks
    loopnez         a4, .convert_end    // use zero-overhead loop

    ee.vld.128.ip   q0, a2, 16         // load 16 packed bytes (64 pixels)

    // Clear output accumulator for this 16-byte block
    ee.zero.q       q3

    // ----- Handle pixels with value 11 (both bits set) -----
    //
    // A pixel is 11 if its LOW bit is 1 AND its HIGH bit is 1.
    // Strategy: shift q0 right by 1 bit to align the high bit with
    // the low bit position, AND with the original to confirm both are set,
    // then spread back to fill both bit positions of the 2bpp slot.
    //
    ee.vsr.32       q1, q0             // q1 = q0 >> 1 (high bit → low position)
    ee.andq         q1, q1, q7         // keep only low-bit positions → 1 where high bit was 1
    ee.andq         q1, q0, q1         // AND with q0 low bits → 1 only where BOTH bits are 1
                                        // q1 now has 1 in the low-bit position for each 11 pixel
    ee.vsl.32       q2, q1             // q2 = q1 << 1 → high-bit position for same pixels
    ee.orq          q1, q1, q2         // q1 = full 11 mask: both bits set for every 11 pixel

    // Stamp the waveform for 11 pixels onto the output:
    // q4 (broadcast waveform byte) ANDed with q1 (mask) gives the
    // drive pattern only where pixels are 11. OR into q3.
    ee.andq         q2, q4, q1
    ee.orq          q3, q3, q2

    // ----- Handle pixels with value 10 (high bit set, low bit clear) -----
    //
    // Remove the 11 pixels we already handled by XORing them out of q0,
    // leaving only 01 and 10 pixels. Then isolate the 10s.
    //
    ee.xorq         q0, q0, q1         // clear 11 pixels from q0; remaining = 01 and 10 pixels

    // q7 = 0x55 = 01010101. NOT(q7) = 0xAA = 10101010 → masks the HIGH bit of each 2bpp pixel.
    ee.notq         q7, q7             // q7 = 0xAA (temporarily)
    ee.andq         q1, q0, q7         // q1 = high bits only (10 pixels have high=1, low=0)
    ee.notq         q7, q7             // restore q7 = 0x55

    // Spread the high bit down into the low bit position so both bits of
    // the 2bpp slot are filled (turns 10 into 11 within the mask):
    ee.vsr.32       q2, q1             // q2 = q1 >> 1 → high bit into low position
    ee.orq          q1, q1, q2         // q1 = full mask for 10 pixels (both bits set)

    // Stamp waveform for 10 pixels:
    ee.andq         q2, q5, q1
    ee.orq          q3, q3, q2

    // ----- Handle pixels with value 01 (low bit set, high bit clear) -----
    //
    // After removing 11 pixels above, q0 contains only 01 and 10 pixels.
    // The 10 pixels have already been handled; now isolate the 01s.
    // 01 pixels: low bit = 1, high bit = 0. Use 0x55 mask to get low bits.
    //
    ee.andq         q1, q0, q7         // q1 = low bits of remaining pixels → 1 for 01 pixels
    ee.vsl.32       q2, q1             // q2 = q1 << 1 → spread up into high bit position
    ee.orq          q1, q1, q2         // q1 = full mask for 01 pixels (both bits set)

    // Stamp waveform for 01 pixels:
    ee.andq         q2, q6, q1
    ee.orq          q3, q3, q2

    // Store the completed 16-byte ink output for this chunk
    ee.vst.128.ip   q3, a3, 16

.convert_end:
    retw.n


// =============================================================================
// epd_painter_ink_on
//
// Determines which pixels need to TURN ON (become grey/black) and records
// them in both the output buffer and the screen buffer.
//
// A pixel is eligible to turn on only if it is currently WHITE (00) in
// the screen buffer — the EPD cannot directly transition between two
// non-white shades. If the screen buffer already shows a grey or black
// pixel there, it must be cleared to white first (via ink_off) before
// ink_on can set it to the desired shade.
//
// For eligible pixels (screen buffer = 00), the new pixel value from
// packed_fastbuffer is copied to the output and written into the screen
// buffer so future calls know the pixel is no longer white.
//
// Operates on interleaved rows: processes every other row (even or odd
// depending on interlace_period), skipping the alternate row each time.
// This matches the interlaced waveform drive used in paint().
//
// Parameters:
//   a2 - packed_fastbuffer (new desired pixel values, 2bpp packed)
//   a3 - packed_screenbuffer (current physical display state, read+write)
//   a4 - Output pointer (pixels to drive in this pass)
//   a5 - Width in bytes (packed bytes per row = display_width / 4)
//   a6 - Height in rows
//   a7 - Interlace period: 0 = start on even rows, non-zero = start on odd
//
// Registers used:
//   a10 - Scratch
//   a11 - Byte counter within a row (counts down in steps of 16)
//   q0  - Mask: identifies pixels where screen_buffer is non-zero (occupied)
//   q1  - Temporary / shifted copy
//   q2  - Data loaded from packed_fastbuffer (desired new pixels)
//   q3  - Data loaded from packed_screenbuffer (current screen state)
//   q4  - Output: masked new pixel data to drive
//   q5  - Mask 0xAA = 10101010 (isolates HIGH bit of each 2bpp pixel)
//   q6  - Mask 0x55 = 01010101 (isolates LOW bit of each 2bpp pixel)
//   q7  - Zero (unused directly but initialised)
// =============================================================================
    .global epd_painter_ink_on
    .type   epd_painter_ink_on, @function
    .text
    .align 4

epd_painter_ink_on:
    entry           a1, 32

    // SAR = 1: shifts used below are 1 bit within each 32-bit lane
    movi            a10, 1
    wsr.sar         a10

    // Build q6 = 0x55 (01010101) and q5 = 0xAA (10101010)
    movi            a10, 0b01010101
    s32i            a10, a1, 0
    ee.vldbc.8      q6, a1             // q6 = 0x55 = low-bit mask per pixel
    ee.vsl.32       q5, q6             // q5 = 0xAA = high-bit mask per pixel

    ee.zero.q       q7
    mov             a11, a5            // a11 = row width in bytes

    // If interlace_period is non-zero, skip the first row so we start
    // on odd rows instead of even rows
    beqz            a7, .ink_on_loop
    add             a2, a2, a5
    add             a3, a3, a5
    add             a4, a4, a5

.ink_on_loop:

    // Load 16 bytes from the new framebuffer and screen buffer (no advance on a3 yet)
    ee.vld.128.ip   q2, a2, 16         // q2 = desired new pixels
    ee.vld.128.ip   q3, a3, 0          // q3 = current screen state (read without advancing)

    // Build a mask identifying pixels where the screen buffer is NOT 00
    // (i.e. pixels that are already grey or black — cannot be overwritten).
    //
    // Strategy: spread each pixel's bits so that if EITHER bit is set,
    // BOTH bits become set. Then invert → we get 11 only where both bits
    // were originally 00 (i.e. the pixel is white and available).

    // Spread low bit → high: shift left 1, mask to high-bit positions, OR back
    ee.vsl.32       q1, q3             // q1 = q3 << 1
    ee.andq         q1, q1, q5         // keep only high-bit positions (0xAA mask)
    ee.orq          q0, q3, q1         // q0 = q3 with low bit spread to high position

    // Spread high bit → low: shift right 1, mask to low-bit positions, OR back
    ee.vsr.32       q1, q0             // q1 = q0 >> 1
    ee.andq         q1, q1, q6         // keep only low-bit positions (0x55 mask)
    ee.orq          q0, q0, q1         // q0 = both bits set for any non-zero pixel

    // Invert: q0 now has 11 only where the original screen pixel was 00 (white)
    // These are the pixels we are allowed to write
    ee.notq         q0, q0

    // Copy the new pixel values, but only for allowed (white) pixels
    ee.andq         q4, q2, q0         // q4 = new pixels masked to writable positions only

    // Write drive data to output
    ee.vst.128.ip   q4, a4, 16

    // Update screen buffer: OR in the newly written pixels so future calls
    // know these positions are no longer white
    ee.orq          q3, q3, q4
    ee.vst.128.ip   q3, a3, 16         // write updated screen buffer, advance pointer

    // Process remaining bytes in this row
    addi            a11, a11, -16
    bnez            a11, .ink_on_loop

    // End of row: skip the NEXT row (interlacing — we only drive every other row)
    mov             a11, a5             // reset byte counter for next active row
    add             a2, a2, a5          // skip one row in source
    add             a3, a3, a5          // skip one row in screen buffer
    add             a4, a4, a5          // skip one row in output

    addi            a6, a6, -2          // we consumed 2 rows (1 active + 1 skipped)
    bnez            a6, .ink_on_loop

    retw.n


// =============================================================================
// epd_painter_ink_off
//
// Determines which pixels need to TURN OFF (return to white, 00) and
// drives them in the output, clearing them from the screen buffer.
//
// A pixel needs turning off when the screen buffer shows a non-white value
// but the new desired framebuffer value differs from what's on screen.
// Since the EPD cannot jump directly between grey shades, any pixel that
// needs to change shade must pass through white first — ink_off clears it,
// and a subsequent ink_on call will set it to the new shade.
//
// For each pixel where screen_buffer != packed_fastbuffer (they differ),
// the current screen_buffer value is written to the output (to drive the
// pixel back toward white), and that pixel is cleared in the screen buffer.
//
// Also operates on interleaved rows (same interlacing scheme as ink_on,
// but called with the opposite interlace_period so together they cover
// all rows).
//
// Parameters:
//   a2 - packed_fastbuffer (new desired pixel values, used to detect changes)
//   a3 - packed_screenbuffer (current physical display state, read+write)
//   a4 - Output pointer (pixels to drive in this pass)
//   a5 - Width in bytes (packed bytes per row = display_width / 4)
//   a6 - Height in rows
//   a7 - Interlace period: 0 = start on even rows, non-zero = start on odd
//
// Registers used:
//   a10 - Scratch
//   a11 - Byte counter within a row
//   q0  - Mask: identifies pixels that differ between framebuffer and screen
//   q1  - Output data / temporary
//   q2  - Data from packed_fastbuffer (desired new pixels)
//   q3  - Data from packed_screenbuffer (current screen state)
//   q5  - Mask 0xAA = 10101010
//   q6  - Mask 0x55 = 01010101
//   q7  - Zero (initialised, not used directly)
// =============================================================================
    .global epd_painter_ink_off
    .type   epd_painter_ink_off, @function
    .text
    .align 4

epd_painter_ink_off:
    entry           a1, 32

    // SAR = 1: 1-bit shift within each 32-bit lane
    movi            a10, 1
    wsr.sar         a10

    // Build q6 = 0x55, q5 = 0xAA
    movi            a10, 0b01010101
    s32i            a10, a1, 0
    ee.vldbc.8      q6, a1
    ee.vsl.32       q5, q6

    ee.zero.q       q7
    mov             a11, a5

    // If interlace_period is non-zero, skip the first row (start on odd rows)
    beqz            a7, .ink_off_loop
    add             a2, a2, a5
    add             a3, a3, a5
    add             a4, a4, a5

.ink_off_loop:

    ee.vld.128.ip   q2, a2, 16         // q2 = desired new pixels
    ee.vld.128.ip   q3, a3, 0          // q3 = current screen state

    // Detect changed pixels: XOR gives 1 in any bit position that differs.
    // A pixel needs to be cleared if the framebuffer differs from the screen —
    // whether it changed shade or changed to white, it must go through white.
    ee.xorq         q0, q2, q3         // q0 = bits that differ between desired and screen

    // Spread the difference across both bits of each 2bpp pixel slot,
    // so that if EITHER bit changed, BOTH bits of the slot are flagged.
    // This ensures we operate on whole pixels, not individual bits.

    // Spread low bit → high
    ee.vsl.32       q1, q0
    ee.andq         q1, q1, q5         // keep high-bit positions only
    ee.orq          q0, q0, q1         // q0: high bit set for any changed pixel

    // Spread high bit → low
    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q6         // keep low-bit positions only
    ee.orq          q0, q0, q1         // q0: both bits set for any changed pixel

    // Write the screen buffer value to output, but only for changed pixels.
    // This drives the pixel back toward white using whatever value it currently has.
    ee.andq         q1, q3, q0
    ee.vst.128.ip   q1, a4, 16

    // Clear the changed pixels from the screen buffer (mark them as white/00)
    // so ink_on can set them to the new value on the next frame.
    ee.notq         q0, q0             // invert mask: 1 = pixels to KEEP in screen buffer
    ee.andq         q3, q3, q0         // clear changed pixels from screen buffer
    ee.vst.128.ip   q3, a3, 16         // write updated screen buffer

    // Process remaining bytes in this row
    addi            a11, a11, -16
    bnez            a11, .ink_off_loop

    // Skip the next (inactive) row — interlacing
    mov             a11, a5
    add             a2, a2, a5
    add             a3, a3, a5
    add             a4, a4, a5

    addi            a6, a6, -2
    bnez            a6, .ink_off_loop

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