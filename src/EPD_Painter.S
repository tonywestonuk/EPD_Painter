#ifdef __ASSEMBLER__
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
// epd_painter_convert_packed_fb_to_ink  [OPTIMISED]
//
// Converts packed 2bpp framebuffer into "ink" drive format for one
// waveform pass. Only processes 64-pixel chunks whose corresponding
// bit is set in the chunk_flags parameter.
//
// Parameters:
//   a2 - Input pointer  (packed 2bpp framebuffer row)
//   a3 - Output pointer (ink drive data)
//   a4 - Length in bytes (packed row size, multiple of 16)
//   a5 - Pointer to 3-byte waveform entry for this pass
//   a6 - chunk_flags (1 bit per 64-pixel chunk, MSB = first chunk;
//         bit=1 means process this chunk, bit=0 means skip)
//
// Register allocation:
//   a6  - chunk_flags shift register (consumed MSB-first via slli)
//   a7  - SAR constant (1) — written once, never changed
//   a9  - chunk flag bit extracted each iteration
//   a10 - scratch
//   q0  - loaded input pixels / scratch during mask derivation / mask_10 spread
//   q1  - scratch (mask_11 single-bit, then spread; reused for spread temps)
//   q2  - hi sub-mask / mask_01
//   q3  - lo sub-mask (borrowed) / output accumulator
//   q4  - waveform for pixel value 11 (broadcast, set once)
//   q5  - waveform for pixel value 10 (broadcast, set once)
//   q6  - waveform for pixel value 01 (broadcast, set once)
//   q7  - constant 0x55 mask (broadcast, set once, NEVER modified in loop)
//
// Optimisations over original:
//
//   1. PARALLEL MASK DERIVATION
//      The original derived pixel-class masks serially using XOR to
//      subtract already-handled pixels, creating a dependency chain:
//        mask_11 → xor → mask_10 → (implied) → mask_01
//      The new approach splits q0 into independent hi/lo sub-masks up
//      front and derives all three class masks with no inter-dependencies:
//        lo  = q0 & 0x55
//        hi  = (q0 >> 1) & 0x55
//        m11 = lo & hi          (independent)
//        m10 = hi & ~lo         (independent)
//        m01 = lo & ~hi         (independent)
//      The LX7 PIE unit issues up to 2 vector ops/cycle; with no hazards
//      between these three derivations the pipeline can overlap them.
//
//   2. ELIMINATED q7 TOGGLE
//      The original temporarily flipped q7 between 0x55 and 0xAA using
//      two ee.notq instructions to isolate the high bit of each pixel
//      pair. This created a false RAW dependency on q7. The new code
//      uses ee.notq on a scratch register (q0) instead, leaving q7
//      permanently 0x55 with no hazard.
//
//   3. ELIMINATED ee.zero.q
//      The original zeroed q3 before accumulating. The first waveform
//      term now initialises q3 directly (ee.andq q3, q4, q3) so the
//      zero instruction is gone.
//
//   4. REGISTER BORROWING FOR lo
//      q3 (the output register) holds the lo sub-mask temporarily while
//      all three class masks are derived, then is overwritten with the
//      first waveform output. This keeps all mask derivations live at
//      the same time within the 8-register file without spilling.
//
// Cycle estimate (active chunk, 2-wide issue, no cache misses):
//   Original: ~15+ cycles (serial XOR chain limits ILP)
//   Optimised: ~10 cycles  (parallel mask derivation, no false deps)
// =============================================================================
    .text
    .align 4
    .global epd_painter_convert_packed_fb_to_ink
    .type   epd_painter_convert_packed_fb_to_ink, @function

epd_painter_convert_packed_fb_to_ink:
    entry       a1, 32

    // SAR = 1 for ee.vsr.32 / ee.vsl.32 (shift by 1 bit within 32-bit lanes)
    movi        a7, 1
    wsr.sar     a7

    // Build q7 = [0x55 0x55 ... 0x55] (16 bytes)
    // 0x55 = 0b01010101 — isolates the low bit of each 2bpp pixel pair
    // q7 is NEVER modified inside the loop.
    movi        a10, 0b01010101
    s32i        a10, a1, 0
    ee.vldbc.8  q7, a1

    // Broadcast the 3 waveform bytes from the waveform entry pointer (a5).
    // q4 = drive pattern for pixel value 11 (both bits set   → darkest)
    // q5 = drive pattern for pixel value 10 (high bit only   → dark grey)
    // q6 = drive pattern for pixel value 01 (low  bit only   → light grey)
    ee.vldbc.8.ip   q4, a5, 1
    ee.vldbc.8.ip   q5, a5, 1
    ee.vldbc.8.ip   q6, a5, 1

    // -------------------------------------------------------------------------
    // Main loop — one iteration per 16-byte chunk (64 pixels)
    // -------------------------------------------------------------------------

.convert_loop:

    // Extract the MSB of chunk_flags into a9 (1 = process, 0 = skip),
    // then shift flags left so the next bit is ready for the next iteration.
    extui       a9, a6, 31, 1
    slli        a6, a6, 1

    beqz        a9, .convert_skip

    // =========================================================================
    // ACTIVE CHUNK: derive pixel-class masks and apply waveforms
    // =========================================================================

    ee.vld.128.ip   q0, a2, 16      // load 16 packed bytes (64 pixels)

    // ------------------------------------------------------------------
    // Step 1: split into independent lo and hi sub-masks.
    //
    //   q3 = lo = q0 & 0x55   — bit 0 of each pixel pair, in-place
    //   q2 = hi = (q0>>1)&0x55 — bit 1 of each pixel pair, shifted to bit 0
    //
    // q3 is borrowed here as a temporary to hold lo. It will be
    // overwritten with the waveform output before the end of the block.
    // ------------------------------------------------------------------
    ee.andq         q3, q0, q7          // q3 = lo  (bit 0 of each pixel pair)
    ee.vsr.32       q2, q0              // q2 = q0 >> 1
    ee.andq         q2, q2, q7          // q2 = hi  (bit 1 shifted into bit 0)

    // ------------------------------------------------------------------
    // Step 2: derive all three single-bit class masks.
    //
    // Each mask has a 1 in the lo-bit position for matching pixels,
    // and 0 everywhere else. No dependency between q1, q0, q2 derivations.
    //
    //   q1 = mask_11 = lo & hi          pixels where both bits were 1
    //   q0 = mask_10 = hi & ~lo         pixels where only the high bit was 1
    //   q2 = mask_01 = lo & ~hi         pixels where only the low  bit was 1
    // ------------------------------------------------------------------
    ee.andq         q1, q3, q2          // q1 = mask_11 = lo & hi

    ee.notq         q0, q3              // q0 = ~lo
    ee.andq         q0, q2, q0          // q0 = mask_10 = hi & ~lo

    ee.notq         q2, q2              // q2 = ~hi  (hi no longer needed)
    ee.andq         q2, q3, q2          // q2 = mask_01 = lo & ~hi
                                        // (q3=lo is still valid here)

    // ------------------------------------------------------------------
    // Step 3: spread each single-bit mask across both bits of its pixel
    // pair, so the mask covers the full 2-bit field for each pixel.
    //
    //   spread(m) = m | (m << 1)   within each 32-bit lane
    //
    // After spreading:
    //   q1 = mask_11 covering both bit positions of each 11-pixel
    //   q0 = mask_10 covering both bit positions of each 10-pixel
    //   q2 = mask_01 covering both bit positions of each 01-pixel
    // ------------------------------------------------------------------

    // spread mask_11 (into q3 directly — this overwrites the borrowed lo)
    ee.vsl.32       q3, q1              // q3 = mask_11 << 1
    ee.orq          q3, q3, q1          // q3 = mask_11 spread (both bits)

    // spread mask_10 in place
    ee.vsl.32       q1, q0              // q1 = mask_10 << 1  (q1 now free)
    ee.orq          q0, q0, q1          // q0 = mask_10 spread

    // spread mask_01 in place
    ee.vsl.32       q1, q2              // q1 = mask_01 << 1
    ee.orq          q2, q2, q1          // q2 = mask_01 spread

    // ------------------------------------------------------------------
    // Step 4: AND each spread mask with its waveform byte, then OR all
    // three terms into the output register q3.
    //
    // The three ee.andq ops have no dependencies on each other; the
    // pipeline can overlap them.
    // ------------------------------------------------------------------
    ee.andq         q3, q4, q3          // q3  = waveform_11 where pixel==11
                                        //       (initialises output — no zero needed)
    ee.andq         q0, q5, q0          // q0  = waveform_10 where pixel==10
    ee.orq          q3, q3, q0          // accumulate

    ee.andq         q2, q6, q2          // q2  = waveform_01 where pixel==01
    ee.orq          q3, q3, q2          // accumulate → q3 = final ink output

    ee.vst.128.ip   q3, a3, 16          // write 16 bytes of ink data, advance

    j               .convert_next

    // =========================================================================
    // SKIPPED CHUNK: advance both pointers without processing
    // =========================================================================
.convert_skip:
    addi        a2, a2, 16
    addi        a3, a3, 16

.convert_next:
    addi        a4, a4, -16
    bnez        a4, .convert_loop

    retw.n


// =============================================================================
// epd_painter_ink_on
//
// Determines which pixels need to TURN ON (become grey/black).
//
// For each 2-bit pixel: if the screen buffer is zero (white/available),
// the fastbuffer pixel value is kept. If the screen buffer is non-zero
// (already occupied), the fastbuffer pixel is cleared to zero.
//
// The fastbuffer is modified in place. The screenbuffer is updated to
// reflect newly written pixels (OR'd with the output so future calls
// know those positions are now occupied).
//
// Writes are skipped for 16-byte chunks where the output is all zero.
//
// Returns a 32-bit bitmask (MSB-first): bit 31 = first chunk, bit 30 =
// second chunk, etc. A bit is set if that chunk had any non-zero output.
// This format matches epd_painter_convert_packed_fb_to_ink which reads
// flags from the MSB downward.
//
// Parameters:
//   a2 - packed_fastbuffer (desired pixels, read+write, modified in place)
//   a3 - packed_screenbuffer (current display state, read+write)
//   a4 - Length in bytes (must be multiple of 16, max 512)
//
// Returns:
//   a2 - 32-bit bitmask of active chunks (MSB-first)
//
// Registers used:
//   a10 - Scratch
//   a11 - Return bitmask accumulator
//   a12 - Current bit position (starts 0x80000000, shifts right)
//   a13 - Saved fastbuffer pointer for conditional write
//   a14 - Scratch
//   q0  - Mask: 11 for white pixels, 00 for occupied
//   q1  - Temporary / shifted copy
//   q2  - Data loaded from packed_fastbuffer
//   q3  - Data loaded from packed_screenbuffer
//   q5  - Mask 0xAA (high bit of each 2bpp pixel)
//   q6  - Mask 0x55 (low bit of each 2bpp pixel)
//   q7  - Zero register
// =============================================================================
    .global epd_painter_ink_on
    .type   epd_painter_ink_on, @function
    .text
    .align 4

epd_painter_ink_on:
    entry           a1, 32

    movi            a10, 1
    wsr.sar         a10

    movi            a10, 0b01010101
    s32i            a10, a1, 0
    ee.vldbc.8      q6, a1
    ee.vsl.32       q5, q6

    ee.zero.q       q7

    movi            a11, 0             // bitmask accumulator
    movi            a12, 31
    ssl             a12                // SAR = 31 for shift
    movi            a12, 1
    sll             a12, a12           // a12 = 1 << 31 = 0x80000000

    // Restore SAR = 1 for vector bit shifts
    movi            a10, 1
    wsr.sar         a10

.ink_on_loop:

    mov             a13, a2            // save fastbuffer pointer

    ee.vld.128.ip   q2, a2, 16        // q2 = fastbuffer, advance
    ee.vld.128.ip   q3, a3, 16        // q3 = screenbuffer, advance

    // Build mask: 11 where screen==00, 00 where screen!=00
    ee.vsl.32       q1, q3
    ee.andq         q1, q1, q5
    ee.orq          q0, q3, q1

    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q6
    ee.orq          q0, q0, q1

    ee.notq         q0, q0

    // Keep fastbuffer pixels only where screen is white
    ee.andq         q2, q2, q0

    // Check if any bytes non-zero: extract all 4 words, OR together
    ee.movi.32.a    q2, a10, 0
    ee.movi.32.a    q2, a14, 1
    or              a10, a10, a14
    ee.movi.32.a    q2, a14, 2
    or              a10, a10, a14
    ee.movi.32.a    q2, a14, 3
    or              a10, a10, a14

    beqz            a10, .ink_on_skip_write

    // Set bitmask flag (MSB-first)
    or              a11, a11, a12

    // Write masked fastbuffer back
    ee.vst.128.ip   q2, a13, 0

    // Update screenbuffer: OR in the newly active pixels
    addi            a10, a3, -16       // screenbuffer was advanced, step back
    ee.vld.128.ip   q3, a10, 0        // reload screenbuffer chunk
    ee.orq          q3, q3, q2         // mark new pixels as occupied
    ee.vst.128.ip   q3, a10, 0        // write back

.ink_on_skip_write:

    // Shift bit position right for next chunk
    srli            a12, a12, 1

    addi            a4, a4, -16
    bnez            a4, .ink_on_loop

    mov             a2, a11
    retw.n

// =============================================================================
// epd_painter_ink_off
//
// Processes chunks that were NOT handled by ink_on (flag==0 in bitmask).
// For those chunks:
//   - Where screen and fastbuffer pixels differ: copy screen value to
//     fastbuffer (to drive pixel back toward white), clear from screenbuffer.
//   - Where they match: set fastbuffer to zero (no drive needed).
//
// Chunks where the bitmask bit is 1 (ink_on already processed) are skipped.
// Bitmask is MSB-first: bit 31 = first chunk, bit 30 = second, etc.
//
// Parameters:
//   a2 - packed_fastbuffer (read+write, modified in place)
//   a3 - packed_screenbuffer (current display state, read+write)
//   a4 - Length in bytes (must be multiple of 16, max 512)
//   a5 - Bitmask from ink_on (bit=1 means skip, MSB-first)
//
// Registers used:
//   a10 - Scratch
//   a11 - Copy of bitmask
//   a12 - Current bit position (starts 0x80000000, shifts right)
//   a13 - Saved fastbuffer pointer
//   a14 - Saved screenbuffer pointer
//   q0  - XOR / spread mask (changed pixel mask)
//   q1  - Temporary / shifted copy
//   q2  - Data loaded from packed_fastbuffer
//   q3  - Data loaded from packed_screenbuffer
//   q4  - Output: screen values masked to changed pixels only
//   q5  - Mask 0xAA (high bit of each 2bpp pixel)
//   q6  - Mask 0x55 (low bit of each 2bpp pixel)
// =============================================================================
    .global epd_painter_ink_off
    .type   epd_painter_ink_off, @function
    .text
    .align 4

epd_painter_ink_off:
    entry           a1, 32

    movi            a10, 1
    wsr.sar         a10

    movi            a10, 0b01010101
    s32i            a10, a1, 0
    ee.vldbc.8      q6, a1
    ee.vsl.32       q5, q6

    mov             a11, a5            // a11 = bitmask

    // Build a12 = 0x80000000 (MSB-first)
    movi            a12, 31
    ssl             a12
    movi            a12, 1
    sll             a12, a12

    // Restore SAR = 1 for vector bit shifts
    movi            a10, 1
    wsr.sar         a10

.ink_off_loop:

    // Test if this chunk was already processed by ink_on — skip if so
    and             a10, a11, a12
    bnez            a10, .ink_off_skip

    // This chunk was NOT processed by ink_on
    mov             a13, a2            // save fastbuffer pointer
    mov             a14, a3            // save screenbuffer pointer

    ee.vld.128.ip   q2, a2, 0         // q2 = fastbuffer (no advance)
    ee.vld.128.ip   q3, a3, 0         // q3 = screenbuffer (no advance)

    // 1. XOR: find pixels that differ
    ee.xorq         q0, q2, q3

    // 2. Spread across both bits of each 2bpp pixel
    ee.vsl.32       q1, q0
    ee.andq         q1, q1, q5
    ee.orq          q0, q0, q1

    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q6
    ee.orq          q0, q0, q1

    // 3. AND screenbuffer with mask → screen value for changed, zero for same
    ee.andq         q4, q3, q0

    // Write result to fastbuffer
    ee.vst.128.ip   q4, a13, 0

    // Clear changed pixels from screenbuffer
    ee.notq         q0, q0
    ee.andq         q3, q3, q0
    ee.vst.128.ip   q3, a14, 0

.ink_off_skip:

    // Advance pointers past this chunk
    addi            a2, a2, 16
    addi            a3, a3, 16

    // Shift bit position right for next chunk (MSB-first)
    srli            a12, a12, 1

    addi            a4, a4, -16
    bnez            a4, .ink_off_loop

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

// =============================================================================
// epd_painter_ink
//
// Combined ink_on + ink_off in a single pass.
//
// For each 16-byte chunk:
//   1. ink_on logic: keep fastbuffer pixels only where screen is white (00).
//      If any non-zero result → write to fastbuffer, OR into screenbuffer,
//      set bitmask bit.
//   2. If ink_on produced all zeros → ink_off logic: XOR to find differing
//      pixels, write screen values for changed pixels to fastbuffer, clear
//      those pixels from screenbuffer.
//
// Parameters:
//   a2 - packed_fastbuffer (read+write)
//   a3 - packed_screenbuffer (read+write)
//   a4 - Length in bytes (multiple of 16, max 512)
//
// Returns:
//   a2 - 32-bit bitmask of ink_on chunks (MSB-first)
//
// Registers:
//   a10 - Scratch
//   a11 - Bitmask accumulator
//   a12 - Current bit (0x80000000 >> n)
//   a13 - Saved fastbuffer pointer
//   a14 - Saved screenbuffer pointer
//   q0  - Spread mask
//   q1  - Temp / shifted
//   q2  - Fastbuffer data
//   q3  - Screenbuffer data
//   q4  - ink_off output
//   q5  - 0xAA mask
//   q6  - 0x55 mask
//   q7  - Zero
// =============================================================================
.global epd_painter_ink
    .type   epd_painter_ink, @function
    .text
    .align 4

epd_painter_ink:
    entry           a1, 32

    movi            a10, 1
    wsr.sar         a10

    movi            a10, 0b01010101
    s32i            a10, a1, 0
    ee.vldbc.8      q6, a1
    ee.vsl.32       q5, q6

    ee.zero.q       q7

    mov             a11, a5

    movi            a12, 31
    ssl             a12
    movi            a12, 1
    sll             a12, a12

    movi            a10, 1
    wsr.sar         a10

.ink_loop:
    mov             a13, a2

    and             a10, a11, a12
    xor             a11, a11, a12      // flip bit

    bnez            a10, .ink_off_first

    // ============== BIT WAS 0 → now 1 → try ink_on first ==============

    ee.vld.128.ip   q2, a2, 16
    ee.vld.128.ip   q3, a3, 16

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

    beqz            a10, .bit0_on_empty

    // ink_on produced output: keep bit=1, write, skip ink_off
    ee.vst.128.ip   q2, a13, 0

    addi            a10, a3, -16
    ee.vld.128.ip   q3, a10, 0
    ee.orq          q3, q3, q2
    ee.vst.128.ip   q3, a10, 0

    j               .ink_next

.bit0_on_empty:
    // ink_on produced nothing: revert bit to 0, run ink_off instead
    xor             a11, a11, a12

    // Reload original data
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

    // ============== BIT WAS 1 → now 0 → try ink_off first ==============

.ink_off_first:
    ee.vld.128.ip   q2, a2, 16
    ee.vld.128.ip   q3, a3, 16

    ee.xorq         q0, q2, q3

    ee.vsl.32       q1, q0
    ee.andq         q1, q1, q5
    ee.orq          q0, q0, q1

    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q6
    ee.orq          q0, q0, q1

    ee.andq         q4, q3, q0

    ee.movi.32.a    q4, a10, 0
    ee.movi.32.a    q4, a14, 1
    or              a10, a10, a14
    ee.movi.32.a    q4, a14, 2
    or              a10, a10, a14
    ee.movi.32.a    q4, a14, 3
    or              a10, a10, a14

    beqz            a10, .bit1_off_empty

    // ink_off produced output: keep bit=0, write, skip ink_on
    ee.vst.128.ip   q4, a13, 0

    ee.notq         q0, q0
    ee.andq         q3, q3, q0
    addi            a10, a3, -16
    ee.vst.128.ip   q3, a10, 0

    j               .ink_next

.bit1_off_empty:
    // ink_off produced nothing: revert bit to 1, run ink_on instead
    xor             a11, a11, a12

    // Reload original data
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

    // Check if ink_on produced anything
    ee.movi.32.a    q2, a10, 0
    ee.movi.32.a    q2, a14, 1
    or              a10, a10, a14
    ee.movi.32.a    q2, a14, 2
    or              a10, a10, a14
    ee.movi.32.a    q2, a14, 3
    or              a10, a10, a14

    beqz            a10, .bit1_on_empty

    ee.vst.128.ip   q2, a13, 0

    addi            a10, a3, -16
    ee.vld.128.ip   q3, a10, 0
    ee.orq          q3, q3, q2
    ee.vst.128.ip   q3, a10, 0

    j               .ink_next

.bit1_on_empty:
    ee.vst.128.ip   q7, a13, 0

.ink_next:
    srli            a12, a12, 1
    addi            a4, a4, -16
    bnez            a4, .ink_loop

    mov             a2, a11
    retw.n
#endif