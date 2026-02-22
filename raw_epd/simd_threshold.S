// =============================================================================
// EPD Painter Assembly Functions
// Optimized routines for e-paper display pixel manipulation
// =============================================================================

    .text
    .align 4

// =============================================================================
// epd_painter_compact_pixels
//
// Compacts 4 pixels (4 × 8-bit = 32 bits) into 1 byte (4 × 2-bit pixels).
// The two least-significant bits of each input byte are packed together.
//
// Parameters:
//   a2 - Input pointer (1 byte per pixel)
//   a3 - Output pointer (2 bits per pixel)
//   a4 - Size (width*height)
//
// Registers:
//   a10 - Temporary/scratch
//   a11 - Pixel counter
//   q0  - Input pixel data
//   q1  - Shifted data for packing
//   q5  - Mask (0x03 broadcast to all bytes)
//   q7  - Zero register
// =============================================================================
    .global epd_painter_compact_pixels
    .type   epd_painter_compact_pixels, @function

epd_painter_compact_pixels:
    entry   a1, 32

    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------
    ee.zero.q       q7

    // Build 0x03 mask and broadcast to all bytes in q5
    movi            a10, 0b00000011
    s32i            a10, a1, 0
    ee.vldbc.8      q5, a1

    // Configure shift registers:
    //   SAR_BYTE = 1 byte
    //   SAR      = 2 bits
    movi            a10, 1
    wur.sar_byte    a10
    movi            a10, 2
    wsr.sar         a10

    // -------------------------------------------------------------------------
    // Main loop: Process 16 input bytes → 4 output bytes
    // -------------------------------------------------------------------------
    mov             a11, a4

.compact_process_pixels:
    // Load 16 pixels (bytes) into q0
    ee.vld.128.ip   q0, a2, 16

    // Mask off all but the 2 LSBs of each pixel
    ee.andq         q0, q0, q5

    // Pack bits: shift right by 1 byte and left by 2 bits, OR back into q0
    // Repeat 3 times to collect 4 pixels per output byte
    ee.src.q        q1, q0, q7
    ee.vsl.32       q0, q0
    ee.orq          q0, q0, q1

    ee.vsl.32       q0, q0
    ee.src.q        q1, q1, q7
    ee.orq          q0, q0, q1

    ee.vsl.32       q0, q0
    ee.src.q        q1, q1, q7
    ee.orq          q0, q0, q1

    // Unzip twice to group packed bytes together
    ee.vunzip.8     q0, q1
    ee.vunzip.8     q0, q1

    // Store 4 packed output bytes
    ee.stxq.32      q0, q7, a3, 0, 0

    // Advance output pointer and pixel counter
    addi            a3, a3, 4
    addi            a11, a11, -16
    bnez            a11, .compact_process_pixels

    retw.n



// =============================================================================
// epd_painter_convert_packed_fb_to_ink
//
// Converts a packed 2bpp framebuffer chunk into ink format.
// Operates on pre-packed data to avoid PSRAM reloads.
//
// Parameters:
//   a2 - Input pointer (packed framebuffer, 2 bpp)
//   a3 - Output pointer (ink output)
//   a4 - Length (bytes)
//   a5 - waveform
//
// Registers:
//   a7  - SAR constant (1)
//   a10 - Temporary
//   q0  - Input/working data
//   q1  - Shifted data
//   q6  - Mask 0x55 (01010101)
//   q7  - Mask 0xAA (10101010)
// =============================================================================
    .global epd_painter_convert_packed_fb_to_ink
    .type   epd_painter_convert_packed_fb_to_ink, @function
    .align 4
epd_painter_convert_packed_fb_to_ink:
    entry           a1, 32

    // Initialize
    movi            a7, 1
    wsr.sar         a7

    // Set q7 = 010101010
    movi            a10, 0b01010101
    s32i            a10, a1, 0
    ee.vldbc.8      q7, a1
    
    ee.vldbc.8.ip  q4, a5, 1
    ee.vldbc.8.ip  q5, a5, 1
    ee.vldbc.8.ip  q6, a5, 1

    // Main loop: process 16 bytes = 64 pixels (2 bpp)
    srli            a4, a4, 4           // iterations = length / 16
    loopnez         a4, .convert_end

    ee.vld.128.ip   q0, a2, 16
// clear output q reg...q3.
    ee.zero.q      q3

//  Calculate for 11 bits:
    ee.vsr.32      q1, q0
    ee.andq        q1, q1, q7
    ee.andq        q1, q0, q1
    ee.vsl.32      q2, q1
    ee.orq         q1, q1, q2 

// load waveform for mask11 into  q5
    ee.andq        q2, q4, q1    // <- blanks out the waveform that are not needed.
    ee.orq         q3, q3, q2    // <- stamps waveform onto the output

//  Calculate for 10 bits:
    ee.xorq        q0, q0, q1      // remove 11's from input
    ee.notq        q7, q7
    ee.andq        q1, q0, q7      // isolate high bits (q7 = 0xAA...)
    ee.notq        q7, q7
    ee.vsr.32      q2, q1          // shift high bit down to low position
    ee.orq         q1, q1, q2      // combine → 10 becomes 11, 01 becomes 00

// load waveform for mask10 into q5
    ee.andq        q2, q5, q1      // blank out waveform where not needed
    ee.orq         q3, q3, q2      // stamp waveform onto output

//  Calculate for 01 bits:
    ee.andq        q1, q0, q7      // isolate low bits 
    ee.vsl.32      q2, q1          // shift low bit up to high position
    ee.orq         q1, q1, q2      // combine → 01 becomes 11, 10 becomes 00

// load waveform for mask01 into q5
    ee.andq        q2, q6, q1      // blank out waveform where not needed
    ee.orq         q3, q3, q2      // stamp waveform onto output

// Store the output
    ee.vst.128.ip   q3, a3, 16

.convert_end:
    retw.n

// =============================================================================
// epd_painter_ink_on
//
// Copies pixels from THIS_TIME to OUTPUT if SCREEN_BUFFER is 00.
// Updates SCREEN_BUFFER with copied pixels.
//
// Parameters:
//   a2 - This time pointer (source)
//   a3 - Screen buffer (mask/accumulator)
//   a4 - Output pointer (destination)
//   a5 - Width (bytes per row)
//   a6 - Height (number of rows)
//   a7 - Interlace period (0 = even rows, non-zero = odd rows)
//
// Registers:
//   a10 - Temporary
//   a11 - Width counter
//   q0  - Working mask
//   q1  - Temporary
//   q2  - This time data
//   q3  - Screen buffer data
//   q4  - Output data
//   q5  - Mask 0xAA (10101010)
//   q6  - Mask 0x55 (01010101)
//   q7  - Zero register
// =============================================================================
    .global epd_painter_ink_on
    .type   epd_painter_ink_on, @function
    .text
    .align 4
epd_painter_ink_on:
    entry           a1, 32

    // Configure shift register: SAR = 1 bit
    movi            a10, 1
    wsr.sar         a10

    // Set q6 = 01010101, q5 = 10101010
    movi            a10, 0b01010101
    s32i            a10, a1, 0
    ee.vldbc.8      q6, a1
    ee.vsl.32       q5, q6

    ee.zero.q       q7
    mov             a11, a5

    // Start from line 0 or line 1 based on interlace period
    beqz            a7, .ink_on_loop
    add             a2, a2, a5
    add             a3, a3, a5
    add             a4, a4, a5

.ink_on_loop:
    ee.vld.128.ip   q2, a2, 16          // Load this_time
    ee.vld.128.ip   q3, a3, 0           // Load screen_buffer

    // Determine which pixels can move (screen_buffer must be 00)
    // Stamp right bit to left
    ee.vsl.32       q1, q3
    ee.andq         q1, q1, q5
    ee.orq          q0, q3, q1
    
    // Stamp left bit to right
    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q6
    ee.orq          q0, q0, q1
    
    // q0 now contains 00 only where both pixel bits were 0
    ee.notq         q0, q0              // Invert to create mask (11 = can move)
    
    // Copy masked bits from this_time to output
    ee.andq         q4, q2, q0
    ee.vst.128.ip   q4, a4, 16

    // Update screen buffer with newly written values
    ee.orq          q3, q3, q4
    ee.vst.128.ip   q3, a3, 16

    // Continue processing line
    addi            a11, a11, -16
    bnez            a11, .ink_on_loop

    // Skip next line (interlacing)
    mov             a11, a5
    add             a2, a2, a5
    add             a3, a3, a5
    add             a4, a4, a5

    addi            a6, a6, -2          // Subtract 2 lines
    bnez            a6, .ink_on_loop

    retw.n

// =============================================================================
// epd_painter_ink_off
//
// Copies pixels from SCREEN_BUFFER to OUTPUT where THIS_TIME is 00.
// Clears those pixels in SCREEN_BUFFER.
//
// Parameters:
//   a2 - This time pointer (mask source)
//   a3 - Screen buffer (source/destination)
//   a4 - Output pointer (destination)
//   a5 - Width (bytes per row)
//   a6 - Height (number of rows)
//   a7 - Interlace period (0 = even rows, non-zero = odd rows)
//
// Registers:
//   a10 - Temporary
//   a11 - Width counter
//   q0  - Working mask
//   q1  - Output data / temporary
//   q2  - This time data
//   q3  - Screen buffer data
//   q5  - Mask 0xAA (10101010)
//   q6  - Mask 0x55 (01010101)
//   q7  - Zero register
// =============================================================================
    .global epd_painter_ink_off
    .type   epd_painter_ink_off, @function
    .text
    .align 4
epd_painter_ink_off:
    entry           a1, 32

    // Configure shift register: SAR = 1 bit
    movi            a10, 1
    wsr.sar         a10

    // Set q6 = 01010101, q5 = 10101010
    movi            a10, 0b01010101
    s32i            a10, a1, 0
    ee.vldbc.8      q6, a1
    ee.vsl.32       q5, q6

    ee.zero.q       q7
    mov             a11, a5

    // Start from line 0 or line 1 based on interlace period
    beqz            a7, .ink_off_loop
    add             a2, a2, a5
    add             a3, a3, a5
    add             a4, a4, a5

.ink_off_loop:
    ee.vld.128.ip   q2, a2, 16          // Load this_time
    ee.vld.128.ip   q3, a3, 0           // Load screen_buffer

    // Determine which pixels need to be cleared (screen buffer different to this time )
    ee.xorq         q0, q2, q3

    
    // Stamp right bit to left
    ee.vsl.32       q1, q0
    ee.andq         q1, q1, q5
    ee.orq          q0, q0, q1
    
    // Stamp left bit to right
    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q6
    ee.orq          q0, q0, q1

    //ee.notq         q0, q0              // Invert to create mask (11 = pixels to move)

    // Copy masked bits from screen_buffer to output
    ee.andq         q1, q3, q0
    ee.vst.128.ip   q1, a4, 16

    // Clear those bits in screen_buffer
    ee.notq         q0, q0
    ee.andq         q3, q3, q0
    ee.vst.128.ip   q3, a3, 16

    // Continue processing line
    addi            a11, a11, -16
    bnez            a11, .ink_off_loop

    // Skip next line (interlacing)
    mov             a11, a5
    add             a2, a2, a5
    add             a3, a3, a5
    add             a4, a4, a5

    addi            a6, a6, -2          // Subtract 2 lines
    bnez            a6, .ink_off_loop

    retw.n

// =============================================================================
// epd_painter_interleaved_copy
//
// Copies every other line from source to destination (interleaved rows).
//
// Parameters:
//   a2 - Input pointer (source)
//   a3 - Output pointer (destination)
//   a4 - Width (bytes per row)
//   a5 - Height (total number of rows)
//   a6 - Interlace period (0 = even rows, non-zero = odd rows)
//
// Registers:
//   a10 - Temporary
//   a11 - Width counter
//   q0  - Data transfer register
// =============================================================================
    .global epd_painter_interleaved_copy
    .type   epd_painter_interleaved_copy, @function

epd_painter_interleaved_copy:
    entry           a1, 32

    // If starting on odd rows, skip first line
    beqz            a6, .interleave_line_loop
    add             a2, a2, a4
    add             a3, a3, a4

.interleave_line_loop:
    mov             a11, a4

.interleave_copy_16:
    // Copy 16 bytes at a time
    ee.vld.128.ip   q0, a2, 16
    ee.vst.128.ip   q0, a3, 16

    addi            a11, a11, -16
    bnez            a11, .interleave_copy_16

    // Skip next line in both source and destination (interlacing)
    add             a2, a2, a4
    add             a3, a3, a4

    addi            a5, a5, -2          // Subtract 2 lines
    bnez            a5, .interleave_line_loop

    retw.n