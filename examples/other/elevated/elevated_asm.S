#ifdef __ASSEMBLER__
// =============================================================================
//  elevated_asm.S  —  ESP32-S3 PIE vector routines for Elevated renderer
//
//  Uses the ESP32-S3 "EE" PIE 128-bit SIMD extension, the same instruction
//  set used in EPD_Painter.S for the e-paper pixel packing.
// =============================================================================

    .text
    .align 4

// =============================================================================
// elevated_upscale_row
//
// Expand one source row by 4×: each byte is replicated four times in the
// output.  Used to scale a 240-pixel render row to a 960-pixel display row.
//
//   src[0]  → dst[0..3]
//   src[1]  → dst[4..7]
//   ...
//   src[15] → dst[60..63]
//
// The algorithm uses two rounds of ee.vzip.8 to achieve 4× replication:
//   Round 1:  [A B C … P]  →  q0=[A A B B … H H]  q1=[I I J J … P P]
//   Round 2:  vzip each with its own copy → four identical adjacent bytes
//
// Parameters:
//   a2 - Source pointer       (src_w bytes)
//   a3 - Destination pointer  (src_w * 4 bytes)
//   a4 - src_w  (must be a multiple of 16)
//
// Registers:
//   q0..q3 - Working registers  (q0/q1 after first zip, q0-q3 after second)
//   q7     - Zero register      (ee.zero.q, used for Q-register copies)
// =============================================================================
    .global elevated_upscale_row
    .type   elevated_upscale_row, @function

elevated_upscale_row:
    entry   a1, 32

    // q7 = 0 — used as a neutral source for Q-register copies via ee.orq
    ee.zero.q   q7

.upscale_loop:
    // ---- Load 16 source pixels ----
    ee.vld.128.ip   q0, a2, 16
    // q0 = [A B C D E F G H | I J K L M N O P]

    // ---- Round 1: interleave q0 with a copy of itself ----
    // ee.vzip.8 qa, qb interleaves the two 16-byte registers byte-by-byte:
    //   new_qa[i*2]   = old_qa[i]
    //   new_qa[i*2+1] = old_qb[i]   for i in 0..7
    //   new_qb holds the upper 8 interleaved pairs
    ee.orq          q1, q0, q7          // q1 = q0  (copy via OR with zero)
    ee.vzip.8       q0, q1
    // q0 = [A A B B C C D D | E E F F G G H H]
    // q1 = [I I J J K K L L | M M N N O O P P]

    // ---- Round 2: interleave each register with a copy of itself ----
    ee.orq          q2, q0, q7          // q2 = q0
    ee.orq          q3, q1, q7          // q3 = q1

    ee.vzip.8       q0, q2
    // q0 = [A A A A B B B B | C C C C D D D D]
    // q2 = [E E E E F F F F | G G G G H H H H]

    ee.vzip.8       q1, q3
    // q1 = [I I I I J J J J | K K K K L L L L]
    // q3 = [M M M M N N N N | O O O O P P P P]

    // ---- Store 64 output bytes in pixel order ----
    ee.vst.128.ip   q0, a3, 16          // pixels  0-3  (×4 each)
    ee.vst.128.ip   q2, a3, 16          // pixels  4-7
    ee.vst.128.ip   q1, a3, 16          // pixels  8-11
    ee.vst.128.ip   q3, a3, 16          // pixels 12-15

    addi    a4, a4, -16
    bnez    a4, .upscale_loop

    retw.n

#endif
