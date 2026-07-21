/* Build configuration for the vendored umac + Musashi sources.
 *
 * Upstream sets these with -D compiler flags from its Makefile; the
 * Arduino builder has no per-sketch flags, so machw.h includes this
 * header instead.
 *
 * UMAC_MEMSIZE is the emulated Mac's RAM in KB.  4096 makes a 4MB
 * Mac Plus-class machine (System 6/7 capable).  128 gives a faithful
 * Mac 128K (only System <= 3.2).
 *
 * DISP_WIDTH/HEIGHT set the Mac's own screen resolution — umac patches
 * the ROM so Mac OS genuinely runs at this size.  Width must be a
 * multiple of 32.  The sketch scales by MAC_SCALE (its own setting):
 * the default 480x270 is drawn 2x, filling the panel with readable,
 * chunky pixels.  960x540 at scale 1 fills the panel 1:1 but is tiny;
 * 512x342 at scale 1 is the authentic Mac, centred with a bezel.
 */
#ifndef UMAC_CFG_H
#define UMAC_CFG_H

#ifndef UMAC_MEMSIZE
#define UMAC_MEMSIZE    4096
#endif

#ifndef DISP_WIDTH
#define DISP_WIDTH      480
#endif

#ifndef DISP_HEIGHT
#define DISP_HEIGHT     270
#endif

#endif
