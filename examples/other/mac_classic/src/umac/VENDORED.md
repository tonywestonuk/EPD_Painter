# Vendored sources

This folder is a flattened copy of Matt Evans's **umac** Mac 128K/Plus
emulator and Karl Stenerud's **Musashi** 68000 core, both MIT licensed
(see the header of each file).

- umac: https://github.com/evansm7/umac @ `b62d3c6e725ca2ac2224e710b3cee765dbc35e45`
- Musashi: https://github.com/evansm7/Musashi (umac branch) @ `b3144f12c1296d8fbb600eea13f5db71ffb1b8d8`

`m68kops.c/.h` are the generated opcode files (output of Musashi's
`m68kmake`, checked in so no build step is needed).

Local changes, kept deliberately minimal:

- `umac_cfg.h` added; `machw.h` includes it (replaces upstream's `-D`
  compiler flags, which the Arduino builder cannot pass).
- `m68kfpu.c` renamed to `m68kfpu_inc.h` (it is `#include`d by
  `m68kcpu.c` and must not be compiled standalone); the include in
  `m68kcpu.c` updated to match.  Likewise `softfloat-macros` and
  `softfloat-specialize` gained `.h` extensions (the Arduino builder
  does not stage extension-less files).
- `rom.c`: screen-resolution patching extended past upstream's 64KB
  screen-distance limit (full 32-bit writes for the suba.l immediate
  and the screen-base longwords at 0x8a/0x146, the latter verified
  against the ROM's bytes at runtime).  Enables the 960x540 full-panel
  mode; identical behaviour for upstream-sized screens.
- Upstream's `unix_main.c` (SDL frontend), `keymap_sdl.h` and
  Musashi's own `m68kconf.h` (superseded by umac's) are not copied.
