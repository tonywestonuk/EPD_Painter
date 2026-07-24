# TuneUp — the self-tuning rig

The shipped presets were tuned with a closed optical loop: paint a pattern,
scan the glass, measure, adjust. This example puts that loop **on the board
itself**: lay it face-down on any eSCL/AirScan network scanner and it tunes
its own 16-grey waveform trains, saves them to flash, and uses them from
then on. No PC in the loop.

## The flow

1. **WiFi** — credentials are compiled in. Set `WIFI_SSID` / `WIFI_PASS` at
   the top of the sketch; if they're still blank, the panel shows
   instructions to edit the file and reflash.
2. **Scanner** — discovers eSCL scanners via mDNS (`_uscan._tcp`), draws
   them as buttons; tap yours (or type its number over serial). The choice
   is remembered in NVS, so later boots need no interaction.
3. **DEMO SCAN** — scans the bed and shows the image on the panel in 16
   native greys. Proves the plumbing.
4. **TUNE ME** — instructions + a 30 s countdown to lay the board face-down
   and close the lid, then the self-tune runs (10–15 minutes, progress on
   serial). Or send `T` over serial to start immediately.

## What the self-tune does (see tuner.h)

- **Registration**: paints full black and takes a wide scan to find the
  panel on the bed, then a corner mark to learn the face-down mirroring.
- **Reference**: every measurement is a dither-match card — 16 columns,
  each pairing a Bayer-dithered black/white reference (the optical truth
  for level g/15) with the native waveform grey. Scanner gamma, lamp
  falloff and drift cancel inside each frame; the L0/L15 pairs calibrate
  out the panel's vertical gradient.
- **Probe**: a pure-darken dose ladder measures the panel's response curve
  at the tuning pass period — the initial gain table.
- **Converge**: damped proportional edits over the (darkens, whitens,
  re-darkens) train grammar, one edit per out-of-tolerance level per
  cycle, expected move sizes updated online from what each edit actually
  did, overshoots reverted. Typically converges in 5–15 cycles to within
  ~4 scanner units.
- **Removes**: charge-matched erase trains derived from the tuned applies
  (the DC ledger stays exact), then verified optically — paint the card,
  repaint white, scan the residual; ghosting levels get a wider erase.
- **Save**: tables + tuning period go to LittleFS via
  `src/EPD_Painter_tuned.h`, and are reloaded through the same path every
  boot.

## Using the tuned tables in YOUR sketch

```cpp
#include <LittleFS.h>
#include "EPD_Painter_tuned.h"
...
epd.begin();
LittleFS.begin(true);                        // true = format if unformatted

// Pass the driver: EPD_PainterAdafruit wraps one, so hand over
// epd.driver(). With a raw EPD_Painter, pass the painter itself.
if (EPD_PainterTuned::load(epd.driver()))    // no-op if there is no valid blob
  Serial.println("using flash-tuned trains");

epd.setGreyLevels(16);
```

Order doesn't matter relative to `setGreyLevels(16)` — call it either side
and the train library ends up built from the tuned tables. `load()` also
restores the **pass period** the tuning was done at, which is essential:
trains are only valid at the period they were calibrated against.

The blob is board-keyed (pins + geometry) and CRC-guarded; on any mismatch
`load()` returns false and the preset tables stay in charge, so it is safe
to call unconditionally on any board.

**Scope:** version 1 of the blob holds the **16-grey NORMAL** apply and
remove trains only. Your 4-level, FAST and direct grey-to-grey trains
still come from the board preset — tuneup improves greyscale rendering,
not the speed modes.

## Requirements

- **Adafruit GFX Library**, **gt911-arduino** (touch),
  **JPEGDEC** (image analysis — required for tuning)
- A scanner speaking plain-HTTP eSCL (`GET /eSCL/ScannerCapabilities`).
  Nearly every network scanner/MFP since ~2015 does; older USB scanners
  can be bridged with `sane-airscan`.

## Serial commands (115200)

| Key | Action |
|-----|--------|
| `s` | demo scan |
| `T` | self-tune now (no countdown) |
| `q` | abort a running tune (between cycles) |
| `P <us>` | override the tuning pass period (0 = preset) |
| `L` | tuned-blob status |
| `X` | erase the tuned blob (reboot to fall back to presets) |
| `n` | re-pick the scanner |
| `m` | reprint the menu |
| `r` | reboot |

**Don't commit the sketch with your WiFi password in it.**
