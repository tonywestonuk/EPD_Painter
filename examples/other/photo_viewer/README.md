# Photo Viewer

Shows JPEG photographs from an SD card in the panel's 16 native greys,
rendered against the board's own measured grey curve.

Put `.jpg` files in a `/photos` folder in the root of an SD card, flash,
and tap: right half of the screen for the next photo, left half for the
previous. Over serial, `n` / `p` / `space` do the same and `i` reports
what is on screen.

## Why it looks better than a straight conversion

A panel's 16 greys are **not evenly spaced**. Measured on a LilyGo T5 S3:

```
255 253 231 219 205 190 172 135 126 115  61  55  47  35  21   0
```

Levels 0 and 1 differ by 2 units; levels 9 and 10 differ by 54. A dither
that assumes even steps therefore diffuses error into levels that cannot
express it, and smooth gradients band badly.

This viewer quantises against the **measured** curve — for each pixel it
finds the level whose real luminance is closest, and diffuses the true
residual (Floyd-Steinberg). That is what keeps skin tones and skies
smooth.

The curve is produced by the [tuneup](../tuneup) example, which tunes the
board against a flatbed scanner and stores the curve alongside the tuned
trains. This sketch just loads it:

```cpp
EPD_PainterTuned::install(epd.driver(), buf, n);
const uint8_t *curve = EPD_PainterTuned::levelLuminance();
```

On a board that has never been tuned, `levelLuminance()` returns null and
the viewer falls back to assuming even levels. It still works — it will
just band on gradients. Run tuneup once to fix that.

## Requirements

- **Adafruit GFX**, **JPEGDEC**, **gt911-arduino**
- An SD card with a `/photos` folder. Pin maps for the M5PaperS3, LilyGo
  T5 S3 GPS/Pro, H716 and H752 are built in (same map as the `bad_apple`
  example); other boards need adding to `sdPinsForBoard()`.
- PSRAM — the compressed file and the decoded image both live there.

## Notes

- Photographs much larger than the panel are decoded at 1/2, 1/4 or 1/8
  scale first, so a full-size phone photo does not need more PSRAM than
  the board has, then scaled to fit and centred.
- Colour JPEGs are fine; they are converted to greyscale on decode.
- Files that fail to decode are skipped rather than stalling the
  slideshow.
- Rendering is `QUALITY_NORMAL` 16-grey — the tier tuneup calibrates.
