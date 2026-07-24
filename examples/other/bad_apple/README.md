# Bad Apple — e-paper video player

Plays video on an EPD_Painter panel at full motion, streamed from SD card.
Named after the [Bad Apple!!](https://en.wikipedia.org/wiki/Bad_Apple!!) shadow-art
video it was built for, but it plays anything: `video2epv.py` converts any file
ffmpeg can read into the driver's native packed format, so the ESP32 does no
pixel processing at all — each frame is one SD read and one `paintPacked()` call.

## Quick start

1. **Convert a video** (needs `python3` with `numpy`, and `ffmpeg` on PATH):

   ```sh
   python3 video2epv.py your_video.mp4 heaven7.epv --fps 20
   ```

2. **Copy** your `.epv` files into a **`videos` folder** in the root of a
   FAT32 SD card. One video plays immediately; several bring up a picker
   on the panel — tap a title to play it (or send its number over serial),
   and tap the screen during playback to get the picker back. A lone
   `badapple.epv` in the card root still works, for cards made before the
   picker existed.

3. **Pick your board** at the top of `bad_apple.ino`:

   ```cpp
   #define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
   // #define EPD_PAINTER_PRESET_M5PAPER_S3
   ```

   The sketch needs the
   [GT911 Lite](https://github.com/tonywestonuk/gt911-arduino) library for
   the picker's touch support (both supported boards have a GT911 touch
   panel). If no touch controller answers at runtime, the picker still
   works over serial.

4. **Flash** the sketch, insert the card, enjoy. The serial monitor (115200)
   prints a stats line every 5 seconds:

   ```
   [bad_apple] 19.9 fps, read 4 ms (1.71 MB/s), paint 41 ms
   ```

   `read` is SD time per frame, `paint` is panel drive time. If the panel
   can't hold the file's frame rate the video just plays slower — every frame
   is shown, and normal tempo resumes when the load lightens.

## Choosing conversion settings

| Option | Effect |
|---|---|
| `--fps N` | Output frame rate (default 15). 20 is a good match for what the panel sustains at NORMAL quality; the converter drops/keeps source frames to hit it. |
| `--fill fit\|crop\|stretch` | `fit` letterboxes on white (default), `crop` fills the panel and trims the overflow, `stretch` ignores aspect ratio. A 16:9 source fills the 960×540 panel exactly with any mode. |
| `--line-double` | Store half-height frames; the player drives each row twice. Halves file size and SD bandwidth — free quality-wise when the source has ≤ 480 lines. |
| `--no-dither` | Plain 4-level quantise instead of ordered dithering. |
| `--invert` | Swap black/white — handy for dark-background sources. |
| `--no-rle` | Raw fixed-size frames instead of RLE. Only useful for debugging. |
| `--preview out.png` | Also writes one frame, decoded back from the packed data, so you can check framing and tone before copying to SD. |

Notes:

- **Grey mapping**: video luma is converted to linear reflectance and quantised
  to the panel's 4 levels with a Bayer 8×8 ordered dither. Ordered dithering is
  temporally stable — static areas produce identical patterns every frame, so
  the driver's delta engine skips them entirely.
- **File size**: frames are PackBits RLE. High-contrast animation compresses
  10–20×; live-action with textured backgrounds more like 5–8×. A 3½-minute
  clip is typically 40–100 MB.
- **Playback quality**: `PLAY_QUALITY` in the sketch selects the waveform set.
  `QUALITY_NORMAL` gives the calibrated greys; `QUALITY_FAST` roughly halves
  paint time if you want higher frame rates.

## SD wiring

Pins and bus mode are picked at runtime from the detected board:

| Board | Mode | CS | CLK | MOSI/CMD | MISO/D0 |
|---|---|---|---|---|---|
| M5PaperS3 | 1-bit SDMMC | 47 | 39 | 38 | 40 |
| LilyGo T5 S3 GPS / Pro | SPI 25 MHz | 12 | 14 | 13 | 21 |

The M5PaperS3 slot is dedicated wiring, so it uses the native SD protocol
(via the S3's GPIO matrix) for ~2× SPI throughput. The LilyGo shares its SPI
bus with the LoRa radio, so per LilyGo's own examples it must run in SPI mode
with every chip-select parked high — the sketch handles that. Other boards:
add an entry to `sdPinsForBoard()`.

## .epv format

16-byte header (little-endian): magic `EPV1`, u16 width, u16 height (frame
rows), u16 fps, u16 flags (bit 0 line-doubled, bit 1 RLE), u32 frame count.
Frames follow: raw files pack 4 pixels/byte (first pixel in the MSBs, 0=white,
3=black); RLE files prefix each frame with a u32 record header (bits 0–30
payload length, bit 31 = stored raw) followed by a PackBits payload. Full
details in the header comments of `video2epv.py` and `bad_apple.ino`.
