#!/usr/bin/env python3
"""video2epv.py — convert a video into EPD_Painter native packed-frame format.

Output (.epv) is a stream of frames in the driver's 2bpp packed layout —
4 pixels/byte, first pixel in the MSBs, 0=white 1=lt-grey 2=dk-grey 3=black —
exactly what EPD_Painter::paintPacked() accepts, so the player on the ESP32
does no pixel processing at all.

File layout (little-endian):
    offset 0   char[4]  magic "EPV1"
    offset 4   uint16   width   (frame pixels)
    offset 6   uint16   height  (frame rows — panel rows / 2 when line-doubled)
    offset 8   uint16   fps
    offset 10  uint16   flags   (bit 0: line-doubled — each row drives 2 panel rows
                                 bit 1: RLE-compressed frames)
    offset 12  uint32   frame count
    offset 16  frames

Raw frames (flags bit 1 clear) are width*height/4 bytes each, back to back.

RLE frames (bit 1 set, the default) are each prefixed by a uint32 record
header: bits 0-30 = payload length, bit 31 = payload is a raw uncompressed
frame (used when compression doesn't help). Compressed payloads are PackBits:
a signed header byte h, 0..127 → copy h+1 literal bytes, -1..-127 → repeat
the next byte 1-h times, -128 → no-op. High-contrast animation compresses
~10-20x, which takes SD bandwidth out of the picture entirely and keeps the
access pattern sequential (no cluster-chain seeking).

With --line-double each stored frame is half the panel height and the player
passes line_repeat=2 to paintPacked(), which drives every row twice. Halves
the SD bandwidth for no visible cost when the source is already lower
resolution than the panel.

Grey mapping: video luma is converted from sRGB to linear reflectance and
quantised to the 4 drive levels with an ordered (Bayer 8x8) dither. The
calibrated NORMAL waveforms space the greys linearly in reflectance, so
nearest-level in linear space is the perceptually correct mapping. Ordered
dither is used instead of error diffusion because it is temporally stable:
a region whose source value doesn't change produces the identical dither
pattern every frame, so the delta-update engine sees no spurious changes.

Requires: ffmpeg on PATH, numpy.

Example:
    python3 video2epv.py badapple.mp4 badapple.epv --fps 15
"""

import argparse
import struct
import subprocess
import sys

import numpy as np

BAYER8 = np.array([
    [ 0, 32,  8, 40,  2, 34, 10, 42],
    [48, 16, 56, 24, 50, 18, 58, 26],
    [12, 44,  4, 36, 14, 46,  6, 38],
    [60, 28, 52, 20, 62, 30, 54, 22],
    [ 3, 35, 11, 43,  1, 33,  9, 41],
    [51, 19, 59, 27, 49, 17, 57, 25],
    [15, 47,  7, 39, 13, 45,  5, 37],
    [63, 31, 55, 23, 61, 29, 53, 21],
], dtype=np.float32)


def probe_size(path):
    out = subprocess.check_output([
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=width,height",
        "-of", "csv=p=0", path], text=True)
    w, h = out.strip().split("\n")[0].split(",")
    return int(w), int(h)


def packbits_encode(data):
    """PackBits-encode a uint8 array. Runs >= 3 become repeat tokens; shorter
    runs are gathered into literal segments so dithered regions cost only
    ~1/128 overhead instead of doubling."""
    a = np.asarray(data, dtype=np.uint8)
    change = np.flatnonzero(np.diff(a))
    starts = np.concatenate(([0], change + 1))
    lens = np.diff(np.concatenate((starts, [a.size])))
    vals = a[starts]

    out = bytearray()
    lit = bytearray()

    def flush_lit():
        p = 0
        while p < len(lit):
            c = min(128, len(lit) - p)
            out.append(c - 1)
            out.extend(lit[p:p + c])
            p += c
        lit.clear()

    for length, v, s in zip(lens.tolist(), vals.tolist(), starts.tolist()):
        if length >= 3:
            flush_lit()
            while length >= 2:
                c = min(128, length)
                out.append((1 - c) & 0xFF)
                out.append(v)
                length -= c
            if length == 1:
                lit.append(v)
        else:
            lit += a[s:s + length].tobytes()
    flush_lit()
    return bytes(out)


def packbits_decode(data, expected):
    out = bytearray()
    i = 0
    while i < len(data):
        h = data[i]; i += 1
        if h < 128:
            out += data[i:i + h + 1]; i += h + 1
        elif h > 128:
            out += bytes([data[i]]) * (257 - h); i += 1
    assert len(out) == expected
    return bytes(out)


def fit_geometry(src_w, src_h, dst_w, dst_h, mode):
    """Return (scaled_w, scaled_h, x_off, y_off) for the chosen fill mode."""
    if mode == "stretch":
        return dst_w, dst_h, 0, 0
    scale_fit = min(dst_w / src_w, dst_h / src_h)
    scale_crop = max(dst_w / src_w, dst_h / src_h)
    scale = scale_crop if mode == "crop" else scale_fit
    w = int(round(src_w * scale)) & ~1
    h = int(round(src_h * scale)) & ~1
    return w, h, (dst_w - w) // 2, (dst_h - h) // 2


def main():
    ap = argparse.ArgumentParser(description="Convert video to EPD_Painter .epv")
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--fps", type=int, default=15, help="output frame rate (default 15)")
    ap.add_argument("--width", type=int, default=960, help="panel width (default 960)")
    ap.add_argument("--height", type=int, default=540, help="panel height (default 540)")
    ap.add_argument("--fill", choices=["fit", "crop", "stretch"], default="fit",
                    help="fit: letter/pillarbox (default), crop: fill panel and crop, "
                         "stretch: ignore aspect ratio")
    ap.add_argument("--line-double", action="store_true",
                    help="store half-height frames; player drives each row twice "
                         "(halves file size and SD bandwidth)")
    ap.add_argument("--no-rle", action="store_true",
                    help="store raw fixed-size frames instead of RLE-compressed")
    ap.add_argument("--gamma", type=float, default=2.2, help="sRGB->linear exponent (default 2.2)")
    ap.add_argument("--no-dither", action="store_true", help="plain quantise, no ordered dither")
    ap.add_argument("--invert", action="store_true", help="invert black/white")
    ap.add_argument("--preview", metavar="PNG",
                    help="also write frame ~10%% in, decoded from packed data, as a PNG")
    args = ap.parse_args()

    W, H = args.width, args.height
    if W % 4:
        sys.exit("panel width must be a multiple of 4")

    src_w, src_h = probe_size(args.input)
    vw, vh, xoff, yoff = fit_geometry(src_w, src_h, W, H, args.fill)
    vf = f"fps={args.fps},scale={vw}:{vh}"
    if args.fill == "crop":
        vf += f",crop={W}:{H}"
        vw, vh, xoff, yoff = W, H, 0, 0

    # Geometry is computed against the full panel, then squashed vertically:
    # the player un-squashes by driving each stored row twice.
    ysquash = 2 if args.line_double else 1
    if ysquash > 1:
        vh //= ysquash
        yoff //= ysquash
        H //= ysquash
        vf += f",scale={vw}:{vh}"

    print(f"source {src_w}x{src_h} -> video area {vw}x{vh} at +{xoff}+{yoff} "
          f"on {W}x{H} frame"
          f"{' (line-doubled to ' + str(H * ysquash) + ' panel rows)' if ysquash > 1 else ''}"
          f", {args.fps} fps")

    proc = subprocess.Popen(
        ["ffmpeg", "-v", "error", "-i", args.input,
         "-vf", vf, "-f", "rawvideo", "-pix_fmt", "gray", "-"],
        stdout=subprocess.PIPE)

    # Dither threshold tiled over the video area, in units of one grey step.
    bayer = np.tile((BAYER8 + 0.5) / 64.0 - 0.5,
                    ((vh + 7) // 8, (vw + 7) // 8))[:vh, :vw]

    frame_bytes = vw * vh
    frame_size = W * H // 4
    shifts = np.array([6, 4, 2, 0], dtype=np.uint8)
    use_rle = not args.no_rle
    flags = (1 if ysquash > 1 else 0) | (2 if use_rle else 0)
    n = 0
    written = 0
    preview_png = None

    with open(args.output, "wb") as out:
        out.write(struct.pack("<4sHHHHI", b"EPV1", W, H, args.fps, flags, 0))
        while True:
            raw = proc.stdout.read(frame_bytes)
            if len(raw) < frame_bytes:
                break
            grey = np.frombuffer(raw, dtype=np.uint8).reshape(vh, vw)
            lin = (grey.astype(np.float32) / 255.0) ** args.gamma  # reflectance 0..1
            if args.invert:
                lin = 1.0 - lin
            dark = (1.0 - lin) * 3.0                               # 0=white .. 3=black
            if not args.no_dither:
                dark = dark + bayer
            q = np.clip(np.rint(dark), 0, 3).astype(np.uint8)

            canvas = np.zeros((H, W), dtype=np.uint8)              # 0 = white
            canvas[yoff:yoff + vh, xoff:xoff + vw] = q

            packed = (canvas.reshape(H, W // 4, 4) << shifts).astype(np.uint8)
            packed = np.bitwise_or.reduce(packed, axis=2)
            frame = packed.tobytes()
            if use_rle:
                comp = packbits_encode(packed.reshape(-1))
                if n == 0:                       # round-trip check, once
                    assert packbits_decode(comp, frame_size) == frame
                if len(comp) < frame_size:
                    out.write(struct.pack("<I", len(comp)))
                    out.write(comp)
                    written += 4 + len(comp)
                else:                            # incompressible — store raw
                    out.write(struct.pack("<I", 0x80000000 | frame_size))
                    out.write(frame)
                    written += 4 + frame_size
            else:
                out.write(frame)
                written += frame_size

            if args.preview and preview_png is None and n >= 200:
                # decode the packed bytes back — verifies the pack step end-to-end
                pix = np.stack([(packed >> s) & 3 for s in shifts], axis=2).reshape(H, W)
                preview_png = np.repeat((255 - pix * 85).astype(np.uint8),
                                        ysquash, axis=0)
            n += 1
            if n % 500 == 0:
                print(f"  {n} frames...")

        out.seek(12)
        out.write(struct.pack("<I", n))

    proc.wait()
    size_mb = (16 + written) / 1e6
    rle_note = (f", RLE {written / n / 1024:.1f} KB/frame avg "
                f"({n * frame_size / written:.1f}x)" if use_rle and n else "")
    print(f"wrote {args.output}: {n} frames, {W}x{H} @ {args.fps} fps, {size_mb:.0f} MB "
          f"({n / args.fps:.0f}s of video){rle_note}")

    if args.preview and preview_png is not None:
        import zlib
        def png_chunk(tag, data):
            c = tag + data
            return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c))
        ph = preview_png.shape[0]
        rows = b"".join(b"\x00" + preview_png[y].tobytes() for y in range(ph))
        with open(args.preview, "wb") as f:
            f.write(b"\x89PNG\r\n\x1a\n")
            f.write(png_chunk(b"IHDR", struct.pack(">IIBBBBB", W, ph, 8, 0, 0, 0, 0)))
            f.write(png_chunk(b"IDAT", zlib.compress(rows, 6)))
            f.write(png_chunk(b"IEND", b""))
        print(f"preview: {args.preview}")


if __name__ == "__main__":
    main()
