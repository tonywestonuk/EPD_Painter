#!/usr/bin/env python3
"""Scanner calibration rig for EPD_Painter waveforms.

The wavecal sketch renders a patch chart (black / dark grey / light grey /
white, with 40px black corner fiducials); this script commands it over serial,
scans the panel face-down on the Canon MG5200 (SANE pixma backend), locates
the panel via the fiducials, and measures each patch's reflectance.

Usage:
  python3 rig.py temp                      # read panel temperature
  python3 rig.py chart [--tag NAME]        # P -> scan -> measure -> log
  python3 rig.py measure scan.png          # re-measure an existing scan
  python3 rig.py send "W ND 1 1 1 ..."     # raw serial command
  python3 rig.py get                       # dump current waveforms
"""

import argparse, csv, datetime, glob, os, re, subprocess, sys, time

import numpy as np
from PIL import Image

try:
    import serial  # pyserial
except ImportError:
    serial = None

BAUD = 115200
SCAN_DPI = 300
PANEL_DPI = 223.0
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")

# Panel-space geometry — must match wavecal.ino
FID = 40
FID_INSET = 20
FID_CENTERS = [(40.0, 40.0), (920.0, 40.0), (920.0, 500.0), (40.0, 500.0)]  # TL TR BR BL
PATCH_W, PATCH_H, PATCH_Y = 160, 280, 130
PATCH_X = [100, 300, 500, 700]
PATCH_NAMES = ["black", "dark_grey", "light_grey", "white"]


# ---------------------------------------------------------------- serial ----
def find_port():
    ports = glob.glob("/dev/cu.usbmodem*")
    if not ports:
        sys.exit("No /dev/cu.usbmodem* port — is the board plugged in?")
    return ports[0]


def open_serial(port=None):
    if serial is None:
        sys.exit("pyserial missing: pip3 install pyserial --break-system-packages")
    port = port or find_port()
    s = serial.Serial(port, BAUD, timeout=1)
    time.sleep(0.3)
    s.reset_input_buffer()
    return s


def command(s, cmd, terminator=("DONE", "OK", "ERR", "TEMP", "READY", "BQTEMP"), timeout=120):
    s.write((cmd + "\n").encode())
    lines, t0 = [], time.time()
    while time.time() - t0 < timeout:
        raw = s.readline().decode(errors="replace").strip()
        if not raw:
            continue
        lines.append(raw)
        if any(raw.startswith(t) for t in terminator):
            return lines
    sys.exit(f"Timeout waiting for response to {cmd!r}: {lines}")


# ------------------------------------------------------------------ scan ----
def discover_scanner():
    out = subprocess.run(["scanimage", "-L"], capture_output=True, text=True, timeout=60).stdout
    m = re.search(r"`(pixma:[^']+)'", out)
    if not m:
        sys.exit(f"Scanner not found by scanimage -L (deep sleep? press its power button):\n{out}")
    return m.group(1)


def scan(out_path):
    dev = discover_scanner()
    print(f"Scanning on {dev} ...")
    with open(out_path, "wb") as f:
        subprocess.run(
            ["scanimage", "-d", dev, "--resolution", str(SCAN_DPI),
             "--mode", "Gray", "--format=png"],
            stdout=f, check=True, timeout=300)
    return out_path


# --------------------------------------------------------- panel locator ----
def find_fiducials(img):
    """Return 4 fiducial centers (x, y) in scan pixels.

    A cold, sluggish panel paints its "black" fiducials only ~20 grey levels
    darker than the ghost-textured background, so thresholding is hopeless.
    Instead: matched box filter (dark square vs surrounding ring — the ring
    averages away high-frequency ghosting), then find 4 response peaks whose
    geometry matches the known fiducial rectangle.
    """
    scale = SCAN_DPI / PANEL_DPI               # panel px -> scan px
    side = int(round(FID * scale))

    ii = np.zeros((img.shape[0] + 1, img.shape[1] + 1))
    ii[1:, 1:] = np.cumsum(np.cumsum(img, 0), 1)

    def box_mean(y0, x0, h, w):                # arrays of top-left corners
        return (ii[y0 + h, x0 + w] - ii[y0, x0 + w]
                - ii[y0 + h, x0] + ii[y0, x0]) / (h * w)

    stride = 2
    ring = side                                 # ring margin around the square
    ys = np.arange(ring, img.shape[0] - 2 * side - ring, stride)
    xs = np.arange(ring, img.shape[1] - 2 * side - ring, stride)
    Y, X = np.meshgrid(ys, xs, indexing="ij")
    inner = box_mean(Y, X, side, side)
    outer_sum = box_mean(Y - ring, X - ring, side + 2 * ring, side + 2 * ring) \
        * (side + 2 * ring) ** 2
    ring_mean = (outer_sum - inner * side * side) / ((side + 2 * ring) ** 2 - side ** 2)
    resp = ring_mean - inner                   # high = dark square, light ring

    # Non-max suppression: greedily take the strongest well-separated peaks.
    order = np.argsort(resp, axis=None)[::-1]
    peaks = []
    for idx in order:
        if resp.flat[idx] <= 4:                # below any plausible fiducial
            break
        py, px = np.unravel_index(idx, resp.shape)
        cy, cx = ys[py] + side / 2, xs[px] + side / 2
        if all((cy - q[0]) ** 2 + (cx - q[1]) ** 2 > (2 * side) ** 2 for q in peaks):
            peaks.append((cy, cx, resp.flat[idx]))
        if len(peaks) >= 40:
            break

    # Geometric match: find two diagonal pairs sharing a midpoint whose
    # lengths fit the fiducial rectangle's diagonal.
    (x0, y0), (x1, _), (_, y2) = FID_CENTERS[0], FID_CENTERS[1], FID_CENTERS[3]
    rect_w, rect_h = (x1 - x0) * scale, (y2 - y0) * scale
    diag = (rect_w ** 2 + rect_h ** 2) ** 0.5
    diag_pairs = []
    for i in range(len(peaks)):
        for j in range(i + 1, len(peaks)):
            d = ((peaks[i][0] - peaks[j][0]) ** 2 + (peaks[i][1] - peaks[j][1]) ** 2) ** 0.5
            if 0.9 * diag < d < 1.12 * diag:
                diag_pairs.append((i, j))
    best, best_err = None, 1e18
    for a in range(len(diag_pairs)):
        i, j = diag_pairs[a]
        m1 = ((peaks[i][0] + peaks[j][0]) / 2, (peaks[i][1] + peaks[j][1]) / 2)
        for b in range(a + 1, len(diag_pairs)):
            k, l = diag_pairs[b]
            if len({i, j, k, l}) != 4:
                continue
            m2 = ((peaks[k][0] + peaks[l][0]) / 2, (peaks[k][1] + peaks[l][1]) / 2)
            mid_err = ((m1[0] - m2[0]) ** 2 + (m1[1] - m2[1]) ** 2) ** 0.5
            if mid_err > side:                 # diagonals of a rectangle bisect
                continue
            quad = [peaks[i], peaks[k], peaks[j], peaks[l]]
            sides = []
            for q in range(4):
                p, n = quad[q], quad[(q + 1) % 4]
                sides.append(((p[0] - n[0]) ** 2 + (p[1] - n[1]) ** 2) ** 0.5)
            side_err = (abs(sides[0] - sides[2]) + abs(sides[1] - sides[3])
                        + abs(min(sides) / max(sides) - min(rect_w, rect_h) / max(rect_w, rect_h)) * diag)
            err = mid_err * 2 + side_err
            if err < best_err:
                best, best_err = quad, err
    if best is None:
        sys.exit(f"No fiducial rectangle found among {len(peaks)} peaks — is the panel on the bed?")
    return [(x, y) for y, x, _ in best]


def fit_affine(panel_pts, scan_pts):
    """Least-squares affine panel->scan for a given correspondence order."""
    A, b = [], []
    for (px, py), (sx, sy) in zip(panel_pts, scan_pts):
        A.append([px, py, 1, 0, 0, 0]); b.append(sx)
        A.append([0, 0, 0, px, py, 1]); b.append(sy)
    A, b = np.array(A), np.array(b)
    coef, *_ = np.linalg.lstsq(A, b, rcond=None)
    resid = np.abs(A @ coef - b).max()
    return coef, resid


def panel_transforms(scan_pts):
    """All 8 orientation hypotheses (4 rotations x mirror) as affine coefs.

    The fiducial quad is a symmetric rectangle and an affine fit absorbs
    aspect, so every hypothesis fits with ~zero residual — geometry alone
    cannot disambiguate. The caller must pick the hypothesis whose sampled
    patches make physical sense (see measure()).
    """
    pts = np.array(scan_pts)
    c = pts.mean(axis=0)
    order = np.argsort(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))
    cyc = [scan_pts[i] for i in order]
    out = []
    for direction in (1, -1):
        seq = cyc if direction == 1 else cyc[::-1]
        for shift in range(4):
            trial = seq[shift:] + seq[:shift]
            coef, r = fit_affine(FID_CENTERS, trial)
            if r < FID * (SCAN_DPI / PANEL_DPI):
                out.append(coef)
    if not out:
        sys.exit("Fiducial geometry didn't fit any orientation — bad scan?")
    return out


def measure(png_path, transform=None):
    """Measure the patch chart. Returns (results, transform); pass a previous
    transform to skip fiducial detection (e.g. for an all-white ghost scan of
    a panel that has not moved)."""
    img = np.asarray(Image.open(png_path).convert("L"), dtype=np.float64)

    def sample_patches(coef):
        h, w = img.shape
        res = {}
        for name, x0 in zip(PATCH_NAMES, PATCH_X):
            cx, cy = x0 + PATCH_W / 2, PATCH_Y + PATCH_H / 2
            vals = []
            for dx in np.linspace(-PATCH_W / 2 + 25, PATCH_W / 2 - 25, 24):
                for dy in np.linspace(-PATCH_H / 2 + 25, PATCH_H / 2 - 25, 40):
                    px, py = cx + dx, cy + dy
                    sx = int(round(coef[0] * px + coef[1] * py + coef[2]))
                    sy = int(round(coef[3] * px + coef[4] * py + coef[5]))
                    vals.append(img[min(max(sy, 0), h - 1), min(max(sx, 0), w - 1)])
            res[name] = (float(np.mean(vals)), float(np.std(vals)))
        return res

    if transform is not None:
        results = sample_patches(transform)
    else:
        # Every orientation of the symmetric fiducial rectangle fits the
        # affine equally well; pick the one whose patches read in the
        # physically expected order (black darkest ... white lightest) with
        # the largest contrast.
        best_coef, results, best_score = None, None, -1e18
        for coef in panel_transforms(find_fiducials(img)):
            res = sample_patches(coef)
            means = [res[n][0] for n in PATCH_NAMES]
            monotonic = all(means[i] <= means[i + 1] + 2 for i in range(3))
            score = (means[3] - means[0]) + (1000 if monotonic else 0)
            if score > best_score:
                best_coef, results, best_score = coef, res, score
        transform = best_coef

    blk, wht = results["black"][0], results["white"][0]
    print(f"\n{png_path}")
    print(f"{'patch':<11}{'mean':>7}{'std':>7}{'reflect':>9}{'L*':>7}")
    out = {}
    for name in PATCH_NAMES:
        mean, std = results[name]
        refl = (mean - blk) / (wht - blk) if wht != blk else 0.0
        y = max(refl, 0.0)
        lstar = 116 * y ** (1 / 3) - 16 if y > 0.008856 else 903.3 * y
        out[name] = {"mean": mean, "std": std, "reflect": refl, "lstar": lstar}
        print(f"{name:<11}{mean:7.1f}{std:7.1f}{refl:9.3f}{lstar:7.1f}")
    # Even-L* targets between measured black and white:
    lb, lw = out["black"]["lstar"], out["white"]["lstar"]
    print(f"targets    dark_grey L*={lb + (lw - lb) / 3:.1f}  "
          f"light_grey L*={lb + 2 * (lw - lb) / 3:.1f}")
    return out, transform


# ------------------------------------------------------------------ main ----
def log_result(tag, temp, out):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    path = os.path.join(RESULTS_DIR, "results.csv")
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["timestamp", "tag", "temp_c"] +
                       [f"{n}_{k}" for n in PATCH_NAMES for k in ("mean", "reflect", "lstar")])
        w.writerow([datetime.datetime.now().isoformat(timespec="seconds"), tag, temp] +
                   [round(out[n][k], 3) for n in PATCH_NAMES for k in ("mean", "reflect", "lstar")])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["temp", "chart", "measure", "send", "get"])
    ap.add_argument("arg", nargs="?")
    ap.add_argument("--tag", default="")
    ap.add_argument("--port")
    a = ap.parse_args()

    if a.action == "measure":
        measure(a.arg)
        return

    s = open_serial(a.port)
    if a.action == "temp":
        print("\n".join(command(s, "T")))
    elif a.action == "send":
        print("\n".join(command(s, a.arg)))
    elif a.action == "get":
        print("\n".join(command(s, "G", terminator=("DONE",))))
    elif a.action == "chart":
        lines = command(s, "P", terminator=("DONE",))
        print("\n".join(lines))
        temp = ""
        m = re.search(r"T=(-?\d+)", lines[-1])
        if m:
            temp = m.group(1)
        os.makedirs(RESULTS_DIR, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = a.tag or "chart"
        png = os.path.join(RESULTS_DIR, f"{stamp}_{tag}_T{temp}.png")
        scan(png)
        out, _ = measure(png)
        log_result(tag, temp, out)


if __name__ == "__main__":
    main()
