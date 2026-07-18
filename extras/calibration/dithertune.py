#!/usr/bin/env python3
"""Dither-match waveform tuner — the standard calibration method.

Uses the wavecal 'M' chart: each grey patch is split into a dithered
reference half (66%/33% black dots made of the panel's own black+white) and a
driven half. Correct grey levels make the halves match, which puts the four
levels at linear-reflectance spacing — exactly what dither() assumes. The
reference is in-frame, so scanner tone response cancels out.

Usage:
  python3 dithertune.py [--condition 8] [--max-iters 10] [--tol 3] [--tag x]
"""

import argparse, os, sys

import numpy as np
from PIL import Image

import rig
from tune import darken_step, lighten_step, balance

GREY_ROW = {"light_grey": 0, "dark_grey": 1}


def measure_match(tag):
    """Paint nothing — scan the current M chart and return per-grey
    (driven - dithered) deltas plus the transform."""
    png = os.path.join(rig.RESULTS_DIR, f"{tag}.png")
    rig.scan(png)
    out, coef = rig.measure(png)
    img = np.asarray(Image.open(png).convert("L"), float)

    def region_mean(x0, x1):
        vals = []
        for px in np.linspace(x0 + 12, x1 - 12, 20):
            for py in np.linspace(rig.PATCH_Y + 25, rig.PATCH_Y + rig.PATCH_H - 25, 40):
                sx = int(round(coef[0] * px + coef[1] * py + coef[2]))
                sy = int(round(coef[3] * px + coef[4] * py + coef[5]))
                vals.append(img[sy, sx])
        return float(np.mean(vals))

    deltas = {}
    for slot, grey in ((1, "dark_grey"), (2, "light_grey")):
        x = rig.PATCH_X[slot]
        deltas[grey] = region_mean(x + rig.PATCH_W / 2, x + rig.PATCH_W) \
            - region_mean(x, x + rig.PATCH_W / 2)
    return deltas, coef


def read_tables(s):
    import re
    lines = rig.command(s, "G", terminator=("DONE",))
    tables = {}
    for ln in lines:
        m = re.match(r"([FNH][LD]) (\d):((?: \d)+)", ln)
        if m:
            tables.setdefault(m.group(1), {})[int(m.group(2))] = \
                [int(v) for v in m.group(3).split()]
    return tables


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--condition", type=int, default=8)
    ap.add_argument("--max-iters", type=int, default=10)
    ap.add_argument("--tol", type=float, default=3.0)
    ap.add_argument("--tag", default="dtune")
    ap.add_argument("--port")
    a = ap.parse_args()

    s = rig.open_serial(a.port)
    tables = read_tables(s)
    nd, nl = tables["ND"], tables["NL"]
    for row in (0, 1):
        b = balance(nd[row], nl[row])
        if b:
            sys.exit(f"Starting tables unbalanced at row {row} (B={b})")

    if a.condition:
        print(f"Conditioning: {a.condition} black/white cycles...", flush=True)
        for _ in range(a.condition):
            rig.command(s, "K", terminator=("DONE",))
            rig.command(s, "U", terminator=("DONE",))

    prev = {}
    for it in range(1, a.max_iters + 1):
        rig.command(s, "M", terminator=("DONE",))
        deltas, coef = measure_match(f"{a.tag}_i{it}")
        print(f"[iter {it}] " + "  ".join(f"{g}: {d:+.1f}" for g, d in deltas.items()),
              flush=True)
        edits = False
        for grey, delta in deltas.items():
            if abs(delta) <= a.tol:
                continue
            row = GREY_ROW[grey]
            steps = 1 if abs(delta) < 12 else 2
            if grey in prev and prev[grey] * delta < 0:
                steps = 1
            cur_nd, cur_nl = nd[row], nl[row]
            for _ in range(steps):
                stepped = (lighten_step if delta < 0 else darken_step)(cur_nd, cur_nl)
                if stepped is None:
                    break
                cur_nd, cur_nl = stepped
            if (cur_nd, cur_nl) == (nd[row], nl[row]):
                print(f"   {grey}: no move available", flush=True)
                continue
            assert balance(cur_nd, cur_nl) == 0
            nd[row], nl[row] = cur_nd, cur_nl
            rig.command(s, f"W ND {row} " + " ".join(map(str, cur_nd)))
            rig.command(s, f"W NL {row} " + " ".join(map(str, cur_nl)))
            print(f"   ND{row} -> {cur_nd}\n   NL{row} -> {cur_nl}", flush=True)
            edits = True
        prev = deltas
        if not edits:
            print("CONVERGED" if all(abs(d) <= a.tol for d in deltas.values())
                  else "No moves left", flush=True)
            break

    # Ghost check
    rig.command(s, "U", terminator=("DONE",))
    png = os.path.join(rig.RESULTS_DIR, f"{a.tag}_ghost.png")
    rig.scan(png)
    out2, _ = rig.measure(png, coef)
    worst = max(abs(out2[n]["mean"] - out2["white"]["mean"]) for n in rig.PATCH_NAMES)
    print(f"ghost residual: {worst:.1f} ({'CLEAN' if worst < 4 else 'ghosting'})", flush=True)

    print("\nFinal C initializers:")
    print("  .normal_lighter rows 0-2:")
    for r in (0, 1, 2):
        print(f"    {{ {', '.join(map(str, nl[r]))} }}")
    print("  .normal_darker rows 0-2:")
    for r in (0, 1, 2):
        print(f"    {{ {', '.join(map(str, nd[r]))} }}")


if __name__ == "__main__":
    main()
