#!/usr/bin/env python3
"""One-shot cold session: run the MOMENT the chilled board is plugged in.

Sequence (no waiting, coldest-first):
  1. temps (TPS + BQ) -> log
  2. chart paint + scan at the coldest point
  3. repeat chart every ~2 min while warming, logging temps, until TPS
     reaches cutoff (default 20°C) or max iterations.

The board must already be flashed with wavecal (it is — flashed warm).

Usage: python3 coldrun.py [--cutoff 20] [--max-iters 10]
"""

import argparse, datetime, os, re, sys, time

import rig


def temps(s):
    t = int(rig.command(s, "T", terminator=("TEMP",))[-1].split()[1])
    m = re.search(r"BQTEMP ([-\d.]+)", rig.command(s, "J", terminator=("BQTEMP", "ERR"))[-1])
    bq = float(m.group(1)) if m else None
    return t, bq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cutoff", type=float, default=20.0)
    ap.add_argument("--max-iters", type=int, default=10)
    ap.add_argument("--port")
    a = ap.parse_args()

    s = rig.open_serial(a.port)
    transform = None
    for it in range(1, a.max_iters + 1):
        t, bq = temps(s)
        print(f"[cold iter {it}] TPS={t}°C BQ={bq}°C", flush=True)
        lines = rig.command(s, "P", terminator=("DONE",))
        m = re.search(r"T=(-?\d+)", lines[-1])
        t_paint = int(m.group(1)) if m else t
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        png = os.path.join(rig.RESULTS_DIR, f"{stamp}_cold{it}_T{t_paint}.png")
        os.makedirs(rig.RESULTS_DIR, exist_ok=True)
        rig.scan(png)
        try:
            out, transform = rig.measure(png, None)
            rig.log_result(f"cold{it}", t_paint, out)
        except SystemExit as e:
            print(f"   measurement failed ({e}); scan kept for later analysis", flush=True)
        if t_paint >= a.cutoff and it > 1:
            print(f"Cutoff {a.cutoff}°C reached — cold window over.", flush=True)
            break
        time.sleep(60)
    print("Cold run complete.", flush=True)


if __name__ == "__main__":
    main()
