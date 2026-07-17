#!/usr/bin/env python3
"""Temperature sweep characterization.

As the chilled panel warms on the scanner bed, wait for each temperature
milestone; at each one, exercise the sluggish ink (black flood, white flood,
twice), then paint the patch chart and scan/measure it. Waveforms are NOT
changed — this captures the pure temperature response of the current tables.

Usage:
  python3 sweep.py [--milestones 10,15,20,24] [--port /dev/...]
"""

import argparse, datetime, os, re, sys, time

import rig


def read_temp(s):
    lines = rig.command(s, "T", terminator=("TEMP",))
    return int(lines[-1].split()[1])


def flood(s, cmd):
    lines = rig.command(s, cmd, terminator=("DONE",))
    m = re.search(r"T=(-?\d+)", lines[-1])
    return int(m.group(1)) if m else None


def chart(s, tag, transform):
    lines = rig.command(s, "P", terminator=("DONE",))
    m = re.search(r"T=(-?\d+)", lines[-1])
    temp = int(m.group(1)) if m else None
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    png = os.path.join(rig.RESULTS_DIR, f"{stamp}_{tag}_T{temp}.png")
    os.makedirs(rig.RESULTS_DIR, exist_ok=True)
    rig.scan(png)
    out, tr = rig.measure(png, transform)
    rig.log_result(tag, temp, out)
    return out, tr, temp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--milestones", default="10,15,20,24")
    ap.add_argument("--port")
    a = ap.parse_args()
    milestones = [int(m) for m in a.milestones.split(",")]

    s = rig.open_serial(a.port)
    transform = None
    t = read_temp(s)
    print(f"Start temp: {t}°C, milestones: {milestones}", flush=True)

    for m in milestones:
        if t > m + 2:
            print(f"-- milestone {m}°C already passed (now {t}°C), skipping", flush=True)
            continue
        while t < m:
            time.sleep(15)
            t = read_temp(s)
            print(f"   waiting for {m}°C... now {t}°C", flush=True)
        print(f"== milestone {m}°C (reading {t}°C): exercising panel", flush=True)
        for cmd in ("K", "U", "K", "U"):
            t2 = flood(s, cmd)
            print(f"   flood {cmd} done (T={t2})", flush=True)
        out, transform, temp = chart(s, f"sweep{m}", transform)
        t = read_temp(s)
        print(f"== milestone {m}°C complete (panel now {t}°C)\n", flush=True)

    print("Sweep finished.", flush=True)


if __name__ == "__main__":
    main()
