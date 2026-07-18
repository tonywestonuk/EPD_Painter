#!/usr/bin/env python3
"""Closed-loop waveform tuner for EPD_Painter (QUALITY_NORMAL grey levels).

Iterates: paint chart -> scan -> measure -> nudge waveform rows -> repeat,
until dark grey and light grey sit at even L* spacing between the panel's
black and white (targets 33.3 / 66.7), then runs a return-to-white ghost
check. Every edit keeps the round trip DC-balanced: across a grey's darker
row + lighter row, #dark pulses (1) always equals #light pulses (2).

Usage:
  python3 tune.py [--max-iters 12] [--tol 3] [--tag session1]
"""

import argparse, datetime, json, os, re, sys, time

import rig

TOL_DEFAULT = 3.0        # acceptable |L* error|
TARGETS = {"dark_grey": 1 / 3, "light_grey": 2 / 3}   # fraction of L* range
GREY_ROW = {"light_grey": 0, "dark_grey": 1}          # waveform table row


# ------------------------------------------------------------ wave edits ----
def balance(nd, nl):
    return (nd.count(1) + nl.count(1)) - (nd.count(2) + nl.count(2))


def _flip_last(row, frm, to):
    for i in range(len(row) - 1, -1, -1):
        if row[i] == frm:
            row[i] = to
            return True
    return False


def _flip_first(row, frm, to):
    for i in range(len(row)):
        if row[i] == frm:
            row[i] = to
            return True
    return False


# Lesson from the T5 sessions: the darker row's TRAILING elements dominate the
# reached grey, so edit there; the lighter row's compensations must go at the
# FRONT — a trailing dark pulse in a lighter row leaves ghosting.

def darken_step(nd, nl):
    """One balanced step darker. Mutates copies; returns new (nd, nl) or None."""
    nd, nl = nd[:], nl[:]
    # Darker-row move (B+1), strongest-preference first
    if not (_flip_last(nd, 2, 3) or _flip_last(nd, 3, 1)):
        return None  # darker row already all-dark
    # Lighter-row compensation (B-1), front-loaded
    if not (_flip_first(nl, 1, 3) or _flip_first(nl, 3, 2)):
        return None
    return nd, nl


def lighten_step(nd, nl):
    nd, nl = nd[:], nl[:]
    if not (_flip_last(nd, 3, 2) or _flip_last(nd, 1, 3)):
        return None
    if not (_flip_first(nl, 3, 1) or _flip_first(nl, 2, 3)):
        return None
    return nd, nl


# ---------------------------------------------------------------- serial ----
def read_tables(s):
    lines = rig.command(s, "G", terminator=("DONE",))
    tables = {}
    for ln in lines:
        m = re.match(r"([FNH][LD]) (\d):((?: \d)+)", ln)
        if m:
            tables.setdefault(m.group(1), {})[int(m.group(2))] = \
                [int(v) for v in m.group(3).split()]
    return tables


def write_row(s, tbl, row, vals):
    resp = rig.command(s, f"W {tbl} {row} " + " ".join(map(str, vals)))
    if not resp[-1].startswith("OK"):
        sys.exit(f"Board rejected W {tbl} {row}: {resp}")


def chart_and_measure(s, tag, transform=None):
    lines = rig.command(s, "P", terminator=("DONE",))
    temp = re.search(r"T=(-?\d+)", lines[-1])
    temp = int(temp.group(1)) if temp else None
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    png = os.path.join(rig.RESULTS_DIR, f"{stamp}_{tag}_T{temp}.png")
    os.makedirs(rig.RESULTS_DIR, exist_ok=True)
    rig.scan(png)
    out, tr = rig.measure(png, transform)
    rig.log_result(tag, temp, out)
    return out, tr, temp


# ------------------------------------------------------------------ main ----
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-iters", type=int, default=12)
    ap.add_argument("--tol", type=float, default=TOL_DEFAULT)
    ap.add_argument("--tag", default="tune")
    ap.add_argument("--port")
    a = ap.parse_args()

    s = rig.open_serial(a.port)
    tables = read_tables(s)
    nd, nl = tables["ND"], tables["NL"]
    for row in (0, 1):
        b = balance(nd[row], nl[row])
        if b:
            sys.exit(f"Starting tables unbalanced at row {row} (B={b}) — fix first")

    transform = None
    history = []
    prev_errs = {}
    for it in range(1, a.max_iters + 1):
        out, transform, temp = chart_and_measure(s, f"{a.tag}_i{it}", transform)
        lb, lw = out["black"]["lstar"], out["white"]["lstar"]
        errs, edits = {}, []
        for grey, frac in TARGETS.items():
            target = lb + frac * (lw - lb)
            err = out[grey]["lstar"] - target        # + = too light
            errs[grey] = err
            if abs(err) <= a.tol:
                continue
            row = GREY_ROW[grey]
            steps = 1 if abs(err) < 15 else 2 if abs(err) < 25 else 3
            # Overshoot damping: if the error changed sign since last
            # iteration we jumped over the target — creep back one step.
            if grey in prev_errs and prev_errs[grey] * err < 0:
                steps = 1
            cur_nd, cur_nl = nd[row], nl[row]
            for _ in range(steps):
                stepped = (darken_step if err > 0 else lighten_step)(cur_nd, cur_nl)
                if stepped is None:
                    print(f"  {grey}: no more headroom in that direction")
                    break
                cur_nd, cur_nl = stepped
            if (cur_nd, cur_nl) != (nd[row], nl[row]):
                nd[row], nl[row] = cur_nd, cur_nl
                edits.append((row, cur_nd, cur_nl))

        prev_errs = errs
        print(f"[iter {it}] T={temp}°C  "
              + "  ".join(f"{g}: err {e:+.1f}L*" for g, e in errs.items()), flush=True)
        history.append({"iter": it, "temp": temp, "errs": errs,
                        "ND": {r: v[:] for r, v in nd.items()},
                        "NL": {r: v[:] for r, v in nl.items()}})

        if not edits:
            if all(abs(e) <= a.tol for e in errs.values()):
                print("Converged.")
            else:
                print("No further moves possible; stopping.")
            break
        for row, new_nd, new_nl in edits:
            assert balance(new_nd, new_nl) == 0, "edit broke DC balance"
            write_row(s, "ND", row, new_nd)
            write_row(s, "NL", row, new_nl)
            print(f"  row {row}: ND -> {new_nd}")
            print(f"  row {row}: NL -> {new_nl}")

    # Ghost check: return to white, rescan with the cached transform.
    print("\nGhost check (paint white, rescan)...")
    rig.command(s, "U", terminator=("DONE",))
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    png = os.path.join(rig.RESULTS_DIR, f"{stamp}_{a.tag}_ghost.png")
    rig.scan(png)
    ghost, _ = rig.measure(png, transform)
    worst = max(abs(ghost[n]["mean"] - ghost["white"]["mean"]) for n in rig.PATCH_NAMES)
    print(f"Worst patch-region residual vs white: {worst:.1f} grey levels "
          f"({'OK' if worst < 4 else 'GHOSTING — lighter rows need work'})")

    final = {"temp_end": history[-1]["temp"] if history else None,
             "normal_darker": [nd[r] for r in (0, 1, 2)] if 2 in nd else
                              [nd.get(0), nd.get(1)],
             "normal_lighter": [nl[r] for r in (0, 1, 2)] if 2 in nl else
                               [nl.get(0), nl.get(1)],
             "history": history}
    out_path = os.path.join(rig.RESULTS_DIR, f"{a.tag}_final.json")
    with open(out_path, "w") as f:
        json.dump(final, f, indent=1)
    print(f"\nFinal tables saved to {out_path}")
    print("C initializers (rows 0-2):")
    for name, t in (("normal_darker", nd), ("normal_lighter", nl)):
        rows = ",\n                    ".join(
            "{ " + ", ".join(map(str, t[r])) + " }" for r in sorted(t))
        print(f"  .{name} = {{ {rows} }}")


if __name__ == "__main__":
    main()
