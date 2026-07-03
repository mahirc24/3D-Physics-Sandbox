#!/usr/bin/env python3
"""plot_trajectories.py — plot body trajectories from a scenario run.

Runs a scenario (or reads an existing CSV) and plots height-vs-time and the X-Y
path for the tracked bodies. Requires matplotlib.

    python3 scripts/plot_trajectories.py --scenario bounce --steps 400 --out bounce.png
    python3 scripts/plot_trajectories.py --scenario projectile --out proj.png
"""
from __future__ import annotations

import argparse
import sys

import sandbox_runner as sb


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--binary", help="path to the sandbox executable")
    ap.add_argument("--scenario", default="bounce")
    ap.add_argument("--steps", type=int, default=400)
    ap.add_argument("--count", type=int)
    ap.add_argument("--drop", type=float)
    ap.add_argument("--out", default="trajectory.png")
    args = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is required for plotting: pip install matplotlib")
        return 1

    binary = sb.find_binary(args.binary)
    r = sb.run(args.scenario, binary=binary, steps=args.steps,
               count=args.count, drop=args.drop)

    # Choose which bodies to draw: the tracked ones, else up to 6 logged bodies.
    tracked_n = int(r.metrics.get("tracked_count", 0))
    if tracked_n:
        ids = [int(r.metrics[f"tracked_{i}"]) for i in range(tracked_n)]
    else:
        ids = r.body_ids()[:6]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    cmap = plt.get_cmap("tab10")
    for k, bid in enumerate(ids):
        track = r.tracks.get(bid, [])
        if not track:
            continue
        t = [s.time for s in track]
        x = [s.pos[0] for s in track]
        y = [s.pos[1] for s in track]
        color = cmap(k % 10)
        ax1.plot(t, y, color=color, label=f"body {bid}")
        ax2.plot(x, y, color=color, label=f"body {bid}")

    ax1.set_xlabel("time (s)"); ax1.set_ylabel("height y (m)")
    ax1.set_title(f"{args.scenario}: height vs time")
    ax1.grid(True, alpha=0.3); ax1.legend(fontsize=8)

    ax2.set_xlabel("x (m)"); ax2.set_ylabel("y (m)")
    ax2.set_title(f"{args.scenario}: X-Y path")
    ax2.grid(True, alpha=0.3); ax2.set_aspect("equal", adjustable="datalim")
    ax2.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print(f"Wrote {args.out} ({len(ids)} bodies, {args.steps} steps)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
