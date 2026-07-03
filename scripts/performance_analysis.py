#!/usr/bin/env python3
"""performance_analysis.py — benchmark the sandbox.

Measures step time as the body count grows (the `perf` scenario) and times every
standard scenario once. Prints a table and, if matplotlib is available, writes a
scaling plot. This is the performance-analysis half of the testing workflow.

    python3 scripts/performance_analysis.py
    python3 scripts/performance_analysis.py --counts 100 200 400 800 --plot perf.png
"""
from __future__ import annotations

import argparse
import sys

import sandbox_runner as sb


def bench_scaling(binary, counts, steps):
    print("Body-count scaling (perf scenario)")
    print(f"{'bodies':>8} {'mean ms/step':>14} {'steps/sec':>12} {'contacts':>10}")
    rows = []
    for n in counts:
        r = sb.run("perf", binary=binary, steps=steps, count=n, want_csv=False)
        m = r.metrics
        rows.append((n, m["step_ms_mean"], m["steps_per_second"], m["contact_points"]))
        print(f"{n:8d} {m['step_ms_mean']:14.4f} {m['steps_per_second']:12.1f} "
              f"{int(m['contact_points']):10d}")
    return rows


def bench_scenarios(binary, steps):
    print("\nPer-scenario timing")
    print(f"{'scenario':>14} {'mean ms/step':>14} {'steps/sec':>12} {'bodies':>8}")
    for name in sb.list_scenarios(binary):
        r = sb.run(name, binary=binary, steps=steps, want_csv=False)
        m = r.metrics
        print(f"{name:>14} {m['step_ms_mean']:14.4f} {m['steps_per_second']:12.1f} "
              f"{int(m['total_bodies']):8d}")


def make_plot(rows, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print(f"(matplotlib not installed — skipping plot {path})")
        return
    ns = [r[0] for r in rows]
    ms = [r[1] for r in rows]
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(ns, ms, "o-", color="#c0392b", linewidth=2, markersize=6)
    ax.set_xlabel("number of rigid bodies")
    ax.set_ylabel("mean step time (ms)")
    ax.set_title("Physics sandbox — step time vs body count")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"\nWrote scaling plot -> {path}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--binary", help="path to the sandbox executable")
    ap.add_argument("--counts", type=int, nargs="+",
                    default=[50, 100, 200, 400, 800])
    ap.add_argument("--steps", type=int, default=300)
    ap.add_argument("--plot", help="write a scaling plot to this PNG path")
    args = ap.parse_args()

    binary = sb.find_binary(args.binary)
    print(f"Using sandbox binary: {binary}\n")

    rows = bench_scaling(binary, args.counts, args.steps)
    bench_scenarios(binary, args.steps)
    if args.plot:
        make_plot(rows, args.plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
