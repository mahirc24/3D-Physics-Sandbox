#!/usr/bin/env python3
"""run_tests.py — automated regression harness.

Runs every scenario and validates the outputs against expected physical behavior,
exactly the "benchmark outputs against expected physical behavior" workflow. Pure
standard library. Exits non-zero if any check fails.

    python3 scripts/run_tests.py            # run all
    python3 scripts/run_tests.py --binary /path/to/sandbox
"""
from __future__ import annotations

import argparse
import math
import sys
from typing import List, Tuple

import sandbox_runner as sb


class Checker:
    def __init__(self, name: str):
        self.name = name
        self.results: List[Tuple[bool, str]] = []

    def check(self, cond: bool, msg: str):
        self.results.append((bool(cond), msg))

    def near(self, got: float, want: float, tol: float, msg: str):
        self.check(abs(got - want) <= tol,
                   f"{msg}: got {got:.4f}, want {want:.4f} ±{tol}")

    def passed(self) -> bool:
        return all(ok for ok, _ in self.results)


def peak_after_contact(track, contact_y: float) -> float:
    """Highest y reached after the body first descends to contact_y."""
    contacted = False
    peak = -1e9
    for s in track:
        if s.pos[1] <= contact_y:
            contacted = True
        if contacted:
            peak = max(peak, s.pos[1])
    return peak


def energy_series(track, g: float, mass: float = 1.0):
    out = []
    for s in track:
        v2 = sum(c * c for c in s.linvel)
        ke = 0.5 * mass * v2
        pe = mass * g * s.pos[1]
        out.append(ke + pe)
    return out


# --- per-scenario validators -------------------------------------------------
def t_stack(binary) -> Checker:
    c = Checker("stack")
    r = sb.run("stack", binary=binary, steps=400, count=6)
    top = r.metrics["expected_top_y"]
    track = r.tracked(0)
    c.near(track[-1].pos[1], top, 0.15, "top box resting height")
    c.check(not track[-1].active, "top box came to rest (asleep)")
    return c


def t_bounce(binary) -> Checker:
    c = Checker("bounce")
    r = sb.run("bounce", binary=binary, steps=250, drop=5.0)
    track = r.tracked(0)
    peak = peak_after_contact(track, 0.55)
    c.near(peak, r.metrics["expected_first_peak_y"], 0.4, "first-bounce peak")
    return c


def t_projectile(binary) -> Checker:
    c = Checker("projectile")
    # Semi-implicit Euler differs from the continuous parabola by ~0.5*g*h*t,
    # so use a fine fixed step to converge. steps=120 (t=2s), h=1/480 -> ~0.02.
    r = sb.run("projectile", binary=binary, steps=120, drop=10.0, fixedstep=1/480.0)
    m = r.metrics
    track = r.tracked(0)
    g, x0, y0, vx0, vy0 = m["g"], m["x0"], m["y0"], m["vx0"], m["vy0"]
    max_err = 0.0
    for s in track:
        t = s.time
        ax = x0 + vx0 * t
        ay = y0 + vy0 * t - 0.5 * g * t * t
        max_err = max(max_err, math.hypot(s.pos[0] - ax, s.pos[1] - ay))
    c.check(max_err < 0.04, f"trajectory matches analytic parabola (max err {max_err:.4f})")
    apex = max(s.pos[1] for s in track)
    c.near(apex, m["apex_y"], 0.05, "apex height")
    return c


def t_pendulum(binary) -> Checker:
    c = Checker("pendulum")
    r = sb.run("pendulum", binary=binary, steps=600, substeps=10, extra=None)
    g = r.metrics["g"]
    track = r.tracked(0)
    L = r.metrics["rod_length"]
    pivot_y = r.metrics["initial_height"]
    # rod length maintained
    max_dev = max(abs(math.dist(s.pos, (0.0, pivot_y, 0.0)) - L) for s in track)
    c.check(max_dev < 0.10, f"rod length maintained (max dev {max_dev:.4f})")
    # energy roughly conserved (sequential-impulse joints leak a little)
    e = energy_series(track, g)
    drift = (max(e) - min(e)) / (sum(e) / len(e))
    c.check(drift < 0.15, f"energy conserved within 15% (drift {drift*100:.1f}%)")
    return c


def t_friction(binary) -> Checker:
    c = Checker("friction_ramp")
    # mu=0.8 > tan(25)=0.466 -> box should stay put
    r = sb.run("friction_ramp", binary=binary, steps=300, angle=25.0, friction=0.8)
    track = r.tracked(0)
    start = track[0].pos
    disp = max(math.dist(s.pos, start) for s in track)
    c.check(r.metrics["predicted_static"] == 1.0, "predicted static (mu > tan theta)")
    c.check(disp < 0.15, f"box stays on ramp (max displacement {disp:.4f})")

    # Now a slippery ramp: mu=0.2 < tan(35)=0.70 -> should slide
    r2 = sb.run("friction_ramp", binary=binary, steps=300, angle=35.0, friction=0.2)
    tr2 = r2.tracked(0)
    disp2 = math.dist(tr2[-1].pos, tr2[0].pos)
    c.check(r2.metrics["predicted_static"] == 0.0, "predicted sliding (mu < tan theta)")
    c.check(disp2 > 1.0, f"box slides down slippery ramp (displacement {disp2:.4f})")
    return c


def t_domino(binary) -> Checker:
    c = Checker("domino")
    r = sb.run("domino", binary=binary, steps=400, count=8)
    last = r.tracked(0)
    # Standing center y=0.75; a toppled thin domino lies much lower.
    c.check(last[-1].pos[1] < 0.55,
            f"last domino toppled (final y {last[-1].pos[1]:.3f})")
    return c


def t_ccd(binary) -> Checker:
    c = Checker("ccd")
    on = sb.run("ccd", binary=binary, steps=200)
    off = sb.run("ccd", binary=binary, steps=200, no_ccd=True)
    wall_x = on.metrics["wall_x"]
    x_on = on.tracked(0)[-1].pos[0]
    x_off = off.tracked(0)[-1].pos[0]
    c.check(x_on < wall_x + 0.2, f"with CCD sphere stopped at wall (x={x_on:.2f})")
    c.check(x_off > wall_x + 2.0, f"without CCD sphere tunnels through (x={x_off:.2f})")
    return c


def t_raycast(binary) -> Checker:
    c = Checker("raycast")
    r = sb.run("raycast", binary=binary, steps=30, want_csv=False)
    m = r.metrics
    c.near(m["raycast_down_point_x"], m["box_left_x"], 0.01, "pick-down hits left box")
    c.check(m["los_through_row_blocked"] == 1.0, "line-of-sight through row is blocked")
    c.check(m["los_above_row_clear"] == 1.0, "line-of-sight above row is clear")
    c.check(m["raycast_nearest_dist"] > 0, "nearest-pick returned a hit")
    return c


VALIDATORS = [t_stack, t_bounce, t_projectile, t_pendulum,
              t_friction, t_domino, t_ccd, t_raycast]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--binary", help="path to the sandbox executable")
    args = ap.parse_args()

    binary = sb.find_binary(args.binary)
    print(f"Using sandbox binary: {binary}\n")

    total = 0
    failed = 0
    for v in VALIDATORS:
        chk = v(binary)
        print(f"[{chk.name}]")
        for ok, msg in chk.results:
            total += 1
            failed += 0 if ok else 1
            print(f"  {'PASS' if ok else 'FAIL'}  {msg}")
        print()

    passed = total - failed
    print(f"{'='*50}\n{passed}/{total} checks passed"
          + ("" if failed == 0 else f"  ({failed} FAILED)"))
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
