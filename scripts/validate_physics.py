#!/usr/bin/env python3
"""validate_physics.py — validate the simulator against analytic physics.

Goes beyond pass/fail: it prints measured-vs-analytic numbers so you can see how
closely the sandbox tracks textbook results and how the integrator converges as
the fixed step shrinks.

    python3 scripts/validate_physics.py
    python3 scripts/validate_physics.py --binary /path/to/sandbox
"""
from __future__ import annotations

import argparse
import math
import sys

import sandbox_runner as sb


def hr(title: str):
    print(f"\n{title}\n" + "-" * len(title))


def validate_projectile(binary):
    hr("Projectile motion — trajectory vs analytic parabola")
    print(f"{'fixed step':>12} {'max pos err':>14} {'apex err':>12}")
    prev = None
    for fs in (1/60, 1/120, 1/240, 1/480, 1/960):
        # Keep timeStep <= substeps*fixedStep so no simulation time is dropped.
        substeps = max(10, math.ceil((1/60) / fs) + 2)
        r = sb.run("projectile", binary=binary, steps=120, drop=10.0,
                   fixedstep=fs, substeps=substeps)
        m = r.metrics
        g, x0, y0, vx0, vy0 = m["g"], m["x0"], m["y0"], m["vx0"], m["vy0"]
        track = r.tracked(0)
        max_err = max(
            math.hypot(s.pos[0] - (x0 + vx0 * s.time),
                       s.pos[1] - (y0 + vy0 * s.time - 0.5 * g * s.time**2))
            for s in track)
        apex_err = abs(max(s.pos[1] for s in track) - m["apex_y"])
        ratio = f"  (x{prev/max_err:.1f} better)" if prev else ""
        print(f"{fs:12.5f} {max_err:14.5f} {apex_err:12.5f}{ratio}")
        prev = max_err
    print("Error ~ 0.5*g*h*t (semi-implicit Euler); it halves as the step halves.")


def validate_restitution(binary):
    hr("Restitution — measured coefficient from successive bounce heights")
    r = sb.run("bounce", binary=binary, steps=500, drop=8.0)
    track = r.tracked(0)
    ys = [s.pos[1] for s in track]

    # Find bounce peaks: local maxima above the resting height.
    peaks = []
    for i in range(1, len(ys) - 1):
        if ys[i] > ys[i - 1] and ys[i] >= ys[i + 1] and ys[i] > 0.7:
            peaks.append(ys[i] - 0.5)  # height above rest
    print(f"configured restitution e = {r.metrics['restitution']}")
    print(f"{'bounce':>8} {'peak height':>14} {'e = sqrt(h_i/h_(i-1))':>24}")
    for i, h in enumerate(peaks[:5]):
        if i == 0:
            print(f"{i:8d} {h:14.4f} {'(drop)':>24}")
        else:
            e = math.sqrt(h / peaks[i - 1]) if peaks[i - 1] > 0 else 0
            print(f"{i:8d} {h:14.4f} {e:24.4f}")


def validate_energy(binary):
    hr("Energy conservation — frictionless pendulum (point-to-point joint)")
    r = sb.run("pendulum", binary=binary, steps=1200, substeps=20)
    g = r.metrics["g"]
    track = r.tracked(0)
    energies = [0.5 * sum(c * c for c in s.linvel) + g * s.pos[1] for s in track]
    e0, emin, emax = energies[0], min(energies), max(energies)
    mean = sum(energies) / len(energies)
    print(f"initial energy : {e0:.4f} J/kg")
    print(f"min / max      : {emin:.4f} / {emax:.4f} J/kg")
    print(f"drift          : {(emax - emin) / mean * 100:.2f}% of mean")
    print("A sequential-impulse joint leaks a little energy; drift stays small.")


def validate_resting(binary):
    hr("Resting height — dropped sphere settles at exactly its radius")
    for radius in (0.25, 0.5, 1.0):
        # Reuse the bounce scene shape via a direct check: drop from 6, radius set
        # implicitly by scenario is 0.5, so validate that case plus stack tops.
        pass
    r = sb.run("stack", binary=binary, steps=500, count=5)
    top = r.tracked(0)[-1].pos[1]
    print(f"5-box stack, top box rest height: {top:.4f} (expected {r.metrics['expected_top_y']:.4f})")
    r2 = sb.run("bounce", binary=binary, steps=600, drop=4.0)
    final_y = r2.tracked(0)[-1].pos[1]
    print(f"bouncing ball final rest height : {final_y:.4f} (expected 0.5000 = radius)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--binary", help="path to the sandbox executable")
    args = ap.parse_args()
    binary = sb.find_binary(args.binary)
    print(f"Using sandbox binary: {binary}")

    validate_projectile(binary)
    validate_restitution(binary)
    validate_energy(binary)
    validate_resting(binary)
    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
