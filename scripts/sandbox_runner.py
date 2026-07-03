"""sandbox_runner.py — locate, invoke, and parse output from the C++ sandbox.

Pure standard library so it runs anywhere. The other scripts import this to run
scenarios and read back the trajectory CSV and metrics JSON.
"""
from __future__ import annotations

import csv
import json
import os
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional


def find_binary(explicit: Optional[str] = None) -> Path:
    """Locate the `sandbox` executable."""
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    if os.environ.get("SANDBOX_BIN"):
        candidates.append(Path(os.environ["SANDBOX_BIN"]))
    here = Path(__file__).resolve().parent
    candidates += [
        here.parent / "build" / "sandbox",
        here.parent / "build" / "Release" / "sandbox",
        Path.cwd() / "build" / "sandbox",
    ]
    for c in candidates:
        if c.exists() and os.access(c, os.X_OK):
            return c
    raise FileNotFoundError(
        "Could not find the `sandbox` binary. Build it first (see README) or pass "
        "--binary / set SANDBOX_BIN."
    )


@dataclass
class Sample:
    step: int
    time: float
    body_id: int
    pos: tuple
    quat: tuple
    linvel: tuple
    angvel: tuple
    active: bool


@dataclass
class RunResult:
    scenario: str
    stdout: str
    metrics: Dict[str, float]
    # body_id -> chronological list of Samples
    tracks: Dict[int, List[Sample]] = field(default_factory=dict)

    def body_ids(self) -> List[int]:
        return sorted(self.tracks.keys())

    def tracked(self, i: int = 0) -> List["Sample"]:
        """Return the trajectory of the scenario's i-th tracked body."""
        key = f"tracked_{i}"
        if key in self.metrics:
            bid = int(self.metrics[key])
            if bid in self.tracks:
                return self.tracks[bid]
        # Fall back to i-th logged body if the scenario didn't tag one.
        return self.tracks[self.body_ids()[i]]


def _read_csv(path: Path) -> Dict[int, List[Sample]]:
    tracks: Dict[int, List[Sample]] = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            bid = int(r["body_id"])
            s = Sample(
                step=int(r["step"]),
                time=float(r["time"]),
                body_id=bid,
                pos=(float(r["px"]), float(r["py"]), float(r["pz"])),
                quat=(float(r["qx"]), float(r["qy"]), float(r["qz"]), float(r["qw"])),
                linvel=(float(r["vx"]), float(r["vy"]), float(r["vz"])),
                angvel=(float(r["wx"]), float(r["wy"]), float(r["wz"])),
                active=(r["active"] == "1"),
            )
            tracks.setdefault(bid, []).append(s)
    return tracks


def run(scenario: str,
        binary: Optional[Path] = None,
        steps: int = 300,
        timestep: Optional[float] = None,
        fixedstep: Optional[float] = None,
        substeps: Optional[int] = None,
        gravity: Optional[float] = None,
        count: Optional[int] = None,
        drop: Optional[float] = None,
        angle: Optional[float] = None,
        friction: Optional[float] = None,
        no_ccd: bool = False,
        want_csv: bool = True,
        extra: Optional[List[str]] = None) -> RunResult:
    """Run one scenario and return parsed metrics + trajectory tracks."""
    binary = Path(binary) if binary else find_binary()

    with tempfile.TemporaryDirectory() as td:
        csv_path = Path(td) / "traj.csv"
        json_path = Path(td) / "metrics.json"
        cmd = [str(binary), "--scenario", scenario, "--steps", str(steps),
               "--metrics", str(json_path)]
        if want_csv:
            cmd += ["--out", str(csv_path)]
        if timestep is not None:  cmd += ["--timestep", repr(timestep)]
        if fixedstep is not None: cmd += ["--fixedstep", repr(fixedstep)]
        if substeps is not None:  cmd += ["--substeps", str(substeps)]
        if gravity is not None:   cmd += ["--gravity", repr(gravity)]
        if count is not None:     cmd += ["--count", str(count)]
        if drop is not None:      cmd += ["--drop", repr(drop)]
        if angle is not None:     cmd += ["--angle", repr(angle)]
        if friction is not None:  cmd += ["--friction", repr(friction)]
        if no_ccd:                cmd += ["--no-ccd"]
        if extra:                 cmd += extra

        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            raise RuntimeError(
                f"sandbox failed ({proc.returncode}): {proc.stderr or proc.stdout}")

        metrics = json.loads(json_path.read_text()) if json_path.exists() else {}
        tracks = _read_csv(csv_path) if (want_csv and csv_path.exists()) else {}

    return RunResult(scenario=scenario, stdout=proc.stdout, metrics=metrics,
                     tracks=tracks)


def list_scenarios(binary: Optional[Path] = None) -> List[str]:
    binary = Path(binary) if binary else find_binary()
    out = subprocess.run([str(binary), "--list"], capture_output=True, text=True).stdout
    names = []
    for line in out.splitlines():
        line = line.strip()
        if line and not line.startswith("Available"):
            names.append(line.split()[0])
    return names
