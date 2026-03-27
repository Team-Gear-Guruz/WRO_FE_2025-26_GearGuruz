"""
S4 – Sensor Dropout Degradation (paper §7.7.4)
===============================================
Simulates progressive HC-SR04 failure on the WRO 2026 rectangular corridor.
Failed sensors return no-echo sentinel (400 cm) → zero repulsion contribution.

12 configurations tested (paper §7.7.4):
  - Baseline (no failures)
  - 6 single-sensor failures (one per position)
  - 4 critical two-sensor pairs: L+FLD, R+FRD, L+R, FLD+FRD
  - 1 three-sensor failure: L+FLD+FC

Results (paper):
  Single-sensor: no extra collisions for FC/diag/rear; +5.4–6.1 cm lateral error for sides
  Two-sensor ipsilateral (L+FLD or R+FRD): 1 collision per 3000 steps, ≈10.5 cm error
  Three-sensor (L+FLD+FC): same collision rate → FC primary role is speed regulation
"""

import math, random
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os, sys

# Import shared modules from S1/S2 simulator
sys.path.insert(0, os.path.dirname(__file__))
from S1_S2_simulator import (Track, Bicycle, VP, VS, Sensors, SM,
                               NavCtrl, DEFAULT_SENSORS)

STEPS = 3000   # simulation steps per configuration (paper §7.7.4)

# Sensor index mapping
SENSOR_NAMES = {
    0: "FC",    # front-centre
    1: "FLD",   # front-left diagonal
    2: "FRD",   # front-right diagonal
    3: "B",     # rear
    4: "L",     # lateral-left
    5: "R",     # lateral-right
}

# 12 failure configurations (paper §7.7.4)
CONFIGS = [
    ("Baseline",    []),
    ("FC fail",     [0]),
    ("FLD fail",    [1]),
    ("FRD fail",    [2]),
    ("B fail",      [3]),
    ("L fail",      [4]),
    ("R fail",      [5]),
    ("L+FLD fail",  [4, 1]),
    ("R+FRD fail",  [5, 2]),
    ("L+R fail",    [4, 5]),
    ("FLD+FRD fail",[1, 2]),
    ("L+FLD+FC",    [4, 1, 0]),
]


def make_faulty_sensors(track, failed_indices):
    """
    Return a Sensors instance whose read() method replaces readings at
    failed_indices with the 400 cm no-echo sentinel.
    """
    base_sens = Sensors(track)

    class FaultySensors:
        def read(self, s, noise=True):
            readings = base_sens.read(s, noise=noise)
            for idx in failed_indices:
                readings[idx] = 400.0   # no-echo sentinel
            return readings

    return FaultySensors()


def run_S4(out="outputs", verbose=True):
    os.makedirs(out, exist_ok=True)

    if verbose:
        print("\n" + "="*60)
        print("S4 – Sensor Dropout Degradation")
        print(f"    ({STEPS} simulation steps per configuration)")
        print("="*60)

    track = Track()
    vp    = VP()
    veh   = Bicycle(vp)
    ctrl  = NavCtrl(k_o=0.008, k_fy=0.0025)
    DT    = vp.dt_ms / 1000.0

    results = []

    for name, failed in CONFIGS:
        sens = make_faulty_sensors(track, failed)

        collisions   = 0
        lateral_errs = []

        # Start in middle of bottom straight, heading east
        s = VS(x=1500.0, y=500.0, theta=0.0, v=0.0)

        for _ in range(STEPS):
            rdgs = sens.read(s)
            δ, thr = ctrl.control(s, rdgs, track)
            s = veh.step(s, δ, thr, DT)

            if not track.in_corridor(s.x, s.y):
                collisions += 1
                # Reset to nearest safe position (centre of current straight)
                s = VS(x=1500.0, y=500.0, theta=0.0, v=0.0)

            lateral_errs.append(abs(track.lateral_offset(s.x, s.y)))

        mean_err = float(np.mean(lateral_errs))
        n_failed = len(failed)
        results.append(dict(name=name, failed=failed, n_failed=n_failed,
                            collisions=collisions, mean_err=mean_err))

        if verbose:
            fail_str = "+".join(SENSOR_NAMES[i] for i in failed) if failed else "none"
            print(f"  {name:20s} | failed={fail_str:12s} | "
                  f"collisions={collisions:4d} | mean_err={mean_err:.1f} cm")

    # ── Figure (paper Fig.10) ─────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("S4 · Sensor Dropout Degradation under Progressive HC-SR04 Failure\n"
                 f"({STEPS} simulation steps per configuration)", fontsize=10)

    color_map = {0: "seagreen", 1: "darkorange", 2: "tomato", 3: "mediumpurple"}
    names      = [r["name"]       for r in results]
    colls      = [r["collisions"] for r in results]
    errs       = [r["mean_err"]   for r in results]
    n_faileds  = [r["n_failed"]   for r in results]
    bar_colors = [color_map.get(n, "gray") for n in n_faileds]

    ax = axes[0]
    bars = ax.bar(range(len(names)), colls, color=bar_colors, alpha=0.85)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=45, ha="right", fontsize=8)
    ax.set_ylabel("Wall collisions per run")
    ax.set_title("(a) Wall Collision Count")
    # Legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(fc=color_map[0], label="0 failed (baseline)"),
        Patch(fc=color_map[1], label="1 failed"),
        Patch(fc=color_map[2], label="2 failed"),
        Patch(fc=color_map[3], label="3 failed"),
    ]
    ax.legend(handles=legend_elements, fontsize=8)
    ax.grid(axis="y", alpha=0.3)

    ax = axes[1]
    ax.bar(range(len(names)), errs, color=bar_colors, alpha=0.85)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=45, ha="right", fontsize=8)
    ax.set_ylabel("Mean lateral tracking error (cm)")
    ax.set_title("(b) Mean Lateral Tracking Error")
    ax.legend(handles=legend_elements, fontsize=8)
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    fig.savefig(f"{out}/S4_sensor_dropout.png", dpi=150, bbox_inches="tight")
    plt.close()

    if verbose:
        print(f"\n  Figure saved: {out}/S4_sensor_dropout.png")

    return results


if __name__ == "__main__":
    run_S4()
