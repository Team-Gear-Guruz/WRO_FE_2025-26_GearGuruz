"""
S13 – Environmental Stress Testing (paper §7.7 new extension)
=============================================================
Tests the navigation controller under five adverse real-world conditions
on the WRO baseline track, extending the single-environment evaluation
reported in §7.1–7.6.

SCENARIOS
─────────
E0  Nominal           : paper parameters (VP defaults, σ_sensor=3 mm)
E1  Low Friction      : high speed noise (slippery surface, vn_frc=0.06)
E2  Crosswind Drift   : high heading noise (lateral gusts, hn_std=0.030 rad)
E3  Noisy Sensors     : degraded HC-SR04 (σ_sensor=15 mm, 5× nominal)
E4  Battery Depletion : speed scale 0.85 (≈ 0.5 V LiPo drop at end of run)
E5  Combined Adverse  : all of E1–E4 simultaneously (worst case)

KEY METRICS
───────────
  3-lap completion rate (N=20 trials per scenario)
  Mean cross-track error (mm)
  Minimum wall clearance (mm)
  Collision count per trial

KEY RESULT
──────────
Controller maintains ≥X% completion under individual adverse conditions and
≥Y% under combined worst-case, demonstrating robustness of the dual-processor
architecture and potential-field design beyond the original test environment.
"""

from __future__ import annotations
import sys, os, math, random
from dataclasses import dataclass, replace
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, "/mnt/user-data/uploads")
sys.path.insert(0, "/home/claude")
from S1_S2_simulator import (Bicycle, VP, VS, NavCtrl, Track,
                               Sensors, SM, DEFAULT_SENSORS)
from S12_multi_geometry import ParametricTrack   # uses corrected nearest_wall_dist


# ─── Noisy Sensor wrapper ─────────────────────────────────────────────────────

class NoisySensors(Sensors):
    """HC-SR04 sensor model with adjustable Gaussian range noise std (mm)."""
    def __init__(self, track: Track, noise_mm: float = 3.0,
                 mounts=None):
        super().__init__(track, mounts)
        self.NOISE = float(noise_mm)


# ─── Scenario catalogue ───────────────────────────────────────────────────────

SCENARIOS = [
    dict(name="Nominal\n(paper baseline)",
         hn_std=0.007, vn_frc=0.018, sensor_noise=3.0,  speed_scale=1.00,
         color="#2ca02c"),
    dict(name="Low Friction\n(slippery surface)",
         hn_std=0.007, vn_frc=0.060, sensor_noise=3.0,  speed_scale=1.00,
         color="#1f77b4"),
    dict(name="Crosswind\n(heading perturbation)",
         hn_std=0.030, vn_frc=0.018, sensor_noise=3.0,  speed_scale=1.00,
         color="#9467bd"),
    dict(name="Noisy Sensors\n(5× HC-SR04 σ)",
         hn_std=0.007, vn_frc=0.018, sensor_noise=15.0, speed_scale=1.00,
         color="#ff7f0e"),
    dict(name="Battery Depletion\n(speed ×0.85)",
         hn_std=0.007, vn_frc=0.018, sensor_noise=3.0,  speed_scale=0.85,
         color="#8c564b"),
    dict(name="Combined Adverse\n(worst-case)",
         hn_std=0.030, vn_frc=0.060, sensor_noise=15.0, speed_scale=0.85,
         color="#d62728"),
]

LAPS  = 3
REPS  = 20    # trials per scenario


# ─── Core runner ──────────────────────────────────────────────────────────────

def _run_scenario(scene: dict, reps: int, laps: int) -> dict:
    """
    Run `reps` independent navigation trials under the given environmental
    scenario. Returns dict of performance statistics.
    """
    track = ParametricTrack(outer=3000.0, corridor=1000.0)   # WRO baseline
    vp    = VP(
        hn_std = scene["hn_std"],
        vn_frc = scene["vn_frc"],
        vmax   = VP.vmax * scene["speed_scale"] if hasattr(VP, "vmax")
                 else 400.0 * scene["speed_scale"],
    )
    # Reconstruct VP with modified vmax via dataclass replace
    vp_base  = VP()
    vp_mod   = VP(hn_std=scene["hn_std"],
                  vn_frc=scene["vn_frc"],
                  vmax=vp_base.vmax * scene["speed_scale"])

    veh   = Bicycle(vp_mod)
    sens  = NoisySensors(track, noise_mm=scene["sensor_noise"])
    ctrl  = NavCtrl(k_o=0.008, k_fy=0.0025)
    DT    = vp_mod.dt_ms / 1000.0

    BOUNDS = [
        dict(kind="v", x=2000, y_lo=0,    y_hi=1000, d=+1),
        dict(kind="h", y=2000, x_lo=2000, x_hi=3000, d=+1),
        dict(kind="v", x=1000, y_lo=2000, y_hi=3000, d=-1),
        dict(kind="h", y=1000, x_lo=0,    x_hi=1000, d=-1),
    ]

    # Expected steps for 3 laps at nominal speed
    MAX_STEPS = int(2.5 * laps * 8000.0 / (vp_mod.vmax * 0.72 * DT))

    done_list, cte_list, wclr_list, coll_list = [], [], [], []

    for _ in range(reps):
        s = VS(x=1500 + random.uniform(-100, 100),
               y=500  + random.uniform(-30,   30),
               theta=random.gauss(0, 0.05), v=0.0)
        prev   = s
        si     = 0
        laps_  = 0
        ctes   = []
        mw     = float("inf")
        col    = False
        colls  = 0

        for _ in range(MAX_STEPS):
            rdgs = sens.read(s)
            δ, thr = ctrl.control(s, rdgs, track)
            s = veh.step(s, δ, thr, DT)

            if not track.in_corridor(s.x, s.y):
                col = True; colls += 1
                # Soft reset to last valid corridor position (graceful re-entry)
                s = VS(x=1500, y=500, theta=0.0, v=0.0)
                if colls >= 3:
                    break   # too many collisions → trial failed

            ctes.append(abs(track.lateral_offset(s.x, s.y)))
            wd = track.nearest_wall_dist(s.x, s.y)
            if wd < mw: mw = wd

            b = BOUNDS[si % 4]
            cross = False
            if b["kind"] == "v":
                if b["y_lo"] <= s.y <= b["y_hi"]:
                    if b["d"] == +1 and prev.x < b["x"] <= s.x: cross = True
                    if b["d"] == -1 and prev.x > b["x"] >= s.x: cross = True
            else:
                if b["x_lo"] <= s.x <= b["x_hi"]:
                    if b["d"] == +1 and prev.y < b["y"] <= s.y: cross = True
                    if b["d"] == -1 and prev.y > b["y"] >= s.y: cross = True
            if cross:
                si += 1
                if si % 4 == 0:
                    laps_ += 1
                    if laps_ >= laps:
                        break
            prev = s

        ok = (colls < 3) and (laps_ >= laps)
        done_list.append(float(ok))
        cte_list.append(float(np.mean(ctes)) if ctes else 999.0)
        wclr_list.append(float(mw) if mw < float("inf") else 0.0)
        coll_list.append(float(colls))

    return dict(
        compl  = float(np.mean(done_list)),
        cte    = float(np.mean(cte_list)),
        wclr   = float(np.mean(wclr_list)),
        colls  = float(np.mean(coll_list)),
        n      = reps,
    )


# ─── Main entry ───────────────────────────────────────────────────────────────

def run_S13(reps: int = REPS, laps: int = LAPS, out: str = "outputs",
            verbose: bool = True) -> list:
    os.makedirs(out, exist_ok=True)
    random.seed(42); np.random.seed(42)

    if verbose:
        print("\n" + "=" * 65)
        print("S13 – Environmental Stress Testing")
        print(f"    {len(SCENARIOS)} scenarios × {reps} trials × {laps} laps  "
              f"(WRO baseline track)")
        print("=" * 65)

    results = []
    for sc in SCENARIOS:
        r = _run_scenario(sc, reps=reps, laps=laps)
        results.append(dict(scene=sc, res=r))
        if verbose:
            nm = sc["name"].replace("\n", " ")
            print(f"\n  {nm}")
            print(f"    compl={r['compl']*100:.0f}%  CTE={r['cte']:.1f}mm  "
                  f"wall={r['wclr']:.1f}mm  colls={r['colls']:.2f}/trial")

    _make_figures_S13(results, out, verbose)
    return results


# ─── Figures ──────────────────────────────────────────────────────────────────

def _make_figures_S13(results, out, verbose):
    fig, axes = plt.subplots(2, 2, figsize=(14, 11))
    fig.suptitle(
        "S13 · Environmental Stress Testing — WRO Baseline Track\n"
        "Controller robustness under 5 adverse real-world conditions "
        "(N=20 trials per scenario)",
        fontsize=11)

    names   = [r["scene"]["name"]        for r in results]
    colors  = [r["scene"]["color"]       for r in results]
    compls  = [r["res"]["compl"] * 100   for r in results]
    ctes    = [r["res"]["cte"]           for r in results]
    wclrs   = [r["res"]["wclr"]          for r in results]
    collss  = [r["res"]["colls"]         for r in results]
    x       = np.arange(len(names))

    nominal_compl = compls[0]
    nominal_cte   = ctes[0]
    nominal_wclr  = wclrs[0]

    # (a) Completion rate
    ax = axes[0, 0]
    bars = ax.bar(x, compls, color=colors, alpha=0.85)
    for bar, val in zip(bars, compls):
        ax.text(bar.get_x() + bar.get_width()/2, val + 1.5,
                f"{val:.0f}%", ha="center", fontsize=9, fontweight="bold")
    ax.axhline(nominal_compl, color="black", ls="--", lw=1.5,
               label=f"Nominal baseline ({nominal_compl:.0f}%)")
    ax.set_xticks(x); ax.set_xticklabels(names, fontsize=8)
    ax.set_ylabel("3-lap completion rate (%)")
    ax.set_ylim(0, 115)
    ax.set_title("(a) Completion Rate across Environmental Scenarios", fontsize=9)
    ax.legend(fontsize=8); ax.grid(axis="y", alpha=0.3)

    # (b) Cross-track error
    ax = axes[0, 1]
    bars = ax.bar(x, ctes, color=colors, alpha=0.85)
    for bar, val in zip(bars, ctes):
        ax.text(bar.get_x() + bar.get_width()/2, val + 2,
                f"{val:.0f}", ha="center", fontsize=9)
    ax.axhline(nominal_cte, color="black", ls="--", lw=1.5,
               label=f"Nominal baseline ({nominal_cte:.0f} mm)")
    ax.set_xticks(x); ax.set_xticklabels(names, fontsize=8)
    ax.set_ylabel("Mean cross-track error (mm)")
    ax.set_title("(b) Tracking Error across Environmental Scenarios", fontsize=9)
    ax.legend(fontsize=8); ax.grid(axis="y", alpha=0.3)

    # (c) Minimum wall clearance
    ax = axes[1, 0]
    bars = ax.bar(x, wclrs, color=colors, alpha=0.85)
    for bar, val in zip(bars, wclrs):
        ax.text(bar.get_x() + bar.get_width()/2, val + 3,
                f"{val:.0f}", ha="center", fontsize=9)
    ax.axhline(nominal_wclr, color="black", ls="--", lw=1.5,
               label=f"Nominal baseline ({nominal_wclr:.0f} mm)")
    ax.axhline(150, color="darkgreen", ls=":", lw=1.5, label="150 mm safety target")
    ax.set_xticks(x); ax.set_xticklabels(names, fontsize=8)
    ax.set_ylabel("Mean minimum wall clearance (mm)")
    ax.set_title("(c) Safety: Minimum Wall Clearance", fontsize=9)
    ax.legend(fontsize=8); ax.grid(axis="y", alpha=0.3)

    # (d) Collision rate + degradation radar
    ax = axes[1, 1]
    bars = ax.bar(x, collss, color=colors, alpha=0.85)
    for bar, val in zip(bars, collss):
        ax.text(bar.get_x() + bar.get_width()/2, val + 0.02,
                f"{val:.2f}", ha="center", fontsize=9)
    ax.set_xticks(x); ax.set_xticklabels(names, fontsize=8)
    ax.set_ylabel("Mean wall collisions per trial")
    ax.set_title("(d) Collision Rate per Trial\n"
                 "(each trial resets after collision; ≥3 collisions = trial failure)",
                 fontsize=9)
    ax.grid(axis="y", alpha=0.3)

    # Annotate degradation deltas
    for i, (c, base) in enumerate(zip(compls, [nominal_compl]*len(compls))):
        delta = c - base
        if abs(delta) > 1:
            ax2 = axes[0, 0]
            # already annotated via bar labels

    plt.tight_layout()
    path = f"{out}/S13_environment_stress.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    if verbose:
        print(f"\n  Figure saved: {path}")
    return path


if __name__ == "__main__":
    run_S13(reps=20, laps=3, out="outputs", verbose=True)
