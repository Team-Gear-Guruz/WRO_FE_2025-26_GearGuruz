"""
S14 – Speed × Geometry Operating Envelope (paper §7.7 new extension)
====================================================================
Produces a comprehensive operating-envelope map by crossing 5 vehicle speeds
against 4 track geometries, using S8-scaled gains for each geometry.

This simulation integrates the S3 analytical safety argument (reaction distance
vs. latency) with the S12 kinematic geometry validation: for each (speed,
geometry) operating point, it reports both the SIMULATED completion rate and the
ANALYTICAL safety margin for vision-only vs. Arduino architectures, showing
which combinations require the dual-processor split (S8 Rule 1) and whether the
scaled gains (S8 Rule 2) maintain performance across all of them.

GEOMETRIES  (subset of S12 – chosen for breadth)
─────────────────────────────────────────────────
G1  WRO Baseline      : OW=3000, W=1000 mm
G2  Narrow            : OW=3000, W=750  mm
G3  Wide              : OW=3000, W=1250 mm
G4  Large+Wide        : OW=4000, W=1500 mm

SPEEDS  (cm/s → mm/s for simulator)
────────────────────────────────────
v ∈ {10, 15, 20, 25, 30} cm/s
  → vmax ∈ {100, 150, 200, 250, 300} mm/s  (paper baseline: 25 cm/s = 250 mm/s)

METRICS (per operating point, REPS=5 trials)
────────────────────────────────────────────
  Kinematic : 3-lap completion rate, mean CTE, min wall clearance
  Analytical: vision-only safety margin at that speed (S3 Eq.)
              dual-processor safety margin at that speed

OUTPUT FIGURES
──────────────
  (a) Completion rate heatmap: speed × geometry
  (b) Min wall clearance heatmap: speed × geometry
  (c) Safety margin comparison: vision-only vs. Arduino across speeds
  (d) Combined operating envelope with necessity boundary overlay (S8 Rule 1)
"""

from __future__ import annotations
import sys, os, math, random
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, "/mnt/user-data/uploads")
from S1_S2_simulator import Bicycle, VP, VS, Sensors, NavCtrl, Track
sys.path.insert(0, "/home/claude")
from S12_multi_geometry import ParametricTrack, _run_config

# ─── Experiment grid ──────────────────────────────────────────────────────────

GEOMETRIES_S14 = [
    dict(name="WRO Baseline\n(W=1000)",    outer=3000, corridor=1000),
    dict(name="Narrow\n(W=750)",           outer=3000, corridor=750),
    dict(name="Wide\n(W=1250)",            outer=3000, corridor=1250),
    dict(name="Large+Wide\n(OW=4000,W=1500)", outer=4000, corridor=1500),
]

SPEEDS_CMS = [10.0, 15.0, 20.0, 25.0, 30.0]   # cm/s

REPS = 5
LAPS = 3

# S3 analytical constants
HARD_STOP_CM   = 15.0
DECEL_A        = 80.0    # cm/s²
ARDUINO_RT_S   = 0.020   # s
PI_LATENCY_S   = 0.0909  # mean (Table 8)


def _safety_margin_vision(v_cms, tau_s=PI_LATENCY_S, a=DECEL_A, d=HARD_STOP_CM):
    """Vision-only safety margin (cm): d_safe - v*tau - v²/2a."""
    return d - v_cms * tau_s - (v_cms ** 2) / (2.0 * a)


def _safety_margin_arduino(v_cms, rt_s=ARDUINO_RT_S, a=DECEL_A, d=HARD_STOP_CM):
    """Arduino-loop safety margin (cm)."""
    return d - v_cms * rt_s - (v_cms ** 2) / (2.0 * a)


# ─── Main entry ───────────────────────────────────────────────────────────────

def run_S14(reps: int = REPS, laps: int = LAPS, out: str = "outputs",
            verbose: bool = True) -> dict:
    os.makedirs(out, exist_ok=True)
    random.seed(42); np.random.seed(42)

    nG = len(GEOMETRIES_S14)
    nS = len(SPEEDS_CMS)

    if verbose:
        print("\n" + "=" * 65)
        print("S14 – Speed × Geometry Operating Envelope")
        print(f"    {nG} geometries × {nS} speeds × {reps} reps × {laps} laps")
        print("=" * 65)

    compl_grid = np.zeros((nG, nS))
    wclr_grid  = np.zeros((nG, nS))
    cte_grid   = np.zeros((nG, nS))

    for gi, g in enumerate(GEOMETRIES_S14):
        track = ParametricTrack(outer=g["outer"], corridor=g["corridor"])
        sc    = track.scaled_gains()

        for si, v_cms in enumerate(SPEEDS_CMS):
            vmax_mm_s = v_cms * 10.0   # cm/s → mm/s
            vp = VP(vmax=vmax_mm_s)

            r = _run_config(track, ko=sc["ko"], kfy=sc["kfy"],
                            reps=reps, laps=laps, vp=vp)

            compl_grid[gi, si] = r["compl"]
            wclr_grid [gi, si] = r["wclr"]
            cte_grid  [gi, si] = r["cte"]

            if verbose:
                nm = g["name"].replace("\n", " ")
                print(f"  {nm:30s} v={v_cms:.0f}cm/s → "
                      f"compl={r['compl']*100:.0f}%  "
                      f"CTE={r['cte']:.0f}mm  wall={r['wclr']:.0f}mm")

    # Analytical safety margins across speeds
    margins_vision  = [_safety_margin_vision(v)  for v in SPEEDS_CMS]
    margins_arduino = [_safety_margin_arduino(v) for v in SPEEDS_CMS]

    _make_figures_S14(
        compl_grid, wclr_grid, cte_grid,
        margins_vision, margins_arduino, out, verbose)

    return dict(compl=compl_grid, wclr=wclr_grid, cte=cte_grid,
                margins_vision=margins_vision, margins_arduino=margins_arduino)


# ─── Figures ──────────────────────────────────────────────────────────────────

def _make_figures_S14(compl, wclr, cte, m_vis, m_ard, out, verbose):
    fig, axes = plt.subplots(2, 2, figsize=(14, 11))
    fig.suptitle(
        "S14 · Speed × Geometry Operating Envelope\n"
        "S8-Scaled Gains; Kinematic completion + Analytical safety margin "
        "(S3 Eq.)",
        fontsize=11)

    geom_labels  = [g["name"] for g in GEOMETRIES_S14]
    speed_labels = [f"{v:.0f}" for v in SPEEDS_CMS]

    # (a) Completion rate heatmap
    ax = axes[0, 0]
    im = ax.imshow(compl * 100, origin="upper", cmap="RdYlGn",
                   vmin=0, vmax=100, aspect="auto")
    plt.colorbar(im, ax=ax, label="Completion rate (%)")
    ax.set_xticks(range(len(SPEEDS_CMS))); ax.set_xticklabels(speed_labels)
    ax.set_yticks(range(len(GEOMETRIES_S14))); ax.set_yticklabels(geom_labels, fontsize=8)
    ax.set_xlabel("Vehicle speed (cm/s)")
    ax.set_title("(a) 3-Lap Completion Rate (%)\n"
                 "S8-Scaled Gains — green=safe, red=collision-prone", fontsize=9)
    for i in range(compl.shape[0]):
        for j in range(compl.shape[1]):
            ax.text(j, i, f"{compl[i,j]*100:.0f}%",
                    ha="center", va="center", fontsize=9,
                    color="black" if compl[i,j] > 0.4 else "white")

    # (b) Min wall clearance heatmap
    ax = axes[0, 1]
    im = ax.imshow(wclr, origin="upper", cmap="RdYlGn",
                   vmin=0, vmax=400, aspect="auto")
    plt.colorbar(im, ax=ax, label="Min wall clearance (mm)")
    ax.set_xticks(range(len(SPEEDS_CMS))); ax.set_xticklabels(speed_labels)
    ax.set_yticks(range(len(GEOMETRIES_S14))); ax.set_yticklabels(geom_labels, fontsize=8)
    ax.set_xlabel("Vehicle speed (cm/s)")
    ax.set_title("(b) Mean Minimum Wall Clearance (mm)", fontsize=9)
    for i in range(wclr.shape[0]):
        for j in range(wclr.shape[1]):
            ax.text(j, i, f"{wclr[i,j]:.0f}",
                    ha="center", va="center", fontsize=9)

    # (c) Analytical safety margins: vision-only vs Arduino across speeds
    ax = axes[1, 0]
    ax.plot(SPEEDS_CMS, m_vis, "r-o", lw=2.0, ms=7,
            label=f"Vision-only (τ={PI_LATENCY_S*1000:.0f}ms)")
    ax.plot(SPEEDS_CMS, m_ard, "b-s", lw=2.0, ms=7,
            label=f"Arduino loop (τ={ARDUINO_RT_S*1000:.0f}ms)")
    ax.axhline(0, color="black", lw=0.8)
    ax.fill_between(SPEEDS_CMS, m_vis, 0,
                    where=[m < 0 for m in m_vis],
                    color="red", alpha=0.20, label="Vision-only unsafe zone")
    ax.fill_between(SPEEDS_CMS, m_ard, 0,
                    where=[m > 0 for m in m_ard],
                    color="blue", alpha=0.10, label="Arduino safe zone")
    # Mark the speeds tested in kinematic simulation
    ax.scatter(SPEEDS_CMS, m_vis, color="red",  s=80, zorder=5)
    ax.scatter(SPEEDS_CMS, m_ard, color="blue", s=80, zorder=5)
    ax.set_xlabel("Vehicle speed (cm/s)")
    ax.set_ylabel("Safety margin (cm)  [positive=safe]")
    ax.set_title("(c) S3 Safety Margin Analysis across Tested Speeds\n"
                 "Arduino loop maintains positive margin at all tested speeds",
                 fontsize=9)
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (d) Mean CTE heatmap
    ax = axes[1, 1]
    im = ax.imshow(cte, origin="upper", cmap="RdYlGn_r",
                   aspect="auto")
    plt.colorbar(im, ax=ax, label="Mean CTE (mm)")
    ax.set_xticks(range(len(SPEEDS_CMS))); ax.set_xticklabels(speed_labels)
    ax.set_yticks(range(len(GEOMETRIES_S14))); ax.set_yticklabels(geom_labels, fontsize=8)
    ax.set_xlabel("Vehicle speed (cm/s)")
    ax.set_title("(d) Mean Cross-Track Error (mm)\n"
                 "Lower is better; tracks S8 Rule 2 validity across speeds",
                 fontsize=9)
    for i in range(cte.shape[0]):
        for j in range(cte.shape[1]):
            ax.text(j, i, f"{cte[i,j]:.0f}",
                    ha="center", va="center", fontsize=9)

    plt.tight_layout()
    path = f"{out}/S14_speed_geometry_sweep.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    if verbose:
        print(f"\n  Figure saved: {path}")
    return path


if __name__ == "__main__":
    run_S14(reps=5, laps=3, out="outputs", verbose=True)
