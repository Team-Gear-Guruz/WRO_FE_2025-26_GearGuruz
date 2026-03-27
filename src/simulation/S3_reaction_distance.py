"""
S3 – Reaction Distance Analysis (paper §7.7.3)
===============================================
Analytically characterises the safety benefit of the Arduino's independent
50 Hz loop vs a vision-only pipeline as a function of vision latency.

Key quantity: reaction distance = vehicle speed × reaction interval
  Arduino   : reaction interval = 1 control tick = 20 ms (constant)
  Vision-only: reaction interval = full pipeline latency (varies)

Total stop distance = reaction distance + braking distance (v²/2a)
Safety margin = hard-stop threshold (15 cm) − total stop distance

Result (paper §7.7.3):
  At 25 cm/s, Arduino reaction distance = 0.5 cm (4.5× less than vision mean)
  Vision-only margin becomes negative at ≈190 ms latency → structural necessity
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os

# ─── Physical constants (paper §7.7.3) ────────────────────────────────────────
SPEEDS_CMS  = [15.0, 20.0, 25.0, 35.0]          # representative speeds (cm/s)
COMP_SPEED  = 25.0                                # competition speed
DECEL_A     = 80.0                                # deceleration (cm/s²) – 5 physical measurements
DECEL_A_LO  = 60.0                                # sensitivity lower bound (paper §7.7.3)
HARD_STOP   = 15.0                                # safety boundary (cm, paper §5.2)
ARDUINO_RT  = 0.020                               # Arduino reaction interval (s) = 20 ms
PI_LATENCY_MEAN = 0.0909                          # measured mean Pi latency (s, Table 8)
PI_LATENCY_MAX  = 0.1183                          # measured worst-case Pi latency (s)

latency_ms  = np.linspace(0, 300, 500)            # x-axis: vision pipeline latency (ms)
latency_s   = latency_ms / 1000.0


def reaction_dist(speed_cms, interval_s):
    """Distance travelled during reaction interval (cm)."""
    return speed_cms * interval_s


def braking_dist(speed_cms, a=DECEL_A):
    """Braking distance v²/2a (cm)."""
    return (speed_cms ** 2) / (2 * a)


def stop_dist(speed_cms, interval_s, a=DECEL_A):
    return reaction_dist(speed_cms, interval_s) + braking_dist(speed_cms, a)


def safety_margin(speed_cms, interval_s, a=DECEL_A):
    return HARD_STOP - stop_dist(speed_cms, interval_s, a)


def threshold_latency_ms(speed_cms, a=DECEL_A):
    """
    Vision latency above which safety margin becomes negative (cm/s).
    Solve: HARD_STOP = speed*t + speed²/2a  →  t = (HARD_STOP - v²/2a) / v
    """
    bd  = braking_dist(speed_cms, a)
    rem = HARD_STOP - bd
    if rem <= 0:
        return 0.0
    return (rem / speed_cms) * 1000.0   # ms


def run_S3(out="outputs", verbose=True):
    os.makedirs(out, exist_ok=True)

    if verbose:
        print("\n" + "="*60)
        print("S3 – Reaction Distance Analysis")
        print("="*60)

    v = COMP_SPEED
    ard_rd  = reaction_dist(v, ARDUINO_RT)
    vis_rd_mean = reaction_dist(v, PI_LATENCY_MEAN)
    ratio   = vis_rd_mean / ard_rd

    bd      = braking_dist(v, DECEL_A)
    ard_sd  = ard_rd  + bd
    vis_sd_mean = vis_rd_mean + bd
    margin_mean = HARD_STOP - vis_sd_mean
    margin_max  = HARD_STOP - stop_dist(v, PI_LATENCY_MAX)

    thr_a80 = threshold_latency_ms(v, DECEL_A)
    thr_a60 = threshold_latency_ms(v, DECEL_A_LO)

    if verbose:
        print(f"\n  Competition speed : {v} cm/s")
        print(f"  Deceleration      : {DECEL_A} cm/s²")
        print(f"  Braking distance  : {bd:.2f} cm")
        print(f"\n  Arduino reaction distance : {ard_rd:.2f} cm  ({ARDUINO_RT*1000:.0f} ms)")
        print(f"  Vision reaction dist mean : {vis_rd_mean:.2f} cm  ({PI_LATENCY_MEAN*1000:.1f} ms)")
        print(f"  Ratio                     : {ratio:.1f}×  (paper: 4.5×)")
        print(f"\n  Safety margin @ mean Pi latency  : {margin_mean:.2f} cm")
        print(f"  Safety margin @ worst Pi latency : {margin_max:.2f} cm")
        print(f"\n  Zero-margin threshold (a=80 cm/s²): {thr_a80:.0f} ms  (paper: ≈190 ms)")
        print(f"  Zero-margin threshold (a=60 cm/s²): {thr_a60:.0f} ms  (sensitivity check)")
        print(f"  Headroom above worst Pi latency   : {thr_a80 - PI_LATENCY_MAX*1000:.0f} ms")

    # ── Figure (paper Fig.9) ──────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("S3 · Reaction Distance Analysis: Arduino 50 Hz Safety Loop "
                 "vs Vision-Only Pipeline", fontsize=11)

    # Panel (a): reaction distance & stop distance
    ax = axes[0]
    ax.set_title(f"(a) At competition speed = {COMP_SPEED} cm/s", fontsize=9)

    ard_rd_arr  = np.full_like(latency_s, reaction_dist(COMP_SPEED, ARDUINO_RT))
    ard_sd_arr  = np.full_like(latency_s, stop_dist(COMP_SPEED, ARDUINO_RT))
    vis_rd_arr  = reaction_dist(COMP_SPEED, latency_s)
    vis_sd_arr  = stop_dist(COMP_SPEED, latency_s)

    ax.plot(latency_ms, vis_rd_arr,  "r-",  lw=1.8, label="Vision-only reaction dist")
    ax.plot(latency_ms, vis_sd_arr,  "r--", lw=1.8, label="Vision-only total stop dist")
    ax.plot(latency_ms, ard_rd_arr,  "b-",  lw=1.8, label=f"Arduino reaction dist ({ard_rd:.1f} cm, always 20 ms)")
    ax.plot(latency_ms, ard_sd_arr,  "b--", lw=1.8, label=f"Arduino total stop dist ({ard_sd:.1f} cm)")
    ax.axhline(HARD_STOP, color="darkorange", ls="-.", lw=1.5, label="Hard-stop threshold (15 cm)")
    ax.axvline(PI_LATENCY_MEAN*1000, color="green", ls=":", lw=1.5,
               label=f"Measured Pi latency ({PI_LATENCY_MEAN*1000:.1f} ms)")
    ax.set_xlabel("Vision pipeline latency (ms)")
    ax.set_ylabel("Distance (cm)")
    ax.set_xlim(0, 300); ax.set_ylim(0, 14)
    ax.legend(fontsize=7); ax.grid(alpha=0.3)

    # Panel (b): safety margin for vision-only
    ax = axes[1]
    ax.set_title("(b) Safety Margin vs Vision Latency (dashed = Arduino baseline)", fontsize=9)
    colors = ["#2ca02c", "#1f77b4", "#ff7f0e", "#d62728"]
    for spd, col in zip(SPEEDS_CMS, colors):
        margin = safety_margin(spd, latency_s)
        ax.plot(latency_ms, margin, color=col, lw=1.8, label=f"Vision-only @ {spd:.0f} cm/s")
        ard_m = HARD_STOP - stop_dist(spd, ARDUINO_RT)
        ax.axhline(ard_m, color=col, ls="--", lw=1.0, alpha=0.6)

    ax.axhline(0, color="black", lw=0.8, ls="-")
    ax.fill_between(latency_ms,
                    safety_margin(COMP_SPEED, latency_s),
                    0,
                    where=safety_margin(COMP_SPEED, latency_s) < 0,
                    color="red", alpha=0.15, label="Unsafe zone (25 cm/s)")
    ax.axvline(PI_LATENCY_MEAN*1000, color="green", ls=":", lw=1.5,
               label=f"Measured Pi latency ({PI_LATENCY_MEAN*1000:.1f} ms)")
    ax.set_xlabel("Vision pipeline latency (ms)")
    ax.set_ylabel("Safety margin (cm)  [positive = safe]")
    ax.set_xlim(0, 300)
    ax.legend(fontsize=7); ax.grid(alpha=0.3)

    plt.tight_layout()
    fig.savefig(f"{out}/S3_reaction_distance.png", dpi=150, bbox_inches="tight")
    plt.close()
    if verbose:
        print(f"\n  Figure saved: {out}/S3_reaction_distance.png")

    return dict(ratio=ratio, thr_a80=thr_a80, thr_a60=thr_a60,
                margin_mean=margin_mean, margin_max=margin_max)


if __name__ == "__main__":
    run_S3()
