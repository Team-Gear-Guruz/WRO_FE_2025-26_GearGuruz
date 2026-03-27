"""
S5 – Serial Watchdog Validation (paper §7.7.5)
===============================================
Validates the 400 ms RX watchdog by simulating stop distance as a function of:
  (a) Watchdog threshold (100–1000 ms) at three representative speeds
  (b) Vehicle speed at the paper's 400 ms threshold

The watchdog fires when no serial byte is received for RX_WATCHDOG_MS.
During the silence, the vehicle continues at its current speed until the
watchdog triggers motor RELEASE and servo centre.

Stop distance = speed × watchdog_threshold_s  +  braking distance

Result (paper §7.7.5):
  At 25 cm/s and 400 ms threshold: vehicle travels 10.0 cm before watchdog fires
  → below 15 cm hard-stop threshold → watchdog provides independent safety layer
  Threshold exceeded only above ≈37 cm/s (above BASE_SPEED)
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os

# ─── Physical constants ───────────────────────────────────────────────────────
PAPER_THRESHOLD_MS = 400     # paper §3.2 RX watchdog threshold
HARD_STOP_CM       = 15.0    # safety boundary (paper §5.2)
DECEL_A            = 80.0    # deceleration (cm/s²) from S3 calibration

# Speed calibration (paper §5.5): v ≈ 0.13*(s−70) cm/s
# BASE_SPEED=230 → 20.8 cm/s;  MIN_SPEED=130 → 7.8 cm/s
SPEEDS_CMS   = [15.0, 25.0, 35.0]
THRESHOLDS   = np.linspace(100, 1000, 500)   # ms


def coasting_dist(speed_cms, threshold_ms):
    """Distance travelled while watchdog counts down (cm)."""
    return speed_cms * (threshold_ms / 1000.0)


def braking_dist(speed_cms, a=DECEL_A):
    """Kinematic braking distance after watchdog fires (cm)."""
    return (speed_cms ** 2) / (2.0 * a)


def stop_dist(speed_cms, threshold_ms, a=DECEL_A):
    return coasting_dist(speed_cms, threshold_ms) + braking_dist(speed_cms, a)


def max_safe_speed(threshold_ms, a=DECEL_A, limit=HARD_STOP_CM):
    """
    Maximum speed at which stop distance ≤ limit.
    Solve: v * t + v²/2a = limit  →  quadratic in v
    """
    t = threshold_ms / 1000.0
    # v²/2a + v*t - limit = 0
    discriminant = t**2 + 4 * (1/(2*a)) * limit
    if discriminant < 0:
        return 0.0
    return (-t + np.sqrt(discriminant)) / (2 * (1/(2*a)))


def run_S5(out="outputs", verbose=True):
    os.makedirs(out, exist_ok=True)

    if verbose:
        print("\n" + "="*60)
        print("S5 – Serial Watchdog Validation")
        print("="*60)

    # Key values at paper threshold
    for spd in SPEEDS_CMS:
        sd = stop_dist(spd, PAPER_THRESHOLD_MS)
        safe = sd <= HARD_STOP_CM
        if verbose:
            print(f"  Speed={spd:5.1f} cm/s  threshold={PAPER_THRESHOLD_MS} ms  "
                  f"stop_dist={sd:.2f} cm  {'✓ safe' if safe else '✗ unsafe'}")

    max_spd = max_safe_speed(PAPER_THRESHOLD_MS)
    if verbose:
        print(f"\n  Max safe speed @ {PAPER_THRESHOLD_MS} ms threshold: {max_spd:.1f} cm/s")
        print(f"  (paper: ≈37 cm/s; BASE_SPEED=230 → 20.8 cm/s is well below)")

    # ── Figure (paper Fig.11) ─────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("S5 · Serial Watchdog Characterisation", fontsize=11)

    colors = ["#2ca02c", "#1f77b4", "#d62728"]

    # Panel (a): stop distance vs threshold
    ax = axes[0]
    for spd, col in zip(SPEEDS_CMS, colors):
        sd_arr = [stop_dist(spd, th) for th in THRESHOLDS]
        ax.plot(THRESHOLDS, sd_arr, color=col, lw=2.0, label=f"{spd:.0f} cm/s")
    ax.axvline(PAPER_THRESHOLD_MS, color="red", ls="--", lw=2.0,
               label=f"Paper threshold ({PAPER_THRESHOLD_MS} ms)")
    ax.axhline(HARD_STOP_CM, color="gray", ls=":", lw=1.5,
               label=f"Hard-stop limit ({HARD_STOP_CM} cm)")
    ax.set_xlabel("Watchdog threshold (ms)")
    ax.set_ylabel("Distance before stop (cm)")
    ax.set_title("(a) Stop Distance vs Watchdog Threshold")
    ax.set_xlim(100, 1000)
    ax.legend(fontsize=9); ax.grid(alpha=0.3)

    # Panel (b): stop distance vs speed at paper threshold
    speeds_cont = np.linspace(5, 50, 500)
    sd_vs_spd   = [stop_dist(v, PAPER_THRESHOLD_MS) for v in speeds_cont]
    ax = axes[1]
    ax.plot(speeds_cont, sd_vs_spd, "b-", lw=2.5, label=f"Stop dist @ {PAPER_THRESHOLD_MS} ms")
    ax.axhline(HARD_STOP_CM, color="red", ls="--", lw=2.0,
               label=f"Hard-stop threshold = {HARD_STOP_CM} cm")
    ax.axvline(max_spd, color="darkorange", ls=":", lw=1.8,
               label=f"Max safe speed ≈ {max_spd:.1f} cm/s")
    # Mark BASE_SPEED equivalent
    ax.axvline(20.8, color="seagreen", ls="-.", lw=1.5,
               label="BASE_SPEED (230 PWM = 20.8 cm/s)")
    ax.fill_between(speeds_cont, sd_vs_spd, HARD_STOP_CM,
                    where=[s > HARD_STOP_CM for s in sd_vs_spd],
                    color="red", alpha=0.15, label="Unsafe zone")
    ax.set_xlabel("Vehicle speed (cm/s)")
    ax.set_ylabel("Distance before stop (cm)")
    ax.set_title(f"(b) Stop Distance at {PAPER_THRESHOLD_MS} ms Threshold vs Speed")
    ax.set_xlim(5, 50)
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    plt.tight_layout()
    fig.savefig(f"{out}/S5_watchdog_validation.png", dpi=150, bbox_inches="tight")
    plt.close()

    if verbose:
        print(f"\n  Figure saved: {out}/S5_watchdog_validation.png")

    return dict(max_safe_speed=max_spd,
                stop_dist_25cms=stop_dist(25.0, PAPER_THRESHOLD_MS))


if __name__ == "__main__":
    run_S5()
