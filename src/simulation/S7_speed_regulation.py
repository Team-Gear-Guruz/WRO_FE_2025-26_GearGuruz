"""
S7 – Speed Regulation Policy (paper §7.7.7)
============================================
Evaluates the mapToSpeed() function (paper Eq.14) analytically across the
full front-centre clearance range [0, 120] cm.

Speed law (Eq.14):
  s = smin              if dFC ≤ cslow
  s = smax              if dFC ≥ cfast
  s = smin + (smax-smin)*(dFC-cslow)/(cfast-cslow)  otherwise

PWM-to-velocity calibration (paper §5.5):
  v ≈ 0.13*(s − 70) cm/s  (R²=0.98)
  smin=130 → 7.8 cm/s
  smax=230 → 20.8 cm/s

Stopping distance: v²/2a  [conservative a=50 cm/s²]
Safety check: stopping distance must remain below 15 cm hard-stop threshold.

Result (paper §7.7.7):
  At smax → v=20.8 cm/s: stopping distance = 20.8²/(2×50) ≈ 4.3 cm < 15 cm
  Stopping distance stays below 4.3 cm across entire clearance range.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os

# ─── Speed policy parameters (paper Eq.14, §5.5) ─────────────────────────────
S_MIN   = 130    # minimum PWM (7.8 cm/s)
S_MAX   = 230    # maximum PWM (20.8 cm/s)
C_SLOW  = 20     # cslow: clearance at which minimum speed applies (cm)
C_FAST  = 90     # cfast: clearance at which maximum speed applies (cm)

# PWM-to-velocity: v = 0.13*(s-70) cm/s (calibrated, paper §5.5)
def pwm_to_cms(s):
    return 0.13 * (s - 70.0)

V_MIN = pwm_to_cms(S_MIN)   # ≈ 7.8 cm/s
V_MAX = pwm_to_cms(S_MAX)   # ≈ 20.8 cm/s

HARD_STOP_CM = 15.0   # safety boundary
DECEL_A      = 50.0   # conservative lower bound deceleration (paper §7.7.7)
                       # (S3 uses 80 cm/s² for safety margin; S7 uses 50 cm/s² as
                       #  conservative lower bound to give a more demanding test)


def map_to_speed_pwm(dfc_cm):
    """mapToSpeed() function – returns PWM value (paper Eq.14)."""
    if dfc_cm <= C_SLOW:
        return S_MIN
    if dfc_cm >= C_FAST:
        return S_MAX
    t = (dfc_cm - C_SLOW) / (C_FAST - C_SLOW)
    return int(S_MIN + t * (S_MAX - S_MIN))


def map_to_speed_cms(dfc_cm):
    return pwm_to_cms(map_to_speed_pwm(dfc_cm))


def stop_dist_cm(dfc_cm, a=DECEL_A):
    v = map_to_speed_cms(dfc_cm)
    return (v ** 2) / (2.0 * a)


def run_S7(out="outputs", verbose=True):
    os.makedirs(out, exist_ok=True)

    if verbose:
        print("\n" + "="*60)
        print("S7 – Speed Regulation Policy")
        print("="*60)
        print(f"  S_MIN={S_MIN} PWM → {V_MIN:.2f} cm/s")
        print(f"  S_MAX={S_MAX} PWM → {V_MAX:.2f} cm/s")
        print(f"  Deceleration (conservative): {DECEL_A} cm/s²")

    clearance = np.linspace(0, 120, 500)
    spd_pwm   = np.array([map_to_speed_pwm(d) for d in clearance])
    spd_cms   = np.array([map_to_speed_cms(d) for d in clearance])
    stop_cm   = np.array([stop_dist_cm(d)     for d in clearance])

    max_stop = np.max(stop_cm)
    if verbose:
        print(f"\n  Max stopping distance (@ smax={S_MAX} PWM): {max_stop:.2f} cm")
        print(f"  Hard-stop threshold                       : {HARD_STOP_CM} cm")
        print(f"  Safety margin                             : {HARD_STOP_CM - max_stop:.2f} cm")
        print(f"  Stopping distance < hard-stop across all clearances: "
              f"{'✓ YES' if all(stop_cm < HARD_STOP_CM) else '✗ NO'}")

    # ── Figure (paper Fig.13) ─────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("S7 · Speed Regulation Policy", fontsize=11)

    # Panel (a): mapToSpeed profile
    ax = axes[0]
    ax.plot(clearance, spd_pwm, "b-", lw=2.5, label="mapToSpeed() (PWM)")
    ax2 = ax.twinx()
    ax2.plot(clearance, spd_cms, "b--", lw=1.5, alpha=0.6, label="Speed (cm/s)")
    ax.axvline(C_SLOW, color="red",   ls="--", lw=1.5,
               label=f"C_slow = {C_SLOW} cm")
    ax.axvline(C_FAST, color="green", ls="--", lw=1.5,
               label=f"C_fast = {C_FAST} cm")
    ax.axhline(S_MIN, color="gray", ls=":", lw=1.2,
               label=f"S_min = {S_MIN}")
    ax.axhline(S_MAX, color="gray", ls=":",  lw=1.2,
               label=f"S_max = {S_MAX}")
    ax.set_xlabel("Front-centre clearance (cm)")
    ax.set_ylabel("Motor speed (PWM units, 0–255)")
    ax2.set_ylabel("Speed (cm/s)", color="blue")
    ax.set_title("(a) mapToSpeed() Profile")
    ax.set_xlim(0, 120)
    lines1, labels1 = ax.get_legend_handles_labels()
    ax.legend(lines1, labels1, fontsize=8)
    ax.grid(alpha=0.3)

    # Panel (b): safety envelope
    ax = axes[1]
    ax.plot(clearance, stop_cm, "b-", lw=2.5, label=f"Stopping distance (v²/2a, a={DECEL_A} cm/s²)")
    ax.axhline(HARD_STOP_CM, color="red", ls="--", lw=2.0,
               label=f"Hard-stop threshold = {HARD_STOP_CM} cm")
    ax.fill_between(clearance, stop_cm, HARD_STOP_CM,
                    where=(stop_cm > HARD_STOP_CM),
                    color="red", alpha=0.2, label="Unsafe zone")
    ax.fill_between(clearance, 0, stop_cm,
                    where=(stop_cm <= HARD_STOP_CM),
                    color="green", alpha=0.08, label="Safe zone")
    ax.set_xlabel("Front-centre clearance (cm)")
    ax.set_ylabel("Stopping distance (cm)")
    ax.set_title("(b) Safety Envelope")
    ax.set_xlim(0, 120)
    ax.set_ylim(0, max(HARD_STOP_CM * 1.2, max_stop * 1.1))
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    plt.tight_layout()
    fig.savefig(f"{out}/S7_speed_regulation.png", dpi=150, bbox_inches="tight")
    plt.close()

    if verbose:
        print(f"\n  Figure saved: {out}/S7_speed_regulation.png")

    return dict(max_stop=max_stop, v_min=V_MIN, v_max=V_MAX,
                all_safe=bool(np.all(stop_cm < HARD_STOP_CM)))


if __name__ == "__main__":
    run_S7()
