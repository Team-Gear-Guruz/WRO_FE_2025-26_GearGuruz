"""
S6 – HSV Colour Detection Robustness Under Noise (paper §7.7.6)
================================================================
Simulates pixel-level HSV detection rate as a function of hue noise σH
for all four colour classes, with and without CLAHE V-channel normalisation.

Model (paper §7.7.6):
  H ~ N(μH, σH)      hue noise
  S ~ N(μS, 20)      saturation noise
  V ~ N(μV·gCLAHE, 30)  value noise; gCLAHE=1.25 models V-channel boost
  N=500 synthetic pixels per noise level

A pixel is detected if H, S, V all fall within the colour's HSV band.
For red (two-band hue wrap), either band may fire.

CLAHE benefit: lifts near-threshold V values into detection band.
Result: maintains detection rate above worst physical F1 (0.83) up to σH≈8–10.

Colours and HSV band centres (from Table 2):
  Orange : H=13, S=180, V=180
  Red    : H=6  (band 1), S=165, V=160
  Green  : H=62, S=160, V=160
  Magenta: H=152, S=165, V=165
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os

# ─── HSV band definitions (paper Table 2) ────────────────────────────────────
# (H_lo, H_hi, S_lo, S_hi, V_lo, V_hi)
COLOUR_BANDS = {
    "Orange":  [(  5, 22,  120, 255, 110, 255)],
    "Red":     [(  0, 12,   80, 255,  70, 255),
                (168, 179,  80, 255,  70, 255)],   # two-band wrap
    "Green":   [( 40, 85,   70, 255,  70, 255)],
    "Magenta": [(135, 170,  80, 255,  80, 255)],
}

# Band-centre values for synthetic pixel generation
COLOUR_CENTRES = {
    "Orange":  dict(H=13,  S=187, V=182),
    "Red":     dict(H=6,   S=167, V=162),
    "Green":   dict(H=62,  S=162, V=162),
    "Magenta": dict(H=152, S=167, V=172),
}

# Noise parameters (paper §7.7.6)
SIGMA_S    = 20.0   # saturation noise std
SIGMA_V    = 30.0   # value noise std
G_CLAHE    = 1.25   # CLAHE V-channel contrast boost factor
N_PIXELS   = 500    # pixels per noise level
SIGMA_H_RANGE = np.linspace(0, 25, 60)  # hue noise sweep
WORST_F1   = 0.83   # worst physical F1 (ambient glare, red pillar, Table 9)

# Worst physical F1 threshold
WORST_PHY_F1 = 0.83


def in_band(H, S, V, bands):
    """True if pixel falls within any of the colour's HSV bands."""
    for h_lo, h_hi, s_lo, s_hi, v_lo, v_hi in bands:
        in_h = (h_lo <= H <= h_hi)
        in_s = (s_lo <= S <= s_hi)
        in_v = (v_lo <= V <= v_hi)
        if in_h and in_s and in_v:
            return True
    return False


def detection_rate(colour, sigma_h, clahe=False, n=N_PIXELS, rng=None):
    """
    Sample N pixels with Gaussian noise and compute fraction detected.
    clahe=True multiplies V centre by G_CLAHE before noising.
    """
    if rng is None:
        rng = np.random.default_rng(42)
    c     = COLOUR_CENTRES[colour]
    bands = COLOUR_BANDS[colour]
    mu_V  = c["V"] * (G_CLAHE if clahe else 1.0)

    H = rng.normal(c["H"], sigma_h, n)
    S = rng.normal(c["S"], SIGMA_S,  n)
    V = rng.normal(mu_V,  SIGMA_V,  n)

    # Clip to valid OpenCV HSV ranges
    H = np.clip(H, 0, 179)
    S = np.clip(S, 0, 255)
    V = np.clip(V, 0, 255)

    detected = sum(in_band(h, s, v, bands) for h, s, v in zip(H, S, V))
    return detected / n


def run_S6(out="outputs", verbose=True):
    os.makedirs(out, exist_ok=True)

    if verbose:
        print("\n" + "="*60)
        print("S6 – HSV Colour Detection Robustness Under Hue Noise")
        print(f"    N={N_PIXELS} pixels per noise level, gCLAHE={G_CLAHE}")
        print("="*60)

    rng = np.random.default_rng(42)
    colours = list(COLOUR_BANDS.keys())

    # Compute detection rates
    rates = {}
    for col in colours:
        rates[col] = {
            "clahe":    [detection_rate(col, sh, clahe=True,  rng=rng) for sh in SIGMA_H_RANGE],
            "no_clahe": [detection_rate(col, sh, clahe=False, rng=rng) for sh in SIGMA_H_RANGE],
        }
        if verbose:
            # Find σH where CLAHE rate drops below WORST_F1
            for i, (sh, r) in enumerate(zip(SIGMA_H_RANGE, rates[col]["clahe"])):
                if r < WORST_PHY_F1:
                    print(f"  {col:8s}: CLAHE rate drops below F1={WORST_PHY_F1} at σH≈{sh:.1f}")
                    break
            else:
                print(f"  {col:8s}: CLAHE rate stays above F1={WORST_PHY_F1} across full range")

    # ── Figure (paper Fig.12) ─────────────────────────────────────────────────
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    fig.suptitle("S6 · HSV Detection Rate under Gaussian Hue Noise\n"
                 f"(σH ∈ [0,25], N={N_PIXELS} pixels per point)", fontsize=11)

    for ax, col in zip(axes.flat, colours):
        r_clahe = rates[col]["clahe"]
        r_none  = rates[col]["no_clahe"]
        benefit = [max(0, c-n) for c, n in zip(r_clahe, r_none)]

        ax.fill_between(SIGMA_H_RANGE, r_none, r_clahe,
                        alpha=0.25, color="dodgerblue", label="CLAHE benefit")
        ax.plot(SIGMA_H_RANGE, r_clahe, "b-",  lw=2.0, label="With CLAHE")
        ax.plot(SIGMA_H_RANGE, r_none,  "r-",  lw=2.0, label="Without CLAHE")
        ax.axhline(WORST_PHY_F1, color="darkorange", ls=":",  lw=1.8,
                   label=f"Worst physical F1 ({WORST_PHY_F1})")
        ax.set_xlabel("Hue noise σH")
        ax.set_ylabel("Detection rate")
        ax.set_title(col, fontsize=10)
        ax.set_xlim(0, 25); ax.set_ylim(0, 1.05)
        ax.legend(fontsize=7); ax.grid(alpha=0.3)

    plt.tight_layout()
    fig.savefig(f"{out}/S6_hsv_robustness.png", dpi=150, bbox_inches="tight")
    plt.close()

    if verbose:
        print(f"\n  Figure saved: {out}/S6_hsv_robustness.png")

    return rates


if __name__ == "__main__":
    run_S6()
