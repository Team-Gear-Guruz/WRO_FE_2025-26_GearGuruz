#!/usr/bin/env python3
"""
run_all_simulations.py
======================
Runs all seven simulation studies from the paper in sequence.
Outputs all figures to the ./outputs/ directory.

Usage:
  python run_all_simulations.py            # full run (S1: N=500, S2: 81 configs)
  python run_all_simulations.py --quick    # fast run for testing (S1: N=50, S2: 3×3)

Simulations:
  S1  Monte Carlo Parking FSM        (§7.7.1) – kinematic, N=500
  S2  Potential-Field Gain Sweep     (§7.7.2) – 9×9 grid, 3 reps
  S3  Reaction Distance Analysis     (§7.7.3) – analytical
  S4  Sensor Dropout Degradation     (§7.7.4) – 12 configs, 3000 steps each
  S5  Serial Watchdog Validation     (§7.7.5) – analytical
  S6  HSV Colour Detection Robustness(§7.7.6) – 500 pixels per noise level
  S7  Speed Regulation Policy        (§7.7.7) – analytical
"""

import sys, time, os, random
import numpy as np

OUTPUT_DIR = "outputs"
os.makedirs(OUTPUT_DIR, exist_ok=True)

QUICK = "--quick" in sys.argv

if QUICK:
    print("=== QUICK MODE: reduced sample sizes ===\n")

random.seed(42)
np.random.seed(42)

results = {}
t_start = time.time()

# ─── S1 ───────────────────────────────────────────────────────────────────────
print("\n" + "▓"*60)
from S1_S2_simulator import run_S1, run_S2
results["S1"] = run_S1(n=50 if QUICK else 500)

# ─── S2 ───────────────────────────────────────────────────────────────────────
print("\n" + "▓"*60)
import numpy as _np
if QUICK:
    k_o_v  = _np.logspace(-4, -1.5, 3)
    k_fy_v = _np.linspace(0, 0.6, 3)
    results["S2"] = run_S2(k_o_vals=k_o_v, k_fy_vals=k_fy_v, reps=1)
else:
    results["S2"] = run_S2()

# Make S1+S2 figures
from S1_S2_simulator import make_figures as make_s1s2
make_s1s2(results["S1"], results["S2"], out=OUTPUT_DIR)

# ─── S3 ───────────────────────────────────────────────────────────────────────
print("\n" + "▓"*60)
from S3_reaction_distance import run_S3
results["S3"] = run_S3(out=OUTPUT_DIR)

# ─── S4 ───────────────────────────────────────────────────────────────────────
print("\n" + "▓"*60)
from S4_sensor_dropout import run_S4
results["S4"] = run_S4(out=OUTPUT_DIR)

# ─── S5 ───────────────────────────────────────────────────────────────────────
print("\n" + "▓"*60)
from S5_watchdog_validation import run_S5
results["S5"] = run_S5(out=OUTPUT_DIR)

# ─── S6 ───────────────────────────────────────────────────────────────────────
print("\n" + "▓"*60)
from S6_hsv_robustness import run_S6
results["S6"] = run_S6(out=OUTPUT_DIR)

# ─── S7 ───────────────────────────────────────────────────────────────────────
print("\n" + "▓"*60)
from S7_speed_regulation import run_S7
results["S7"] = run_S7(out=OUTPUT_DIR)

# ─── Summary ──────────────────────────────────────────────────────────────────
elapsed = time.time() - t_start
print("\n" + "="*60)
print("SIMULATION SUITE COMPLETE")
print("="*60)
print(f"  Total runtime : {elapsed:.1f}s")
print(f"  Output dir    : {os.path.abspath(OUTPUT_DIR)}/")
print()
print("  S1 Parking FSM:")
print(f"     Simulated sub-sequence success : {results['S1']['rate']*100:.1f}%")
print(f"     95% CI                         : [{results['S1']['ci_lo']*100:.1f}%, {results['S1']['ci_hi']*100:.1f}%]")
print(f"     Physical full-FSM result       : 84% (N=50, CI [71%,93%])")

s2c = results["S2"]["compl"]
s2ko = results["S2"]["k_o"]
s2kfy = results["S2"]["k_fy"]
pi_ = _np.argmin(_np.abs(s2ko  - 0.008))
pj_ = _np.argmin(_np.abs(s2kfy - 0.0025))
print(f"\n  S2 Gain Sweep:")
print(f"     Paper gains completion  : {s2c[pi_,pj_]*100:.0f}%")
print(f"     Stable configs (≥67%)   : {_np.sum(s2c>=0.67)}/{s2c.size}")

print(f"\n  S3 Reaction Distance:")
print(f"     Arduino/vision ratio    : {results['S3']['ratio']:.1f}× (paper: 4.5×)")
print(f"     Zero-margin threshold   : {results['S3']['thr_a80']:.0f} ms (paper: ≈190 ms)")

print(f"\n  S5 Watchdog:")
print(f"     Stop dist @ 25 cm/s     : {results['S5']['stop_dist_25cms']:.1f} cm (paper: 10.0 cm)")
print(f"     Max safe speed          : {results['S5']['max_safe_speed']:.1f} cm/s (paper: ≈37 cm/s)")

print(f"\n  S7 Speed Regulation:")
print(f"     Max stop dist           : {results['S7']['max_stop']:.2f} cm < 15 cm: "
      f"{'✓' if results['S7']['all_safe'] else '✗'}")

print("\n  Figures generated:")
for f in sorted(os.listdir(OUTPUT_DIR)):
    if f.endswith(".png"):
        print(f"    {f}")
