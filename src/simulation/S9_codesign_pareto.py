"""
S9 – Vision-Actuation Co-Design: Single-Chip vs Dual-Processor Safety Analysis
===============================================================================
Novel contribution: frames the dual-processor architecture as a necessary
response to a fundamental BLOCKING CONSTRAINT — not a performance choice.

CORE INSIGHT
────────────
On a single SBC, HC-SR04 sensor reads are BLOCKING: pulseIn() holds the CPU
for up to 20ms per sensor × 6 sensors × 3 median measurements = up to 90ms.
This is a hardware-level mutual exclusion: while the CPU is in pulseIn(),
it cannot process camera frames, run control loops, or handle serial I/O.

Consequence: single-chip f_act = 1 / (T_vision + T_sensor_read)
           = 1 / (90.9ms + 90ms) = 5.5 Hz

vs. Arduino dedicated: f_act = 50 Hz (hardware independent)

This produces a 9× actuator rate degradation that is STRUCTURAL — it cannot
be eliminated by faster SBCs because the sensor read time is dominated by
acoustic propagation (5ms × 6 sensors is fixed by the speed of sound).

EXPERIMENTS
────────────
E1: Safety margin comparison across platform taxonomy at each platform's v_max
E2: Blocking constraint surface: f_act_single(T_vis, n_sensors, n_median)
E3: Speed envelope: maximum safe operating speed for each architecture
E4: Single-chip f_act degradation as sensors are added (1→6 sensors)
E5: The "sensor scaling trap": adding sensors improves robustness but
    reduces single-chip actuator rate, making dual-proc MORE necessary
"""

from __future__ import annotations
import os, time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

D_SAFE  = 15.0   # cm
A_DECEL = 80.0   # cm/s²
F_ARD   = 50.0   # Hz  (Arduino dedicated)

# Physical sensor timing
T_PULSE_PER_READ_MS = 5.0    # ms per single HC-SR04 read (acoustic round trip)
N_MEDIAN            = 3      # median-of-three filter
T_SENSOR_TOTAL_MS   = 6 * N_MEDIAN * T_PULSE_PER_READ_MS  # 90ms blocking

# Platform taxonomy
PLATFORMS = [
    dict(name="Pi 3B+\n(this paper)", tau_ms=90.9,  B_rel=1.0,  color="#d62728", marker="*",  v_max=25),
    dict(name="Pi 4B",                tau_ms=45.0,  B_rel=3.5,  color="#ff7f0e", marker="s",  v_max=42),
    dict(name="Pi 5",                 tau_ms=28.0,  B_rel=8.0,  color="#9467bd", marker="D",  v_max=48),
    dict(name="Jetson Nano",          tau_ms=22.0,  B_rel=12.0, color="#8c564b", marker="P",  v_max=50),
]

def f_act_single(tau_ms, n_sensors=6, n_median=3):
    """Single-chip actuator rate limited by blocking sensor reads."""
    t_block = n_sensors * n_median * T_PULSE_PER_READ_MS  # ms
    return 1000.0 / (tau_ms + t_block)  # Hz

def safety_margin(v, f_act, a=A_DECEL, d=D_SAFE):
    return d - v/f_act - v**2/(2*a)

def max_safe_speed(f_act, a=A_DECEL, d=D_SAFE):
    """Solve: d - v/f - v²/2a = 0 for v."""
    # v²/2a + v/f - d = 0
    coeffs = [1/(2*a), 1/f_act, -d]
    roots = np.roots(coeffs)
    pos = [r for r in roots if r.real > 0 and abs(r.imag) < 1e-9]
    return float(pos[0].real) if pos else 0.0

def run_S9(out="outputs", verbose=True):
    os.makedirs(out, exist_ok=True)
    results = {}

    if verbose:
        print("\n" + "="*65)
        print("S9 – Vision-Actuation Co-Design: Blocking Constraint Analysis")
        print("="*65)

    # E1: Safety margin table
    if verbose:
        print(f"\n  E1: Safety margin at each platform's operating speed")
        print(f"  {'Platform':18s} {'v_max':>6s} {'f_single':>10s} {'m_single':>10s} {'m_dual':>10s} {'Δ':>8s}")
    e1 = []
    for p in PLATFORMS:
        fs = f_act_single(p['tau_ms'])
        ms = safety_margin(p['v_max'], fs)
        md = safety_margin(p['v_max'], F_ARD)
        e1.append(dict(name=p['name'].replace('\n',' '), v=p['v_max'],
                       fs=fs, ms=ms, md=md, delta=md-ms))
        if verbose:
            print(f"  {p['name'].replace(chr(10),' '):18s} {p['v_max']:>6.0f} "
                  f"{fs:>10.1f} {ms:>+10.2f} {md:>+10.2f} {md-ms:>+8.2f}")
    results['e1'] = e1

    # E2: Blocking surface
    tau_range = np.linspace(5, 200, 80)
    n_s_range = np.arange(1, 10)
    TAU2, NS2 = np.meshgrid(tau_range, n_s_range)
    F_SURF    = np.vectorize(lambda t, n: f_act_single(t, n_sensors=n))(TAU2, NS2)
    results['e2'] = dict(TAU=TAU2, NS=NS2, F=F_SURF)

    # E3: Max safe speed envelope
    v_max_single = [max_safe_speed(f_act_single(p['tau_ms'])) for p in PLATFORMS]
    v_max_dual   = [max_safe_speed(F_ARD)] * len(PLATFORMS)
    results['e3'] = dict(v_single=v_max_single, v_dual=v_max_dual)
    if verbose:
        print(f"\n  E3: Maximum safe operating speed")
        for p, vs, vd in zip(PLATFORMS, v_max_single, v_max_dual):
            name = p['name'].replace('\n', ' ')
            op_speed = p['v_max']
            safe_s = 'SAFE' if op_speed <= vs else 'UNSAFE'
            safe_d = 'SAFE' if op_speed <= vd else 'UNSAFE'
            print(f"  {name:18s}: single={vs:.1f}cm/s({safe_s})  dual={vd:.1f}cm/s({safe_d})")

    # E4 & E5: Sensor scaling trap
    n_sensors_range = np.arange(1, 9)
    v_test = 25.0
    tau_test = 90.9
    f_acts   = [f_act_single(tau_test, n) for n in n_sensors_range]
    margins  = [safety_margin(v_test, f) for f in f_acts]
    results['e5'] = dict(n=n_sensors_range, f=f_acts, margins=margins)
    if verbose:
        print(f"\n  E5: Sensor scaling trap at v={v_test}cm/s, τ={tau_test}ms")
        for n, f, m in zip(n_sensors_range, f_acts, margins):
            print(f"  {n} sensors: f_act={f:.1f}Hz margin={m:+.2f}cm")

    _make_figures_S9(results, PLATFORMS, out, verbose)
    return results

def _make_figures_S9(res, PLATFORMS, out, verbose):
    fig, axes = plt.subplots(2, 2, figsize=(13, 11))
    fig.suptitle("S9 · Vision-Actuation Co-Design: Blocking Constraint Analysis\n"
                 "Single-chip f_act = 1/(τ_vision + τ_sensor_block)  vs  Arduino 50 Hz",
                 fontsize=11)

    # (a) Safety margin bar chart
    ax = axes[0, 0]
    e1 = res['e1']
    names  = [d['name'] for d in e1]
    ms_arr = [d['ms']   for d in e1]
    md_arr = [d['md']   for d in e1]
    colors = [p['color'] for p in PLATFORMS]
    x = np.arange(len(names)); w = 0.35
    b1 = ax.bar(x-w/2, ms_arr, w, color=colors, alpha=0.5, label='Single-chip')
    b2 = ax.bar(x+w/2, md_arr, w, color=colors, alpha=0.95, label='Dual-processor')
    for bar, val in zip(b1, ms_arr):
        ax.text(bar.get_x()+bar.get_width()/2, val+0.2 if val>0 else val-0.6,
                f'{val:+.1f}', ha='center', fontsize=8)
    for bar, val in zip(b2, md_arr):
        ax.text(bar.get_x()+bar.get_width()/2, val+0.2,
                f'{val:+.1f}', ha='center', fontsize=8, fontweight='bold')
    ax.axhline(0, color='black', lw=1.2)
    ax.fill_between([-0.5, len(names)-0.5], [0,0], [-15,-15],
                    alpha=0.07, color='red', label='Unsafe zone (margin<0)')
    ax.set_xticks(x); ax.set_xticklabels(names, fontsize=8)
    ax.set_ylabel('Safety margin (cm)')
    ax.set_title('(a) Safety margin at each platform\'s v_max\n'
                 'Single-chip goes negative; dual-proc stays positive longer', fontsize=9)
    ax.legend(fontsize=8); ax.grid(axis='y', alpha=0.25)

    # (b) Blocking surface: f_act_single(τ, n_sensors)
    ax = axes[0, 1]
    e2 = res['e2']
    cf = ax.contourf(e2['TAU'], e2['NS'], e2['F'],
                     levels=[1,2,3,5,8,10,15,20,30,50], cmap='RdYlGn', extend='both')
    cs = ax.contour(e2['TAU'], e2['NS'], e2['F'],
                    levels=[5, 10, 20, 50], colors='white', linewidths=0.8)
    ax.clabel(cs, fmt='%.0f Hz', fontsize=7)
    plt.colorbar(cf, ax=ax, label='f_act single-chip (Hz)')
    for p in PLATFORMS:
        ax.scatter(p['tau_ms'], 6, marker=p['marker'], color=p['color'],
                   s=160, zorder=5, edgecolors='black', linewidths=1.5)
        ax.annotate(p['name'].replace('\n',' '), xy=(p['tau_ms'], 6),
                    xytext=(p['tau_ms']+2, 6.3), fontsize=7, color=p['color'])
    ax.axhline(6, color='navy', lw=1.5, ls='--', alpha=0.7, label='Paper: 6 sensors')
    ax.axhline(50/1000*1000, color='none')  # dummy
    ax.set_xlabel('Vision pipeline latency τ (ms)')
    ax.set_ylabel('Number of HC-SR04 sensors')
    ax.set_title('(b) Single-chip f_act limited by sensor blocking\n'
                 f'τ_block = n × {N_MEDIAN} × {T_PULSE_PER_READ_MS:.0f}ms (fixed by acoustics)', fontsize=9)
    ax.legend(fontsize=8)

    # (c) Max safe speed envelope
    ax = axes[1, 0]
    tau_arr = np.linspace(5, 200, 300)
    for n_s, col, ls in [(2, 'blue', ':'), (4, 'orange', '--'), (6, 'red', '-')]:
        f_arr = [f_act_single(t, n_s) for t in tau_arr]
        v_arr = [max_safe_speed(f) for f in f_arr]
        ax.plot(tau_arr, v_arr, color=col, ls=ls, lw=2,
                label=f'{n_s} sensors (single-chip)')
    ax.axhline(max_safe_speed(F_ARD), color='navy', lw=2.5, ls='-',
               label=f'Dual-processor (50 Hz) → {max_safe_speed(F_ARD):.0f} cm/s')
    for p in PLATFORMS:
        ax.scatter(p['tau_ms'], p['v_max'], marker=p['marker'],
                   color=p['color'], s=180, zorder=5, edgecolors='black')
        ax.annotate(p['name'].replace('\n',' '), xy=(p['tau_ms'], p['v_max']),
                    xytext=(p['tau_ms']+3, p['v_max']+0.5),
                    fontsize=7, color=p['color'])
    ax.set_xlabel('Vision latency τ (ms)')
    ax.set_ylabel('Maximum safe operating speed (cm/s)')
    ax.set_title('(c) Speed envelope: max safe v for each architecture\n'
                 'Points above curve → UNSAFE operating condition', fontsize=9)
    ax.set_xlim(5, 200); ax.set_ylim(0, 60)
    ax.legend(fontsize=7); ax.grid(alpha=0.25)

    # (d) Sensor scaling trap
    ax = axes[1, 1]
    e5 = res['e5']
    color = plt.cm.RdYlGn(np.linspace(0.1, 0.9, len(e5['n'])))
    bars = ax.bar(e5['n'], e5['f'], color=[plt.cm.RdYlGn(0.8 if m > 0 else 0.15)
                                            for m in e5['margins']])
    ax.axhline(F_ARD, color='navy', lw=2, ls='--',
               label=f'Arduino dedicated: {F_ARD:.0f} Hz')
    ax2 = ax.twinx()
    ax2.plot(e5['n'], e5['margins'], 'k-o', lw=2, ms=7, label='Safety margin (cm)')
    ax2.axhline(0, color='red', lw=1, ls='--', alpha=0.7)
    ax2.set_ylabel('Safety margin (cm)', color='black')
    ax.set_xlabel('Number of HC-SR04 sensors')
    ax.set_ylabel('Single-chip f_act (Hz)')
    ax.set_title('(d) The sensor scaling trap\n'
                 'More sensors → better robustness BUT lower f_act → lower margin', fontsize=9)
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1+lines2, labels1+labels2, fontsize=8)
    ax.set_xticks(e5['n']); ax.grid(axis='y', alpha=0.25)

    plt.tight_layout()
    p1 = f"{out}/S9_codesign.png"
    fig.savefig(p1, dpi=150, bbox_inches='tight')
    plt.close()
    if verbose: print(f"\n  Figure saved: {p1}")
    return p1

if __name__ == "__main__":
    t0 = time.time()
    run_S9(out="outputs", verbose=True)
    print(f"\nS9 total runtime: {time.time()-t0:.1f}s")
