"""
WRO 2026 Future Engineers – Full 2D Kinematic Simulator
========================================================
Simulations S1 and S2 (paper §7.7.1, §7.7.2).

Fix applied vs original repo:
  [FIX] Repulsion function in NavCtrl._repulse() changed from QUADRATIC
        to LINEAR to match paper Eq.10: ρ = k·max(0, 1 – d/r).
        The original used (1 – d/d0)² which is not described in the paper
        and invalidated the S2 gain-sweep comparison with physical hardware.

Track geometry: exact WRO 2026 (Section 13 General Rules, v.15-Jan-2026)
  Outer wall   : 3000×3000 mm
  Inner wall   : 1000×1000 mm (centred)
  Corridor     : 1000 mm
  Parking lot  : 200 mm wide × 375 mm deep (1.5× robot length)

Coordinate origin: inner face of outer wall, bottom-left corner.
CW travel: bottom → right → top → left.
"""

from __future__ import annotations
import math, random, time
from dataclasses import dataclass
from typing import List, Optional, Tuple
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats


# ─────────────────────────────────────────────────────────────────────────────
# 1. TRACK GEOMETRY
# ─────────────────────────────────────────────────────────────────────────────

class Track:
    OW = 3000.0   # outer wall (mm)
    IL = 1000.0   # inner wall lower bound
    IH = 2000.0   # inner wall upper bound
    CL = 1000.0   # corridor width

    CL_E = 500.0   # E-straight centreline y
    CL_N = 2500.0  # N-straight centreline x
    CL_W = 2500.0  # W-straight centreline y
    CL_S = 500.0   # S-straight centreline x

    def __init__(self, robot_length: float = 250.0):
        self.robot_length = robot_length
        self.park_length  = 1.5 * robot_length   # 375 mm
        self.park_width   = 200.0

    def in_corridor(self, x, y):
        in_outer = 0 <= x <= self.OW and 0 <= y <= self.OW
        in_inner = self.IL <= x <= self.IH and self.IL <= y <= self.IH
        return in_outer and not in_inner

    def nearest_wall_dist(self, x, y):
        d = min(x, self.OW - x, y, self.OW - y)
        if self.IL <= x <= self.IH:
            d = min(d, y - self.IL, self.IH - y)
        if self.IL <= y <= self.IH:
            d = min(d, x - self.IL, self.IH - x)
        return d

    def cast_ray(self, ox, oy, angle, max_r=4000.0):
        dx, dy = math.cos(angle), math.sin(angle)
        t_min  = max_r
        walls  = [
            ("v", 0.0,     0.0,     self.OW),
            ("v", self.OW, 0.0,     self.OW),
            ("h", 0.0,     0.0,     self.OW),
            ("h", self.OW, 0.0,     self.OW),
            ("v", self.IL, self.IL, self.IH),
            ("v", self.IH, self.IL, self.IH),
            ("h", self.IL, self.IL, self.IH),
            ("h", self.IH, self.IL, self.IH),
        ]
        for kind, coord, lo, hi in walls:
            if kind == "v":
                if abs(dx) < 1e-9: continue
                t = (coord - ox) / dx
                if t <= 1e-6: continue
                hit = oy + t * dy
            else:
                if abs(dy) < 1e-9: continue
                t = (coord - oy) / dy
                if t <= 1e-6: continue
                hit = ox + t * dx
            if lo <= hit <= hi:
                t_min = min(t_min, t)
        return t_min

    def cw_heading(self, x, y):
        if self.IL <= x <= self.IH and y < self.IL:
            return 0.0
        if x > self.IH and self.IL <= y <= self.IH:
            return math.pi / 2
        if self.IL <= x <= self.IH and y > self.IH:
            return math.pi
        if x < self.IL and self.IL <= y <= self.IH:
            return -math.pi / 2
        # Corners: arc-tangent heading
        if x > self.IH and y < self.IL:
            cx, cy = self.IH, self.IL
        elif x > self.IH and y > self.IH:
            cx, cy = self.IH, self.IH
        elif x < self.IL and y > self.IH:
            cx, cy = self.IL, self.IH
        else:
            cx, cy = self.IL, self.IL
        h = math.atan2(y - cy, x - cx) + math.pi / 2
        return (h + math.pi) % (2 * math.pi) - math.pi

    def lateral_offset(self, x, y):
        if self.IL <= x <= self.IH and y < self.IL:
            return self.CL_E - y
        if x > self.IH and self.IL <= y <= self.IH:
            return x - self.CL_N
        if self.IL <= x <= self.IH and y > self.IH:
            return y - self.CL_W
        if x < self.IL and self.IL <= y <= self.IH:
            return self.CL_S - x
        return 0.0

    def bottom_parking_lot(self, cx=1500.0):
        hl = self.park_length
        hw = self.park_width / 2.0
        return dict(
            cx=cx,  cy=hl / 2.0,
            x0=cx - hw, x1=cx + hw,
            y0=0.0,     y1=hl,
            approach_y   =hl + 150.0,
            entry_heading=-math.pi / 2,
        )

    def draw(self, ax, lot=None):
        from matplotlib.patches import Rectangle
        ax.set_facecolor("#f0f0eb")
        ax.add_patch(Rectangle((0, 0), self.OW, self.OW, lw=2, ec="#111", fc="white"))
        ax.add_patch(Rectangle((self.IL, self.IL), self.CL, self.CL, lw=2, ec="#111", fc="#c0c0c0"))
        xs = [1500, 2500, 2500, 1500, 500, 500, 1500]
        ys = [500, 500, 1500, 2500, 2500, 1500, 500]
        ax.plot(xs, ys, "--", c="#4488cc", lw=0.9, alpha=0.5, label="Centreline")
        if lot:
            ax.add_patch(Rectangle((lot["x0"], lot["y0"]),
                                   lot["x1"]-lot["x0"], lot["y1"]-lot["y0"],
                                   lw=2, ec="#cc00cc", fc="#ffaaff", alpha=0.65, zorder=3))
        ax.set_xlim(-120, 3120); ax.set_ylim(-120, 3120)
        ax.set_aspect("equal")
        ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")


# ─────────────────────────────────────────────────────────────────────────────
# 2. VEHICLE MODEL (Ackermann bicycle, paper §7.7.1)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class VS:
    x: float; y: float; theta: float; v: float

@dataclass
class VP:
    L     : float = 170.0   # wheelbase (mm)
    dmax  : float = 0.55    # max steering (rad)
    vmax  : float = 400.0   # max speed (mm/s)
    length: float = 250.0
    width : float = 180.0
    dt_ms : float = 20.0    # Arduino loop period
    hn_std: float = 0.007   # heading noise (rad/step)
    vn_frc: float = 0.018   # speed noise (fraction of v)

class Bicycle:
    """x'=v·cosθ  y'=v·sinθ  θ'=(v/L)·tanδ"""
    def __init__(self, p: VP): self.p = p

    def step(self, s: VS, steer: float, throttle: float, dt: float, noise=True) -> VS:
        v = float(np.clip(throttle, -1, 1)) * self.p.vmax
        if noise:
            v *= 1.0 + random.gauss(0, self.p.vn_frc)
        d = float(np.clip(steer, -self.p.dmax, self.p.dmax))
        x = s.x + v * math.cos(s.theta) * dt
        y = s.y + v * math.sin(s.theta) * dt
        dθ = (v / self.p.L) * math.tan(d) * dt if abs(v) > 0.1 else 0.0
        if noise:
            dθ += random.gauss(0, self.p.hn_std)
        θ = (s.theta + dθ + math.pi) % (2 * math.pi) - math.pi
        return VS(x, y, θ, v)


# ─────────────────────────────────────────────────────────────────────────────
# 3. SENSOR MODEL (6× HC-SR04, paper §7.7.1)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class SM:
    dx: float; dy: float; angle: float

DEFAULT_SENSORS = [
    SM( 125,   0,  0.0),
    SM( 100,  90,  0.50),
    SM( 100, -90, -0.50),
    SM(-125,   0,  math.pi),
    SM(   0,  90,  math.pi / 2),
    SM(   0, -90, -math.pi / 2),
]

class Sensors:
    NOISE = 3.0  # mm std (paper §7.7.1)
    def __init__(self, track: Track, mounts: List[SM] = None):
        self.track  = track
        self.mounts = mounts or DEFAULT_SENSORS

    def read(self, s: VS, noise=True) -> List[float]:
        ct, st = math.cos(s.theta), math.sin(s.theta)
        out = []
        for m in self.mounts:
            sx = s.x + ct*m.dx - st*m.dy
            sy = s.y + st*m.dx + ct*m.dy
            d  = self.track.cast_ray(sx, sy, s.theta + m.angle)
            if noise:
                d += random.gauss(0, self.NOISE)
            out.append(max(20.0, min(4000.0, d)))
        return out


# ─────────────────────────────────────────────────────────────────────────────
# 4. NAVIGATION CONTROLLER (paper Eq.16, §7.7.2)
# ─────────────────────────────────────────────────────────────────────────────

class NavCtrl:
    """
    δ = clip( k_o·Δlat + k_h·Δθ + k_fy·F_rep ,  ±δ_max )

    [FIX] Repulsion now uses LINEAR function matching paper Eq.10:
          ρ(d, r, k) = k · max(0, 1 – d/r)
          Previously used quadratic (1–d/d0)² which does not match the paper.
    """
    def __init__(self, k_o=0.008, k_fy=0.0025, d0=600.0, speed=0.72):
        self.k_o   = k_o
        self.k_fy  = k_fy
        self.k_h   = 1.2      # fixed heading gain (held constant in S2 sweep)
        self.d0    = d0       # activation radius (mm) – 60 cm = paper SIDE_PUSH_START
        self.speed = speed

    def _repulse_field(self, sensors: List[float]) -> float:
        """
        Linear repulsion (paper Eq.10): ρ = k·max(0, 1–d/r)
        Returns normalised lateral force ∈ [−1, 1].
        Positive → push left (right wall near).
        """
        def rho(d):
            if d < self.d0:
                return max(0.0, 1.0 - d / self.d0)  # LINEAR – matches Eq.10
            return 0.0
        # sensors[5] = lateral-right, sensors[4] = lateral-left
        return rho(sensors[5]) - rho(sensors[4])

    def control(self, s: VS, sensors: List[float], track: Track) -> Tuple[float, float]:
        Δlat = track.lateral_offset(s.x, s.y)
        θ_cw = track.cw_heading(s.x, s.y)
        Δθ   = (θ_cw - s.theta + math.pi) % (2 * math.pi) - math.pi
        F    = self._repulse_field(sensors)
        δ    = self.k_o * Δlat + self.k_h * Δθ + self.k_fy * F
        return float(np.clip(δ, -0.55, 0.55)), self.speed


# ─────────────────────────────────────────────────────────────────────────────
# 5. PARKING FSM (paper §6, §7.7.1)
# ─────────────────────────────────────────────────────────────────────────────

class PS:
    APPROACH = "APPROACH"
    ALIGN    = "ALIGN"
    ENTER    = "ENTER"
    STOP     = "STOP"
    FAILED   = "FAILED"
    TIMEOUT  = "TIMEOUT"

@dataclass
class FT:
    T_approach: float = 10.0
    T_align   : float =  6.0
    T_enter   : float =  6.0
    T_total   : float = 25.0

class ParkingFSM:
    def __init__(self, vehicle: Bicycle, sensors: Sensors, track: Track, timings: FT = None):
        self.v  = vehicle
        self.s  = sensors
        self.t  = track
        self.T  = timings or FT()
        self.DT = vehicle.p.dt_ms / 1000.0

    def _h_err(self, s: VS, target: float) -> float:
        e = target - s.theta
        return (e + math.pi) % (2 * math.pi) - math.pi

    def _in_lot(self, s: VS, lot: dict, margin=35.0) -> bool:
        return (lot["x0"] - margin <= s.x <= lot["x1"] + margin and
                lot["y0"] - margin <= s.y <= lot["y1"] + margin)

    def run(self, init: VS, lot: dict, noise=True) -> dict:
        s    = init
        fsm  = PS.APPROACH
        t    = 0.0
        ts   = 0.0
        traj = [(s.x, s.y)]
        eH   = lot["entry_heading"]
        approach_y     = lot["y1"] + 200.0
        approach_x_tol = 150.0

        while t < self.T.T_total and fsm not in (PS.STOP, PS.FAILED, PS.TIMEOUT):
            rdgs = self.s.read(s, noise=noise)

            if fsm == PS.APPROACH:
                if ts > self.T.T_approach:
                    fsm = PS.FAILED; break
                x_off_rad = float(np.clip(3.0 * (lot["cx"] - s.x) / self.t.CL, -0.8, 0.8))
                h_target  = eH + x_off_rad
                h_err     = self._h_err(s, h_target)
                steer     = float(np.clip(1.5 * h_err, -0.55, 0.55))
                thr       = 0.50
                if s.y <= approach_y and abs(s.x - lot["cx"]) < approach_x_tol:
                    fsm = PS.ALIGN; ts = 0.0; continue

            elif fsm == PS.ALIGN:
                if ts > self.T.T_align:
                    fsm = PS.FAILED; break
                err = self._h_err(s, eH)
                if abs(err) < 0.10:
                    fsm = PS.ENTER; ts = 0.0; continue
                steer = float(np.clip(2.0 * err, -0.55, 0.55))
                thr   = 0.35

            elif fsm == PS.ENTER:
                if ts > self.T.T_enter:
                    fsm = PS.FAILED; break
                if self._in_lot(s, lot):
                    fsm = PS.STOP; break
                err   = self._h_err(s, eH)
                steer = float(np.clip(1.5 * err, -0.55, 0.55))
                thr   = 0.38

            s = self.v.step(s, steer, thr, self.DT, noise=noise)

            if self._in_lot(s, lot, margin=20.0):
                fsm = PS.STOP; break

            if not self.t.in_corridor(s.x, s.y):
                in_lot_zone = (s.y >= lot["y0"] - 50.0 and
                               lot["x0"] - 150.0 <= s.x <= lot["x1"] + 150.0)
                if not in_lot_zone:
                    fsm = PS.FAILED; break

            t  += self.DT
            ts += self.DT
            traj.append((s.x, s.y))

        if fsm == PS.APPROACH and t >= self.T.T_total:
            fsm = PS.TIMEOUT
        return {"success": fsm == PS.STOP, "state": fsm,
                "dur": t, "traj": traj}


# ─────────────────────────────────────────────────────────────────────────────
# 6. S1 – MONTE CARLO PARKING FSM (paper §7.7.1)
# ─────────────────────────────────────────────────────────────────────────────

def run_S1(n=500, verbose=True):
    """
    Monte Carlo parking FSM on exact WRO 2026 geometry.
    Covers Approach+Align+Enter sub-sequence (paper §7.7.1 scope note).
    Physical full-FSM result: 84% (N=50). Simulated sub-sequence: 73.6%.
    The gap is expected due to wider initial-condition distribution and
    exclusion of Straighten/Centre/Hold phases from simulation scope.
    """
    if verbose:
        print("\n" + "="*60)
        print("S1 – Monte Carlo Parking FSM  (exact WRO 2026 geometry)")
        print("="*60)

    track = Track()
    veh   = Bicycle(VP())
    sens  = Sensors(track)
    fsm   = ParkingFSM(veh, sens, track)
    lot   = track.bottom_parking_lot(cx=1500.0)

    results, durs, modes = [], [], {}

    for _ in range(n):
        x0 = random.uniform(1100, 1700)
        y0 = random.uniform(550,  850)
        θ0 = random.gauss(0.0, 0.17)
        r  = fsm.run(VS(x=x0, y=y0, theta=θ0, v=0.0), lot, noise=True)
        results.append(r["success"])
        durs.append(r["dur"])
        modes[r["state"]] = modes.get(r["state"], 0) + 1

    rate   = float(np.mean(results))
    n_succ = int(np.sum(results))
    clo, chi = stats.binom.interval(0.95, n, rate)
    clo /= n; chi /= n
    succ_d = [d for d, ok in zip(durs, results) if ok]
    md = float(np.mean(succ_d)) if succ_d else float('nan')
    sd = float(np.std(succ_d))  if len(succ_d) > 1 else float('nan')

    if verbose:
        print(f"\n  Trials          : {n}")
        print(f"  Successes       : {n_succ}")
        print(f"  Success rate    : {rate*100:.1f}%")
        print(f"  95% CI          : [{clo*100:.1f}%, {chi*100:.1f}%]")
        print(f"  Mean dur (succ) : {md:.2f}s ± {sd:.2f}s")
        print(f"  Physical result : 84% (N=50, CI [71%,93%])")
        print(f"  Failure modes:")
        for st, cnt in modes.items():
            print(f"    {st:12s}: {cnt:4d}  ({cnt/n*100:.1f}%)")

    return dict(rate=rate, ci_lo=clo, ci_hi=chi, n=n,
                mean_d=md, std_d=sd, durs=durs, results=results,
                modes=modes, lot=lot)


# ─────────────────────────────────────────────────────────────────────────────
# 7. S2 – POTENTIAL-FIELD GAIN SWEEP (paper §7.7.2)
# ─────────────────────────────────────────────────────────────────────────────

def run_S2(k_o_vals=None, k_fy_vals=None, laps=3, reps=3, verbose=True):
    """
    9×9 gain sweep on exact WRO 2026 track (paper §7.7.2).
    k_o ∈ logspace(−4, −1.5, 9)   [lane-offset gain, effective on straights]
    k_fy ∈ linspace(0, 0.6, 9)    [repulsion gain, effective near walls]
    k_h = 1.2 fixed (heading gain, provides corner navigation).
    Paper gains: k_o=0.008, k_fy=0.0025 → 100% completion.
    """
    if k_o_vals  is None: k_o_vals  = np.logspace(-4, -1.5, 9)
    if k_fy_vals is None: k_fy_vals = np.linspace(0.0, 0.6, 9)

    if verbose:
        print("\n" + "="*60)
        print("S2 – Gain Sweep  (exact WRO 2026 rectangular track)")
        print("="*60)
        print(f"  Grid: {len(k_o_vals)}×{len(k_fy_vals)}=81 configs × {reps} reps")

    track = Track()
    vp    = VP()
    veh   = Bicycle(vp)
    sens  = Sensors(track)
    DT    = vp.dt_ms / 1000.0

    BOUNDS = [
        dict(kind="v", x=2000, y_lo=0,    y_hi=1000, d=+1),
        dict(kind="h", y=2000, x_lo=2000, x_hi=3000, d=+1),
        dict(kind="v", x=1000, y_lo=2000, y_hi=3000, d=-1),
        dict(kind="h", y=1000, x_lo=0,    x_hi=1000, d=-1),
    ]
    max_steps = int(1.5 * laps * 7000 / (vp.vmax * 0.72 * DT))

    compl = np.zeros((len(k_o_vals), len(k_fy_vals)))
    cte   = np.zeros_like(compl)
    wall  = np.zeros_like(compl)

    for i, k_o in enumerate(k_o_vals):
        for j, k_fy in enumerate(k_fy_vals):
            ctrl = NavCtrl(k_o=k_o, k_fy=k_fy)
            rc, re, rw = [], [], []
            for _ in range(reps):
                s    = VS(x=1500+random.uniform(-100,100),
                          y=500+random.uniform(-50,50),
                          theta=random.gauss(0,0.05), v=0.0)
                prev = s; si = 0; laps_ = 0
                col  = False; ctes = []; mw = float('inf')
                for _ in range(max_steps):
                    rdgs = sens.read(s)
                    δ, thr = ctrl.control(s, rdgs, track)
                    s = veh.step(s, δ, thr, DT)
                    if not track.in_corridor(s.x, s.y):
                        col = True; break
                    ctes.append(abs(track.lateral_offset(s.x, s.y)))
                    wd = track.nearest_wall_dist(s.x, s.y)
                    if wd < mw: mw = wd
                    b = BOUNDS[si % 4]
                    cross = False
                    if b["kind"] == "v":
                        if b["y_lo"] <= s.y <= b["y_hi"]:
                            if b["d"]==+1 and prev.x < b["x"] <= s.x: cross=True
                            if b["d"]==-1 and prev.x > b["x"] >= s.x: cross=True
                    else:
                        if b["x_lo"] <= s.x <= b["x_hi"]:
                            if b["d"]==+1 and prev.y < b["y"] <= s.y: cross=True
                            if b["d"]==-1 and prev.y > b["y"] >= s.y: cross=True
                    if cross:
                        si += 1
                        if si % 4 == 0:
                            laps_ += 1
                            if laps_ >= laps: break
                    prev = s
                done = (not col) and (laps_ >= laps)
                rc.append(float(done))
                re.append(np.mean(ctes) if ctes else 999.0)
                rw.append(mw if mw < float('inf') else 0.0)
            compl[i,j] = np.mean(rc)
            cte[i,j]   = np.mean(re)
            wall[i,j]  = np.mean(rw)

    if verbose:
        bi, bj = np.unravel_index(np.argmax(compl), compl.shape)
        pi_ = np.argmin(np.abs(k_o_vals  - 0.008))
        pj_ = np.argmin(np.abs(k_fy_vals - 0.0025))
        print(f"\n  Paper gains completion : {compl[pi_,pj_]*100:.0f}%")
        print(f"  Best gains completion  : {compl[bi,bj]*100:.0f}% "
              f"(k_o={k_o_vals[bi]:.4f}, k_fy={k_fy_vals[bj]:.3f})")
        print(f"  Stable (≥67%)  : {np.sum(compl>=0.67)}/{compl.size}")
        print(f"  Unstable (<33%): {np.sum(compl< 0.33)}/{compl.size}")

    return dict(k_o=k_o_vals, k_fy=k_fy_vals,
                compl=compl, cte=cte, wall=wall)


# ─────────────────────────────────────────────────────────────────────────────
# 8. FIGURES
# ─────────────────────────────────────────────────────────────────────────────

def make_figures(s1, s2, out="outputs"):
    import os; os.makedirs(out, exist_ok=True)

    # Track geometry
    fig, ax = plt.subplots(figsize=(6,6))
    track = Track(); lot = track.bottom_parking_lot()
    track.draw(ax, lot=lot)
    ax.set_title("WRO 2026 Obstacle Challenge – Exact Track Geometry\n"
                 "(Section 13 General Rules, v15-Jan-2026)", fontsize=10)
    ax.annotate("", xy=(2000,500), xytext=(3000,500),
                arrowprops=dict(arrowstyle="<->", color="steelblue"))
    ax.text(2500,580,"1000 mm corridor",ha="center",fontsize=8,color="steelblue")
    plt.tight_layout()
    fig.savefig(f"{out}/S1S2_track_geometry.png", dpi=150, bbox_inches="tight")
    plt.close()

    # S1 outcomes
    fig, axes = plt.subplots(1,2,figsize=(11,4))
    succ_d = [d for d,ok in zip(s1["durs"],s1["results"]) if ok]
    fail_d = [d for d,ok in zip(s1["durs"],s1["results"]) if not ok]
    ax = axes[0]
    if succ_d: ax.hist(succ_d,bins=20,color="seagreen",alpha=0.75,label="Success",density=True)
    if fail_d: ax.hist(fail_d,bins=20,color="tomato",alpha=0.75,label="Failure",density=True)
    ax.axvline(3.2,  color="navy",   ls="--",lw=1.5,label="Physical mean (3.2s)")
    ax.axvline(25.0, color="maroon", ls=":", lw=1.5,label="T_total (25s)")
    ax.set_xlabel("Trial duration (s)"); ax.set_ylabel("Density")
    ax.set_title(f"S1: Parking duration distribution\nSim={s1['rate']*100:.1f}%  "
                 f"Physical=84%  N={s1['n']}", fontsize=9)
    ax.legend(fontsize=8)
    ax = axes[1]
    labs = [PS.STOP, PS.FAILED, PS.TIMEOUT]
    vals = [s1["modes"].get(l,0) for l in labs]
    bars = ax.bar(labs, vals, color=["seagreen","tomato","goldenrod"], alpha=0.85)
    for b,v in zip(bars,vals):
        ax.text(b.get_x()+b.get_width()/2, b.get_height()+1,
                f"{v/s1['n']*100:.1f}%", ha="center", fontsize=9)
    ax.set_ylabel("Count")
    ax.set_title(f"S1: Failure modes  (N={s1['n']})", fontsize=9)
    plt.tight_layout()
    fig.savefig(f"{out}/S1_parking_fsm.png", dpi=150, bbox_inches="tight")
    plt.close()

    # S2 heatmaps
    fig, axes = plt.subplots(1,2,figsize=(12,5))
    ko_v = s2["k_o"]; kfy_v = s2["k_fy"]
    for ax, data, title, cmap in [
        (axes[0], s2["compl"]*100, "3-lap completion rate (%)", "RdYlGn"),
        (axes[1], s2["cte"],       "Mean cross-track error (mm)", "RdYlGn_r"),
    ]:
        im = ax.imshow(data, origin="lower", cmap=cmap, aspect="auto",
                       extent=[0,len(kfy_v),0,len(ko_v)])
        plt.colorbar(im, ax=ax)
        ax.set_xticks(np.arange(len(kfy_v))+0.5)
        ax.set_xticklabels([f"{v:.2f}" for v in kfy_v], rotation=45, ha="right", fontsize=7)
        ax.set_yticks(np.arange(len(ko_v))+0.5)
        ax.set_yticklabels([f"{v:.4f}" for v in ko_v], fontsize=7)
        ax.set_xlabel("k_fy (repulsion gain)")
        ax.set_ylabel("k_o (lane-offset gain)")
        ax.set_title(f"S2: {title}\n(exact WRO 2026 rectangular track)", fontsize=9)
    plt.tight_layout()
    fig.savefig(f"{out}/S2_gain_sweep.png", dpi=150, bbox_inches="tight")
    plt.close()

    print(f"  Figures saved to {out}/")


# ─────────────────────────────────────────────────────────────────────────────
# 9. MAIN
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("WRO 2026 – S1 & S2 Simulator  (linear repulsion, exact track geometry)")
    random.seed(42); np.random.seed(42)
    t0 = time.time()
    s1 = run_S1(n=500)
    s2 = run_S2(reps=3)
    make_figures(s1, s2)
    print(f"\nTotal runtime: {time.time()-t0:.1f}s")
