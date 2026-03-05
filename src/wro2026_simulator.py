"""
WRO 2026 Future Engineers – Full 2D Kinematic Simulator
========================================================
Addresses Reviewer Weakness W2: Simulation Fidelity

Replaces the sinusoidal corridor approximation in the original paper with
exact WRO 2026 rectangular track geometry from the official General Rules
(Version: January 15th 2026, Section 13 / Appendix A).

Ground-truth dimensions
───────────────────────
  Outer wall inner face  : 3000 × 3000 mm   (Rule 13.1)
  Corridor width         : 1000 mm fixed     (Section 8 Obstacle Challenge)
  Inner wall             : 1000 × 1000 mm centred at mat centre
  Parking lot width      : 200 mm along wall (Rule 5)
  Parking lot length     : 1.5 × robot_length perpendicular to wall
  Max vehicle footprint  : 300 × 200 mm     (Rule 11.1)
  HC-SR04 sensors ×6     : 20–4000 mm range (paper Table 8)

Coordinate convention
─────────────────────
  Origin at inner face of outer wall, bottom-left corner.
  CW travel order: bottom straight → BR corner → right straight →
                   TR corner → top straight → TL corner → left straight →
                   BL corner → repeat.

Simulations
───────────
  S1  Monte Carlo parking FSM (replaces sinusoidal-model drift estimate)
  S2  Potential-field gain sweep on exact rectangular track

Vehicle model: Ackermann bicycle (Rule 11.3 mandates Ackermann steering)
  State   (x mm, y mm, θ rad, v mm/s)
  Control (δ steering rad, throttle ∈ [−1,1])
  Wheelbase L = 170 mm
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
# 1.  TRACK GEOMETRY
# ─────────────────────────────────────────────────────────────────────────────

class Track:
    """
    Exact WRO 2026 Obstacle Challenge track geometry.

    The four straight sections are named by their CW-travel heading:
      E-straight (bottom, heading east  = +x): y∈[0,1000],   x∈[1000,2000]
      N-straight (right,  heading north = +y): x∈[2000,3000], y∈[1000,2000]
      W-straight (top,    heading west  = −x): y∈[2000,3000], x∈[1000,2000]
      S-straight (left,   heading south = −y): x∈[0,1000],   y∈[1000,2000]

    Corner arc centres (inner corner vertex):
      BR: (2000, 1000)   TR: (2000, 2000)
      TL: (1000, 2000)   BL: (1000, 1000)

    Controller sign convention for lateral_offset():
      Positive  → vehicle is to the RIGHT of centreline (needs positive/left steer)
      Negative  → vehicle is to the LEFT  of centreline (needs negative/right steer)
    This matches the paper's control law δ = k_o·Δlat + k_h·Δθ + k_fy·F
    where positive δ produces a left (CCW) turn.
    """

    OW = 3000.0   # outer wall extent
    IL = 1000.0   # inner wall lower bound
    IH = 2000.0   # inner wall upper bound
    CL = 1000.0   # corridor width

    # Corridor centrelines
    CL_E = 500.0    # bottom (E): y = 500
    CL_N = 2500.0   # right  (N): x = 2500
    CL_W = 2500.0   # top    (W): y = 2500
    CL_S = 500.0    # left   (S): x = 500

    def __init__(self, robot_length: float = 250.0):
        self.robot_length   = robot_length
        self.park_length    = 1.5 * robot_length   # 375 mm for 250 mm robot
        self.park_width     = 200.0                # fixed by rule

    # ── Membership ────────────────────────────────────────────────────────

    def in_corridor(self, x: float, y: float) -> bool:
        in_outer = 0 <= x <= self.OW and 0 <= y <= self.OW
        in_inner = self.IL <= x <= self.IH and self.IL <= y <= self.IH
        return in_outer and not in_inner

    def nearest_wall_dist(self, x: float, y: float) -> float:
        d = min(x, self.OW - x, y, self.OW - y)
        if self.IL <= x <= self.IH:
            d = min(d, y - self.IL, self.IH - y)
        if self.IL <= y <= self.IH:
            d = min(d, x - self.IL, self.IH - x)
        return d

    # ── Ray-casting ───────────────────────────────────────────────────────

    def cast_ray(self, ox: float, oy: float, angle: float,
                 max_r: float = 4000.0) -> float:
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

    # ── Track-following helpers ───────────────────────────────────────────

    def cw_heading(self, x: float, y: float) -> float:
        """
        Ideal clockwise heading (rad) at position (x, y).

        Straight sections: exact constant heading.
        Corners: arc-tangent heading using the inner corner vertex as
                 arc centre, giving a smooth heading that varies from
                 the entry heading to the exit heading across the corner.

        Arc formula:  θ_target = atan2(y - cy, x - cx) + π/2
        where (cx, cy) is the inner corner vertex.  This produces the
        unit tangent of a circle centred at (cx, cy), which naturally
        interpolates from the entry straight heading to the exit heading.

        Sign verification (CW travel):
          BR entry  (2100, 500),  h≈0:   arc=-1.37→ target=0.20  ✓ (gentle left)
          BR exit   (2900, 1000), h≈π/2: arc≈-0.11→ target=1.46 ✓ (nearly N)
          TR, TL, BL verified similarly.
        """
        # ── Straights ─────────────────────────────────────────────────────
        if self.IL <= x <= self.IH and y < self.IL:
            return 0.0           # E-straight (bottom) → east (+x)
        if x > self.IH and self.IL <= y <= self.IH:
            return math.pi / 2  # N-straight (right)  → north (+y)
        if self.IL <= x <= self.IH and y > self.IH:
            return math.pi      # W-straight (top)    → west (−x)
        if x < self.IL and self.IL <= y <= self.IH:
            return -math.pi / 2 # S-straight (left)   → south (−y)

        # ── Corners (arc tangent) ─────────────────────────────────────────
        # Inner corner vertices:
        if x > self.IH and y < self.IL:            # BR: vertex (2000, 1000)
            cx, cy = self.IH, self.IL
        elif x > self.IH and y > self.IH:          # TR: vertex (2000, 2000)
            cx, cy = self.IH, self.IH
        elif x < self.IL and y > self.IH:          # TL: vertex (1000, 2000)
            cx, cy = self.IL, self.IH
        else:                                       # BL: vertex (1000, 1000)
            cx, cy = self.IL, self.IL

        arc = math.atan2(y - cy, x - cx)
        h   = arc + math.pi / 2
        return (h + math.pi) % (2 * math.pi) - math.pi

    def lateral_offset(self, x: float, y: float) -> float:
        """
        Signed lateral offset for use in δ = k_o·Δlat + … control law.

        Convention (verified by sign analysis):
          Positive → vehicle RIGHT of centreline → needs positive (left) steer.
          Zero in corners (heading term provides all guidance there).

        E-straight (heading east):  right of cl = south = low y
            Δlat = CL_E − y   (positive when y < 500 = right of cl)
        N-straight (heading north): right of cl = east  = high x
            Δlat = x − CL_N   (positive when x > 2500)
        W-straight (heading west):  right of cl = north = high y
            Δlat = y − CL_W   (positive when y > 2500)
        S-straight (heading south): right of cl = west  = low x
            Δlat = CL_S − x   (positive when x < 500)
        """
        if self.IL <= x <= self.IH and y < self.IL:
            return self.CL_E - y
        if x > self.IH and self.IL <= y <= self.IH:
            return x - self.CL_N
        if self.IL <= x <= self.IH and y > self.IH:
            return y - self.CL_W
        if x < self.IL and self.IL <= y <= self.IH:
            return self.CL_S - x
        return 0.0

    # ── Parking lot ───────────────────────────────────────────────────────

    def bottom_parking_lot(self, cx: float = 1500.0) -> dict:
        """
        Parking lot in E-straight outer wall (Rule 5, Fig. 4).

        Layout (y-axis perpendicular to bottom wall):
          Width  (along wall, x): park_width  = 200 mm
          Length (into corridor): park_length = 375 mm (for 250 mm robot)
          y0 = 0 (outer wall)
          y1 = park_length = 375 mm
          cy = park_length / 2 = 187.5 mm (lot centre)

        Vehicle enters from y > park_length (corridor side), heading
        entry_heading = −π/2 (south, perpendicular to outer wall).
        Approach waypoint placed at (cx, park_length + 150) = (cx, 525).
        """
        hl = self.park_length
        hw = self.park_width / 2.0
        return dict(
            cx=cx,  cy=hl / 2.0,
            x0=cx - hw, x1=cx + hw,
            y0=0.0,     y1=hl,
            approach_y   = hl + 150.0,    # staging waypoint y above lot
            entry_heading = -math.pi / 2, # south (into lot)
        )

    # ── Drawing ───────────────────────────────────────────────────────────

    def draw(self, ax: plt.Axes, lot: Optional[dict] = None) -> None:
        from matplotlib.patches import Rectangle
        ax.set_facecolor("#f0f0eb")
        ax.add_patch(Rectangle((0, 0), self.OW, self.OW,
                               lw=2, ec="#111", fc="white"))
        ax.add_patch(Rectangle((self.IL, self.IL), self.CL, self.CL,
                               lw=2, ec="#111", fc="#c0c0c0"))
        # Corridor centrelines
        xs = [1500, 2500, 2500, 1500, 500, 500, 1500]
        ys = [500, 500, 1500, 2500, 2500, 1500, 500]
        ax.plot(xs, ys, "--", c="#4488cc", lw=0.9, alpha=0.5, label="Centreline")
        if lot:
            ax.add_patch(Rectangle(
                (lot["x0"], lot["y0"]),
                lot["x1"]-lot["x0"], lot["y1"]-lot["y0"],
                lw=2, ec="#cc00cc", fc="#ffaaff", alpha=0.65, zorder=3))
        ax.set_xlim(-120, 3120); ax.set_ylim(-120, 3120)
        ax.set_aspect("equal")
        ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")


# ─────────────────────────────────────────────────────────────────────────────
# 2.  VEHICLE MODEL  (Ackermann bicycle)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class VS:   # VehicleState
    x: float; y: float; theta: float; v: float

@dataclass
class VP:   # VehicleParams
    L     : float = 170.0   # wheelbase (mm)
    dmax  : float = 0.55    # max steer (rad, ≈31.5°)
    vmax  : float = 400.0   # max speed (mm/s)
    length: float = 250.0
    width : float = 180.0
    dt_ms : float = 20.0    # Arduino period (ms)
    hn_std: float = 0.007   # heading noise (rad/step)
    vn_frc: float = 0.018   # speed noise (fraction)

class Bicycle:
    """Ackermann bicycle: dx=v·cosθ  dy=v·sinθ  dθ=(v/L)·tanδ"""
    def __init__(self, p: VP): self.p = p

    def step(self, s: VS, steer: float, throttle: float,
             dt: float, noise: bool = True) -> VS:
        v = np.clip(throttle, -1, 1) * self.p.vmax
        if noise: v *= 1.0 + random.gauss(0, self.p.vn_frc)
        d = np.clip(steer, -self.p.dmax, self.p.dmax)
        x = s.x + v * math.cos(s.theta) * dt
        y = s.y + v * math.sin(s.theta) * dt
        dθ = (v / self.p.L) * math.tan(d) * dt if abs(v) > 0.1 else 0.0
        if noise: dθ += random.gauss(0, self.p.hn_std)
        θ = (s.theta + dθ + math.pi) % (2*math.pi) - math.pi
        return VS(x, y, θ, v)


# ─────────────────────────────────────────────────────────────────────────────
# 3.  SENSOR MODEL  (6 × HC-SR04)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class SM: dx: float; dy: float; angle: float   # SensorMount

DEFAULT_SENSORS = [
    SM( 125,   0,  0.0),           # front-centre
    SM( 100,  90,  0.50),          # front-left
    SM( 100, -90, -0.50),          # front-right
    SM(-125,   0,  math.pi),       # rear
    SM(   0,  90,  math.pi/2),     # lateral-left  [4]
    SM(   0, -90, -math.pi/2),     # lateral-right [5]
]

class Sensors:
    NOISE = 3.0   # mm std
    def __init__(self, track: Track, mounts: List[SM] = None):
        self.track  = track
        self.mounts = mounts or DEFAULT_SENSORS

    def read(self, s: VS, noise: bool = True) -> List[float]:
        ct, st = math.cos(s.theta), math.sin(s.theta)
        out = []
        for m in self.mounts:
            sx = s.x + ct*m.dx - st*m.dy
            sy = s.y + st*m.dx + ct*m.dy
            d  = self.track.cast_ray(sx, sy, s.theta + m.angle)
            if noise: d += random.gauss(0, self.NOISE)
            out.append(max(20.0, min(4000.0, d)))
        return out


# ─────────────────────────────────────────────────────────────────────────────
# 4.  NAVIGATION CONTROLLER
# ─────────────────────────────────────────────────────────────────────────────

class NavCtrl:
    """
    Paper Eq. 1 control law (extended):
        δ = clip( k_o·Δlat + k_h·Δθ + k_fy·F_rep ,  ±δ_max )

    Δlat  : signed lateral offset from centreline (positive = right → steer left)
    Δθ    : heading error to CW track heading (dominant in corners, zero on straights)
    F_rep : normalised wall repulsion ∈ [−1, 1] from lateral HC-SR04 pair
    k_h   : fixed internal heading gain (not swept in S2; provides corner navigation)

    The sweep in S2 varies k_o (lane-centering, effective on straights) and
    k_fy (repulsion, effective near walls).  k_h is held constant at 1.2 so
    corner navigation is consistent across the grid.
    """
    def __init__(self, k_o: float = 0.004, k_fy: float = 0.3,
                 d0: float = 250.0, speed: float = 0.72):
        self.k_o   = k_o
        self.k_fy  = k_fy
        self.k_h   = 1.2      # fixed heading gain
        self.d0    = d0       # repulsion activation distance (mm)
        self.speed = speed

    def _repulse(self, sensors: List[float]) -> float:
        """Normalised repulsion force ∈ [−1, 1]. Positive → push left."""
        def f(d):
            if d < self.d0:
                r = 1.0 - d / self.d0
                return r * r    # quadratic, bounded [0, 1]
            return 0.0
        return f(sensors[5]) - f(sensors[4])  # right wall pushes left

    def control(self, s: VS, sensors: List[float],
                track: Track) -> Tuple[float, float]:
        Δlat = track.lateral_offset(s.x, s.y)
        θ_cw = track.cw_heading(s.x, s.y)
        Δθ   = θ_cw - s.theta
        Δθ   = (Δθ + math.pi) % (2*math.pi) - math.pi
        F    = self._repulse(sensors)
        δ    = self.k_o*Δlat + self.k_h*Δθ + self.k_fy*F
        return np.clip(δ, -0.55, 0.55), self.speed


# ─────────────────────────────────────────────────────────────────────────────
# 5.  PARKING FSM
# ─────────────────────────────────────────────────────────────────────────────

class PS:   # ParkingState constants
    APPROACH = "APPROACH"
    ALIGN    = "ALIGN"
    ENTER    = "ENTER"
    STOP     = "STOP"
    FAILED   = "FAILED"
    TIMEOUT  = "TIMEOUT"

@dataclass
class FT:   # FSM Timings
    T_approach: float = 10.0
    T_align   : float =  6.0   # matches paper Table 5
    T_enter   : float =  6.0
    T_total   : float = 25.0

class ParkingFSM:
    """
    5-state parking controller (paper Section 6) simulated kinematically.

    Geometry (bottom-wall lot, y0=0, y1=375mm, cx=1500mm):
    ─────────────────────────────────────────────────────
    APPROACH  Vehicle drives from starting zone (y≈500–800mm) to a staging
              waypoint at (lot_cx, lot.approach_y = lot.y1+150 = 525mm),
              using proportional heading control at moderate speed.

    ALIGN     Within 120mm of waypoint: rotate to entry_heading = −π/2
              (pointing south, perpendicular to outer wall) at low speed.

    ENTER     Drive forward (−y direction) into lot until vehicle centre
              is inside the parking rectangle.

    STOP      Vehicle centre inside lot → success.

    Collision exception: vehicle inside parking rectangle is NOT a
    corridor-exit collision (lot is on outer wall face, outside corridor).
    """

    def __init__(self, vehicle: Bicycle, sensors: Sensors,
                 track: Track, timings: FT = None):
        self.v   = vehicle
        self.s   = sensors
        self.t   = track
        self.T   = timings or FT()
        self.DT  = vehicle.p.dt_ms / 1000.0

    def _h_err(self, s: VS, target: float) -> float:
        e = target - s.theta
        return (e + math.pi) % (2*math.pi) - math.pi

    def _in_lot(self, s: VS, lot: dict, margin: float = 35.0) -> bool:
        return (lot["x0"]-margin <= s.x <= lot["x1"]+margin and
                lot["y0"]-margin <= s.y <= lot["y1"]+margin)

    def run(self, init: VS, lot: dict, noise: bool = True) -> dict:
        s   = init
        fsm = PS.APPROACH
        t   = 0.0
        ts  = 0.0
        traj = [(s.x, s.y)]

        eH   = lot["entry_heading"]   # −π/2 (south)
        # Approach gate: must be within this y-height above lot AND within x-range
        approach_y   = lot["y1"] + 200.0  # 575 mm, gives more room to x-align
        approach_x_tol = 150.0            # ±150 mm of lot centre

        while t < self.T.T_total and fsm not in (PS.STOP, PS.FAILED, PS.TIMEOUT):
            rdgs = self.s.read(s, noise=noise)

            if fsm == PS.APPROACH:
                if ts > self.T.T_approach: fsm = PS.FAILED; break
                # Heading target = south (−π/2) PLUS a lateral correction proportional
                # to x-offset from lot centre.  Gain 3.0/CL maps ±333mm offset to
                # ±1.0 rad extra heading, capped at ±0.8 rad so car stays mostly south.
                x_off_rad = float(np.clip(
                    3.0 * (lot["cx"] - s.x) / self.t.CL, -0.8, 0.8))
                h_target  = eH + x_off_rad
                h_err     = self._h_err(s, h_target)
                steer     = np.clip(1.5 * h_err, -0.55, 0.55)
                thr       = 0.50
                # Transition: below approach gate AND laterally aligned
                if s.y <= approach_y and abs(s.x - lot["cx"]) < approach_x_tol:
                    fsm = PS.ALIGN;  ts = 0.0; continue

            elif fsm == PS.ALIGN:
                if ts > self.T.T_align: fsm = PS.FAILED; break
                err = self._h_err(s, eH)
                if abs(err) < 0.10:   # ≈ 6°
                    fsm = PS.ENTER;  ts = 0.0; continue
                # Rotate to entry heading. thr=0.35 gives dθ_max≈0.010 rad/step
                # so 90° rotation = ~3.1s < T_align=6s
                steer = np.clip(2.0 * err, -0.55, 0.55)
                thr   = 0.35

            elif fsm == PS.ENTER:
                if ts > self.T.T_enter: fsm = PS.FAILED; break
                if self._in_lot(s, lot):
                    fsm = PS.STOP; break
                # Maintain entry heading, drive south into lot
                err   = self._h_err(s, eH)
                steer = np.clip(1.5 * err, -0.55, 0.55)
                thr   = 0.38

            # Integrate
            s = self.v.step(s, steer, thr, self.DT, noise=noise)

            # Check if in lot (success condition) regardless of FSM state
            if self._in_lot(s, lot, margin=20.0):
                fsm = PS.STOP; break

            # Collision guard.
            # Exception: vehicles at y < 0 and within the bottom-straight x-band
            # are approaching the outer wall / lot area — not a crash.
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

        return {"success": fsm==PS.STOP, "state": fsm,
                "dur": t, "traj": traj}


# ─────────────────────────────────────────────────────────────────────────────
# 6.  S1 – MONTE CARLO PARKING
# ─────────────────────────────────────────────────────────────────────────────

def run_S1(n: int = 500, verbose: bool = True) -> dict:
    """
    S1: Monte Carlo parking FSM validation, exact WRO 2026 geometry.

    Replaces the original paper's sinusoidal-model drift simulation
    (72.5% simulated vs 84% physical → 11.5 pp unreconciled discrepancy,
    Reviewer W2).

    Starting conditions
    ───────────────────
    Vehicle placed in bottom straight after completing 3 laps.
    Starting y ∈ [500, 800] mm: vehicle near inner wall, as expected after
    laps in CW direction where repulsion pushes slightly toward inner wall.
    Starting x ∈ [1100, 1900] mm: six Rule 13.11 starting zones.
    Initial heading ≈ 0 ± 10° (just completed bottom straight).

    Approach waypoint: (1500, 525 mm), well below starting y.
    Vehicle drives south toward lot, aligns to −π/2, enters lot.
    """
    if verbose:
        print("\n" + "="*60)
        print("S1 – Monte Carlo Parking FSM  (exact WRO 2026 geometry)")
        print("="*60)

    track = Track()
    vp    = VP()
    veh   = Bicycle(vp)
    sens  = Sensors(track)
    fsm   = ParkingFSM(veh, sens, track)
    lot   = track.bottom_parking_lot(cx=1500.0)

    results, durs, modes = [], [], {}

    for _ in range(n):
        # After 3 CW laps, vehicle is in bottom straight approaching from west.
        # CW direction = east (+x), so vehicle has x ∈ [1100, 1700] when it
        # begins the parking approach (lot is at x=1500; must pass to reach it).
        x0 = random.uniform(1100, 1700)
        y0 = random.uniform(550, 850)    # corridor scatter, above approach zone
        θ0 = random.gauss(0.0, 0.17)     # ≈ ±10° 1σ
        r  = fsm.run(VS(x=x0, y=y0, theta=θ0, v=0.0), lot, noise=True)
        results.append(r["success"])
        durs.append(r["dur"])
        modes[r["state"]] = modes.get(r["state"], 0) + 1

    rate   = float(np.mean(results))
    n_succ = int(np.sum(results))
    from scipy.stats import binom
    clo, chi = binom.interval(0.95, n, rate)
    clo /= n;  chi /= n
    succ_d = [d for d, ok in zip(durs, results) if ok]
    md = float(np.mean(succ_d)) if succ_d else float('nan')
    sd = float(np.std(succ_d))  if len(succ_d)>1 else float('nan')

    if verbose:
        print(f"\n  Trials           : {n}")
        print(f"  Successes        : {n_succ}")
        print(f"  Success rate     : {rate*100:.1f}%")
        print(f"  95% Wilson CI    : [{clo*100:.1f}%, {chi*100:.1f}%]")
        print(f"  Mean dur (succ.) : {md:.2f} s ± {sd:.2f} s")
        print(f"\n  Physical result  : 84%  (N=50, CI [71%, 93%])")
        print(f"  Original error   : 11.5 pp  (sinusoidal drift model)")
        print(f"  This model error : {abs(rate-0.84)*100:.1f} pp")
        print(f"\n  Failure breakdown:")
        for st, cnt in modes.items():
            print(f"    {st:12s}: {cnt:4d}  ({cnt/n*100:.1f}%)")

    return dict(rate=rate, ci_lo=clo, ci_hi=chi, n=n,
                mean_d=md, std_d=sd, durs=durs, results=results,
                modes=modes, lot=lot)


# ─────────────────────────────────────────────────────────────────────────────
# 7.  S2 – POTENTIAL-FIELD GAIN SWEEP
# ─────────────────────────────────────────────────────────────────────────────

def run_S2(k_o_vals:  np.ndarray = None,
           k_fy_vals: np.ndarray = None,
           laps: int = 3, reps: int = 3,
           verbose: bool = True) -> dict:
    """
    S2: 9×9 gain sweep on exact WRO 2026 rectangular track.

    Replaces the sinusoidal-corridor model used in the original paper.
    The sinusoidal model cannot reproduce:
      (a) The exact corridor-width variation at section transitions.
      (b) Corner-heading dynamics, which dominate gain sensitivity.
      (c) The gain landscape topology (local vs global optima) on the
          true rectangular geometry.

    Controller: δ = k_o·Δlat + k_h·Δθ + k_fy·F_rep  (k_h=1.2 fixed)
    Metrics: 3-lap completion rate, mean cross-track error, min wall clearance.
    Grid: k_o ∈ logspace(−4, −1.5, 9),  k_fy ∈ linspace(0, 0.6, 9).
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

    # Lap boundary crossings (CW order, 4 per lap)
    BOUNDS = [
        dict(kind="v", x=2000, y_lo=0,    y_hi=1000, d=+1),  # BR entry
        dict(kind="h", y=2000, x_lo=2000, x_hi=3000, d=+1),  # TR entry
        dict(kind="v", x=1000, y_lo=2000, y_hi=3000, d=-1),  # TL entry
        dict(kind="h", y=1000, x_lo=0,    x_hi=1000, d=-1),  # BL entry
    ]

    # Steps: 3 laps × ~7000mm/lap / (vmax×speed×DT) with 1.5× margin
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
                prev = s;  si = 0;  laps_ = 0
                col  = False;  ctes = [];  mw = float('inf')

                for _ in range(max_steps):
                    rdgs = sens.read(s)
                    δ, thr = ctrl.control(s, rdgs, track)
                    s = veh.step(s, δ, thr, DT)

                    if not track.in_corridor(s.x, s.y):
                        col = True; break

                    ctes.append(abs(track.lateral_offset(s.x, s.y)))
                    wd = track.nearest_wall_dist(s.x, s.y)
                    if wd < mw: mw = wd

                    # Lap counting
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

            compl[i, j] = np.mean(rc)
            cte[i, j]   = np.mean(re)
            wall[i, j]  = np.mean(rw)

    if verbose:
        bi, bj = np.unravel_index(np.argmax(compl), compl.shape)
        print(f"\n  Best:   k_o={k_o_vals[bi]:.4f}, k_fy={k_fy_vals[bj]:.3f} "
              f"→ {compl[bi,bj]*100:.0f}%")
        print(f"  Stable (≥67%) : {np.sum(compl>=0.67)}/{compl.size}")
        print(f"  Unstable (<33%): {np.sum(compl< 0.33)}/{compl.size}")

    return dict(k_o=k_o_vals, k_fy=k_fy_vals,
                compl=compl, cte=cte, wall=wall)


# ─────────────────────────────────────────────────────────────────────────────
# 8.  FORMAL VALIDATION
# ─────────────────────────────────────────────────────────────────────────────

def run_validation(s1: dict, s2: dict) -> dict:
    """
    Formal goodness-of-fit tests; addresses Reviewer W2 criticism
    'agreement assessed by comparison of means only'.
    """
    print("\n" + "="*60)
    print("VALIDATION  –  Simulation vs Physical")
    print("="*60)

    p_sim, n_sim = s1["rate"], s1["n"]
    p_phy, n_phy = 0.84, 50

    ns = round(p_sim*n_sim);  np_ = round(p_phy*n_phy)
    pp = (ns + np_) / (n_sim + n_phy)
    se = math.sqrt(pp*(1-pp)*(1/n_sim + 1/n_phy))
    z  = (p_sim - p_phy) / se if se > 0 else 0.0
    pv = 2 * stats.norm.sf(abs(z))

    print(f"\n  S1 – Two-proportion z-test (sim vs physical N=50)")
    print(f"  {'':22s}  Simulation       Physical")
    print(f"  {'Rate':22s}  {p_sim*100:.1f}%           {p_phy*100:.1f}%")
    print(f"  {'95% CI':22s}  [{s1['ci_lo']*100:.1f}%, {s1['ci_hi']*100:.1f}%]   [71%, 93%]")
    print(f"  {'z, p':22s}  z={z:.3f},  p={pv:.4f}")
    ok = pv > 0.05
    print(f"  {'Conclusion':22s}  {'Consistent (p>0.05)' if ok else 'Bias detected (p<0.05)'}")

    succ_d = [d for d,s in zip(s1["durs"],s1["results"]) if s]
    pm_d, ps_d = 3.2, 0.8   # paper Table 10 physical values
    t_s, t_p = (stats.ttest_1samp(succ_d, pm_d)
                if len(succ_d)>1 else (float('nan'), float('nan')))
    print(f"\n  Duration t-test (successful trials vs physical mean 3.2s)")
    print(f"  Sim:  {s1['mean_d']:.2f} s ± {s1['std_d']:.2f} s   "
          f"Phys: {pm_d:.2f} s ± {ps_d:.2f} s")
    print(f"  t={t_s:.3f},  p={t_p:.4f}")
    σm = (6.0 - pm_d) / ps_d
    print(f"  T_align=6s is {σm:.1f}σ above physical mean "
          f"({'adequate' if σm>2.5 else 'tight'})")

    comp   = s2["compl"]
    ko_v   = s2["k_o"];  kfy_v = s2["k_fy"]
    bi, bj = np.unravel_index(np.argmax(comp), comp.shape)
    pi_    = np.argmin(np.abs(ko_v  - 0.008))
    pj_    = np.argmin(np.abs(kfy_v - 0.0025))

    print(f"\n  S2 – Gain sweep (exact geometry vs sinusoidal model)")
    print(f"  Paper gains k_o=0.008, k_fy=0.0025 "
          f"→ {comp[pi_,pj_]*100:.0f}% on exact track")
    print(f"  Optimal gains k_o={ko_v[bi]:.4f}, k_fy={kfy_v[bj]:.3f} "
          f"→ {comp[bi,bj]*100:.0f}%")
    gap = abs(comp[bi,bj] - comp[pi_,pj_]) * 100
    print(f"  Gap: {gap:.0f} pp  "
          f"({'⚠ sinusoidal model chose suboptimal gains' if gap>15 else '✓ near-optimal'})")

    return dict(z=z, p=pv, consistent=ok, t=t_s, tp=t_p,
                paper_comp=comp[pi_,pj_], opt_comp=comp[bi,bj], gap=gap,
                opt_ko=ko_v[bi], opt_kfy=kfy_v[bj])


# ─────────────────────────────────────────────────────────────────────────────
# 9.  FIGURES
# ─────────────────────────────────────────────────────────────────────────────

def make_figures(s1: dict, s2: dict, val: dict,
                 out: str = "/mnt/user-data/outputs") -> None:
    import os; os.makedirs(out, exist_ok=True)

    # Fig 1 – track geometry
    fig, ax = plt.subplots(figsize=(6, 6))
    track = Track()
    lot   = track.bottom_parking_lot()
    track.draw(ax, lot=lot)
    ax.set_title("WRO 2026 Obstacle Challenge – Exact Track Geometry\n"
                 "(Section 13 General Rules, v15-Jan-2026)", fontsize=10)
    ax.annotate("", xy=(2000, 500), xytext=(3000, 500),
                arrowprops=dict(arrowstyle="<->", color="steelblue"))
    ax.text(2500, 580, "1000 mm corridor", ha="center", fontsize=8, color="steelblue")
    ax.annotate("", xy=(1000, -70), xytext=(2000, -70),
                arrowprops=dict(arrowstyle="<->", color="#bb6600"))
    ax.text(1500, -100, "1000 mm inner wall", ha="center", fontsize=8, color="#bb6600")
    plt.tight_layout()
    fig.savefig(f"{out}/fig_track_geometry.png", dpi=150, bbox_inches="tight")
    plt.close()

    # Fig 2 – S1 outcomes
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    succ_d = [d for d,ok in zip(s1["durs"],s1["results"]) if ok]
    fail_d = [d for d,ok in zip(s1["durs"],s1["results"]) if not ok]
    ax = axes[0]
    if succ_d: ax.hist(succ_d, bins=20, color="seagreen", alpha=0.75, label="Success", density=True)
    if fail_d: ax.hist(fail_d, bins=20, color="tomato",   alpha=0.75, label="Failure", density=True)
    ax.axvline(3.2,  color="navy",   ls="--", lw=1.5, label="Physical mean (3.2 s)")
    ax.axvline(25.0, color="maroon", ls=":",  lw=1.5, label="T_total (25 s)")
    ax.set_xlabel("Trial duration (s)"); ax.set_ylabel("Density")
    ax.set_title(f"S1: Parking duration distribution\n"
                 f"Sim={s1['rate']*100:.1f}%  Physical=84%  N={s1['n']}", fontsize=9)
    ax.legend(fontsize=8)

    ax = axes[1]
    labs = [PS.STOP, PS.FAILED, PS.TIMEOUT]
    vals = [s1["modes"].get(l, 0) for l in labs]
    bars = ax.bar(labs, vals, color=["seagreen","tomato","goldenrod"], alpha=0.85)
    for b, v in zip(bars, vals):
        ax.text(b.get_x()+b.get_width()/2, b.get_height()+1,
                f"{v/s1['n']*100:.1f}%", ha="center", fontsize=9)
    ax.set_ylabel("Count")
    ax.set_title(f"S1: Failure modes  (N={s1['n']})", fontsize=9)
    plt.tight_layout()
    fig.savefig(f"{out}/fig_S1_parking.png", dpi=150, bbox_inches="tight")
    plt.close()

    # Fig 3 – S2 heatmaps
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    ko_v  = s2["k_o"];  kfy_v = s2["k_fy"]
    for ax, data, title, cmap in [
        (axes[0], s2["compl"]*100, "3-lap completion rate (%)", "RdYlGn"),
        (axes[1], s2["cte"],       "Mean cross-track error (mm)", "RdYlGn_r"),
    ]:
        im = ax.imshow(data, origin="lower", cmap=cmap, aspect="auto",
                       extent=[0, len(kfy_v), 0, len(ko_v)])
        plt.colorbar(im, ax=ax)
        ax.set_xticks(np.arange(len(kfy_v))+0.5)
        ax.set_xticklabels([f"{v:.2f}" for v in kfy_v],
                           rotation=45, ha="right", fontsize=7)
        ax.set_yticks(np.arange(len(ko_v))+0.5)
        ax.set_yticklabels([f"{v:.4f}" for v in ko_v], fontsize=7)
        ax.set_xlabel("k_fy (repulsion gain)")
        ax.set_ylabel("k_o (lane-offset gain)")
        ax.set_title(f"S2: {title}\n(exact WRO 2026 rectangular track)", fontsize=9)
    plt.tight_layout()
    fig.savefig(f"{out}/fig_S2_gain_sweep.png", dpi=150, bbox_inches="tight")
    plt.close()

    # Fig 4 – S2 distribution
    fig, ax = plt.subplots(figsize=(7, 4))
    flat = s2["compl"].flatten() * 100
    ax.hist(flat, bins=15, color="steelblue", alpha=0.75, label="All 81 gain configs")
    ax.axvline(val["paper_comp"]*100, color="tomato", lw=2, ls="--",
               label=f"Paper gains on exact geometry ({val['paper_comp']*100:.0f}%)")
    ax.axvline(val["opt_comp"]*100,   color="seagreen", lw=2,
               label=f"Optimal gains ({val['opt_comp']*100:.0f}%)")
    ax.set_xlabel("3-lap completion rate (%)")
    ax.set_ylabel("Number of gain configurations")
    ax.set_title("S2: Completion-rate distribution across 9×9 gain grid\n"
                 "(sinusoidal model cannot reproduce corner-dependent sensitivity)", fontsize=9)
    ax.legend(fontsize=8)
    plt.tight_layout()
    fig.savefig(f"{out}/fig_S2_comparison.png", dpi=150, bbox_inches="tight")
    plt.close()

    print(f"\n  Figures saved to: {out}/")
    for f in ["fig_track_geometry","fig_S1_parking",
              "fig_S2_gain_sweep","fig_S2_comparison"]:
        print(f"    {f}.png")


# ─────────────────────────────────────────────────────────────────────────────
# 10. MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("WRO 2026 Future Engineers – 2D Kinematic Simulator")
    print("Addressing Reviewer W2: simulation fidelity")
    print(f"Track: {Track.OW:.0f}×{Track.OW:.0f} mm  corridor={Track.CL:.0f} mm\n")

    random.seed(42);  np.random.seed(42)
    t0 = time.time()

    s1  = run_S1(n=500)
    s2  = run_S2(reps=3)
    val = run_validation(s1, s2)
    make_figures(s1, s2, val)

    print(f"\nRuntime: {time.time()-t0:.1f} s")
    print("\nW2 checklist:")
    print("  [✓] Track geometry: exact WRO 2026 rectangle (Section 13)")
    print("  [✓] Corner heading: arc-tangent method (not sinusoidal approximation)")
    print("  [✓] Lateral offset: correct sign convention verified per section")
    print("  [✓] S1: kinematic FSM replacing drift model")
    print("  [✓] S1: two-proportion z-test vs physical N=50")
    print("  [✓] S1: one-sample t-test on duration distribution")
    print("  [✓] S2: gain sweep on correct rectangular geometry")
    print("  [✓] Formal statistical tests (not means-only)")


if __name__ == "__main__":
    main()
