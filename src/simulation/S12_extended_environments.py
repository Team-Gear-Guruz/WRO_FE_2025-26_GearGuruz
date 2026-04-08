#!/usr/bin/env python3
"""
S12 – Extended Environment Simulation Suite
==========================================
Extends S1–S11 with seven new experiments that vary environmental conditions
not explored in the baseline paper.

  E1  Track Geometry Comparison  – WRO rect, non-square rect, L-shape, T-junction
  E2  Corridor Width (fixed gains)– performance when paper gains meet width ≠ 1000 mm
  E3  Obstacle Configurations    – 0, 2, 4, 8 cylindrical pillars (r = 50 mm)
  E4  Surface Friction Sweep     – deceleration 30 → 120 cm/s² (stop-distance safety)
  E5  Exhaustive Sensor Failure  – all C(6,1)=6 single + C(6,2)=15 pair HC-SR04 combos
  E6  Speed vs Safety Envelope   – velocity 10 → 50 cm/s × friction × architecture
  E7  Outdoor Conditions         – wind, ground vibration, thermal sensor noise, dust

Note on relation to S8:
  S8 derives OPTIMAL gain scaling laws as a function of corridor width W.
  E2 here instead holds the paper's published gains fixed and measures the
  resulting performance degradation — a complementary robustness question.

Run standalone:
  python S12_extended_environments.py

Integrate into run_all_simulations.py:
  from S12_extended_environments import run_S12
  results["S12"] = run_S12(out=OUTPUT_DIR)
"""

from __future__ import annotations
import math, random, os, sys, itertools, time
from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Polygon as MplPolygon, FancyArrowPatch
from matplotlib.lines import Line2D

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from S1_S2_simulator import Bicycle, VP, VS, Sensors, SM, NavCtrl, DEFAULT_SENSORS

OUTPUT_DIR   = "outputs"
STEPS        = 3_000   # simulation steps per episode (matches S4 baseline)
PAPER_K_O    = 0.008
PAPER_K_FY   = 0.0025
PAPER_DECEL  = 80.0    # cm/s² (from S3 calibration)
HARD_STOP_CM = 15.0    # cm
SENSOR_NAMES = {0:"FC", 1:"FLD", 2:"FRD", 3:"B", 4:"L", 5:"R"}


# ══════════════════════════════════════════════════════════════════════════════
# §1  GEOMETRIC HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def _cross2d(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * by - ay * bx


def _pt_seg_closest(px, py, ax, ay, bx, by):
    """Return (foot_x, foot_y, t∈[0,1], distance) from P to segment AB."""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return ax, ay, 0.0, math.hypot(px - ax, py - ay)
    t  = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    fx, fy = ax + t * dx, ay + t * dy
    return fx, fy, t, math.hypot(px - fx, py - fy)


def _pt_in_polygon(x: float, y: float, poly: list) -> bool:
    """Ray-casting point-in-polygon test (correct for any simple polygon)."""
    n, inside, j = len(poly), False, len(poly) - 1
    for i in range(n):
        xi, yi = poly[i];  xj, yj = poly[j]
        if (yi > y) != (yj > y):
            xint = xi + (y - yi) * (xj - xi) / (yj - yi + 1e-15)
            if x < xint:
                inside = not inside
        j = i
    return inside


# ══════════════════════════════════════════════════════════════════════════════
# §2  GENERAL SEGMENT-WALL TRACK
# ══════════════════════════════════════════════════════════════════════════════

class SegmentWallTrack:
    """
    2-D track defined by polygon walls and a piecewise-linear centreline.

    Supports duck-typing with the original Track class:
      .cast_ray(ox, oy, angle)  →  float
      .in_corridor(x, y)        →  bool
      .cw_heading(x, y)         →  float  [rad]
      .lateral_offset(x, y)     →  float  [mm], +ve = right of travel direction
      .nearest_wall_dist(x, y)  →  float  [mm]

    Parameters
    ----------
    outer_poly  : list[(x,y)]        outer boundary vertices
    inner_polys : list[list[(x,y)]]  inner obstacles / holes to exclude
    centerline  : list[(x,y)]        ordered travel waypoints (CW convention)
    name        : str
    """

    def __init__(self, outer_poly, inner_polys, centerline,
                 name="Custom", robot_length=250.0):
        self.outer_poly  = list(outer_poly)
        self.inner_polys = [list(p) for p in (inner_polys or [])]
        self.centerline  = list(centerline)
        self.name        = name
        self.robot_length = robot_length
        self.park_length  = 1.5 * robot_length
        self.park_width   = 200.0
        # CL corridor-width estimate (for info only)
        self.CL = 1000.0

        self.all_segs: List[Tuple] = []
        self._add_poly(self.outer_poly)
        for ip in self.inner_polys:
            self._add_poly(ip)

        self._cl_segs = [
            (self.centerline[i], self.centerline[i + 1])
            for i in range(len(self.centerline) - 1)
        ]

    def _add_poly(self, poly):
        for i in range(len(poly)):
            self.all_segs.append((poly[i], poly[(i + 1) % len(poly)]))

    # ── Ray casting ────────────────────────────────────────────────────────────
    def cast_ray(self, ox: float, oy: float, angle: float,
                 max_r: float = 4000.0) -> float:
        dx, dy = math.cos(angle), math.sin(angle)
        t_min  = max_r
        for (ax, ay), (bx, by) in self.all_segs:
            vx, vy = bx - ax, by - ay
            denom  = _cross2d(dx, dy, vx, vy)
            if abs(denom) < 1e-9:
                continue
            aox, aoy = ax - ox, ay - oy
            t = _cross2d(aox, aoy, vx, vy) / denom
            s = _cross2d(aox, aoy, dx, dy) / denom
            if t > 1e-6 and -1e-6 <= s <= 1 + 1e-6:
                t_min = min(t_min, t)
        return t_min

    # ── Corridor membership ────────────────────────────────────────────────────
    def in_corridor(self, x: float, y: float) -> bool:
        if not _pt_in_polygon(x, y, self.outer_poly):
            return False
        for ip in self.inner_polys:
            if _pt_in_polygon(x, y, ip):
                return False
        return True

    def nearest_wall_dist(self, x: float, y: float) -> float:
        d_min = float("inf")
        for (ax, ay), (bx, by) in self.all_segs:
            _, _, _, d = _pt_seg_closest(x, y, ax, ay, bx, by)
            d_min = min(d_min, d)
        return d_min

    # ── Centreline helpers ─────────────────────────────────────────────────────
    def _nearest_cl(self, x: float, y: float):
        """(segment_idx, foot_x, foot_y, t, dist)"""
        best = None
        for i, ((ax, ay), (bx, by)) in enumerate(self._cl_segs):
            fx, fy, t, d = _pt_seg_closest(x, y, ax, ay, bx, by)
            if best is None or d < best[4]:
                best = (i, fx, fy, t, d)
        return best

    def cw_heading(self, x: float, y: float) -> float:
        res = self._nearest_cl(x, y)
        if res is None:
            return 0.0
        i = res[0]
        ax, ay = self._cl_segs[i][0];  bx, by = self._cl_segs[i][1]
        return math.atan2(by - ay, bx - ax)

    def lateral_offset(self, x: float, y: float) -> float:
        """
        Signed lateral offset (mm).
        +ve  →  vehicle is RIGHT of centreline  →  NavCtrl steers left (+δ).
        """
        res = self._nearest_cl(x, y)
        if res is None:
            return 0.0
        i, fx, fy, t, _ = res
        ax, ay = self._cl_segs[i][0];  bx, by = self._cl_segs[i][1]
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length < 1e-9:
            return 0.0
        # cross(direction, P-foot): +ve when P is LEFT of direction
        # negate so that +ve = right (outer wall) → steer left
        cross = _cross2d(dx, dy, x - fx, y - fy)
        return -cross / length

    # ── Drawing ────────────────────────────────────────────────────────────────
    def draw(self, ax_mpl, title=None, traj=None):
        ax_mpl.set_facecolor("#f0f0eb")
        outer = MplPolygon(self.outer_poly, closed=True, lw=2,
                           ec="#222", fc="white", zorder=1)
        ax_mpl.add_patch(outer)
        for ip in self.inner_polys:
            inner = MplPolygon(ip, closed=True, lw=2,
                               ec="#222", fc="#c0c0c0", zorder=2)
            ax_mpl.add_patch(inner)
        xs = [p[0] for p in self.centerline]
        ys = [p[1] for p in self.centerline]
        ax_mpl.plot(xs, ys, "--", c="#4488cc", lw=0.9, alpha=0.5, zorder=3)
        if traj:
            tx, ty = zip(*traj)
            ax_mpl.plot(tx, ty, "-", c="#cc4400", lw=0.7, alpha=0.55, zorder=4)
        all_x = [p[0] for p in self.outer_poly]
        all_y = [p[1] for p in self.outer_poly]
        pad = 200
        ax_mpl.set_xlim(min(all_x) - pad, max(all_x) + pad)
        ax_mpl.set_ylim(min(all_y) - pad, max(all_y) + pad)
        ax_mpl.set_aspect("equal")
        ax_mpl.set_title(title or self.name, fontsize=8)
        ax_mpl.set_xlabel("x (mm)", fontsize=7)
        ax_mpl.set_ylabel("y (mm)", fontsize=7)
        ax_mpl.tick_params(labelsize=6)


# ══════════════════════════════════════════════════════════════════════════════
# §3  TRACK FACTORY FUNCTIONS
# ══════════════════════════════════════════════════════════════════════════════

def make_wro_rect(outer=3000.0, corridor=1000.0) -> SegmentWallTrack:
    """Standard WRO 2026 square loop, parameterised."""
    il = corridor; ih = outer - corridor          # inner wall extents
    cw = corridor / 2                             # centreline offset
    outer_poly = [(0,0),(outer,0),(outer,outer),(0,outer)]
    inner_poly = [(il,il),(ih,il),(ih,ih),(il,ih)]
    # CW centreline through corner midpoints
    cl = [
        (outer/2, cw), (ih+cw, cw), (ih+cw, outer/2),
        (ih+cw, ih+cw), (outer/2, ih+cw),
        (cw, ih+cw), (cw, outer/2), (cw, cw), (outer/2, cw),
    ]
    t = SegmentWallTrack(outer_poly, [inner_poly], cl,
                         name=f"WRO Rect {outer/10:.0f}×{outer/10:.0f} cm "
                              f"(W={corridor/10:.0f} cm)")
    t.CL = corridor
    return t


def make_nonsquare_rect(outer_w=4000.0, outer_h=2500.0,
                        corridor=1000.0) -> SegmentWallTrack:
    """Elongated rectangular loop (wider than tall), equal corridor all sides."""
    inner_w = outer_w - 2 * corridor
    inner_h = outer_h - 2 * corridor
    ix0, iy0 = corridor, corridor
    ix1, iy1 = outer_w - corridor, outer_h - corridor
    outer_poly = [(0,0),(outer_w,0),(outer_w,outer_h),(0,outer_h)]
    inner_poly = [(ix0,iy0),(ix1,iy0),(ix1,iy1),(ix0,iy1)]
    cw = corridor / 2
    cl = [
        (outer_w/2, cw), (ix1+cw, cw), (ix1+cw, outer_h/2),
        (ix1+cw, iy1+cw), (outer_w/2, iy1+cw),
        (cw, iy1+cw), (cw, outer_h/2), (cw, cw), (outer_w/2, cw),
    ]
    t = SegmentWallTrack(outer_poly, [inner_poly], cl,
                         name=f"Non-Sq Rect {outer_w/10:.0f}×{outer_h/10:.0f} cm "
                              f"(W={corridor/10:.0f} cm)")
    t.CL = corridor
    return t


def make_l_shape(horiz_len=3000.0, corridor=1000.0,
                 vert_len=3000.0) -> SegmentWallTrack:
    """
    L-shaped corridor: horizontal section full width, then a right turn into a
    vertical section.  No inner obstacle — walls define the L.

    Horizontal: x = 0 … horiz_len,  y = 0 … corridor
    Vertical  : x = horiz_len-corridor … horiz_len,  y = 0 … vert_len
    """
    vx0 = horiz_len - corridor          # start of vertical section (x)
    outer_poly = [
        (0, 0), (horiz_len, 0), (horiz_len, vert_len),
        (vx0, vert_len), (vx0, corridor), (0, corridor),
    ]
    cw_h = corridor / 2                 # centreline y in horizontal section
    cw_x = vx0 + corridor / 2          # centreline x in vertical section
    cl = [
        (cw_h, cw_h),
        (vx0 - corridor / 2, cw_h),
        (cw_x, cw_h),
        (cw_x, corridor + corridor / 2),
        (cw_x, vert_len - corridor / 2),
    ]
    t = SegmentWallTrack(outer_poly, [], cl,
                         name=f"L-Shape "
                              f"{horiz_len/10:.0f}×{vert_len/10:.0f} cm "
                              f"(W={corridor/10:.0f} cm)")
    t.CL = corridor
    return t


def make_t_junction(horiz_len=4000.0, corridor=1000.0,
                    stem_len=2000.0) -> SegmentWallTrack:
    """
    T-shaped corridor: horizontal bar, then a straight stem upward from centre.
    Robot route: left entry → centre → up stem.
    """
    sx0 = (horiz_len - corridor) / 2   # stem left edge x
    sx1 = sx0 + corridor               # stem right edge x
    sy1 = corridor + stem_len          # stem top y
    outer_poly = [
        (0, 0), (horiz_len, 0), (horiz_len, corridor),
        (sx1, corridor), (sx1, sy1), (sx0, sy1),
        (sx0, corridor), (0, corridor),
    ]
    cw_h = corridor / 2
    cw_x = sx0 + corridor / 2
    cl = [
        (cw_h, cw_h),
        (cw_x - corridor / 2, cw_h),
        (cw_x, cw_h),
        (cw_x, corridor + corridor / 2),
        (cw_x, sy1 - corridor / 2),
    ]
    t = SegmentWallTrack(outer_poly, [], cl,
                         name=f"T-Junction "
                              f"{horiz_len/10:.0f}×{(corridor+stem_len)/10:.0f} cm "
                              f"(W={corridor/10:.0f} cm)")
    t.CL = corridor
    return t


def add_cylindrical_obstacles(track: SegmentWallTrack,
                               positions: List[Tuple[float, float]],
                               radius: float = 50.0) -> SegmentWallTrack:
    """
    Return a NEW track with cylindrical pillars at `positions` (x,y centres).
    Each pillar is approximated as a 12-gon. The original track is unchanged.
    """
    import copy
    t2 = copy.deepcopy(track)
    t2.name += f" + {len(positions)} pillar(s)"
    for cx, cy in positions:
        pts = [(cx + radius * math.cos(2 * math.pi * k / 12),
                cy + radius * math.sin(2 * math.pi * k / 12))
               for k in range(12)]
        t2.inner_polys.append(pts)
        t2._add_poly(pts)
    return t2


# ══════════════════════════════════════════════════════════════════════════════
# §4  SIMULATION KERNEL
# ══════════════════════════════════════════════════════════════════════════════

def _make_vp(speed_scale=1.0, heading_noise_scale=1.0) -> VP:
    base = VP()
    return VP(
        L=base.L, dmax=base.dmax,
        vmax=base.vmax * speed_scale,
        length=base.length, width=base.width, dt_ms=base.dt_ms,
        hn_std=base.hn_std * heading_noise_scale,
        vn_frc=base.vn_frc,
    )


def simulate(track: SegmentWallTrack,
             ctrl_kw: dict = None,
             steps: int = STEPS,
             init: Optional[VS] = None,
             failed_sensors: tuple = (),
             sensor_noise_scale: float = 1.0,
             speed_scale: float = 1.0,
             heading_noise_scale: float = 1.0,
             rng_seed: int = 42) -> dict:
    """
    Run one episode on `track` and return performance metrics.

    Returns
    -------
    dict: collisions, mean_cte, min_wall_dist, lateral_errs, traj
    """
    random.seed(rng_seed);  np.random.seed(rng_seed)
    ctrl_kw = ctrl_kw or dict(k_o=PAPER_K_O, k_fy=PAPER_K_FY)

    ctrl = NavCtrl(**ctrl_kw)
    vp   = _make_vp(speed_scale, heading_noise_scale)
    veh  = Bicycle(vp)
    sens = Sensors(track)
    sens.NOISE = Sensors.NOISE * sensor_noise_scale   # instance override
    DT   = vp.dt_ms / 1000.0

    # Auto-place: midpoint of first CL segment, heading along it
    if init is None:
        if len(track.centerline) >= 2:
            ax, ay = track.centerline[0];  bx, by = track.centerline[1]
            x0, y0 = (ax + bx) / 2, (ay + by) / 2
            θ0 = math.atan2(by - ay, bx - ax)
        else:
            x0, y0, θ0 = 1500.0, 500.0, 0.0
    else:
        x0, y0, θ0 = init.x, init.y, init.theta

    s = VS(x=x0, y=y0, theta=θ0, v=0.0)
    collisions, lateral_errs, wall_dists, traj = 0, [], [], [(s.x, s.y)]

    for _ in range(steps):
        rdgs = sens.read(s)
        for fi in failed_sensors:
            if 0 <= fi < len(rdgs):
                rdgs[fi] = 400.0           # no-echo sentinel
        δ, thr = ctrl.control(s, rdgs, track)
        s = veh.step(s, δ, thr, DT)

        if not track.in_corridor(s.x, s.y):
            collisions += 1
            s = VS(x=x0, y=y0, theta=θ0, v=0.0)   # reset

        lateral_errs.append(abs(track.lateral_offset(s.x, s.y)))
        wall_dists.append(track.nearest_wall_dist(s.x, s.y))
        traj.append((s.x, s.y))

    return dict(
        collisions    = collisions,
        mean_cte      = float(np.mean(lateral_errs)),
        min_wall_dist = float(np.min(wall_dists)) if wall_dists else 0.0,
        lateral_errs  = lateral_errs,
        traj          = traj,
    )


def _mean_result(track, reps=5, **sim_kw):
    """Run `reps` independent replications and average key metrics."""
    cols, ctes, mwds = [], [], []
    traj_sample = None
    for seed in range(reps):
        r = simulate(track, rng_seed=seed * 17 + 3, **sim_kw)
        cols.append(r["collisions"])
        ctes.append(r["mean_cte"])
        mwds.append(r["min_wall_dist"])
        if traj_sample is None:
            traj_sample = r["traj"]
    return dict(
        collisions    = float(np.mean(cols)),
        mean_cte      = float(np.mean(ctes)),
        min_wall_dist = float(np.mean(mwds)),
        traj          = traj_sample,
    )


# ══════════════════════════════════════════════════════════════════════════════
# §5  E1 — TRACK GEOMETRY COMPARISON
# ══════════════════════════════════════════════════════════════════════════════

def run_E1(out=OUTPUT_DIR, verbose=True, reps=5):
    if verbose:
        print("\n" + "=" * 60)
        print("E1 – Track Geometry Comparison")
        print("=" * 60)

    tracks = [
        make_wro_rect(),
        make_nonsquare_rect(),
        make_l_shape(),
        make_t_junction(),
    ]
    results = []
    for tr in tracks:
        r = _mean_result(tr, reps=reps)
        results.append(r)
        if verbose:
            print(f"  {tr.name[:42]:42s} | "
                  f"coll={r['collisions']:5.1f} | "
                  f"CTE={r['mean_cte']:6.1f} mm | "
                  f"min_wall={r['min_wall_dist']:6.1f} mm")

    # ── Figure ────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(18, 9))
    gs  = GridSpec(2, 4, figure=fig, hspace=0.45, wspace=0.35)

    # Row 0: track maps with one sample trajectory
    for k, (tr, r) in enumerate(zip(tracks, results)):
        ax = fig.add_subplot(gs[0, k])
        tr.draw(ax, traj=r["traj"])

    labels = [f"T{k+1}\n{tr.name.split('(')[0].strip()[:18]}"
              for k, tr in enumerate(tracks)]
    colors  = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    cols    = [r["collisions"] for r in results]
    ctes    = [r["mean_cte"]   for r in results]
    mwds    = [r["min_wall_dist"] for r in results]

    ax_c = fig.add_subplot(gs[1, :2])
    bars = ax_c.bar(labels, cols, color=colors, alpha=0.85)
    for b, v in zip(bars, cols):
        ax_c.text(b.get_x() + b.get_width()/2, v + 0.05,
                  f"{v:.1f}", ha="center", fontsize=9, fontweight="bold")
    ax_c.set_ylabel(f"Wall collisions per {STEPS} steps (mean {reps} reps)")
    ax_c.set_title("(a) Collision Count by Track Geometry", fontsize=10)
    ax_c.grid(axis="y", alpha=0.3)

    ax_m = fig.add_subplot(gs[1, 2:])
    x = np.arange(len(tracks));  w = 0.4
    ax_m.bar(x - w/2, ctes, w, color=colors, alpha=0.85, label="Mean CTE (mm)")
    ax_m.bar(x + w/2, mwds, w, color=colors, alpha=0.45, label="Min wall clearance (mm)")
    ax_m.set_xticks(x);  ax_m.set_xticklabels(labels)
    ax_m.set_ylabel("Distance (mm)")
    ax_m.set_title("(b) Tracking Error & Wall Clearance by Track Geometry", fontsize=10)
    ax_m.legend(fontsize=8);  ax_m.grid(axis="y", alpha=0.3)

    fig.suptitle(
        "E1 · Track Geometry Comparison — NavCtrl with paper gains "
        f"(k_o={PAPER_K_O}, k_fy={PAPER_K_FY}) across 4 track types\n"
        f"{reps} reps × {STEPS} steps each", fontsize=11
    )
    path = f"{out}/S12_E1_track_geometry.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    if verbose:
        print(f"  → {path}")
    return dict(tracks=tracks, results=results)


# ══════════════════════════════════════════════════════════════════════════════
# §6  E2 — CORRIDOR WIDTH (FIXED GAINS)
# ══════════════════════════════════════════════════════════════════════════════

def run_E2(out=OUTPUT_DIR, verbose=True, reps=3):
    """
    Sweep corridor width 500 → 1600 mm on the WRO square track,
    keeping paper gains (k_o=0.008, k_fy=0.0025) fixed.

    S8 asks: what OPTIMAL gains are needed for each width?
    E2 asks: what happens to performance if you DON'T rescale the gains?
    """
    if verbose:
        print("\n" + "=" * 60)
        print("E2 – Corridor Width Sweep (fixed paper gains)")
        print("=" * 60)

    widths = np.arange(500, 1650, 100, dtype=float)
    cols, ctes, mwds = [], [], []

    for W in widths:
        tr = make_wro_rect(corridor=W)
        r  = _mean_result(tr, reps=reps)
        cols.append(r["collisions"])
        ctes.append(r["mean_cte"])
        mwds.append(r["min_wall_dist"])
        if verbose:
            print(f"  W={W:6.0f} mm | coll={r['collisions']:5.1f} | "
                  f"CTE={r['mean_cte']:6.1f} mm | min_wall={r['min_wall_dist']:6.1f} mm")

    # ── Figure ────────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle(
        "E2 · Corridor Width Sweep — fixed paper gains (k_o=0.008, k_fy=0.0025)\n"
        f"WRO 3000×3000 mm square track, {reps} reps × {STEPS} steps",
        fontsize=10
    )
    ax = axes[0]
    ax.plot(widths, cols, "r-o", lw=2, ms=5, label="Collisions / run")
    ax.axvline(1000, color="navy", ls="--", lw=1.5, label="Paper width (1000 mm)")
    ax.axhline(0,    color="black", lw=0.8)
    ax.set_xlabel("Corridor width (mm)");  ax.set_ylabel("Wall collisions per run")
    ax.set_title("(a) Collision Count vs Corridor Width")
    ax.legend(fontsize=8);  ax.grid(alpha=0.3)

    ax = axes[1]
    ax.plot(widths, ctes,  "b-o",  lw=2, ms=5, label="Mean CTE (mm)")
    ax.plot(widths, mwds,  "g--s", lw=2, ms=5, label="Min wall clearance (mm)")
    ax.axvline(1000, color="navy", ls="--", lw=1.5, label="Paper width")
    ax.set_xlabel("Corridor width (mm)");  ax.set_ylabel("Distance (mm)")
    ax.set_title("(b) Tracking Quality vs Corridor Width")
    ax.legend(fontsize=8);  ax.grid(alpha=0.3)

    path = f"{out}/S12_E2_corridor_width.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    if verbose:
        print(f"  → {path}")
    return dict(widths=widths, collisions=cols, mean_cte=ctes, min_wall=mwds)


# ══════════════════════════════════════════════════════════════════════════════
# §7  E3 — OBSTACLE CONFIGURATIONS
# ══════════════════════════════════════════════════════════════════════════════

def run_E3(out=OUTPUT_DIR, verbose=True, reps=3):
    """
    Place 0, 2, 4, 8 cylindrical pillars (r=50 mm) in the WRO corridor,
    at positions chosen to be representative of WRO competition obstacle layouts.

    Pillar positions (mm):
      2 pillars : one on bottom straight, one on top straight
      4 pillars : one per straight section
      8 pillars : two per straight section (staggered inner/outer)
    """
    if verbose:
        print("\n" + "=" * 60)
        print("E3 – Obstacle Configurations")
        print("=" * 60)

    base = make_wro_rect()
    R    = 50.0   # pillar radius (mm)

    # Bottom straight (y≈500), right straight (x≈2500), etc.
    PILLAR_SETS = {
        0: [],
        2: [(1000, 500), (2000, 2500)],
        4: [(1000, 500), (2500, 1000), (2000, 2500), (500, 2000)],
        8: [(800,  500), (1800, 500),
            (2500, 800),  (2500, 1800),
            (2200, 2500), (1200, 2500),
            (500,  2200), (500,  1200)],
    }

    CONFIGS = [
        # Two pillars that straddle the centreline — robot must weave between them
        ("Staggered pair",  [(1200, 650), (1800, 350)]),
        # Single pillar placed mid-turn approaching the bottom-right corner
        ("Corner ambush",   [(2400, 600)]),
        # Three pillars in an arc across the bottom straight
        ("Arc trio",        [(1000, 680), (1500, 350), (2000, 680)]),
    ]

    results = []
    for n, positions in PILLAR_SETS.items():
        tr  = add_cylindrical_obstacles(base, positions, radius=R)
        r   = _mean_result(tr, reps=reps)
        r["n_pillars"] = n
        r["label"]     = f"{n} pillars"
        results.append(r)
        if verbose:
            print(f"  {n:2d} pillars | coll={r['collisions']:5.1f} | "
                  f"CTE={r['mean_cte']:6.1f} mm")

    for lbl, positions in CONFIGS:
        tr  = add_cylindrical_obstacles(base, positions, radius=R)
        r   = _mean_result(tr, reps=reps)
        r["n_pillars"] = len(positions)
        r["label"]     = lbl
        results.append(r)
        if verbose:
            print(f"  {lbl:20s} | coll={r['collisions']:5.1f} | "
                  f"CTE={r['mean_cte']:6.1f} mm")

    # ── Figure ────────────────────────────────────────────────────────────────
    labels = [r["label"]       for r in results]
    cols   = [r["collisions"]  for r in results]
    ctes   = [r["mean_cte"]    for r in results]
    nps    = [r["n_pillars"]   for r in results]
    cmap   = {0:"seagreen", 1:"#1f77b4", 2:"darkorange", 3:"tomato", 4:"mediumpurple", 8:"maroon"}
    colors = [cmap.get(n, "#666") for n in nps]

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle(
        f"E3 · Obstacle Configurations — WRO track, pillar r=50 mm\n"
        f"{reps} reps × {STEPS} steps", fontsize=10
    )
    for ax, data, ylabel, title_tag in [
        (axes[0], cols, "Wall collisions per run", "(a) Collision Count"),
        (axes[1], ctes, "Mean CTE (mm)",           "(b) Tracking Error"),
    ]:
        bars = ax.bar(range(len(labels)), data, color=colors, alpha=0.85)
        for b, v in zip(bars, data):
            ax.text(b.get_x() + b.get_width()/2, v + max(data)*0.01,
                    f"{v:.1f}", ha="center", fontsize=8)
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=8)
        ax.set_ylabel(ylabel)
        ax.set_title(title_tag, fontsize=10)
        ax.grid(axis="y", alpha=0.3)

    path = f"{out}/S12_E3_obstacles.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    if verbose:
        print(f"  → {path}")
    return results


# ══════════════════════════════════════════════════════════════════════════════
# §8  E4 — SURFACE FRICTION SWEEP (analytical, extends S3/S5/S7)
# ══════════════════════════════════════════════════════════════════════════════

def run_E4(out=OUTPUT_DIR, verbose=True):
    """
    Sweep effective deceleration a ∈ [30, 120] cm/s² (surface friction proxy).
    Computes stop distance and watchdog safety margin across speed range.

    Friction surfaces:
      Polished concrete / linoleum : a ≈ 35 cm/s²
      Standard carpet (WRO mat)    : a ≈ 80 cm/s²  (paper baseline)
      Rubberised foam mat           : a ≈ 100 cm/s²
      High-grip anti-slip mat       : a ≈ 120 cm/s²
    """
    if verbose:
        print("\n" + "=" * 60)
        print("E4 – Surface Friction Sweep (analytical)")
        print("=" * 60)

    FRICTION_SURFACES = [
        ("Polished concrete", 35.0,  "#d62728"),
        ("WRO mat (paper)",   80.0,  "#1f77b4"),
        ("Rubberised foam",   100.0, "#2ca02c"),
        ("Anti-slip mat",     120.0, "#9467bd"),
    ]
    WATCHDOG_MS = 400.0
    speeds_cms  = np.linspace(5, 50, 300)
    decel_range = np.linspace(20, 130, 200)

    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    fig.suptitle(
        "E4 · Surface Friction Sweep — stop distance & safety margin\n"
        f"Watchdog threshold = {WATCHDOG_MS:.0f} ms, hard-stop = {HARD_STOP_CM} cm",
        fontsize=10
    )

    ax = axes[0]
    ax.set_title("(a) Braking Distance vs Speed", fontsize=9)
    for name, a, col in FRICTION_SURFACES:
        bd = speeds_cms ** 2 / (2 * a)
        ax.plot(speeds_cms, bd, color=col, lw=2, label=f"{name} (a={a:.0f} cm/s²)")
    ax.axvline(25, color="gray", ls=":", lw=1.5, label="Paper competition speed (25 cm/s)")
    ax.axhline(HARD_STOP_CM, color="black", ls="--", lw=1.5,
               label=f"Hard-stop limit ({HARD_STOP_CM} cm)")
    ax.set_xlabel("Speed (cm/s)");  ax.set_ylabel("Braking distance (cm)")
    ax.legend(fontsize=7);  ax.grid(alpha=0.3)

    ax = axes[1]
    ax.set_title("(b) Watchdog Stop Distance vs Speed", fontsize=9)
    for name, a, col in FRICTION_SURFACES:
        coast = speeds_cms * (WATCHDOG_MS / 1000)
        brake = speeds_cms ** 2 / (2 * a)
        total = coast + brake
        ax.plot(speeds_cms, total, color=col, lw=2, label=f"{name}")
        # Mark where safe limit crossed
        unsafe = np.where(total > HARD_STOP_CM)[0]
        if len(unsafe):
            v_thr = speeds_cms[unsafe[0]]
            ax.axvline(v_thr, color=col, ls=":", lw=1.0, alpha=0.7)
    ax.axhline(HARD_STOP_CM, color="black", ls="--", lw=1.5,
               label=f"Hard-stop = {HARD_STOP_CM} cm")
    ax.axvline(25, color="gray", ls=":", lw=1.5, label="Paper speed")
    ax.set_xlabel("Speed (cm/s)");  ax.set_ylabel("Total stop distance (cm)")
    ax.legend(fontsize=7);  ax.grid(alpha=0.3)

    ax = axes[2]
    ax.set_title("(c) Max Safe Speed vs Deceleration", fontsize=9)
    # Analytical: v*t_wd + v²/2a = D_safe → quadratic in v
    def max_v_safe(a_cms2):
        t  = WATCHDOG_MS / 1000.0
        # (1/2a)v² + t·v - D = 0
        disc = t**2 + 4*(1/(2*a_cms2))*HARD_STOP_CM
        return (-t + math.sqrt(disc)) / (2 * (1/(2*a_cms2)))

    v_safe = [max_v_safe(a) for a in decel_range]
    ax.plot(decel_range, v_safe, "b-", lw=2.5)
    for name, a, col in FRICTION_SURFACES:
        ax.axvline(a, color=col, ls="--", lw=1.5, label=f"{name} (a={a:.0f})")
        ax.scatter([a], [max_v_safe(a)], color=col, s=80, zorder=5)
    ax.axhline(25, color="gray", ls=":", lw=1.5, label="Paper speed (25 cm/s)")
    ax.set_xlabel("Deceleration (cm/s²)");  ax.set_ylabel("Max safe speed (cm/s)")
    ax.legend(fontsize=7);  ax.grid(alpha=0.3)

    if verbose:
        for name, a, _ in FRICTION_SURFACES:
            ms = max_v_safe(a)
            bd = 25.0 ** 2 / (2 * a)
            print(f"  {name:22s} a={a:5.0f}  max_safe={ms:.1f} cm/s  "
                  f"brake_dist@25={bd:.2f} cm")

    path = f"{out}/S12_E4_friction.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    if verbose:
        print(f"  → {path}")
    return dict(surfaces=FRICTION_SURFACES)


# ══════════════════════════════════════════════════════════════════════════════
# §9  E5 — EXHAUSTIVE SENSOR FAILURE (all C(6,1) + C(6,2) = 21 combinations)
# ══════════════════════════════════════════════════════════════════════════════

def run_E5(out=OUTPUT_DIR, verbose=True, steps_per=1500):
    """
    S4 tested 12 hand-picked configurations.
    E5 tests ALL 21 single/pair combinations systematically.
    """
    if verbose:
        print("\n" + "=" * 60)
        print(f"E5 – Exhaustive Sensor Failure ({steps_per} steps/config)")
        print("=" * 60)

    track = make_wro_rect()

    singles = [(i,) for i in range(6)]
    pairs   = list(itertools.combinations(range(6), 2))
    combos  = [()] + singles + list(pairs)   # baseline + 21 failures

    # 6×6 result matrices  (diagonal = single, off-diag = pair)
    cte_mat  = np.full((6, 6), np.nan)
    coll_mat = np.full((6, 6), np.nan)

    rows = []
    for combo in combos:
        r = simulate(track, failed_sensors=combo, steps=steps_per)
        name = "+".join(SENSOR_NAMES[i] for i in combo) if combo else "Baseline"
        rows.append(dict(combo=combo, name=name,
                         collisions=r["collisions"],
                         mean_cte=r["mean_cte"]))
        if verbose:
            print(f"  {name:20s} | coll={r['collisions']:4d} | CTE={r['mean_cte']:.1f} mm")

        if len(combo) == 1:
            i = combo[0]
            cte_mat[i, i]  = r["mean_cte"]
            coll_mat[i, i] = r["collisions"]
        elif len(combo) == 2:
            i, j = combo
            cte_mat[i, j] = cte_mat[j, i] = r["mean_cte"]
            coll_mat[i, j] = coll_mat[j, i] = r["collisions"]

    # ── Figure ────────────────────────────────────────────────────────────────
    snames = [SENSOR_NAMES[i] for i in range(6)]
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(
        f"E5 · Exhaustive Sensor Failure — all 21 single/pair HC-SR04 combinations\n"
        f"WRO track, {steps_per} steps. "
        f"Diagonal = single failure; off-diagonal = pair failure.",
        fontsize=10
    )

    for ax, mat, title, cmap_name in [
        (axes[0], cte_mat,  "(a) Mean CTE (mm)\n+ve = larger tracking error", "YlOrRd"),
        (axes[1], coll_mat, "(b) Collision Count\n+ve = more wall hits",       "YlOrRd"),
    ]:
        im = ax.imshow(mat, cmap=cmap_name, aspect="auto", vmin=0)
        plt.colorbar(im, ax=ax, shrink=0.85)
        ax.set_xticks(range(6));  ax.set_xticklabels(snames, fontsize=9)
        ax.set_yticks(range(6));  ax.set_yticklabels(snames, fontsize=9)
        ax.set_xlabel("Failed sensor B");  ax.set_ylabel("Failed sensor A")
        ax.set_title(title, fontsize=9)
        # Annotate values
        for i in range(6):
            for j in range(6):
                v = mat[i, j]
                if not np.isnan(v):
                    ax.text(j, i, f"{v:.0f}", ha="center", va="center",
                            fontsize=7, color="black" if v < np.nanmax(mat)*0.7 else "white")
        # Baseline reference line
        ax.set_title(ax.get_title() +
                     f"\n  (Baseline = {rows[0]['mean_cte'] if 'cte' in title.lower() else rows[0]['collisions']:.0f})",
                     fontsize=9)

    path = f"{out}/S12_E5_sensor_failure.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    if verbose:
        worst = max(rows[1:], key=lambda r: r["collisions"])
        print(f"  Worst combination: {worst['name']}  ({worst['collisions']} collisions)")
        print(f"  → {path}")
    return rows


# ══════════════════════════════════════════════════════════════════════════════
# §10  E6 — SPEED vs SAFETY ENVELOPE (analytical, extends S5/S9)
# ══════════════════════════════════════════════════════════════════════════════

def run_E6(out=OUTPUT_DIR, verbose=True):
    """
    Maps the maximum safe operating speed as a function of:
      (a) friction (deceleration coefficient)   × architecture
      (b) watchdog threshold                     × speed
    Extends S5 (single watchdog threshold) and S9 (single friction value).
    """
    if verbose:
        print("\n" + "=" * 60)
        print("E6 – Speed vs Safety Envelope (analytical)")
        print("=" * 60)

    WATCHDOG_MS    = 400.0
    DECEL_RANGE    = np.linspace(20, 130, 120)
    THRESHOLD_RANGE = np.linspace(100, 1000, 120)
    SPEEDS         = np.linspace(5, 55, 300)
    F_ARDUINO      = 50.0   # Hz

    def stop_single(v, tau_ms, a, n_sensors=6, n_med=3):
        T_block = n_sensors * n_med * 5.0   # ms  (5 ms per HC-SR04 read)
        f_act   = 1000.0 / (tau_ms + T_block)
        return v / f_act + v**2 / (2 * a)

    def stop_dual(v, watchdog_ms, a):
        return v * (watchdog_ms / 1000) + v**2 / (2 * a)

    PLATFORMS = [
        ("Pi 3B+ (this paper)", 90.9,  "#d62728", "*"),
        ("Pi 4B",               45.0,  "#ff7f0e", "s"),
        ("Pi 5",                28.0,  "#9467bd", "D"),
    ]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(
        "E6 · Speed vs Safety Envelope\n"
        "Maximum safe operating speed across architecture, friction, and watchdog threshold",
        fontsize=10
    )

    # Panel (a): max safe speed vs deceleration, by architecture
    ax = axes[0]
    ax.set_title("(a) Max Safe Speed vs Surface Friction\n"
                 f"Watchdog={WATCHDOG_MS:.0f} ms", fontsize=9)
    for name, tau, col, mk in PLATFORMS:
        v_max_single = []
        for a in DECEL_RANGE:
            # Binary search for max v s.t. stop_single(v, tau, a) ≤ 15
            lo, hi = 0.0, 100.0
            for _ in range(40):
                mid = (lo + hi) / 2
                (lo, hi) = (mid, hi) if stop_single(mid, tau, a) <= HARD_STOP_CM else (lo, mid)
            v_max_single.append(lo)
        ax.plot(DECEL_RANGE, v_max_single, color=col, ls="--", lw=1.8,
                label=f"{name} (single-chip)")
    # Dual-processor (same for all SBCs)
    v_max_dual = []
    for a in DECEL_RANGE:
        lo, hi = 0.0, 100.0
        for _ in range(40):
            mid = (lo + hi) / 2
            (lo, hi) = (mid, hi) if stop_dual(mid, WATCHDOG_MS, a) <= HARD_STOP_CM else (lo, mid)
        v_max_dual.append(lo)
    ax.plot(DECEL_RANGE, v_max_dual, "navy", lw=2.5, label="Dual-processor (50 Hz)")
    ax.axvline(80, color="gray", ls=":", lw=1.5, label="Paper a=80 cm/s²")
    ax.axhline(25, color="green", ls="-.", lw=1.5, label="Paper speed 25 cm/s")
    ax.set_xlabel("Deceleration a (cm/s²)")
    ax.set_ylabel("Max safe speed (cm/s)")
    ax.legend(fontsize=7);  ax.grid(alpha=0.3);  ax.set_ylim(0, 60)

    if verbose:
        print("  Max safe speeds at a=80 cm/s²:")
        for name, tau, col, mk in PLATFORMS:
            a = 80.0
            lo, hi = 0.0, 100.0
            for _ in range(40):
                mid = (lo + hi) / 2
                (lo, hi) = (mid, hi) if stop_single(mid, tau, a) <= HARD_STOP_CM else (lo, mid)
            print(f"    {name:22s} single={lo:.1f} cm/s")
        lo, hi = 0.0, 100.0
        for _ in range(40):
            mid = (lo + hi) / 2
            (lo, hi) = (mid, hi) if stop_dual(mid, 400.0, 80.0) <= HARD_STOP_CM else (lo, mid)
        print(f"    {'Dual-processor':22s} dual  ={lo:.1f} cm/s")

    # Panel (b): stop distance heatmap (watchdog × speed)
    ax = axes[1]
    ax.set_title("(b) Dual-Processor Stop Distance (cm)\n"
                 f"a={PAPER_DECEL:.0f} cm/s²", fontsize=9)
    V2D, TH2D = np.meshgrid(SPEEDS, THRESHOLD_RANGE)
    DIST2D     = V2D * (TH2D / 1000) + V2D**2 / (2 * PAPER_DECEL)
    cf = ax.contourf(V2D, TH2D, DIST2D,
                     levels=np.linspace(0, 30, 25), cmap="RdYlGn_r", extend="max")
    plt.colorbar(cf, ax=ax, label="Stop distance (cm)")
    cs = ax.contour(V2D, TH2D, DIST2D, levels=[HARD_STOP_CM],
                    colors="red", linewidths=2)
    ax.clabel(cs, fmt=f"Unsafe >{HARD_STOP_CM:.0f} cm", fontsize=8)
    ax.axvline(25,         color="white", ls="--", lw=1.5, label="Paper speed")
    ax.axhline(WATCHDOG_MS, color="white", ls=":",  lw=1.5, label="Paper watchdog")
    ax.scatter([25], [WATCHDOG_MS], s=120, color="white", zorder=5, marker="*")
    ax.set_xlabel("Vehicle speed (cm/s)")
    ax.set_ylabel("Watchdog threshold (ms)")
    ax.legend(fontsize=8)

    path = f"{out}/S12_E6_speed_envelope.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    if verbose:
        print(f"  → {path}")
    return {}


# ══════════════════════════════════════════════════════════════════════════════
# §11  E7 — OUTDOOR CONDITIONS
# ══════════════════════════════════════════════════════════════════════════════

def run_E7(out=OUTPUT_DIR, verbose=True, reps=3, steps_per=1500):
    """
    Five outdoor / adverse-environment scenarios modelled via perturbations to
    the baseline VP and Sensors noise parameters.

    Outdoor effects and their parameter mappings
    ─────────────────────────────────────────────
    Wind (lateral drift)       → increased heading noise (hn_std × scale)
    Ground vibration           → increased heading + position noise
    Thermal expansion / heat   → increased HC-SR04 noise (speed-of-sound drift)
    Dust / aerosol scattering  → severe HC-SR04 range noise
    Bright sunlight (optical)  → no direct mechanical effect; shown analytically
                                 as HSV F1 degradation (from S6 model)

    The HSV column shows the expected detection-rate ratio
    (outdoor / indoor) using S6's Gaussian noise model at σH=15° (strong glare).
    """
    if verbose:
        print("\n" + "=" * 60)
        print(f"E7 – Outdoor Conditions ({reps} reps × {steps_per} steps/scenario)")
        print("=" * 60)

    SCENARIOS = [
        # name                  hn_scale  sens_scale  speed_sc  hsv_ratio
        ("Indoor baseline",     1.0,      1.0,        1.0,      1.00),
        ("Light wind (~1 m/s)", 2.5,      1.0,        1.0,      1.00),
        ("Strong wind (~3 m/s)",6.0,      1.0,        1.0,      0.95),
        ("Ground vibration",    3.5,      2.0,        1.0,      0.98),
        ("Thermal noise (HC-SR04)", 1.0,  4.0,        1.0,      0.97),
        ("Dust / aerosol",      1.5,      7.0,        1.0,      0.90),
        ("Outdoor worst-case",  5.0,      6.0,        0.9,      0.72),
    ]

    track = make_wro_rect()
    rows  = []
    for name, hn_sc, sn_sc, sp_sc, hsv in SCENARIOS:
        r = _mean_result(track, reps=reps, steps=steps_per,
                         heading_noise_scale=hn_sc,
                         sensor_noise_scale=sn_sc,
                         speed_scale=sp_sc)
        rows.append(dict(name=name, hn_sc=hn_sc, sn_sc=sn_sc,
                         collisions=r["collisions"],
                         mean_cte=r["mean_cte"],
                         min_wall=r["min_wall_dist"],
                         hsv=hsv))
        if verbose:
            print(f"  {name:30s} | hn×{hn_sc:.1f} sn×{sn_sc:.1f} | "
                  f"coll={r['collisions']:.1f} | CTE={r['mean_cte']:.0f} mm")

    # ── Figure ────────────────────────────────────────────────────────────────
    labels = [r["name"] for r in rows]
    cols   = [r["collisions"] for r in rows]
    ctes   = [r["mean_cte"]   for r in rows]
    hsvs   = [r["hsv"]        for r in rows]
    x      = np.arange(len(rows))

    # Normalise to indoor baseline
    col0, cte0 = max(cols[0], 0.01), max(ctes[0], 0.01)
    cols_n = [c / col0 for c in cols]
    ctes_n = [c / cte0 for c in ctes]

    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    fig.suptitle(
        "E7 · Outdoor Environmental Conditions — Mechanical & Optical Degradation\n"
        f"WRO track, {reps} reps × {steps_per} steps",
        fontsize=10
    )

    ax = axes[0]
    bars = ax.bar(x, cols, color=plt.cm.RdYlGn_r(np.linspace(0.1, 0.9, len(rows))),
                  alpha=0.9)
    for b, v in zip(bars, cols):
        ax.text(b.get_x() + b.get_width()/2, v + 0.02, f"{v:.1f}",
                ha="center", fontsize=8)
    ax.set_xticks(x);  ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=8)
    ax.set_ylabel("Wall collisions per run")
    ax.set_title("(a) Collision Count", fontsize=10)
    ax.grid(axis="y", alpha=0.3)

    ax = axes[1]
    ax.bar(x - 0.22, ctes, 0.4, color="steelblue",  alpha=0.85, label="Mean CTE (mm)")
    rax = ax.twinx()
    rax.plot(x, [r["min_wall"] for r in rows], "r-o", lw=2, ms=6,
             label="Min wall clearance (mm)")
    rax.set_ylabel("Min wall clearance (mm)", color="red")
    ax.set_xticks(x);  ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=8)
    ax.set_ylabel("Mean CTE (mm)", color="steelblue")
    ax.set_title("(b) Tracking Quality", fontsize=10)
    lines1, l1 = ax.get_legend_handles_labels()
    lines2, l2 = rax.get_legend_handles_labels()
    ax.legend(lines1+lines2, l1+l2, fontsize=7)
    ax.grid(axis="y", alpha=0.3)

    ax = axes[2]
    bar_c = ax.bar(x - 0.22, ctes_n, 0.4,  color="steelblue",  alpha=0.75,
                   label="CTE (normalised)")
    bar_h = ax.bar(x + 0.22, hsvs,   0.4,  color="darkorange", alpha=0.75,
                   label="HSV detection rate (relative)")
    ax.axhline(1.0, color="black", lw=0.8, ls="--")
    ax.set_xticks(x);  ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=8)
    ax.set_ylabel("Value (normalised to indoor baseline)")
    ax.set_title("(c) Normalised Degradation: Mechanical + Optical", fontsize=10)
    ax.legend(fontsize=8);  ax.grid(axis="y", alpha=0.3)

    path = f"{out}/S12_E7_outdoor.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    if verbose:
        print(f"  → {path}")
    return rows


# ══════════════════════════════════════════════════════════════════════════════
# §12  MASTER RUNNER
# ══════════════════════════════════════════════════════════════════════════════

def run_S12(out=OUTPUT_DIR, verbose=True, quick=False):
    os.makedirs(out, exist_ok=True)
    t0 = time.time()

    steps_e = 500 if quick else STEPS
    reps_e  = 1   if quick else 3

    print("\n" + "▓" * 60)
    print("S12 – Extended Environment Simulation Suite")
    print("▓" * 60)
    if quick:
        print("  [QUICK mode: reduced steps and reps]")

    results = {}

    print("\n" + "─" * 60)
    results["E1"] = run_E1(out=out, verbose=verbose,
                            reps=reps_e)

    print("\n" + "─" * 60)
    results["E2"] = run_E2(out=out, verbose=verbose,
                            reps=reps_e)

    print("\n" + "─" * 60)
    results["E3"] = run_E3(out=out, verbose=verbose,
                            reps=reps_e)

    print("\n" + "─" * 60)
    results["E4"] = run_E4(out=out, verbose=verbose)

    print("\n" + "─" * 60)
    results["E5"] = run_E5(out=out, verbose=verbose,
                            steps_per=max(300, steps_e // 2))

    print("\n" + "─" * 60)
    results["E6"] = run_E6(out=out, verbose=verbose)

    print("\n" + "─" * 60)
    results["E7"] = run_E7(out=out, verbose=verbose,
                            reps=reps_e,
                            steps_per=max(300, steps_e // 2))

    elapsed = time.time() - t0
    print("\n" + "=" * 60)
    print("S12 COMPLETE")
    print("=" * 60)
    print(f"  Runtime : {elapsed:.1f}s")
    print("  Figures :")
    for f in sorted(os.listdir(out)):
        if f.startswith("S12_") and f.endswith(".png"):
            print(f"    {f}")
    return results


# ══════════════════════════════════════════════════════════════════════════════
# §13  CLI
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import sys
    quick = "--quick" in sys.argv
    random.seed(42);  np.random.seed(42)
    run_S12(out=OUTPUT_DIR, verbose=True, quick=quick)
