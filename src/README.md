#!/usr/bin/env python3
# -- coding: utf-8 --
"""
WRO Future Engineers – Obstacle Challenge (Consolidated, Vision-Verified Parking)
•⁠  ⁠Raspberry Pi 3B+ friendly (defaults 320x240)
•⁠  ⁠Modes:
    sim         : synthetic simulator
    cam         : camera-driven (use --dry-run for no PWM)
    hsv_tuner   : live HSV threshold tuner
•⁠  ⁠Features:
    * Lane centering via PD (vision)
    * Pass RED on right, GREEN on left (vision)
    * 3-lap logic; last sign on lap 2 picks lap 3 direction
    * Magenta parking bay detection + VISION-VERIFIED parallel park
"""

import os, sys, time, math, argparse
from dataclasses import dataclass, field
from collections import deque
import numpy as np
import cv2

# ---- Optional PWM (PCA9685); auto fall back to dry-run if not present ----
HW_AVAILABLE = False
try:
    from adafruit_servokit import ServoKit
    HW_AVAILABLE = True
except Exception:
    HW_AVAILABLE = False

# ======================= Configs ==========================
@dataclass
class VisionConfig:
    frame_w: int = 320
    frame_h: int = 240
    roi_y0: int = 120
    roi_y1: int = 230
    # Default HSV ranges (tune with hsv_tuner)
    white_low: tuple = (0, 0, 190)
    white_high: tuple = (179, 40, 255)
    red1_low: tuple = (0, 90, 90)
    red1_high: tuple = (10, 255, 255)
    red2_low: tuple = (170, 90, 90)
    red2_high: tuple = (179, 255, 255)
    green_low: tuple = (40, 80, 80)
    green_high: tuple = (85, 255, 255)
    magenta_low: tuple = (130, 80, 80)
    magenta_high: tuple = (165, 255, 255)
    kernel: np.ndarray = field(default_factory=lambda: np.ones((3, 3), np.uint8))

@dataclass
class ControlConfig:
    # PD
    kp: float = 0.018
    kd: float = 0.006
    steer_limit: float = 0.8
    # Speeds (normalized -1..+1)
    base_speed: float = 0.33
    slow_speed: float = 0.24
    # Lap & parking
    lap_length_m: float = 22.0
    goal_laps: int = 3
    park_detect_min_width_px: int = 30

@dataclass
class HardwareConfig:
    channels: dict = field(default_factory=lambda: {"steer": 0, "throttle": 1})
    servo_freq: int = 50
    steer_us_min: int = 1000
    steer_us_mid: int = 1500
    steer_us_max: int = 2000
    esc_us_min: int = 1000
    esc_us_stop: int = 1500
    esc_us_max: int = 2000

# ==================== Hardware I/O ========================
class Car:
    def _init_(self, hw: HardwareConfig, dry_run: bool = True):
        self.hw = hw
        self.dry_run = dry_run or not HW_AVAILABLE
        self._last_throttle = 0.0
        self.kit = None
        if not self.dry_run and HW_AVAILABLE:
            try:
                self.kit = ServoKit(channels=16)
                self.kit.frequency = hw.servo_freq
            except Exception as e:
                print("[WARN] PWM init failed, falling back to dry-run:", e, file=sys.stderr)
                self.dry_run = True

    def _norm_to_us(self, val_norm: float, us_min: int, us_mid: int, us_max: int) -> int:
        v = max(-1.0, min(1.0, val_norm))
        return int(us_mid + (us_max - us_mid) * v) if v >= 0 else int(us_mid + (us_mid - us_min) * v)

    def steer(self, val_norm: float):
        pulse = self._norm_to_us(val_norm, self.hw.steer_us_min, self.hw.steer_us_mid, self.hw.steer_us_max)
        if self.dry_run or not self.kit:
            print(f"[STEER] {val_norm:+.2f}  ({pulse}us)")
            return
        self.kit.servo[self.hw.channels["steer"]].set_pulse_width_range(self.hw.steer_us_min, self.hw.steer_us_max)
        self.kit.servo[self.hw.channels["steer"]].fraction = (pulse - self.hw.steer_us_min) / (self.hw.steer_us_max - self.hw.steer_us_min)

    def throttle(self, val_norm: float):
        v = max(-1.0, min(1.0, val_norm))
        alpha = 0.2
        smoothed = alpha * v + (1 - alpha) * self._last_throttle
        self._last_throttle = smoothed
        pulse = self._norm_to_us(smoothed, self.hw.esc_us_min, self.hw.esc_us_stop, self.hw.esc_us_max)
        if self.dry_run or not self.kit:
            print(f"[THROT] {smoothed:+.2f} ({pulse}us)")
            return
        self.kit.servo[self.hw.channels["throttle"]].set_pulse_width_range(self.hw.esc_us_min, self.hw.esc_us_max)
        self.kit.servo[self.hw.channels["throttle"]].fraction = (pulse - self.hw.esc_us_min) / (self.hw.esc_us_max - self.hw.esc_us_min)

    def stop(self):
        self.throttle(0.0)

# =================== Perception ===========================
class Vision:
    def _init_(self, cfg: VisionConfig):
        self.cfg = cfg

    def _mask(self, hsv, low, high):
        return cv2.inRange(hsv, np.array(low), np.array(high))

    def _mask_red(self, hsv):
        return cv2.bitwise_or(
            self._mask(hsv, self.cfg.red1_low, self.cfg.red1_high),
            self._mask(hsv, self.cfg.red2_low, self.cfg.red2_high)
        )

    def process(self, bgr, debug=False):
        h, w = bgr.shape[:2]
        y0, y1 = self.cfg.roi_y0, self.cfg.roi_y1
        roi = bgr[y0:y1, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        white = self._mask(hsv, self.cfg.white_low, self.cfg.white_high)
        white = cv2.morphologyEx(white, cv2.MORPH_OPEN, self.cfg.kernel)
        M = cv2.moments(white)
        cx = (M['m10'] / M['m00']) if M['m00'] != 0 else w / 2
        lane_err = cx - (w / 2)
        lane_q = 1.0 if M['m00'] > 0 else 0.0

        red = self._mask_red(hsv)
        green = self._mask(hsv, self.cfg.green_low, self.cfg.green_high)
        magenta = self._mask(hsv, self.cfg.magenta_low, self.cfg.magenta_high)

        dets = []
        for color, mask in [('red', red), ('green', green), ('magenta', magenta)]:
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                x, y, w2, h2 = cv2.boundingRect(c)
                area = cv2.contourArea(c)
                if area > 180 and h2 > 16:
                    dets.append({'color': color, 'bbox': (x, y + y0, w2, h2), 'area': float(area)})

        dbg = None
        if debug:
            dbg = bgr.copy()
            cv2.rectangle(dbg, (0, y0), (w, y1), (255, 255, 0), 1)
            for d in dets:
                x, y, w2, h2 = d['bbox']
                col = (0,0,255) if d['color']=='red' else ((0,255,0) if d['color']=='green' else (255,0,255))
                cv2.rectangle(dbg, (x, y), (x+w2, y+h2), col, 2)
                cv2.putText(dbg, d['color'], (x, y-2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
            cv2.line(dbg, (int(cx), y0), (int(cx), y1), (255, 255, 255), 1)
            cv2.line(dbg, (w//2, y0), (w//2, y1), (128, 128, 128), 1)

        return lane_err, lane_q, dets, dbg

# =================== Behavior / Logic =====================
class ObstacleLogic:
    def _init_(self, cfg: ControlConfig):
        self.cfg = cfg
        self.last_err = 0.0
        self.dir = 1              # +1 or -1 (turn-around flips this)
        self.lap_distance_m = 0.0
        self.lap_count = 0
        self.last_seen_sign = None
        self.state = 'LANE_FOLLOW'  # PASSING, TURN_AROUND, PARK_SEARCH, PARK_EXEC
        self.pass_timer = 0.0
        self.park_timer = 0.0
        self._noenc_scale_mps = 0.55   # distance estimate when no encoders
        self.sign_buf = deque(maxlen=6)

    def update_distance(self, dt, throttle_norm):
        v = abs(throttle_norm) * self._noenc_scale_mps
        self.lap_distance_m += v * dt
        if self.lap_distance_m >= self.cfg.lap_length_m:
            self.lap_distance_m = 0.0
            self.lap_count += 1
            print(f"[LAP] Completed lap {self.lap_count}")

    def pd_steer(self, err_px):
        de = err_px - self.last_err
        self.last_err = err_px
        u = self.cfg.kp * err_px + self.cfg.kd * de
        return max(-self.cfg.steer_limit, min(self.cfg.steer_limit, u))

    def stable_sign(self, dets):
        pillars = [d for d in dets if d['color'] in ('red', 'green')]
        if not pillars:
            self.sign_buf.append(None)
            return None
        p = max(pillars, key=lambda d: d['area'])
        self.sign_buf.append(p['color'])
        if len(self.sign_buf) >= 5 and len({c for c in self.sign_buf if c}) == 1:
            return p['color']
        return None

    def pass_offset_px(self, color):
        if color == 'red':   return +90    # pass on right: bias right
        if color == 'green': return -90    # pass on left: bias left
        return 0

# =================== Parking helpers (VISION-VERIFIED) ====
def find_magenta_pair(dets, min_w=30, max_y_diff=20):
    """
    From detections, return the best pair of magenta bars:
    returns ( (xL,yL,wL,hL), (xR,yR,wR,hR) ) with xL < xR  OR  None
    """
    mags = [d['bbox'] for d in dets if d['color']=='magenta']
    best = None
    best_gap = -1
    for i in range(len(mags)):
        x1,y1,w1,h1 = mags[i]
        if w1 < min_w: 
            continue
        for j in range(i+1, len(mags)):
            x2,y2,w2,h2 = mags[j]
            if w2 < min_w: 
                continue
            if abs(y1 - y2) > max_y_diff:
                continue
            (L, R) = ((x1,y1,w1,h1), (x2,y2,w2,h2)) if x1 < x2 else ((x2,y2,w2,h2), (x1,y1,w1,h1))
            gap = R[0] - (L[0] + L[2])  # space between inner edges
            if gap > best_gap:
                best_gap = gap
                best = (L, R)
    return best

def parking_ok(pair, frame_w, center_margin_px=22, min_gap_px=55):
    """
    Visual condition for 'we're inside the bay':
    - gap between inner edges >= min_gap_px
    - camera center is between bars with some margin
    """
    if not pair:
        return False
    (xL,yL,wL,hL), (xR,yR,wR,hR) = pair
    inner_left  = xL + wL
    inner_right = xR
    gap = inner_right - inner_left
    cx = frame_w // 2
    centered = (inner_left + center_margin_px) <= cx <= (inner_right - center_margin_px)
    wide_enough = gap >= min_gap_px
    return centered and wide_enough

# =================== Camera/Drive Mode ====================
def run_camera(args):
    vcfg = VisionConfig(frame_w=args.width, frame_h=args.height)
    ccfg = ControlConfig()
    hcfg = HardwareConfig()
    vis = Vision(vcfg)
    car = Car(hcfg, dry_run=args.dry_run)
    logic = ObstacleLogic(ccfg)

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, vcfg.frame_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, vcfg.frame_h)
    time.sleep(1.2)
    t_prev = time.time()

    print("[INFO] Camera mode started", "(DRY-RUN)" if car.dry_run else "(PWM ACTIVE!)")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[ERR] Camera frame failed"); break

            lane_err, lane_q, dets, dbg = vis.process(frame, debug=True)
            now = time.time(); dt = now - t_prev; t_prev = now

            sign = logic.stable_sign(dets) or logic.last_seen_sign
            if sign: logic.last_seen_sign = sign

            if logic.state == 'LANE_FOLLOW':
                steer = logic.pd_steer(lane_err * logic.dir)
                car.steer(steer)
                speed = ccfg.base_speed if lane_q > 0.04 else ccfg.slow_speed
                car.throttle(speed * logic.dir)
                logic.update_distance(dt, speed)

                if sign:
                    logic.state = 'PASSING'; logic.pass_timer = 0.0

                if logic.lap_count == 2 and logic.dir == 1:
                    if logic.last_seen_sign == 'red':
                        print("[TURN] Lap3 opposite (red at lap 2).")
                        logic.state = 'TURN_AROUND'
                    else:
                        print("[TURN] Lap3 same direction.")

                if logic.lap_count >= ccfg.goal_laps:
                    print("[PARK] Searching for bay...")
                    logic.state = 'PARK_SEARCH'

            elif logic.state == 'PASSING':
                logic.pass_timer += dt
                offset = logic.pass_offset_px(logic.last_seen_sign)
                steer = logic.pd_steer((lane_err - offset) * logic.dir)
                car.steer(steer)
                car.throttle(ccfg.slow_speed * logic.dir)
                logic.update_distance(dt, ccfg.slow_speed)
                if logic.pass_timer > 0.9:
                    logic.state = 'LANE_FOLLOW'

            elif logic.state == 'TURN_AROUND':
                # time-based 3-point (tune as needed)
                car.throttle(-0.22); car.steer(+0.8); time.sleep(0.7)
                car.throttle(+0.25); time.sleep(0.85)
                car.stop()
                logic.dir = -1
                logic.state = 'LANE_FOLLOW'
                logic.last_err = 0.0

            elif logic.state == 'PARK_SEARCH':
                # Creep & search for a consistent pair of magenta bars
                steer = logic.pd_steer(lane_err * logic.dir)
                car.steer(steer)
                car.throttle(ccfg.slow_speed * logic.dir)

                pair = find_magenta_pair(dets, min_w=ccfg.park_detect_min_width_px, max_y_diff=20)
                if pair:
                    print("[PARK] Bay detected → executing park (vision-guided).")
                    car.stop(); time.sleep(0.12)
                    logic.state = 'PARK_EXEC'
                    logic.park_timer = 0.0
                    logic._ok_stable_time = 0.0

            elif logic.state == 'PARK_EXEC':
                # Vision-guided reverse into bay, only stop when camera confirms we're centered
                logic.park_timer += dt
                pair = find_magenta_pair(dets, min_w=ccfg.park_detect_min_width_px, max_y_diff=20)
                frame_w = frame.shape[1]
                ok_now = parking_ok(pair, frame_w, center_margin_px=22, min_gap_px=55) if pair else False

                if not ok_now:
                    # 2-phase: short pull-forward to align, then reverse in
                    if logic.park_timer < 0.55:
                        car.steer(0.0); car.throttle(0.16 * logic.dir)
                    else:
                        car.steer(-0.65 * logic.dir)
                        car.throttle(-0.20 * logic.dir)
                    logic._ok_stable_time = 0.0
                else:
                    # Inside bay by vision; confirm stability before stopping
                    logic._ok_stable_time += dt
                    car.steer(0.0)
                    car.throttle(-0.08 * logic.dir)   # creep while confirming
                    if logic._ok_stable_time >= 0.5:
                        car.stop()
                        print("[DONE] Vision confirms parking (centered between magenta bars).")
                        break

                if logic.park_timer > 6.0:
                    car.stop()
                    print("[WARN] Parking timed out → stopping for safety.")
                    break

            # HUD / visualization
            if dbg is not None:
                cv2.putText(dbg, f"state:{logic.state} lap:{logic.lap_count} dir:{logic.dir:+d} sign:{logic.last_seen_sign}",
                            (5, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,255), 1, cv2.LINE_AA)
                cv2.putText(dbg, f"err:{lane_err:+.1f}px q:{lane_q:.2f}",
                            (5, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,255), 1, cv2.LINE_AA)
                # Draw inner edges of chosen pair for judge visibility
                pair_dbg = find_magenta_pair(dets, min_w=ccfg.park_detect_min_width_px, max_y_diff=20)
                if pair_dbg:
                    vcfg = VisionConfig()  # for ROI limits
                    (xL,yL,wL,hL), (xR,yR,wR,hR) = pair_dbg
                    inner_left  = xL + wL
                    inner_right = xR
                    cv2.line(dbg, (inner_left, vcfg.roi_y0), (inner_left, vcfg.roi_y1), (255,0,255), 1)
                    cv2.line(dbg, (inner_right, vcfg.roi_y0), (inner_right, vcfg.roi_y1), (255,0,255), 1)

                cv2.imshow("WRO FE (camera)", dbg)
                if cv2.waitKey(1) & 0xFF == 27: break

    except KeyboardInterrupt:
        pass
    finally:
        car.stop(); cap.release(); cv2.destroyAllWindows()

# =================== Simulator Mode =======================
def draw_scene(w, h, pillar, bay):
    img = np.zeros((h, w, 3), np.uint8)
    img[:] = (50, 50, 50)
    # lane lines
    cv2.line(img, (w//2-55, 0), (w//2-55, h), (255,255,255), 3)
    cv2.line(img, (w//2+55, 0), (w//2+55, h), (255,255,255), 3)
    # pillar
    if pillar['on']:
        x, y, cw, ch = pillar['x'], pillar['y'], 18, 40
        color = (0,0,255) if pillar['color']=='red' else (0,255,0)
        cv2.rectangle(img, (x,y), (x+cw,y+ch), color, -1)
    # parking bay (two magenta bars)
    if bay['on']:
        x, y, w2, h2 = bay['x'], bay['y'], 40, 8
        cv2.rectangle(img, (x,y), (x+w2,y+h2), (255,0,255), -1)
        cv2.rectangle(img, (x+70,y), (x+70+w2,y+h2), (255,0,255), -1)
    return img

def run_sim(args):
    vcfg = VisionConfig()
    ccfg = ControlConfig()
    vis = Vision(vcfg)
    logic = ObstacleLogic(ccfg)

    pillar = {'on': False, 'color': 'red', 'x': 140, 'y': 130}
    bay    = {'on': False, 'x': 110, 'y': 150}

    t_prev = time.time()
    while True:
        frame = draw_scene(vcfg.frame_w, vcfg.frame_h, pillar, bay)
        lane_err, lane_q, dets, dbg = vis.process(frame, debug=True)
        now = time.time(); dt = now - t_prev; t_prev = now

        sign = logic.stable_sign(dets) or logic.last_seen_sign
        if sign: logic.last_seen_sign = sign

        if logic.state == 'LANE_FOLLOW':
            # emulate distance accumulation
            speed = ccfg.base_speed if lane_q > 0.05 else ccfg.slow_speed
            logic.update_distance(dt, speed)
            if sign:
                logic.state = 'PASSING'; logic.pass_timer = 0.0
            if logic.lap_count == 2 and logic.dir == 1:
                logic.state = 'TURN_AROUND' if logic.last_seen_sign == 'red' else 'LANE_FOLLOW'
            if logic.lap_count >= ccfg.goal_laps:
                logic.state = 'PARK_SEARCH'

        elif logic.state == 'PASSING':
            logic.pass_timer += dt
            if logic.pass_timer > 0.9: logic.state = 'LANE_FOLLOW'

        elif logic.state == 'TURN_AROUND':
            logic.dir = -1; logic.state = 'LANE_FOLLOW'; logic.last_err = 0.0

        elif logic.state == 'PARK_SEARCH':
            pair = find_magenta_pair(dets, min_w=ccfg.park_detect_min_width_px, max_y_diff=20)
            if pair:
                logic.state = 'PARK_EXEC'
                logic.park_timer = 0.0
                logic._ok_stable_time = 0.0

        elif logic.state == 'PARK_EXEC':
            logic.park_timer += dt
            pair = find_magenta_pair(dets, min_w=ccfg.park_detect_min_width_px, max_y_diff=20)
            ok_now = parking_ok(pair, vcfg.frame_w, center_margin_px=22, min_gap_px=55)
            if ok_now:
                logic._ok_stable_time += dt
                if logic._ok_stable_time >= 0.5:
                    # parked (we keep sim running for play; you can break here)
                    logic.state = 'LANE_FOLLOW'
            else:
                logic._ok_stable_time = 0.0
            if logic.park_timer > 6.0:
                logic.state = 'LANE_FOLLOW'

        # HUD
        cv2.putText(dbg, f"state:{logic.state} lap:{logic.lap_count} dir:{logic.dir:+d} sign:{logic.last_seen_sign}",
                    (5, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,255), 1, cv2.LINE_AA)
        pair_dbg = find_magenta_pair(dets, min_w=ccfg.park_detect_min_width_px, max_y_diff=20)
        if pair_dbg:
            (xL,yL,wL,hL), (xR,yR,wR,hR) = pair_dbg
            inner_left  = xL + wL
            inner_right = xR
            cv2.line(dbg, (inner_left, vcfg.roi_y0), (inner_left, vcfg.roi_y1), (255,0,255), 1)
            cv2.line(dbg, (inner_right, vcfg.roi_y0), (inner_right, vcfg.roi_y1), (255,0,255), 1)

        cv2.imshow("WRO FE Simulator", dbg)
        k = cv2.waitKey(16) & 0xFF
        if k==27: break
        elif k==ord('r'): pillar['on']=not pillar['on']; pillar['color']='red'
        elif k==ord('g'): pillar['on']=not pillar['on']; pillar['color']='green'
        elif k==ord('p'): bay['on']=not bay['on']
        elif k==ord('a'): pillar['x']=max(10, pillar['x']-8); bay['x']=max(10, bay['x']-8)
        elif k==ord('d'): pillar['x']=min(vcfg.frame_w-20, pillar['x']+8); bay['x']=min(vcfg.frame_w-120, bay['x']+8)
        elif k==ord('w'): pillar['y']=max(40, pillar['y']-6); bay['y']=max(60,bay['y']-6)
        elif k==ord('s'): pillar['y']=min(vcfg.frame_h-60, pillar['y']+6); bay['y']=min(vcfg.frame_h-30,bay['y']+6)
        elif k==ord(' '): pillar['on']=False; bay['on']=False

    cv2.destroyAllWindows()

# =================== HSV Tuner ============================
def nothing(_): pass
def run_hsv_tuner(args):
    cam = cv2.VideoCapture(args.camera)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    time.sleep(0.8)

    cv2.namedWindow("HSV Tuner")
    for name, init in [("Hlo",0),("Hhi",179),("Slo",0),("Shi",255),("Vlo",0),("Vhi",255)]:
        cv2.createTrackbar(name, "HSV Tuner", init, 255 if name[0] in "SV" else 179, nothing)

    print("[INFO] Use tuner to find HSV for WHITE/RED/GREEN/MAGENTA. Press 1/2/3/4 to switch preview mask.")
    mode = 1  # 1=white, 2=red, 3=green, 4=magenta

    while True:
        ok, bgr = cam.read()
        if not ok: break
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        Hlo = cv2.getTrackbarPos("Hlo","HSV Tuner")
        Hhi = cv2.getTrackbarPos("Hhi","HSV Tuner")
        Slo = cv2.getTrackbarPos("Slo","HSV Tuner")
        Shi = cv2.getTrackbarPos("Shi","HSV Tuner")
        Vlo = cv2.getTrackbarPos("Vlo","HSV Tuner")
        Vhi = cv2.getTrackbarPos("Vhi","HSV Tuner")

        if mode==2:  # red wraps hue
            m1 = cv2.inRange(hsv, (0, Slo, Vlo), (min(Hhi,10), Shi, Vhi))
            m2 = cv2.inRange(hsv, (170, Slo, Vlo), (179, Shi, Vhi))
            mask = cv2.bitwise_or(m1, m2)
        else:
            mask = cv2.inRange(hsv, (Hlo,Slo,Vlo), (Hhi,Shi,Vhi))

        vis = cv2.bitwise_and(bgr, bgr, mask=mask)
        cv2.putText(vis, f"Mode {mode} (1=white, 2=red, 3=green, 4=magenta)",
                    (6,18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255),1, cv2.LINE_AA)
        cv2.imshow("HSV Tuner", vis)
        k = cv2.waitKey(1) & 0xFF
        if k==27: break
        elif k in (ord('1'),ord('2'),ord('3'),ord('4')): mode = int(chr(k))

    cam.release(); cv2.destroyAllWindows()

# =================== Entry Point ==========================
def main():
    ap = argparse.ArgumentParser(description="WRO FE consolidated tool (vision-verified parking)")
    sub = ap.add_subparsers(dest="mode", required=True)

    sub.add_parser("sim", help="Run synthetic simulator")

    ap_cam = sub.add_parser("cam", help="Run camera/drive mode")
    ap_cam.add_argument("--dry-run", action="store_true", help="Don't send PWM; print commands")
    ap_cam.add_argument("--camera", type=int, default=0)
    ap_cam.add_argument("--width", type=int, default=320)
    ap_cam.add_argument("--height", type=int, default=240)

    ap_hsv = sub.add_parser("hsv_tuner", help="Interactive HSV threshold tuner")
    ap_hsv.add_argument("--camera", type=int, default=0)

    args = ap.parse_args()
    if args.mode == "sim":        run_sim(args)
    elif args.mode == "cam":      run_camera(args)
    elif args.mode == "hsv_tuner":run_hsv_tuner(args)

