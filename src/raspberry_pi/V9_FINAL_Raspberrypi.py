#!/usr/bin/env python3
"""
WRO 2025 – Vision → Arduino High-Level Control (Obstacle Challenge)
====================================================================
Raspberry Pi 3B+ perception and high-level FSM.

Fixes applied vs original repo (V9_FINAL_Raspberrypi.py):
  [FIX-1] Normal speed hint corrected to 230 (= Arduino BASE_SPEED, paper §5.5).
           Previously sent 180, causing Arduino to always cap at 180 via
           min(scmd, sPi). Near-pillar hint remains 120 as specified.
  [FIX-2] Lap counter uses laps_seen >= 3 (not 4). The original >=4 counted
           the start-zone placement as lap 0 correctly, but the variable name
           and comments were misleading. Clarified with explicit guard-bit logic
           matching paper §4.5.3 / Eq.6.

Pipeline stages (paper §3.3 / Fig.2):
  1. Frame capture (picamera2, RGB888, 640×480)
  2. Channel-order auto-detect (18-frame calibration, paper §4.2)
  3. Preprocessing: Gaussian blur 5×5 + CLAHE on V channel (paper §4.3)
  4. HSV colour masking – 6 classes (paper Table 2, §4.4)
  5. Object inference: lane offset, pillar detection, lap counting (paper §4.5)
"""

import os
import sys
import time
import cv2
import numpy as np

# ─── Headless mode ────────────────────────────────────────────────────────────
HEADLESS = False
if HEADLESS:
    os.environ["QT_QPA_PLATFORM"] = "offscreen"

# ─── Serial ───────────────────────────────────────────────────────────────────
import serial
_last_sent = ""
_last_tx   = 0.0
BAUD       = 115200
ser        = None

# Command rate-limit: never faster than 80 ms (paper §3.2)
CMD_MIN_INTERVAL_MS = 0.080

def serial_open(auto_ports=None, retries=6, wait_between=1.0):
    import glob
    global ser
    if auto_ports is None:
        auto_ports = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
        if not auto_ports:
            auto_ports = ["/dev/ttyACM0", "/dev/ttyUSB0"]
    for attempt in range(1, retries + 1):
        for p in auto_ports:
            try:
                print(f"[Pi] Trying {p} @ {BAUD}…")
                s = serial.Serial(p, BAUD, timeout=0.02)
                time.sleep(2)
                ser = s
                print(f"[Pi] Connected to {p}")
                return p
            except Exception as e:
                print(f"[Pi] Port {p} failed: {e}")
        print(f"[Pi] Retry {attempt}/{retries} in {wait_between}s")
        time.sleep(wait_between)
    raise RuntimeError("[Pi] Could not open any serial port. Check cable/permissions.")

def wait_for_arduino_ready(max_wait=10.0):
    """Block until Arduino sends RDY (paper §3.2)."""
    print("[Pi] Waiting for Arduino RDY…")
    t0 = time.time()
    while time.time() - t0 < max_wait:
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except Exception:
            line = ""
        if line == "RDY":
            print("[Pi] Arduino RDY received")
            return
    print("[Pi] Warning: no RDY seen, continuing anyway")

def send_line(s: str, min_interval: float = CMD_MIN_INTERVAL_MS, dedupe: bool = True):
    """
    Send ASCII command to Arduino.
    Deduplication: suppress repeated identical commands within min_interval.
    (paper §3.2 – 80 ms rate limit and deduplication filter)
    """
    global _last_sent, _last_tx
    now = time.time()
    if dedupe and s == _last_sent and (now - _last_tx) < min_interval:
        return
    try:
        ser.write((s + "\n").encode())
    except Exception as e:
        print(f"[Pi] Serial write failed: {e}")
    _last_sent = s
    _last_tx   = now

# ─── GPIO (optional start button) ─────────────────────────────────────────────
HAS_GPIO  = False
START_PIN = 17  # BCM
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except Exception:
    pass

def wait_for_start_button():
    if not HAS_GPIO:
        print("[Pi] GPIO unavailable – press Enter to START…")
        try:
            input()
        except EOFError:
            pass
        return
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(START_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print("[Pi] Waiting for start button…")
    stable_ms = 0
    while True:
        if GPIO.input(START_PIN) == 0:
            stable_ms += 10
            if stable_ms >= 300:
                print("[Pi] START pressed!")
                break
        else:
            stable_ms = 0
        time.sleep(0.01)

# ─── Camera ───────────────────────────────────────────────────────────────────
try:
    from picamera2 import Picamera2
except Exception as e:
    print("[Pi] picamera2 not available:", e)
    raise

def camera_open():
    """
    Open Pi Camera in RGB888 layout.
    Lock AWB after warm-up to prevent hue drift during run (paper §4.1).
    """
    cam = Picamera2()
    cfg = cam.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    cam.configure(cfg)
    try:
        cam.set_controls({"AwbEnable": True, "AeEnable": True})
    except Exception:
        pass
    cam.start()
    time.sleep(1.5)  # thermal equilibrium: discard first 1.5 s of frames (paper §4.1)
    try:
        cam.set_controls({"AwbEnable": False})  # lock AWB
    except Exception:
        pass
    return cam

# ─── Vision settings ──────────────────────────────────────────────────────────
LINE_MODE     = "orange"   # or "blue" – set per track configuration
DEBUG_OVERLAY = True

# HSV colour bands (paper Table 2)
color_ranges = {
    "red1":    ((  0,  80,  70), ( 12, 255, 255)),
    "red2":    ((168,  80,  70), (179, 255, 255)),
    "green":   (( 40,  70,  70), ( 85, 255, 255)),
    "magenta": ((135,  80,  80), (170, 255, 255)),
    "orange":  ((  5, 120, 110), ( 22, 255, 255)),
    "blue":    (( 95, 110,  80), (135, 255, 255)),
    "black":   ((  0,   0,   0), (179,  80,  80)),
}

def preprocess_hsv(frame_bgr):
    """
    Paper §4.3:
    1. Gaussian blur 5×5 (suppress sensor noise)
    2. Convert to HSV
    3. CLAHE on V channel (tile 8×8, clip 2.0) – normalise local contrast
    """
    blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
    hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    clahe   = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    v       = clahe.apply(v)
    return cv2.merge([h, s, v])

def clean_mask(mask):
    """
    Morphological open (3×3 ellipse, 1 iter) then close (5×5 ellipse, 1 iter).
    Elliptical kernels match rounded pillar/lane silhouettes (paper §4.4).
    """
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k3, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5, iterations=1)
    return mask

def detect_colors(frame_bgr):
    """Compute six binary masks from preprocessed HSV frame (paper §4.4)."""
    hsv = preprocess_hsv(frame_bgr)
    # Red requires two hue-wrapping bands (paper §4.4, Table 2)
    r1  = cv2.inRange(hsv, color_ranges["red1"][0],  color_ranges["red1"][1])
    r2  = cv2.inRange(hsv, color_ranges["red2"][0],  color_ranges["red2"][1])
    masks = {"red": clean_mask(cv2.bitwise_or(r1, r2))}
    for c in ["green", "magenta", "orange", "blue", "black"]:
        m = cv2.inRange(hsv, color_ranges[c][0], color_ranges[c][1])
        masks[c] = clean_mask(m)
    areas = {k: int(cv2.countNonZero(v)) for k, v in masks.items()}
    return masks, areas

def centroid_from_mask(mask):
    """Return (cx, cy) of largest contour centroid, or None."""
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] <= 0:
        return None
    return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

def track_line(frame_bgr, masks, mode):
    """
    Lane-offset estimation (paper §4.5.1, Eq.5).
    Returns cx (pixel x of lane centroid). Falls back to frame centre if
    lane area < 800 px (no lane in view → drive straight).
    """
    key  = "orange" if mode != "blue" else "blue"
    mask = masks[key]
    h, w = frame_bgr.shape[:2]
    cx   = w // 2  # default: straight ahead

    if cv2.countNonZero(mask) > 800:
        cen = centroid_from_mask(mask)
        if cen:
            cx = cen[0]
    return cx

# ─── Channel-order auto-detect ────────────────────────────────────────────────
USE_SWAP_RGB2BGR = None   # None = undecided; True = swap; False = as-is
DECIDE_FRAMES    = 18     # calibration window (paper §4.2, Eq.4)

def _score_mapping(bgr_frame):
    """
    Higher score → mapping fits expected colours better.
    Rewards lane-line colour pixels; penalises opposite channel colour (paper Eq.4).
    """
    _, areas = detect_colors(bgr_frame)
    line_key = "orange" if LINE_MODE != "blue" else "blue"
    opp_key  = "blue"   if line_key == "orange" else "orange"
    return (2.5 * areas.get(line_key, 0)
            - 1.0 * areas.get(opp_key, 0)
            + 0.5 * (areas.get("red", 0) + areas.get("green", 0)))

# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    time.sleep(2)  # boot stabilisation

    # Serial
    try:
        serial_open()
    except RuntimeError as e:
        print(str(e))
        sys.exit(1)

    # Camera
    print("[Pi] Initialising camera…")
    cam = None
    for attempt in range(1, 4):
        try:
            cam = camera_open()
            break
        except Exception as e:
            print(f"[Pi] Camera attempt {attempt}/3 failed: {e}")
            time.sleep(0.5)
    if cam is None:
        print("[Pi] Camera could not be initialised. Exiting.")
        sys.exit(2)

    if not HEADLESS:
        cv2.namedWindow("WRO FE – Vision", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("WRO FE – Vision", 900, 600)

    # Wait for start
    wait_for_start_button()

    # Arduino handshake (paper §3.2)
    wait_for_arduino_ready()
    send_line("GO", dedupe=False)
    send_line("MODE RACE")

    # ── State ─────────────────────────────────────────────────────────────────
    laps_completed    = 0          # increments on rising edge of magenta zone
    magenta_seen_last = False      # previous-frame in-zone flag
    left_start_zone   = False      # guard: True after first zone exit (paper Eq.6)
    STATE             = "RACE"
    last_hb           = 0.0

    # Channel-order calibration accumulators
    ch_scores = {"as_is": 0.0, "swap": 0.0, "count": 0}

    try:
        while True:
            # ── Frame capture ────────────────────────────────────────────────
            try:
                frame = cam.capture_array("main")
            except Exception as e:
                print(f"[Pi] Camera read failed: {e}; reinitialising…")
                try:
                    cam.stop()
                except Exception:
                    pass
                cam = camera_open()
                continue

            if frame is None or frame.size == 0:
                time.sleep(0.02)
                continue

            # ── Channel-order detection + locking (paper §4.2, Eq.4) ────────
            global USE_SWAP_RGB2BGR
            if USE_SWAP_RGB2BGR is None:
                bgr_as_is   = frame
                bgr_swapped = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                s_as_is     = _score_mapping(bgr_as_is)
                s_swapped   = _score_mapping(bgr_swapped)
                ch_scores["as_is"] += s_as_is
                ch_scores["swap"]  += s_swapped
                ch_scores["count"] += 1
                frame_bgr = bgr_swapped if s_swapped > s_as_is else bgr_as_is
                if ch_scores["count"] >= DECIDE_FRAMES:
                    USE_SWAP_RGB2BGR = ch_scores["swap"] > ch_scores["as_is"]
                    winner = "RGB→BGR swap" if USE_SWAP_RGB2BGR else "as-is"
                    ratio  = (max(ch_scores["swap"], ch_scores["as_is"]) /
                              max(1, min(ch_scores["swap"], ch_scores["as_is"])))
                    print(f"[Pi] Channel mapping locked: {winner} "
                          f"(score ratio {ratio:.1f}×, {DECIDE_FRAMES} frames)")
            else:
                frame_bgr = (cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                             if USE_SWAP_RGB2BGR else frame)

            masks, areas = detect_colors(frame_bgr)
            h, w = frame_bgr.shape[:2]
            centre_x = w // 2

            hud = []
            A_frame = w * h
            min_area  = int(0.002 * A_frame)   # Amin: pillar gate (paper §4.5.2)
            near_area = int(0.004 * A_frame)   # Anear: close-pillar speed reduction
            start_area = int(0.0035 * A_frame) # Astart: magenta threshold (paper §4.5.3)

            # ── Pillar detection (paper §4.5.2) ──────────────────────────────
            if areas["red"] > min_area:
                send_line("KEEP RIGHT")
                hud.append("RED → KEEP RIGHT")
            if areas["green"] > min_area:
                send_line("KEEP LEFT")
                hud.append("GREEN → KEEP LEFT")

            # ── Lap counting (paper §4.5.3, Eq.6) ────────────────────────────
            in_start = areas["magenta"] > start_area
            if STATE == "RACE":
                # Rising edge: outside→inside, and previously left zone at least once
                if in_start and not magenta_seen_last and left_start_zone:
                    laps_completed += 1
                    hud.append(f"LAP {laps_completed}")
                    if laps_completed >= 3:
                        STATE = "PARK"
                        send_line("MODE PARK", dedupe=False)
                        hud.append("MODE → PARK")

                # Mark that vehicle has left the start zone at least once
                if not in_start and magenta_seen_last:
                    left_start_zone = True

            magenta_seen_last = in_start

            # ── Lane-offset estimation (paper §4.5.1, Eq.5) ──────────────────
            cx       = track_line(frame_bgr, masks, LINE_MODE)
            offset_px = cx - centre_x
            offset    = float(np.clip(offset_px / (w / 2.0), -1.0, 1.0))

            # ── Speed hint (paper §5.5) ───────────────────────────────────────
            # [FIX-1] Normal speed = BASE_SPEED = 230 (was 180 in original code).
            # Near pillar → 120 (paper §4.5.2: "speed hint s reduced to 120 PWM").
            near_pillar = (areas["red"] > near_area) or (areas["green"] > near_area)
            speed = 120 if near_pillar else 230

            # ── Serial dispatch ───────────────────────────────────────────────
            if STATE == "RACE":
                send_line(f"DRIVE {offset:.3f} {int(speed)}")
            else:
                # Park mode: stop sending drive; keep heartbeat alive so watchdog
                # does not fire (paper §3.2 – Arduino takes full control in PARK)
                send_line("DRIVE 0.000 0")
                now = time.time()
                if now - last_hb > 0.2:
                    send_line("HB", dedupe=False)
                    last_hb = now

            # ── Preview ───────────────────────────────────────────────────────
            if not HEADLESS and DEBUG_OVERLAY:
                cv2.putText(frame_bgr, f"STATE: {STATE}  LAPS: {laps_completed}",
                            (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame_bgr, f"offset: {offset:+.3f}  speed: {speed}",
                            (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                y = 66
                for line in hud[:4]:
                    cv2.putText(frame_bgr, line, (10, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                    y += 22
                cv2.imshow("WRO FE – Vision", frame_bgr)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\n[Pi] Ctrl-C – shutting down…")

    finally:
        try:
            send_line("STOP", dedupe=False)
        except Exception:
            pass
        try:
            cam.stop()
        except Exception:
            pass
        if ser:
            try:
                ser.close()
            except Exception:
                pass
        if HAS_GPIO:
            GPIO.cleanup()
        if not HEADLESS:
            cv2.destroyAllWindows()
        print("[Pi] Clean exit.")

if __name__ == "__main__":
    main()
