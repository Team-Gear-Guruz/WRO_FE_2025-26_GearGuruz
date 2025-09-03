
#!/usr/bin/env python3
"""
WRO 2025 – Vision → Arduino High-Level Control (Obstacle Challenge)
"""

import os
import sys
import time
import cv2
import numpy as np

# ---------------------- Preview / display mode ---------------------------------
HEADLESS = False
if HEADLESS:
    os.environ["QT_QPA_PLATFORM"] = "offscreen"

# ------------------------ Serial ------------------------------------------------
import serial
_last_sent = ""
_last_tx = 0.0
BAUD = 115200
ser = None

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
                print(f"[Pi] Trying serial port {p} @ {BAUD}…")
                s = serial.Serial(p, BAUD, timeout=0.02)
                time.sleep(2)  # let Arduino reset
                ser = s
                print(f"[Pi] Connected to {p}")
                return p
            except Exception as e:
                print(f"[Pi] Port {p} failed: {e}")
        print(f"[Pi] Retry {attempt}/{retries}… waiting {wait_between}s")
        time.sleep(wait_between)
    raise RuntimeError("[Pi] Could not open any serial port (ACM/USB). Check cable/permissions (dialout).")

def wait_for_arduino_ready(max_wait=10.0):
    print("[Pi] Waiting for Arduino RDY…")
    t0 = time.time()
    while time.time() - t0 < max_wait:
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except Exception:
            line = ""
        if line == "RDY":
            print("[Pi] Arduino RDY")
            return
    print("[Pi] No RDY seen, continuing…")

def send_line(s: str, min_interval=0.08, dedupe=True):
    global _last_sent, _last_tx
    now = time.time()
    if dedupe and s == _last_sent and (now - _last_tx) < min_interval:
        return
    try:
        ser.write((s + "\n").encode())
    except Exception as e:
        print(f"[Pi] Serial write failed: {e}")
    _last_sent = s
    _last_tx = now
    # print("[Pi] TX:", s)

# ------------------------ Optional Start Button --------------------------------
HAS_GPIO = False
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except Exception:
    HAS_GPIO = False

START_PIN = 17  # BCM

def wait_for_start_button():
    if not HAS_GPIO:
        print("[Pi] GPIO not available, press Enter to START…")
        try:
            input()
        except EOFError:
            pass
        return
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(START_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print("[Pi] Waiting for Start button (hold to ground)…")
    stable_low_ms = 0
    while True:
        if GPIO.input(START_PIN) == 0:
            stable_low_ms += 10
            if stable_low_ms >= 300:
                print("[Pi] START pressed!")
                break
        else:
            stable_low_ms = 0
        time.sleep(0.01)

# ----------------------------- Picamera2 setup ---------------------------------
try:
    from picamera2 import Picamera2
except Exception as e:
    print("[Pi] Picamera2 is not installed or failed to import.")
    print("     Install with: sudo apt update && sudo apt install -y python3-picamera2")
    raise

def camera_open():
    """
    Initialize PiCamera2 in RGB888 (deterministic), then we convert/choose BGR per frame.
    Locks AWB after warm-up to avoid hue drift.
    """
    cam = Picamera2()
    cfg = cam.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    cam.configure(cfg)
    try:
        cam.set_controls({"AwbEnable": True, "AeEnable": True})
    except Exception:
        pass
    cam.start()
    time.sleep(0.7)
    try:
        cam.set_controls({"AwbEnable": False})  # lock AWB so hues don’t drift
    except Exception:
        pass
    return cam

# ----------------------------- Vision settings ---------------------------------
LINE_MODE = "orange"   # or "blue"
DEBUG_OVERLAY = True
SWAP_RB = False  # legacy toggle; auto-detect below should make this unnecessary

color_ranges = {
    "red1":     ((  0,  80,  70), ( 12, 255, 255)),
    "red2":     ((168,  80,  70), (179, 255, 255)),
    "green":    (( 40,  70,  70), ( 85, 255, 255)),
    "magenta":  ((135,  80,  80), (170, 255, 255)),
    "orange":   ((  5, 120, 110), ( 22, 255, 255)),
    "blue":     (( 95, 110,  80), (135, 255, 255)),
    "black":    ((  0,   0,   0), (179,  80,  80)),
}

def preprocess_hsv(frame_bgr):
    b = cv2.GaussianBlur(frame_bgr, (5,5), 0)
    hsv = cv2.cvtColor(b, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    v = clahe.apply(v)
    return cv2.merge([h, s, v])

def clean_mask(mask):
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5, iterations=1)
    return mask

def detect_colors(frame_bgr):
    hsv = preprocess_hsv(frame_bgr)
    masks = {}
    r1 = cv2.inRange(hsv, color_ranges["red1"][0], color_ranges["red1"][1])
    r2 = cv2.inRange(hsv, color_ranges["red2"][0], color_ranges["red2"][1])
    red = cv2.bitwise_or(r1, r2)
    masks["red"] = clean_mask(red)
    for c in ["green", "magenta", "orange", "blue", "black"]:
        m = cv2.inRange(hsv, color_ranges[c][0], color_ranges[c][1])
        masks[c] = clean_mask(m)
    if SWAP_RB:
        masks["red"], masks["blue"] = masks["blue"], masks["red"]
    areas = {k: int(cv2.countNonZero(v)) for k, v in masks.items()}
    return masks, areas

def centroid_from_mask(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] <= 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy)

def track_line(frame_bgr, masks, mode):
    chosen = "orange" if mode != "blue" else "blue"
    mask = masks[chosen]
    area = cv2.countNonZero(mask)
    h, w = frame_bgr.shape[:2]
    cx = w // 2
    if area > 800:
        cen = centroid_from_mask(mask)
        if cen: cx = cen[0]
    else:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        cen = centroid_from_mask(binary)
        if cen: cx = cen[0]
    return cx

def largest_contour_bbox(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, 0
    c = max(cnts, key=cv2.contourArea)
    x,y,w,h = cv2.boundingRect(c)
    return (x,y,w,h), w*h

# ---------------------- Channel-order auto-detect -------------------------------
# Decide once whether frames must be swapped RGB->BGR or used as-is
USE_SWAP_RGB2BGR = None   # None=undecided, True=swap, False=as-is
DECIDE_FRAMES = 18        # number of frames to evaluate before locking

def _color_areas_for_frame(bgr_frame):
    masks, areas = detect_colors(bgr_frame)
    return areas

def _score_mapping(bgr_frame, line_mode="orange"):
    """Higher score == mapping fits expectations better."""
    areas = _color_areas_for_frame(bgr_frame)
    line_key = "orange" if line_mode != "blue" else "blue"
    opp_key  = "blue"   if line_key == "orange" else "orange"
    score = 2.5 * areas.get(line_key, 0) - 1.0 * areas.get(opp_key, 0)
    score += 0.5 * (areas.get("red", 0) + areas.get("green", 0))
    return score

# ------------------------------ Main -------------------------------------------
def main():
    time.sleep(2)

    # Serial: scan and connect
    try:
        port_used = serial_open()
    except Exception as e:
        print(str(e))
        print("[Pi] Tip: Add your user to 'dialout' group, then reboot:")
        print("     sudo usermod -a -G dialout $USER")
        sys.exit(1)

    print("[Pi] Initializing camera…")
    cam = None
    for attempt in range(1, 4):
        try:
            cam = camera_open()
            break
        except Exception as e:
            print(f"[Pi] Camera init failed (attempt {attempt}/3): {e}")
            time.sleep(0.5)
    if cam is None:
        print("[Pi] Camera could not be initialized. Exiting.")
        sys.exit(2)

    # Create preview window if not headless
    if not HEADLESS:
        cv2.namedWindow("WRO FE – Vision", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("WRO FE – Vision", 900, 600)

    # WRO Start button (waiting state)
    wait_for_start_button()

    # Arduino handshake
    wait_for_arduino_ready()
    send_line("GO", dedupe=False)
    send_line("MODE RACE")

    laps_seen = 0
    magenta_seen_last = False
    STATE = "RACE"
    last_hb = 0.0

    try:
        while True:
            # Read a frame, re-open camera on transient errors
            try:
                frame = cam.capture_array("main")
            except Exception as e:
                print(f"[Pi] Camera read failed: {e}; reinitializing camera…")
                try:
                    cam.stop()
                except Exception:
                    pass
                cam = camera_open()
                continue

            if frame is None or frame.size == 0:
                print("[Pi] Empty frame, skipping this cycle.")
                time.sleep(0.02)
                continue

            # --- Auto-detect channel order once, then lock it ---
            global USE_SWAP_RGB2BGR
            if USE_SWAP_RGB2BGR is None:
                # Try: assume AS-IS is already BGR (no swap)
                bgr_as_is = frame
                # And: swap RGB->BGR
                bgr_swapped = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                score_as_is   = _score_mapping(bgr_as_is,   line_mode=LINE_MODE)
                score_swapped = _score_mapping(bgr_swapped, line_mode=LINE_MODE)

                if not hasattr(main, "_decide_hist"):
                    main._decide_hist = {"as_is": 0.0, "swap": 0.0, "count": 0}
                main._decide_hist["as_is"]  += score_as_is
                main._decide_hist["swap"]   += score_swapped
                main._decide_hist["count"]  += 1

                # Use the better mapping for THIS frame too
                if score_swapped > score_as_is:
                    frame_bgr = bgr_swapped
                else:
                    frame_bgr = bgr_as_is

                # Lock after enough frames
                if main._decide_hist["count"] >= DECIDE_FRAMES:
                    USE_SWAP_RGB2BGR = main._decide_hist["swap"] > main._decide_hist["as_is"]
                    print(f"[Pi] Channel mapping locked: "
                          f"{'RGB->BGR swap' if USE_SWAP_RGB2BGR else 'as-is'} "
                          f"(scores as_is={main._decide_hist['as_is']:.1f}, "
                          f"swap={main._decide_hist['swap']:.1f})")
            else:
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if USE_SWAP_RGB2BGR else frame

            # Optional orientation (uncomment if needed):
            # frame_bgr = cv2.flip(frame_bgr, 1)   # mirror left/right
            # frame_bgr = cv2.flip(frame_bgr, 0)   # upside down
            # frame_bgr = cv2.flip(frame_bgr, -1)  # both

            masks, areas = detect_colors(frame_bgr)
            h, w = frame_bgr.shape[:2]
            center_x = w // 2

            hud = []
            min_area = int(0.002 * (w*h))         # pillar detection
            near_area = int(0.004 * (w*h))        # "close" pillar
            start_mag_area = int(0.0035 * (w*h))  # start section threshold

            # Pillar hints
            if areas["red"] > min_area:
                send_line("KEEP RIGHT")
                hud.append("RED → KEEP RIGHT")
            if areas["green"] > min_area:
                send_line("KEEP LEFT")
                hud.append("GREEN → KEEP LEFT")

            # Lap counting via magenta
            in_start = areas["magenta"] > start_mag_area
            if STATE == "RACE":
                if in_start and not magenta_seen_last:
                    laps_seen += 1
                    if laps_seen >= 4:  # first pass after GO counts as 1
                        STATE = "PARK"
                        send_line("MODE PARK")
                        hud.append("MODE → PARK")
            magenta_seen_last = in_start

            # Line tracking
            cx = track_line(frame_bgr, masks, LINE_MODE)
            offset_px = cx - center_x
            offset = max(-1.0, min(1.0, offset_px / (w/2.0)))

            near_pillar = (areas["red"] > near_area) or (areas["green"] > near_area)
            speed = 120 if near_pillar else 180

            if STATE == "RACE":
                send_line(f"DRIVE {offset:.3f} {int(speed)}")
            else:
                send_line("DRIVE 0.000 0")
                now = time.time()
                if now - last_hb > 0.2:
                    send_line("HB", dedupe=False)
                    last_hb = now

            # Preview (only if not headless)
            if not HEADLESS:
                if DEBUG_OVERLAY:
                    cv2.putText(frame_bgr, f"STATE: {STATE}", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    cv2.putText(frame_bgr, f"offset: {offset:+.3f}", (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    y = 66
                    for line in hud[:4]:
                        cv2.putText(frame_bgr, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,0), 2)
                        y += 20
                cv2.imshow("WRO FE – Vision", frame_bgr)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\n[Pi] Ctrl-C received, shutting down…")

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
