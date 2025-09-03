#!/usr/bin/env python3
"""
WRO 2025 – Vision → Arduino High-Level Control (Obstacle Challenge)

Raspberry Pi responsibilities (camera + CV only):
- Track the mat line (orange/blue) to compute a normalized lane offset
- Detect red/green pillars and send "keep-side" hints
- Detect the starting section (magenta parking markers) to count laps
- After 3 laps → tell Arduino to enter PARK mode
- Stream "DRIVE <offset> <speed>" ~15–25 Hz (rate-limited & deduplicated)
- Simple handshake: wait for Arduino "RDY", then send "GO" and "MODE RACE"
- Heartbeat if idle, clean shutdown on exit

Arduino responsibilities (you implement):
- Motor + servo actuation
- State machine: RACE / PILLAR_AVOID / PARK_xxx
- Reversing, speed control, safety bubbles
- Complete parallel parking using 6 HC-SR04 sensors
- Watchdog stop if Pi goes silent

Run:
  python3 pi_vision_obstacle_highlevel.py
"""

import cv2
import numpy as np
import time
import serial
import sys
from picamera2 import Picamera2

# --------- Optional GPIO Start Button (WRO-compliant "waiting state") ----------
HAS_GPIO = False
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except Exception:
    HAS_GPIO = False

START_PIN = 17  # BCM pin for the Start pushbutton (3.3V safe, wired to ground when pressed)

def wait_for_start_button():
    """
    Meet WRO requirement: after power on, vehicle waits for a physical Start button.
    On Pi-only dev boxes (no GPIO), fallback to Enter key.
    """
    if not HAS_GPIO:
        print("[Pi] GPIO not available, press Enter to START…")
        input()
        return
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(START_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print("[Pi] Waiting for Start button (hold to ground)…")
    # Debounced wait for a falling edge
    stable_low_ms = 0
    while True:
        if GPIO.input(START_PIN) == 0:  # pressed (LOW)
            stable_low_ms += 10
            if stable_low_ms >= 300:    # 300 ms stable press
                print("[Pi] START pressed!")
                break
        else:
            stable_low_ms = 0
        time.sleep(0.01)


# ------------------------- Serial (Pi ↔ Arduino) -------------------------------
PORT = '/dev/ttyACM0'  # change if needed (Windows: 'COM3' etc.)
BAUD = 115200

ser = None
_last_sent = ""
_last_tx = 0.0

def serial_open():
    global ser
    ser = serial.Serial(PORT, BAUD, timeout=0.02)
    time.sleep(0.5)

def wait_for_arduino_ready(max_wait=10.0):
    """
    Arduino should print 'RDY' when it has booted and is waiting for GO.
    We don't block forever—if not seen, we continue (helps during bench tests).
    """
    print("[Pi] Waiting for Arduino RDY…")
    t0 = time.time()
    while time.time() - t0 < max_wait:
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except Exception:
            line = ""
        if line:
            # Uncomment next line to debug other Arduino prints
            # print("[Pi] RX:", line)
            if line == "RDY":
                print("[Pi] Arduino RDY")
                return
    print("[Pi] No RDY seen, continuing…")

def send_line(s: str, min_interval=0.08, dedupe=True):
    """
    Rate-limit & de-duplicate serial writes to avoid spamming the Arduino every frame.
    """
    global _last_sent, _last_tx
    now = time.time()
    if dedupe and s == _last_sent and (now - _last_tx) < min_interval:
        return
    ser.write((s + "\n").encode())
    _last_sent = s
    _last_tx = now
    # Uncomment to see every TX:
    # print("[Pi] TX:", s)


# ----------------------------- Vision settings ---------------------------------
# Choose the mat line color here
LINE_MODE = "orange"   # or "blue"
DEBUG_OVERLAY = True   # draw HUD

# HSV color bands (OpenCV H:0–179, S/V:0–255)
color_ranges = {
    # Red wraps around hue
    "red1":     ((  0,  80,  70), ( 12, 255, 255)),
    "red2":     ((168,  80,  70), (179, 255, 255)),
    "green":    (( 40,  70,  70), ( 85, 255, 255)),
    "magenta":  ((135,  80,  80), (170, 255, 255)),
    "orange":   ((  5, 120, 110), ( 22, 255, 255)),
    "blue":     (( 95, 110,  80), (135, 255, 255)),
    "black":    ((  0,   0,   0), (179,  80,  80)),   # walls (low V)
}

def preprocess_hsv(frame_bgr):
    """
    Light blur + HSV + CLAHE on V for robustness on venue lighting.
    """
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
    """
    Returns dict of masks + pixel areas for red, green, magenta, orange, blue, black.
    """
    hsv = preprocess_hsv(frame_bgr)
    masks = {}
    # Red = two bands
    r1 = cv2.inRange(hsv, color_ranges["red1"][0], color_ranges["red1"][1])
    r2 = cv2.inRange(hsv, color_ranges["red2"][0], color_ranges["red2"][1])
    red = cv2.bitwise_or(r1, r2)
    masks["red"] = clean_mask(red)

    for c in ["green", "magenta", "orange", "blue", "black"]:
        m = cv2.inRange(hsv, color_ranges[c][0], color_ranges[c][1])
        masks[c] = clean_mask(m)

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
    """
    Get horizontal line centroid x — fallback to grayscale-invert if color weak.
    Returns cx (x position), or image center if not found.
    """
    chosen = "orange" if mode != "blue" else "blue"
    mask = masks[chosen]
    area = cv2.countNonZero(mask)

    h, w = frame_bgr.shape[:2]
    cx = w // 2

    if area > 800:  # threshold tuned for 640x480; adjust if needed
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


# ------------------------------- Camera init -----------------------------------
def camera_open():
    picam2 = Picamera2()
    cfg = picam2.create_video_configuration(main={"size": (640, 480), "format": "BGR888"})
    picam2.configure(cfg)
    # Let AE/AWB settle
    try:
        picam2.set_controls({"AwbEnable": True, "AeEnable": True})
    except Exception:
        pass
    picam2.start()
    time.sleep(0.6)
    return picam2


# ------------------------------ Main program -----------------------------------
def main():
    global _last_sent, _last_tx

    print("[Pi] Opening serial on", PORT)
    serial_open()

    print("[Pi] Initializing camera…")
    cam = camera_open()

    # Create preview window (resizable)
    cv2.namedWindow("WRO FE – Vision", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("WRO FE – Vision", 900, 600)

    # WRO Start button (waiting state)
    wait_for_start_button()

    # Arduino handshake
    wait_for_arduino_ready()
    send_line("GO", dedupe=False)
    send_line("MODE RACE")

    # Lap counting via magenta (start section)
    laps_seen = 0
    magenta_seen_last = False
    STATE = "RACE"

    last_hb = 0.0

    try:
        while True:
            frame = cam.capture_array("main")
            if frame is None or frame.size == 0:
                print("[Pi] No camera frame, exiting.")
                break

            masks, areas = detect_colors(frame)
            h, w = frame.shape[:2]
            center_x = w // 2

            hud = []
            min_area = int(0.002 * (w*h))      # pillar detection
            near_area = int(0.004 * (w*h))     # "close" pillar
            start_mag_area = int(0.0035 * (w*h))  # start section threshold

            # --- Pillar hints (Pi only hints; Arduino decides maneuvers) ---
            if areas["red"] > min_area:
                send_line("KEEP RIGHT")
                hud.append("RED → KEEP RIGHT")
            if areas["green"] > min_area:
                send_line("KEEP LEFT")
                hud.append("GREEN → KEEP LEFT")

            # --- Lap counting using magenta (start section contains parking lot markers) ---
            in_start = areas["magenta"] > start_mag_area
            if STATE == "RACE":
                if in_start and not magenta_seen_last:
                    laps_seen += 1
                    # First time we see start section after GO counts as pass 1.
                    # After 3 complete laps, we expect to see it 4 times.
                    if laps_seen >= 4:
                        STATE = "PARK"
                        send_line("MODE PARK")
                        hud.append("MODE → PARK")
            magenta_seen_last = in_start

            # --- Line centroid & drive streaming (RACE only) ---
            cx = track_line(frame, masks, LINE_MODE)

            # Normalize offset to [-1, +1] (1.0 means far right half)
            offset_px = cx - center_x
            offset = max(-1.0, min(1.0, offset_px / (w/2.0)))

            # Slow down a bit when pillars are very close
            near_pillar = (areas["red"] > near_area) or (areas["green"] > near_area)
            speed = 120 if near_pillar else 180

            if STATE == "RACE":
                send_line(f"DRIVE {offset:.3f} {int(speed)}")
            else:
                # In PARK mode, Pi stops commanding motion.
                # Still send a "neutral" drive & heartbeat so Arduino watchdog stays happy.
                send_line("DRIVE 0.000 0")
                # Heartbeat at ~5 Hz
                if time.time() - last_hb > 0.2:
                    send_line("HB", dedupe=False)
                    last_hb = time.time()

            # --- HUD overlay for your tuning ---
            if DEBUG_OVERLAY:
                cv2.line(frame, (center_x, 0), (center_x, h), (0, 255, 0), 1)
                cx_vis = int(center_x + offset * (w/2.0))
                cv2.line(frame, (cx_vis, 0), (cx_vis, h), (255, 0, 0), 1)
                hud.append(f"STATE:{STATE} laps_seen:{laps_seen}")
                hud.append(f"offset:{offset:+.3f} speed:{speed}")
                hud.append(f"areas: R{areas['red']} G{areas['green']} M{areas['magenta']}")
                for i, msg in enumerate(hud):
                    cv2.putText(frame, msg, (10, 28 + 22*i),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

            cv2.imshow("WRO FE – Vision", frame)
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
        cv2.destroyAllWindows()
        print("[Pi] Clean exit.")


if __name__ == "__main__":
    main()
