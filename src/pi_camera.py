#!/usr/bin/env python3
# -- coding: utf-8 --
"""
Pi Camera Capture + Color Detection
Captures frames from Raspberry Pi camera (or USB webcam) 
and uses ColorDetector to highlight RED, GREEN, and MAGENTA objects.
"""

import cv2
from color_detection import ColorDetector

def run_camera(camera_index=0, width=320, height=240):
    detector = ColorDetector()
    cap = cv2.VideoCapture(camera_index)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    print("[INFO] Starting Pi Camera stream. Press ESC to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[ERR] Failed to read frame")
            break

        masks = detector.detect(frame)

        # Visualize results
        vis = frame.copy()
        for color, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                x, y, w, h = cv2.boundingRect(c)
                if cv2.contourArea(c) > 180 and h > 16:
                    col = (0,0,255) if color=="red" else ((0,255,0) if color=="green" else (255,0,255))
                    cv2.rectangle(vis, (x,y), (x+w,y+h), col, 2)
                    cv2.putText(vis, color, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

        cv2.imshow("Pi Camera - Color Detection", vis)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_camera()
