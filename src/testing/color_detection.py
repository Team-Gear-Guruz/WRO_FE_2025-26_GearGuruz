#!/usr/bin/env python3
# -- coding: utf-8 --
"""
Color Detection Module
Detects RED, GREEN, and MAGENTA (pink) objects using HSV masking.
"""

import cv2
import numpy as np

class ColorDetector:
    def __init__(self):
        # HSV ranges for colors
        self.red1_low, self.red1_high = (0, 90, 90), (10, 255, 255)
        self.red2_low, self.red2_high = (170, 90, 90), (179, 255, 255)
        self.green_low, self.green_high = (40, 80, 80), (85, 255, 255)
        self.magenta_low, self.magenta_high = (130, 80, 80), (165, 255, 255)

    def _mask(self, hsv, low, high):
        return cv2.inRange(hsv, np.array(low), np.array(high))

    def detect(self, frame_bgr):
        """Detect red, green, and magenta objects in the given BGR frame."""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # Red is split in two ranges
        red = cv2.bitwise_or(
            self._mask(hsv, self.red1_low, self.red1_high),
            self._mask(hsv, self.red2_low, self.red2_high)
        )
        green = self._mask(hsv, self.green_low, self.green_high)
        magenta = self._mask(hsv, self.magenta_low, self.magenta_high)

        return {"red": red, "green": green, "magenta": magenta}
