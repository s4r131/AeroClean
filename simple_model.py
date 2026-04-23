import cv2
import numpy as np
import json


def _load_config(path: str = "config.json") -> dict:
    with open(path) as f:
        return json.load(f)


class SimpleModel:
    """
    Fast black/white detector with adaptive threshold (handles beige boards).
    """

    def __init__(self, config_path="config.json"):
        cfg = _load_config(config_path)
        self._roi = cfg.get("board_roi")

        # Tuning parameters
        self.min_area = 400   # ignore small noise
        self.block_size = 31  # must be odd
        self.C = 12           # increase to ignore beige more

    def _crop(self, frame):
        if self._roi is None:
            return frame, (0, 0)
        x, y, w, h = self._roi
        return frame[y:y+h, x:x+w], (x, y)

    def run(self, frame):
        cropped, offset = self._crop(frame)
        ox, oy = offset

        # Convert to grayscale
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)

        # Smooth image (reduces noise + lighting variation)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Adaptive threshold (key fix for beige problem)
        thresh = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            self.block_size,
            self.C
        )

        # Find contours (dark regions)
        contours, _ = cv2.findContours(
            thresh,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        annotated = frame.copy()
        dirty = False

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area > self.min_area:
                dirty = True
                x, y, w, h = cv2.boundingRect(cnt)

                # Adjust for ROI offset
                x += ox
                y += oy

                cv2.rectangle(
                    annotated,
                    (x, y),
                    (x + w, y + h),
                    (0, 0, 255),
                    2
                )

        # Status banner
        if dirty:
            text = "BLACK (DIRTY)"
            color = (0, 0, 255)
        else:
            text = "WHITE (CLEAN)"
            color = (0, 255, 0)

        cv2.rectangle(annotated, (0, 0), (420, 60), (0, 0, 0), -1)
        cv2.putText(
            annotated,
            text,
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.1,
            color,
            3
        )

        return dirty, annotated