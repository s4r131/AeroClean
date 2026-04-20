"""
sensor_ocr_test.py — Live test: range sensor + OCR model together.

Shows an OpenCV window with:
  - Bounding boxes around the word "dirty" (drawn by OCRModel)
  - Distance overlay on the frame when "dirty" is detected
  - Status banner: DIRTY + distance in red, or CLEAN in green

Two range sensor options:
  --sensor a  VL53L3CX (I2C, default) — reads i2c_address from config.json
  --sensor b  TF-Luna / TFMini (UART) — reads uart/baud from config.json tf_sensor
              Override UART with --uart /dev/ttyAMAx

Usage:
    python sensor_ocr_test.py                              # sensor A, Pi camera
    python sensor_ocr_test.py --sensor b --uart /dev/ttyAMA3
    python sensor_ocr_test.py --source board.jpg           # offline image
    python sensor_ocr_test.py --config config.json

Press  q  to quit.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import cv2
import numpy as np

from ocr_model import OCRModel
from sensors import RangeSensor, TFRangeSensor


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="AeroClean — sensor + OCR integration test")
    p.add_argument("--config", default="config.json", help="Path to config.json")
    p.add_argument("--source", default=None, help="Image or video file for offline testing")
    p.add_argument(
        "--sensor", choices=["a", "b"], default="a",
        help=(
            "Range sensor type:\n"
            "  a  VL53L3CX (I2C, default) — reads range_sensor.i2c_address from config.json\n"
            "  b  TF-Luna / TFMini (UART) — reads tf_sensor.uart from config.json"
        ),
    )
    return p


# ─────────────────────────────────────────────────────────────────────────────
# Frame source
# ─────────────────────────────────────────────────────────────────────────────

def _frames_from_camera(config_path: str):
    from camera import Camera
    with Camera(config_path) as cam:
        while True:
            yield cam.capture()


def _frames_from_file(path: str):
    if not os.path.exists(path):
        print(f"[ERROR] File not found: {path}", file=sys.stderr)
        sys.exit(1)
    ext = os.path.splitext(path)[1].lower()
    if ext in (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"):
        frame = cv2.imread(path)
        if frame is None:
            print(f"[ERROR] Could not read image: {path}", file=sys.stderr)
            sys.exit(1)
        while True:
            yield frame
    else:
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            print(f"[ERROR] Could not open video: {path}", file=sys.stderr)
            sys.exit(1)
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                yield frame
        finally:
            cap.release()


# ─────────────────────────────────────────────────────────────────────────────
# Overlay helpers
# ─────────────────────────────────────────────────────────────────────────────

def _draw_distance(frame: np.ndarray, dist: float | None) -> np.ndarray:
    """Draw distance reading onto the frame near the top-left."""
    if dist is None:
        dist_text = "range: waiting..."
    else:
        dist_text = f"range: {dist:.2f}m"
    cv2.putText(frame, dist_text, (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    return frame


def _draw_banner(frame: np.ndarray, dirty: bool, dist: float | None) -> np.ndarray:
    """Replace the OCRModel status banner with one that includes the distance."""
    # Cover the existing banner drawn by OCRModel
    cv2.rectangle(frame, (0, 0), (420, 40), (0, 0, 0), -1)
    if dirty:
        if dist is not None:
            text = f"DIRTY DETECTED  {dist:.2f}m"
        else:
            text = "DIRTY DETECTED  (ranging...)"
        color = (0, 0, 255)
    else:
        text = "CLEAN"
        color = (0, 200, 0)
    cv2.putText(frame, text, (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    return frame


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    args = _build_parser().parse_args()

    with open(args.config) as f:
        cfg = json.load(f)

    if args.sensor == "b":
        tf_cfg = cfg.get("tf_sensor", {})
        uart = tf_cfg.get("uart")
        if not uart:
            _build_parser().error(
                "tf_sensor.uart is not set in config.json.\n"
                "  Find your device path with: ls -l /dev/ttyAMA*\n"
                "  Then set it in config.json:  \"tf_sensor\": { \"uart\": \"/dev/ttyAMAx\" }"
            )
        sensor = TFRangeSensor(uart, baud=tf_cfg.get("baud", 115200))
        print(f"[TEST] Using sensor B — TF-Luna/TFMini on {uart}")
    else:
        range_cfg     = cfg.get("range_sensor", {})
        timing_budget = int(range_cfg.get("timing_budget_ms", 50))
        sensor = RangeSensor(range_cfg.get("i2c_address", 0x29), timing_budget_ms=timing_budget)
        print("[TEST] Using sensor A — VL53L3CX (I2C)")
    sensor.start()

    ocr = OCRModel(config_path=args.config)

    frames = _frames_from_file(args.source) if args.source else _frames_from_camera(args.config)

    cv2.namedWindow("AeroClean — Sensor+OCR Test", cv2.WINDOW_NORMAL)
    print("[TEST] Running. Press  q  to quit.")

    try:
        for frame in frames:
            dirty, annotated = ocr.run(frame)

            dist: float | None = None
            if dirty:
                dist = sensor.get_distance()
                dist_str = f"{dist:.3f}m" if dist is not None else "waiting"
                print(f"[TEST] DIRTY detected — range={dist_str}")

            _draw_banner(annotated, dirty, dist)
            if dirty:
                _draw_distance(annotated, dist)

            cv2.imshow("AeroClean — Sensor+OCR Test", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("[TEST] Quit.")
                break
    finally:
        sensor.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
