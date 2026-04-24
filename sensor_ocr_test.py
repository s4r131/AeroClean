"""
sensor_ocr_test.py — Live test: range sensor + OCR model together.

Shows an OpenCV window with:
  - Bounding boxes around the word "dirty" (drawn by OCRModel)
  - Distance overlay on the frame when "dirty" is detected
  - Status banner: DIRTY + distance in red, or CLEAN in green

Two range sensor options:
  --sensor a  TF-Luna / TFMini (UART, default) — reads range_sensor.uart from config.json
  --sensor b  VL53L3CX (I2C) — reads range_sensor.i2c_address from config.json

Usage:
    python sensor_ocr_test.py                              # sensor A (TF-Luna), Pi camera
    python sensor_ocr_test.py --sensor b                  # sensor B (VL53L3CX)
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


def _banner(groups: list[list[str]]) -> None:
    width = max(len(l) for g in groups for l in g) + 4
    sep   = "═" * width
    print(f"╔{sep}╗")
    for i, group in enumerate(groups):
        for line in group:
            print(f"║  {line:<{width - 2}}║")
        if i < len(groups) - 1:
            print(f"╠{sep}╣")
    print(f"╚{sep}╝")


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "AeroClean — range sensor + OCR integration test.\n"
            "\n"
            "Shows a live OpenCV window with OCR bounding boxes and a distance overlay.\n"
            "When the word 'dirty' is detected the banner turns red and shows the range reading.\n"
            "\n"
            "Use this script to:\n"
            "  - Confirm the OCR pipeline and range sensor work together correctly\n"
            "  - Verify distance readings appear on screen when a dirty board is detected\n"
            "  - Test with a saved image or video before connecting live hardware\n"
            "  - Compare sensor A (UART) vs sensor B (I2C) in a real scenario\n"
            "\n"
            "Troubleshooting:\n"
            "  No distance shown    →  check sensor wiring and run the sensor test script first\n"
            "  OCR never triggers   →  write the word 'dirty' on the board in clear marker\n"
            "  Sensor A not found   →  set range_sensor.uart in config.json first\n"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument(
        "--config", default="config.json",
        help="Path to config.json (default: config.json).",
    )
    p.add_argument(
        "--source", default=None,
        help=(
            "Path to an image or video file for offline testing.\n"
            "Omit to use the live Raspberry Pi camera.\n"
            "Supported: .jpg .png .bmp .mp4 .avi and other OpenCV formats."
        ),
    )
    p.add_argument(
        "--sensor", choices=["a", "b"], default="a",
        help=(
            "Which range sensor to use:\n"
            "  a  TF-Luna / TFMini over UART (default) — reads range_sensor.uart from config.json\n"
            "  b  VL53L3CX over I2C — reads range_sensor.i2c_address from config.json\n"
            "Run sensor_tf_test.py (A) or sensor_tf_i2c_test.py (B) first to confirm the sensor works."
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

    if args.sensor == "a":
        tf_cfg = cfg.get("range_sensor", {})
        uart = tf_cfg.get("uart")
        if not uart:
            _build_parser().error(
                "range_sensor.uart is not set in config.json.\n"
                "  Find your device path with: ls -l /dev/ttyAMA*\n"
                "  Then set it in config.json:  \"range_sensor\": { \"uart\": \"/dev/ttyAMAx\" }"
            )
        baud   = tf_cfg.get("baud", 115200)
        sensor = TFRangeSensor(uart, baud=baud)
        sensor_desc = [f"Sensor : TF-Luna / TFMini (sensor A)", f"UART   : {uart}  {baud} baud"]
    else:
        range_cfg     = cfg.get("range_sensor", {})
        timing_budget = int(range_cfg.get("timing_budget_ms", 50))
        i2c_address   = range_cfg.get("i2c_address", 0x29)
        sensor        = RangeSensor(i2c_address, timing_budget_ms=timing_budget)
        sensor_desc   = [f"Sensor : VL53L3CX (sensor B)", f"I2C    : 0x{i2c_address:02X}  {timing_budget} ms budget"]
    sensor.start()

    _banner([
        [
            "AeroClean — Sensor + OCR Test",
            "Verifies OCR detection and range sensor work together",
        ],
        sensor_desc,
        [
            "Expect : prints only on CLEAN / DIRTY state change or distance change",
            "Stop   : press  q  in the live window",
        ],
    ])

    ocr = OCRModel(config_path=args.config)

    frames = _frames_from_file(args.source) if args.source else _frames_from_camera(args.config)

    cv2.namedWindow("AeroClean — Sensor+OCR Test", cv2.WINDOW_NORMAL)

    prev_dirty    = None
    last_dist_cm  = None

    try:
        for frame in frames:
            dirty, annotated = ocr.run(frame)

            dist: float | None = None
            if dirty:
                dist = sensor.get_distance()

            state_changed = dirty != prev_dirty
            dist_cm       = None if dist is None else round(dist * 100, 1)
            dist_changed  = dirty and (dist_cm != last_dist_cm)

            if state_changed or dist_changed:
                if dirty:
                    dist_str = f"{dist:.3f}m" if dist is not None else "waiting"
                    print(f"[TEST] DIRTY detected — range={dist_str}")
                else:
                    print("[TEST] CLEAN")
                prev_dirty   = dirty
                last_dist_cm = dist_cm

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
