"""
preflight_test.py — Full pipeline preflight test (no flight required).

Runs the selected ML model on the camera (or a file), reads the TF range
sensor, and triggers the pump + wiper when the board is detected as dirty
and within approach_stop_dist_m. Use this to verify the entire
camera → model → sensor → pump → wiper chain on the ground before flying.

No MAVLink, no arming, no drone movement. Safe to run on a bench.

Prerequisites:
    config.json configured:
        range_sensor.uart              — UART path for TF-Luna / TFMini
        mission.pump_gpio_pin       — BCM GPIO pin for pump relay
        wiper.wiper_gpio_pin        — BCM GPIO pin for wiper relay
        mission.approach_stop_dist_m — trigger distance in metres
    gpiozero installed:
        pip install gpiozero
    Tesseract installed (for --model ocr):
        sudo apt install tesseract-ocr

Usage:
    python preflight_test.py --model ocr
    python preflight_test.py --model yolo
    python preflight_test.py --model ocr  --source board.jpg
    python preflight_test.py --model yolo --source board.mp4
    python preflight_test.py --model ocr  --once
    python preflight_test.py --config /path/to/config.json --model ocr
    Ctrl+C or press  q  to stop.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

import cv2

from pump import Pump
from sensors import TFRangeSensor
from wiper import Wiper


# ─────────────────────────────────────────────────────────────────────────────
# Cleaning trigger
# ─────────────────────────────────────────────────────────────────────────────

class CleaningTrigger:
    """
    Runs the pump + wiper cleaning sequence in a background thread so the
    display stays live during the spray and wipe. Has a cooldown between
    cycles and exposes the current state for the live overlay.
    """

    def __init__(self, pump: Pump, wiper: Wiper, spray_seconds: float, cooldown: float):
        self._pump          = pump
        self._wiper         = wiper
        self._spray_seconds = spray_seconds
        self._cooldown      = cooldown

        self._last_trigger  = 0.0
        self._in_progress   = False
        self._state         = "IDLE"   # IDLE | SPRAYING | WIPING
        self._lock          = threading.Lock()

    def get_state(self) -> str:
        with self._lock:
            return self._state

    def trigger_async(self) -> None:
        """Fire spray → wipe in a background thread if not already running and cooldown has passed."""
        now = time.time()
        with self._lock:
            if self._in_progress:
                return
            if now - self._last_trigger < self._cooldown:
                return
            self._in_progress = True

        threading.Thread(target=self._worker, daemon=True).start()

    def _worker(self) -> None:
        try:
            print("[ACTION] SPRAY → WIPE")
            with self._lock:
                self._state = "SPRAYING"
            self._pump.spray(self._spray_seconds)

            with self._lock:
                self._state = "WIPING"
            self._wiper.wipe()

            self._last_trigger = time.time()
        except Exception as e:
            print(f"[PREFLIGHT] Cleaning cycle error: {e}")
        finally:
            with self._lock:
                self._state       = "IDLE"
                self._in_progress = False

    def close(self) -> None:
        self._pump.cleanup()
        self._wiper.cleanup()


# ─────────────────────────────────────────────────────────────────────────────
# Frame sources
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
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "AeroClean — full pipeline preflight test (no flight required).\n"
            "\n"
            "Runs the selected ML model on the camera and reads the TF range sensor.\n"
            "When the board is dirty AND within approach_stop_dist_m, the pump and\n"
            "wiper trigger automatically. Use this to verify all hardware is wired\n"
            "and responding correctly before a flight.\n"
            "\n"
            "Troubleshooting:\n"
            "  No camera        →  use --source board.jpg to test offline\n"
            "  Sensor not found →  run sensor_tf_test.py first to confirm UART path\n"
            "  Pump/wiper no-op →  set mission.pump_gpio_pin and wiper.wiper_gpio_pin\n"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument(
        "--model", choices=["ocr", "yolo"], default="ocr",
        help="Which model to run:\n  ocr  — Tesseract OCR [default]\n  yolo — YOLO board-state detector",
    )
    p.add_argument(
        "--source", default=None,
        help="Path to an image or video file for offline testing.\nOmit to use the Raspberry Pi camera.",
    )
    p.add_argument("--once",   action="store_true", help="Process a single frame then exit.")
    p.add_argument("--conf",   type=float, default=None, help="YOLO confidence threshold override (0.0–1.0).")
    p.add_argument("--save",   action="store_true",      help="Save annotated frames to output_dir.")
    p.add_argument("--config", default="config.json",    help="Path to config.json (default: config.json).")
    return p


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    args = _build_parser().parse_args()

    if not os.path.exists(args.config):
        print(f"[ERROR] config.json not found at '{args.config}'", file=sys.stderr)
        sys.exit(1)
    with open(args.config) as f:
        cfg = json.load(f)

    display    = bool(cfg.get("display", True))
    output_dir = str(cfg.get("output_dir", "output"))

    tf_cfg      = cfg.get("range_sensor", {})
    mission_cfg = cfg.get("mission", {})
    w_cfg       = cfg.get("wiper", {})

    tf_port = tf_cfg.get("uart")
    tf_baud = int(tf_cfg.get("baud", 115200))

    if tf_port is None:
        _build_parser().error(
            "range_sensor.uart is not set in config.json.\n"
            "  Find your device path with: ls -l /dev/ttyAMA*\n"
            "  Then set it: \"range_sensor\": { \"uart\": \"/dev/ttyAMAx\" }"
        )

    pump_pin = mission_cfg.get("pump_gpio_pin")
    wipe_pin = w_cfg.get("wiper_gpio_pin")

    if pump_pin is None:
        print("[WARN] mission.pump_gpio_pin not set — pump will run as no-op.")
    if wipe_pin is None:
        print("[WARN] wiper.wiper_gpio_pin not set — wiper will run as no-op.")

    distance_threshold_m = float(mission_cfg.get("approach_stop_dist_m", 0.5))
    spray_seconds        = float(mission_cfg.get("pump_duration_s", 5.0))
    cooldown             = float(mission_cfg.get("cleaning_cooldown_s", 5.0))

    pump    = Pump(pin=int(pump_pin) if pump_pin is not None else None)
    wiper   = Wiper(pin=int(wipe_pin) if wipe_pin is not None else None,
                    wipe_duration_s=float(w_cfg.get("wipe_duration_s", 2.0)))
    trigger = CleaningTrigger(pump=pump, wiper=wiper,
                              spray_seconds=spray_seconds, cooldown=cooldown)

    if args.model == "ocr":
        from ocr_model import OCRModel
        model = OCRModel(config_path=args.config)
        print("[INFO] OCR model loaded")
    else:
        from yolo_model import YOLOModel
        model = YOLOModel(config_path=args.config, conf_override=args.conf)
        print("[INFO] YOLO model loaded")

    tf_sensor = TFRangeSensor(tf_port, baud=tf_baud)
    tf_sensor.start()
    print(f"[TF] Sensor on {tf_port} @ {tf_baud} baud")
    print(f"[TF] Cleaning triggers at ≤ {distance_threshold_m:.2f} m")

    frames      = _frames_from_file(args.source) if args.source else _frames_from_camera(args.config)
    window_name = f"AeroClean — Preflight ({args.model.upper()})"
    prev_dirty      = False
    prev_within     = None
    last_dist_cm    = None

    try:
        if display:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        for frame in frames:
            result, annotated = model.run(frame)
            dirty = bool(result)

            dist_m           = tf_sensor.get_distance()
            within_threshold = dist_m is not None and dist_m <= distance_threshold_m
            dist_cm          = None if dist_m is None else round(dist_m * 100, 1)

            state_changed = dirty != prev_dirty
            range_changed = within_threshold != prev_within
            dist_changed  = dist_cm != last_dist_cm

            if state_changed or range_changed or dist_changed:
                status_str = "DIRTY" if dirty else "CLEAN"
                dist_str   = f"{dist_m:.2f} m" if dist_m is not None else "waiting..."
                print(f"[{args.model.upper()}] {status_str}   [TF] {dist_str}")
                prev_within  = within_threshold
                last_dist_cm = dist_cm

            if dirty and within_threshold and not prev_dirty:
                trigger.trigger_async()

            prev_dirty = dirty

            if args.save:
                os.makedirs(output_dir, exist_ok=True)
                ts   = int(time.time() * 1000)
                dest = os.path.join(output_dir, f"{'dirty' if dirty else 'clean'}_{ts}.jpg")
                cv2.imwrite(dest, annotated)

            if display:
                overlay      = annotated.copy()
                status_color = (0, 0, 255) if dirty else (0, 200, 0)
                cv2.putText(overlay, f"Status: {status_str}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2, cv2.LINE_AA)
                cv2.putText(overlay, f"TF: {dist_str}", (20, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(overlay, f"Threshold: {distance_threshold_m:.2f} m", (20, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

                if dist_m is not None and not within_threshold:
                    cv2.putText(overlay, "Too far to trigger", (20, 160),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)

                clean_state = trigger.get_state()
                if clean_state != "IDLE":
                    action_color = (255, 0, 0) if clean_state == "SPRAYING" else (0, 255, 255)
                    cv2.putText(overlay, clean_state, (20, 200),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, action_color, 3, cv2.LINE_AA)

                cv2.imshow(window_name, overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("[INFO] Quit.")
                    break

            if args.once:
                break

    except KeyboardInterrupt:
        print("\n[INFO] Stopped.")
    finally:
        tf_sensor.stop()
        trigger.close()
        if display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
