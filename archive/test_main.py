"""
main.py — AeroClean entry point (Pi 5 compatible, gpiozero).
"""

import argparse
import json
import os
import sys
import time
import threading

import cv2
from gpiozero import LED

from sensors import TFRangeSensor


# ─────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="AeroClean — dirty dry-erase board detector",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument("--mode", choices=["inference", "mission"], default="inference")
    p.add_argument("--model", choices=["ocr", "yolo", "simple"], default=None)
    p.add_argument("--source", default=None)
    p.add_argument("--once", action="store_true")
    p.add_argument("--conf", type=float, default=None)
    p.add_argument("--save", action="store_true")
    p.add_argument("--config", default="config.json")
    return p


# ─────────────────────────────────────────────────────────────

def load_config(path: str) -> dict:
    if not os.path.exists(path):
        print(f"[ERROR] config.json not found at '{path}'")
        sys.exit(1)
    with open(path, "r") as f:
        return json.load(f)


def get_required_int(cfg: dict, path: str) -> int:
    """
    Read a required integer from a dotted config path like:
    - mission.pump_gpio_pin
    - wiper.wiper_gpio_pin
    """
    cur = cfg
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            print(f"[ERROR] Missing required config key: {path}")
            sys.exit(1)
        cur = cur[part]

    if cur is None:
        print(f"[ERROR] Config key is null but required: {path}")
        sys.exit(1)

    try:
        return int(cur)
    except (TypeError, ValueError):
        print(f"[ERROR] Config key must be an integer: {path}={cur!r}")
        sys.exit(1)


# ─────────────────────────────────────────────────────────────

def _source_from_camera(config_path: str):
    from camera import Camera
    with Camera(config_path) as cam:
        while True:
            yield cam.capture()


def _source_from_file(path: str):
    if not os.path.exists(path):
        print(f"[ERROR] Source file not found: {path}")
        sys.exit(1)

    ext = os.path.splitext(path)[1].lower()

    if ext in (".jpg", ".jpeg", ".png", ".bmp"):
        frame = cv2.imread(path)
        if frame is None:
            print(f"[ERROR] Could not read image: {path}")
            sys.exit(1)
        while True:
            yield frame
    else:
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            print(f"[ERROR] Could not open video: {path}")
            sys.exit(1)

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                yield frame
        finally:
            cap.release()


# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────

def main():
    args = build_parser().parse_args()

    if args.model is None:
        print("[ERROR] --model required (ocr, yolo, simple)")
        sys.exit(1)

    cfg = load_config(args.config)
    display = cfg.get("display", True)

    tf_cfg = cfg.get("tf_sensor", {})
    mission_cfg = cfg.get("mission", {})
    wiper_cfg = cfg.get("wiper", {})

    tf_port = tf_cfg.get("uart", "/dev/serial0")
    tf_baud = tf_cfg.get("baud", 115200)

    distance_threshold_m = tf_cfg.get(
        "distance_threshold_m",
        mission_cfg.get("approach_stop_dist_m", 0.75),
    )

    # ── LOAD MODEL ─────────────────────────────────────────
    if args.model == "ocr":
        from ocr_model import OCRModel
        model = OCRModel(config_path=args.config)
        print("[INFO] OCR model loaded")

    elif args.model == "simple":
        from simple_model import SimpleModel
        model = SimpleModel(config_path=args.config)
        print("[INFO] Simple model loaded")

    else:
        from yolo_model import YOLOModel
        model = YOLOModel(config_path=args.config, conf_override=args.conf)
        print("[INFO] YOLO model loaded")

    # ── GPIO SETUP FROM CONFIG ─────────────────────────────
    spray_pin = get_required_int(cfg, "mission.pump_gpio_pin")
    wipe_pin = get_required_int(cfg, "wiper.wiper_gpio_pin")

    spray = LED(spray_pin)
    wipe = LED(wipe_pin)

    print(f"[GPIO] SPRAY pin = BCM {spray_pin}")
    print(f"[GPIO] WIPE  pin = BCM {wipe_pin}")

    # ── TF MINI SENSOR SETUP ───────────────────────────────
    tf_sensor = TFRangeSensor(tf_port, baud=tf_baud)
    tf_sensor.start()
    print(f"[TF MINI] Using {tf_port} @ {tf_baud}")
    print(f"[TF MINI] Distance threshold = {distance_threshold_m:.2f} m")

    # ── CONTROL LOGIC ──────────────────────────────────────
    last_trigger_time = 0
    COOLDOWN = 5  # seconds
    prev_dirty = False

    cleaning_lock = threading.Lock()
    cleaning_in_progress = False

    def _cleaning_worker():
        nonlocal last_trigger_time, cleaning_in_progress

        try:
            print("[ACTION] SPRAY → WIPE")

            spray.on()
            time.sleep(2)
            spray.off()

            wipe.on()
            time.sleep(2)
            wipe.off()

            last_trigger_time = time.time()
        finally:
            with cleaning_lock:
                cleaning_in_progress = False

    def trigger_cleaning_async():
        nonlocal cleaning_in_progress, last_trigger_time

        now = time.time()

        with cleaning_lock:
            if cleaning_in_progress:
                return
            if now - last_trigger_time < COOLDOWN:
                return
            cleaning_in_progress = True

        threading.Thread(target=_cleaning_worker, daemon=True).start()

    # ── FRAME SOURCE ───────────────────────────────────────
    frames = _source_from_file(args.source) if args.source else _source_from_camera(args.config)

    # ── LOOP ───────────────────────────────────────────────
    try:
        for frame in frames:
            result, annotated = model.run(frame)

            dirty = bool(result)

            distance_m = tf_sensor.get_distance()
            within_threshold = (
                distance_m is not None and distance_m <= distance_threshold_m
            )

            if dirty:
                print(f"[{args.model.upper()}] DIRTY")
            else:
                print(f"[{args.model.upper()}] CLEAN")

            if distance_m is None:
                print("[TF MINI] Waiting for data...")
            else:
                print(f"[TF MINI] Distance: {distance_m:.2f} m")

            # Trigger cleaning without blocking the camera loop
            if dirty and within_threshold and not prev_dirty:
                trigger_cleaning_async()

            prev_dirty = dirty

            if display:
                overlay = annotated.copy()

                status_text = "DIRTY" if dirty else "CLEAN"
                status_color = (0, 0, 255) if dirty else (0, 255, 0)

                cv2.putText(
                    overlay,
                    f"Status: {status_text}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    status_color,
                    2,
                    cv2.LINE_AA,
                )

                if distance_m is None:
                    dist_text = "TF Mini: waiting..."
                else:
                    dist_text = f"TF Mini: {distance_m:.2f} m"

                cv2.putText(
                    overlay,
                    dist_text,
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

                cv2.putText(
                    overlay,
                    f"Threshold: {distance_threshold_m:.2f} m",
                    (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

                if distance_m is not None and not within_threshold:
                    cv2.putText(
                        overlay,
                        "Too far to trigger",
                        (20, 160),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

                cv2.imshow("AeroClean", overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            if args.once:
                break

    finally:
        tf_sensor.stop()
        if display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()