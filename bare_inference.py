"""bare_inference.py — inference loop + cleaning trigger, but no drone movement."""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from typing import Any

import cv2
from pump import Pump
from sensors import TFRangeSensor
from wiper import Wiper


def load_config(path: str) -> dict:
    if not os.path.exists(path):
        print(f"[ERROR] config.json not found at '{path}'", file=sys.stderr)
        sys.exit(1)
    with open(path, "r") as f:
        return json.load(f)


class CleaningTrigger:
    """
    Config-driven cleaning sequence:
      1) pump.spray(mission.pump_duration_s)
      2) wiper.wipe()
    """

    def __init__(
        self,
        pump: Pump,
        wiper: Wiper,
        cooldown: float,
        spray_seconds: float,
    ):
        self.pump = pump
        self.wiper = wiper
        self.cooldown = cooldown
        self.spray_seconds = spray_seconds

        self._last_trigger_time = 0.0
        self._lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._in_progress = False
        self._state = "IDLE"  # IDLE | SPRAYING | WIPING

    @classmethod
    def from_config(cls, cfg: dict[str, Any], config_path: str) -> "CleaningTrigger":
        mission_cfg = cfg.get("mission", {})
        wiper_cfg = cfg.get("wiper", {})

        spray_pin = mission_cfg.get("pump_gpio_pin")
        wipe_pin = wiper_cfg.get("wiper_gpio_pin")

        if spray_pin is None:
            raise ValueError("mission.pump_gpio_pin must be set in config.json")
        if wipe_pin is None:
            raise ValueError("wiper.wiper_gpio_pin must be set in config.json")

        cooldown = float(mission_cfg.get("cleaning_cooldown_s", 5.0))
        spray_seconds = float(mission_cfg.get("pump_duration_s", 5.0))

        pump = Pump(pin=int(spray_pin))
        wiper = Wiper.from_config(config_path)

        return cls(
            pump=pump,
            wiper=wiper,
            cooldown=cooldown,
            spray_seconds=spray_seconds,
        )

    def _set_state(self, state: str) -> None:
        with self._state_lock:
            self._state = state

    def get_state(self) -> str:
        with self._state_lock:
            return self._state

    def _worker(self):
        try:
            print("[ACTION] SPRAY → WIPE")

            self._set_state("SPRAYING")
            self.pump.spray(self.spray_seconds)

            self._set_state("WIPING")
            self.wiper.wipe()

            self._last_trigger_time = time.time()
        except Exception as e:
            print(f"[CLEANING] Error during cleaning cycle: {e}")
        finally:
            self._set_state("IDLE")
            with self._lock:
                self._in_progress = False

    def trigger_async(self):
        now = time.time()
        with self._lock:
            if self._in_progress:
                return
            if now - self._last_trigger_time < self.cooldown:
                return
            self._in_progress = True
            threading.Thread(target=self._worker, daemon=True).start()

    def close(self):
        self.pump.cleanup()
        self.wiper.cleanup()


def _source_from_camera(config_path: str):
    from camera import Camera

    with Camera(config_path) as cam:
        while True:
            yield cam.capture()


def _source_from_file(path: str):
    if not os.path.exists(path):
        print(f"[ERROR] Source file not found: {path}", file=sys.stderr)
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


def _save_frame(frame, output_dir: str, prefix: str):
    os.makedirs(output_dir, exist_ok=True)
    ts = int(time.time() * 1000)
    path = os.path.join(output_dir, f"{prefix}_{ts}.jpg")
    cv2.imwrite(path, frame)


def run_bare_inference(
    config_path: str,
    model_name: str,
    source: str | None = None,
    once: bool = False,
    conf_override: float | None = None,
    save: bool = False,
    display: bool | None = None,
    output_dir: str | None = None,
):
    cfg = load_config(config_path)

    if display is None:
        display = bool(cfg.get("display", True))
    if output_dir is None:
        output_dir = str(cfg.get("output_dir", "output"))

    tf_cfg = cfg.get("tf_sensor", {})
    mission_cfg = cfg.get("mission", {})
    range_cfg = cfg.get("range_sensor", {})

    tf_port = tf_cfg.get("uart", "/dev/serial0")
    tf_baud = tf_cfg.get("baud", 115200)
    distance_threshold_m = tf_cfg.get(
        "distance_threshold_m",
        mission_cfg.get("approach_stop_dist_m", 0.75),
    )

    try:
        cleaning = CleaningTrigger.from_config(cfg, config_path)
    except ValueError as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

    if model_name == "ocr":
        from ocr_model import OCRModel

        model = OCRModel(config_path=config_path)
        print("[INFO] OCR model loaded")
    elif model_name == "yolo":
        from yolo_model import YOLOModel

        model = YOLOModel(config_path=config_path, conf_override=conf_override)
        print("[INFO] YOLO model loaded")
    elif model_name == "simple":
        from simple_model import SimpleModel

        model = SimpleModel(config_path=config_path)
        print("[INFO] Simple model loaded")
    else:
        print("[ERROR] --model required (ocr, yolo, simple)", file=sys.stderr)
        sys.exit(1)

    if range_cfg.get("type", "a") != "a":
        print("[WARN] range_sensor.type is not 'a'; this inference loop uses tf_sensor settings.")

    tf_sensor = TFRangeSensor(tf_port, baud=tf_baud)
    tf_sensor.start()
    print(f"[TF MINI] Using {tf_port} @ {tf_baud}")
    print(f"[TF MINI] Distance threshold = {distance_threshold_m:.2f} m")

    if source:
        frames = _source_from_file(source)
    else:
        frames = _source_from_camera(config_path)

    prev_dirty = False
    window_name = f"AeroClean — {model_name.upper()}"

    try:
        if display:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        for frame in frames:
            result, annotated = model.run(frame)
            dirty = bool(result)

            distance_m = tf_sensor.get_distance()
            within_threshold = distance_m is not None and distance_m <= distance_threshold_m

            print(f"[{model_name.upper()}] {'DIRTY' if dirty else 'CLEAN'}")
            if distance_m is None:
                print("[TF MINI] Waiting for data...")
            else:
                print(f"[TF MINI] Distance: {distance_m:.2f} m")

            if dirty and within_threshold and not prev_dirty:
                cleaning.trigger_async()

            prev_dirty = dirty

            if save:
                prefix = "dirty" if dirty else "clean"
                _save_frame(annotated, output_dir, prefix)

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

                clean_state = cleaning.get_state()
                if clean_state != "IDLE":
                    cv2.putText(
                        overlay,
                        f"Cleaning: {clean_state}",
                        (20, 200),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (0, 165, 255),
                        2,
                        cv2.LINE_AA,
                    )

                    if clean_state == "SPRAYING":
                        cv2.putText(
                            overlay,
                            "SPRAYING",
                            (400, 60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.5,
                            (255, 0, 0),
                            3,
                            cv2.LINE_AA,
                        )
                    elif clean_state == "WIPING":
                        cv2.putText(
                            overlay,
                            "WIPING",
                            (400, 60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.5,
                            (0, 255, 255),
                            3,
                            cv2.LINE_AA,
                        )

                cv2.imshow(window_name, overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("[INFO] Quit.")
                    break

            if once:
                break

    finally:
        tf_sensor.stop()
        cleaning.close()
        if display:
            cv2.destroyAllWindows()