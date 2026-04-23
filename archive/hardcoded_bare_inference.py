"""
bare_inference.py — inference loop + cleaning trigger, but no drone movement.
"""

from __future__ import annotations

import os
import sys
import json
import threading
import time
from typing import Any

import cv2

from sensors import TFRangeSensor

try:
    from gpiozero import LED
    GPIOZERO_AVAILABLE = True
except Exception:
    GPIOZERO_AVAILABLE = False


class _GPIOOutput:
    """
    GPIO output wrapper with safe no-op behavior when GPIO is unavailable.
    Uses gpiozero when possible.
    """

    def __init__(self, pin: int | None, label: str):
        self.pin = pin
        self.label = label
        self._available = False
        self._device = None

        if pin is None:
            print(f"[{label}] GPIO pin not configured — running as no-op.")
            return

        if not GPIOZERO_AVAILABLE:
            print(f"[{label}] gpiozero not available — running as no-op.")
            return

        try:
            self._device = LED(pin)
            self._device.off()
            self._available = True
        except Exception as e:
            print(f"[{label}] GPIO init failed for pin {pin} — running as no-op: {e}")
            self._available = False

    def on(self) -> None:
        if self._available and self._device is not None:
            self._device.on()

    def off(self) -> None:
        if self._available and self._device is not None:
            self._device.off()

    def cleanup(self) -> None:
        if self._device is not None:
            try:
                self._device.off()
            except Exception:
                pass
            try:
                self._device.close()
            except Exception:
                pass


def load_config(path: str) -> dict:
    if not os.path.exists(path):
        print(f"[ERROR] config.json not found at '{path}'", file=sys.stderr)
        sys.exit(1)
    with open(path, "r") as f:
        return json.load(f)


def _estimated_wipe_seconds(wiper_cfg: dict[str, Any]) -> float:
    home_angle = float(wiper_cfg.get("home_angle", 90.0))
    press_angle = float(wiper_cfg.get("press_angle", 45.0))
    sweep_left = float(wiper_cfg.get("sweep_left", 30.0))
    sweep_right = float(wiper_cfg.get("sweep_right", 150.0))
    sweep_passes = int(wiper_cfg.get("sweep_passes", 2))
    sweep_speed = float(wiper_cfg.get("sweep_speed", 0.01))

    arm_extend_degrees = abs(home_angle - press_angle)
    sweep_span_degrees = abs(sweep_right - sweep_left)

    one_pass_seconds = arm_extend_degrees * sweep_speed + sweep_span_degrees * sweep_speed
    return max(1.0, sweep_passes * one_pass_seconds)


class CleaningTrigger:
    """
    Config-driven cleaning sequence:
      1) pump on for mission.pump_duration_s
      2) wiper on for estimated duration from wiper config
    """

    def __init__(
        self,
        spray_pin: int | None,
        wipe_pin: int | None,
        cooldown: float,
        spray_seconds: float,
        wipe_seconds: float,
    ):
        self.spray = _GPIOOutput(spray_pin, "PUMP")
        self.wipe = _GPIOOutput(wipe_pin, "WIPER")

        self.cooldown = cooldown
        self.spray_seconds = spray_seconds
        self.wipe_seconds = wipe_seconds

        self._last_trigger_time = 0.0
        self._lock = threading.Lock()
        self._in_progress = False

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> "CleaningTrigger":
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
        wipe_seconds = _estimated_wipe_seconds(wiper_cfg)

        return cls(
            spray_pin=int(spray_pin),
            wipe_pin=int(wipe_pin),
            cooldown=cooldown,
            spray_seconds=spray_seconds,
            wipe_seconds=wipe_seconds,
        )

    def _worker(self):
        try:
            print("[ACTION] SPRAY → WIPE")

            self.spray.on()
            time.sleep(self.spray_seconds)
            self.spray.off()

            self.wipe.on()
            time.sleep(self.wipe_seconds)
            self.wipe.off()

            self._last_trigger_time = time.time()
        finally:
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
        self.spray.cleanup()
        self.wipe.cleanup()


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
        cleaning = CleaningTrigger.from_config(cfg)
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