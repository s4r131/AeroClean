"""
yolo_model.py — Model 2: YOLO26n board-state detector.

Detects two classes:
    0  clean_board  — erased, usable board surface
    1  dirty_board  — marker residue, ghost marks, or heavy smudging

Model
-----
YOLO26n (Ultralytics 2026) exported to NCNN format for Raspberry Pi 5.
NCNN runs entirely on the ARM Cortex-A76 CPU — no GPU required.
NCNN is the fastest export format on Pi 5 (~68ms/frame at 640×640).

Weights expected at: weights/best_ncnn_model/  (see README — Training)

Dependencies (requirements.txt):
    ultralytics, opencv-python, numpy
"""

from __future__ import annotations

import json
import os
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

CLASS_NAMES = {0: "clean_board", 1: "dirty_board"}

# BGR colours per class
CLASS_COLORS = {
    "clean_board": (0, 200, 0),    # green
    "dirty_board": (0, 0, 255),    # red
}


# ─────────────────────────────────────────────────────────────────────────────
# Config helper
# ─────────────────────────────────────────────────────────────────────────────

def _load_config(path: str = "config.json") -> dict:
    with open(path) as f:
        return json.load(f)


# ─────────────────────────────────────────────────────────────────────────────
# Overlay
# ─────────────────────────────────────────────────────────────────────────────

def _draw_detection(frame: np.ndarray, detection: dict | None) -> np.ndarray:
    """Draw bounding box, class label, and confidence on a copy of frame."""
    out = frame.copy()

    if detection is None:
        cv2.rectangle(out, (0, 0), (360, 40), (0, 0, 0), -1)
        cv2.putText(out, "NO BOARD DETECTED", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 180), 2)
        return out

    x1, y1, x2, y2 = detection["bbox"]
    cls = detection["class_name"]
    conf = detection["confidence"]
    color = CLASS_COLORS.get(cls, (255, 255, 0))

    cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
    label = f"{cls}  {conf:.0%}"
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    cv2.rectangle(out, (x1, y1 - th - 10), (x1 + tw + 6, y1), color, -1)
    cv2.putText(out, label, (x1 + 3, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # Status banner
    cv2.rectangle(out, (0, 0), (340, 40), (0, 0, 0), -1)
    cv2.putText(out, cls.upper().replace("_", " "), (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    return out


# ─────────────────────────────────────────────────────────────────────────────
# Public API
# ─────────────────────────────────────────────────────────────────────────────

class YOLOModel:
    """
    YOLO26n inference wrapper for board-state detection.

    Usage:
        model = YOLOModel()
        detection, annotated = model.run(frame)
        # detection is a dict or None
        # detection["class_name"] is "dirty_board" or "clean_board"

    Parameters
    ----------
    config_path : str
        Path to config.json.
    conf_override : float | None
        Override the confidence threshold from config.json.
    """

    def __init__(self, config_path: str = "config.json", conf_override: float | None = None):
        if not ULTRALYTICS_AVAILABLE:
            raise RuntimeError(
                "ultralytics is not installed. Run:\n"
                "  pip install ultralytics"
            )

        cfg = _load_config(config_path)
        weights = cfg.get("yolo_weights", "weights/best_ncnn_model")
        self._conf = conf_override if conf_override is not None else cfg.get("yolo_conf", 0.45)

        if not os.path.exists(weights):
            raise FileNotFoundError(
                f"YOLO weights not found at '{weights}'.\n"
                "Train and export the model first — see README: Training walkthrough."
            )

        self._model = YOLO(weights, task="detect")

    def run(self, frame: np.ndarray) -> tuple[dict | None, np.ndarray]:
        """
        Run inference on a single BGR frame.

        Parameters
        ----------
        frame : np.ndarray
            BGR image (from camera.py or cv2.imread).

        Returns
        -------
        detection : dict | None
            Best detection (highest confidence) as:
                {
                    "class_name": str,       # "clean_board" or "dirty_board"
                    "confidence": float,     # 0.0 – 1.0
                    "bbox": [x1, y1, x2, y2] # pixel coords
                }
            None if no board detected above the confidence threshold.
        annotated : np.ndarray
            Copy of frame with drawn bounding box and status overlay.
        """
        results = self._model.predict(frame, conf=self._conf, imgsz=640, verbose=False)
        detection = self._parse(results)
        annotated = _draw_detection(frame, detection)
        return detection, annotated

    def _parse(self, results) -> dict | None:
        """Extract the highest-confidence detection from Ultralytics results."""
        if not results or results[0].boxes is None:
            return None

        boxes = results[0].boxes
        if len(boxes) == 0:
            return None

        # Pick the detection with the highest confidence
        confs = boxes.conf.cpu().numpy()
        best_idx = int(confs.argmax())

        cls_id = int(boxes.cls[best_idx].cpu().numpy())
        conf = float(confs[best_idx])
        xyxy = boxes.xyxy[best_idx].cpu().numpy().astype(int).tolist()

        return {
            "class_name": CLASS_NAMES.get(cls_id, str(cls_id)),
            "confidence": conf,
            "bbox": xyxy,
        }
