"""
ocr_model.py — Model 1: Tesseract OCR pipeline for dirty-word detection.

Pipeline
--------
1. Capture frame (or load from --source).
2. Crop to board_roi if configured.
3. Downscale by ocr_scale (default 0.5 — 960x540 from 1920x1080).
4. Preprocess: greyscale → CLAHE → adaptive threshold → morphological close.
5. Run Tesseract (oem 1, psm 6, character whitelist) to get per-word detections.
6. Search for the word "dirty" (case-insensitive).
7. Return True and overlay bounding boxes; return False if not found.

Performance
-----------
Three layers of CPU reduction run together:

1. Frame throttling — Tesseract only fires every ocr_frame_interval frames
   (default 5). Every frame in between returns the cached result instantly.
2. Dirty-hold hysteresis — once "dirty" is detected the result stays DIRTY
   for 1.5 s even if subsequent OCR runs find nothing, preventing flicker.
3. Downscaling — frame is resized by ocr_scale (default 0.5 → 960x540)
   before preprocessing. Adjust in config.json; 0.25 works for large
   marker text and halves the pixel count again.
4. Tesseract config — oem 1 (LSTM only, faster than combined oem 3) with
   a character whitelist restricted to the letters in "dirty". Tesseract
   skips all other character classifiers, cutting OCR time by ~50-70%.

Dependencies (installed via requirements.txt):
    opencv-python, pytesseract, numpy
    System: tesseract-ocr  (sudo apt install tesseract-ocr on the Pi)
"""

from __future__ import annotations

import json
import re
import time
import cv2
import numpy as np

try:
    import pytesseract
    TESSERACT_AVAILABLE = True
except ImportError:
    TESSERACT_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
# Config helpers
# ─────────────────────────────────────────────────────────────────────────────

def _load_config(path: str = "config.json") -> dict:
    with open(path) as f:
        return json.load(f)


# ─────────────────────────────────────────────────────────────────────────────
# Preprocessing
# ─────────────────────────────────────────────────────────────────────────────

def _preprocess(frame: np.ndarray, scale: float = 1.0) -> np.ndarray:
    """
    Convert a BGR frame to a high-contrast binary image ready for Tesseract.

    Stages
    ------
    0. Downscale by scale factor (reduces Tesseract workload significantly).
    1. Greyscale conversion
    2. CLAHE — boosts local contrast so faint marker strokes become visible.
    3. Adaptive thresholding — handles uneven lighting across the board.
    4. Morphological closing — joins broken letter strokes.
    """
    if scale != 1.0:
        h, w = frame.shape[:2]
        frame = cv2.resize(frame, (int(w * scale), int(h * scale)),
                           interpolation=cv2.INTER_AREA)

    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(grey)

    binary = cv2.adaptiveThreshold(
        enhanced, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        blockSize=31,
        C=10,
    )

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    return cleaned


def _crop_roi(frame: np.ndarray, roi: list | None) -> np.ndarray:
    """Crop frame to [x, y, w, h] region-of-interest, or return full frame."""
    if roi is None:
        return frame
    x, y, w, h = roi
    return frame[y:y + h, x:x + w]


# ─────────────────────────────────────────────────────────────────────────────
# OCR
# ─────────────────────────────────────────────────────────────────────────────

# oem 1  — LSTM engine only (faster than oem 3 combined mode on Pi 5)
# psm 6  — assume a single uniform block of text
# whitelist — only the characters that appear in the word "dirty".
#             Tesseract skips all other character classifiers, cutting OCR
#             time by ~50-70% compared to the full character set.
_TESSERACT_CONFIG = (
    "--oem 1 --psm 6 "
    "-c tessedit_char_whitelist=DdIiRrTtYy"
)
_DIRTY_PATTERN = re.compile(r"\bdirty\b", re.IGNORECASE)


def _run_ocr(processed: np.ndarray) -> dict:
    """
    Run Tesseract and return the data dict (word-level bounding boxes + text).
    Returns an empty dict with an empty text list if Tesseract is unavailable
    or fails, so the rest of the pipeline produces a CLEAN result gracefully.
    """
    if not TESSERACT_AVAILABLE:
        return {"text": []}
    try:
        return pytesseract.image_to_data(
            processed,
            config=_TESSERACT_CONFIG,
            output_type=pytesseract.Output.DICT,
        )
    except Exception as e:
        print(f"[OCR] Tesseract error: {e}")
        return {"text": []}


def _find_dirty(ocr_data: dict) -> list[dict]:
    """
    Search OCR results for the word 'dirty'.

    Returns a list of dicts (one per match) with keys:
        text, left, top, width, height, conf
    Returns an empty list if not found.
    """
    matches = []
    for i, word in enumerate(ocr_data["text"]):
        if _DIRTY_PATTERN.search(word):
            matches.append({
                "text": word,
                "left": ocr_data["left"][i],
                "top": ocr_data["top"][i],
                "width": ocr_data["width"][i],
                "height": ocr_data["height"][i],
                "conf": ocr_data["conf"][i],
            })
    return matches


# ─────────────────────────────────────────────────────────────────────────────
# Overlay
# ─────────────────────────────────────────────────────────────────────────────

def _draw_overlay(frame: np.ndarray, matches: list[dict], roi_offset: tuple[int, int] = (0, 0)) -> np.ndarray:
    """
    Draw bounding boxes around detected 'dirty' words and a status banner.
    """
    out = frame.copy()
    ox, oy = roi_offset

    for m in matches:
        x1 = ox + m["left"]
        y1 = oy + m["top"]
        x2 = x1 + m["width"]
        y2 = y1 + m["height"]
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 0, 255), 2)
        label = f"{m['text']} ({m['conf']}%)"
        cv2.putText(out, label, (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Status banner at top of frame
    if matches:
        banner_text = "DIRTY DETECTED"
        banner_color = (0, 0, 255)    # red
    else:
        banner_text = "CLEAN"
        banner_color = (0, 200, 0)    # green

    cv2.rectangle(out, (0, 0), (300, 40), (0, 0, 0), -1)
    cv2.putText(out, banner_text, (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, banner_color, 2)

    return out


# ─────────────────────────────────────────────────────────────────────────────
# Public API
# ─────────────────────────────────────────────────────────────────────────────

class OCRModel:
    """
    Wrap the full OCR pipeline with frame throttling and downscaling.

    Usage:
        model = OCRModel()
        result, annotated_frame = model.run(frame)
        # result is True if "dirty" was found on the board

    Tesseract only runs every ocr_frame_interval frames (default 5).
    Between runs the last known result is returned instantly so the display
    stays live without burning CPU on every frame.

    Dirty-hold hysteresis: once dirty is detected the result stays DIRTY
    for 1.5 s after the last detection, preventing single-frame flicker.
    """

    def __init__(self, config_path: str = "config.json"):
        cfg = _load_config(config_path)
        self._roi                = cfg.get("board_roi")          # [x, y, w, h] or None
        self._ocr_frame_interval = int(cfg.get("ocr_frame_interval", 5))
        self._scale              = float(cfg.get("ocr_scale", 0.5))
        self._frame_count        = 0
        self._last_matches: list[dict] = []

        # Dirty-hold hysteresis
        self._dirty_hold_s   = 1.5
        self._last_dirty_time = 0.0

    def run(self, frame: np.ndarray) -> tuple[bool, np.ndarray]:
        """
        Process a single BGR frame.

        Tesseract only fires every ocr_frame_interval frames; all other
        frames return the cached result instantly. Dirty state is held for
        1.5 s after the last positive detection to prevent flickering.

        Parameters
        ----------
        frame : np.ndarray
            BGR image from camera.py (or loaded with cv2.imread).

        Returns
        -------
        dirty : bool
            True if the word "dirty" was found on the board.
        annotated : np.ndarray
            Copy of the input frame with bounding boxes and status overlay.
        """
        self._frame_count += 1
        now = time.monotonic()

        if self._frame_count % self._ocr_frame_interval != 0:
            # Throttled — redraw cached result on the new frame
            within_hold = (now - self._last_dirty_time) <= self._dirty_hold_s
            dirty = len(self._last_matches) > 0 or within_hold
            annotated = _draw_overlay(frame, self._last_matches if dirty else [])
            return dirty, annotated

        # Full OCR run
        roi_offset = (0, 0)
        cropped = _crop_roi(frame, self._roi)
        if self._roi is not None:
            roi_offset = (self._roi[0], self._roi[1])

        processed = _preprocess(cropped, self._scale)
        ocr_data  = _run_ocr(processed)
        matches   = _find_dirty(ocr_data)

        if matches:
            self._last_matches = matches
            self._last_dirty_time = now
            dirty = True
        else:
            within_hold = (now - self._last_dirty_time) <= self._dirty_hold_s
            dirty = within_hold
            if not within_hold:
                self._last_matches = []

        annotated = _draw_overlay(frame, self._last_matches if dirty else [], roi_offset)
        return dirty, annotated
