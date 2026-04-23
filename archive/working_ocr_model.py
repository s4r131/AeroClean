"""
ocr_model.py — Model 1: Tesseract OCR pipeline for dirty-word detection.
"""

from __future__ import annotations

import json
import re
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
# Resize (returns scale)
# ─────────────────────────────────────────────────────────────────────────────

def _resize(frame: np.ndarray, max_width: int = 640):
    h, w = frame.shape[:2]
    if w <= max_width:
        return frame, 1.0
    scale = max_width / w
    resized = cv2.resize(frame, (int(w * scale), int(h * scale)))
    return resized, scale


# ─────────────────────────────────────────────────────────────────────────────
# Preprocessing
# ─────────────────────────────────────────────────────────────────────────────

def _preprocess(frame: np.ndarray) -> np.ndarray:
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
    if roi is None:
        return frame
    x, y, w, h = roi
    return frame[y:y + h, x:x + w]


# ─────────────────────────────────────────────────────────────────────────────
# OCR
# ─────────────────────────────────────────────────────────────────────────────

_TESSERACT_CONFIG = "--oem 3 --psm 6"
_DIRTY_PATTERN = re.compile(r"\bdirty\b", re.IGNORECASE)


def _run_ocr(processed: np.ndarray) -> dict:
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
    matches = []
    # print(ocr_data["text"]) # debugging use only
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
# Overlay (FIXED scaling)
# ─────────────────────────────────────────────────────────────────────────────

def _draw_overlay(frame: np.ndarray, matches: list[dict],
                  roi_offset: tuple[int, int] = (0, 0),
                  scale: float = 1.0) -> np.ndarray:

    out = frame.copy()
    ox, oy = roi_offset

    for m in matches:
        # scale coordinates back to original frame
        x1 = ox + int(m["left"] / scale)
        y1 = oy + int(m["top"] / scale)
        x2 = x1 + int(m["width"] / scale)
        y2 = y1 + int(m["height"] / scale)

        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 0, 255), 2)
        label = f"{m['text']} ({m['conf']}%)"
        cv2.putText(out, label, (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    #if matches:
     #   banner_text = "DIRTY DETECTED"
       # banner_color = (0, 0, 255)
    #else:
     #   banner_text = "CLEAN"
      #  banner_color = (0, 200, 0)

   # cv2.rectangle(out, (0, 0), (300, 40), (0, 0, 0), -1)
   # cv2.putText(out, banner_text, (10, 28),
    #            cv2.FONT_HERSHEY_SIMPLEX, 0.9, banner_color, 2)

    return out


# ─────────────────────────────────────────────────────────────────────────────
# Public API
# ─────────────────────────────────────────────────────────────────────────────

class OCRModel:
    def __init__(self, config_path: str = "config.json"):
        cfg = _load_config(config_path)
        self._roi = cfg.get("board_roi")

        self._frame_count = 0
        self._last_result = False
        self._last_matches = []   # so boxes persist
        self._last_scale = 1.0

    def run(self, frame: np.ndarray) -> tuple[bool, np.ndarray]:
        self._frame_count += 1

        roi_offset = (0, 0)
        cropped = _crop_roi(frame, self._roi)

        if self._roi is not None:
            roi_offset = (self._roi[0], self._roi[1])

        # ── Skip OCR most frames
        if self._frame_count % 5 != 0:
            annotated = _draw_overlay(
                frame,
                self._last_matches,
                roi_offset,
                self._last_scale
            )
            return self._last_result, annotated

        # ── Downscale
        cropped, scale = _resize(cropped)

        processed = _preprocess(cropped)
        ocr_data = _run_ocr(processed)
        matches = _find_dirty(ocr_data)

        # store for skipped frames
        self._last_matches = matches
        self._last_scale = scale

        annotated = _draw_overlay(frame, matches, roi_offset, scale)
        dirty = len(matches) > 0
        self._last_result = dirty

        return dirty, annotated