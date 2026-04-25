"""
camera.py — Arducam 2MP Global Shutter USB Camera (OV2311) capture wrapper.

Uses OpenCV VideoCapture (UVC) — no picamera2 or libcamera required.
Returns BGR numpy arrays compatible with OpenCV and Ultralytics.
"""

import json

import cv2
import numpy as np


def _load_config(path: str = "config.json") -> dict:
    with open(path) as f:
        return json.load(f)


class Camera:
    """
    Thin wrapper around cv2.VideoCapture for the OV2311 USB camera.

    Usage:
        cam = Camera()
        cam.start()
        frame = cam.capture()   # numpy BGR array, shape (H, W, 3)
        cam.stop()

    Or as a context manager:
        with Camera() as cam:
            frame = cam.capture()
    """

    def __init__(self, config_path: str = "config.json"):
        cfg = _load_config(config_path)
        self._width, self._height = cfg.get("resolution", [1280, 720])
        self._framerate = cfg.get("framerate", 50)
        device = cfg.get("camera_device", 0)

        self._cap = cv2.VideoCapture(device)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(cv2.CAP_PROP_FPS,          self._framerate)

        if not self._cap.isOpened():
            raise RuntimeError(
                f"Could not open USB camera at device index {device}.\n"
                "  Run: ls /dev/video* to list available devices,\n"
                "  then set camera_device in config.json to the correct index."
            )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        """No-op — VideoCapture opens on __init__."""

    def stop(self):
        """Release the camera."""
        self._cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.stop()

    # ------------------------------------------------------------------
    # Capture
    # ------------------------------------------------------------------

    def capture(self) -> np.ndarray:
        """
        Grab a single frame.

        Returns:
            numpy ndarray, BGR, shape (H, W, 3) — OpenCV convention.
        """
        ret, frame = self._cap.read()
        if not ret:
            raise RuntimeError("Camera read failed — check USB connection.")
        return frame

    @property
    def resolution(self) -> tuple[int, int]:
        return (self._width, self._height)
