"""
camera.py — Camera capture wrapper supporting two hardware options.

Two implementations, both with the same public API (start / stop / capture):

    _USBCamera  — Camera A (default): Arducam OV2311 2MP Global Shutter USB camera.
                  Uses cv2.VideoCapture (UVC). No drivers or config.txt changes needed.
    _PiCamera   — Camera B: Raspberry Pi Camera Module 3 (IMX708).
                  Uses picamera2 / libcamera. Requires dtoverlay=imx708 in config.txt.

Select via config.json:
    "camera_type": "a"   →  OV2311 USB  (default)
    "camera_type": "b"   →  IMX708 CSI

Call Camera(config_path) — it reads camera_type and returns the right instance.
All callers (main.py, mission.py, tests) use Camera() unchanged.
"""

from __future__ import annotations

import json

import cv2
import numpy as np

"""
picamera2 / libcamera — required for Camera B (IMX708 CSI) only.
Pi-only libraries. The try block below sets PICAMERA_AVAILABLE = False when they
are absent so this file can be imported on a dev machine without crashing.
Camera B raises a clear RuntimeError at __init__ time if the flag is False.
"""
try:
    from picamera2 import Picamera2
    from libcamera import controls as libcontrols
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False


def _load_config(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


# ─────────────────────────────────────────────────────────────────────────────
# Camera A — OV2311 USB (default)
# ─────────────────────────────────────────────────────────────────────────────

class _USBCamera:
    """
    Arducam OV2311 2MP Global Shutter USB camera via cv2.VideoCapture (camera A).

    Reads camera_device, resolution, and framerate from config.json.
    Returns BGR numpy arrays — OpenCV convention, no conversion needed.
    """

    def __init__(self, cfg: dict):
        self._width, self._height = cfg.get("resolution", [1280, 720])
        self._framerate = cfg.get("framerate", 50)
        device = cfg.get("camera_device", 0)

        self._cap = cv2.VideoCapture(device)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(cv2.CAP_PROP_FPS,          self._framerate)

        if not self._cap.isOpened():
            raise RuntimeError(
                f"Camera A: could not open USB camera at device index {device}.\n"
                "  Run: ls /dev/video* before and after plugging in — the new entry is your device.\n"
                "  Then set camera_device in config.json (_s1) to the correct index."
            )

    def start(self):
        """No-op — VideoCapture opens on __init__."""

    def stop(self):
        self._cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.stop()

    def capture(self) -> np.ndarray:
        ret, frame = self._cap.read()
        if not ret:
            raise RuntimeError("Camera A: frame read failed — check USB connection.")
        return frame

    @property
    def resolution(self) -> tuple[int, int]:
        return (self._width, self._height)


# ─────────────────────────────────────────────────────────────────────────────
# Camera B — IMX708 CSI (picamera2)
# ─────────────────────────────────────────────────────────────────────────────

class _PiCamera:
    """
    Raspberry Pi Camera Module 3 (IMX708) via picamera2 (camera B).

    Reads resolution and framerate from config.json.
    Enables Continuous CDAF autofocus (AfMode=Continuous, AfSpeed=Fast).
    Returns BGR numpy arrays — picamera2 gives RGB, flipped here to match OpenCV.

    Requires:
        sudo apt install python3-picamera2
        /boot/firmware/config.txt: camera_auto_detect=0 + dtoverlay=imx708
    """

    def __init__(self, cfg: dict):
        if not PICAMERA_AVAILABLE:
            raise RuntimeError(
                "Camera B (IMX708) requires picamera2 which is not installed.\n"
                "  Install: sudo apt install python3-picamera2\n"
                "  Also ensure /boot/firmware/config.txt has:\n"
                "    camera_auto_detect=0\n"
                "    dtoverlay=imx708\n"
                "  Or switch to Camera A (OV2311 USB) by setting camera_type: 'a' in config.json."
            )

        self._width, self._height = cfg.get("resolution", [1920, 1080])
        self._framerate = cfg.get("framerate", 30)

        self._cam = Picamera2()
        video_cfg = self._cam.create_video_configuration(
            main={"size": (self._width, self._height), "format": "RGB888"},
            controls={"FrameRate": self._framerate},
        )
        self._cam.configure(video_cfg)
        self._cam.set_controls({
            "AfMode": libcontrols.AfModeEnum.Continuous,
            "AfSpeed": libcontrols.AfSpeedEnum.Fast,
        })

    def start(self):
        self._cam.start()

    def stop(self):
        self._cam.stop()
        self._cam.close()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()

    def capture(self) -> np.ndarray:
        rgb = self._cam.capture_array()   # RGB from picamera2
        return rgb[:, :, ::-1].copy()     # flip to BGR for OpenCV

    @property
    def resolution(self) -> tuple[int, int]:
        return (self._width, self._height)


# ─────────────────────────────────────────────────────────────────────────────
# Factory
# ─────────────────────────────────────────────────────────────────────────────

def Camera(config_path: str = "config.json") -> _USBCamera | _PiCamera:
    """
    Return the correct camera instance based on camera_type in config.json.

        "a"  (default) — OV2311 USB via cv2.VideoCapture  (_USBCamera)
        "b"            — IMX708 CSI via picamera2          (_PiCamera)

    Usage (unchanged from before):
        with Camera("config.json") as cam:
            frame = cam.capture()
    """
    cfg = _load_config(config_path)
    camera_type = str(cfg.get("camera_type", "a")).lower()

    if camera_type == "b":
        print("[CAMERA] Camera B — IMX708 CSI (picamera2)")
        return _PiCamera(cfg)

    print("[CAMERA] Camera A — OV2311 USB (cv2.VideoCapture)")
    return _USBCamera(cfg)
