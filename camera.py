"""
camera.py — Arducam / Raspberry Pi Camera Module 3 (IMX708) capture wrapper.

Uses picamera2 (libcamera backend, pre-installed on RPi OS Bookworm).
Returns BGR numpy arrays compatible with OpenCV and Ultralytics.
"""

import json
import numpy as np

# picamera2 is only available on the Raspberry Pi.
# On other platforms the import will fail — main.py handles this gracefully
# when --source is used for offline testing.
try:
    from picamera2 import Picamera2
    from libcamera import controls as libcontrols
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False


def _load_config(path: str = "config.json") -> dict:
    with open(path) as f:
        return json.load(f)


class Camera:
    """
    Thin wrapper around picamera2 for the IMX708 sensor.

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
        if not PICAMERA_AVAILABLE:
            raise RuntimeError(
                "picamera2 is not installed. On the Raspberry Pi run:\n"
                "  sudo apt install python3-picamera2\n"
                "On a desktop use --source <image_or_video> for offline testing."
            )

        cfg = _load_config(config_path)
        self._width, self._height = cfg.get("resolution", [1920, 1080])
        self._framerate = cfg.get("framerate", 30)

        self._cam = Picamera2()
        video_cfg = self._cam.create_video_configuration(
            main={"size": (self._width, self._height), "format": "RGB888"},
            controls={"FrameRate": self._framerate},
        )
        self._cam.configure(video_cfg)

        # IMX708 supports Contrast Detection Auto-Focus (CDAF)
        self._cam.set_controls({
            "AfMode": libcontrols.AfModeEnum.Continuous,
            "AfSpeed": libcontrols.AfSpeedEnum.Fast,
        })

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        """Start the camera stream."""
        self._cam.start()

    def stop(self):
        """Stop the camera stream and release resources."""
        self._cam.stop()
        self._cam.close()

    def __enter__(self):
        self.start()
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
        rgb = self._cam.capture_array()   # RGB from picamera2
        bgr = rgb[:, :, ::-1].copy()      # flip to BGR for OpenCV
        return bgr

    @property
    def resolution(self) -> tuple[int, int]:
        return (self._width, self._height)
