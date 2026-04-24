"""
camera_test.py — Pi camera live test.

Shows a live OpenCV window and prints FPS to the terminal.

Usage:
    python camera_test.py
    python camera_test.py --config config.json
    Press  q  to quit.
"""

from __future__ import annotations

import argparse
import time

import cv2

from camera import Camera


def main() -> None:
    p = argparse.ArgumentParser(description="Pi camera live test")
    p.add_argument("--config", default="config.json")
    args = p.parse_args()

    print("[CAM TEST] Opening camera — press  q  to quit")

    frame_count = 0
    t_start     = time.monotonic()
    t_last_fps  = t_start
    last_fps_i  = None

    with Camera(args.config) as cam:
        cv2.namedWindow("AeroClean — Camera Test", cv2.WINDOW_NORMAL)

        while True:
            frame = cam.capture()
            frame_count += 1

            now = time.monotonic()
            if now - t_last_fps >= 1.0:
                fps   = frame_count / (now - t_start)
                fps_i = int(fps)
                if fps_i != last_fps_i:
                    h, w = frame.shape[:2]
                    print(f"[CAM TEST] {w}x{h}  {fps:.1f} FPS")
                    last_fps_i = fps_i
                t_last_fps = now

            cv2.imshow("AeroClean — Camera Test", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("[CAM TEST] Quit.")
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
