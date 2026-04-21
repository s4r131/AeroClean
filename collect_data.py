"""
collect_data.py — Dataset collection helper for YOLO training.

Captures images from the Raspberry Pi camera and saves them into
images/raw/<class_name>/ for later labeling in Label Studio.

Usage
-----
    # Capture 100 images of a CLEAN board (press SPACE to capture, q to quit)
    python collect_data.py --class clean_board --count 100

    # Capture 100 images of a DIRTY board
    python collect_data.py --class dirty_board --count 100

    # Burst mode: auto-capture every 2 seconds (no key press needed)
    python collect_data.py --class dirty_board --count 50 --interval 2

Tips for a good dataset
------------------------
  - Vary lighting: overhead fluorescent, natural light, lamp angle
  - Vary board coverage: lightly smudged, heavily marked, partially erased
  - Vary angles: straight-on, slight left/right tilt (15°)
  - Capture at the same distance you'll use in production
  - Aim for 100–200 images per class minimum
"""

import argparse
import os
import sys
import time
import cv2


SAVE_ROOT = "images/raw"


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Capture training images for YOLO")
    p.add_argument(
        "--class", dest="cls", required=True,
        choices=["clean_board", "dirty_board"],
        help="Class label for the captured images",
    )
    p.add_argument(
        "--count", type=int, default=100,
        help="Number of images to capture (default: 100)",
    )
    p.add_argument(
        "--interval", type=float, default=None,
        help="Auto-capture interval in seconds. "
             "Omit to use manual SPACE-to-capture mode.",
    )
    p.add_argument(
        "--config", default="config.json",
        help="Path to config.json (default: config.json)",
    )
    return p


def main():
    args = build_parser().parse_args()

    save_dir = os.path.join(SAVE_ROOT, args.cls)
    os.makedirs(save_dir, exist_ok=True)

    from camera import Camera

    mode = "auto" if args.interval else "manual"
    print(f"[INFO] Saving to: {save_dir}/")
    print(f"[INFO] Mode: {mode}")
    if mode == "manual":
        print("[INFO] Press SPACE to capture a frame, q to quit.")
    else:
        print(f"[INFO] Auto-capturing every {args.interval}s. Press q to quit early.")

    captured = 0
    window = "collect_data.py"

    with Camera(args.config) as cam:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        last_capture = 0.0

        while captured < args.count:
            frame = cam.capture()

            preview = frame.copy()
            cv2.putText(preview, f"{args.cls}  {captured}/{args.count}", (10, 36),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imshow(window, preview)

            key = cv2.waitKey(1) & 0xFF
            now = time.time()

            do_capture = False
            if mode == "manual" and key == ord(" "):
                do_capture = True
            elif mode == "auto" and (now - last_capture) >= args.interval:
                do_capture = True

            if do_capture:
                ts = int(now * 1000)
                filename = os.path.join(save_dir, f"{args.cls}_{ts}.jpg")
                cv2.imwrite(filename, frame)
                captured += 1
                last_capture = now
                print(f"  [{captured}/{args.count}] Saved {filename}")

            if key == ord("q"):
                print("[INFO] Quit early.")
                break

    cv2.destroyAllWindows()
    print(f"\n[DONE] Captured {captured} images to {save_dir}/")
    if captured < args.count:
        print(f"       (Target was {args.count} — run again to collect more)")


if __name__ == "__main__":
    main()
