"""
main.py — AeroClean entry point.

Switch between the two models with --model:

    python main.py --model ocr            # continuous OCR loop (live camera)
    python main.py --model yolo           # continuous YOLO loop (live camera)
    python main.py --model ocr  --once    # single frame, print result, exit
    python main.py --model yolo --conf 0.45
    python main.py --model yolo --save    # save annotated frames to output/
    python main.py --model ocr  --source board.jpg   # offline image (no camera)
    python main.py --model yolo --source board.mp4   # offline video

Press  q  to quit the live window.
"""

import argparse
import json
import os
import sys
import time
import cv2


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="AeroClean — dirty dry-erase board detector",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument(
        "--mode",
        choices=["inference", "mission"],
        default="inference",
        help="Operating mode:\n"
             "  inference — run OCR or YOLO model loop (default)\n"
             "  mission   — run full MAVLink drone mission state machine",
    )
    p.add_argument(
        "--model", choices=["ocr", "yolo"], default="ocr",
        help="Which model to run (inference mode only):\n  ocr  — Tesseract OCR (finds word 'dirty') [default]\n  yolo — YOLO11n board-state detector",
    )
    p.add_argument(
        "--source", default=None,
        help="Path to an image or video file for offline testing.\n"
             "Omit to use the Raspberry Pi camera.",
    )
    p.add_argument(
        "--once", action="store_true",
        help="Process a single frame, print the result, then exit.",
    )
    p.add_argument(
        "--conf", type=float, default=None,
        help="YOLO confidence threshold (0.0–1.0). Overrides config.json.",
    )
    p.add_argument(
        "--save", action="store_true",
        help="Save every annotated frame to the output/ directory.",
    )
    p.add_argument(
        "--config", default="config.json",
        help="Path to config.json  (default: config.json)",
    )
    return p


# ─────────────────────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────────────────────

def load_config(path: str) -> dict:
    if not os.path.exists(path):
        print(f"[ERROR] config.json not found at '{path}'", file=sys.stderr)
        sys.exit(1)
    with open(path) as f:
        return json.load(f)


# ─────────────────────────────────────────────────────────────────────────────
# Frame source helpers
# ─────────────────────────────────────────────────────────────────────────────

def _source_from_camera(config_path: str):
    """Yield frames from the Raspberry Pi camera."""
    from camera import Camera
    with Camera(config_path) as cam:
        while True:
            yield cam.capture()


def _source_from_file(path: str):
    """Yield frames from an image or video file (desktop testing)."""
    if not os.path.exists(path):
        print(f"[ERROR] Source file not found: {path}", file=sys.stderr)
        sys.exit(1)

    # Single image
    ext = os.path.splitext(path)[1].lower()
    if ext in (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"):
        frame = cv2.imread(path)
        if frame is None:
            print(f"[ERROR] Could not read image: {path}", file=sys.stderr)
            sys.exit(1)
        while True:   # loop so --once can break after the first yield
            yield frame
    else:
        # Video
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


# ─────────────────────────────────────────────────────────────────────────────
# Save helper
# ─────────────────────────────────────────────────────────────────────────────

def _save_frame(frame, output_dir: str, prefix: str):
    os.makedirs(output_dir, exist_ok=True)
    ts = int(time.time() * 1000)
    path = os.path.join(output_dir, f"{prefix}_{ts}.jpg")
    cv2.imwrite(path, frame)


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    args = build_parser().parse_args()

    # ── Mission mode — hand off entirely to the state machine ────────────────
    if args.mode == "mission":
        from mission import Mission
        Mission(config_path=args.config).run()
        return

    cfg = load_config(args.config)

    display = cfg.get("display", True)
    output_dir = cfg.get("output_dir", "output")

    # ── Load the selected model ──────────────────────────────────────────────
    if args.model == "ocr":
        from ocr_model import OCRModel
        model = OCRModel(config_path=args.config)
        print("[INFO] OCR model loaded — searching for the word 'dirty'")
    else:
        from yolo_model import YOLOModel
        model = YOLOModel(config_path=args.config, conf_override=args.conf)
        print("[INFO] YOLO11n model loaded — detecting board state")

    # ── Choose frame source ──────────────────────────────────────────────────
    if args.source:
        frames = _source_from_file(args.source)
    else:
        frames = _source_from_camera(args.config)

    window_name = f"AeroClean — {args.model.upper()}"
    if display:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    print("[INFO] Running. Press  q  to quit.")

    prev_state = None

    # ── Inference loop ───────────────────────────────────────────────────────
    for frame in frames:
        result, annotated = model.run(frame)

        # Print only on state change ------------------------------------------
        if args.model == "ocr":
            state = bool(result)
            if state != prev_state:
                print(f"[OCR]  active state — {'dirty' if state else 'clean'}")
                prev_state = state
        else:
            state = result["class_name"] if result else None
            if state != prev_state:
                if result:
                    print(f"[YOLO] active state — {result['class_name']}  conf={result['confidence']:.2f}")
                else:
                    print("[YOLO] active state — no board detected")
                prev_state = state

        # Save ----------------------------------------------------------------
        if args.save:
            prefix = "dirty" if (args.model == "ocr" and result) else args.model
            _save_frame(annotated, output_dir, prefix)

        # Display -------------------------------------------------------------
        if display:
            cv2.imshow(window_name, annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("[INFO] Quit.")
                break

        # Single-frame mode ---------------------------------------------------
        if args.once:
            break

    if display:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
