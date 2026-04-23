"""
main.py — AeroClean entry point (Pi 5 compatible, gpiozero).
"""

import argparse
import json
import os
import sys
import time
import cv2
from gpiozero import LED


# ─────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="AeroClean — dirty dry-erase board detector",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument("--mode", choices=["inference", "mission"], default="inference")
    p.add_argument("--model", choices=["ocr", "yolo", "simple"], default=None)
    p.add_argument("--source", default=None)
    p.add_argument("--once", action="store_true")
    p.add_argument("--conf", type=float, default=None)
    p.add_argument("--save", action="store_true")
    p.add_argument("--config", default="config.json")
    return p


# ─────────────────────────────────────────────────────────────

def load_config(path: str) -> dict:
    if not os.path.exists(path):
        print(f"[ERROR] config.json not found at '{path}'")
        sys.exit(1)
    with open(path) as f:
        return json.load(f)


# ─────────────────────────────────────────────────────────────

def _source_from_camera(config_path: str):
    from camera import Camera
    with Camera(config_path) as cam:
        while True:
            yield cam.capture()


def _source_from_file(path: str):
    if not os.path.exists(path):
        print(f"[ERROR] Source file not found: {path}")
        sys.exit(1)

    ext = os.path.splitext(path)[1].lower()

    if ext in (".jpg", ".jpeg", ".png", ".bmp"):
        frame = cv2.imread(path)
        while True:
            yield frame
    else:
        cap = cv2.VideoCapture(path)
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            yield frame


# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────

def main():
    args = build_parser().parse_args()

    if args.model is None:
        print("[ERROR] --model required (ocr, yolo, simple)")
        sys.exit(1)

    cfg = load_config(args.config)

    display = cfg.get("display", True)

    # ── LOAD MODEL ─────────────────────────────────────────
    if args.model == "ocr":
        from ocr_model import OCRModel
        model = OCRModel(config_path=args.config)
        print("[INFO] OCR model loaded")

    elif args.model == "simple":
        from simple_model import SimpleModel
        model = SimpleModel(config_path=args.config)
        print("[INFO] Simple model loaded")

    else:
        from yolo_model import YOLOModel
        model = YOLOModel(config_path=args.config, conf_override=args.conf)
        print("[INFO] YOLO model loaded")

    # ── GPIO (gpiozero) SETUP ───────────────────────────────
    spray = LED(17)
    wipe = LED(18)

    # ── CONTROL LOGIC ──────────────────────────────────────
    last_trigger_time = 0
    COOLDOWN = 5  # seconds
    prev_result = False

    def trigger_cleaning():
        nonlocal last_trigger_time

        now = time.time()
        if now - last_trigger_time < COOLDOWN:
            return

        print("[ACTION] SPRAY → WIPE")

        spray.on()
        time.sleep(2)
        spray.off()

        wipe.on()
        time.sleep(2)
        wipe.off()

        last_trigger_time = now

    # ── FRAME SOURCE ───────────────────────────────────────
    frames = _source_from_file(args.source) if args.source else _source_from_camera(args.config)

    # ── LOOP ───────────────────────────────────────────────
    for frame in frames:
        result, annotated = model.run(frame)

        # Logging
        if args.model in ["ocr", "simple"]:
            print(f"[{args.model.upper()}] {'DIRTY' if result else 'CLEAN'}")
        else:
            print(f"[YOLO] {result if result else 'No detection'}")

        # 🔥 Trigger only on CLEAN → DIRTY
        if args.model == "ocr" and result and not prev_result:
            trigger_cleaning()

        prev_result = result

        # Display
        if display:
            cv2.imshow("AeroClean", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        if args.once:
            break

    if display:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()