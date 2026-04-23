"""
main.py — AeroClean entry point.

Switch between the two modes with --mode:

    python main.py --mode inference --model ocr
    python main.py --mode inference --model yolo --conf 0.45
    python main.py --mode inference --model ocr --once
    python main.py --mode inference --model yolo --save
    python main.py --mode inference --model yolo --source board.mp4

    python main.py --mode mission

Press q to quit the live window in inference mode.
"""

from __future__ import annotations

import argparse
import json
import os
import sys


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
        "--model",
        choices=["ocr", "yolo"],
        default="ocr",
        help="Which model to run in inference mode:\n"
             "  ocr  — Tesseract OCR (finds word 'dirty') [default]\n"
             "  yolo — YOLO board-state detector",
    )
    p.add_argument(
        "--source",
        default=None,
        help="Path to an image or video file for offline testing.\n"
             "Omit to use the Raspberry Pi camera.",
    )
    p.add_argument(
        "--once",
        action="store_true",
        help="Process a single frame, print the result, then exit.",
    )
    p.add_argument(
        "--conf",
        type=float,
        default=None,
        help="YOLO confidence threshold (0.0–1.0). Overrides config.json.",
    )
    p.add_argument(
        "--save",
        action="store_true",
        help="Save every annotated frame to the output/ directory.",
    )
    p.add_argument(
        "--config",
        default="config.json",
        help="Path to config.json (default: config.json)",
    )
    return p


def load_config(path: str) -> dict:
    if not os.path.exists(path):
        print(f"[ERROR] config.json not found at '{path}'", file=sys.stderr)
        sys.exit(1)
    with open(path, "r") as f:
        return json.load(f)


def main():
    args = build_parser().parse_args()

    if args.mode == "mission":
        from mission import Mission
        Mission(config_path=args.config).run()
        return

    cfg = load_config(args.config)

    display = cfg.get("display", True)
    output_dir = cfg.get("output_dir", "output")

    from bare_inference import run_bare_inference

    run_bare_inference(
        config_path=args.config,
        model_name=args.model,
        source=args.source,
        once=args.once,
        conf_override=args.conf,
        save=args.save,
        display=display,
        output_dir=output_dir,
    )


if __name__ == "__main__":
    main()