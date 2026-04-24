"""
wiper_test.py — Wiper relay test.

Triggers the wiper relay for a configurable duration to confirm the GPIO
pin is wired and the relay closes. No camera, no sensor.

Prerequisites:
    gpiozero installed:
        pip install gpiozero
    wiper.wiper_gpio_pin set in config.json

Usage:
    python wiper_test.py
    python wiper_test.py --config config.json
    python wiper_test.py --duration 3.0
    Ctrl+C to stop.
"""

from __future__ import annotations

import argparse
import json
import sys


def _banner(groups: list[list[str]]) -> None:
    width = max(len(l) for g in groups for l in g) + 4
    sep   = "═" * width
    print(f"╔{sep}╗")
    for i, group in enumerate(groups):
        for line in group:
            print(f"║  {line:<{width - 2}}║")
        if i < len(groups) - 1:
            print(f"╠{sep}╣")
    print(f"╚{sep}╝")


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "AeroClean — wiper relay test.\n"
            "\n"
            "Fires the wiper relay for a set duration and confirms the GPIO\n"
            "pin responds. Use this to verify the relay is wired correctly\n"
            "before running the full preflight or mission.\n"
            "\n"
            "Troubleshooting:\n"
            "  No-op / pin not set  →  set wiper.wiper_gpio_pin in config.json\n"
            "  gpiozero error       →  run: pip install gpiozero\n"
            "  Relay clicks but arm does not move  →  check wiper power and mechanical linkage\n"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument("--config",   default="config.json", help="Path to config.json.")
    p.add_argument("--duration", type=float, default=None,
                   help="Wipe duration in seconds (overrides config value).")
    return p


def main() -> None:
    args = _build_parser().parse_args()

    with open(args.config) as f:
        cfg = json.load(f)

    w_cfg    = cfg.get("wiper", {})
    wipe_pin = w_cfg.get("wiper_gpio_pin")
    duration = args.duration or float(w_cfg.get("wipe_duration_s", 2.0))

    if wipe_pin is None:
        print("[WIPER TEST] wiper.wiper_gpio_pin is not set in config.json — cannot run test.")
        print("[WIPER TEST] Set it to the BCM GPIO pin number for the relay and retry.")
        sys.exit(1)

    _banner([
        [
            "AeroClean — Wiper Test",
            "Verifies the wiper relay fires on the configured GPIO pin",
        ],
        [
            f"GPIO pin : BCM {wipe_pin}",
            f"Duration : {duration:.1f} s",
        ],
        [
            "Expect : relay clicks ON, wiper arm moves, relay clicks OFF",
            "Stop   : Ctrl+C",
        ],
    ])

    from wiper import Wiper
    wiper = Wiper(pin=int(wipe_pin), wipe_duration_s=duration)

    try:
        input("\n  Press Enter to fire the wiper...\n")
        print(f"[WIPER TEST] Firing wiper on pin {wipe_pin} for {duration:.1f}s...")
        wiper.wipe()
        print("[WIPER TEST] Done — relay should have clicked OFF.")
        print("[WIPER TEST] If you heard two relay clicks and the arm moved, the wiper is confirmed.")
    except KeyboardInterrupt:
        print("\n[WIPER TEST] Stopped.")
        wiper.off()
    finally:
        wiper.cleanup()


if __name__ == "__main__":
    main()
