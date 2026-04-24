"""
pump_test.py — Pump relay test.

Triggers the pump relay for a configurable duration to confirm the GPIO
pin is wired and the relay closes. No camera, no sensor.

Prerequisites:
    gpiozero installed:
        pip install gpiozero
    mission.pump_gpio_pin set in config.json

Usage:
    python pump_test.py
    python pump_test.py --config config.json
    python pump_test.py --duration 3.0
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
            "AeroClean — pump relay test.\n"
            "\n"
            "Fires the pump relay for a set duration and confirms the GPIO\n"
            "pin responds. Use this to verify the relay is wired correctly\n"
            "before running the full preflight or mission.\n"
            "\n"
            "Troubleshooting:\n"
            "  No-op / pin not set  →  set mission.pump_gpio_pin in config.json\n"
            "  gpiozero error       →  run: pip install gpiozero\n"
            "  Relay clicks but no water  →  check pump power and tube connections\n"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument("--config",   default="config.json", help="Path to config.json.")
    p.add_argument("--duration", type=float, default=None,
                   help="Spray duration in seconds (overrides config value).")
    return p


def main() -> None:
    args = _build_parser().parse_args()

    with open(args.config) as f:
        cfg = json.load(f)

    mission_cfg = cfg.get("mission", {})
    pump_pin    = mission_cfg.get("pump_gpio_pin")
    duration    = args.duration or float(mission_cfg.get("pump_duration_s", 5.0))

    if pump_pin is None:
        print("[PUMP TEST] mission.pump_gpio_pin is not set in config.json — cannot run test.")
        print("[PUMP TEST] Set it to the BCM GPIO pin number for the relay and retry.")
        sys.exit(1)

    _banner([
        [
            "AeroClean — Pump Test",
            "Verifies the pump relay fires on the configured GPIO pin",
        ],
        [
            f"GPIO pin : BCM {pump_pin}",
            f"Duration : {duration:.1f} s",
        ],
        [
            "Expect : relay clicks ON, pump runs, relay clicks OFF",
            "Stop   : Ctrl+C",
        ],
    ])

    from pump import Pump
    pump = Pump(pin=int(pump_pin))

    try:
        input("\n  Press Enter to fire the pump...\n")
        print(f"[PUMP TEST] Firing pump on pin {pump_pin} for {duration:.1f}s...")
        pump.spray(duration)
        print("[PUMP TEST] Done — relay should have clicked OFF.")
        print("[PUMP TEST] If you heard two relay clicks and saw water, the pump is confirmed.")
    except KeyboardInterrupt:
        print("\n[PUMP TEST] Stopped.")
        pump.stop()
    finally:
        pump.cleanup()


if __name__ == "__main__":
    main()
