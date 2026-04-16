"""
wiper.py — Wiper arm controller for AeroClean.

TODO: Actuator type not yet decided. This is a placeholder skeleton.
      Implement extend(), sweep(), and retract() once the mechanism
      and control interface are confirmed.

The mission state machine calls wipe() during the CLEAN state:
    wiper.wipe()   # extend → sweep → retract

Usage:
    wiper = Wiper.from_config("config.json")
    wiper.wipe()
    wiper.cleanup()
"""

from __future__ import annotations

import json


class Wiper:
    """
    Wiper arm placeholder.

    Public API is fixed — mission.py calls wipe(), which runs the full
    extend → sweep → retract sequence. Fill in the implementation once
    the actuator type and wiring are confirmed.
    """

    def __init__(
        self,
        pin: int | None,
        home_angle: float    = 90.0,
        press_angle: float   = 45.0,
        sweep_left: float    = 30.0,
        sweep_right: float   = 150.0,
        sweep_passes: int    = 2,
        sweep_speed: float   = 0.01,
    ):
        self._pin          = pin
        self._home_angle   = home_angle
        self._press_angle  = press_angle
        self._sweep_left   = sweep_left
        self._sweep_right  = sweep_right
        self._sweep_passes = sweep_passes
        self._sweep_speed  = sweep_speed

        if pin is None:
            print("[WIPER] GPIO pin not yet assigned — running as no-op. Set 'wiper_gpio_pin' in config.json.")
        else:
            print(f"[WIPER] Wiper initialised on pin {pin} — actuator implementation TBD.")

    @classmethod
    def from_config(cls, config_path: str = "config.json") -> "Wiper":
        """Construct a Wiper from the 'wiper' block in config.json."""
        with open(config_path) as f:
            cfg = json.load(f)
        w = cfg.get("wiper", {})
        return cls(
            pin          = w.get("wiper_gpio_pin"),
            home_angle   = float(w.get("home_angle",   90.0)),
            press_angle  = float(w.get("press_angle",  45.0)),
            sweep_left   = float(w.get("sweep_left",   30.0)),
            sweep_right  = float(w.get("sweep_right",  150.0)),
            sweep_passes = int(w.get("sweep_passes",   2)),
            sweep_speed  = float(w.get("sweep_speed",  0.01)),
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Public API
    # ─────────────────────────────────────────────────────────────────────────

    def wipe(self) -> None:
        """Full cleaning sequence: extend → sweep → retract."""
        print("[WIPER] Starting wipe sequence")
        self.extend()
        self.sweep()
        self.retract()
        print("[WIPER] Wipe complete")

    def extend(self) -> None:
        """Press the wipe against the board surface. TODO: implement."""
        print("[WIPER] extend() — not yet implemented")

    def sweep(self) -> None:
        """Sweep the wipe across the board. TODO: implement."""
        print("[WIPER] sweep() — not yet implemented")

    def retract(self) -> None:
        """Return the arm to the home position. TODO: implement."""
        print("[WIPER] retract() — not yet implemented")

    def cleanup(self) -> None:
        """Release any hardware resources. TODO: implement."""
        self.retract()
        print("[WIPER] cleanup done")
