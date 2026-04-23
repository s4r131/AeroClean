"""wiper.py — simple GPIO on/off controller for AeroClean (gpiozero, Pi 5 safe).

Treats the wiper like a relay or motor enable:
turn ON → wait → turn OFF.

Duration is configurable via config.json.
"""

from __future__ import annotations

import json
import time

try:
    from gpiozero import LED
    GPIOZERO_AVAILABLE = True
except Exception:
    LED = None
    GPIOZERO_AVAILABLE = False


class Wiper:
    """
    Simple on/off wiper controller using gpiozero.

    Safe no-op on systems without GPIO support.
    """

    def __init__(self, pin: int | None, wipe_duration_s: float = 2.0):
        self._pin = pin
        self._device = None
        self._available = False
        self._wipe_duration = float(wipe_duration_s)

        if pin is None:
            print("[WIPER] GPIO pin not set — running as no-op. Set 'wiper_gpio_pin' in config.json.")
            return

        if not GPIOZERO_AVAILABLE:
            print(f"[WIPER] gpiozero not available — Wiper(pin={pin}) running as no-op.")
            return

        try:
            self._device = LED(pin)
            self._device.off()
            self._available = True
            print(f"[WIPER] Initialized on pin {pin} (gpiozero)")
        except Exception as e:
            self._device = None
            self._available = False
            print(f"[WIPER] GPIO init failed on pin {pin} — running as no-op: {e}")

    @classmethod
    def from_config(cls, config_path: str = "config.json") -> "Wiper":
        """Construct a Wiper from config.json."""
        with open(config_path, "r") as f:
            cfg = json.load(f)

        w = cfg.get("wiper", {})

        return cls(
            pin=w.get("wiper_gpio_pin"),
            wipe_duration_s=float(w.get("wipe_duration_s", 2.0)),
        )

    def on(self) -> None:
        """Turn the wiper ON."""
        print(f"[WIPER] ON (pin {self._pin})")
        if self._available and self._device is not None:
            self._device.on()

    def off(self) -> None:
        """Turn the wiper OFF."""
        if self._available and self._device is not None:
            self._device.off()
        print(f"[WIPER] OFF (pin {self._pin})")

    def wipe(self, duration_sec: float | None = None) -> None:
        """Run the wiper for configured duration (or override)."""
        duration = duration_sec if duration_sec is not None else self._wipe_duration
        print(f"[WIPER] Wiping for {duration:.1f}s on pin {self._pin}")
        self.on()
        time.sleep(duration)
        self.off()

    def cleanup(self) -> None:
        """Release GPIO resources."""
        self.off()
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
        print("[WIPER] cleanup done")