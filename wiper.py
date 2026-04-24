"""
wiper.py — GPIO on/off relay controller for the AeroClean wiper (gpiozero, Pi 5 safe).

Treats the wiper mechanism as a simple relay: turn ON, wait, turn OFF.
Duration is passed in at construction time (read from config by the caller).
Gracefully degrades on non-Pi systems — all methods are safe no-ops
when gpiozero is unavailable or the pin is not set.

Usage:
    wiper = Wiper(pin=17, wipe_duration_s=5.0)
    wiper.wipe()       # blocking — turns on, waits, turns off
    wiper.cleanup()    # call once on shutdown
"""

from __future__ import annotations

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
            print("[WIPER] GPIO pin not set — running as no-op. Set wiper.wiper_gpio_pin in config.json.")
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
            print(f"[WIPER] GPIO init failed on pin {pin} — running as no-op: {e}")

    # ─────────────────────────────────────────────────────────────────────────
    # Control
    # ─────────────────────────────────────────────────────────────────────────

    def on(self) -> None:
        """Turn the wiper relay ON."""
        print(f"[WIPER] ON (pin {self._pin})")
        if self._available and self._device is not None:
            self._device.on()

    def off(self) -> None:
        """Turn the wiper relay OFF."""
        if self._available and self._device is not None:
            self._device.off()
        print(f"[WIPER] OFF (pin {self._pin})")

    def wipe(self, duration_sec: float | None = None) -> None:
        """Run the wiper for the configured duration (or an override). Blocking."""
        duration = duration_sec if duration_sec is not None else self._wipe_duration
        print(f"[WIPER] Wiping for {duration:.1f}s on pin {self._pin}")
        self.on()
        time.sleep(duration)
        self.off()

    # ─────────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ─────────────────────────────────────────────────────────────────────────

    def cleanup(self) -> None:
        """Turn off and release GPIO resources."""
        self.off()
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
        print("[WIPER] cleanup done")
