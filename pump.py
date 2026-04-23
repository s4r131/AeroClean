"""pump.py — GPIO pump controller for AeroClean (gpiozero version, Pi 5 safe)"""

from __future__ import annotations

import time

try:
    from gpiozero import LED
    GPIOZERO_AVAILABLE = True
except Exception:
    LED = None
    GPIOZERO_AVAILABLE = False


class Pump:
    """
    Pump controller using gpiozero.

    Safe no-op on systems without GPIO (dev laptops, etc).
    """

    def __init__(self, pin: int | None):
        self._pin = pin
        self._device = None
        self._available = False

        if pin is None:
            print("[PUMP] GPIO pin not set — running as no-op.")
            return

        if not GPIOZERO_AVAILABLE:
            print(f"[PUMP] gpiozero not available — Pump(pin={pin}) running as no-op.")
            return

        try:
            self._device = LED(pin)
            self._device.off()
            self._available = True
            print(f"[PUMP] Initialized on pin {pin} (gpiozero)")
        except Exception as e:
            print(f"[PUMP] GPIO init failed on pin {pin} — running as no-op: {e}")
            self._available = False

    def spray(self, duration_sec: float) -> None:
        """Activate pump for duration_sec seconds (blocking)."""
        print(f"[PUMP] Spraying for {duration_sec:.1f}s on pin {self._pin}")

        if self._available and self._device is not None:
            self._device.on()
            time.sleep(duration_sec)
            self.stop()
        else:
            time.sleep(duration_sec)

    def stop(self) -> None:
        """Stop pump immediately."""
        if self._available and self._device is not None:
            self._device.off()
        print(f"[PUMP] Stopped (pin {self._pin})")

    def cleanup(self) -> None:
        """Cleanup GPIO."""
        self.stop()
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
        print("[PUMP] cleanup done")