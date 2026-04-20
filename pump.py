"""
pump.py — GPIO pump controller for AeroClean cleaning mechanism.

Controls a pump attached to a BCM GPIO pin on the Raspberry Pi.
Gracefully degrades on non-Pi systems (import guard matches camera.py pattern).

Usage:
    pump = Pump(pin=17)
    pump.spray(duration_sec=5.0)   # blocking
    pump.cleanup()                 # call once on shutdown
"""

from __future__ import annotations

import time

try:
    import RPi.GPIO as GPIO
    RPI_GPIO_AVAILABLE = True
except ImportError:
    RPI_GPIO_AVAILABLE = False


class Pump:
    """
    Thin wrapper around a single GPIO output pin driving a pump relay.

    All methods are safe no-ops on non-Pi hardware so the mission state
    machine can run in simulation without special casing.
    """

    def __init__(self, pin: int | None):
        self._pin = pin
        self._available = RPI_GPIO_AVAILABLE and pin is not None

        if pin is None:
            print("[PUMP] GPIO pin not yet assigned — running as no-op. Set 'pump_gpio_pin' in config.json.")
        elif not RPI_GPIO_AVAILABLE:
            print(
                f"[PUMP] RPi.GPIO not available — Pump(pin={pin}) running as no-op. "
                "Install RPi.GPIO on a Raspberry Pi for real operation."
            )
        else:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._pin, GPIO.OUT, initial=GPIO.LOW)

    def spray(self, duration_sec: float) -> None:
        """
        Activate the pump for duration_sec seconds, then stop.

        Blocking — the mission state machine does nothing else during cleaning.
        """
        print(f"[PUMP] Spraying for {duration_sec:.1f}s on pin {self._pin}")
        if self._available:
            GPIO.output(self._pin, GPIO.HIGH)
        time.sleep(duration_sec)
        self.stop()

    def stop(self) -> None:
        """Deactivate the pump immediately. Safe to call at any time."""
        if self._available:
            GPIO.output(self._pin, GPIO.LOW)
        print(f"[PUMP] Stopped (pin {self._pin})")

    def cleanup(self) -> None:
        """
        Release GPIO resources. Call once when the mission terminates.
        Calls stop() first to guarantee the pin is LOW before releasing,
        even if the pump was still running when shutdown was triggered.
        """
        self.stop()   # always ensure pin is LOW before releasing GPIO
        if self._available:
            GPIO.cleanup(self._pin)
        print("[PUMP] GPIO cleanup done")
