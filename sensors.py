"""
sensors.py — Sensor readers for the AeroClean companion computer.

Two classes, both with thread-safe background reads:

    TFRangeSensor  — TF-Luna / TFMini forward range sensor over UART  (sensor A, default).
    RangeSensor    — VL53L3CX ToF forward range sensor over I2C  (sensor B).

RangeSensor and TFRangeSensor share the same public API (start / stop / get_distance)
so the mission approach controller works with either sensor without modification.
"""

from __future__ import annotations

import threading
import time

"""
Three libraries required for the VL53L3CX (sensor B, I2C):
  board             — knows the Pi's physical pin layout (board.SCL / board.SDA gives the correct GPIO pins)
  busio             — opens the I2C bus on those pins so the sensor can communicate
  adafruit_vl53l4cd — the sensor driver; hides all low-level register reads, exposes a simple .distance property
All three are Pi-only. The try statement below sets the flag to False if any are missing so this
file can be imported on a dev machine without crashing — the sensor just runs as a no-op.
"""
try:
    import board
    import busio
    import adafruit_vl53l4cd
    ADAFRUIT_VL53_AVAILABLE = True
except ImportError:
    ADAFRUIT_VL53_AVAILABLE = False

"""
One library required for the TF-Luna / TFMini (sensor A, UART):
  serial (pyserial) — opens and reads from the UART serial port on the Pi.
                      Aliased to _serial to avoid clashing with Python's own serial namespace.
Pi-only in practice. The try statement below sets the flag to False if it is missing so this
file can be imported on a dev machine without crashing — the sensor just runs as a no-op.
"""
try:
    import serial as _serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class RangeSensor:
    """
    VL53L3CX ToF range sensor over I2C (sensor B).
    Reads distance in a background thread and caches the latest value in metres.
    Call get_distance() from any thread — returns the latest reading or None if no reading yet.
    """

    def __init__(self, i2c_address: int = 0x29, timing_budget_ms: int = 50):
        self._i2c_address    = i2c_address
        self._timing_budget  = timing_budget_ms
        self._sensor         = None
        self._available      = False

        self._latest_distance_m: float | None = None
        self._lock    = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

        if not ADAFRUIT_VL53_AVAILABLE:
            print(
                "[RANGE] adafruit-circuitpython-vl53l4cd not installed — "
                "RangeSensor running as no-op. "
                "Install with: pip install adafruit-circuitpython-vl53l4cd adafruit-blinka"
            )

    # ─────────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ─────────────────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Initialise the sensor and start the background polling thread."""
        if not ADAFRUIT_VL53_AVAILABLE:
            return

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            """
            self._sensor is an adafruit_vl53l4cd.VL53L4CD object — the Adafruit driver for the VL53L3CX.
            Its built-in properties and methods used here: .distance (mm), .data_ready,
            .clear_interrupt(), .timing_budget, and .start_ranging().
            """
            self._sensor = adafruit_vl53l4cd.VL53L4CD(i2c, address=self._i2c_address)
            self._sensor.timing_budget = self._timing_budget
            self._sensor.start_ranging()
            self._available = True
        except Exception as e:
            print(f"[RANGE] Failed to initialise sensor at 0x{self._i2c_address:02X}: {e}")
            return

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True, name="RangeSensor")
        self._thread.start()
        print(f"[RANGE] VL53L3CX started at I2C address 0x{self._i2c_address:02X}")

    def stop(self) -> None:
        """Stop the background thread and shut down the sensor."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._sensor is not None:
            try:
                self._sensor.stop_ranging()
            except Exception:
                pass
        print("[RANGE] Sensor stopped")

    # ─────────────────────────────────────────────────────────────────────────
    # Public accessor (thread-safe)
    # ─────────────────────────────────────────────────────────────────────────

    def get_distance(self) -> float | None:
        """
        Latest distance reading in metres.
        Returns None until the first reading is available or if hardware
        is not present.
        """
        with self._lock:
            return self._latest_distance_m

    # ─────────────────────────────────────────────────────────────────────────
    # Background thread
    # ─────────────────────────────────────────────────────────────────────────

    def _read_loop(self) -> None:
        """Poll the sensor and update the cached distance."""
        while self._running:
            try:
                if self._sensor.data_ready:
                    # distance is in mm per adafruit library
                    dist_m = self._sensor.distance / 1000.0
                    self._sensor.clear_interrupt()
                    with self._lock:
                        self._latest_distance_m = dist_m
                else:
                    time.sleep(0.005)   # 5ms poll when no data ready
            except Exception as e:
                print(f"[RANGE] Read error: {e}")
                time.sleep(0.05)


class TFRangeSensor:
    """
    TF-Luna / TFMini range sensor over UART (sensor A).
    Reads distance in a background thread and caches the latest value in metres.
    Call get_distance() from any thread — returns the latest reading or None if no reading yet.
    """

    def __init__(self, uart_port: str, baud: int = 115200):
        self._uart_port = uart_port
        self._baud      = baud
        self._ser       = None

        self._latest_distance_m: float | None = None
        self._lock    = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

        if not SERIAL_AVAILABLE:
            print(
                "[TF RANGE] pyserial not installed — TFRangeSensor running as no-op. "
                "Install with: pip install pyserial"
            )

    # ─────────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ─────────────────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Open the UART and start the background reader thread."""
        if not SERIAL_AVAILABLE:
            return

        try:
            """
            self._ser is a _serial.Serial object — the pyserial handle for the UART port.
            Its built-in properties and methods used here: .read(n) (reads n bytes from the port),
            and .close() (releases the port when the sensor stops).
            """
            self._ser = _serial.Serial(self._uart_port, self._baud, timeout=1)
        except _serial.SerialException as e:
            print(f"[TF RANGE] Could not open {self._uart_port}: {e}")
            return

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True, name="TFRangeSensor")
        self._thread.start()
        print(f"[TF RANGE] TF sensor started on {self._uart_port} @ {self._baud}")

    def stop(self) -> None:
        """Stop the background thread and close the UART."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        print("[TF RANGE] Sensor stopped")

    # ─────────────────────────────────────────────────────────────────────────
    # Public accessor (thread-safe)
    # ─────────────────────────────────────────────────────────────────────────

    def get_distance(self) -> float | None:
        """
        Latest distance reading in metres.
        Returns None until the first valid frame is received.
        """
        with self._lock:
            return self._latest_distance_m

    # ─────────────────────────────────────────────────────────────────────────
    # Background thread
    # ─────────────────────────────────────────────────────────────────────────

    def _parse_frame(self) -> float | None:
        """
        Read and validate one 9-byte TF frame.
        Returns distance in metres, or None on invalid/out-of-range reading.
        """
        while self._running:
            b1 = self._ser.read(1)
            if not b1:
                return None
            if b1[0] != 0x59:
                continue

            b2 = self._ser.read(1)
            if not b2 or b2[0] != 0x59:
                continue

            rest = self._ser.read(7)
            if len(rest) != 7:
                return None

            frame = bytes([0x59, 0x59]) + rest
            if (sum(frame[:8]) & 0xFF) != frame[8]:
                continue

            dist_cm = frame[2] | (frame[3] << 8)
            if dist_cm == 0xFFFF:
                return None

            return dist_cm / 100.0
        return None

    def _read_loop(self) -> None:
        """Parse incoming TF frames and update the cached distance."""
        while self._running:
            try:
                dist_m = self._parse_frame()
                if dist_m is not None:
                    with self._lock:
                        self._latest_distance_m = dist_m
            except Exception as e:
                print(f"[TF RANGE] Read error: {e}")
                time.sleep(0.05)
