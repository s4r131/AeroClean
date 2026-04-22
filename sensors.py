"""
sensors.py — Sensor readers for the AeroClean companion computer.

Three classes, all with thread-safe background reads:

    SensorReader   — MicoAir MTF-02P optical flow + range sensor (MAVLink over UART).
    TFRangeSensor  — TF-Luna / TFMini forward range sensor over UART  (sensor A, default).
    RangeSensor    — VL53L3CX ToF forward range sensor over I2C  (sensor B).

RangeSensor and TFRangeSensor share the same public API (start / stop / get_distance)
so the mission approach controller works with either sensor without modification.

MTF-02P detail
--------------
The MTF-02P speaks MAVLink natively over UART at 115200 baud.
It emits two message types:
    DISTANCE_SENSOR  (msg ID 132) — range in centimetres
    OPTICAL_FLOW     (msg ID 100) — integrated flow velocities + quality

This module opens a pymavlink connection on a dedicated UART (separate from
the DroneKit/ArduPilot connection in mavlink_controller.py) and parses both
message types in a background daemon thread.

Usage:
    reader = SensorReader("/dev/ttyAMA1", baud=115200)
    reader.start()

    dist = reader.get_distance()      # metres, or None if not yet received
    flow = reader.get_optical_flow()  # dict or None

    reader.stop()
"""

from __future__ import annotations

import threading
import time

from pymavlink import mavutil

try:
    import board
    import busio
    import adafruit_vl53l4cd
    ADAFRUIT_VL53_AVAILABLE = True
except ImportError:
    ADAFRUIT_VL53_AVAILABLE = False

try:
    import serial as _serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class SensorReader:
    """
    Background reader for the MicoAir MTF-02P over UART.

    Thread-safe: get_distance() and get_optical_flow() may be called
    from any thread.
    """

    def __init__(self, uart_port: str, baud: int = 115200):
        self._uart_port = uart_port
        self._baud = baud

        self._conn = None
        self._latest_distance_m: float | None = None
        self._latest_flow: dict | None = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    # ─────────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ─────────────────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Open the UART connection and start the background reader thread."""
        self._conn = mavutil.mavlink_connection(
            self._uart_port,
            baud=self._baud,
            autoreconnect=True,
        )

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True, name="SensorReader")
        self._thread.start()
        print(f"[SENSORS] Reader started on {self._uart_port} @ {self._baud}")

    def stop(self) -> None:
        """Stop the background thread and close the UART connection."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._conn is not None:
            self._conn.close()
        print("[SENSORS] Reader stopped")

    # ─────────────────────────────────────────────────────────────────────────
    # Public accessors (thread-safe)
    # ─────────────────────────────────────────────────────────────────────────

    def get_distance(self) -> float | None:
        """
        Latest range reading from the MTF-02P in metres.
        Returns None until the first DISTANCE_SENSOR message is received.
        """
        with self._lock:
            return self._latest_distance_m

    def get_optical_flow(self) -> dict | None:
        """
        Latest optical flow reading.
        Returns None until the first OPTICAL_FLOW message is received.

        Dict keys:
            flow_x   (float) — integrated X flow velocity, m/s
            flow_y   (float) — integrated Y flow velocity, m/s
            quality  (int)   — sensor quality 0–255 (255 = best)
        """
        with self._lock:
            return self._latest_flow.copy() if self._latest_flow is not None else None

    # ─────────────────────────────────────────────────────────────────────────
    # Background thread
    # ─────────────────────────────────────────────────────────────────────────

    def _read_loop(self) -> None:
        """Parse incoming MAVLink messages and update cached values."""
        while self._running:
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.1)
                if msg is None:
                    continue

                msg_type = msg.get_type()

                if msg_type == "DISTANCE_SENSOR":
                    # current_distance is in centimetres per MAVLink spec
                    dist_m = msg.current_distance / 100.0
                    with self._lock:
                        self._latest_distance_m = dist_m
                    print(f"[MTF-02P] distance={dist_m:.3f}m")

                elif msg_type == "OPTICAL_FLOW":
                    flow = {
                        "flow_x": float(msg.flow_comp_m_x),
                        "flow_y": float(msg.flow_comp_m_y),
                        "quality": int(msg.quality),
                    }
                    with self._lock:
                        self._latest_flow = flow
                    print(f"[MTF-02P] flow_x={flow['flow_x']:.3f}  flow_y={flow['flow_y']:.3f}  quality={flow['quality']}")

            except Exception as e:
                # Log but never crash the thread — sensor dropout is recoverable
                print(f"[SENSORS] Read error: {e}")
                time.sleep(0.05)


class RangeSensor:
    """
    Forward-facing VL53L3CX ToF range sensor over I2C.

    The VL53L3CX is register-compatible with the VL53L4CD family.
    Uses the adafruit-circuitpython-vl53l4cd library on the Pi's default
    I2C bus (GPIO 3 = SDA, GPIO 5 = SCL).

    Wiring:
        VIN → Pi 3.3V (pin 1)
        GND → Pi GND  (pin 6)
        SDA → Pi GPIO 2 / pin 3
        SCL → Pi GPIO 3 / pin 5

    Enable I2C on the Pi before use:
        Add dtparam=i2c_arm=on to /boot/firmware/config.txt and reboot
    Verify with:
        sudo i2cdetect -y 1   (should show 0x29)

    Thread-safe: get_distance() may be called from any thread.
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
                    print(f"[VL53L3CX] distance={dist_m:.3f}m")
                else:
                    time.sleep(0.005)   # 5ms poll when no data ready
            except Exception as e:
                print(f"[RANGE] Read error: {e}")
                time.sleep(0.05)


class TFRangeSensor:
    """
    Forward-facing TF-Luna / TFMini range sensor over UART.

    Reads the Benewake 9-byte binary frame (0x59 0x59 header) in a
    background daemon thread. Same public API as RangeSensor so either
    sensor can be used interchangeably in tests and the mission.

    Wiring:
        VCC → Pi 5V  (pin 2 or 4)
        GND → Pi GND (pin 6)
        TX  → Pi RX  (e.g. GPIO15 / pin 10 for UART0)
        RX  → Pi TX  (e.g. GPIO14 / pin 8  for UART0)

    Enable the UART in /boot/firmware/config.txt (e.g. dtoverlay=uart3)
    and verify with pinctrl -p before use.

    Thread-safe: get_distance() may be called from any thread.
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
            self._ser.close()
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
                    print(f"[TF RANGE] distance={dist_m:.3f}m")
            except Exception as e:
                print(f"[TF RANGE] Read error: {e}")
                time.sleep(0.05)
