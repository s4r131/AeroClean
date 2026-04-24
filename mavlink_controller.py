"""
mavlink_controller.py — pymavlink interface to ArduPilot over UART.

All ArduPilot interaction lives here. mission.py never touches pymavlink
directly — it only calls this module's clean interface.

Connection:
    ArduPilot FC ↔ Raspberry Pi via UART (/dev/ttyAMA0 by default).
    ArduPilot must be configured with SERIAL port baud matching mavlink_baud
    in config.json (default 57600).

Flight mode:
    The drone is placed in GUIDED mode before arming. In GUIDED mode ArduPilot
    accepts velocity setpoints from the companion computer while handling all
    low-level attitude stabilisation with its own IMU.

Velocity commands:
    send_velocity_body() uses SET_POSITION_TARGET_LOCAL_NED with
    MAV_FRAME_BODY_OFFSET_NED (frame=9) so vx/vy/vz are body-relative:
        vx > 0  — forward
        vy > 0  — right
        vz > 0  — down (NED convention)
        yaw_rate (rad/s) > 0 — clockwise yaw
"""

from __future__ import annotations

import threading
import time

from pymavlink import mavutil


# ArduCopter custom mode numbers
_GUIDED_MODE = 4
_RTL_MODE    = 6

_TAKEOFF_SETTLE_S   = 5.0    # seconds to hover after reaching altitude
_ARM_TIMEOUT_S      = 15.0   # seconds to wait for arm confirmation
_MODE_TIMEOUT_S     = 5.0    # seconds to wait for mode confirmation
_ALTITUDE_FRACTION  = 0.95   # fraction of target alt that counts as "reached"
_POSITION_STREAM_HZ = 4      # GLOBAL_POSITION_INT rate requested from FC


class MAVLinkController:
    """
    Companion-computer interface to an ArduPilot flight controller.

    Usage:
        ctrl = MAVLinkController("/dev/ttyAMA0", baud=57600)
        ctrl.connect()
        ctrl.arm_and_takeoff(1.5)
        ctrl.send_velocity_body(0.3, 0.0, 0.0, 0.0)   # fly forward
        ctrl.rtl()
        ctrl.close()
    """

    def __init__(self, uart_port: str, baud: int):
        self._uart_port = uart_port
        self._baud      = baud
        self._conn      = None

        # Telemetry cache — written by background thread, read by public API
        self._alt_m:       float | None = None
        self._heading_deg: float | None = None
        self._armed:       bool         = False
        self._custom_mode: int          = -1
        self._lock = threading.Lock()

        self._running = False
        self._thread: threading.Thread | None = None

    # ─────────────────────────────────────────────────────────────────────────
    # Connection
    # ─────────────────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Connect to ArduPilot, wait for heartbeat, and start telemetry thread."""
        print(f"[FC] Connecting to ArduPilot on {self._uart_port} @ {self._baud}...")
        self._conn = mavutil.mavlink_connection(
            self._uart_port,
            baud=self._baud,
            autoreconnect=True,
        )
        self._conn.wait_heartbeat()
        print(
            f"[FC] Heartbeat received — system {self._conn.target_system}, "
            f"component {self._conn.target_component}"
        )

        # Ask the FC to stream position messages
        self._conn.mav.request_data_stream_send(
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            _POSITION_STREAM_HZ,
            1,   # 1 = start
        )

        self._running = True
        self._thread = threading.Thread(
            target=self._read_loop, daemon=True, name="MAVTelemetry"
        )
        self._thread.start()
        print("[FC] Telemetry thread started")

    def close(self) -> None:
        """Stop the telemetry thread and close the MAVLink connection."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._conn is not None:
            self._conn.close()
        print("[FC] Connection closed")

    # ─────────────────────────────────────────────────────────────────────────
    # Telemetry background thread
    # ─────────────────────────────────────────────────────────────────────────

    def _read_loop(self) -> None:
        """Parse incoming MAVLink messages and update the telemetry cache."""
        while self._running:
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.1)
                if msg is None:
                    continue

                t = msg.get_type()

                if t == "HEARTBEAT":
                    armed = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    with self._lock:
                        self._armed       = armed
                        self._custom_mode = msg.custom_mode

                elif t == "GLOBAL_POSITION_INT":
                    alt_m     = msg.relative_alt / 1000.0   # mm → m
                    hdg_deg   = msg.hdg / 100.0             # cdeg → deg
                    with self._lock:
                        self._alt_m       = alt_m
                        self._heading_deg = hdg_deg

                elif t == "STATUSTEXT":
                    text = msg.text.strip()
                    if text:
                        print(f"[FC] STATUS: {text}")

            except Exception as e:
                print(f"[FC] Telemetry read error: {e}")
                time.sleep(0.05)

    # ─────────────────────────────────────────────────────────────────────────
    # Internal helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _set_mode(self, mode_id: int) -> None:
        """Send a mode-change command (non-blocking)."""
        self._conn.mav.set_mode_send(
            self._conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

    def _wait_for_mode(self, mode_id: int, timeout: float = _MODE_TIMEOUT_S) -> None:
        """Block until the cached custom_mode matches mode_id, or raise on timeout."""
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                if self._custom_mode == mode_id:
                    return
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"Flight controller did not confirm mode {mode_id} "
                    f"within {timeout:.0f}s."
                )
            time.sleep(0.1)

    # ─────────────────────────────────────────────────────────────────────────
    # Arming and takeoff
    # ─────────────────────────────────────────────────────────────────────────

    def arm_and_takeoff(self, altitude_m: float) -> None:
        """
        Switch to GUIDED, arm, take off, and block until altitude_m is reached.

        Raises RuntimeError if the vehicle does not arm within _ARM_TIMEOUT_S.
        ArduPilot enforces all pre-arm checks and will reject the arm command
        until they pass — this method retries until accepted or timeout.
        """
        print("[FC] Setting GUIDED mode")
        self._set_mode(_GUIDED_MODE)
        self._wait_for_mode(_GUIDED_MODE)
        print("[FC] GUIDED mode confirmed")

        print("[FC] Arming...")
        deadline = time.monotonic() + _ARM_TIMEOUT_S
        while True:
            with self._lock:
                if self._armed:
                    break
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"Vehicle did not arm within {_ARM_TIMEOUT_S:.0f}s — "
                    "check pre-arm conditions on the FC."
                )
            self._conn.mav.command_long_send(
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,       # confirmation
                1,       # param1: 1 = arm
                0, 0, 0, 0, 0, 0,
            )
            time.sleep(0.5)
        print("[FC] Armed")

        print(f"[FC] Taking off to {altitude_m:.1f}m")
        self._conn.mav.command_long_send(
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,           # confirmation
            0, 0, 0, 0,  # params 1–4 (unused for copter)
            0, 0,        # lat, lon (unused — take off from current position)
            altitude_m,  # param7: target altitude AGL in metres
        )

        while True:
            with self._lock:
                current_alt = self._alt_m
            if current_alt is not None:
                print(f"[FC] Altitude: {current_alt:.2f}m", end="\r")
                if current_alt >= altitude_m * _ALTITUDE_FRACTION:
                    print(f"\n[FC] Target altitude reached ({current_alt:.2f}m)")
                    break
            time.sleep(0.2)

        time.sleep(_TAKEOFF_SETTLE_S)
        print("[FC] Takeoff complete, hovering")

    # ─────────────────────────────────────────────────────────────────────────
    # Velocity control
    # ─────────────────────────────────────────────────────────────────────────

    def send_velocity_body(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float,
    ) -> None:
        """
        Send a body-frame velocity + yaw-rate setpoint to ArduPilot.

        Parameters
        ----------
        vx : float
            Forward velocity in m/s (positive = forward).
        vy : float
            Lateral velocity in m/s (positive = right).
        vz : float
            Vertical velocity in m/s (positive = down, NED convention).
        yaw_rate : float
            Yaw rate in rad/s (positive = clockwise from above).
        """
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE  |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE  |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE  |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )

        self._conn.mav.set_position_target_local_ned_send(
            0,                                               # time_boot_ms
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,      # body-relative frame
            type_mask,
            0, 0, 0,       # x, y, z position (ignored)
            vx, vy, vz,    # body-frame velocity (m/s)
            0, 0, 0,       # acceleration (ignored)
            0,             # yaw (ignored)
            yaw_rate,      # yaw_rate (rad/s)
        )

    def stop_movement(self) -> None:
        """
        Command the vehicle to hold position (zero all velocities).
        Sent twice to guard against single-packet loss.
        """
        self.send_velocity_body(0.0, 0.0, 0.0, 0.0)
        time.sleep(0.05)
        self.send_velocity_body(0.0, 0.0, 0.0, 0.0)

    # ─────────────────────────────────────────────────────────────────────────
    # Navigation
    # ─────────────────────────────────────────────────────────────────────────

    def rtl(self) -> None:
        """
        Switch to Return-To-Launch mode (non-blocking).
        ArduPilot will fly home and land automatically.
        Poll is_landed() to detect mission completion.
        """
        print("[FC] Switching to RTL")
        self._set_mode(_RTL_MODE)

    # ─────────────────────────────────────────────────────────────────────────
    # Telemetry accessors
    # ─────────────────────────────────────────────────────────────────────────

    def get_altitude_m(self) -> float | None:
        """Current altitude above home in metres (AGL). None until first fix."""
        with self._lock:
            return self._alt_m

    def get_heading_deg(self) -> float | None:
        """Current heading in degrees (0–359). None until first fix."""
        with self._lock:
            return self._heading_deg

    def is_landed(self) -> bool:
        """
        True when the vehicle has landed and disarmed after RTL.
        Used by the RETURN state to detect mission completion.
        """
        with self._lock:
            return (
                self._alt_m is not None
                and self._alt_m < 0.1
                and not self._armed
            )
