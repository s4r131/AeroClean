"""
mavlink_controller.py — DroneKit wrapper for ArduPilot over UART.

All ArduPilot interaction lives here. mission.py never imports DroneKit
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

import time

from dronekit import connect, VehicleMode
from pymavlink import mavutil


# Seconds to wait after reaching target altitude before transitioning
_TAKEOFF_SETTLE_S = 5.0

# Seconds to wait for the vehicle to become armable
_ARM_TIMEOUT_S = 15.0

# Fraction of target altitude that counts as "reached"
_ALTITUDE_FRACTION = 0.95


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
        self._baud = baud
        self._vehicle = None

    # ─────────────────────────────────────────────────────────────────────────
    # Connection
    # ─────────────────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Connect to ArduPilot and wait until the vehicle is ready."""
        print(f"[FC] Connecting to ArduPilot on {self._uart_port} @ {self._baud}...")
        self._vehicle = connect(
            self._uart_port,
            baud=self._baud,
            wait_ready=True,
        )
        print(
            f"[FC] Connected — firmware: {self._vehicle.version}, "
            f"GPS fix: {self._vehicle.gps_0.fix_type}"
        )

    def close(self) -> None:
        """Close the DroneKit connection."""
        if self._vehicle is not None:
            self._vehicle.close()
            print("[FC] Connection closed")

    # ─────────────────────────────────────────────────────────────────────────
    # Arming and takeoff
    # ─────────────────────────────────────────────────────────────────────────

    def arm_and_takeoff(self, altitude_m: float) -> None:
        """
        Switch to GUIDED, arm, take off, and block until altitude_m is reached.

        Raises RuntimeError if the vehicle does not become armable within
        _ARM_TIMEOUT_S seconds.
        """
        v = self._vehicle

        print("[FC] Setting GUIDED mode")
        v.mode = VehicleMode("GUIDED")
        while v.mode.name != "GUIDED":
            time.sleep(0.1)

        print("[FC] Waiting for vehicle to be armable...")
        deadline = time.monotonic() + _ARM_TIMEOUT_S
        while not v.is_armable:
            if time.monotonic() > deadline:
                raise RuntimeError(
                    "Vehicle did not become armable within "
                    f"{_ARM_TIMEOUT_S}s — check GPS and pre-arm checks."
                )
            time.sleep(0.5)

        print("[FC] Arming...")
        v.armed = True
        while not v.armed:
            time.sleep(0.1)
        print("[FC] Armed")

        print(f"[FC] Taking off to {altitude_m:.1f}m")
        v.simple_takeoff(altitude_m)

        while True:
            current_alt = v.location.global_relative_frame.alt
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
        # Type mask: ignore position (bit 0-2) and acceleration (bit 6-8);
        # use velocity (bits 3-5) and yaw rate (bit 10).
        # 0b111000000111 = 0b0000_1000_0111 -> decimal 0x0C07 = 3079
        # Simpler: set all bits then clear velocity and yaw_rate bits.
        # DroneKit message_factory encodes this as a raw MAVLink message.
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )

        msg = self._vehicle.message_factory.set_position_target_local_ned_encode(
            0,           # time_boot_ms (ignored)
            0, 0,        # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame=9 (body-relative)
            type_mask,
            0, 0, 0,     # x, y, z position (ignored)
            vx, vy, vz,  # vx, vy, vz velocity (m/s)
            0, 0, 0,     # ax, ay, az acceleration (ignored)
            0,           # yaw (ignored)
            yaw_rate,    # yaw_rate (rad/s)
        )
        self._vehicle.send_mavlink(msg)
        self._vehicle.flush()

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
        self._vehicle.mode = VehicleMode("RTL")

    # ─────────────────────────────────────────────────────────────────────────
    # Telemetry
    # ─────────────────────────────────────────────────────────────────────────

    def get_altitude_m(self) -> float:
        """Current altitude above home in metres (AGL)."""
        return self._vehicle.location.global_relative_frame.alt

    def get_heading_deg(self) -> float:
        """Current magnetic heading in degrees (0–359)."""
        return self._vehicle.heading

    def is_landed(self) -> bool:
        """
        True when the vehicle has landed and disarmed after RTL.
        Used by the RETURN state to detect mission completion.
        """
        alt = self._vehicle.location.global_relative_frame.alt
        return alt < 0.1 and not self._vehicle.armed
