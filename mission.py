"""
mission.py — AeroClean autonomous drone mission state machine.

Mission flow:
    IDLE → TAKEOFF → SCAN → APPROACH → CLEAN → RETURN → DONE

        SCAN timeout ──────────────────────────────┐
        APPROACH board lost ──────┐                │
                                  ▼                ▼
                                SCAN           RETURN

    Any unhandled exception → ABORTED (shutdown + RTL attempted)

Subsystems wired together here:
    Camera (camera.py)            — frame capture
    YOLOModel (yolo_model.py)     — dirty_board detection
    MAVLinkController             — ArduPilot GUIDED mode commands
    SensorReader                  — MTF-02P range + optical flow
    Pump                          — GPIO spray mechanism
    Wiper                         — servo-driven stick + wipe arm

Run via:
    python main.py --mode mission
"""

from __future__ import annotations

import json
import math
import time
from enum import Enum, auto

import cv2

from camera import Camera
from mavlink_controller import MAVLinkController
from pump import Pump
from sensors import RangeSensor, SensorReader
from wiper import Wiper
from yolo_model import YOLOModel


# Consecutive frames without a detection before APPROACH → SCAN fallback
_APPROACH_LOST_LIMIT = 10


class MissionState(Enum):
    IDLE     = auto()
    TAKEOFF  = auto()   # reserved; transition handled inside _tick_idle
    SCAN     = auto()
    APPROACH = auto()
    CLEAN    = auto()
    RETURN   = auto()
    ABORTED  = auto()   # terminal — error
    DONE     = auto()   # terminal — success


_TERMINAL = {MissionState.DONE, MissionState.ABORTED}


class Mission:
    """
    Autonomous board-cleaning mission state machine.

    All configuration is read from the 'mission' key in config.json.
    Existing top-level keys (camera, YOLO, display) are also respected.
    """

    def __init__(self, config_path: str = "config.json"):
        with open(config_path) as f:
            cfg = json.load(f)

        if "mission" not in cfg:
            raise RuntimeError("config.json is missing the 'mission' block. Check your config file.")
        m = cfg["mission"]

        # Fail early with clear messages for unassigned hardware
        _required = {
            "mavlink_uart":  "UART port to ArduPilot FC (e.g. /dev/ttyAMA0)",
            "sensor_uart":   "UART port to MTF-02P sensor (e.g. /dev/ttyAMA2)",
        }
        missing = [f"  mission.{k}  —  {desc}" for k, desc in _required.items() if m.get(k) is None]
        if missing:
            raise RuntimeError(
                "The following config.json values must be set before running mission mode:\n"
                + "\n".join(missing)
            )

        self._cfg_takeoff_alt       = float(m["takeoff_altitude_m"])
        self._cfg_scan_yaw_rate     = math.radians(float(m["scan_yaw_rate_dps"]))
        self._cfg_scan_timeout      = float(m["scan_timeout_s"])
        self._cfg_stop_dist         = float(m["approach_stop_dist_m"])
        self._cfg_kp                = float(m["approach_kp"])
        self._cfg_kp_forward        = float(m["approach_kp_forward"])
        self._cfg_align_threshold   = int(m["align_threshold_px"])
        self._cfg_max_speed         = float(m["approach_max_speed_ms"])
        self._cfg_cautious_speed    = float(m["approach_cautious_speed_ms"])
        self._cfg_pump_duration     = float(m["pump_duration_s"])
        self._cfg_frame_w           = int(m["frame_width_px"])
        self._cfg_frame_h           = int(m["frame_height_px"])
        self._cfg_display           = bool(cfg.get("display", False))

        # Sanity check: frame dimensions must match camera resolution or the
        # approach controller will compute wrong pixel errors.
        cam_w, cam_h = cfg.get("resolution", [1920, 1080])
        if self._cfg_frame_w != cam_w or self._cfg_frame_h != cam_h:
            raise RuntimeError(
                f"config.json mismatch: mission.frame_width_px={self._cfg_frame_w} / "
                f"frame_height_px={self._cfg_frame_h} does not match "
                f"resolution=[{cam_w}, {cam_h}]. Update one to match the other."
            )

        self._camera     = Camera(config_path)
        self._yolo       = YOLOModel(config_path)
        self._controller = MAVLinkController(m["mavlink_uart"], int(m["mavlink_baud"]))
        self._sensors    = SensorReader(m["sensor_uart"], int(m["sensor_baud"]))
        range_cfg = cfg.get("range_sensor", {})
        self._range      = RangeSensor(range_cfg.get("i2c_address", 0x29))
        pump_pin = m.get("pump_gpio_pin")
        self._pump       = Pump(int(pump_pin) if pump_pin is not None else None)
        self._wiper      = Wiper.from_config(config_path)

        self._state: MissionState = MissionState.IDLE

        # SCAN state
        self._scan_start_time: float | None = None

        # APPROACH state
        self._target_bbox: list | None  = None
        self._approach_lost_count: int  = 0
        self._approach_phase: str       = "ALIGN"   # "ALIGN" or "APPROACH"

        # RETURN state
        self._return_initiated: bool = False

    # ─────────────────────────────────────────────────────────────────────────
    # Entry point
    # ─────────────────────────────────────────────────────────────────────────

    def run(self) -> None:
        """Start the mission. Blocks until DONE or ABORTED."""
        print("[MISSION] Starting up")
        try:
            self._camera.start()
            self._sensors.start()
            self._range.start()
            self._controller.connect()

            if self._cfg_display:
                cv2.namedWindow("AeroClean Mission", cv2.WINDOW_NORMAL)

            while self._state not in _TERMINAL:
                frame = self._camera.capture()
                self._tick(frame)

                if self._cfg_display:
                    cv2.imshow("AeroClean Mission", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        print("[MISSION] Operator quit")
                        self._abort("Operator pressed q")

        except Exception as e:
            self._abort(f"Unhandled exception: {e}")
        finally:
            self._shutdown()
            if self._cfg_display:
                cv2.destroyAllWindows()

        print(f"[MISSION] Finished — final state: {self._state.name}")

    # ─────────────────────────────────────────────────────────────────────────
    # Tick dispatcher
    # ─────────────────────────────────────────────────────────────────────────

    def _tick(self, frame) -> None:
        dispatch = {
            MissionState.IDLE:     self._tick_idle,
            MissionState.SCAN:     self._tick_scan,
            MissionState.APPROACH: self._tick_approach,
            MissionState.CLEAN:    self._tick_clean,
            MissionState.RETURN:   self._tick_return,
        }
        handler = dispatch.get(self._state)
        if handler:
            try:
                handler(frame)
            except Exception as e:
                self._abort(f"Exception in state {self._state.name}: {e}")

    # ─────────────────────────────────────────────────────────────────────────
    # State handlers
    # ─────────────────────────────────────────────────────────────────────────

    def _tick_idle(self, frame) -> None:
        """Arm and take off. Blocking — transitions to SCAN on success."""
        print("[MISSION] IDLE → arming and taking off")
        self._controller.arm_and_takeoff(self._cfg_takeoff_alt)
        self._state = MissionState.SCAN
        self._scan_start_time = None
        print("[MISSION] → SCAN")

    def _tick_scan(self, frame) -> None:
        """Spin and search for a dirty board with YOLO."""
        if self._scan_start_time is None:
            self._scan_start_time = time.monotonic()
            print("[MISSION] SCAN started — yawing to search for dirty board")

        # Keep yawing
        self._controller.send_velocity_body(0.0, 0.0, 0.0, self._cfg_scan_yaw_rate)

        # Run detection
        detection, annotated = self._yolo.run(frame)

        if self._cfg_display:
            cv2.imshow("AeroClean Mission", annotated)

        if detection is not None and detection["class_name"] == "dirty_board":
            print(
                f"[MISSION] Dirty board detected "
                f"(conf={detection['confidence']:.2f}) → APPROACH"
            )
            self._controller.stop_movement()
            self._target_bbox = detection["bbox"]
            self._approach_lost_count = 0
            self._state = MissionState.APPROACH
            return

        # Timeout guard
        elapsed = time.monotonic() - self._scan_start_time
        if elapsed > self._cfg_scan_timeout:
            print(f"[MISSION] Scan timeout ({self._cfg_scan_timeout:.0f}s) — no board found → RETURN")
            self._controller.stop_movement()
            self._state = MissionState.RETURN

    def _tick_approach(self, frame) -> None:
        """
        Two-phase approach controller.

        Phase 1 — ALIGN (vx=0):
            Hold position and correct lateral/vertical until the board centre
            is within align_threshold_px of the frame centre in both axes.
            The range sensor points directly at the board only once aligned,
            so we do not begin moving forward until this is satisfied.

        Phase 2 — APPROACH (proportional vx):
            Drive forward at vx = kp_forward * (dist - stop_dist), which
            naturally decelerates to zero as the board is reached.
            Continue lateral/vertical corrections throughout.
            Stop when range sensor reads <= approach_stop_dist_m.

        Falls back to SCAN if the board is lost for _APPROACH_LOST_LIMIT frames.
        """
        # ── Update YOLO detection ───────────────────────────────────────────
        detection, annotated = self._yolo.run(frame)

        if self._cfg_display:
            cv2.imshow("AeroClean Mission", annotated)

        if detection is not None and detection["class_name"] == "dirty_board":
            self._target_bbox = detection["bbox"]
            self._approach_lost_count = 0
        else:
            self._approach_lost_count += 1
            if self._approach_lost_count >= _APPROACH_LOST_LIMIT:
                print("[MISSION] Board lost during approach → back to SCAN")
                self._controller.stop_movement()
                self._scan_start_time = None
                self._approach_phase  = "ALIGN"
                self._state = MissionState.SCAN
                return

        if self._target_bbox is None:
            return

        # ── Compute lateral/vertical error from bbox centre ─────────────────
        x1, y1, x2, y2 = self._target_bbox
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        # Pixel error from frame centre
        px_err_x = cx - self._cfg_frame_w / 2
        px_err_y = cy - self._cfg_frame_h / 2

        # Normalised error [-0.5, +0.5] for the P controller
        err_x = px_err_x / self._cfg_frame_w
        err_y = px_err_y / self._cfg_frame_h

        vy = max(-self._cfg_max_speed, min(self._cfg_max_speed, self._cfg_kp * err_x))
        vz = max(-self._cfg_max_speed, min(self._cfg_max_speed, self._cfg_kp * err_y))

        # ── Phase 1 — ALIGN ─────────────────────────────────────────────────
        if self._approach_phase == "ALIGN":
            aligned = (
                abs(px_err_x) < self._cfg_align_threshold and
                abs(px_err_y) < self._cfg_align_threshold
            )
            if aligned:
                print(
                    f"[APPROACH] Aligned (err={abs(px_err_x):.0f}px, {abs(px_err_y):.0f}px) "
                    "— switching to forward approach"
                )
                self._approach_phase = "APPROACH"
            else:
                # Hold position, correct lateral/vertical only
                self._controller.send_velocity_body(0.0, vy, vz, 0.0)
                return

        # ── Phase 2 — APPROACH ───────────────────────────────────────────────
        dist = self._range.get_distance()

        if dist is not None and dist <= self._cfg_stop_dist:
            print(f"[APPROACH] Board in range ({dist:.2f}m) → CLEAN")
            self._controller.stop_movement()
            self._approach_phase = "ALIGN"
            self._state = MissionState.CLEAN
            return

        if dist is None:
            # Sensor not yet reading — creep forward slowly until confirmed
            vx = self._cfg_cautious_speed
        else:
            vx = self._cfg_kp_forward * max(0.0, dist - self._cfg_stop_dist)
            vx = min(vx, self._cfg_max_speed)   # hard cap — never exceeds max_speed

        self._controller.send_velocity_body(vx, vy, vz, 0.0)

    def _tick_clean(self, frame) -> None:
        """Spray then wipe: activate pump, then actuate the wiper arm."""
        print("[MISSION] CLEAN — spraying")
        self._pump.spray(self._cfg_pump_duration)
        print("[MISSION] CLEAN — wiping")
        self._wiper.wipe()
        print("[MISSION] Cleaning done → RETURN")
        self._state = MissionState.RETURN

    def _tick_return(self, frame) -> None:
        """Trigger RTL and wait for landing."""
        if not self._return_initiated:
            self._controller.rtl()
            self._return_initiated = True
            print("[MISSION] RTL initiated — waiting for landing")

        if self._controller.is_landed():
            print("[MISSION] Landed → DONE")
            self._state = MissionState.DONE

    # ─────────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _abort(self, reason: str) -> None:
        print(f"[MISSION] ABORT — {reason}")
        self._state = MissionState.ABORTED

    def _shutdown(self) -> None:
        """Safe teardown regardless of mission outcome."""
        print("[MISSION] Shutting down subsystems")
        try:
            self._controller.stop_movement()
        except Exception:
            pass
        try:
            self._pump.stop()
        except Exception:
            pass
        try:
            self._wiper.cleanup()
        except Exception:
            pass
        try:
            self._sensors.stop()
        except Exception:
            pass
        try:
            self._range.stop()
        except Exception:
            pass
        try:
            self._camera.stop()
        except Exception:
            pass
        try:
            self._controller.close()
        except Exception:
            pass
        try:
            self._pump.cleanup()
        except Exception:
            pass
        print("[MISSION] Shutdown complete")
