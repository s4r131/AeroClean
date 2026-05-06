"""
mission.py — AeroClean autonomous drone mission state machine.

Mission flow:
    IDLE → SCAN → APPROACH → CLEAN → RETREAT → DONE

        SCAN timeout ──────────────────────────────┐
        APPROACH board lost ──────┐                │
                                  ▼                ▼
                                SCAN           RETURN

    Any unhandled exception → ABORTED (shutdown + RTL attempted)

Note: IDLE handles arm + takeoff directly before transitioning to SCAN.

Subsystems wired together here:
    Camera (camera.py)            — frame capture
    YOLOModel (yolo_model.py)     — dirty_board detection
    MAVLinkController             — ArduPilot GUIDED mode commands
    RangeSensor / TFRangeSensor   — forward range (type set by range_sensor.type in config.json)
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
from sensors import RangeSensor, TFRangeSensor
from wiper import Wiper
from yolo_model import YOLOModel


# Consecutive frames without a detection before APPROACH → SCAN fallback
_APPROACH_LOST_LIMIT = 10


class MissionState(Enum):
    IDLE     = auto()
    SCAN     = auto()
    APPROACH = auto()
    CLEAN    = auto()
    RETREAT  = auto()   # fly back in body frame, then land in place
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
        self._cfg_clean_timeout     = float(m.get("clean_timeout_s", 30.0))
        self._cfg_retreat_dist      = float(m.get("post_clean_retreat_m", 0.3))
        self._cfg_retreat_speed     = float(m.get("post_clean_retreat_speed_ms", 0.3))
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
        self._use_ocr    = m.get("detection_model", "yolo") == "ocr"
        if self._use_ocr:
            from ocr_model import OCRModel
            self._ocr  = OCRModel(config_path)
            self._yolo = None
            print("[MISSION] Detection model: OCR (Tesseract)")
        else:
            self._yolo = YOLOModel(config_path)
            self._ocr  = None
            print("[MISSION] Detection model: YOLO NCNN")
        self._controller = MAVLinkController(m["mavlink_uart"], int(m["mavlink_baud"]))
        self._range      = self._init_range_sensor(cfg)
        pump_pin = m.get("pump_gpio_pin")
        self._pump       = Pump(int(pump_pin) if pump_pin is not None else None)
        w_cfg = cfg.get("wiper", {})
        self._wiper      = Wiper(
            pin             = w_cfg.get("wiper_gpio_pin"),
            wipe_duration_s = float(w_cfg.get("wipe_duration_s", 2.0)),
        )

        self._state: MissionState = MissionState.IDLE

        # SCAN state
        self._scan_start_time: float | None = None

        # APPROACH state
        self._target_bbox: list | None  = None
        self._approach_lost_count: int  = 0
        self._approach_phase: str       = "ALIGN"   # "ALIGN" or "APPROACH"

        # CLEAN state
        self._clean_start_time: float | None = None

        # RETREAT state
        self._retreat_start_time: float | None = None
        self._retreat_duration:   float        = 0.0
        self._retreat_landing:    bool         = False

        # RETURN state
        self._return_initiated: bool = False

    # ─────────────────────────────────────────────────────────────────────────
    # Range sensor factory
    # ─────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _init_range_sensor(cfg: dict) -> RangeSensor | TFRangeSensor:
        """
        Instantiate the forward range sensor based on range_sensor.type in config.json.

          "a"  (default) — TF-Luna/TFMini over UART (TFRangeSensor)
          "b"            — VL53L3CX over I2C  (RangeSensor)
        """
        range_cfg   = cfg.get("range_sensor", {})
        sensor_type = str(range_cfg.get("type", "a")).lower()

        if sensor_type == "b":
            i2c_address   = range_cfg.get("i2c_address", 0x29)
            timing_budget = int(range_cfg.get("timing_budget_ms", 50))
            print(f"[MISSION] Range sensor: VL53L3CX (I2C 0x{i2c_address:02X}, {timing_budget}ms budget)")
            return RangeSensor(i2c_address, timing_budget_ms=timing_budget)

        # Default — sensor A
        tf_uart = range_cfg.get("uart")
        if not tf_uart:
            raise RuntimeError(
                "range_sensor.type is 'a' (TF-Luna) but range_sensor.uart is not set "
                "in config.json. Set it to the UART path for the TF sensor "
                "(e.g. /dev/ttyAMAx)."
            )
        tf_baud = int(range_cfg.get("baud", 115200))
        print(f"[MISSION] Range sensor: TF-Luna/TFMini on {tf_uart} @ {tf_baud}")
        return TFRangeSensor(tf_uart, baud=tf_baud)

    # ─────────────────────────────────────────────────────────────────────────
    # Entry point
    # ─────────────────────────────────────────────────────────────────────────

    def run(self) -> None:
        """Start the mission. Blocks until DONE or ABORTED."""
        print("[MISSION] Starting up")
        try:
            self._camera.start()
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
            MissionState.RETREAT:  self._tick_retreat,
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
        """Spin and search for a dirty board using the configured detection model."""
        if self._scan_start_time is None:
            self._scan_start_time = time.monotonic()
            print("[MISSION] SCAN started — yawing to search for dirty board")

        # Keep yawing
        self._controller.send_velocity_body(0.0, 0.0, 0.0, self._cfg_scan_yaw_rate)

        detection, annotated = self._run_detector(frame)

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
        detection, annotated = self._run_detector(frame)

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
        if self._clean_start_time is None:
            self._clean_start_time = time.monotonic()

        elapsed = time.monotonic() - self._clean_start_time
        if elapsed > self._cfg_clean_timeout:
            print(f"[MISSION] Clean timeout ({self._cfg_clean_timeout:.0f}s) — returning regardless")
            self._clean_start_time = None
            self._state = MissionState.RETURN
            return

        print("[MISSION] CLEAN — spraying")
        self._pump.spray(self._cfg_pump_duration)
        print("[MISSION] CLEAN — wiping")
        self._wiper.wipe()
        print("[MISSION] Cleaning done → RETREAT")
        self._clean_start_time = None
        self._state = MissionState.RETREAT

    def _tick_retreat(self, frame) -> None:
        """Fly straight back in body frame (no yaw), then land in place."""
        if self._retreat_start_time is None:
            self._retreat_duration   = self._cfg_retreat_dist / self._cfg_retreat_speed
            self._retreat_start_time = time.monotonic()
            self._retreat_landing    = False
            print(
                f"[MISSION] RETREAT — flying back {self._cfg_retreat_dist:.2f}m "
                f"at {self._cfg_retreat_speed:.2f} m/s"
            )

        if not self._retreat_landing:
            elapsed = time.monotonic() - self._retreat_start_time
            if elapsed < self._retreat_duration:
                self._controller.send_velocity_body(-self._cfg_retreat_speed, 0.0, 0.0, 0.0)
            else:
                self._controller.stop_movement()
                print("[MISSION] Retreat complete — landing in place")
                self._controller.land()
                self._retreat_landing = True
            return

        if self._controller.is_landed():
            print("[MISSION] Landed → DONE")
            self._retreat_start_time = None
            self._state = MissionState.DONE

    def _tick_return(self, frame) -> None:
        """Land in place and wait for touchdown."""
        if not self._return_initiated:
            self._controller.land()
            self._return_initiated = True
            print("[MISSION] Land initiated — waiting for touchdown")

        if self._controller.is_landed():
            print("[MISSION] Landed → DONE")
            self._state = MissionState.DONE

    # ─────────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _run_detector(self, frame):
        """Run the configured detection model. Always returns (detection | None, annotated)."""
        if self._use_ocr:
            dirty, annotated = self._ocr.run(frame)
            if dirty:
                h, w = frame.shape[:2]
                detection = {
                    "class_name": "dirty_board",
                    "confidence": 0.0,
                    "bbox": [w // 4, h // 4, 3 * w // 4, 3 * h // 4],
                }
                return detection, annotated
            return None, annotated
        return self._yolo.run(frame)

    def _abort(self, reason: str) -> None:
        print(f"[MISSION] ABORT — {reason}")
        self._state = MissionState.ABORTED
        try:
            self._controller.land()
        except Exception:
            pass

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
