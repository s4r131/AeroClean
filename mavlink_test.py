"""
mavlink_test.py — Interactive MAVLink flight command test.

Verifies that the Raspberry Pi can communicate with the ArduPilot FC, arm the
drone, and respond correctly to velocity commands. Run this outdoors in an open
area before the first autonomous mission.

The movement test uses very low speeds pulled directly from config.json so there
is no need to edit this script — adjust approach_cautious_speed_ms and
scan_yaw_rate_dps in config.json to change test movement speed.

Prerequisites:
    config.json configured:
        mission.mavlink_uart       — UART path to ArduPilot FC
        mission.mavlink_baud       — baud rate (default 57600)
        mission.approach_cautious_speed_ms  — movement test speed (default 0.05)
        mission.scan_yaw_rate_dps  — yaw test rate in deg/s (default 20.0)
    pymavlink installed:
        pip install pymavlink
    Outdoor, clear area with props removed for non-flight steps.

Safety:
    - Arm step requires explicit confirmation before proceeding.
    - Every movement command calls stop_movement() in a finally block.
    - Any unhandled exception triggers RTL before exiting.

Usage:
    python mavlink_test.py
    python mavlink_test.py --config /path/to/config.json --altitude 1.5
    Ctrl+C at any prompt returns to the main menu.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

from mavlink_controller import MAVLinkController


# Duration of each movement burst — short enough to be safe on a bench/outdoors
_TEST_DURATION_S = 1.5


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _print_status(ctrl: MAVLinkController) -> None:
    """Print a one-line telemetry snapshot."""
    armed   = "ARMED" if ctrl.is_armed() else "DISARMED"
    mode    = ctrl.get_mode()
    alt     = ctrl.get_altitude_m()
    hdg     = ctrl.get_heading_deg()
    alt_str = f"{alt:.2f}m" if alt is not None else "---"
    hdg_str = f"{hdg:.0f}°" if hdg is not None else "---"
    print(f"  {armed}  MODE:{mode}  ALT:{alt_str}  HDG:{hdg_str}")


def _confirm(prompt: str) -> bool:
    """Return True only if the user types 'y'."""
    try:
        answer = input(f"{prompt} [y/N]: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        return False
    return answer == "y"


# ─────────────────────────────────────────────────────────────────────────────
# Menu actions
# ─────────────────────────────────────────────────────────────────────────────

def _action_connect(ctrl: MAVLinkController) -> None:
    try:
        ctrl.connect()
        print("[TEST] Connected.")
        _print_status(ctrl)
    except Exception as e:
        print(f"[ERROR] Connection failed: {e}")


def _action_telemetry(ctrl: MAVLinkController) -> None:
    print("[TEST] Live telemetry — Ctrl+C to return to menu")
    try:
        while True:
            _print_status(ctrl)
            time.sleep(1.0)
    except KeyboardInterrupt:
        print()


def _action_arm(ctrl: MAVLinkController) -> None:
    print("[TEST] Current status:")
    _print_status(ctrl)
    if not _confirm("[WARN] About to ARM. Props MUST be removed. Confirm"):
        print("[TEST] Arm cancelled.")
        return
    try:
        ctrl.arm()
        print("[TEST] Armed — use [4] Takeoff when ready to fly.")
        _print_status(ctrl)
    except Exception as e:
        print(f"[ERROR] Arm failed: {e}")


def _action_takeoff(ctrl: MAVLinkController, altitude_m: float) -> None:
    print(f"[TEST] Current status:")
    _print_status(ctrl)
    if not _confirm(f"[WARN] About to ARM and TAKE OFF to {altitude_m:.1f} m. Confirm"):
        print("[TEST] Takeoff cancelled.")
        return
    try:
        ctrl.arm_and_takeoff(altitude_m)
        print(f"[TEST] Airborne at {altitude_m:.1f} m.")
        _print_status(ctrl)
    except Exception as e:
        print(f"[ERROR] Takeoff failed: {e}")


def _action_movement(ctrl: MAVLinkController, speed_ms: float, yaw_rate_rads: float) -> None:
    """Movement submenu — each burst lasts _TEST_DURATION_S then stops."""
    _MOVES = {
        "w": ("Forward",   ( speed_ms, 0.0,       0.0,          0.0)),
        "s": ("Backward",  (-speed_ms, 0.0,       0.0,          0.0)),
        "a": ("Left",      (0.0,      -speed_ms,  0.0,          0.0)),
        "d": ("Right",     (0.0,       speed_ms,  0.0,          0.0)),
        "r": ("Up",        (0.0,       0.0,       -speed_ms,    0.0)),
        "f": ("Down",      (0.0,       0.0,        speed_ms,    0.0)),
        "y": ("Yaw CW",    (0.0,       0.0,        0.0,          yaw_rate_rads)),
        "Y": ("Yaw CCW",   (0.0,       0.0,        0.0,         -yaw_rate_rads)),
    }

    while True:
        print()
        print("  ── Movement test ───────────────────────────────────────")
        for key, (label, _) in _MOVES.items():
            if key in ("y", "Y"):
                print(f"  [{key}] {label:<12}  {math.degrees(yaw_rate_rads):.0f} deg/s  {_TEST_DURATION_S:.1f}s")
            else:
                print(f"  [{key}] {label:<12}  {speed_ms:.3f} m/s  {_TEST_DURATION_S:.1f}s")
        print(f"  [x] Stop (hold position)")
        print(f"  [q] Back to main menu")
        print("  ────────────────────────────────────────────────────────")
        _print_status(ctrl)

        try:
            key = input("  Move: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if key == "q":
            break

        if key == "x":
            ctrl.stop_movement()
            print("[TEST] Stopped.")
            continue

        move = _MOVES.get(key)
        if move is None:
            print(f"[TEST] Unknown key '{key}'")
            continue

        label, (vx, vy, vz, yr) = move
        print(f"[TEST] {label} for {_TEST_DURATION_S:.1f}s ...")
        try:
            ctrl.send_velocity_body(vx, vy, vz, yr)
            time.sleep(_TEST_DURATION_S)
        finally:
            ctrl.stop_movement()


def _action_rtl(ctrl: MAVLinkController) -> None:
    if not _confirm("[WARN] About to trigger RTL. Confirm"):
        print("[TEST] RTL cancelled.")
        return
    try:
        ctrl.rtl()
        print("[TEST] RTL triggered — waiting for landing (120s timeout)...")
        deadline = time.monotonic() + 120.0
        while not ctrl.is_landed():
            if time.monotonic() > deadline:
                print("[WARN] Landing timeout — verify drone landed manually.")
                break
            _print_status(ctrl)
            time.sleep(1.0)
        else:
            print("[TEST] Landed.")
    except Exception as e:
        print(f"[ERROR] RTL failed: {e}")


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "AeroClean — interactive MAVLink flight command test.\n"
            "\n"
            "Verifies FC communication, arming, and velocity commands.\n"
            "Run outdoors in an open area. Remove props before arming.\n"
            "\n"
            "Movement speed is read from config.json:\n"
            "  mission.approach_cautious_speed_ms  — linear axes\n"
            "  mission.scan_yaw_rate_dps            — yaw axis\n"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument("--config",   default="config.json", help="Path to config.json (default: config.json).")
    p.add_argument("--altitude", type=float, default=None,
                   help="Takeoff altitude in metres (default: mission.takeoff_altitude_m from config).")
    return p


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    args = _build_parser().parse_args()

    if not os.path.exists(args.config):
        print(f"[ERROR] config.json not found at '{args.config}'", file=sys.stderr)
        sys.exit(1)
    with open(args.config) as f:
        cfg = json.load(f)

    m = cfg.get("mission", {})

    mavlink_uart = m.get("mavlink_uart")
    if mavlink_uart is None:
        print(
            "[ERROR] mission.mavlink_uart is not set in config.json.\n"
            "  Find your UART with: ls -l /dev/ttyAMA*\n"
            "  Then set: \"mission\": { \"mavlink_uart\": \"/dev/ttyAMAx\" }",
            file=sys.stderr,
        )
        sys.exit(1)

    mavlink_baud = int(m.get("mavlink_baud", 57600))
    altitude_m   = args.altitude if args.altitude is not None else float(m.get("takeoff_altitude_m", 1.5))
    speed_ms     = float(m.get("approach_cautious_speed_ms", 0.05))
    yaw_rate_rads = math.radians(float(m.get("scan_yaw_rate_dps", 20.0)))

    ctrl = MAVLinkController(mavlink_uart, baud=mavlink_baud)

    print("╔══════════════════════════════════════════════════════╗")
    print("║  AeroClean — MAVLink Test                            ║")
    print("║  Verify FC communication and flight commands         ║")
    print("╠══════════════════════════════════════════════════════╣")
    uart_display = mavlink_uart if len(mavlink_uart) <= 42 else mavlink_uart[:39] + "..."
    print(f"║  UART    : {uart_display:<42}║")
    print(f"║  Baud    : {mavlink_baud:<42}║")
    print(f"║  Speed   : {speed_ms:.3f} m/s  Yaw: {math.degrees(yaw_rate_rads):.0f} deg/s{'':<26}║")
    print(f"║  Altitude: {altitude_m:.1f} m{'':<43}║")
    print("╚══════════════════════════════════════════════════════╝")

    try:
        while True:
            print()
            print("  ── Main menu ───────────────────────────────────────────")
            print("  [1] Connect to FC")
            print("  [2] Live telemetry  (Ctrl+C to return)")
            print("  [3] Arm             (confirmation required)")
            print(f"  [4] Takeoff to {altitude_m:.1f} m")
            print("  [5] Movement test")
            print("  [6] RTL")
            print("  [q] Quit")
            print("  ────────────────────────────────────────────────────────")

            try:
                choice = input("  Choice: ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if choice == "q":
                break
            elif choice == "1":
                _action_connect(ctrl)
            elif choice == "2":
                _action_telemetry(ctrl)
            elif choice == "3":
                _action_arm(ctrl)
            elif choice == "4":
                _action_takeoff(ctrl, altitude_m)
            elif choice == "5":
                _action_movement(ctrl, speed_ms, yaw_rate_rads)
            elif choice == "6":
                _action_rtl(ctrl)
            else:
                print(f"[TEST] Unknown choice '{choice}'")

    except KeyboardInterrupt:
        print("\n[TEST] Stopped.")
    except Exception as e:
        print(f"\n[ERROR] Unhandled exception: {e}")
        print("[TEST] Attempting RTL before exit...")
        try:
            ctrl.rtl()
        except Exception:
            pass
    finally:
        try:
            ctrl.stop_movement()
        except Exception:
            pass
        try:
            ctrl.close()
        except Exception:
            pass
        print("[TEST] Done.")


if __name__ == "__main__":
    main()
