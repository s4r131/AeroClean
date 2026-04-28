"""
pi_mavlink_test.py — Pi → FC MAVLink heartbeat and telemetry check.

Connects to the ArduPilot flight controller over UART, waits for a
heartbeat, prints FC state, then streams live telemetry until Ctrl+C.

No arming, no movement. Safe to run on a bench with or without props.

Prerequisites:
    config.json configured:
        mission.mavlink_uart  — UART path to FC (e.g. /dev/ttyAMA0)
        mission.mavlink_baud  — baud rate (default 57600)
    pymavlink installed:
        pip install pymavlink

Usage:
    python pi_mavlink_test.py
    python pi_mavlink_test.py --config /path/to/config.json
    Ctrl+C to stop.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

from pymavlink import mavutil


def _banner(groups: list[list[str]]) -> None:
    width = max(len(l) for g in groups for l in g) + 4
    sep   = "═" * width
    print(f"╔{sep}╗")
    for i, group in enumerate(groups):
        for line in group:
            print(f"║  {line:<{width - 2}}║")
        if i < len(groups) - 1:
            print(f"╠{sep}╣")
    print(f"╚{sep}╝")


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "AeroClean — Pi to FC MAVLink heartbeat and telemetry check.\n"
            "\n"
            "Connects to the ArduPilot FC over UART, waits for a heartbeat,\n"
            "then streams live armed state, flight mode, altitude, and heading.\n"
            "\n"
            "Use this to confirm:\n"
            "  - UART wiring between Pi and FC is correct\n"
            "  - Baud rate matches SERIALx_BAUD set on the FC\n"
            "  - FC is powered and running ArduPilot\n"
            "  - Altitude and heading data are streaming\n"
            "\n"
            "Troubleshooting:\n"
            "  No heartbeat      →  check TX/RX wiring (swap if needed) and baud rate\n"
            "  Permission denied →  run: sudo usermod -aG dialout $USER  then reboot\n"
            "  Port not found    →  run: ls -l /dev/ttyAMA*  to list UART devices\n"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument("--config", default="config.json",
                   help="Path to config.json (default: config.json).")
    return p


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    args = _build_parser().parse_args()

    if not os.path.exists(args.config):
        print(f"[ERROR] config not found: {args.config}", file=sys.stderr)
        sys.exit(1)

    with open(args.config) as f:
        cfg = json.load(f)

    m    = cfg.get("mission", {})
    uart = m.get("mavlink_uart")
    baud = int(m.get("mavlink_baud", 57600))

    if uart is None:
        _build_parser().error(
            "mission.mavlink_uart is not set in config.json.\n"
            "  Find your UART with: ls -l /dev/ttyAMA*\n"
            "  Then set: \"mission\": { \"mavlink_uart\": \"/dev/ttyAMAx\" }"
        )

    _banner([
        [
            "AeroClean — Pi to FC MAVLink Test",
            "Verifies UART link, heartbeat, and telemetry stream",
        ],
        [
            f"UART : {uart}",
            f"Baud : {baud}",
        ],
        [
            "Expect : heartbeat confirms FC is alive, telemetry prints on change",
            "Stop   : Ctrl+C",
        ],
    ])

    print(f"[FC TEST] Connecting on {uart} @ {baud}...")
    conn = mavutil.mavlink_connection(uart, baud=baud)

    print("[FC TEST] Waiting for heartbeat (10s timeout)...")
    hb = conn.wait_heartbeat(timeout=10)

    if hb is None:
        print("[FC TEST] FAIL — no heartbeat received.")
        print("  Check: TX/RX wiring, baud rate, FC is powered and running ArduPilot.")
        sys.exit(1)

    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    print(f"[FC TEST] OK — heartbeat received")
    print(f"  System ID : {conn.target_system}")
    print(f"  Component : {conn.target_component}")
    print(f"  Mode      : {hb.custom_mode}  (0=STABILIZE  4=GUIDED  6=RTL)")
    print(f"  Armed     : {'YES' if armed else 'NO'}")
    print()
    print("[FC TEST] Streaming telemetry — Ctrl+C to stop")

    prev_armed = armed
    prev_mode  = hb.custom_mode
    prev_alt   = None
    prev_hdg   = None

    try:
        while True:
            msg = conn.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT"],
                                  blocking=True, timeout=1.0)
            if msg is None:
                continue

            if msg.get_type() == "HEARTBEAT":
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                mode  = msg.custom_mode
                if armed != prev_armed or mode != prev_mode:
                    print(f"[FC TEST] {'ARMED' if armed else 'DISARMED'}  MODE:{mode}")
                    prev_armed = armed
                    prev_mode  = mode

            elif msg.get_type() == "GLOBAL_POSITION_INT":
                alt = round(msg.relative_alt / 1000.0, 2)
                hdg = round(msg.hdg / 100.0, 1)
                if alt != prev_alt or hdg != prev_hdg:
                    print(f"[FC TEST] ALT:{alt:.2f}m  HDG:{hdg:.1f}°")
                    prev_alt = alt
                    prev_hdg = hdg

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[FC TEST] Stopped.")
    finally:
        try:
            conn.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
