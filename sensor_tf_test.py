"""
sensor_tf_test.py — TF-Luna / TFMini UART range sensor test (sensor A — default).

Reads the Benewake binary frame protocol (0x59 0x59 header) over UART
and prints distance in metres. Use this to verify the TF-series sensor
is wired and responding correctly.

Prerequisites:
    UART enabled in /boot/firmware/config.txt (e.g. dtoverlay=uart3)
    Reboot, then verify:
        pinctrl -p         →  pins show alternate function (not none)
        ls -l /dev/ttyAMA* →  note your device path
    pyserial installed:
        pip install pyserial

Usage:
    python sensor_tf_test.py
    python sensor_tf_test.py --config config.json
    Ctrl+C to stop.

Requires range_sensor.uart to be set in config.json before running.
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
import time

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


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


def read_frame(ser) -> tuple[float | None, int, float] | None:
    """
    Parse one TF-Luna / TFMini 9-byte binary frame.

    Frame layout: 0x59  0x59  distL  distH  strL  strH  tempL  tempH  checksum
    Returns (distance_m, strength, temp_c).
      distance_m is None if the sensor reports an invalid reading (0xFFFF).
    Returns None if the frame cannot be read or fails checksum.
    """
    while True:
        b1 = ser.read(1)
        if not b1:
            return None
        if b1[0] != 0x59:
            continue

        b2 = ser.read(1)
        if not b2 or b2[0] != 0x59:
            continue

        rest = ser.read(7)
        if len(rest) != 7:
            return None

        frame = bytes([0x59, 0x59]) + rest

        if (sum(frame[:8]) & 0xFF) != frame[8]:
            continue

        dist_cm  = frame[2] | (frame[3] << 8)
        strength = frame[4] | (frame[5] << 8)
        temp_c   = (frame[6] | (frame[7] << 8)) / 8.0 - 256

        dist_m = None if dist_cm == 0xFFFF else dist_cm / 100.0
        return dist_m, strength, temp_c


def main() -> None:
    p = argparse.ArgumentParser(
        description=(
            "TF-Luna / TFMini UART range sensor test.\n"
            "\n"
            "Reads the Benewake 9-byte binary frame over UART and prints distance,\n"
            "signal strength, and chip temperature to the terminal every frame.\n"
            "\n"
            "Use this script to:\n"
            "  - Confirm the sensor is wired correctly and sending data\n"
            "  - Verify the UART path and baud rate in config.json\n"
            "  - Check signal strength (low strength = weak reflection, aim for >100)\n"
            "  - Spot out-of-range readings (sensor reports distance=invalid)\n"
            "\n"
            "Troubleshooting:\n"
            "  No data / timeout   →  check TX/RX wiring (swap if needed)\n"
            "  Wrong baud rate     →  TF-Luna default is 115200, TFMini is also 115200\n"
            "  Permission denied   →  run: sudo usermod -aG dialout $USER  then reboot\n"
            "  Port not found      →  run: ls -l /dev/ttyAMA*  to find your device\n"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument(
        "--config", default="config.json",
        help=(
            "Path to config.json (default: config.json).\n"
            "Must contain range_sensor.uart set to your UART device path,\n"
            "e.g.  \"range_sensor\": { \"uart\": \"/dev/ttyAMA3\" }"
        ),
    )
    args = p.parse_args()

    with open(args.config) as f:
        cfg = json.load(f)

    tf_cfg = cfg.get("range_sensor", {})
    uart   = tf_cfg.get("uart")
    baud   = tf_cfg.get("baud", 115200)

    if uart is None:
        p.error(
            "range_sensor.uart is not set in config.json.\n"
            "  Find your device path with: ls -l /dev/ttyAMA*\n"
            "  Then set it in config.json:  \"range_sensor\": { \"uart\": \"/dev/ttyAMAx\" }"
        )

    if not SERIAL_AVAILABLE:
        print("[TF TEST] pyserial is not installed. Run:  pip install pyserial")
        return

    _banner([
        [
            "AeroClean — TF Sensor Test",
            "Verifies TF-Luna / TFMini is wired and responding over UART",
        ],
        [
            f"UART : {uart}",
            f"Baud : {baud}",
        ],
        [
            "Expect : distance prints only when value changes",
            "Stop   : Ctrl+C",
        ],
    ])

    try:
        ser = serial.Serial(uart, baud, timeout=1)
    except serial.SerialException as e:
        print(f"[TF TEST] Could not open {uart}: {e}")
        print("[TF TEST] Check the UART path and that your user is in the dialout group.")
        return

    last_dist_cm = None
    got_first    = False
    spin         = itertools.cycle(r"|/-\\")

    try:
        while True:
            try:
                result = read_frame(ser)
            except serial.SerialException as e:
                if not got_first:
                    print()
                print(f"[TF TEST] Serial error: {e} — reconnect sensor and restart.")
                break

            if result is None:
                if not got_first:
                    sys.stdout.write(f"\r  {next(spin)}  Waiting for sensor data...")
                    sys.stdout.flush()
                else:
                    print("[TF TEST] No valid frame — check wiring and baud rate")
                    time.sleep(0.5)
                continue

            if not got_first:
                sys.stdout.write("\r" + " " * 50 + "\r")
                sys.stdout.flush()
                got_first = True

            dist_m, _, _ = result
            dist_cm = None if dist_m is None else round(dist_m * 100, 1)

            if dist_cm != last_dist_cm:
                if dist_m is None:
                    print("[TF TEST] Distance invalid")
                else:
                    print(f"[TF TEST] {dist_m:.3f} m  ({dist_cm:.1f} cm)")
                last_dist_cm = dist_cm

    except KeyboardInterrupt:
        print("\n[TF TEST] Stopped.")
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
