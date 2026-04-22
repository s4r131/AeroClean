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

Requires tf_sensor.uart to be set in config.json before running.
"""

from __future__ import annotations

import argparse
import json
import time

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


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
            "Must contain tf_sensor.uart set to your UART device path,\n"
            "e.g.  \"tf_sensor\": { \"uart\": \"/dev/ttyAMA3\" }"
        ),
    )
    args = p.parse_args()

    with open(args.config) as f:
        cfg = json.load(f)

    tf_cfg = cfg.get("tf_sensor", {})
    uart   = tf_cfg.get("uart")
    baud   = tf_cfg.get("baud", 115200)

    if uart is None:
        p.error(
            "tf_sensor.uart is not set in config.json.\n"
            "  Find your device path with: ls -l /dev/ttyAMA*\n"
            "  Then set it in config.json:  \"tf_sensor\": { \"uart\": \"/dev/ttyAMAx\" }"
        )

    if not SERIAL_AVAILABLE:
        print("[TF TEST] pyserial is not installed. Run:  pip install pyserial")
        return

    print(f"[TF TEST] Opening {uart} @ {baud} baud")
    print("[TF TEST] Reading distance — Ctrl+C to stop\n")

    try:
        ser = serial.Serial(uart, baud, timeout=1)
    except serial.SerialException as e:
        print(f"[TF TEST] Could not open {uart}: {e}")
        print("[TF TEST] Check the UART path and that your user is in the dialout group.")
        return

    try:
        while True:
            result = read_frame(ser)
            if result is None:
                print("[TF TEST] No valid frame — check wiring and baud rate")
                time.sleep(0.5)
                continue

            dist_m, strength, temp_c = result

            if dist_m is None:
                print(f"[TF TEST] Distance invalid  | strength={strength}  | temp={temp_c:.1f} C")
            else:
                print(f"[TF TEST] {dist_m:.3f} m  ({dist_m * 100:.1f} cm)  | strength={strength}  | temp={temp_c:.1f} C")

    except KeyboardInterrupt:
        print("\n[TF TEST] Stopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
