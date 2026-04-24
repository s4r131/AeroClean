"""
sensor_tf_i2c_test.py — VL53L3CX ToF range sensor test (sensor B — I2C).

Prints live distance readings. No camera, no OpenCV.

Prerequisites:
    /boot/firmware/config.txt:  dtparam=i2c_arm=on
    Reboot, then verify:
        pinctrl -p  →  GPIO2 = SDA1, GPIO3 = SCL1
        sudo i2cdetect -y 1  →  expect 0x29

Usage:
    python sensor_tf_i2c_test.py
    python sensor_tf_i2c_test.py --config config.json
    Ctrl+C to stop.
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
import time

from sensors import RangeSensor


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


def main() -> None:
    p = argparse.ArgumentParser(
        description=(
            "VL53L3CX ToF range sensor test (sensor B — I2C).\n"
            "\n"
            "Reads distance from the VL53L3CX over I2C and prints live readings\n"
            "in metres and centimetres. No camera or OpenCV required.\n"
            "\n"
            "Use this script to:\n"
            "  - Confirm the sensor is wired and detected on the I2C bus\n"
            "  - Verify the I2C address matches config.json (default 0x29)\n"
            "  - Check live distance readings before running the full mission\n"
            "  - Tune timing_budget_ms (lower = faster but noisier readings)\n"
            "\n"
            "Troubleshooting:\n"
            "  No readings / error  →  run: sudo i2cdetect -y 1  (expect 0x29)\n"
            "  I2C not enabled      →  add dtparam=i2c_arm=on to /boot/firmware/config.txt and reboot\n"
            "  Wrong address        →  update range_sensor.i2c_address in config.json\n"
            "  Noisy readings       →  increase timing_budget_ms in config.json (try 100)\n"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument(
        "--config", default="config.json",
        help=(
            "Path to config.json (default: config.json).\n"
            "Reads range_sensor.i2c_address and range_sensor.timing_budget_ms."
        ),
    )
    args = p.parse_args()

    with open(args.config) as f:
        cfg = json.load(f)

    range_cfg       = cfg.get("range_sensor", {})
    i2c_address     = range_cfg.get("i2c_address", 0x29)
    timing_budget   = int(range_cfg.get("timing_budget_ms", 50))

    _banner([
        [
            "AeroClean — I2C Range Sensor Test",
            "Verifies VL53L3CX is wired and responding over I2C",
        ],
        [
            f"I2C address   : 0x{i2c_address:02X}",
            f"Timing budget : {timing_budget} ms",
        ],
        [
            "Expect : distance prints only when value changes",
            "Stop   : Ctrl+C",
        ],
    ])

    sensor = RangeSensor(i2c_address, timing_budget_ms=timing_budget)
    sensor.start()

    last_dist_cm = None
    got_first    = False
    spin         = itertools.cycle(r"|/-\\")

    try:
        while True:
            dist = sensor.get_distance()
            if dist is None:
                if not got_first:
                    sys.stdout.write(f"\r  {next(spin)}  Waiting for sensor data...")
                    sys.stdout.flush()
                time.sleep(0.1)
                continue

            if not got_first:
                sys.stdout.write("\r" + " " * 50 + "\r")
                sys.stdout.flush()
                got_first = True

            dist_cm = round(dist * 100, 1)
            if dist_cm != last_dist_cm:
                print(f"[RANGE TEST] {dist:.3f} m  ({dist_cm:.1f} cm)")
                last_dist_cm = dist_cm
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[RANGE TEST] Stopped.")
    finally:
        sensor.stop()


if __name__ == "__main__":
    main()
