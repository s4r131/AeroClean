"""
sensor_range_test.py — VL53L3CX ToF range sensor test.

Prints live distance readings. No camera, no OpenCV.

Prerequisites:
    /boot/firmware/config.txt:  dtparam=i2c_arm=on
    Reboot, then verify:
        pinctrl -p  →  GPIO2 = SDA1, GPIO3 = SCL1
        sudo i2cdetect -y 1  →  expect 0x29

Usage:
    python sensor_range_test.py
    python sensor_range_test.py --config config.json
    Ctrl+C to stop.
"""

from __future__ import annotations

import argparse
import json
import time

from sensors import RangeSensor


def main() -> None:
    p = argparse.ArgumentParser(
        description=(
            "VL53L3CX ToF range sensor test (sensor A — I2C).\n"
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

    sensor = RangeSensor(i2c_address, timing_budget_ms=timing_budget)
    sensor.start()

    print(f"[RANGE TEST] Sensor started at I2C address 0x{i2c_address:02X}")
    print("[RANGE TEST] Reading distance — Ctrl+C to stop\n")

    try:
        while True:
            dist = sensor.get_distance()
            if dist is None:
                print("[RANGE TEST] Waiting for first reading...")
            else:
                print(f"[RANGE TEST] {dist:.3f} m  ({dist * 100:.1f} cm)")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[RANGE TEST] Stopped.")
    finally:
        sensor.stop()


if __name__ == "__main__":
    main()
