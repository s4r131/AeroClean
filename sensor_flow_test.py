"""
sensor_flow_test.py — MTF-02P optical flow sensor test.

Prints live flow and distance readings. No camera, no OpenCV.

Prerequisites:
    /boot/firmware/config.txt:  dtoverlay=uart3  (or uartX for your port)
    Reboot, then verify:
        pinctrl -p  →  GPIO8 = TXD3, GPIO9 = RXD3
        ls -l /dev/ttyAMA*  →  note your device path
    Set mission.sensor_uart in config.json before running.

Usage:
    python sensor_flow_test.py
    python sensor_flow_test.py --config config.json
    Ctrl+C to stop.
"""

from __future__ import annotations

import argparse
import json
import time

from sensors import SensorReader


def main() -> None:
    p = argparse.ArgumentParser(description="MTF-02P optical flow sensor test")
    p.add_argument("--config", default="config.json")
    args = p.parse_args()

    with open(args.config) as f:
        cfg = json.load(f)

    mission_cfg = cfg.get("mission", {})
    uart = mission_cfg.get("sensor_uart")
    baud = int(mission_cfg.get("sensor_baud", 115200))

    if uart is None:
        p.error(
            "mission.sensor_uart is not set in config.json.\n"
            "  Find your device path with: ls -l /dev/ttyAMA*\n"
            "  Then set it in config.json:  \"mission\": { \"sensor_uart\": \"/dev/ttyAMAx\" }"
        )

    reader = SensorReader(uart, baud=baud)
    reader.start()

    print(f"[FLOW TEST] SensorReader started on {uart} @ {baud}")
    print("[FLOW TEST] Waiting for data — Ctrl+C to stop")
    print("[FLOW TEST] Live readings will appear below as the sensor sends them:\n")

    try:
        while True:
            time.sleep(1.0)
            dist = reader.get_distance()
            flow = reader.get_optical_flow()
            if dist is None and flow is None:
                print("[FLOW TEST] No data yet — check UART wiring and baud rate")
            else:
                if dist is not None:
                    print(f"[FLOW TEST] distance={dist:.3f}m")
                if flow is not None:
                    print(f"[FLOW TEST] flow_x={flow['flow_x']:.3f}  "
                          f"flow_y={flow['flow_y']:.3f}  quality={flow['quality']}")
    except KeyboardInterrupt:
        print("\n[FLOW TEST] Stopped.")
    finally:
        reader.stop()


if __name__ == "__main__":
    main()
