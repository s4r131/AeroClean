import time

from VL53L3CX import VL53L3CX  # package name from pip; adjust import if needed

sensor = VL53L3CX()
sensor.open()
sensor.start_ranging()

print("VL53L3CX test running...")
try:
    while True:
        if sensor.is_ranging_ready():
            dist_mm = sensor.get_distance()
            print(f"Distance: {dist_mm} mm")
        time.sleep(0.05)
finally:
    sensor.stop_ranging()