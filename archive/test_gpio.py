import RPi.GPIO as GPIO
import time

PIN = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.OUT)

print("LED ON")
GPIO.output(PIN, GPIO.HIGH)

time.sleep(3)

print("LED OFF")
GPIO.output(PIN, GPIO.LOW)

GPIO.cleanup()