# -*- coding: utf-8 -*-

import serial

ser = serial.Serial("/dev/serial0", 115200, timeout=1)

def getTFminiData():
    while True:
        count = ser.in_waiting
        if count >= 9:
            recv = ser.read(9)

            if len(recv) == 9 and recv[0] == 0x59 and recv[1] == 0x59:
                distance = recv[2] + (recv[3] << 8)
                print(distance)

if __name__ == '__main__':
    try:
        if not ser.is_open:
            ser.open()
        getTFminiData()
    except KeyboardInterrupt:
        if ser is not None:
            ser.close()