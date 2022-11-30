import serial
import time
from cobs import cobs

ser = serial.Serial('/dev/ttyACM1')  # open serial port
ser.flushInput()
while(True):
    buffer = ser.read_until(b'\x00')
    print(buffer)
    decoded_packet = cobs.decode(buffer[:-1])
    print(decoded_packet)

    # btns = int.from_bytes(decoded_packet[-4:-2], "little")
    # print(decoded_packet[-4:])
    # print(len(decoded_packet[-4:-2]))
    # print(bin(btns))
    # print(btns)
    # print(btns & 0x8000, end = "")
    # print(btns & 0x4000, end = "")
    # print(btns & 0x2000, end = "")
    # print(btns & 0x1000, end = "")
    # print(btns & 0x0800, end = "")
    # print(btns & 0x0400, end = "")
    # print(btns & 0x0200, end = "")
    # print(btns & 0x0100, end = "")
    # print(btns & 0x0080, end = "")
    # print(btns & 0x0040, end = "")
    # print(btns & 0x0020, end = "")
    # print(btns & 0x0010, end = "")
    print()

    print()
    print()
ser.close()             # close portc