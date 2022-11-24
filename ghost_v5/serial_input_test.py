import serial
import time

ser = serial.Serial('/dev/ttyACM1')  # open serial port

while(True):
    print(ser.read_until(b'\x00'))
ser.close()             # close port