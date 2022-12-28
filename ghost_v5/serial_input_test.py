import serial
import time
from cobs import cobs
import signal

global run_
run_ = True

def handler(signum, frame):
    print("Signal Handler")
    global run_
    run_ = False

def main():
    global run_
    signal.signal(signal.SIGINT, handler)

    ser = serial.Serial('/dev/ttyACM1', timeout=0.5)  # open serial port
    ser.flushInput()
    while(run_):
        write_str = b"beepo"
        print()
        print("Writing: " + str(write_str))
        start = time.time_ns()
        ser.write(write_str)

        buffer = ser.read_until(b'\x00')
        end = time.time_ns()
        print("Duration:", (end - start)/1000)
        if(len(buffer) != 0):
            print("Recieved: ")
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
    ser.close()             # close port

main()