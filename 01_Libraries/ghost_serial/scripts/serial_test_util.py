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

    ser = serial.Serial("/dev/ttyACM1", timeout=0.5)  # open serial port
    ser.flushInput()
    while run_:
        # time.sleep(1)
        # write_str = b"test"
        # print()
        # print("Writing: " + write_str.decode('ascii'))
        # ser.write(write_str)

        buffer = ser.read_until(b"\x00")
        if len(buffer) != 0:
            print(buffer)
            decoded_packet = cobs.decode(buffer[:-1])
            # print(decoded_packet[4:].decode("ascii"), end = '')
    ser.close()  # close port


main()
