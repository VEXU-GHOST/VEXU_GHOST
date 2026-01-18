from machine import Pin
import time

led = Pin('PA5', Pin.OUT)

while True:
    led.toggle()
    time.sleep(0.5)
