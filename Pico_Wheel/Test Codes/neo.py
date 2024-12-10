from machine import Pin, bitstream
from time import sleep

timing = [300, 900, 700, 500]
np = Pin(12, Pin.OUT)

red = bytearray([0,10,0])
green = bytearray([10,0,0])
blue = bytearray([0,0,10])

print(red)

while(1):
    bitstream(np, 0, timing, red)
    sleep(1)
    bitstream(np, 0, timing, green)
    sleep(1)
    bitstream(np, 0, timing, blue)
    sleep(1)
    
bytearray(b'\x00\x14\x00')