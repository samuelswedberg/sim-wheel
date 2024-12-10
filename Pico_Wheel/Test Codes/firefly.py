from picozero import pico_led, LED
from time import sleep

pico_led.on()
sleep(1)
pico_led.off()
firefly = LED(13)


for x in range(1, 100):
    firefly.on()  # Turn on the LED
    sleep(0.5)    # Wait for 0.5 seconds
    firefly.off() # Turn off the LED
    sleep(2.5)    # Wait for 2.5 seconds