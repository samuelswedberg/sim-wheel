import machine
import time

class NeoPixel:
    def __init__(self, pin, num_leds):
        # Initialize the pin and number of LEDs
        self.pin = machine.Pin(pin, machine.Pin.OUT)
        self.num_leds = num_leds
        self.led_data = [(0, 0, 0)] * num_leds  # All LEDs start off

    def send_byte(self, byte):
        # Send a byte to the NeoPixel strip
        for i in range(8):
            if byte & 0x80:  # If the highest bit is 1
                self.pin.value(1)
                time.sleep_us(int(0.7 * 1_000_000))  # High time for "1" bit (converted to microseconds)
                self.pin.value(0)
                time.sleep_us(int(0.6 * 1_000_000))  # Low time for "1" bit (converted to microseconds)
            else:
                self.pin.value(1)
                time.sleep_us(int(0.35 * 1_000_000))  # High time for "0" bit (converted to microseconds)
                self.pin.value(0)
                time.sleep_us(int(0.8 * 1_000_000))  # Low time for "0" bit (converted to microseconds)
            byte <<= 1  # Shift to the next bit

    def send_rgb(self, red, green, blue):
        # Send color data in GRB order (NeoPixels expect GRB, not RGB)
        for color in (green, red, blue):
            self.send_byte(color)

    def show(self):
        # Send data for each LED in the strip
        for red, green, blue in self.led_data:
            self.send_rgb(red, green, blue)
        time.sleep_us(50)  # Latch signal to update the LEDs

    def set_pixel(self, n, color):
        # Set the color of a specific LED
        if n < self.num_leds:
            self.led_data[n] = color

    def fill(self, color):
        # Set all LEDs to the same color
        for i in range(self.num_leds):
            self.set_pixel(i, color)

# === Main Code ===
# Set GPIO 17 as the data pin and control 16 LEDs
np = NeoPixel(17, 16)

# Fill all LEDs with red color and display it
np.fill((255, 0, 0))  # Red color
np.show()
