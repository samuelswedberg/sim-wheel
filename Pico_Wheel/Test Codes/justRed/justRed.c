#include "pico/stdlib.h"

// Define the GPIO pin where NeoPixel data line is connected
#define NEOPIXEL_PIN 17

// Function to send a bit to the NeoPixel with precise timing
void send_bit(bool bit) {
    if (bit) {
        gpio_put(NEOPIXEL_PIN, 1);
        asm volatile("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"); // ~0.8 microseconds HIGH
        gpio_put(NEOPIXEL_PIN, 0);
        asm volatile("nop; nop; nop; nop; nop; nop;"); // ~0.45 microseconds LOW
    } else {
        gpio_put(NEOPIXEL_PIN, 1);
        asm volatile("nop; nop; nop; nop; nop;"); // ~0.4 microseconds HIGH
        gpio_put(NEOPIXEL_PIN, 0);
        asm volatile("nop; nop; nop; nop; nop; nop; nop; nop; nop;"); // ~0.85 microseconds LOW
    }
}

// Function to send a byte (8 bits) to the NeoPixel
void send_byte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {   // Send bits from MSB to LSB
        send_bit(byte & (1 << i));    // Extract each bit
    }
}

// Function to send RGB data to a NeoPixel in GRB format
void send_pixel(uint8_t red, uint8_t green, uint8_t blue) {
    send_byte(green); // Send green byte first
    send_byte(red);   // Then red byte
    send_byte(blue);  // Finally blue byte
}

int main() {
    // Initialize GPIO
    gpio_init(NEOPIXEL_PIN);
    gpio_set_dir(NEOPIXEL_PIN, GPIO_OUT);

    // Set the NeoPixel to red
    send_pixel(255, 0, 0); // Full red, no green, no blue

    // Latch the signal by holding low for a bit to display the color
    asm volatile("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;"); // ~80 microseconds LOW

    // Keep the color on by preventing the program from ending
    while (true) {
        tight_loop_contents(); // Infinite loop to keep the color on
    }

    return 0;
}
