#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"

#define LED_PIN 16         // GPIO pin for WS2812 data
#define ONBOARD_LED_PIN 25 // GPIO pin for the onboard LED
#define NUM_PIXELS 16      // Number of WS2812 LEDs in your strip

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} Color;

Color pixels[NUM_PIXELS];

void ws2812_program_init(PIO pio, uint sm, uint offset, uint pin) {
    pio_gpio_init(pio, pin); // Initialize the GPIO pin
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true); // Set pin direction to output
    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin); // Set the sideset pin
    sm_config_set_out_shift(&c, false, true, 32); // Shift out bits, MSB first
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX); // Combine TX FIFO with shift register
    pio_sm_init(pio, sm, offset, &c); // Initialize the state machine
    pio_sm_set_enabled(pio, sm, true); // Enable the state machine
}

void show_pixels(PIO pio, uint sm) {
    for (int i = 0; i < NUM_PIXELS; i++) {
        // WS2812 expects data in GRB format
        uint32_t grb = ((uint32_t)pixels[i].green << 16) |
                       ((uint32_t)pixels[i].red << 8) |
                       pixels[i].blue;
        pio_sm_put_blocking(pio, sm, grb << 8u); // Send data to the PIO state machine
    }
    sleep_ms(1); // Ensure a reset signal by waiting for more than 50µs
}

int main() {
    stdio_init_all();

    // Initialize the onboard LED
    gpio_init(ONBOARD_LED_PIN);
    gpio_set_dir(ONBOARD_LED_PIN, GPIO_OUT);

    // Initialize the WS2812 PIO program
    PIO pio = pio0;
    uint sm = 0; // State machine index
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, LED_PIN);

    while (true) {
        // Blink the onboard LED to confirm the program is running
        gpio_put(ONBOARD_LED_PIN, 1); // Turn on
        sleep_ms(250);
        gpio_put(ONBOARD_LED_PIN, 0); // Turn off
        sleep_ms(250);

        // Set individual colors for each pixel
        for (int i = 0; i < NUM_PIXELS; i++) {
            if (i < 8) {
                pixels[i] = (Color){10, 0, 0}; // First 8 LEDs - Red
            } else {
                pixels[i] = (Color){0, 10, 0}; // Next 8 LEDs - Green
            }
        }

        // Display the colors
        show_pixels(pio, sm);
        sleep_ms(500);

        // Example animation: Rotate colors
        Color temp = pixels[NUM_PIXELS - 1];
        for (int i = NUM_PIXELS - 1; i > 0; i--) {
            pixels[i] = pixels[i - 1];
        }
        pixels[0] = temp;

        // Display the updated colors
        show_pixels(pio, sm);
        sleep_ms(500);
    }

    return 0;
}
