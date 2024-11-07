#include <stdio.h>
#include "pico/stdlib.h"

#define LED_PIN 16       // GPIO pin for NeoPixel data
#define NUM_PIXELS 8     // Number of NeoPixels in your strip
#define T0H  350         // Width of a 0 bit's "high" time in nanoseconds
#define T1H  900         // Width of a 1 bit's "high" time in nanoseconds
#define TOTAL_PERIOD 1250 // Total bit period (including high and low) in nanoseconds

void send_bit(uint bit) {
    if (bit) {
        gpio_put(LED_PIN, 1);
        sleep_us(T1H / 1000); // Send 1 bit: longer high, shorter low
        gpio_put(LED_PIN, 0);
        sleep_us((TOTAL_PERIOD - T1H) / 1000);
    } else {
        gpio_put(LED_PIN, 1);
        sleep_us(T0H / 1000); // Send 0 bit: shorter high, longer low
        gpio_put(LED_PIN, 0);
        sleep_us((TOTAL_PERIOD - T0H) / 1000);
    }
}

void send_byte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
        send_bit((byte >> i) & 1);
    }
}

void send_color(uint8_t red, uint8_t green, uint8_t blue) {
    send_byte(green);  // Send green first per WS2812 protocol
    send_byte(red);
    send_byte(blue);
}

void show_color(uint8_t red, uint8_t green, uint8_t blue) {
    for (int i = 0; i < NUM_PIXELS; i++) {
        send_color(red, green, blue);
    }
    sleep_ms(1);  // Ensure a reset signal by waiting for more than 50µs
}

int main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true) {
        show_color(255, 0, 0);  // Red
        sleep_ms(500);
        show_color(0, 255, 0);  // Green
        sleep_ms(500);
        show_color(0, 0, 255);  // Blue
        sleep_ms(500);
    }

    return 0;
}

