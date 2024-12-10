#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Define button GPIO pins
#define BUTTON_PIN_1 2
#define BUTTON_PIN_2 3
#define BUTTON_PIN_3 15
#define BUTTON_PIN_4 14

// Define NeoPixel GPIO pin
#define NEOPIXEL_PIN 12
#define NUM_PIXELS 8

// Define timing constants for WS2812 (in nanoseconds)
#define T0H 300
#define T0L 900
#define T1H 700
#define T1L 600

#define NS_TO_CYCLES(ns) ((ns) / 62.5)  // Assuming 16 MHz clock (~62.5ns per cycle)

static inline void delay_cycles(uint32_t cycles) {
    for (volatile uint32_t i = 0; i < cycles; i++) {
        asm volatile("nop");
    }
}

// Send a single bit using bit-banging
void send_bit(uint gpio, bool bit) {
    if (bit) {
        gpio_put(gpio, 1);
        delay_cycles(NS_TO_CYCLES(T1H));
        gpio_put(gpio, 0);
        delay_cycles(NS_TO_CYCLES(T1L));
    } else {
        gpio_put(gpio, 1);
        delay_cycles(NS_TO_CYCLES(T0H));
        gpio_put(gpio, 0);
        delay_cycles(NS_TO_CYCLES(T0L));
    }
}

// Send a single byte (8 bits) to the NeoPixel
void send_byte(uint gpio, uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
        send_bit(gpio, byte & (1 << i));
    }
}

// Send RGB data for one pixel
void send_color(uint gpio, uint8_t red, uint8_t green, uint8_t blue) {
    send_byte(gpio, green);
    send_byte(gpio, red);
    send_byte(gpio, blue);
}

// Send RGB data for all pixels in the strip
void send_pixels(uint gpio, uint8_t pixels[][3]) {
    for (int i = 0; i < NUM_PIXELS; i++) {
        send_color(gpio, pixels[i][0], pixels[i][1], pixels[i][2]);
    }
}

int main() {
    stdio_init_all();

    uint8_t pixels[NUM_PIXELS][3] = {0};
    uint8_t red[] = {20, 0, 0};
    uint8_t green[] = {0, 20, 0};
    uint8_t blue[] = {0, 0, 20};
    uint8_t off[] = {0, 0, 0};

    // Initialize buttons
    gpio_init(BUTTON_PIN_1);
    gpio_init(BUTTON_PIN_2);
    gpio_init(BUTTON_PIN_3);
    gpio_init(BUTTON_PIN_4);
    gpio_set_dir(BUTTON_PIN_1, GPIO_IN);
    gpio_set_dir(BUTTON_PIN_2, GPIO_IN);
    gpio_set_dir(BUTTON_PIN_3, GPIO_IN);
    gpio_set_dir(BUTTON_PIN_4, GPIO_IN);
    gpio_pull_up(BUTTON_PIN_1);
    gpio_pull_up(BUTTON_PIN_2);
    gpio_pull_up(BUTTON_PIN_3);
    gpio_pull_up(BUTTON_PIN_4);

    // Initialize NeoPixel GPIO
    gpio_init(NEOPIXEL_PIN);
    gpio_set_dir(NEOPIXEL_PIN, GPIO_OUT);
    gpio_put(NEOPIXEL_PIN, 0);

    absolute_time_t last_check_time = get_absolute_time();
    const uint32_t check_interval_us = 1000;

    printf("Starting Button and NeoPixel Control\n");

    while (true) {
        if (absolute_time_diff_us(last_check_time, get_absolute_time()) >= check_interval_us) {
            last_check_time = get_absolute_time();

            // Read button states
            bool button_1_pressed = !gpio_get(BUTTON_PIN_1);
            bool button_2_pressed = !gpio_get(BUTTON_PIN_2);
            bool button_3_pressed = !gpio_get(BUTTON_PIN_3);
            bool button_4_pressed = !gpio_get(BUTTON_PIN_4);

            // Debug output for button states
            printf("Button 1: %s, Button 2: %s, Button 3: %s, Button 4: %s\n",
                   button_1_pressed ? "Pressed" : "Released",
                   button_2_pressed ? "Pressed" : "Released",
                   button_3_pressed ? "Pressed" : "Released",
                   button_4_pressed ? "Pressed" : "Released");

            // Control NeoPixels based on button states
            if (button_1_pressed) {
                printf("Setting NeoPixels to Red\n");
                for (int i = 0; i < NUM_PIXELS; i++) {
                    memcpy(pixels[i], red, sizeof(red));
                }
            } else if (button_2_pressed) {
                printf("Setting NeoPixels to Green\n");
                for (int i = 0; i < NUM_PIXELS; i++) {
                    memcpy(pixels[i], green, sizeof(green));
                }
            } else if (button_3_pressed) {
                printf("Setting NeoPixels to Blue\n");
                for (int i = 0; i < NUM_PIXELS; i++) {
                    memcpy(pixels[i], blue, sizeof(blue));
                }
            } else if (button_4_pressed) {
                printf("Turning NeoPixels Off\n");
                for (int i = 0; i < NUM_PIXELS; i++) {
                    memcpy(pixels[i], off, sizeof(off));
                }
            }

            send_pixels(NEOPIXEL_PIN, pixels);
        }
    }

    return 0;
}
