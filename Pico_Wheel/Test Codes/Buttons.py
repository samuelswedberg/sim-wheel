#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// Define SPI and button configuration
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19
#define BUTTON_PIN 2

// Struct to hold button press state
typedef struct {
    uint32_t timestamp;
    bool button_pressed;
} ButtonState;

int main() {
    // Initialize GPIO and SPI
    stdio_init_all();
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN); // Pull-up resistor for button

    spi_init(SPI_PORT, 1000000); // Initialize SPI at 1 MHz
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Enable SPI slave mode
    gpio_set_dir(PIN_CS, GPIO_IN);

    // Variables for button state and timing
    ButtonState button_state;
    button_state.timestamp = 0;
    button_state.button_pressed = false;

    absolute_time_t last_check_time = get_absolute_time();
    const uint32_t check_interval_us = 1000; // 1 ms

    uint8_t buffer[sizeof(ButtonState)];

    while (true) {
        // Check button state every 1 ms
        if (absolute_time_diff_us(last_check_time, get_absolute_time()) >= check_interval_us) {
            last_check_time = get_absolute_time();
            button_state.timestamp++;
            button_state.button_pressed = !gpio_get(BUTTON_PIN); // Button pressed = LOW
        }

        // Wait for master request
        if (!gpio_get(PIN_CS)) { // Active low CS
            // Send button state as a struct
            memcpy(buffer, &button_state, sizeof(ButtonState));
            spi_write_blocking(SPI_PORT, buffer, sizeof(ButtonState));
        }
    }
    return 0;
}
