#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Define button GPIO pins
#define BUTTON_PIN_1 14
#define BUTTON_PIN_2 15
#define BUTTON_PIN_3 16
#define BUTTON_PIN_4 17
#define BUTTON_PIN_5 18
#define BUTTON_PIN_6 19
#define BUTTON_PIN_7 20
#define BUTTON_PIN_8 21

// Struct to hold button press state
typedef struct {
    uint32_t timestamp;
    bool button_1_pressed;
    bool button_2_pressed;
    bool button_3_pressed;
    bool button_4_pressed;
    bool button_5_pressed;
    bool button_6_pressed;
    bool button_7_pressed;
    bool button_8_pressed;
} ButtonState;

int main() {
    // Initialize GPIO and USB Serial
    stdio_init_all();
    gpio_init(BUTTON_PIN_1);
    gpio_init(BUTTON_PIN_2);
    gpio_init(BUTTON_PIN_3);
    gpio_init(BUTTON_PIN_4);
    gpio_init(BUTTON_PIN_5);
    gpio_init(BUTTON_PIN_6);
    gpio_init(BUTTON_PIN_7);
    gpio_init(BUTTON_PIN_8);
    gpio_set_dir(BUTTON_PIN_1, GPIO_IN);
    gpio_set_dir(BUTTON_PIN_2, GPIO_IN);
    gpio_set_dir(BUTTON_PIN_3, GPIO_IN);
    gpio_set_dir(BUTTON_PIN_4, GPIO_IN);
    gpio_set_dir(BUTTON_PIN_5, GPIO_IN);
    gpio_set_dir(BUTTON_PIN_6, GPIO_IN);
    gpio_set_dir(BUTTON_PIN_7, GPIO_IN);
    gpio_set_dir(BUTTON_PIN_8, GPIO_IN);
    gpio_pull_up(BUTTON_PIN_1); // Pull-up resistor for button 1
    gpio_pull_up(BUTTON_PIN_2); 
    gpio_pull_up(BUTTON_PIN_3); 
    gpio_pull_up(BUTTON_PIN_4); 
    gpio_pull_up(BUTTON_PIN_5); 
    gpio_pull_up(BUTTON_PIN_6); 
    gpio_pull_up(BUTTON_PIN_7); 
    gpio_pull_up(BUTTON_PIN_8); 

    // Variables for button state and timing
    ButtonState button_state;
    button_state.timestamp = 0;
    button_state.button_1_pressed = false;
    button_state.button_2_pressed = false;
    button_state.button_3_pressed = false;
    button_state.button_4_pressed = false;
    button_state.button_5_pressed = false;
    button_state.button_6_pressed = false;
    button_state.button_7_pressed = false;
    button_state.button_8_pressed = false;

        absolute_time_t last_check_time = get_absolute_time();
        const uint32_t check_interval_us = 1000; // 1 ms

    while (true) {

        
        // Check button state every 1 ms
        if (absolute_time_diff_us(last_check_time, get_absolute_time()) >= check_interval_us) {
            last_check_time = get_absolute_time();
            button_state.timestamp++;
            button_state.button_1_pressed = !gpio_get(BUTTON_PIN_1); // Button 1 pressed = LOW
            button_state.button_2_pressed = !gpio_get(BUTTON_PIN_2); // Button 2 pressed = LOW
            button_state.button_3_pressed = !gpio_get(BUTTON_PIN_3); // Button 1 pressed = LOW
            button_state.button_4_pressed = !gpio_get(BUTTON_PIN_4); // Button 2 pressed = LOW
            button_state.button_5_pressed = !gpio_get(BUTTON_PIN_5); // Button 1 pressed = LOW
            button_state.button_6_pressed = !gpio_get(BUTTON_PIN_6); // Button 2 pressed = LOW
            button_state.button_7_pressed = !gpio_get(BUTTON_PIN_7); // Button 1 pressed = LOW
            button_state.button_8_pressed = !gpio_get(BUTTON_PIN_8); // Button 2 pressed = LOW

            // Print button state to USB terminal
            printf("Timestamp: %u, Button 1: %s, Button 2: %s, Button 3: %s, Button 4: %s, Button 5: %s, Button 6: %s, Button 7: %s, Button 8: %s\n",
                   button_state.timestamp,
                   button_state.button_1_pressed ? "Pressed" : "Released",
                   button_state.button_2_pressed ? "Pressed" : "Released",
                   button_state.button_3_pressed ? "Pressed" : "Released",
                   button_state.button_4_pressed ? "Pressed" : "Released",
                   button_state.button_5_pressed ? "Pressed" : "Released",
                   button_state.button_6_pressed ? "Pressed" : "Released",
                   button_state.button_7_pressed ? "Pressed" : "Released",
                   button_state.button_8_pressed ? "Pressed" : "Released");
        }
    }
    return 0;
}
