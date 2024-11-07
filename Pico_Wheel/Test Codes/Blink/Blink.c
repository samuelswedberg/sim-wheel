#include "pico/stdlib.h"

int main() {
    // Define the GPIO pin for the external red LED (GPIO 34)
    const uint LED_PIN14 = 10;
    const uint LED_PIN15 = 11;
    const uint LED_PIN16 = 12;
    const uint LED_PIN17 = 13;
    const uint Board_LED = 25;
    
    // Initialize the GPIO for the LED
    gpio_init(LED_PIN14);
    gpio_set_dir(LED_PIN14, GPIO_OUT);
    gpio_init(LED_PIN15);
    gpio_set_dir(LED_PIN15, GPIO_OUT);
    gpio_init(LED_PIN16);
    gpio_set_dir(LED_PIN16, GPIO_OUT);
    gpio_init(LED_PIN17);
    gpio_set_dir(LED_PIN17, GPIO_OUT);
    gpio_init(Board_LED);
    gpio_set_dir(Board_LED, GPIO_OUT);

    // Main loop
    while (true) {
        gpio_put(LED_PIN16, 0); 
        gpio_put(LED_PIN14, 1);  
        sleep_ms(500);
        gpio_put(LED_PIN17,0);        
        gpio_put(LED_PIN15, 1);  
        sleep_ms(500);        
        gpio_put(LED_PIN14, 0);
        gpio_put(LED_PIN16, 1);
        sleep_ms(500);
        gpio_put(LED_PIN17, 1); 
        gpio_put(LED_PIN15, 0);
        sleep_ms(500);
               
    }
}
