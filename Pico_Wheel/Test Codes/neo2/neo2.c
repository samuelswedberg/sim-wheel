#include <stdio.h>
#include "pico/stdlib.h"

// Define the GPIO pin for NeoPixel data
#define LED_PIN 12
#define NUM_PIXELS 16  // Total number of NeoPixels
#define STEPS 20
#define Max_Value 50

// Define timing constants for WS2812 (in nanoseconds)
#define T0H 300  // T0H: 300ns high for 0
#define T0L 900  // T0L: 900ns low for 0
#define T1H 700  // T1H: 700ns high for 1
#define T1L 600  // T1L: 600ns low for 1

// Convert nanoseconds to microseconds for Pico SDK sleep functions
#define NS_TO_CYCLES(ns) ((ns) / 62.5)  // Assuming 16 MHz clock (~62.5ns per cycle)

// Delay function for precise timing
static inline void delay_cycles(uint32_t cycles) {
    for (volatile uint32_t i = 0; i < cycles; i++) {
        asm volatile("nop");
    }
}

// Send a single bit using bit-banging
void send_bit(uint gpio, bool bit) {
    if (bit) {
        // Send a 1
        gpio_put(gpio, 1);
        delay_cycles(NS_TO_CYCLES(T1H));
        gpio_put(gpio, 0);
        delay_cycles(NS_TO_CYCLES(T1L));
    } else {
        // Send a 0
        gpio_put(gpio, 1);
        delay_cycles(NS_TO_CYCLES(T0H));
        gpio_put(gpio, 0);
        delay_cycles(NS_TO_CYCLES(T0L));
    }
}

// Send a single byte (8 bits) to the NeoPixel
void send_byte(uint gpio, uint8_t byte) {
    for (int i = 7; i >= 0; i--) {  // Send bits MSB first
        send_bit(gpio, byte & (1 << i));
    }
}

// Send RGB data for one pixel
void send_color(uint gpio, uint8_t red, uint8_t green, uint8_t blue) {
    // WS2812 expects data in GRB order
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
    // Initialize the GPIO pin
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Define the colors for each pixel (17 pixels, each with R, G, B values)
    uint8_t red[3] = {10, 0, 0};   // Red
    uint8_t green[3] = {0, 10, 0}; // Green
    uint8_t blue[3] = {0, 0, 10};  // Blue
    
    uint8_t pixels[NUM_PIXELS][3]; // Array to hold pixel data

    while (1) {
        /*// Set all pixels to red
        for (int i = 0; i < NUM_PIXELS; i++) {
            pixels[i][0] = red[0];  // Red
            pixels[i][1] = red[1];  // Green
            pixels[i][2] = red[2];  // Blue
        }
        send_pixels(LED_PIN, pixels);
        sleep_ms(1000);

        // Set all pixels to green
        for (int i = 0; i < NUM_PIXELS; i++) {
            pixels[i][0] = green[0];  // Red
            pixels[i][1] = green[1];  // Green
            pixels[i][2] = green[2];  // Blue
        }
        send_pixels(LED_PIN, pixels);
        sleep_ms(250);

        // Set all pixels to blue
        for (int i = 0; i < NUM_PIXELS; i++) {
            pixels[i][0] = blue[0];  // Red
            pixels[i][1] = blue[1];  // Green
            pixels[i][2] = blue[2];  // Blue
        }
        send_pixels(LED_PIN, pixels);
        sleep_ms(250);*/
    uint R = 20;
    uint G = 20;
    uint B = 20;

    //RED TO GREEN
    //10:20 and 20:10
    //Yellow
    //GREEN TO BLUE
    //10:20 and 20:10
    //BLUE TO RED
    //10:20 and 20:10
    //Purple 10 RED 10 BLUE
    for(uint cycle =0; cycle < 6; cycle++){
        if(cycle==0){
            R = 20;
            G = 0;
            B = 0;
            uint8_t rainbow[3] = {R,G,B};
            for (int i = 0; i < NUM_PIXELS; i++) {
                pixels[i][0] = rainbow[0];  // Red
                pixels[i][1] = rainbow[1];  // Green
                pixels[i][2] = rainbow[2];  // Blue
            }
            send_pixels(LED_PIN, pixels);
            sleep_ms(250);
        }
        else if(cycle == 1){
    //RED TO GREEN
        for(G = 0;G<21; G++){
        R = R - G;
        B=0;
                uint8_t rainbow[3] = {R,G,B};
                for (int i = 0; i < NUM_PIXELS; i++) {
                    pixels[i][0] = rainbow[0];  // Red
                    pixels[i][1] = rainbow[1];  // Green
                    pixels[i][2] = rainbow[2];  // Blue
                }
                send_pixels(LED_PIN, pixels);
                sleep_ms(250);
                }
            }
        else if(cycle==2){
            R = 0;
            G = 20;
            B = 0;
            uint8_t rainbow[3] = {R,G,B};
            for (int i = 0; i < NUM_PIXELS; i++) {
                pixels[i][0] = rainbow[0];  // Red
                pixels[i][1] = rainbow[1];  // Green
                pixels[i][2] = rainbow[2];  // Blue
            }
            send_pixels(LED_PIN, pixels);
            sleep_ms(250);
        }
        else if(cycle == 3){
    //Green to Blue
        for(B = 0; B<21; B++){
        G = G - B;
        R=0;
        
                uint8_t rainbow[3] = {R,G,B};
                for (int i = 0; i < NUM_PIXELS; i++) {
                    pixels[i][0] = rainbow[0];  // Red
                    pixels[i][1] = rainbow[1];  // Green
                    pixels[i][2] = rainbow[2];  // Blue
                }
                send_pixels(LED_PIN, pixels);
                sleep_ms(250);
                }
            }
        else if(cycle==4){
            R = 0;
            G = 0;
            B = 20;
            uint8_t rainbow[3] = {R,G,B};
            for (int i = 0; i < NUM_PIXELS; i++) {
                pixels[i][0] = rainbow[0];  // Red
                pixels[i][1] = rainbow[1];  // Green
                pixels[i][2] = rainbow[2];  // Blue
            }
            send_pixels(LED_PIN, pixels);
            sleep_ms(250);
            }
        else if(cycle == 5)
        for(R = 0; R<21; R++){
        B = B - R;
        R=0;
            uint8_t rainbow[3] = {R,G,B};
            for (int i = 0; i < NUM_PIXELS; i++) {
                pixels[i][0] = rainbow[0];  // Red
                pixels[i][1] = rainbow[1];  // Green
                pixels[i][2] = rainbow[2];  // Blue
            }
            send_pixels(LED_PIN, pixels);
            sleep_ms(250);
            }
        }

    }
    return 0;
}
