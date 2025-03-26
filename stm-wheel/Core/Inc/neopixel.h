#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include <stdint.h>

#define NUM_LEDS 8  // or 16
#define LED_PIN GPIO_PIN_8
#define LED_PORT GPIOA

typedef struct {
    uint8_t g;
    uint8_t r;
    uint8_t b;
} led_t;

void neopixel_init(void);
void neopixel_set(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void neopixel_show(void);
void neopixel_clear(void);

#endif
