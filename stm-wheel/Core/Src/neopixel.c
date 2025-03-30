#include "neopixel.h"
#include "main.h"

#define NUM_LEDS 8 // MATCH IN main.c
#define TIMER_FREQ 800000   // 800kHz NeoPixel bitrate (1.25 µs per bit)
#define BITS_PER_LED 24
#define RESET_SLOTS 48

#define HIGH_DUTY 64
#define LOW_DUTY 26

#define BUFFER_SIZE ((NUM_LEDS * BITS_PER_LED) + RESET_SLOTS)

static uint16_t pwm_buffer[BUFFER_SIZE];
static led_t leds[NUM_LEDS];

extern TIM_HandleTypeDef htim1; // You need to enable TIM1 in CubeMX
extern DMA_HandleTypeDef hdma_tim1_ch1;

void neopixel_set(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= NUM_LEDS) return;
    leds[index].r = r;
    leds[index].g = g;
    leds[index].b = b;
}

void neopixel_clear(void) {
    for (int i = 0; i < NUM_LEDS; i++) {
        neopixel_set(i, 0, 0, 0);
    }
}

static void encode_byte(uint8_t byte, uint16_t *buf) {
    for (int i = 0; i < 8; i++) {
        buf[i] = (byte & (1 << (7 - i))) ? HIGH_DUTY : LOW_DUTY;
    }
}

void neopixel_show(void) {
    uint16_t i = 0;
    for (uint16_t led = 0; led < NUM_LEDS; led++) {
        encode_byte(leds[led].g, &pwm_buffer[i]); i += 8;
        encode_byte(leds[led].r, &pwm_buffer[i]); i += 8;
        encode_byte(leds[led].b, &pwm_buffer[i]); i += 8;
    }

    // Clear reset slot
    for (; i < BUFFER_SIZE; i++) {
        pwm_buffer[i] = 0;
    }

    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)pwm_buffer, BUFFER_SIZE);
    while (HAL_DMA_GetState(&hdma_tim1_ch1) != HAL_DMA_STATE_READY); // Wait for complete
    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
}
