#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  2
#define PIN_MOSI 3

// Define the number of integers and the size of the float
#define NUM_INTS 16 // Adjust this based on your needs
#define FLOAT_SIZE sizeof(float)
#define DATA_SIZE (NUM_INTS * sizeof(int) + FLOAT_SIZE)

#include "blink.pio.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}



int main()
{
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    // Set the SPI mode to 1 (CPOL = 0, CPHA = 1)
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);
    
    #ifdef PICO_DEFAULT_LED_PIN
    blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
    #else
    blink_pin_forever(pio, 0, offset, 6, 3);
    #endif
    // For more pio examples see https://github.com/raspberrypi/pico-examples/tree/master/pio

    uint8_t dataReceived[DATA_SIZE];
    while (true) {
                // Wait for CS to go low (indicating a transmission is starting)
        while (gpio_get(PIN_CS)) {
            // Wait until CS is low
        }
        // Read the data into the buffer
        spi_read_blocking(SPI_PORT, 0, dataReceived, DATA_SIZE);

        // Process the received data
        int32_t integers[NUM_INTS];
        float receivedFloat;

        // Unpack the received data into integers and float
        for (int i = 0; i < NUM_INTS; i++) {
            integers[i] = *((int32_t*)&dataReceived[i * sizeof(int)]);
        }
        receivedFloat = *((float*)&dataReceived[NUM_INTS * sizeof(int)]);

        // Print the received integers and float
        printf("Received integers: ");
        for (int i = 0; i < NUM_INTS; i++) {
            printf("%d ", integers[i]);
        }
        printf("\nReceived float: %f\n", receivedFloat);

        // Wait until CS is high again (end of transmission)
        while (!gpio_get(PIN_CS)) {
            // Wait until CS is high
        }
    }
}
