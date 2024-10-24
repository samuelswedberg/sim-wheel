#include <stdio.h>
#include <string.h> // For memcpy
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16 // RX
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19 // TX

#include "blink.pio.h"

typedef struct __attribute__((__packed__)) {
	int32_t  tRpm;
	int32_t  tGear;
	int32_t  tSpeedKmh;
	int32_t  tHasDRS;
	int32_t  tDrs;
	int32_t  tPitLim;
	int32_t  tFuel;
	int32_t  tBrakeBias;
	int32_t tForceFB;
} telemetry_packet;

telemetry_packet telemetry_data;
       
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}

void setup_spi() {
    spi_init(SPI_PORT, 328125);

    // Set the SPI format to match the STM32
    spi_set_format(SPI_PORT, 
                8,                // Data size (8 bits)
                SPI_CPOL_0,       // Clock polarity low
                SPI_CPHA_0,       // Clock phase first edge
                SPI_MSB_FIRST);   // MSB first

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
}

void setup() {
    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);
    
    #ifdef PICO_DEFAULT_LED_PIN
    blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
    #else
    blink_pin_forever(pio, 0, offset, 6, 3);
    #endif

    telemetry_data.tRpm = 0;
    telemetry_data.tGear = 0;
    telemetry_data.tSpeedKmh = 0;
    telemetry_data.tHasDRS = 0;
    telemetry_data.tDrs = 0;
    telemetry_data.tPitLim = 0;
    telemetry_data.tFuel = 0;
    telemetry_data.tBrakeBias = 0;
    telemetry_data.tForceFB = 0;
    memset(&telemetry_data, 0, sizeof(telemetry_packet)); // Zero-initialize
}

bool loop() {
    bool flag = false;
    // Create a buffer to hold the received data
    uint8_t buffer[sizeof(telemetry_packet)];

    // Assuming SPI0 is used; adjust if necessary
    spi_inst_t *spi = spi0;
    uint8_t cs_pin = 17; // Adjust CS pin as needed

    while (gpio_get(PIN_CS) == 1) {
        // Do nothing, wait for CS to go low
    }

    // Read the data from SPI
    spi_read_blocking(spi, 0, (uint8_t*)&buffer, sizeof(buffer));

    // Check if the size of the received data matches the struct size
    if (sizeof(buffer) == sizeof(telemetry_packet)) {
        // Copy the data from the buffer to the telemetry_data struct
        memcpy(&telemetry_data, buffer, sizeof(telemetry_packet));
        // Convert received data to byte array for logging
        uint8_t* rawData = (uint8_t*)&buffer;

        // Print raw data
        for (int i = 0; i < sizeof(telemetry_packet); i++) {
            printf("Byte %d: 0x%02X\n", i, rawData[i]);
        }
        flag = true;
    } else {
        // Handle the size mismatch (e.g., log an error, set defaults, etc.)
        printf("Received data size mismatch. Expected: %lu, Received: %lu\n",
               sizeof(telemetry_packet), sizeof(buffer));
        flag = false;
    }

    // Wait for CS to go high again (indicating end of SPI transaction)
    while (gpio_get(PIN_CS) == 0) {
        // Do nothing, wait for CS to go high
    }
    return flag;
}

int main()
{
    stdio_init_all();
    setup();  


    setup_spi();
    while (true) {
        if(loop()){
            printf("tGear: %d, tRpm: %d, tForceFB: %d, tFuel: %d\n", telemetry_data.tGear, telemetry_data.tRpm, telemetry_data.tForceFB, telemetry_data.tFuel);
        }
        sleep_ms(1000); // Adjust the delay as needed
    }
}
