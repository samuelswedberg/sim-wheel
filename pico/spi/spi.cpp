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
    spi_init(SPI_PORT, 656250);

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
    setup_spi();

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
    telemetry_data.tRpm = 0;
    telemetry_data.tSpeedKmh = 0;
    telemetry_data.tHasDRS = 0;
    telemetry_data.tDrs = 0;
    telemetry_data.tPitLim = 0;
    telemetry_data.tFuel = 0;
    telemetry_data.tBrakeBias = 0;
    telemetry_data.tForceFB = 0;
    memset(&telemetry_data, 0, sizeof(telemetry_packet)); // Zero-initialize
}

// Function to unpack telemetry data from buffer
void unpackTelemetryPacket(uint8_t *buffer, telemetry_packet *packet) {
    // Ensure the buffer is large enough and packet is valid
    if (buffer == NULL || packet == NULL) return; // Add appropriate size checks if needed

    size_t offset = 0;

    // Helper macro to unpack data in little-endian
    #define UNPACK_INT32(value) \
        memcpy(&value, buffer + offset, sizeof(value)); \
        offset += sizeof(value);
    
    #define UNPACK_FLOAT(value) \
        memcpy(&value, buffer + offset, sizeof(value)); \
        offset += sizeof(value);

    // Unpack the structure fields from the buffer
    UNPACK_INT32(packet->tRpm);
    UNPACK_INT32(packet->tGear);
    UNPACK_INT32(packet->tSpeedKmh);
    UNPACK_INT32(packet->tHasDRS);
    UNPACK_INT32(packet->tDrs);
    UNPACK_INT32(packet->tPitLim);
    UNPACK_INT32(packet->tFuel);
    UNPACK_INT32(packet->tBrakeBias);
    UNPACK_INT32(packet->tForceFB);
}

void loop() {
    uint8_t spi_rx_buffer[sizeof(telemetry_packet)];
    // Wait for CS to go low (indicating a transmission is starting)
    while (gpio_get(PIN_CS)) {
        // Wait until CS is low
    }
    // Read the data into the buffer
    spi_read_blocking(SPI_PORT, 0, spi_rx_buffer, sizeof(spi_rx_buffer));

    // Call function to unpack data into telemetry_data structure
    unpackTelemetryPacket(spi_rx_buffer, &telemetry_data);

    printf("tGear: %d, tRpm: %d, tForceFB: %.2f, tFuel: %d\n", telemetry_data.tGear, telemetry_data.tRpm, telemetry_data.tForceFB, telemetry_data.tFuel);

    // Wait until CS is high again (end of transmission)
    while (!gpio_get(PIN_CS)) {
        // Wait until CS is high
    }

    // Optionally add a small delay here
    sleep_ms(10);
}
int main()
{
    stdio_init_all();
    setup();  
    while (true) {
        loop();
    }
}
