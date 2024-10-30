#include <stdio.h>
#include <string.h> // For memcpy
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"

// We are going to use SPI 0, and allocate it to the following GPIO pins
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

#include "blink.pio.h"

typedef struct __attribute__((packed)){
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
       
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}
// Function to interpret the SPI data buffer into the telemetry_packet struct
void interpret_data(uint8_t *buffer, telemetry_packet *telemetry_data) {
    // Interpret each 4-byte segment in little-endian format
    telemetry_data->tRpm = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
    telemetry_data->tGear = buffer[4] | (buffer[5] << 8) | (buffer[6] << 16) | (buffer[7] << 24);
    telemetry_data->tSpeedKmh = buffer[8] | (buffer[9] << 8) | (buffer[10] << 16) | (buffer[11] << 24);
    telemetry_data->tHasDRS = buffer[12] | (buffer[13] << 8) | (buffer[14] << 16) | (buffer[15] << 24);
    telemetry_data->tDrs = buffer[16] | (buffer[17] << 8) | (buffer[18] << 16) | (buffer[19] << 24);
    telemetry_data->tPitLim = buffer[20] | (buffer[21] << 8) | (buffer[22] << 16) | (buffer[23] << 24);
    telemetry_data->tFuel = buffer[24] | (buffer[25] << 8) | (buffer[26] << 16) | (buffer[27] << 24);
    telemetry_data->tBrakeBias = buffer[28] | (buffer[29] << 8) | (buffer[30] << 16) | (buffer[31] << 24);
    telemetry_data->tForceFB = buffer[32] | (buffer[33] << 8) | (buffer[34] << 16) | (buffer[35] << 24);
}

void setup_spi() {
    spi_init(SPI_PORT, 1 * 656250);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    spi_set_slave(SPI_PORT, true);
    spi_set_format(SPI_PORT, 
                8,                // Data size (8 bits)
                SPI_CPOL_1,       // Clock polarity low
                SPI_CPHA_1,       // Clock phase first edge
                SPI_MSB_FIRST);   // MSB first
}

void receive_spi_data(telemetry_packet *telemetry_data) {
    if(spi_is_readable (SPI_PORT))
    {
        printf("Receiving data...\n");
        uint8_t buffer[sizeof(telemetry_packet)];  // Buffer to store received data

        memset(buffer, 0, sizeof(buffer));

        // Wait for data from the master
        spi_read_blocking(spi0, 0, buffer, sizeof(buffer));  // Read 36 bytes into buffer

        // Debug: Print each byte received
        // printf("Received data: ");
        // for (int i = 0; i < sizeof(telemetry_packet); i++) {
        //     printf("0x%02X ", buffer[i]);
        // }
        // printf("\n");
        
        // Interpret the data into the telemetry_packet struct
        interpret_data(buffer, telemetry_data);
    }
}

int main()
{
    stdio_init_all();
    sleep_ms (2 * 1000);
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

    telemetry_packet telemetry_data;
    telemetry_data.tRpm = 0;
    telemetry_data.tGear = 0;
    telemetry_data.tSpeedKmh = 0;
    telemetry_data.tHasDRS = 0;
    telemetry_data.tDrs = 0;
    telemetry_data.tPitLim = 0;
    telemetry_data.tFuel = 0;
    telemetry_data.tBrakeBias = 0;
    telemetry_data.tForceFB = 0;
    while (1) {
        receive_spi_data(&telemetry_data);

        //Print the interpreted values
        printf("tRpm: %d\n", telemetry_data.tRpm);
        printf("tGear: %d\n", telemetry_data.tGear);
        printf("tSpeedKmh: %d\n", telemetry_data.tSpeedKmh);
        printf("tHasDRS: %d\n", telemetry_data.tHasDRS);
        printf("tDrs: %d\n", telemetry_data.tDrs);
        printf("tPitLim: %d\n", telemetry_data.tPitLim);
        printf("tFuel: %d\n", telemetry_data.tFuel);
        printf("tBrakeBias: %d\n", telemetry_data.tBrakeBias);
        printf("tForceFB: %d\n", telemetry_data.tForceFB);
    }
}
