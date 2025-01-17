#include <stdio.h>
#include <string.h> // For memcpy
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "mcp2515.h"

// We are going to use SPI 0, and allocate it to the following GPIO pins
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

#include "blink.pio.h"

// !!! MUST MATCH STM STRUCT
typedef struct __attribute__((packed)){
	int32_t  tRpm;
	int32_t  tGear;
	int32_t  tSpeedKmh;
	int32_t  tHasDRS;
	int32_t  tDrs;
	int32_t  tPitLim;
	int32_t  tFuel;
	int32_t  tBrakeBias;
} telemetry_packet;
       
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}

// MCP2515 object
struct mcp2515_can mcp;

// Initialize MCP2515
void mcp2515_init() {
    spi_init(spi0, 1000000);  // 1 MHz SPI clock
    gpio_set_function(18, GPIO_FUNC_SPI);  // SCK
    gpio_set_function(19, GPIO_FUNC_SPI);  // MOSI
    gpio_set_function(16, GPIO_FUNC_SPI);  // MISO
    gpio_set_function(5, GPIO_OUTPUT);    // CS
    gpio_put(5, 1);  // Deselect by default

    // Initialize MCP2515
    if (mcp2515_init(&mcp, spi0, 5, MCP_8MHZ, CAN_500KBPS, MCP_MODE_NORMAL) != MCP2515_OK) {
        printf("MCP2515 initialization failed\n");
        while (1);
    }
    printf("MCP2515 initialized successfully\n");
}

void receive_can_message() {
    uint8_t len;
    uint8_t buf[8];
    uint32_t id;

    if (mcp2515_read_message(&mcp, &id, &len, buf) == MCP2515_OK) {
        printf("Received CAN ID: 0x%03X, Length: %d\n", id, len);

        // Process the frame data
        for (int i = 0; i < len; i++) {
            printf("Data[%d]: 0x%02X\n", i, buf[i]);
        }
    } else {
        printf("Failed to read CAN message\n");
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
    }
}
