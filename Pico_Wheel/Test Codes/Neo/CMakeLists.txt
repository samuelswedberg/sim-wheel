# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)   # Import the Pico SDK

# Project Name
project(neo)

# Initialize the Pico SDK
pico_sdk_init()

# Add the main source file for the project
add_executable(neo
    neo.c        # Your main C file
    ws2812.pio
)

# Add the PIO file (for controlling NeoPixels) and generate the corresponding .h file
pico_generate_pio_header(neo ${CMAKE_CURRENT_LIST_DIR}/neopixel.pio)
pico_generate_pio_header(neo ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)



# Link necessary libraries
target_link_libraries(neo pico_stdlib hardware_pio)



# Enable USB output for debugging via USB, if needed
pico_enable_stdio_usb(neo 1)

# Enable UART output (set to 0 if not using UART for debugging)
pico_enable_stdio_uart(neo 0)

# Create binary, hex, and uf2 files
pico_add_extra_outputs(neo)
