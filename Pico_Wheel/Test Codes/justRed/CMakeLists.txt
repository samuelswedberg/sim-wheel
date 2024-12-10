# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location

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

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(justRed C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable
add_executable(justRed src/justRed.c)

pico_set_program_name(justRed "justRed")
pico_set_program_version(justRed "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(justRed 0)
pico_enable_stdio_usb(justRed 0)

# Add the standard library to the build
target_link_libraries(justRed
    hardware_pio
    pico_stdlib)

# Add the standard include files to the build
target_include_directories(justRed PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

# Add PIO header generation with consistent target name
pico_generate_pio_header(justRed ${CMAKE_CURRENT_LIST_DIR}/src/neopixel.pio)

# Add any user requested libraries (if any)
target_link_libraries(justRed)

# Generate additional outputs (UF2, binary, etc.)
pico_add_extra_outputs(justRed)
