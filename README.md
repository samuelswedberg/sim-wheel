# Sim Racing Wheel System

[![License](https://img.shields.io/badge/license-GPLv3-blue.svg)](LICENSE)

Welcome to the Sim Racing Wheel System repository — a fully embedded, custom-built force feedback steering wheel and pedal system designed for PC sim racing. This project was developed as part of my academic and personal engineering portfolio.

> 🔗 **Project Website:** [samuelswedberg.com](https://www.samuelswedberg.com/projects/simracingsystem.html)

---

## 📄 Documentation

I ported our OneNote document throughout the two semesters to a Markdown document in the docs folder. It isn't the most optimal solution but it has the information there of our development timeline.

## 🧠 About the Project

This system was designed and built from the ground up as part of a senior design capstone project at North Dakota State University. The goal was to create a modular sim racing system that interfaces seamlessly with PC games.

Key components include:

- **Wheelbase**: Powered by an STM32F4 MCU, controlling a DC motor with real-time force feedback.
- **Steering Wheel**: Button and encoder inputs, rotary switches, and telemetry display via CAN and UART.
- **Pedals**: Supports analog sensors with adjustable profiles and calibration.
- **Communication**: USB Composite Driver and CAN bus communication between all devices.
- **RTOS**: Uses FreeRTOS to handle motor control, peripheral processing, and CAN communication efficiently.
- **Display**: Real-time telemetry output using a Nextion touchscreen.

This was an extensive embedded systems project that included circuit design, firmware development, real-time data communication, mechanical design, and signal processing.

---

## 🛠️ Technologies Used

- **MCUs**: STM32F446, STM32F103, Raspberry Pi Pico (RP2040)
- **Languages**: C, Python (telemetry)
- **Frameworks**: FreeRTOS, AL94 USB Composite
- **Protocols**: CAN bus, USB HID, UART, SPI
- **Tools**: Fusion 360 (CAD and Electronic Design), Logic Analyzers, Oscilloscope, Git
- **Misc**: Custom PCBs, Neopixel RPM LEDs, Analog sensors

---

## 🚀 Getting Started

Below is the directory tree description:

```ascii
sim-wheel/
├── .git/                         # Git version control folder
├── .vscode/                      # VSCode config files
├── ac_sharedmemory/              # Telemetry interface program (main)
├── ac_telemetry_client/          # R&D telemetry testing tool
├── docs/                         # Project documentation
├── freertos/                     # STM32 wheelbase firmware (FreeRTOS-based)
├── pico/                         # R&D for Raspberry Pi Pico (RP2040)
├── Pico_Wheel/                   # R&D for Pico wheel firmware
├── Proj/                         # General R&D for RP2040
├── stm-pedal/                    # STM32 firmware for pedal input system
├── stm-wheel/                    # STM32 firmware for steering wheel buttons/inputs
├── stm-wheelbase/                # (Legacy) wheelbase firmware before FreeRTOS refactor
├── .gitignore                    # Git ignore rules
├── LICENSE                       # GNU GPL v3.0 license
└── README.md                     # Project overview and setup instructions
```

To build and run the firmware:

Clone the repo:

   ```bash
   git clone https://github.com/YourUsername/sim-wheel.git
   cd sim-wheel
   ```

We used STM32CubeIDE for all firmware development. To get started, install STM32CubeIDE and optionally STM32CubeProgrammer for flashing and debugging support.

Projects can be imported by selecting:
File > Import > Existing Projects into Workspace in STM32CubeIDE.

To flash and debug the firmware, you'll need either:

- an ST-Link debugger (external or built-in), or
- a Nucleo board with onboard USB ST-Link interface.
