# BLE-Fitness-Tracker
Bluetooth based Smart Fitness Tracker using EFR32BG24 Chipset

# Developers
Prudhvi Raj Belide, Harshal Wadhwa

## Overview
This project is a bare-metal firmware bring-up for a custom wearable-style fitness tracker built around a Silicon Labs EFR32BG24. The firmware initializes board clocks and GPIO, communicates with multiple peripherals over I2C (IMU, temperature sensor, OLED), and exposes runtime telemetry over UART for validation and debugging. The goal is to prove a full embedded data path from sensor configuration and sampling to user-visible output on the display.

## Hardware
The prototype hardware is a custom PCB using an EFR32BG24 2.4 GHz SoC with I2C peripherals for sensing and display. The firmware is written to support typical wearable peripherals on the same I2C bus, including:
- Bosch BMA400 (accelerometer with step counting support)
- TI TMP117 (high-accuracy digital temperature sensor)
- SSD1306 128x64 OLED (I2C display controller)

## Firmware Summary
The firmware is implemented in C using Silicon Labs EMLIB and is structured around three layers.

Low-level drivers handle clock setup, GPIO routing, I2C transfers, and UART output. The I2C layer supports write, read, and write-then-read sequences suitable for register-mapped sensors and displays, with basic status checking and timeout protection.

Device drivers implement register-level configuration and data conversion. BMA400 is configured for periodic sampling and step counter operation, and TMP117 raw readings are converted to degrees Celsius. The SSD1306 driver writes a framebuffer to the display for stable rendering.

The application layer runs a periodic loop driven by a millisecond time base. It refreshes sensor data on a fixed cadence, updates the UI, and prints debug values over UART so you can verify I2C traffic and numeric conversions during bring-up.

## Toolchain
This project targets Silicon Labs Gecko SDK with Simplicity Studio 5.

Required:
1) Simplicity Studio 5
2) Gecko SDK for the EFR32BG24 family
3) A debug probe (onboard debugger or external) for flashing and SWD debug

## Build and Flash
Open the project in Simplicity Studio 5, make sure the correct target device is selected (EFR32BG24), then build the project using the IDE build button. Flash using the built-in Flash Programmer or by clicking Run.

If you use the command line flow, generate the project from the Simplicity Configurator (if applicable), then build with the generated project settings for your toolchain. The exact CLI steps depend on how your Simplicity project was created.

## Run and Verify
After flashing, power the board (USB-C or your board’s supply rail). The OLED should initialize and show live values such as steps and temperature if the sensors are detected on I2C.

UART is used for bring-up logs and sensor validation. Use a serial terminal at 115200 baud, 8-N-1. On Linux, a typical command looks like:
`screen /dev/ttyACM0 115200`

If you do not see logs, confirm the correct UART pins are routed in GPIO/USART configuration and that your board’s USB-UART (or external adapter) shares ground with the PCB.

## Configuration Notes
I2C device addresses, update intervals, and sampling configuration are defined in the driver/config headers. If you change the IMU range, output data rate, or step-count settings, keep the application update rate consistent with the sensor configuration so you avoid aliasing and confusing results.

If the OLED remains blank, validate the I2C pull-ups, confirm the OLED address (commonly 0x3C), and verify that the initialization sequence matches your display module.

## Repository Layout
`app.c` contains the main application loop, periodic update logic, and high-level orchestration.
`i2c_code.c/.h` contains the reusable I2C transaction layer used by all peripherals.
Other files provide board initialization, peripheral setup, and any device-specific register definitions.

## Known Limitations
This is a bring-up focused firmware and does not implement full wearable power management. For a production wearable, you would typically add deeper sleep states, interrupt-driven sampling, sensor FIFO usage where available, and aggressive duty cycling of peripherals.

## License
Use the license required by your course or team policy. If you are using Silicon Labs SDK sources, retain their headers and licensing terms where applicable.

