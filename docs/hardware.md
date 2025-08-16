# Hardware Pinouts for MARV Project

This document outlines the hardware configurations and pin assignments for the MARV project, including the Flight Controller (FC), Radio, and Ground Station (GS). All microcontrollers (MCs) are Raspberry Pi Pico 2 boards based on the RP2350A.

## Flight Controller (FC)

- **Microcontroller (MC)**: Raspberry Pi Pico 2 - RP2350A

- **Sensors and Interfaces**:
  - **GPS - Ublox NEO M9N**: UART0
    - TX: GP0 (FC TX to GPS RX)
    - RX: GP1 (FC RX from GPS TX)
    - Baud: 38_400 (initial)
  - **Debug UART - CP2102**: UART1 (remapped)
    - TX: GP8 (FC TX to CP2102 RX)
    - RX: GP9 (FC RX from CP2102 TX)
    - Baud: 115_200
  - **IMU - BMI088 (dual CS)**: SPI0
    - MOSI: GP19
    - MISO: GP16
    - SCK: GP18
    - CS_ACCEL: GP17
    - CS_GYRO: GP22
  - **microSD Card**: SPI1
    - MOSI: GP11
    - MISO: GP12
    - SCK: GP10
    - CS: GP13
  - **I2C0**: LIS3MDL, DPS310, ICM20948
    - SDA: GP20
    - SCL: GP21
    - 100 kHz (upgradeable to 400 kHz)
  - **I2C1**: RV-8803, MS5611
    - SDA: GP2
    - SCL: GP3
  - **RGB LED (Discrete, Common Anode)**:
    - R: GP14
    - G: GP15
    - B: GP27
    - Active Low
  - **Buzzer (Passive PWM)**:
    - GP26 (PWM Slice 5 CH A)
    - 1–5 kHz typical

**Note**: This mapping is authoritative and matches the summary table in `PINOUT.md`. I2C0 is for high-frequency sensors, I2C1 for RTC and barometer. SPI0 and SPI1 are used for IMU and SD card, respectively. Debug UART is remapped from GP4/GP5 to GP8/GP9.

## Radio

- **Microcontroller (MC)**: Raspberry Pi Pico 2 - RP2350A

- **Sensors and Interfaces**:
  - **LoRa Radio - E32900T30D**: UART0
    - TX: GP0
    - RX: GP1
  - **CAN Controller - MCP2515**: SPI0
    - MOSI: GP19
    - MISO: GP16
    - SCK: GP18
    - CS: GP17

## Ground Station (GS)

- **Microcontroller (MC)**: Raspberry Pi Pico 2 - RP2350A

- **Sensors and Interfaces**:
  - **LoRa Radio - E32900T30D**: UART0
    - TX: GP0
    - RX: GP1

