# Hardware Pinouts for MARV Project

This document outlines the hardware configurations and pin assignments for the MARV project, including the Flight Controller (FC), Radio, and Ground Station (GS). All microcontrollers (MCs) are Raspberry Pi Pico 2 boards based on the RP2350A.

## Flight Controller (FC)

- **Microcontroller (MC)**: Raspberry Pi Pico 2 - RP2350A

- **Sensors and Interfaces**:
  - **RTC - PCF8563P (0x51)**: I2C
    - SDA: GP2 (I2C1)
    - SCL: GP3 (I2C1)
  - **Barometer - BMP388 (0x77)**: I2C
    - SDA: GP2 (I2C1)
    - SCL: GP3 (I2C1)
  - **Barometer - MS5611-01BA03 (0x76)**: I2C
    - SDA: GP2 (I2C1)
    - SCL: GP3 (I2C1)
  - **SD Card Storage**: SPI (not using dedicated SPI controller)
    - CLK: GP6
    - MOSI: GP7
    - MISO: GP8
    - CS: GP9
  - **IMU - BMI088**: SPI
    - CS GYRO: GP20 (SPI0)
    - MOSI: GP19 (SPI0)
    - CLK: GP18 (SPI0)
    - CS ACCEL: GP17 (SPI0)
    - MISO: GP16 (SPI0)
  - **CAN Controller - MCP2515**: SPI
    - CLK: GP10 (SPI1)
    - MOSI: GP11 (SPI1)
    - MISO: GP12 (SPI1)
    - CS: GP13 (SPI1)
  - **Magnetometer - BMM350 (0x14)**: I2C
    - SDA: GP20 (I2C0)
    - SCL: GP21 (I2C0)
  - **IMU - ICM-20948 (0x69)**: I2C
    - SDA: GP20 (I2C0)
    - SCL: GP21 (I2C0)
  - **GPS - Ublox NEO-M9N**: UART
    - TX: GP0 (UART0, FC TX to GPS RX)
    - RX: GP1 (UART0, FC RX from GPS TX)
  - **Debug UART (CP2102 for USB serial)**: UART
    - TX: GP4 (FC TX to CP2102 RX)
    - RX: GP5 (FC RX from CP2102 TX)

**Note**: I2C devices are split across two buses: High-frequency sensors (IMU and magnetometer) on I2C0 (GP20 SDA, GP21 SCL); lower-frequency sensors (barometer and RTC) on I2C1 (GP2 SDA, GP3 SCL).

## Radio

- **Microcontroller (MC)**: Raspberry Pi Pico 2 - RP2350A

- **Sensors and Interfaces**:
  - **LoRa Radio - E32900T30D**: UART
    - TX: GP0 (UART0)
    - RX: GP1 (UART0)
  - **CAN Controller - MCP2515**: SPI
    - MOSI: GP19 (SPI0)
    - CLK: GP18 (SPI0)
    - CS: GP17 (SPI0)
    - MISO: GP16 (SPI0)

## Ground Station (GS)

- **Microcontroller (MC)**: Raspberry Pi Pico 2 - RP2350A

- **Sensors and Interfaces**:
  - **LoRa Radio - E32900T30D**: UART
    - TX: GP0 (UART0)
    - RX: GP1 (UART0)

