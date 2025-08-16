# Project Pinout (Authoritative Mapping)



## Summary Table
| Function | Peripheral / Device | MCU Pins | Notes |
|----------|---------------------|----------|-------|
| UART0    | Ublox NEO M9N GPS   | TX=GP0, RX=GP1 | 38_400 baud (initial) |
| UART1    | CP2102 Debug        | TX=GP8, RX=GP9 | 115_200 baud (remapped from GP4/GP5) |
| SPI0     | BMI088 (dual CS)    | MOSI=GP19, MISO=GP16, SCK=GP18, CS_ACCEL=GP17, CS_GYRO=GP22 | - |
| SPI1| microSD             | MOSI=GP11, MISO=GP12, SCK=GP10, CS=GP13 | Moved onto former CAN (SPI1) pins |
| I2C0     | LIS3MDL, DPS310, ICM20948 | SDA=GP20, SCL=GP21 | 100 kHz -> 400 kHz later |
| I2C1     | RV-8803, MS5611     | SDA=GP2,  SCL=GP3  | RTC + barometer |
| RGB LED  | Discrete (Common Anode) | R=GP14, G=GP15, B=GP27 | Active Low |
| Buzzer   | Passive PWM         | GP26 (PWM Slice 5 CH A) | 1–5 kHz typical |


