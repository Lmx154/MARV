# Project Pinout (Authoritative Mapping)

(Original file moved here from repository root; update this copy going forward.)

Refer to `sd_dma_architecture.md` for bus/DMA strategy.

## Summary Table
| Function | Peripheral / Device | MCU Pins | Notes |
|----------|---------------------|----------|-------|
| UART0    | Ublox NEO M9N GPS   | TX=GP0, RX=GP1 | 38_400 baud (initial) |
| UART1    | CP2102 Debug        | TX=GP8, RX=GP9 | 115_200 baud (remapped from GP4/GP5) |
| SPI0     | BMI088 (dual CS)    | MOSI=GP19, MISO=GP16, SCK=GP18, CS_ACCEL=GP17, CS_GYRO=GP22 | - |
| SD (SPI1)| microSD             | MOSI=GP11, MISO=GP12, SCK=GP10, CS=GP13 | Moved onto former CAN (SPI1) pins |
| I2C0     | LIS3MDL, DPS310, ICM20948 | SDA=GP20, SCL=GP21 | 100 kHz -> 400 kHz later |
| I2C1     | RV-8803, MS5611     | SDA=GP2,  SCL=GP3  | RTC + barometer |
| RGB LED  | Discrete (Common Anode) | R=GP14, G=GP15, B=GP27 | Active Low |
| Buzzer   | Passive PWM         | GP26 (PWM Slice 5 CH A) | 1–5 kHz typical |

## Proposed / Reserved
| Purpose | Pin | Notes |
|---------|-----|-------|
| BMI088 Gyro DRDY | GP23 | Proposed interrupt line |
| BMI088 Accel DRDY | GP24 | Proposed interrupt line |
| Future Expansion | GP4–GP7 | Free (SD moved to SPI1) |

## Notes
- Keep CS lines as push-pull outputs, drive High when idle.
- Add 4.7k pull-ups on both I2C busses if not already present.
- Plan SD card level shifting only if >3V3 system variations (otherwise native 3V3 tolerated).
- SD now on SPI1 (former CAN): SCK=GP10, MOSI=GP11, MISO=GP12, CS=GP13.

## Change Log
- v1: Initial migration into docs/.
- v2: Remapped UART1 to GP8/GP9; SD card moved onto SPI0 with MISO=GP4 and CS=GP5; updated future expansion pins.
- v3: SD card moved to SPI1 (using former CAN pins GP10–GP13); CAN controller removed.