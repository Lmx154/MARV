# RustyPico - GPS Project

A Rust project for the Raspberry Pi Pico 2 (RP2350) that demonstrates GPS communication using the UBX binary protocol.

## Features

- **GPS Communication**: UART0 communication at 38400 baud via GP0 (TX) and GP1 (RX)
- **UBX Protocol Support**: Complete UBX binary protocol parsing
- **Message Parsing**: Full parsing of UBX-NAV-PVT and UBX-NAV-SAT messages
- **Automatic Configuration**: GPS module automatically configured for UBX-only output
- **Comprehensive Display**: Detailed information display for position, velocity, time, and satellite data

## GPS Module Features

### UBX-NAV-PVT Parser
The NAV-PVT (Position, Velocity, Time) message parser provides:
- **Position**: Latitude/longitude (1e-7 degrees), height above ellipsoid and MSL (mm)
- **Velocity**: NED velocity components (mm/s), ground speed, heading
- **Time**: UTC date/time with validity flags and nanosecond precision
- **Accuracy**: Horizontal/vertical position accuracy, speed accuracy, heading accuracy
- **Fix Information**: Fix type, number of satellites, GNSS fix quality
- **Additional Data**: DOP values, differential correction status, power save mode

### UBX-NAV-SAT Parser
The NAV-SAT (Satellite Information) message parser provides:
- **Satellite Details**: GNSS system, satellite ID, signal strength (CNR)
- **Geometry**: Elevation and azimuth angles for each satellite
- **Signal Quality**: Quality indicators, usage status, health information
- **Orbit Data**: Ephemeris, almanac, and AssistNow data availability
- **Corrections**: Differential and smoothing correction status

### Supported GNSS Systems
- GPS (US)
- GLONASS (Russia) 
- Galileo (Europe)
- BeiDou (China)
- QZSS (Japan)
- NavIC/IRNSS (India)
- SBAS augmentation systems

## Hardware Setup

Connect your u-blox GPS module (M8, M9, M10 series) to:
- **GPS TX** → **RP2350 GP1** (UART0 RX)
- **GPS RX** → **RP2350 GP0** (UART0 TX)
- **GPS VCC** → **3.3V**
- **GPS GND** → **GND**

## Usage

The GPS module automatically:
1. Configures UART0 for 38400 baud, 8N1
2. Sends UBX configuration commands to switch GPS to UBX-only mode
3. Enables NAV-PVT and NAV-SAT message output
4. Saves configuration to GPS flash memory
5. Continuously parses and displays received UBX messages

### Example Output

```
GPS[1234]: ===== NAV-PVT Position/Velocity/Time =====
GPS[1234]: UTC Time: 2024-01-15 14:30:45
GPS[1234]: Fix Type: 3 (3D), Satellites: 12, GNSS Fix OK: true
GPS[1234]: Latitude: 520620634 (1e-7 deg), Longitude: -21601284 (1e-7 deg)
GPS[1234]: Height: 86327mm (ellipsoid), 37844mm (MSL)
GPS[1234]: Accuracy: H=1500mm, V=2000mm
GPS[1234]: Velocity: N=0mm/s, E=0mm/s, D=0mm/s
GPS[1234]: Ground Speed: 0mm/s, Heading: 0 (1e-5 deg)
GPS[1234]: ==========================================

GPS[1234]: ===== NAV-SAT Satellite Information =====
GPS[1234]: Sat #01: GPS SV12 - CNR: 43dBHz, Elev: 42°, Azim: 240°
GPS[1234]:   Quality: 5 (code and carrier locked), Used: true, Health: Healthy
GPS[1234]:   Orbit: ephemeris, Eph: true, Alm: true, DiffCorr: false
...
GPS[1234]: Summary: 12 satellites visible, 8 used for navigation
GPS[1234]: ==========================================
```

## Technical Implementation

### UBX Protocol Compliance
The implementation follows the official u-blox Interface Description specification:
- **Sync Pattern**: 0xB5 0x62
- **Message Structure**: Class, ID, Length, Payload, Checksum
- **Little-Endian Encoding**: All multi-byte values in little-endian format
- **Bitfield Parsing**: Proper extraction of flag bits and status fields

### Message Specifications
- **NAV-PVT**: Class 0x01, ID 0x07, Length 92 bytes
- **NAV-SAT**: Class 0x01, ID 0x35, Variable length (8 + 12*numSvs bytes)

### Configuration Commands
- **CFG-PRT**: Port configuration for UART0 UBX-only output
- **CFG-MSG**: Message rate configuration for NAV-PVT and NAV-SAT
- **CFG-CFG**: Save configuration to flash memory

## Prerequisites

- Rust toolchain with `thumbv8m.main-none-eabihf` target installed
- `probe-rs` with RP2350 support installed
- Raspberry Pi Pico configured as a debug probe
- u-blox GPS module (M8, M9, or M10 series recommended)

### Install Rust target for RP2350

```powershell
rustup target add thumbv8m.main-none-eabihf
```

### Install probe-rs with RP2350 support

Since RP2350 support is still in development, install probe-rs from the development branch:

```powershell
cargo install --git https://github.com/konkers/probe-rs --branch wip/2350 probe-rs-tools --locked
```

### Pico Probe Setup

Make sure you have:
1. A second Raspberry Pi Pico flashed with the debug probe firmware
2. Proper SWD connections between your debug probe and target Pico 2:
   - Connect GND to GND
   - Connect GP2 (SWCLK) on probe to SWCLK on target
   - Connect GP3 (SWDIO) on probe to SWDIO on target
   - Power your target Pico 2 via USB

## Build and Flash

It's really that simple! Just run:

```powershell
cargo run
```

This single command will:
- Build the project in debug mode
- Flash the firmware to your Pico 2 via probe-rs
- Start execution and show real-time debug output in your terminal

### Build for release (optional)

```powershell
cargo run --release
```

## Dependencies

- `rp235x-hal`: Hardware abstraction layer for RP2350
- `embedded-hal-nb`: Non-blocking embedded HAL traits
- `defmt`: Efficient logging for embedded systems
- `heapless`: No-allocation collections for embedded systems

## Files

- `src/main.rs`: Main application entry point
- `src/sensors/gps.rs`: Complete UBX GPS module implementation
- `src/sensors/mod.rs`: Sensor module exports
- `src/hardware.rs`: Hardware configuration definitions

## Notes

- The GPS module starts outputting data immediately after power-on
- Position accuracy depends on satellite visibility and atmospheric conditions
- The implementation handles variable-length NAV-SAT messages with up to 32 satellites
- All UBX messages are displayed, but only NAV-PVT and NAV-SAT are parsed in detail
- Configuration is saved to GPS flash memory and persists across power cycles
- NMEA messages are ignored in favor of UBX binary protocol for better precision and efficiency

## Troubleshooting

- Ensure your Pico probe is properly connected and powered
- Verify probe-rs recognizes your setup: `probe-rs list`
- Check that you're using the development version of probe-rs with RP2350 support
- Make sure the target Pico 2 is powered and the SWD connections are correct
