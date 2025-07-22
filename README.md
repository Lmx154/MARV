# MARV Project - Modular Aerial Robotic Vehicle

A Rust embedded project for Raspberry Pi Pico 2 (RP2350) with three separate binaries: Flight Controller (FC), Radio, and Ground Station (GS). This project implements a UAV system where the FC handles flight operations and sensors, the Radio manages wireless communications, and the GS monitors telemetry.

## Project Structure

```
lmx154-rustypico/
├── fc/                     # Flight Controller Project
│   ├── main.rs            # FC main application code
│   └── mcp2515_driver.rs  # MCP2515 CAN controller driver
├── radio/                 # Radio Project
│   └── main.rs           # Radio main application code
├── gs/                    # Ground Station Project
│   └── main.rs           # GS main application code
├── Cargo.toml            # Project configuration with triple binary setup
└── .vscode/tasks.json    # VS Code build tasks
```

## Hardware Requirements

- **Microcontrollers**: Three Raspberry Pi Pico 2 boards (RP2350A) for FC, Radio, and GS.
- **Flight Controller (FC)**:
  - Sensors: BMP388 barometer (I2C1), PCF8563 RTC (I2C1), BMI088 IMU (SPI0), BMM350 magnetometer (I2C0), ICM-20948 IMU (I2C0), Ublox NEO-M9N GPS (UART0).
  - CAN: MCP2515 on SPI1 for communication with Radio.
  - LED: GPIO25.
- **Radio**:
  - LoRa: E32900T30D on UART0.
  - CAN: MCP2515 on SPI0 for communication with FC.
  - LED: GPIO24.
- **Ground Station (GS)**:
  - LoRa: E32900T30D on UART0.
  - LED: GPIO25.
- **Probe for Flashing/Debugging**: Use another Raspberry Pi Pico as a SWD probe (cheaper alternative to dedicated probes). Flash the probe firmware from the official Raspberry Pi documentation.

## Protocols and High-Level Overview

- **Intra-Module (FC-Radio)**: MultiWii Serial Protocol (MSP) over CAN bus for lightweight, binary data transfer (e.g., telemetry, commands). Frames include headers, payloads, and CRC-16-CCITT checksums for integrity.
- **Inter-Module (Radio-GS)**: MAVLink v2 over LoRa (915 MHz) for standardized telemetry and commands, with RSSI/SNR appended.
- **Checksums**: CRC-16-CCITT on all CAN packets to detect corruption.
- **High-Level View**: The FC acquires sensor data (e.g., IMU, GPS), performs estimation (AHRS/EKF) and control, and sends raw data via CAN to the Radio. The Radio encodes/decodes MAVLink and transmits over LoRa to the GS, which forwards to a computer (e.g., QGroundControl). Reverse flow handles commands. This modular setup ensures the FC focuses on real-time flight without comms overhead.

## Installation from Scratch

1. **Install Rust**:
   - Download and install Rustup: `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`.
   - Install nightly toolchain: `rustup install nightly`.
   - Add the target: `rustup target add thumbv8m.main-none-eabihf --toolchain nightly`.

2. **Install probe-rs**:
   - `cargo install probe-rs --features cli`.

3. **Set Up Probe**:
   - Use a Raspberry Pi Pico as a probe (cheaper than dedicated tools). Follow the official guide to flash it as a SWD probe: [Raspberry Pi Debug Probe](https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html).
   - Wiring (essential for flashing/debugging):
     - Connect target SWDIO to probe GP4.
     - Connect target SWDCLK to probe GP5.
     - Ensure common ground (GND) between probe and target.
     - Power the target separately.

## Build Commands

### Checking for Compilation Errors (No Build)

```bash
cargo check --bin FC      # Check FC only
cargo check --bin radio   # Check radio only
cargo check --bin gs      # Check GS only
cargo check               # Check all three
```

### Building Specific Binaries

**Flight Controller:**
```bash
cargo build --bin FC
```
- Builds: `fc/main.rs` and `fc/mcp2515_driver.rs`
- Output: `target/thumbv8m.main-none-eabihf/debug/FC`

**Radio:**
```bash
cargo build --bin radio
```
- Builds: `radio/main.rs`
- Output: `target/thumbv8m.main-none-eabihf/debug/radio`

**Ground Station:**
```bash
cargo build --bin gs
```
- Builds: `gs/main.rs`
- Output: `target/thumbv8m.main-none-eabihf/debug/gs`

**All Binaries:**
```bash
cargo build
```
- Builds FC, radio, and GS binaries simultaneously

## Flashing Commands

Connect the probe as described above. Ensure the target RP2350 is connected and powered.

**Flight Controller:**
```bash
cargo run --bin FC
```
- Compiles `fc/main.rs` + `fc/mcp2515_driver.rs`
- Flashes FC binary to connected RP2350

**Radio:**
```bash
cargo run --bin radio
```
- Compiles `radio/main.rs`
- Flashes radio binary to connected RP2350

**Ground Station:**
```bash
cargo run --bin gs
```
- Compiles `gs/main.rs`
- Flashes GS binary to connected RP2350

**Default (FC):**
```bash
cargo run
```
- Flashes the FC binary (default binary)

## Viewing RTT Output

The project uses defmt-rtt for logging. To view real-time logs:

1. Install defmt tools: `cargo install defmt-print`.
2. Flash the binary first (e.g., `cargo run --bin FC`).
3. In a separate terminal, attach and view RTT:
   ```bash
   probe-rs attach --chip RP2350 --protocol swd
   ```
   - This starts a GDB session; exit it if needed.
   - For defmt: `defmt-print probe-rs -c RP2350 -e target/thumbv8m.main-none-eabihf/debug/FC`
   - If issues, ensure probe-rs is configured for RTT channels.

## VS Code Integration

### Available Tasks (Ctrl+Shift+P → "Tasks: Run Task")
- **Build FC**: Builds flight controller binary only
- **Build Radio**: Builds radio binary only  
- **Build GS**: Builds ground station binary only
- **Build All**: Builds all three binaries

### Configuration Files
- **Cargo.toml**: Defines all three binaries with their respective paths
- **.vscode/tasks.json**: VS Code build tasks for each binary

## Development Workflow

1. **Choose your target**: Decide whether you're working on FC, Radio, or GS
2. **Edit the appropriate files**:
   - FC: Modify files in `fc/` directory
   - Radio: Modify files in `radio/` directory
   - GS: Modify files in `gs/` directory
3. **Build and test**: Use `cargo build --bin <target>` to compile
4. **Flash to device**: Use `cargo run --bin <target>` to upload

## Technical Details

- **Target**: `thumbv8m.main-none-eabihf` (ARM Cortex-M33)
- **HAL**: `rp235x-hal` v0.3.0 for RP2350 support
- **Framework**: RTIC v2.1.1 for real-time concurrency
- **Memory**: Custom `memory.x` linker script for RP2350
- **No Standard Library**: `#![no_std]` embedded environment

## Dependencies

- `cortex-m`: ARM Cortex-M specific functionality
- `embedded-hal`: Hardware abstraction layer traits
- `panic-halt`: Halt on panic for embedded systems
- `rp235x-hal`: Raspberry Pi Pico 2 hardware abstraction
- `rtic`: Real-Time Interrupt-driven Concurrency framework

## License

MIT License - See project metadata for details.

---

Each binary is completely self-contained and can be developed independently with different sensors, pinouts, and functionality. The three systems (FC, Radio, and GS) work together to form the complete MARV project architecture.