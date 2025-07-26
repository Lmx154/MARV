# 🚀 MARV Project - Modular Avionics for Rockets and Vehicles

A Rust embedded project for Raspberry Pi Pico 2 (RP2350) with three separate binaries: Flight Controller (FC), Radio, and Ground Station (GS). This project implements a UAV system where the FC handles flight operations and sensors, the Radio manages wireless communications, and the GS monitors telemetry. ✈️📡🛰️

## 🗂️ Project Structure

```
lmx154-marv/
├── fc/                     # Flight Controller Project
│   ├── main.rs            # FC main application code
│   └── mcp2515_driver.rs  # MCP2515 CAN controller driver
├── radio/                 # Radio Project
│   ├── main.rs           # Radio main application code
│   └── mcp2515_driver.rs  # MCP2515 CAN controller driver
├── gs/                    # Ground Station Project
│   └── main.rs           # GS main application code
├── Cargo.toml            # Project configuration with triple binary setup
└── .vscode/tasks.json    # VS Code build tasks
```

## 🛠️ Hardware Requirements

- **Microcontrollers**: Three Raspberry Pi Pico 2 boards (RP2350A) for FC, Radio, and GS. 🖥️
- **Flight Controller (FC)**: 🛩️
  - Sensors: BMP388 barometer (0x77), PCF8563 RTC (0x51), and MS5611-01BA03 barometer (0x76) (I2C1 on GP2 SDA, GP3 SCL); BMI088 IMU (SPI0 on GP18 CLK, GP19 MOSI, GP16 MISO, GP20 CS GYRO, GP17 CS ACCEL); BMM350 magnetometer (0x14) and ICM-20948 IMU (0x69) (I2C0 on GP20 SDA, GP21 SCL); Ublox NEO-M9N GPS (UART0 on GP0 TX, GP1 RX).
  - CAN: MCP2515 (SPI1 on GP10 CLK, GP11 MOSI, GP12 MISO, GP13 CS) for communication with Radio.
  - SD Card: SPI on GP6 CLK, GP7 MOSI, GP8 MISO, GP9 CS.
  - LED: GPIO25.
  - Debug UART: UART1 on GP4 TX (to CP2102 RX), GP5 RX (from CP2102 TX) for USB serial logging.
- **Radio**: 📻
  - LoRa: E32900T30D (UART0 on GP0 TX, GP1 RX).
  - CAN: MCP2515 (SPI0 on GP18 CLK, GP19 MOSI, GP16 MISO, GP17 CS) for communication with FC.
  - LED: GPIO25.
  - Debug UART: UART1 on GP4 TX (to CP2102 RX), GP5 RX (from CP2102 TX) for USB serial logging.
- **Ground Station (GS)**: 🖥️
  - LoRa: E32900T30D (UART0 on GP0 TX, GP1 RX).
  - LED: GPIO25.
  - Debug UART: UART1 on GP4 TX (to CP2102 RX), GP5 RX (from CP2102 TX) for USB serial logging.
- **Probe for Flashing/Debugging**: Use another Raspberry Pi Pico as a SWD probe (cheaper alternative to dedicated probes). Flash the probe firmware from the official Raspberry Pi documentation. 🧪
- **Debugging Hardware**: CP2102 USB-to-serial converters connected to the debug UART pins on each board for viewing logs on a host computer. 🐞

For detailed pinout diagrams and configurations, refer to `docs/hardware.md`. 📝

## 📡 Protocols and High-Level Overview

- **Intra-Module (FC-Radio)**: MultiWii Serial Protocol (MSP) over CAN bus for lightweight, binary data transfer (e.g., telemetry, commands). Frames include headers, payloads, and CRC-16-CCITT checksums for integrity. 🔗
- **Inter-Module (Radio-GS)**: MAVLink v2 over LoRa (915 MHz) for standardized telemetry and commands, with RSSI/SNR appended. 📶
- **Checksums**: CRC-16-CCITT on all CAN packets to detect corruption. ✅
- **High-Level View**: The FC acquires sensor data (e.g., IMU, GPS), performs estimation (AHRS/EKF) and control, and sends raw data via CAN to the Radio. The Radio encodes/decodes MAVLink and transmits over LoRa to the GS, which forwards to a computer (e.g., QGroundControl). Reverse flow handles commands. This modular setup ensures the FC focuses on real-time flight without comms overhead. 🔄

## 🧑‍💻 Installation from Scratch

1. **Install Rust**: 🦀
   - Download and install Rustup: `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`.
   - Install nightly toolchain: `rustup install nightly`.
   - Add the target: `rustup target add thumbv8m.main-none-eabihf --toolchain nightly`.

2. **Install probe-rs**: 🧪
   - `cargo install probe-rs --features cli`.

3. **Set Up Probe**: 🧰
   - Use a Raspberry Pi Pico as a probe (cheaper than dedicated tools). Follow the official guide to flash it as a SWD probe: [Raspberry Pi Debug Probe](https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html).
   - Wiring (essential for flashing/debugging):
     - Connect target SWDIO to probe GP4.
     - Connect target SWDCLK to probe GP5.
     - Ensure common ground (GND) between probe and target.
     - Power the target separately.

## 🏗️ Build Commands

### 🕵️ Checking for Compilation Errors (No Build)

```bash
cargo check --bin FC      # Check FC only
cargo check --bin radio   # Check radio only
cargo check --bin gs      # Check GS only
cargo check               # Check all three
```

### 🏗️ Building Specific Binaries

**Flight Controller:** 🛩️
```bash
cargo build --bin FC
```
- Builds: `fc/main.rs` and `fc/mcp2515_driver.rs`
- Output: `target/thumbv8m.main-none-eabihf/debug/FC`

**Radio:** 📻
```bash
cargo build --bin radio
```
- Builds: `radio/main.rs` and `radio/mcp2515_driver.rs`
- Output: `target/thumbv8m.main-none-eabihf/debug/radio`

**Ground Station:** 🖥️
```bash
cargo build --bin gs
```
- Builds: `gs/main.rs`
- Output: `target/thumbv8m.main-none-eabihf/debug/gs`

**All Binaries:** 📦
```bash
cargo build
```
- Builds FC, radio, and GS binaries simultaneously

## ⚡ Flashing Commands

Connect the probe as described above. Ensure the target RP2350 is connected and powered. 🔌

**Flight Controller:** 🛩️
```bash
cargo run --bin FC
```
- Compiles `fc/main.rs` + `fc/mcp2515_driver.rs`
- Flashes FC binary to connected RP2350

**Radio:** 📻
```bash
cargo run --bin radio
```
- Compiles `radio/main.rs` + `radio/mcp2515_driver.rs`
- Flashes radio binary to connected RP2350

**Ground Station:** 🖥️
```bash
cargo run --bin gs
```
- Compiles `gs/main.rs`
- Flashes GS binary to connected RP2350

**Default (FC):** 🛩️
```bash
cargo run
```
- Flashes the FC binary (default binary)

## 🖥️ Viewing Debug Output

The project uses UART for logging debug information. Each board (FC, Radio, GS) outputs logs over UART1 (GP4 TX, GP5 RX). Connect a CP2102 USB-to-serial converter to these pins and to a host computer. Use a serial terminal (e.g., minicom, screen, or PuTTY) on the host to view the output at the default baud rate (configured in the UART setup, typically 115200 baud). 🐞

1. Connect the CP2102: Board TX (GP4) to CP2102 RX; Board RX (GP5) to CP2102 TX; share GND.
2. Flash the binary (e.g., `cargo run --bin FC`).
3. Open a serial terminal on the host (e.g., `minicom -b 115200 -D /dev/ttyUSB0` for Linux/macOS, adjusting the device as needed).

Logs will appear in real-time, prefixed with [INFO], [WARN], or [ERROR] for easy identification. 📝

## 💻 VS Code Integration

### 🛠️ Available Tasks (Ctrl+Shift+P → "Tasks: Run Task")
- **Build FC**: Builds flight controller binary only 🛩️
- **Build Radio**: Builds radio binary only 📻
- **Build GS**: Builds ground station binary only 🖥️
- **Build All**: Builds all three binaries 📦

### ⚙️ Configuration Files
- **Cargo.toml**: Defines all three binaries with their respective paths 📝
- **.vscode/tasks.json**: VS Code build tasks for each binary 🛠️

## 🔄 Development Workflow

1. **Choose your target**: Decide whether you're working on FC, Radio, or GS 🎯
2. **Edit the appropriate files**: ✏️
   - FC: Modify files in `fc/` directory
   - Radio: Modify files in `radio/` directory
   - GS: Modify files in `gs/` directory
3. **Build and test**: Use `cargo build --bin <target>` to compile 🧪
4. **Flash to device**: Use `cargo run --bin <target>` to upload ⚡

## ⚙️ Technical Details

- **Target**: `thumbv8m.main-none-eabihf` (ARM Cortex-M33) 🎯
- **HAL**: `rp235x-hal` v0.3.0 for RP2350 support 🛠️
- **Framework**: RTIC v2.1.1 for real-time concurrency ⏱️
- **Memory**: Custom `memory.x` linker script for RP2350 🧠
- **No Standard Library**: `#![no_std]` embedded environment 🚫📚

## 📦 Dependencies

- `cortex-m`: ARM Cortex-M specific functionality 🧠
- `embedded-hal`: Hardware abstraction layer traits 🛠️
- `panic-halt`: Halt on panic for embedded systems 🚨
- `rp235x-hal`: Raspberry Pi Pico 2 hardware abstraction 🖥️
- `rtic`: Real-Time Interrupt-driven Concurrency framework ⏱️

## 📄 License

MIT License - See project metadata for details. 📝

