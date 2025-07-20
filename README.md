# RustyPico - Triple RP2350 Embedded Project

A Rust embedded project for Raspberry Pi Pico 2 (RP2350) with three separate binaries: Flight Controller (FC), Radio, and Ground Station (GS).

## Project Structure

```
rustypico/
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

## Hardware Configurations

### Flight Controller (FC)
- **LED**: GPIO25 (onboard LED)
- **MCP2515 CAN Controller on SPI1**:
  - GP10: SCK (Clock)
  - GP11: MOSI (Master Out, Slave In)
  - GP12: MISO (Master In, Slave Out)  
  - GP13: CS (Chip Select)
- **Features**: CAN bus communication, MCP2515 self-test in loopback mode
- **Status LEDs**:
  - 3 fast blinks: Full MCP2515 success (init + functionality test)
  - 2 fast blinks: MCP2515 init success, test failed
  - 5 fast blinks: MCP2515 initialization error
  - Slow blink (500ms): Normal operation

### Radio
- **LED**: GPIO24
- **Features**: Different pinout for radio-specific sensors
- **Status LEDs**:
  - 4 quick blinks at startup: Radio initialization
  - Fast blink (250ms): Normal operation

### Ground Station (GS)
- **LED**: GPIO25 (onboard LED)
- **LoRa Radio - E32900T30D on UART0**:
  - GP0: TX (UART0)
  - GP1: RX (UART0)
- **Features**: Ground station communication and monitoring
- **Status LEDs**:
  - 3 quick blinks at startup: GS initialization
  - Slow blink (500ms): Normal operation
  - Medium blink (350ms): Normal operation

## Build Commands

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

### Check for Compilation Errors (No Build)

```bash
cargo check --bin FC      # Check FC only
cargo check --bin radio   # Check radio only
cargo check --bin gs      # Check GS only
cargo check               # Check all three
```

## Flashing Commands

### Flash to RP2350 Device

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
