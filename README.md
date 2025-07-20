# RustyPico - Dual RP2350 Embedded Project

A Rust embedded project for Raspberry Pi Pico 2 (RP2350) with two separate binaries: Flight Controller (FC) and Radio.

## Project Structure

```
rustypico/
├── fc/                     # Flight Controller Project
│   ├── main.rs            # FC main application code
│   └── mcp2515_driver.rs  # MCP2515 CAN controller driver
├── radio/                 # Radio Project
│   └── main.rs           # Radio main application code
├── Cargo.toml            # Project configuration with dual binary setup
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

**Both Binaries:**
```bash
cargo build
```
- Builds both FC and radio binaries simultaneously

### Check for Compilation Errors (No Build)

```bash
cargo check --bin FC      # Check FC only
cargo check --bin radio   # Check radio only
cargo check               # Check both
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

**Default (FC):**
```bash
cargo run
```
- Flashes the FC binary (default binary)

### Device Connection
1. Connect your RP2350 via USB
2. Put device in bootloader mode:
   - Hold BOOTSEL button while plugging in USB, OR
   - Hold BOOTSEL + press RESET button

## VS Code Integration

### Available Tasks (Ctrl+Shift+P → "Tasks: Run Task")
- **Build FC**: Builds flight controller binary only
- **Build Radio**: Builds radio binary only  
- **Build All**: Builds both binaries

### Configuration Files
- **Cargo.toml**: Defines both binaries with their respective paths
- **.vscode/tasks.json**: VS Code build tasks for each binary

## Development Workflow

1. **Choose your target**: Decide whether you're working on FC or Radio
2. **Edit the appropriate files**:
   - FC: Modify files in `fc/` directory
   - Radio: Modify files in `radio/` directory
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

Each binary is completely self-contained and can be developed independently with different sensors, pinouts, and functionality.
