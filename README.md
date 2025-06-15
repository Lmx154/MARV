# RustyPi

A Rust embedded project for Raspberry Pi Pico 2 that makes the onboard LED blink.

## What it does

This project demonstrates basic embedded Rust programming by:
- Initializing the Pico 2 hardware (clocks, GPIO, watchdog)
- Configuring GPIO pin 25 (onboard LED) as an output
- Blinking the LED in a continuous loop (500ms on, 500ms off)

## Build and Flash Instructions

### Prerequisites

- Rust toolchain with `thumbv8m.main-none-eabihf` target installed
- `picotool` installed (official Raspberry Pi Pico tool)

### 1. Build the project
```bash
cargo build --release
```

### 2. Convert ELF to UF2 using picotool
```bash
picotool elf2uf2 target/thumbv8m.main-none-eabihf/release/rustypi rustypi.uf2
```

### 3. Flash to Pico 2

**Put your Pico 2 in BOOTSEL mode:**
- Hold the BOOTSEL button while plugging in the USB cable
- OR hold BOOTSEL and press the RUN/RESET button if already connected

**Flash the firmware:**
```bash
picotool load rustypi.uf2 -x
```

The `-x` flag executes the program immediately after loading.

## Alternative: Manual copy method

If you prefer, you can also copy the `rustypi.uf2` file directly to the Pico 2 when it appears as a USB drive in BOOTSEL mode. The device will automatically reboot and run your program.