# RustyPi

A Rust-based project for Raspberry Pi Pico development.

## Build Instructions

To build and prepare the project for deployment, run the following commands in sequence:

### 1. Clean previous builds
```bash
cargo clean
```

### 2. Build the release version
```bash
cargo build --release
```

### 3. Convert to UF2 format
```bash
elf2uf2-rs target/thumbv8m.main-none-eabihf/release/rustypi target/rustypi.uf2
```

## Prerequisites

- Rust toolchain with `thumbv8m.main-none-eabihf` target installed
- `elf2uf2-rs` tool for converting ELF files to UF2 format

## Deployment

After running the build commands, you'll have a `rustypi.uf2` file in the `target/` directory that can be deployed to your Raspberry Pi Pico by copying it to the device when it's in bootloader mode.