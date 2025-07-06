# RustyPi

A Rust embedded project for Raspberry Pi Pico 2 (RP2350) that makes the onboard LED blink and outputs debug messages via probe-rs.

## What it does

This project demonstrates basic embedded Rust programming by:
- Initializing the Pico 2 hardware (clocks, GPIO, watchdog)
- Configuring GPIO pin 25 (onboard LED) as an output
- Blinking the LED in a continuous loop (500ms on, 500ms off)
- Sending "Hello World" and debug messages via probe-rs/defmt to your terminal

## Prerequisites

- Rust toolchain with `thumbv8m.main-none-eabihf` target installed
- `probe-rs` with RP2350 support installed
- Raspberry Pi Pico configured as a debug probe

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

## Debug Output

When running with `cargo run`, you'll see real-time debug messages in your terminal like:

```
0.000001 [INFO ] Starting RustyPi on RP2350!
0.000002 [INFO ] Hello, World! 🦀
0.000003 [INFO ] Watchdog initialized
0.000004 [INFO ] Clocks configured successfully
0.000005 [INFO ] GPIO25 (LED) configured as output
0.000006 [INFO ] LED ON - Blink #0
0.000007 [INFO ] LED OFF
0.000008 [INFO ] LED ON - Blink #1
0.000009 [INFO ] LED OFF
...
[INFO ] 🎉 Completed 10 blink cycles!
```

The timestamps show microsecond precision, and you can see exactly what's happening on your Pico 2 in real-time!

## Memory Configuration

This project uses the correct memory layout for the RP2350:
- Flash: 4MB starting at 0x10000000  
- RAM: 520KB starting at 0x20000000

## Troubleshooting

- Ensure your Pico probe is properly connected and powered
- Verify probe-rs recognizes your setup: `probe-rs list`
- Check that you're using the development version of probe-rs with RP2350 support
- Make sure the target Pico 2 is powered and the SWD connections are correct
