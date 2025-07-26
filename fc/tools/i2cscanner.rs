// fc/tools/i2cscanner.rs
use embedded_hal::i2c::I2c;
use core::fmt::Write;

pub fn scan_i2c_bus<W: Write, I: I2c>(i2c: &mut I, uart: &mut W, bus_name: &str) {
    let _ = write!(uart, "Scanning {}:\r\n", bus_name);
    let mut scan_count = 0;
    let mut error_count = 0;
    for addr in 0x08..=0x77 {
        scan_count += 1;
        let mut device_found = false;

        // Method 1: Address-only write (empty buffer)
        match i2c.write(addr, &[]) {
            Ok(_) => { device_found = true; }
            Err(e) => {
                if error_count < 3 {  // Limit logging to first few errors
                    let _ = write!(uart, "Probe 0x{:02X} write failed: {:?}\r\n", addr, e);
                }
                error_count += 1;
            }
        }

        // Method 2: Fallback to read if write failed
        if !device_found {
            let mut buffer = [0u8; 1];
            match i2c.read(addr, &mut buffer) {
                Ok(_) => { device_found = true; }
                Err(_) => {}  // Expected for absent devices
            }
        }

        if device_found {
            let _ = write!(uart, "Found device at 0x{:02X}\r\n", addr);
        }
    }
    let _ = write!(uart, "{} scan complete: {} addresses scanned, {} errors\r\n\r\n", bus_name, scan_count, error_count);
}