// fc/tools/i2cscanner.rs
use embedded_hal::i2c::I2c;
use core::fmt::Write;

pub fn scan_i2c_bus<W: Write, I: I2c>(i2c: &mut I, uart: &mut W, bus_name: &str) {
    let _ = write!(uart, "Scanning {}:\r\n", bus_name);
    for addr in 0x08..=0x77 {
        if i2c.write(addr, &[0]).is_ok() {
            let _ = write!(uart, "Found device at 0x{:02X}\r\n", addr);
        }
    }
    let _ = write!(uart, "{} scan complete\r\n\r\n", bus_name);
}