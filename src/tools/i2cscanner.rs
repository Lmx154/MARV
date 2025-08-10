// tools/i2cscanner.rs
use embedded_hal::i2c::I2c;
use core::fmt::Write;
use defmt::info;

/// Verbose scanner (kept for reference, currently unused) left behind in case detailed logs are desired later.
#[allow(dead_code)]
pub fn scan_i2c_bus<W: Write, I: I2c>(i2c: &mut I, uart: &mut W, bus_name: &str) {
    let _ = write!(uart, "Scanning {} (verbose)\\r\\n", bus_name);
    for addr in 0x08..=0x77 {
        let mut device_found = false;
        if i2c.write(addr, &[]).is_ok() { device_found = true; }
        else {
            let mut b=[0u8;1];
            if i2c.read(addr,&mut b).is_ok() { device_found = true; }
        }
        if device_found { let _ = write!(uart, "Found 0x{:02X}\\r\\n", addr); }
    }
}

/// Compact scanner: outputs exactly one summary line per bus via UART and a defmt info line.
pub fn scan_i2c_bus_summary<W: Write, I: I2c>(i2c: &mut I, uart: &mut W, bus_name: &str) {
    let mut found: [u8; 16] = [0;16]; // plenty for expected device count
    let mut count: u8 = 0;
    let mut _errors: u16 = 0; // reserved for future diagnostic reporting
    for addr in 0x08..=0x77 {
        let mut present=false;
        match i2c.write(addr,&[]) { Ok(_)=>{present=true;} Err(_)=>{ _errors+=1; } }
        if !present { let mut b=[0u8;1]; if i2c.read(addr,&mut b).is_ok() { present=true; } }
        if present && (count as usize) < found.len() {
            found[count as usize] = addr;
            count += 1;
        }
    }

    // Targeted MS5611 probe (0x76) only on I2C1 where it's expected; avoid touching other buses.
    if bus_name == "I2C1" {
        const MS5611_ADDR: u8 = 0x76;
        let already = (0..count).any(|i| found[i as usize] == MS5611_ADDR);
        if !already {
            if i2c.write(MS5611_ADDR, &[0x1E]).is_ok() { // RESET command
                if (count as usize) < found.len() {
                    found[count as usize] = MS5611_ADDR;
                    count += 1;
                }
            }
        }
    }

    // Build ASCII line
    let _ = write!(uart, "{}:", bus_name);
    if count == 0 { let _ = write!(uart, " (none)\r\n"); info!("{}: none", bus_name); }
    else {
        // Build UART line
        for i in 0..count { let _ = write!(uart, " 0x{:02X}", found[i as usize]); }
        let _ = write!(uart, " ({} dev)\r\n", count);
        // defmt line with addresses (limited to first 8 to keep it short)
        let mut a: [u8;8] = [0;8];
        let copy_len = if count as usize > a.len() { a.len() } else { count as usize };
        for i in 0..copy_len { a[i] = found[i]; }
        // Log first up to 8 addresses individually to stay simple
        info!(
            "{} devices: {=u8}, addrs: {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
            bus_name, count,
            a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]
        );
    }
    let _ = write!(uart, "\r\n");
}