//! RV-8803 Real-Time Clock (RTC) minimal I2C driver
//!
//! Goals:
//! - Bring-up on I2C address 0x32
//! - Read/write time and date registers
//! - Handle BCD conversions, 12/24h hours, weekday 0-6
//! - Expose a tiny, blocking API suitable for middleware usage
//!
//! Notes:
//! - This is a minimal subset for timekeeping. Alarms, timers, and temp-comp
//!   features are omitted for now.

use embedded_hal::i2c::I2c;

pub const RV8803_ADDR: u8 = 0x32;

// Register map (subset)
pub mod reg {
    // Timekeeping registers (sequential BCD)
    pub const SECONDS: u8 = 0x00; // 0-59 BCD, bit7 is "oscillator stop flag" (OSF)
    pub const MINUTES: u8 = 0x01; // 0-59 BCD
    pub const HOURS:   u8 = 0x02; // 0-23 BCD (24h mode)
    pub const WEEKDAY: u8 = 0x03; // 0-6 (MON=0 per some docs; can be app-defined)
    pub const DATE:    u8 = 0x04; // 1-31 BCD
    pub const MONTH:   u8 = 0x05; // 1-12 BCD
    pub const YEAR:    u8 = 0x06; // 0-99 BCD (offset from 2000 in middleware)

    // Control/status (subset)
    pub const CTRL:    u8 = 0x0E; // Control register
    pub const STATUS:  u8 = 0x0F; // Status register (contains OSF bit mirror etc.)
}

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct DateTime {
    pub year: u16,   // full year, e.g., 2025
    pub month: u8,   // 1..=12
    pub day: u8,     // 1..=31
    pub weekday: u8, // 0..=6
    pub hour: u8,    // 0..=23
    pub minute: u8,  // 0..=59
    pub second: u8,  // 0..=59
}

#[derive(Debug)]
pub enum Error {
    I2cRead,
    I2cWrite,
    InvalidBcd,
}

pub struct Rv8803 {
    address: u8,
}

impl Rv8803 {
    pub fn new(address: u8) -> Self { Self { address } }

    /// Read current DateTime from the RTC.
    pub fn read_datetime<I2C, E>(&mut self, i2c: &mut I2C) -> Result<DateTime, Error>
    where I2C: I2c<Error = E> {
        let mut buf = [0u8; 7];
        i2c.write_read(self.address, &[reg::SECONDS], &mut buf).map_err(|_| Error::I2cRead)?;

        let sec_raw = buf[0] & 0x7F; // mask OSF if present in seconds reg bit7
        let seconds = bcd_to_bin(sec_raw)?;
        let minutes = bcd_to_bin(buf[1])?;
        // 24h only here
        let hours = bcd_to_bin(buf[2] & 0x3F)?;
        let weekday = buf[3] & 0x07; // keep as-is, app defines mapping
        let day = bcd_to_bin(buf[4] & 0x3F)?;
        let month = bcd_to_bin(buf[5] & 0x1F)?;
        let year = 2000u16 + (bcd_to_bin(buf[6])? as u16);

        Ok(DateTime { year, month, day, weekday, hour: hours, minute: minutes, second: seconds })
    }

    /// Write DateTime into the RTC (24h mode).
    pub fn write_datetime<I2C, E>(&mut self, i2c: &mut I2C, dt: &DateTime) -> Result<(), Error>
    where I2C: I2c<Error = E> {
        let yr = if dt.year >= 2000 { (dt.year - 2000) as u8 } else { 0 };
        let payload = [
            reg::SECONDS,
            bin_to_bcd(dt.second)?,
            bin_to_bcd(dt.minute)?,
            bin_to_bcd(dt.hour)?,
            dt.weekday & 0x07,
            bin_to_bcd(dt.day)?,
            bin_to_bcd(dt.month)?,
            bin_to_bcd(yr)?,
        ];
        i2c.write(self.address, &payload).map_err(|_| Error::I2cWrite)
    }

    /// Read raw status register for diagnostics (e.g., OSF after power loss).
    pub fn read_status<I2C, E>(&mut self, i2c: &mut I2C) -> Result<u8, Error>
    where I2C: I2c<Error = E> {
        let mut b = [0u8;1];
        i2c.write_read(self.address, &[reg::STATUS], &mut b).map_err(|_| Error::I2cRead)?;
        Ok(b[0])
    }

    /// Write status register (e.g., to clear OSF or alarms). Caller supplies mask/values.
    pub fn write_status<I2C, E>(&mut self, i2c: &mut I2C, value: u8) -> Result<(), Error>
    where I2C: I2c<Error = E> {
        i2c.write(self.address, &[reg::STATUS, value]).map_err(|_| Error::I2cWrite)
    }
}

#[inline]
fn bcd_to_bin(b: u8) -> Result<u8, Error> {
    let hi = (b >> 4) & 0x0F; let lo = b & 0x0F;
    if hi <= 9 && lo <= 9 { Ok(hi * 10 + lo) } else { Err(Error::InvalidBcd) }
}

#[inline]
fn bin_to_bcd(v: u8) -> Result<u8, Error> {
    if v > 99 { return Err(Error::InvalidBcd); }
    Ok(((v / 10) << 4) | (v % 10))
}
