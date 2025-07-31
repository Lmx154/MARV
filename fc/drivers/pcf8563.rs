//! PCF8563 Real-Time Clock Driver
//!
//! This driver provides reading of the current date and time from the PCF8563 RTC.
//! Based on the PCF8563 datasheet:
//! - I2C address: 0x51
//! - Initialization: Clear the STOP bit to ensure the clock is running.
//! - Reading time: Burst read from register 0x02 to 0x08, parse BCD values.
//! - Handles century bit for full year calculation.
//! - Does not require owned delay; methods borrow I2C bus.

use embedded_hal::i2c::I2c;

pub const PCF8563_ADDR: u8 = 0x51;

/// Register addresses
pub mod registers {
    pub const CONTROL_1: u8 = 0x00;
    pub const VL_SECONDS: u8 = 0x02; // Start of time/date registers
    // Sequential: Minutes=0x03, Hours=0x04, Days=0x05, Weekdays=0x06, Century_Months=0x07, Years=0x08
}

/// RTC time structure
#[derive(Debug)]
pub struct RtcTime {
    pub day: u8,
    pub month: u8,
    pub year: u16,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

/// Driver errors
#[derive(Debug)]
pub enum Error {
    I2cError,
    ReadFailed,
    WriteFailed,
    InvalidData,
}

/// PCF8563 driver
pub struct Pcf8563 {
    address: u8,
}

impl Pcf8563 {
    /// Create a new driver instance
    pub fn new(address: u8) -> Self {
        Self { address }
    }

    /// Initialize the sensor (ensure clock is running)
    pub fn init<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(), Error>
    where
        I2C: I2c<Error = E>,
    {
        // Clear STOP bit in CONTROL_1 (0x00) to start clock if stopped
        i2c.write(self.address, &[registers::CONTROL_1, 0x00]).map_err(|_| Error::WriteFailed)?;

        Ok(())
    }

    /// Read current date and time
    pub fn read_time<I2C, E>(&mut self, i2c: &mut I2C) -> Result<RtcTime, Error>
    where
        I2C: I2c<Error = E>,
    {
        let mut buffer = [0u8; 7];
        if i2c.write_read(self.address, &[registers::VL_SECONDS], &mut buffer).is_err() {
            return Err(Error::ReadFailed);
        }

        // Parse BCD values
        let second_bcd = buffer[0] & 0x7F; // Mask VL bit
        let second = Self::bcd_to_dec(second_bcd);
        let minute = Self::bcd_to_dec(buffer[1] & 0x7F);
        let hour = Self::bcd_to_dec(buffer[2] & 0x3F);
        let day = Self::bcd_to_dec(buffer[3] & 0x3F);
        let _weekday = buffer[4] & 0x07; // Ignored
        let month_bcd = buffer[5] & 0x1F;
        let month = Self::bcd_to_dec(month_bcd);
        let year = Self::bcd_to_dec(buffer[6]);

        // Century bit from buffer[5] bit 7
        let century = (buffer[5] >> 7) & 0x01;
        let full_year = if century == 0 { 2000 + year as u16 } else { 1900 + year as u16 };

        // Basic validation (e.g., ranges)
        if second > 59 || minute > 59 || hour > 23 || day < 1 || day > 31 || month < 1 || month > 12 || year > 99 {
            return Err(Error::InvalidData);
        }

        Ok(RtcTime {
            day,
            month,
            year: full_year,
            hour,
            minute,
            second,
        })
    }

    fn bcd_to_dec(bcd: u8) -> u8 {
        ((bcd >> 4) * 10) + (bcd & 0x0F)
    }
}