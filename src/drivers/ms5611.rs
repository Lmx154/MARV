//! MS5611 Barometric Pressure Sensor Driver
//!
//! This driver provides raw ADC value for pressure from the MS5611 sensor.
//! Based on the MS5611-01BA03 datasheet:
//! - I2C address: 0x76 (CSB high) or 0x77 (CSB low)
//! - Initialization: Reset the device.
//! - Raw pressure: Issue conversion command for D1 (pressure), delay, read 24-bit ADC value.
//! - Uses OSR=4096 for high resolution (conversion time ~9ms).
//! - Methods take &mut delay since no dedicated timer available.
//!
//! Usage Notes:
//! - Connected to I2C1 (GP2 SDA, GP3 SCL).
//! - Append to UART output as "; BARO: x" where x is the raw pressure ADC (D1).
//! - Does not compute compensated pressure; only raw ADC for simplicity.

use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;

pub const MS5611_ADDR: u8 = 0x76; // Default (CSB high)
pub const MS5611_ADDR_ALT: u8 = 0x77; // Alternate (CSB low)

/// Register/commands
pub mod commands {
    pub const RESET: u8 = 0x1E;
    pub const CONVERT_D1_OSR4096: u8 = 0x48; // Pressure conversion, OSR=4096
    pub const ADC_READ: u8 = 0x00;
}

/// Driver errors
#[derive(Debug)]
pub enum Error {
    I2cError,
    WriteFailed,
    ReadFailed,
}

/// MS5611 driver (without owned delay)
pub struct Ms5611 {
    address: u8,
}

impl Ms5611 {
    /// Create a new driver instance
    pub fn new(address: u8) -> Self {
        Self { address }
    }

    /// Initialize the sensor
    pub fn init<I2C, DELAY, E>(&mut self, i2c: &mut I2C, delay: &mut DELAY) -> Result<(), Error>
    where
        I2C: I2c<Error = E>,
        DELAY: DelayNs,
    {
        // Reset device
        i2c.write(self.address, &[commands::RESET]).map_err(|_| Error::WriteFailed)?;
        delay.delay_ms(3u32); // Wait for reset to complete

        Ok(())
    }

    /// Read raw pressure ADC value (D1)
    pub fn read_raw_pressure<I2C, DELAY, E>(&mut self, i2c: &mut I2C, delay: &mut DELAY) -> Result<u32, Error>
    where
        I2C: I2c<Error = E>,
        DELAY: DelayNs,
    {
        // Start pressure conversion
        i2c.write(self.address, &[commands::CONVERT_D1_OSR4096]).map_err(|_| Error::WriteFailed)?;
        delay.delay_ms(10u32); // Wait for conversion (max 9.04ms for OSR=4096)

        // Read ADC
        let mut buffer = [0u8; 3];
        i2c.write_read(self.address, &[commands::ADC_READ], &mut buffer).map_err(|_| Error::ReadFailed)?;

        let d1 = ((buffer[0] as u32) << 16) | ((buffer[1] as u32) << 8) | (buffer[2] as u32);
        Ok(d1)
    }
}