//! MS5611 Barometric Pressure & Temperature Sensor Driver
//!
//! Minimal raw driver exposing 24‑bit ADC conversions for pressure (D1) and
//! temperature (D2) using OSR=4096. No compensation / scaling math is applied;
//! higher‐level middleware may later parse PROM coefficients and compute real
//! engineering units. For now we only return the raw unsigned 24‑bit values.
//!
//! Based on the MS5611-01BA03 datasheet:
//! - I2C address: 0x76 (CSB high) or 0x77 (CSB low)
//! - Reset: send 0x1E then wait >2.8ms
//! - Start D1 conversion: 0x40 | OSR (we use 0x48 for OSR=4096)
//! - Start D2 conversion: 0x50 | OSR (we use 0x58 for OSR=4096)
//! - Conversion time OSR=4096 ~9.04ms (we wait 10ms for safety)
//! - Read ADC: command 0x00 returns 24‑bit big‑endian result
//!
//! Usage Notes:
//! - Connected to I2C1 (GP2 SDA, GP3 SCL).
//! - Printing follows: "MS5611: Press[<raw>] Temp[<raw>]" to mirror DPS310 style.
//! - Only raw values: they are NOT compensated or scaled.

use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;

pub const MS5611_ADDR: u8 = 0x76; // Default (CSB high)
pub const MS5611_ADDR_ALT: u8 = 0x77; // Alternate (CSB low)

/// Register/commands
pub mod commands {
    pub const RESET: u8 = 0x1E;
    pub const CONVERT_D1_OSR4096: u8 = 0x48; // Pressure conversion, OSR=4096
    pub const CONVERT_D2_OSR4096: u8 = 0x58; // Temperature conversion, OSR=4096
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

    /// Read raw temperature ADC value (D2)
    pub fn read_raw_temperature<I2C, DELAY, E>(&mut self, i2c: &mut I2C, delay: &mut DELAY) -> Result<u32, Error>
    where
        I2C: I2c<Error = E>,
        DELAY: DelayNs,
    {
        i2c.write(self.address, &[commands::CONVERT_D2_OSR4096]).map_err(|_| Error::WriteFailed)?;
        delay.delay_ms(10u32);
        let mut buffer = [0u8; 3];
        i2c.write_read(self.address, &[commands::ADC_READ], &mut buffer).map_err(|_| Error::ReadFailed)?;
        let d2 = ((buffer[0] as u32) << 16) | ((buffer[1] as u32) << 8) | (buffer[2] as u32);
        Ok(d2)
    }

    /// Convenience: read pressure then temperature sequentially (blocking ~20ms)
    pub fn read_raw_both<I2C, DELAY, E>(&mut self, i2c: &mut I2C, delay: &mut DELAY) -> Result<(u32,u32), Error>
    where
        I2C: I2c<Error = E>,
        DELAY: DelayNs,
    {
        let p = self.read_raw_pressure(i2c, delay)?;
        let t = self.read_raw_temperature(i2c, delay)?;
        Ok((p,t))
    }

    /// Read a 16-bit calibration PROM coefficient (index 0..6 maps to addresses 0xA2..0xAE)
    pub fn read_prom<I2C, E>(&mut self, i2c: &mut I2C, index: u8) -> Result<u16, Error>
    where
        I2C: I2c<Error = E>,
    {
        if index > 6 { return Err(Error::ReadFailed); }
        let addr = 0xA2 + (index * 2);
        let mut buf = [0u8; 2];
        i2c.write_read(self.address, &[addr], &mut buf).map_err(|_| Error::ReadFailed)?;
        Ok(u16::from_be_bytes(buf))
    }
}