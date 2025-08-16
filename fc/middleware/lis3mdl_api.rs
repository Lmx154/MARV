//! Middleware layer for LIS3MDL magnetometer
//! Responsible for converting raw sensor readings to higher-level units or structures.
//! Driver keeps only register-level I2C logic.

use crate::drivers::lis3mdl::{Lis3mdl, Error as Lis3Error};
use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;

/// Parsed magnetometer data (microtesla placeholder units not yet scaled)
#[derive(Debug, Copy, Clone, Default)]
pub struct MagData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

pub struct Lis3MdlMiddleware<'d, D> {
    driver: &'d mut Lis3mdl<D>,
}

impl<'d, D> Lis3MdlMiddleware<'d, D>
where
    D: DelayNs,
{
    pub fn new(driver: &'d mut Lis3mdl<D>) -> Self { Self { driver } }

    pub fn init<I2C: I2c>(&mut self, i2c: &mut I2C) -> Result<(), Lis3Error> {
        self.driver.init(i2c)
    }

    pub fn read<I2C: I2c>(&mut self, i2c: &mut I2C) -> Result<MagData, Lis3Error> {
        let raw = self.driver.read_raw(i2c)?;
        Ok(MagData { x: raw[0], y: raw[1], z: raw[2] })
    }
}
