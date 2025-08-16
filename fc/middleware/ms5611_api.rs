//! Middleware for MS5611 barometric pressure sensor
//! Provides a simple API to get raw pressure and (future) compensated values.

use crate::drivers::ms5611::{Ms5611, Error as Ms5611Error};
use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;

#[derive(Debug, Copy, Clone, Default)]
pub struct Ms5611Raw {
    pub pressure: u32,   // Raw D1
    pub temperature: u32 // Raw D2
}

pub struct Ms5611Middleware<'d> {
    driver: &'d mut Ms5611,
}

impl<'d> Ms5611Middleware<'d> {
    pub fn new(driver: &'d mut Ms5611) -> Self { Self { driver } }

    pub fn init<I2C: I2c, D: DelayNs, E>(&mut self, i2c: &mut I2C, delay: &mut D) -> Result<(), Ms5611Error>
    where I2C: I2c<Error = E> {
        self.driver.init(i2c, delay)
    }

    pub fn read<I2C: I2c, D: DelayNs, E>(&mut self, i2c: &mut I2C, delay: &mut D) -> Result<Ms5611Raw, Ms5611Error>
    where I2C: I2c<Error = E> {
        // Sequential conversions (blocking ~20ms total). For parity with DPS310 format.
        let (p, t) = self.driver.read_raw_both(i2c, delay)?;
        Ok(Ms5611Raw { pressure: p, temperature: t })
    }

    pub fn read_prom<I2C: I2c, E>(&mut self, i2c: &mut I2C, index: u8) -> Result<u16, Ms5611Error>
    where I2C: I2c<Error = E> {
        self.driver.read_prom(i2c, index).map_err(|_| Ms5611Error::ReadFailed)
    }

    pub fn read_raw_pressure_direct<I2C: I2c, D: DelayNs, E>(&mut self, i2c: &mut I2C, delay: &mut D) -> Result<u32, Ms5611Error>
    where I2C: I2c<Error = E> {
        self.driver.read_raw_pressure(i2c, delay).map_err(|_| Ms5611Error::ReadFailed)
    }
}
