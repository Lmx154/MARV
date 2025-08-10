//! Middleware for MS5611 barometric pressure sensor
//! Provides a simple API to get raw pressure and (future) compensated values.

use crate::drivers::ms5611::{Ms5611, Error as Ms5611Error};
use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;

#[derive(Debug, Copy, Clone, Default)]
pub struct PressureRaw {
    pub d1: u32,
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

    pub fn read_pressure<I2C: I2c, D: DelayNs, E>(&mut self, i2c: &mut I2C, delay: &mut D) -> Result<PressureRaw, Ms5611Error>
    where I2C: I2c<Error = E> {
        let d1 = self.driver.read_raw_pressure(i2c, delay)?;
        Ok(PressureRaw { d1 })
    }
}
