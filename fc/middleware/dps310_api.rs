//! Middleware for DPS310 sensor
//! Provides a simple API to fetch raw pressure & temperature. Future
//! enhancement: parse calibration coefficients for compensated outputs.

use crate::drivers::dps310::{Dps310, Error as DpsError};
use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;

#[derive(Debug, Copy, Clone, Default)]
pub struct DpsRaw {
    pub pressure: i32,
    pub temperature: i32,
}

pub struct Dps310Middleware<'d> {
    driver: &'d mut Dps310,
    pub present: bool,
}

impl<'d> Dps310Middleware<'d> {
    pub fn new(driver: &'d mut Dps310) -> Self { Self { driver, present: false } }

    pub fn init<I2C, D, E>(&mut self, i2c: &mut I2C, delay: &mut D) -> Result<(), DpsError>
    where I2C: I2c<Error = E>, D: DelayNs {
        match self.driver.init(i2c, delay) {
            Ok(()) => { self.present = true; Ok(()) },
            Err(e) => { self.present = false; Err(e) }
        }
    }

    pub fn read<I2C, E>(&mut self, i2c: &mut I2C) -> Result<DpsRaw, DpsError>
    where I2C: I2c<Error = E> {
        let (p, t) = self.driver.read_raw(i2c)?;
        Ok(DpsRaw { pressure: p, temperature: t })
    }
}
