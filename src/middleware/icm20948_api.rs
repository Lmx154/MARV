//! Middleware for ICM-20948 IMU (accel + gyro + magnetometer via AK09916)
//! Converts raw register transactions into structured raw sensor frames; scaling left for higher layers.

use crate::drivers::icm20948::{Icm20948, RawImu, Error as ImuError};
use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;

pub type ImuFrame = RawImu; // For now parsed == raw

pub struct Icm20948Middleware<'d, D> {
    driver: &'d mut Icm20948<D>,
}

impl<'d, D> Icm20948Middleware<'d, D>
where
    D: DelayNs,
{
    pub fn new(driver: &'d mut Icm20948<D>) -> Self { Self { driver } }

    pub fn init<I2C: I2c>(&mut self, i2c: &mut I2C) -> Result<(), ImuError> {
        self.driver.init(i2c)
    }

    pub fn read_frame<I2C: I2c>(&mut self, i2c: &mut I2C) -> Result<ImuFrame, ImuError> {
        self.driver.read_raw(i2c)
    }
}
