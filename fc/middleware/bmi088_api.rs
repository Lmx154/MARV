//! Middleware wrapper for BMI088 providing a simplified frame read interface.
//!
//! Exposes raw accelerometer and gyroscope readings as plain i16 counts.

use crate::drivers::bmi088::{Bmi088, Bmi088Raw, Error as BmiError};
use embedded_hal::spi::SpiBus;

pub struct Bmi088Middleware<'d, BUS, CSACC, CSGYR, DELAY>
where
    BUS: SpiBus,
    CSACC: embedded_hal::digital::OutputPin,
    CSGYR: embedded_hal::digital::OutputPin,
    DELAY: embedded_hal::delay::DelayNs,
{
    driver: &'d mut Bmi088<BUS, CSACC, CSGYR, DELAY>,
}

impl<'d, BUS, CSACC, CSGYR, DELAY> Bmi088Middleware<'d, BUS, CSACC, CSGYR, DELAY>
where
    BUS: SpiBus,
    CSACC: embedded_hal::digital::OutputPin,
    CSGYR: embedded_hal::digital::OutputPin,
    DELAY: embedded_hal::delay::DelayNs,
{
    pub fn new(driver: &'d mut Bmi088<BUS, CSACC, CSGYR, DELAY>) -> Self { Self { driver } }

    pub fn init(&mut self) -> Result<(), BmiError> { self.driver.init() }

    pub fn read(&mut self) -> Result<Bmi088Raw, BmiError> { self.driver.read_raw() }
    pub fn read_accel(&mut self) -> Result<[i16;3], BmiError> { self.driver.read_accel() }
    pub fn read_gyro(&mut self) -> Result<[i16;3], BmiError> { self.driver.read_gyro() }
}
