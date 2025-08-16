//! RV-8803 middleware: wraps the raw driver and offers a simple API
//! for initializing and fetching current time in a convenient form.

use crate::drivers::rv8803::{Rv8803, DateTime, Error as RtcError};
use embedded_hal::i2c::I2c;
use heapless::String;
use core::fmt::Write as _;

pub struct Rv8803Api<'d> {
    driver: &'d mut Rv8803,
    pub present: bool,
    last: DateTime,
}

impl<'d> Rv8803Api<'d> {
    pub fn new(driver: &'d mut Rv8803) -> Self { Self { driver, present: false, last: DateTime::default() } }

    pub fn init<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(), RtcError>
    where I2C: I2c<Error = E> {
        // Minimal: just try a read, mark present if OK
        match self.driver.read_datetime(i2c) {
            Ok(dt) => { self.last = dt; self.present = true; Ok(()) },
            Err(e) => { self.present = false; Err(e) }
        }
    }

    pub fn now<I2C, E>(&mut self, i2c: &mut I2C) -> Result<DateTime, RtcError>
    where I2C: I2c<Error = E> {
        let dt = self.driver.read_datetime(i2c)?; self.last = dt; Ok(dt)
    }

    pub fn set_datetime<I2C, E>(&mut self, i2c: &mut I2C, dt: &DateTime) -> Result<(), RtcError>
    where I2C: I2c<Error = E> { self.driver.write_datetime(i2c, dt) }

    pub fn last(&self) -> &DateTime { &self.last }

    pub fn format_iso8601(&self) -> String<32> {
        let mut s = String::new();
        let _ = core::write!(s, "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z",
            self.last.year, self.last.month, self.last.day,
            self.last.hour, self.last.minute, self.last.second);
        s
    }
}
