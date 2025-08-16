//! GPS middleware: transforms raw GpsDriver data into formatted outputs and utility conversions.
use crate::drivers::gps::{GpsDriver, GpsData};
use heapless::String;

pub struct GpsApi {
    last: GpsData,
}

impl GpsApi {
    pub fn new() -> Self { Self { last: GpsData::default() } }
    pub fn update(&mut self, driver: &mut GpsDriver) { driver.poll(); self.last = *driver.last(); }
    pub fn last(&self) -> &GpsData { &self.last }
    pub fn csv(&self) -> String<128> { self.last.format_csv() }
    pub fn utc_seconds_of_day(&self) -> u32 { (self.last.hour as u32)*3600 + (self.last.minute as u32)*60 + self.last.second as u32 }
}
