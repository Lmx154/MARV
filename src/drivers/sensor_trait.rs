//! Generic sensor driver trait abstraction (temporary bootstrap version)
//! Keeps a consistent interface for drivers that provide raw + parsed data.

pub trait SensorDriver {
    type Bus;
    type RawData;
    type ParsedData;
    type Error;

    fn read_raw(&mut self, bus: &mut Self::Bus) -> Result<Self::RawData, Self::Error>;
    fn parse(&self, raw: Self::RawData) -> Result<Self::ParsedData, Self::Error>;
}
