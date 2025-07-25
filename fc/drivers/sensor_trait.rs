// fc/drivers/sensor_trait.rs

pub trait SensorDriver {
    type Bus;
    type RawData;
    type ParsedData;
    type Error;

    fn read_raw(&mut self, bus: &mut Self::Bus) -> Result<Self::RawData, Self::Error>;

    fn parse(&self, raw: Self::RawData) -> Result<Self::ParsedData, Self::Error>;

    fn calibrate(&mut self, _params: &()) -> bool { true }
}