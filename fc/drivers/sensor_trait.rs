use core::ops::DerefMut;

/// Common trait for sensor drivers, aligning with driver layer abstractions.
pub trait SensorDriver {
    type Bus;
    type RawData;
    type ParsedData;
    type Error;

    /// Reads raw data from the sensor using the borrowed bus (O(n) where n is small).
    fn read_raw(&mut self, bus: impl DerefMut<Target = Self::Bus>) -> Result<Self::RawData, Self::Error>;

    /// Parses raw data into usable format (O(1) operations).
    fn parse(&self, raw: Self::RawData) -> Result<Self::ParsedData, Self::Error>;

    /// Applies calibration (future extension; currently no-op for most sensors).
    fn calibrate(&mut self, _params: &()) -> bool { true } // Placeholder; extend with params system later.
}