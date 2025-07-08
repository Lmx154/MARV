//! I2C Sensor Collection Manager
//!
//! A collection manager that orchestrates multiple I2C sensors on a shared bus.
//! This manager owns the I2C bus and provides temporary access to individual
//! sensor drivers, maintaining clean separation of concerns and avoiding
//! duplication of sensor-specific logic.
//!
//! Architecture Benefits:
//! - Single point of I2C bus ownership (solves Rust ownership issues)
//! - Reuses existing sensor drivers (no code duplication)
//! - Clean separation: manager handles orchestration, drivers handle sensors
//! - Easy to extend with new sensors
//! - Maintains sensor driver independence for testing/reuse

use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;
use defmt::*;

use crate::sensors::pcf8563::{Pcf8563, DateTime, Error as Pcf8563Error};
use crate::sensors::bmp388::{Bmp388, SensorData as BarometerData, Bmp388Error};

/// Sensor Collection Error types
/// 
/// Only covers orchestration-level errors. Sensor-specific errors
/// are passed through from their respective drivers.
#[derive(Debug, Format)]
pub enum SensorCollectionError {
    /// I2C communication error at orchestration level
    I2cError,
    /// RTC driver error
    RtcError(Pcf8563Error),
    /// Barometer driver error  
    BarometerError(Bmp388Error),
    /// Sensor not available in collection
    SensorNotAvailable,
}

/// I2C Sensor Collection Manager
/// 
/// Orchestrates multiple sensors by temporarily creating driver instances
/// for each operation. This avoids complex lifetime management while maintaining
/// clean separation of concerns.
/// 
/// The manager owns the I2C bus and tracks which sensors are available,
/// but delegates all sensor-specific operations to their respective drivers.
pub struct SensorCollection<I2C> {
    i2c: I2C,
    rtc_available: bool,
    barometer_available: bool,
}
impl<I2C> SensorCollection<I2C>
where
    I2C: I2c,
{
    /// Create a new sensor collection from an I2C bus
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            rtc_available: false,
            barometer_available: false,
        }
    }
    
    /// Initialize all available sensors on the I2C bus
    /// 
    /// Attempts to initialize each sensor and tracks availability.
    /// Returns Ok if at least one sensor is available.
    pub fn init_all_sensors<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), SensorCollectionError> {
        info!("Initializing I2C sensor collection...");
        
        // Try to initialize RTC
        self.rtc_available = self.try_init_rtc().is_ok();
        if self.rtc_available {
            info!("PCF8563 RTC is available");
        } else {
            warn!("PCF8563 RTC not available");
        }
        
        // Try to initialize barometer
        self.barometer_available = self.try_init_barometer(delay).is_ok();
        if self.barometer_available {
            info!("BMP388 barometer is available");
        } else {
            warn!("BMP388 barometer not available");
        }
        
        if self.rtc_available || self.barometer_available {
            info!("Sensor collection initialized (RTC: {}, Barometer: {})", 
                  self.rtc_available, self.barometer_available);
            Ok(())
        } else {
            error!("No I2C sensors could be initialized");
            Err(SensorCollectionError::SensorNotAvailable)
        }
    }
    
    /// Check if RTC is available
    pub fn is_rtc_available(&self) -> bool {
        self.rtc_available
    }
    
    /// Check if barometer is available
    pub fn is_barometer_available(&self) -> bool {
        self.barometer_available
    }
    
    /// Get sensor availability status (RTC, Barometer)
    pub fn get_sensor_status(&self) -> (bool, bool) {
        (self.rtc_available, self.barometer_available)
    }
    
    // === RTC Operations ===
    // All RTC operations use the PCF8563 driver temporarily
    
    /// Read RTC date and time
    pub fn read_rtc_datetime(&mut self) -> Result<DateTime, SensorCollectionError> {
        if !self.rtc_available {
            return Err(SensorCollectionError::SensorNotAvailable);
        }
        
        // Create temporary RTC driver instance for this operation
        let mut rtc = Pcf8563::new(&mut self.i2c);
        rtc.read_datetime()
            .map_err(|e| SensorCollectionError::RtcError(e))
    }
    
    /// Write RTC date and time
    pub fn write_rtc_datetime(&mut self, datetime: &DateTime) -> Result<(), SensorCollectionError> {
        if !self.rtc_available {
            return Err(SensorCollectionError::SensorNotAvailable);
        }
        
        // Create temporary RTC driver instance for this operation
        let mut rtc = Pcf8563::new(&mut self.i2c);
        rtc.write_datetime(datetime)
            .map_err(|e| SensorCollectionError::RtcError(e))
    }
    
    /// Check if RTC is running
    pub fn is_rtc_running(&mut self) -> Result<bool, SensorCollectionError> {
        if !self.rtc_available {
            return Err(SensorCollectionError::SensorNotAvailable);
        }
        
        // Create temporary RTC driver instance for this operation
        let mut rtc = Pcf8563::new(&mut self.i2c);
        rtc.is_running()
            .map_err(|e| SensorCollectionError::RtcError(e))
    }
    
    // === Barometer Operations ===
    // All barometer operations use the BMP388 driver temporarily
    
    /// Read barometer measurement
    pub fn read_barometer<D: DelayNs>(&mut self, delay: &mut D) -> Result<BarometerData, SensorCollectionError> {
        if !self.barometer_available {
            return Err(SensorCollectionError::SensorNotAvailable);
        }
        
        // Create temporary barometer driver instance for this operation
        let mut barometer = Bmp388::new_primary(&mut self.i2c);
        // Initialize the barometer with its calibration data
        info!("Reinitializing BMP388 for measurement...");
        barometer.init(delay)
            .map_err(|e| SensorCollectionError::BarometerError(e))?;
        
        barometer.read_measurement(delay)
            .map_err(|e| SensorCollectionError::BarometerError(e))
    }
    
    // === Private Helper Methods ===
    
    /// Try to initialize RTC (internal helper)
    fn try_init_rtc(&mut self) -> Result<(), SensorCollectionError> {
        let mut rtc = Pcf8563::new(&mut self.i2c);
        rtc.init()
            .map_err(|e| SensorCollectionError::RtcError(e))
    }
    
    /// Try to initialize barometer (internal helper)
    fn try_init_barometer<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), SensorCollectionError> {
        let mut barometer = Bmp388::new_primary(&mut self.i2c);
        barometer.init(delay)
            .map_err(|e| SensorCollectionError::BarometerError(e))
    }
}

// === Future Extension Examples ===
// 
// To add a new sensor (e.g., BME280 humidity sensor):
// 1. Create the driver in its own module (e.g., sensors/bme280.rs)
// 2. Add availability tracking field: `humidity_available: bool`
// 3. Add init helper: `try_init_humidity()` 
// 4. Add public methods: `read_humidity()`, `is_humidity_available()`
// 5. Update `init_all_sensors()` and `get_sensor_status()`
//
// For SPI sensors, create a separate `SpiSensorCollection<SPI>` following
// the same pattern but with SPI instead of I2C.