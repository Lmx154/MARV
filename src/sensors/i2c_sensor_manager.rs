//! I2C Sensor Manager
//!
//! Manages multiple I2C sensors on a shared bus to avoid ownership conflicts.
//! This allows the PCF8563 RTC and BMP388 barometer to coexist on the same I2C bus.

use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;
use defmt::*;

// Re-export sensor data types for convenience
pub use crate::sensors::pcf8563::DateTime;
pub use crate::sensors::bmp388::SensorData as BarometerData;

/// PCF8563 RTC I2C address
const PCF8563_ADDRESS: u8 = 0x51;

/// BMP388 Barometer I2C address  
const BMP388_ADDRESS: u8 = 0x77;

/// BMP388 chip ID for verification
const BMP388_CHIP_ID: u8 = 0x50;

/// PCF8563 register addresses
mod pcf8563_registers {
    pub const CONTROL_STATUS_1: u8 = 0x00;
    pub const CONTROL_STATUS_2: u8 = 0x01;
    pub const VL_SECONDS: u8 = 0x02;
    pub const MINUTES: u8 = 0x03;
    pub const HOURS: u8 = 0x04;
    pub const DAYS: u8 = 0x05;
    pub const WEEKDAYS: u8 = 0x06;
    pub const CENTURY_MONTHS: u8 = 0x07;
    pub const YEARS: u8 = 0x08;
}

/// BMP388 register addresses
mod bmp388_registers {
    pub const CHIP_ID: u8 = 0x00;
    pub const STATUS: u8 = 0x03;
    pub const DATA_0: u8 = 0x04;  // Pressure data start
    pub const DATA_3: u8 = 0x07;  // Temperature data start
    pub const PWR_CTRL: u8 = 0x1B;
    pub const OSR: u8 = 0x1C;
    pub const ODR: u8 = 0x1D;
    pub const CONFIG: u8 = 0x1F;
    pub const CMD: u8 = 0x7E;
    pub const CALIB_DATA_START: u8 = 0x31;
    pub const CALIB_DATA_LENGTH: usize = 21;
}

/// BMP388 calibration coefficients
#[derive(Debug, Default)]
struct Bmp388CalibData {
    par_t1: u16,
    par_t2: u16,
    par_t3: i8,
    par_p1: i16,
    par_p2: i16,
    par_p3: i8,
    par_p4: i8,
    par_p5: u16,
    par_p6: u16,
    par_p7: i8,
    par_p8: i8,
    par_p9: i16,
    par_p10: i8,
    par_p11: i8,
}

/// I2C Sensor Manager errors
#[derive(Debug, Format)]
pub enum I2cSensorManagerError {
    /// I2C communication error
    I2cError,
    /// Sensor not found or invalid response
    SensorNotFound,
    /// Invalid chip ID
    InvalidChipId,
    /// Measurement timeout
    Timeout,
    /// RTC is not running
    RtcNotRunning,
}

/// I2C Sensor Manager
/// 
/// Manages multiple sensors on a shared I2C bus including:
/// - PCF8563 Real-Time Clock (0x51)
/// - BMP388 Digital Pressure Sensor (0x77)
pub struct I2cSensorManager<I2C> {
    i2c: I2C,
    rtc_initialized: bool,
    bmp388_initialized: bool,
    bmp388_calib_data: Bmp388CalibData,
    bmp388_t_fine: i32, // Fine temperature for pressure compensation
}

impl<I2C> I2cSensorManager<I2C>
where
    I2C: I2c,
{
    /// Create a new I2C sensor manager
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            rtc_initialized: false,
            bmp388_initialized: false,
            bmp388_calib_data: Bmp388CalibData::default(),
            bmp388_t_fine: 0,
        }
    }
    
    /// Initialize all detected sensors on the I2C bus
    pub fn init_all_sensors<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), I2cSensorManagerError> {
        info!("Initializing I2C sensors...");
        
        // Try to initialize PCF8563 RTC
        match self.init_rtc(delay) {
            Ok(()) => {
                self.rtc_initialized = true;
                info!("PCF8563 RTC initialized successfully");
            }
            Err(e) => {
                warn!("Failed to initialize PCF8563 RTC: {:?}", e);
                self.rtc_initialized = false;
            }
        }
        
        // Try to initialize BMP388 Barometer
        match self.init_bmp388(delay) {
            Ok(()) => {
                self.bmp388_initialized = true;
                info!("BMP388 barometer initialized successfully");
            }
            Err(e) => {
                warn!("Failed to initialize BMP388 barometer: {:?}", e);
                self.bmp388_initialized = false;
            }
        }
        
        if self.rtc_initialized || self.bmp388_initialized {
            info!("I2C sensor manager initialized (RTC: {}, BMP388: {})", 
                  self.rtc_initialized, self.bmp388_initialized);
            Ok(())
        } else {
            error!("No I2C sensors could be initialized");
            Err(I2cSensorManagerError::SensorNotFound)
        }
    }
    
    /// Check if RTC is available and initialized
    pub fn is_rtc_available(&self) -> bool {
        self.rtc_initialized
    }
    
    /// Check if BMP388 barometer is available and initialized
    pub fn is_bmp388_available(&self) -> bool {
        self.bmp388_initialized
    }
    
    /// Read date and time from PCF8563 RTC
    pub fn read_rtc_datetime(&mut self) -> Result<DateTime, I2cSensorManagerError> {
        if !self.rtc_initialized {
            return Err(I2cSensorManagerError::SensorNotFound);
        }
        
        // Read time registers (7 bytes starting from VL_SECONDS)
        let mut time_data = [0u8; 7];
        self.read_sensor_registers(PCF8563_ADDRESS, pcf8563_registers::VL_SECONDS, &mut time_data)?;
        
        // Parse BCD data
        let second = bcd_to_binary(time_data[0] & 0x7F);
        let minute = bcd_to_binary(time_data[1] & 0x7F);
        let hour = bcd_to_binary(time_data[2] & 0x3F);
        let day = bcd_to_binary(time_data[3] & 0x3F);
        let weekday = time_data[4] & 0x07;
        let month = bcd_to_binary(time_data[5] & 0x1F);
        let year = 2000 + bcd_to_binary(time_data[6]) as u16;
        
        Ok(DateTime {
            year,
            month,
            day,
            weekday,
            hour,
            minute,
            second,
        })
    }
    
    /// Write date and time to PCF8563 RTC
    pub fn write_rtc_datetime(&mut self, datetime: &DateTime) -> Result<(), I2cSensorManagerError> {
        if !self.rtc_initialized {
            return Err(I2cSensorManagerError::SensorNotFound);
        }
        
        // Convert to BCD and prepare data
        let time_data = [
            binary_to_bcd(datetime.second),           // VL_SECONDS (VL bit clear)
            binary_to_bcd(datetime.minute),           // MINUTES
            binary_to_bcd(datetime.hour),             // HOURS
            binary_to_bcd(datetime.day),              // DAYS
            datetime.weekday,                         // WEEKDAYS
            binary_to_bcd(datetime.month),            // CENTURY_MONTHS
            binary_to_bcd((datetime.year % 100) as u8), // YEARS
        ];
        
        // Write time registers
        self.write_sensor_registers(PCF8563_ADDRESS, pcf8563_registers::VL_SECONDS, &time_data)?;
        
        // Enable the clock if it's not running
        self.write_sensor_register(PCF8563_ADDRESS, pcf8563_registers::CONTROL_STATUS_1, 0x00)?;
        
        Ok(())
    }
    
    /// Check if PCF8563 RTC is running
    pub fn is_rtc_running(&mut self) -> Result<bool, I2cSensorManagerError> {
        if !self.rtc_initialized {
            return Err(I2cSensorManagerError::SensorNotFound);
        }
        
        // Check the STOP bit in Control Status 1 register
        let control1 = self.read_sensor_register(PCF8563_ADDRESS, pcf8563_registers::CONTROL_STATUS_1)?;
        Ok((control1 & 0x20) == 0) // STOP bit clear means clock is running
    }
    
    /// Read barometer data from BMP388
    pub fn read_bmp388_data<D: DelayNs>(&mut self, delay: &mut D) -> Result<BarometerData, I2cSensorManagerError> {
        if !self.bmp388_initialized {
            return Err(I2cSensorManagerError::SensorNotFound);
        }
        
        // Wait for data ready
        let mut retries = 50;
        loop {
            let status = self.read_sensor_register(BMP388_ADDRESS, bmp388_registers::STATUS)?;
            if (status & 0x60) == 0x60 { // Both pressure and temperature ready
                break;
            }
            
            retries -= 1;
            if retries == 0 {
                return Err(I2cSensorManagerError::Timeout);
            }
            delay.delay_ms(1);
        }
        
        // Read raw data (6 bytes: 3 for pressure, 3 for temperature)
        let mut data = [0u8; 6];
        self.read_sensor_registers(BMP388_ADDRESS, bmp388_registers::DATA_0, &mut data)?;
        
        // Convert raw data
        let raw_pressure = ((data[2] as u32) << 16) | ((data[1] as u32) << 8) | (data[0] as u32);
        let raw_temperature = ((data[5] as u32) << 16) | ((data[4] as u32) << 8) | (data[3] as u32);
        
        // Compensate temperature and pressure
        let temperature = self.compensate_bmp388_temperature(raw_temperature);
        let pressure = self.compensate_bmp388_pressure(raw_pressure);
        let altitude = self.calculate_altitude_from_pressure(pressure);
        
        Ok(BarometerData {
            temperature,
            pressure,
            altitude,
        })
    }
    
    /// Initialize PCF8563 RTC
    fn init_rtc<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), I2cSensorManagerError> {
        // Test communication by reading control register
        let _control1 = self.read_sensor_register(PCF8563_ADDRESS, pcf8563_registers::CONTROL_STATUS_1)?;
        
        // Clear any alarm and timer interrupts
        self.write_sensor_register(PCF8563_ADDRESS, pcf8563_registers::CONTROL_STATUS_2, 0x00)?;
        
        delay.delay_ms(10);
        Ok(())
    }
    
    /// Initialize BMP388 barometer
    fn init_bmp388<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), I2cSensorManagerError> {
        // Check chip ID
        let chip_id = self.read_sensor_register(BMP388_ADDRESS, bmp388_registers::CHIP_ID)?;
        if chip_id != BMP388_CHIP_ID {
            error!("Invalid BMP388 chip ID: 0x{:02X} (expected 0x{:02X})", chip_id, BMP388_CHIP_ID);
            return Err(I2cSensorManagerError::InvalidChipId);
        }
        
        // Soft reset
        self.write_sensor_register(BMP388_ADDRESS, bmp388_registers::CMD, 0xB6)?;
        delay.delay_ms(10);
        
        // Read calibration data
        self.read_bmp388_calibration_data()?;
        
        // Configure sensor
        // Set oversampling: 4x for pressure, 2x for temperature
        self.write_sensor_register(BMP388_ADDRESS, bmp388_registers::OSR, 0x02 | 0x10)?;
        
        // Set output data rate to 50Hz
        self.write_sensor_register(BMP388_ADDRESS, bmp388_registers::ODR, 0x05)?;
        
        // Set IIR filter coefficient to 3
        self.write_sensor_register(BMP388_ADDRESS, bmp388_registers::CONFIG, 0x02)?;
        
        // Enable pressure and temperature measurement, set to normal mode
        self.write_sensor_register(BMP388_ADDRESS, bmp388_registers::PWR_CTRL, 0x01 | 0x02 | 0x30)?;
        
        delay.delay_ms(50); // Wait for first measurement
        Ok(())
    }
    
    /// Read BMP388 calibration data
    fn read_bmp388_calibration_data(&mut self) -> Result<(), I2cSensorManagerError> {
        let mut calib_data = [0u8; bmp388_registers::CALIB_DATA_LENGTH];
        self.read_sensor_registers(BMP388_ADDRESS, bmp388_registers::CALIB_DATA_START, &mut calib_data)?;
        
        // Parse calibration coefficients
        self.bmp388_calib_data.par_t1 = u16::from_le_bytes([calib_data[0], calib_data[1]]);
        self.bmp388_calib_data.par_t2 = u16::from_le_bytes([calib_data[2], calib_data[3]]);
        self.bmp388_calib_data.par_t3 = calib_data[4] as i8;
        self.bmp388_calib_data.par_p1 = i16::from_le_bytes([calib_data[5], calib_data[6]]);
        self.bmp388_calib_data.par_p2 = i16::from_le_bytes([calib_data[7], calib_data[8]]);
        self.bmp388_calib_data.par_p3 = calib_data[9] as i8;
        self.bmp388_calib_data.par_p4 = calib_data[10] as i8;
        self.bmp388_calib_data.par_p5 = u16::from_le_bytes([calib_data[11], calib_data[12]]);
        self.bmp388_calib_data.par_p6 = u16::from_le_bytes([calib_data[13], calib_data[14]]);
        self.bmp388_calib_data.par_p7 = calib_data[15] as i8;
        self.bmp388_calib_data.par_p8 = calib_data[16] as i8;
        self.bmp388_calib_data.par_p9 = i16::from_le_bytes([calib_data[17], calib_data[18]]);
        self.bmp388_calib_data.par_p10 = calib_data[19] as i8;
        self.bmp388_calib_data.par_p11 = calib_data[20] as i8;
        
        Ok(())
    }
    
    /// Compensate BMP388 temperature
    fn compensate_bmp388_temperature(&mut self, raw_temp: u32) -> i32 {
        let par_t1 = self.bmp388_calib_data.par_t1 as f32;
        let par_t2 = self.bmp388_calib_data.par_t2 as f32;
        let par_t3 = self.bmp388_calib_data.par_t3 as f32;
        
        let partial_data1 = raw_temp as f32 - (256.0 * par_t1);
        let partial_data2 = par_t2 * partial_data1;
        let partial_data3 = partial_data1 * partial_data1;
        let partial_data4 = partial_data3 * par_t3;
        let partial_data5 = partial_data2 + partial_data4;
        
        // Store t_fine for pressure compensation
        self.bmp388_t_fine = (partial_data5 / 5120.0) as i32;
        
        // Return temperature in degrees Celsius * 100
        (partial_data5 / 5120.0 * 100.0) as i32
    }
    
    /// Compensate BMP388 pressure
    fn compensate_bmp388_pressure(&self, raw_press: u32) -> u32 {
        let par_p1 = self.bmp388_calib_data.par_p1 as f32;
        let par_p2 = self.bmp388_calib_data.par_p2 as f32;
        let par_p3 = self.bmp388_calib_data.par_p3 as f32;
        let par_p4 = self.bmp388_calib_data.par_p4 as f32;
        let par_p5 = self.bmp388_calib_data.par_p5 as f32;
        let par_p6 = self.bmp388_calib_data.par_p6 as f32;
        let par_p7 = self.bmp388_calib_data.par_p7 as f32;
        let par_p8 = self.bmp388_calib_data.par_p8 as f32;
        let par_p9 = self.bmp388_calib_data.par_p9 as f32;
        let par_p10 = self.bmp388_calib_data.par_p10 as f32;
        let par_p11 = self.bmp388_calib_data.par_p11 as f32;
        
        let t_fine = self.bmp388_t_fine as f32;
        
        let partial_data1 = par_p6 * t_fine;
        let partial_data2 = par_p7 * (t_fine * t_fine);
        let partial_data3 = par_p8 * (t_fine * t_fine * t_fine);
        let partial_out1 = par_p5 + partial_data1 + partial_data2 + partial_data3;
        
        let partial_data1 = par_p2 * t_fine;
        let partial_data2 = par_p3 * (t_fine * t_fine);
        let partial_data3 = par_p4 * (t_fine * t_fine * t_fine);
        let partial_out2 = raw_press as f32 * (par_p1 + partial_data1 + partial_data2 + partial_data3);
        
        let partial_data1 = raw_press as f32 * raw_press as f32;
        let partial_data2 = par_p9 + par_p10 * t_fine;
        let partial_data3 = partial_data1 * partial_data2;
        let partial_data4 = partial_data3 + (raw_press as f32 * raw_press as f32 * raw_press as f32) * par_p11;
        
        let comp_press = partial_out1 + partial_out2 + partial_data4;
        
        // Return pressure in Pascals * 100
        (comp_press * 100.0) as u32
    }
    
    /// Calculate altitude from pressure
    fn calculate_altitude_from_pressure(&self, pressure: u32) -> i32 {
        // Standard sea level pressure in Pa * 100
        const SEA_LEVEL_PRESSURE: f32 = 101325.0 * 100.0;
        
        // Simplified altitude calculation
        let pressure_diff = SEA_LEVEL_PRESSURE - (pressure as f32);
        let altitude = (pressure_diff / SEA_LEVEL_PRESSURE) * 44330.0;
        
        // Return altitude in meters * 100
        (altitude * 100.0) as i32
    }
    
    /// Read a single register from a sensor
    fn read_sensor_register(&mut self, address: u8, register: u8) -> Result<u8, I2cSensorManagerError> {
        let mut data = [0u8; 1];
        self.i2c.write_read(address, &[register], &mut data)
            .map_err(|_| I2cSensorManagerError::I2cError)?;
        Ok(data[0])
    }
    
    /// Read multiple registers from a sensor
    fn read_sensor_registers(&mut self, address: u8, start_register: u8, data: &mut [u8]) -> Result<(), I2cSensorManagerError> {
        self.i2c.write_read(address, &[start_register], data)
            .map_err(|_| I2cSensorManagerError::I2cError)
    }
    
    /// Write a single register to a sensor
    fn write_sensor_register(&mut self, address: u8, register: u8, value: u8) -> Result<(), I2cSensorManagerError> {
        self.i2c.write(address, &[register, value])
            .map_err(|_| I2cSensorManagerError::I2cError)
    }
    
    /// Write multiple registers to a sensor
    fn write_sensor_registers(&mut self, address: u8, start_register: u8, data: &[u8]) -> Result<(), I2cSensorManagerError> {
        let mut buffer = [0u8; 32]; // Adjust size as needed
        if data.len() + 1 > buffer.len() {
            return Err(I2cSensorManagerError::I2cError);
        }
        
        buffer[0] = start_register;
        buffer[1..=data.len()].copy_from_slice(data);
        
        self.i2c.write(address, &buffer[..=data.len()])
            .map_err(|_| I2cSensorManagerError::I2cError)
    }
}

/// Convert BCD to binary
fn bcd_to_binary(bcd: u8) -> u8 {
    (bcd & 0x0F) + ((bcd >> 4) * 10)
}

/// Convert binary to BCD
fn binary_to_bcd(binary: u8) -> u8 {
    ((binary / 10) << 4) | (binary % 10)
}
