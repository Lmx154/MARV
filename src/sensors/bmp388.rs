//! BMP388 Digital Pressure Sensor Driver
//!
//! Driver for the Bosch BMP388 I2C Digital Pressure Sensor
//! 
//! Features:
//! - Temperature measurement (-40 to +85°C)
//! - Pressure measurement (300 to 1250 hPa)
//! - High precision and low noise
//! - Low power consumption
//! - Altitude calculation from pressure

use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;
use defmt::*;

/// BMP388 I2C address (SDO connected to GND)
pub const BMP388_ADDRESS: u8 = 0x77;

/// Alternative BMP388 I2C address (SDO connected to VDD)
pub const BMP388_ADDRESS_ALT: u8 = 0x76;

/// BMP388 register addresses
#[allow(dead_code)]
mod registers {
    pub const CHIP_ID: u8 = 0x00;
    pub const ERR_REG: u8 = 0x02;
    pub const STATUS: u8 = 0x03;
    pub const DATA_0: u8 = 0x04;  // Pressure data start
    pub const DATA_1: u8 = 0x05;
    pub const DATA_2: u8 = 0x06;
    pub const DATA_3: u8 = 0x07;  // Temperature data start
    pub const DATA_4: u8 = 0x08;
    pub const DATA_5: u8 = 0x09;
    pub const SENSORTIME_0: u8 = 0x0C;
    pub const SENSORTIME_1: u8 = 0x0D;
    pub const SENSORTIME_2: u8 = 0x0E;
    pub const EVENT: u8 = 0x10;
    pub const INT_STATUS: u8 = 0x11;
    pub const FIFO_LENGTH_0: u8 = 0x12;
    pub const FIFO_LENGTH_1: u8 = 0x13;
    pub const FIFO_DATA: u8 = 0x14;
    pub const FIFO_WM_0: u8 = 0x15;
    pub const FIFO_WM_1: u8 = 0x16;
    pub const FIFO_CONFIG_1: u8 = 0x17;
    pub const FIFO_CONFIG_2: u8 = 0x18;
    pub const INT_CTRL: u8 = 0x19;
    pub const IF_CONF: u8 = 0x1A;
    pub const PWR_CTRL: u8 = 0x1B;
    pub const OSR: u8 = 0x1C;
    pub const ODR: u8 = 0x1D;
    pub const CONFIG: u8 = 0x1F;
    pub const CMD: u8 = 0x7E;
    
    // Calibration data registers
    pub const CALIB_DATA_START: u8 = 0x31;
    pub const CALIB_DATA_LENGTH: usize = 21;
}

/// BMP388 chip ID
const BMP388_CHIP_ID: u8 = 0x50;

/// Power control register bits
#[allow(dead_code)]
mod power_ctrl {
    pub const PRESS_EN: u8 = 0x01;    // Pressure enable
    pub const TEMP_EN: u8 = 0x02;     // Temperature enable
    pub const MODE_MASK: u8 = 0x30;   // Mode bits
    pub const MODE_SLEEP: u8 = 0x00;  // Sleep mode
    pub const MODE_FORCED: u8 = 0x10; // Forced mode
    pub const MODE_NORMAL: u8 = 0x30; // Normal mode
}

/// Oversampling register bits
#[allow(dead_code)]
mod osr {
    pub const OSR_P_MASK: u8 = 0x07;  // Pressure oversampling mask
    pub const OSR_T_MASK: u8 = 0x38;  // Temperature oversampling mask
    pub const OSR_P_1X: u8 = 0x00;    // 1x oversampling
    pub const OSR_P_2X: u8 = 0x01;    // 2x oversampling
    pub const OSR_P_4X: u8 = 0x02;    // 4x oversampling
    pub const OSR_P_8X: u8 = 0x03;    // 8x oversampling
    pub const OSR_P_16X: u8 = 0x04;   // 16x oversampling
    pub const OSR_P_32X: u8 = 0x05;   // 32x oversampling
    pub const OSR_T_1X: u8 = 0x00;    // 1x oversampling (shifted)
    pub const OSR_T_2X: u8 = 0x08;    // 2x oversampling (shifted)
    pub const OSR_T_4X: u8 = 0x10;    // 4x oversampling (shifted)
    pub const OSR_T_8X: u8 = 0x18;    // 8x oversampling (shifted)
    pub const OSR_T_16X: u8 = 0x20;   // 16x oversampling (shifted)
    pub const OSR_T_32X: u8 = 0x28;   // 32x oversampling (shifted)
}

/// Status register bits
#[allow(dead_code)]
mod status {
    pub const CMD_RDY: u8 = 0x10;     // Command ready
    pub const DRDY_PRESS: u8 = 0x20;  // Pressure data ready
    pub const DRDY_TEMP: u8 = 0x40;   // Temperature data ready
}

/// Calibration coefficients
#[derive(Debug, Default)]
struct CalibData {
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

/// BMP388 sensor data
#[derive(Debug, Clone, Copy, Default)]
pub struct SensorData {
    /// Temperature in degrees Celsius * 100 (e.g., 2573 = 25.73°C)
    pub temperature: i32,
    /// Pressure in Pascals * 100 (e.g., 101325 = 1013.25 hPa)
    pub pressure: u32,
    /// Altitude in meters * 100 (calculated from pressure, e.g., 12345 = 123.45m)
    pub altitude: i32,
}

impl SensorData {
    /// Get temperature in degrees Celsius as floating point representation
    /// Returns (whole_degrees, fractional_part) where fractional_part is 0-99
    pub fn temperature_celsius(&self) -> (i16, u8) {
        let whole = self.temperature / 100;
        let frac = (self.temperature % 100).abs() as u8;
        (whole as i16, frac)
    }
    
    /// Get pressure in hPa as floating point representation
    /// Returns (whole_hpa, fractional_part) where fractional_part is 0-99
    pub fn pressure_hpa(&self) -> (u16, u8) {
        let whole = self.pressure / 100;
        let frac = (self.pressure % 100) as u8;
        (whole as u16, frac)
    }
    
    /// Get altitude in meters as floating point representation
    /// Returns (whole_meters, fractional_part) where fractional_part is 0-99
    pub fn altitude_meters(&self) -> (i16, u8) {
        let whole = self.altitude / 100;
        let frac = (self.altitude % 100).abs() as u8;
        (whole as i16, frac)
    }
}

/// BMP388 errors
#[derive(Debug, Format)]
pub enum Bmp388Error {
    /// I2C communication error
    I2cError,
    /// Invalid chip ID
    InvalidChipId,
    /// Calibration data error
    CalibrationError,
    /// Measurement timeout
    Timeout,
    /// Invalid configuration
    InvalidConfig,
}

/// BMP388 Digital Pressure Sensor
pub struct Bmp388<I2C> {
    i2c: I2C,
    address: u8,
    calib_data: CalibData,
    t_fine: i32, // Fine temperature value for pressure compensation
}

impl<I2C> Bmp388<I2C>
where
    I2C: I2c,
{
    /// Create a new BMP388 instance
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            calib_data: CalibData::default(),
            t_fine: 0,
        }
    }
    
    /// Create a new BMP388 instance with default address (0x77)
    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, BMP388_ADDRESS)
    }
    
    /// Create a new BMP388 instance with alternative address (0x76)
    pub fn new_secondary(i2c: I2C) -> Self {
        Self::new(i2c, BMP388_ADDRESS_ALT)
    }
    
    /// Initialize the BMP388 sensor
    pub fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Bmp388Error> {
        info!("Initializing BMP388 sensor at address 0x{:02X}", self.address);
        
        // Check chip ID
        let chip_id = self.read_register(registers::CHIP_ID)?;
        if chip_id != BMP388_CHIP_ID {
            error!("Invalid BMP388 chip ID: 0x{:02X} (expected 0x{:02X})", chip_id, BMP388_CHIP_ID);
            return Err(Bmp388Error::InvalidChipId);
        }
        info!("BMP388 chip ID verified: 0x{:02X}", chip_id);
        
        // Soft reset
        self.write_register(registers::CMD, 0xB6)?;
        delay.delay_ms(10);
        
        // Read calibration data
        self.read_calibration_data()?;
        info!("BMP388 calibration data loaded");
        
        // Configure sensor
        // Set oversampling: 4x for pressure, 2x for temperature
        self.write_register(registers::OSR, osr::OSR_P_4X | osr::OSR_T_2X)?;
        
        // Set output data rate to 50Hz
        self.write_register(registers::ODR, 0x05)?;
        
        // Set IIR filter coefficient to 3
        self.write_register(registers::CONFIG, 0x02)?;
        
        // Enable pressure and temperature measurement, set to normal mode
        self.write_register(registers::PWR_CTRL, 
            power_ctrl::PRESS_EN | power_ctrl::TEMP_EN | power_ctrl::MODE_NORMAL)?;
        
        delay.delay_ms(50); // Wait for first measurement
        
        info!("BMP388 initialized successfully");
        Ok(())
    }
    
    /// Read a single measurement from the sensor
    pub fn read_measurement<D: DelayNs>(&mut self, delay: &mut D) -> Result<SensorData, Bmp388Error> {
        // Wait for data ready
        let mut retries = 50;
        loop {
            let status = self.read_register(registers::STATUS)?;
            if (status & (status::DRDY_PRESS | status::DRDY_TEMP)) == (status::DRDY_PRESS | status::DRDY_TEMP) {
                info!("BMP388 data ready, status: 0x{:02X}", status);
                break;
            }
            
            retries -= 1;
            if retries == 0 {
                warn!("BMP388 timeout waiting for data ready, final status: 0x{:02X}", status);
                return Err(Bmp388Error::Timeout);
            }
            delay.delay_ms(1);
        }
        
        // Read raw data (6 bytes: 3 for pressure, 3 for temperature)
        let mut data = [0u8; 6];
        self.read_registers(registers::DATA_0, &mut data)?;
        
        // Convert raw data
        let raw_pressure = ((data[2] as u32) << 16) | ((data[1] as u32) << 8) | (data[0] as u32);
        let raw_temperature = ((data[5] as u32) << 16) | ((data[4] as u32) << 8) | (data[3] as u32);
        
        info!("BMP388 raw data - temp: 0x{:06X} ({}), press: 0x{:06X} ({})", 
              raw_temperature, raw_temperature, raw_pressure, raw_pressure);
        
        // Compensate temperature
        let temperature = self.compensate_temperature(raw_temperature);
        
        // Compensate pressure (requires temperature compensation first)
        let pressure = self.compensate_pressure(raw_pressure);
        
        // Calculate altitude (using standard atmosphere model)
        let altitude = self.calculate_altitude(pressure);
        
        info!("BMP388 compensated - temp: {} (x100°C), press: {} (x100Pa), alt: {} (x100m)", 
              temperature, pressure, altitude);
        
        Ok(SensorData {
            temperature,
            pressure,
            altitude,
        })
    }
    
    /// Get the current mode of the sensor
    pub fn get_mode(&mut self) -> Result<u8, Bmp388Error> {
        let pwr_ctrl = self.read_register(registers::PWR_CTRL)?;
        Ok((pwr_ctrl & power_ctrl::MODE_MASK) >> 4)
    }
    
    /// Set the sensor to sleep mode
    pub fn sleep(&mut self) -> Result<(), Bmp388Error> {
        let mut pwr_ctrl = self.read_register(registers::PWR_CTRL)?;
        pwr_ctrl &= !power_ctrl::MODE_MASK;
        pwr_ctrl |= power_ctrl::MODE_SLEEP;
        self.write_register(registers::PWR_CTRL, pwr_ctrl)
    }
    
    /// Wake up the sensor (set to normal mode)
    pub fn wake_up(&mut self) -> Result<(), Bmp388Error> {
        let mut pwr_ctrl = self.read_register(registers::PWR_CTRL)?;
        pwr_ctrl &= !power_ctrl::MODE_MASK;
        pwr_ctrl |= power_ctrl::MODE_NORMAL;
        self.write_register(registers::PWR_CTRL, pwr_ctrl)
    }
    
    /// Read calibration data from the sensor
    fn read_calibration_data(&mut self) -> Result<(), Bmp388Error> {
        let mut calib_data = [0u8; registers::CALIB_DATA_LENGTH];
        self.read_registers(registers::CALIB_DATA_START, &mut calib_data)?;
        
        // Parse calibration coefficients according to datasheet
        self.calib_data.par_t1 = u16::from_le_bytes([calib_data[0], calib_data[1]]);
        self.calib_data.par_t2 = u16::from_le_bytes([calib_data[2], calib_data[3]]);
        self.calib_data.par_t3 = calib_data[4] as i8;
        self.calib_data.par_p1 = i16::from_le_bytes([calib_data[5], calib_data[6]]);
        self.calib_data.par_p2 = i16::from_le_bytes([calib_data[7], calib_data[8]]);
        self.calib_data.par_p3 = calib_data[9] as i8;
        self.calib_data.par_p4 = calib_data[10] as i8;
        self.calib_data.par_p5 = u16::from_le_bytes([calib_data[11], calib_data[12]]);
        self.calib_data.par_p6 = u16::from_le_bytes([calib_data[13], calib_data[14]]);
        self.calib_data.par_p7 = calib_data[15] as i8;
        self.calib_data.par_p8 = calib_data[16] as i8;
        self.calib_data.par_p9 = i16::from_le_bytes([calib_data[17], calib_data[18]]);
        self.calib_data.par_p10 = calib_data[19] as i8;
        self.calib_data.par_p11 = calib_data[20] as i8;
        
        // Debug log calibration data
        info!("BMP388 calibration data:");
        info!("  T1={}, T2={}, T3={}", self.calib_data.par_t1, self.calib_data.par_t2, self.calib_data.par_t3);
        info!("  P1={}, P2={}, P3={}, P4={}", self.calib_data.par_p1, self.calib_data.par_p2, self.calib_data.par_p3, self.calib_data.par_p4);
        info!("  P5={}, P6={}, P7={}, P8={}", self.calib_data.par_p5, self.calib_data.par_p6, self.calib_data.par_p7, self.calib_data.par_p8);
        info!("  P9={}, P10={}, P11={}", self.calib_data.par_p9, self.calib_data.par_p10, self.calib_data.par_p11);
        
        Ok(())
    }
    
    /// Compensate temperature according to BMP388 datasheet
    fn compensate_temperature(&mut self, raw_temp: u32) -> i32 {
        // Temperature compensation using Bosch's algorithm from BMP388 datasheet
        let var1 = raw_temp as f64 - (self.calib_data.par_t1 as f64 * 256.0);
        let var2 = self.calib_data.par_t2 as f64 * var1;
        let var3 = var1 * var1;
        let var4 = var3 * (self.calib_data.par_t3 as f64 / 16777216.0);
        
        // Calculate compensated temperature
        let t_comp = (var2 + var4) / 5120.0;
        
        // Store t_fine for pressure compensation (this is the intermediate value, not scaled)
        self.t_fine = (var2 + var4) as i32;
        
        info!("BMP388 temp compensation: raw={}, var1={}, var2={}, var4={}, t_comp={}, t_fine={}", 
              raw_temp, var1, var2, var4, t_comp, self.t_fine);
        
        // Return temperature in degrees Celsius * 100
        (t_comp * 100.0) as i32
    }
    
    /// Compensate pressure according to BMP388 datasheet
    fn compensate_pressure(&self, raw_press: u32) -> u32 {
        // Pressure compensation using Bosch's algorithm from BMP388 datasheet
        let t_lin = self.t_fine as f64;
        
        // First part of pressure compensation
        let var1 = self.calib_data.par_p6 as f64 * t_lin;
        let var2 = self.calib_data.par_p7 as f64 * t_lin * t_lin;
        let var3 = self.calib_data.par_p8 as f64 * t_lin * t_lin * t_lin;
        let var4 = self.calib_data.par_p5 as f64 + var1 + var2 + var3;
        
        // Second part of pressure compensation
        let var5 = self.calib_data.par_p2 as f64 * t_lin;
        let var6 = self.calib_data.par_p3 as f64 * t_lin * t_lin;
        let var7 = self.calib_data.par_p4 as f64 * t_lin * t_lin * t_lin;
        let var8 = raw_press as f64 * (self.calib_data.par_p1 as f64 + var5 + var6 + var7);
        
        // Third part of pressure compensation
        let var9 = raw_press as f64 * raw_press as f64;
        let var10 = self.calib_data.par_p9 as f64 + self.calib_data.par_p10 as f64 * t_lin;
        let var11 = var9 * var10;
        let var12 = var11 + (raw_press as f64 * raw_press as f64 * raw_press as f64 * self.calib_data.par_p11 as f64);
        
        // Apply scaling factors according to datasheet
        let var4_scaled = var4 / 4.0;
        let var8_scaled = var8 / 524288.0;
        let var12_scaled = var12 / 4294967296.0;
        
        let comp_press = var4_scaled + var8_scaled + var12_scaled;
        
        info!("BMP388 press compensation: raw={}, t_lin={}, var4_scaled={}, var8_scaled={}, var12_scaled={}, comp_press={}", 
              raw_press, t_lin, var4_scaled, var8_scaled, var12_scaled, comp_press);
        
        // Return pressure in Pascals * 100 (ensure positive value)
        if comp_press > 0.0 {
            (comp_press * 100.0) as u32
        } else {
            0
        }
    }
    
    /// Calculate altitude from pressure using standard atmosphere model
    fn calculate_altitude(&self, pressure: u32) -> i32 {
        // Standard sea level pressure in Pa * 100
        const SEA_LEVEL_PRESSURE: f32 = 101325.0 * 100.0;
        
        // Simplified altitude calculation without powf (not available in no_std)
        // This is an approximation: altitude ≈ 44330 * (1 - (P/P0)^(1/5.255))
        // We'll use a linear approximation for now: altitude ≈ 44.33 * (P0 - P) / P0 * 1000
        let pressure_diff = SEA_LEVEL_PRESSURE - (pressure as f32);
        let altitude = (pressure_diff / SEA_LEVEL_PRESSURE) * 44330.0;
        
        // Return altitude in meters * 100
        (altitude * 100.0) as i32
    }
    
    /// Read a single register
    fn read_register(&mut self, register: u8) -> Result<u8, Bmp388Error> {
        let mut data = [0u8; 1];
        self.i2c.write_read(self.address, &[register], &mut data)
            .map_err(|_| Bmp388Error::I2cError)?;
        Ok(data[0])
    }
    
    /// Read multiple registers
    fn read_registers(&mut self, start_register: u8, data: &mut [u8]) -> Result<(), Bmp388Error> {
        self.i2c.write_read(self.address, &[start_register], data)
            .map_err(|_| Bmp388Error::I2cError)
    }
    
    /// Write a single register
    fn write_register(&mut self, register: u8, value: u8) -> Result<(), Bmp388Error> {
        self.i2c.write(self.address, &[register, value])
            .map_err(|_| Bmp388Error::I2cError)
    }
    
    /// Release the I2C bus (consume self and return the I2C instance)
    pub fn release_i2c(self) -> I2C {
        self.i2c
    }
}
