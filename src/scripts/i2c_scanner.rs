//! I2C Scanner Module
//! 
//! Scans all I2C buses for connected devices and reports their addresses.

use rp235x_hal as hal;
use hal::gpio::{FunctionI2C, Pin, PullUp, bank0::{Gpio4, Gpio5, Gpio6, Gpio7}};
use hal::i2c::{I2C, Error as I2cError};
use hal::pac;
use defmt::*;
use embedded_hal::i2c::I2c;
use crate::hardware::HARDWARE;

/// I2C Scanner that can scan both I2C0 and I2C1 buses
pub struct I2cScanner {
    i2c0: Option<I2C<pac::I2C0, (Pin<Gpio4, FunctionI2C, PullUp>, Pin<Gpio5, FunctionI2C, PullUp>)>>,
    i2c1: Option<I2C<pac::I2C1, (Pin<Gpio6, FunctionI2C, PullUp>, Pin<Gpio7, FunctionI2C, PullUp>)>>,
}

impl I2cScanner {
    /// Create a new I2C scanner
    pub fn new() -> Self {
        Self {
            i2c0: None,
            i2c1: None,
        }
    }
    
    /// Get the configured I2C0 pin numbers from hardware config
    pub fn get_i2c0_pins() -> (u8, u8) {
        HARDWARE.i2c0_pins()
    }
    
    /// Get the configured I2C1 pin numbers from hardware config
    pub fn get_i2c1_pins() -> (u8, u8) {
        HARDWARE.i2c1_pins()
    }
    
    /// Validate that the hardcoded pins match the hardware configuration
    pub fn validate_pin_configuration() -> Result<(), &'static str> {
        let (hw_i2c0_sda, hw_i2c0_scl) = HARDWARE.i2c0_pins();
        let (hw_i2c1_sda, hw_i2c1_scl) = HARDWARE.i2c1_pins();
        
        // Check if configured pins match what this scanner supports
        if hw_i2c0_sda != 4 || hw_i2c0_scl != 5 {
            error!("I2C0 pin configuration mismatch! Hardware config: GP{}/GP{}, Scanner expects: GP4/GP5", 
                   hw_i2c0_sda, hw_i2c0_scl);
            return Err("I2C0 pin configuration not supported by scanner");
        }
        
        if hw_i2c1_sda != 6 || hw_i2c1_scl != 7 {
            error!("I2C1 pin configuration mismatch! Hardware config: GP{}/GP{}, Scanner expects: GP6/GP7", 
                   hw_i2c1_sda, hw_i2c1_scl);
            return Err("I2C1 pin configuration not supported by scanner");
        }
        
        info!("I2C pin configuration validated successfully");
        Ok(())
    }
    
    /// Initialize I2C0 bus
    pub fn init_i2c0(
        &mut self,
        i2c0_device: pac::I2C0,
        sda_pin: Pin<Gpio4, hal::gpio::FunctionNull, hal::gpio::PullDown>,
        scl_pin: Pin<Gpio5, hal::gpio::FunctionNull, hal::gpio::PullDown>,
        resets: &mut pac::RESETS,
        system_clock: hal::fugit::HertzU32,
    ) -> Result<(), I2cError> {
        let (sda_pin_num, scl_pin_num) = Self::get_i2c0_pins();
        info!("Initializing I2C0 bus on pins GP{} (SDA) and GP{} (SCL)", sda_pin_num, scl_pin_num);
        
        let sda = sda_pin.into_function::<FunctionI2C>().into_pull_type::<PullUp>();
        let scl = scl_pin.into_function::<FunctionI2C>().into_pull_type::<PullUp>();
        
        let i2c = I2C::i2c0(
            i2c0_device,
            sda,
            scl,
            hal::fugit::HertzU32::Hz(100_000), // 100kHz
            resets,
            system_clock,
        );
        
        self.i2c0 = Some(i2c);
        Ok(())
    }
    
    /// Initialize I2C1 bus
    pub fn init_i2c1(
        &mut self,
        i2c1_device: pac::I2C1,
        sda_pin: Pin<Gpio6, hal::gpio::FunctionNull, hal::gpio::PullDown>,
        scl_pin: Pin<Gpio7, hal::gpio::FunctionNull, hal::gpio::PullDown>,
        resets: &mut pac::RESETS,
        system_clock: hal::fugit::HertzU32,
    ) -> Result<(), I2cError> {
        let (sda_pin_num, scl_pin_num) = Self::get_i2c1_pins();
        info!("Initializing I2C1 bus on pins GP{} (SDA) and GP{} (SCL)", sda_pin_num, scl_pin_num);
        
        let sda = sda_pin.into_function::<FunctionI2C>().into_pull_type::<PullUp>();
        let scl = scl_pin.into_function::<FunctionI2C>().into_pull_type::<PullUp>();
        
        let i2c = I2C::i2c1(
            i2c1_device,
            sda,
            scl,
            hal::fugit::HertzU32::Hz(100_000), // 100kHz
            resets,
            system_clock,
        );
        
        self.i2c1 = Some(i2c);
        Ok(())
    }
    
    /// Scan I2C0 bus for devices
    pub fn scan_i2c0(&mut self) -> heapless::Vec<u8, 128> {
        let mut found_addresses = heapless::Vec::new();
        
        if let Some(ref mut i2c) = self.i2c0 {
            info!("Scanning I2C0 bus...");
            let mut scan_count = 0;
            let mut error_count = 0;
            
            // Scan addresses 0x08 to 0x77 (valid 7-bit I2C addresses)
            for addr in 0x08..=0x77 {
                scan_count += 1;
                
                // Try multiple approaches to detect devices
                let mut device_found = false;
                
                // Method 1: Try to write 0 bytes (address-only transaction)
                match i2c.write(addr, &[]) {
                    Ok(_) => {
                        device_found = true;
                    }
                    Err(e) => {
                        // Log first few errors for debugging
                        if error_count < 3 {
                            warn!("I2C0 write error at 0x{:02X}: {:?}", addr, e);
                        }
                        error_count += 1;
                    }
                }
                
                // Method 2: If write failed, try read (some devices only respond to reads)
                if !device_found {
                    let mut buffer = [0u8; 1];
                    match i2c.read(addr, &mut buffer) {
                        Ok(_) => {
                            device_found = true;
                        }
                        Err(_) => {
                            // Continue - this is expected for non-existent devices
                        }
                    }
                }
                
                if device_found {
                    info!("Device found at address: 0x{:02X}", addr);
                    let _ = found_addresses.push(addr);
                }
            }
            
            info!("I2C0 scan complete: {} addresses scanned, {} errors, {} devices found", 
                  scan_count, error_count, found_addresses.len());
        } else {
            warn!("I2C0 not initialized");
        }
        
        found_addresses
    }
    
    /// Scan I2C1 bus for devices
    pub fn scan_i2c1(&mut self) -> heapless::Vec<u8, 128> {
        let mut found_addresses = heapless::Vec::new();
        
        if let Some(ref mut i2c) = self.i2c1 {
            info!("Scanning I2C1 bus...");
            let mut scan_count = 0;
            let mut error_count = 0;
            
            // Scan addresses 0x08 to 0x77 (valid 7-bit I2C addresses)
            for addr in 0x08..=0x77 {
                scan_count += 1;
                
                // Try multiple approaches to detect devices
                let mut device_found = false;
                
                // Method 1: Try to write 0 bytes (address-only transaction)
                match i2c.write(addr, &[]) {
                    Ok(_) => {
                        device_found = true;
                    }
                    Err(e) => {
                        // Log first few errors for debugging
                        if error_count < 3 {
                            warn!("I2C1 write error at 0x{:02X}: {:?}", addr, e);
                        }
                        error_count += 1;
                    }
                }
                
                // Method 2: If write failed, try read (some devices only respond to reads)
                if !device_found {
                    let mut buffer = [0u8; 1];
                    match i2c.read(addr, &mut buffer) {
                        Ok(_) => {
                            device_found = true;
                        }
                        Err(_) => {
                            // Continue - this is expected for non-existent devices
                        }
                    }
                }
                
                if device_found {
                    info!("Device found at address: 0x{:02X}", addr);
                    let _ = found_addresses.push(addr);
                }
            }
            
            info!("I2C1 scan complete: {} addresses scanned, {} errors, {} devices found", 
                  scan_count, error_count, found_addresses.len());
        } else {
            warn!("I2C1 not initialized");
        }
        
        found_addresses
    }
    
    /// Scan both I2C buses and print results
    pub fn scan_and_print(&mut self) {
        info!("Starting I2C bus scan...");
        
        // Show hardware configuration
        let (i2c0_sda, i2c0_scl) = Self::get_i2c0_pins();
        let (i2c1_sda, i2c1_scl) = Self::get_i2c1_pins();
        info!("Hardware config - I2C0: GP{} (SDA), GP{} (SCL)", i2c0_sda, i2c0_scl);
        info!("Hardware config - I2C1: GP{} (SDA), GP{} (SCL)", i2c1_sda, i2c1_scl);
        
        // Scan I2C0
        let i2c0_devices = self.scan_i2c0();
        info!("I2C 0:");
        if i2c0_devices.is_empty() {
            info!("  No devices found on bus 0");
        } else {
            for addr in &i2c0_devices {
                info!("  Device found at address: 0x{:02X}", addr);
            }
        }
        
        // Scan I2C1  
        let i2c1_devices = self.scan_i2c1();
        info!("I2C 1:");
        if i2c1_devices.is_empty() {
            info!("  No devices found on bus 1");
        } else {
            for addr in &i2c1_devices {
                info!("  Device found at address: 0x{:02X}", addr);
            }
        }
        
        info!("I2C scan complete");
    }
    
    /// Test specific I2C addresses that are commonly used by devices
    pub fn test_common_addresses(&mut self) {
        info!("Testing common I2C device addresses...");
        
        // Common I2C device addresses
        let common_addresses: [(u8, &str); 18] = [
            (0x1C, "Accelerometer (LIS3DH)"),
            (0x1D, "Accelerometer (MMA8451)"),
            (0x1E, "Magnetometer (HMC5883L)"),
            (0x3C, "OLED Display (SSD1306)"),
            (0x3D, "OLED Display (SSD1306 alt)"),
            (0x40, "PCA9685 PWM driver"),
            (0x48, "ADS1115 ADC"),
            (0x49, "ADS1115 ADC alt"),
            (0x4A, "ADS1115 ADC alt"),
            (0x4B, "ADS1115 ADC alt"),
            (0x50, "EEPROM"),
            (0x51, "EEPROM alt"),
            (0x57, "EEPROM alt"),
            (0x68, "MPU6050/DS3231 RTC"),
            (0x69, "MPU6050 alt"),
            (0x70, "I2C Multiplexer"),
            (0x76, "BME280 sensor"),
            (0x77, "BME280/BMP280 sensor alt"),
        ];
        
        for (addr, device_name) in &common_addresses {
            // Test on I2C0
            if let Some(ref mut i2c) = self.i2c0 {
                match i2c.write(*addr, &[]) {
                    Ok(_) => info!("I2C0: Device at 0x{:02X} ({})", addr, device_name),
                    Err(_) => {
                        // Try read instead
                        let mut buffer = [0u8; 1];
                        match i2c.read(*addr, &mut buffer) {
                            Ok(_) => info!("I2C0: Device at 0x{:02X} ({}) - read only", addr, device_name),
                            Err(_) => {} // No device
                        }
                    }
                }
            }
            
            // Test on I2C1
            if let Some(ref mut i2c) = self.i2c1 {
                match i2c.write(*addr, &[]) {
                    Ok(_) => info!("I2C1: Device at 0x{:02X} ({})", addr, device_name),
                    Err(_) => {
                        // Try read instead
                        let mut buffer = [0u8; 1];
                        match i2c.read(*addr, &mut buffer) {
                            Ok(_) => info!("I2C1: Device at 0x{:02X} ({}) - read only", addr, device_name),
                            Err(_) => {} // No device
                        }
                    }
                }
            }
        }
        
        info!("Common address test complete");
    }
}

impl Default for I2cScanner {
    fn default() -> Self {
        Self::new()
    }
}
