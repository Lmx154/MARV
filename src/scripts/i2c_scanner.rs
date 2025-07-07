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
        let (hw_i2c0_sda, hw_i2c0_scl) = Self::get_i2c0_pins();
        let (hw_i2c1_sda, hw_i2c1_scl) = Self::get_i2c1_pins();
        
        // Hardcoded pins in this implementation
        const HARDCODED_I2C0_SDA: u8 = 4;
        const HARDCODED_I2C0_SCL: u8 = 5;
        const HARDCODED_I2C1_SDA: u8 = 6;
        const HARDCODED_I2C1_SCL: u8 = 7;
        
        if hw_i2c0_sda != HARDCODED_I2C0_SDA || hw_i2c0_scl != HARDCODED_I2C0_SCL {
            error!("I2C0 pin mismatch! Hardware config: GP{}/GP{}, Hardcoded: GP{}/GP{}", 
                   hw_i2c0_sda, hw_i2c0_scl, HARDCODED_I2C0_SDA, HARDCODED_I2C0_SCL);
            return Err("I2C0 pin configuration mismatch");
        }
        
        if hw_i2c1_sda != HARDCODED_I2C1_SDA || hw_i2c1_scl != HARDCODED_I2C1_SCL {
            error!("I2C1 pin mismatch! Hardware config: GP{}/GP{}, Hardcoded: GP{}/GP{}", 
                   hw_i2c1_sda, hw_i2c1_scl, HARDCODED_I2C1_SDA, HARDCODED_I2C1_SCL);
            return Err("I2C1 pin configuration mismatch");
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
            
            // Scan addresses 0x08 to 0x77 (valid 7-bit I2C addresses)
            for addr in 0x08..=0x77 {
                // Try to write 0 bytes to the address to check if device responds
                match i2c.write(addr, &[]) {
                    Ok(_) => {
                        info!("Device found at address: 0x{:02X}", addr);
                        let _ = found_addresses.push(addr);
                    }
                    Err(_) => {
                        // No device at this address, continue scanning
                    }
                }
            }
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
            
            // Scan addresses 0x08 to 0x77 (valid 7-bit I2C addresses)
            for addr in 0x08..=0x77 {
                // Try to write 0 bytes to the address to check if device responds
                match i2c.write(addr, &[]) {
                    Ok(_) => {
                        info!("Device found at address: 0x{:02X}", addr);
                        let _ = found_addresses.push(addr);
                    }
                    Err(_) => {
                        // No device at this address, continue scanning
                    }
                }
            }
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
}

impl Default for I2cScanner {
    fn default() -> Self {
        Self::new()
    }
}
