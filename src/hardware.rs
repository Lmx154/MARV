//! # Hardware Abstraction Layer
//!
//! This module defines all hardware pin configurations and provides a unified
//! interface for accessing hardware resources. All pin assignments should be
//! defined here to make hardware changes easy to manage.

use rp235x_hal as hal;

/// Hardware configuration structure that holds all pin definitions
/// and hardware-specific constants
pub struct HardwareConfig {
    /// LED pin (typically GPIO25 on Pico boards)
    pub led_pin: u8,
    
    /// UART0 pins for serial communication
    pub uart0_tx_pin: u8,
    pub uart0_rx_pin: u8,
    
    /// I2C pins (if needed for sensors)
    pub i2c_sda_pin: u8,
    pub i2c_scl_pin: u8,
    
    /// SPI pins (if needed)
    pub spi_mosi_pin: u8,
    pub spi_miso_pin: u8,
    pub spi_sck_pin: u8,
    pub spi_cs_pin: u8,
    
    /// ADC pins for analog inputs
    pub adc_pin0: u8,
    pub adc_pin1: u8,
    pub adc_pin2: u8,
    
    /// External high-speed crystal frequency
    pub xtal_freq_hz: u32,
    
    /// System clock frequency
    pub system_clock_hz: u32,
}

impl Default for HardwareConfig {
    fn default() -> Self {
        Self {
            // Standard Raspberry Pi Pico 2 pin assignments
            led_pin: 25,           // Built-in LED
            
            // UART0 pins
            uart0_tx_pin: 0,       // GP0 - UART0 TX
            uart0_rx_pin: 1,       // GP1 - UART0 RX
            
            // I2C pins (I2C0)
            i2c_sda_pin: 4,        // GP4 - I2C0 SDA
            i2c_scl_pin: 5,        // GP5 - I2C0 SCL
            
            // SPI pins (SPI0)
            spi_mosi_pin: 16,      // GP16 - SPI0 TX (MOSI)
            spi_miso_pin: 17,      // GP17 - SPI0 RX (MISO)
            spi_sck_pin: 18,       // GP18 - SPI0 SCK
            spi_cs_pin: 19,        // GP19 - SPI0 CSn
            
            // ADC pins
            adc_pin0: 26,          // GP26 - ADC0
            adc_pin1: 27,          // GP27 - ADC1
            adc_pin2: 28,          // GP28 - ADC2
            
            // Clock frequencies
            xtal_freq_hz: 12_000_000,     // 12 MHz external crystal
            system_clock_hz: 125_000_000, // 125 MHz system clock
        }
    }
}

/// Global hardware configuration instance
pub static HARDWARE: HardwareConfig = HardwareConfig {
    // You can override specific pins here if needed
    // For now, using all defaults
    ..HardwareConfig {
        led_pin: 25,
        uart0_tx_pin: 0,
        uart0_rx_pin: 1,
        i2c_sda_pin: 4,
        i2c_scl_pin: 5,
        spi_mosi_pin: 16,
        spi_miso_pin: 17,
        spi_sck_pin: 18,
        spi_cs_pin: 19,
        adc_pin0: 26,
        adc_pin1: 27,
        adc_pin2: 28,
        xtal_freq_hz: 12_000_000,
        system_clock_hz: 125_000_000,
    }
};

/// Helper functions to get specific pin configurations
impl HardwareConfig {
    /// Get the LED pin number
    pub const fn led_pin(&self) -> u8 {
        self.led_pin
    }
    
    /// Get UART0 pin configuration as a tuple (TX, RX)
    pub const fn uart0_pins(&self) -> (u8, u8) {
        (self.uart0_tx_pin, self.uart0_rx_pin)
    }
    
    /// Get I2C pin configuration as a tuple (SDA, SCL)
    pub const fn i2c_pins(&self) -> (u8, u8) {
        (self.i2c_sda_pin, self.i2c_scl_pin)
    }
    
    /// Get SPI pin configuration as a tuple (MOSI, MISO, SCK, CS)
    pub const fn spi_pins(&self) -> (u8, u8, u8, u8) {
        (self.spi_mosi_pin, self.spi_miso_pin, self.spi_sck_pin, self.spi_cs_pin)
    }
    
    /// Get ADC pin configuration as a tuple (ADC0, ADC1, ADC2)
    pub const fn adc_pins(&self) -> (u8, u8, u8) {
        (self.adc_pin0, self.adc_pin1, self.adc_pin2)
    }
    
    /// Get external crystal frequency
    pub const fn xtal_frequency(&self) -> u32 {
        self.xtal_freq_hz
    }
    
    /// Get system clock frequency
    pub const fn system_clock_frequency(&self) -> u32 {
        self.system_clock_hz
    }
}

/// Pin configuration macros for easy access
/// These macros help reduce boilerplate when configuring pins

/// Macro to get a specific GPIO pin from the pins structure
#[macro_export]
macro_rules! get_pin {
    ($pins:expr, $pin_num:expr) => {
        match $pin_num {
            0 => $pins.gpio0,
            1 => $pins.gpio1,
            2 => $pins.gpio2,
            3 => $pins.gpio3,
            4 => $pins.gpio4,
            5 => $pins.gpio5,
            6 => $pins.gpio6,
            7 => $pins.gpio7,
            8 => $pins.gpio8,
            9 => $pins.gpio9,
            10 => $pins.gpio10,
            11 => $pins.gpio11,
            12 => $pins.gpio12,
            13 => $pins.gpio13,
            14 => $pins.gpio14,
            15 => $pins.gpio15,
            16 => $pins.gpio16,
            17 => $pins.gpio17,
            18 => $pins.gpio18,
            19 => $pins.gpio19,
            20 => $pins.gpio20,
            21 => $pins.gpio21,
            22 => $pins.gpio22,
            23 => $pins.gpio23,
            24 => $pins.gpio24,
            25 => $pins.gpio25,
            26 => $pins.gpio26,
            27 => $pins.gpio27,
            28 => $pins.gpio28,
            _ => panic!("Invalid GPIO pin number: {}", $pin_num),
        }
    };
}

/// Hardware-specific constants
pub mod constants {
    /// Default UART baud rate
    pub const DEFAULT_UART_BAUD: u32 = 9600;
    
    /// Default I2C frequency (100 kHz)
    pub const DEFAULT_I2C_FREQ: u32 = 100_000;
    
    /// Default SPI frequency (1 MHz)
    pub const DEFAULT_SPI_FREQ: u32 = 1_000_000;
    
    /// LED blink intervals (in loop iterations)
    pub const LED_BLINK_INTERVAL: u32 = 1000;
    
    /// Status print interval (in loop iterations)
    pub const STATUS_PRINT_INTERVAL: u32 = 10_000;
    
    /// Time update interval (in loop iterations)
    pub const TIME_UPDATE_INTERVAL: u32 = 10_000;
    
    /// Main loop delay in microseconds
    pub const MAIN_LOOP_DELAY_US: u32 = 100;
}

/// Board-specific pin mappings for common peripherals
pub mod board {
    /// Raspberry Pi Pico 2 specific pin definitions
    pub mod pico2 {
        /// Built-in LED pin
        pub const LED: u8 = 25;
        
        /// UART pins
        pub const UART0_TX: u8 = 0;
        pub const UART0_RX: u8 = 1;
        pub const UART1_TX: u8 = 8;
        pub const UART1_RX: u8 = 9;
        
        /// I2C pins
        pub const I2C0_SDA: u8 = 4;
        pub const I2C0_SCL: u8 = 5;
        pub const I2C1_SDA: u8 = 6;
        pub const I2C1_SCL: u8 = 7;
        
        /// SPI pins
        pub const SPI0_MOSI: u8 = 16;
        pub const SPI0_MISO: u8 = 17;
        pub const SPI0_SCK: u8 = 18;
        pub const SPI0_CS: u8 = 19;
        
        /// ADC pins
        pub const ADC0: u8 = 26;
        pub const ADC1: u8 = 27;
        pub const ADC2: u8 = 28;
        pub const ADC3: u8 = 29; // Also connected to VSYS/3
    }
}

// Example usage documentation
// 
// ```rust
// use crate::hardware::{HARDWARE, constants};
// 
// // Get LED pin number
// let led_pin_num = HARDWARE.led_pin();
// 
// // Get UART pins
// let (tx_pin, rx_pin) = HARDWARE.uart0_pins();
// 
// // Use constants
// let baud_rate = constants::DEFAULT_UART_BAUD;
// ```
