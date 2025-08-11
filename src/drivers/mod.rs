//! Drivers module
//!
//! Contains reusable peripheral drivers (e.g., RGB LED).

pub mod rgb_led;
pub mod buzzer;
pub mod gps;
pub mod lis3mdl;
pub mod ms5611;
pub mod icm20948;
pub mod mcp2515;
pub mod sensor_trait;
pub mod bmi088; // BMI088 driver (hardware SPI)
pub mod dps310; // DPS310 barometer driver
