//! ICM-20948 IMU Driver
//!
//! Simple driver for the TDK InvenSense ICM-20948 9-axis IMU (accelerometer and gyroscope only).
//! This driver focuses on reading raw accelerometer and gyroscope data over I2C.
//! It does not handle the internal magnetometer (AK09916); use a separate driver for external magnetometers like BMM350.
//!
//! The driver provides raw data as i16 values per axis, suitable for further processing in filters (e.g., low-pass or Kalman).
//! Scaling to physical units (g for accel, dps for gyro) can be done externally if needed.
//!
//! Based on the ICM-20948 datasheet (DS-000189 v1.3):
//! - I2C addresses: 0x68 (default, AD0=0) or 0x69 (AD0=1)
//! - Initialization: Reset, set clock, enable sensors, configure default ranges (±2g accel, ±250 dps gyro)
//! - Raw data: 16-bit two's complement, MSB first
//! - Sensitivity (for scaling reference, not implemented here): 16384 LSB/g (accel ±2g), 131 LSB/dps (gyro ±250 dps)
//!
//! Usage Notes:
//! - Connect via I2C0 (GP22 SDA, GP21 SCL as per hardware.md).
//! - In main code, read raw data and format as text for UART output (e.g., "Accel: x y z Gyro: x y z").
//! - Delays in init use the provided Delay trait implementation.
//! - Burst read for efficiency: Read 12 bytes from ACCEL_XOUT_H (0x2D) to GYRO_ZOUT_L (0x38).
//!
//! Error Handling: Basic I2C errors and chip ID verification.

use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;

/// ICM-20948 I2C addresses
pub const ICM20948_ADDR_AD0_LOW: u8 = 0x68; // Default (AD0 = 0)
pub const ICM20948_ADDR_AD0_HIGH: u8 = 0x69; // AD0 = 1

/// Register addresses (User Bank 0 unless specified)
mod registers {
    pub const WHO_AM_I: u8 = 0x00;           // Expected: 0xEA
    pub const USER_CTRL: u8 = 0x03;          // For FIFO/DMP control if needed
    pub const PWR_MGMT_1: u8 = 0x06;
    pub const PWR_MGMT_2: u8 = 0x07; 
    pub const REG_BANK_SEL: u8 = 0x7F;       // Bank select register

    // Bank 2 registers
    pub const GYRO_CONFIG_1: u8 = 0x01;      // Bank 2
    pub const ACCEL_CONFIG: u8 = 0x14;       // Bank 2

    // Raw data registers (Bank 0)
    pub const ACCEL_XOUT_H: u8 = 0x2D;
    // Sequential: ACCEL_XOUT_L=0x2E, Y_H=0x2F, Y_L=0x30, Z_H=0x31, Z_L=0x32
    // GYRO_XOUT_H=0x33, ... Z_L=0x38
}

/// Raw IMU data structure
#[derive(Debug, Clone, Copy, Default)]
pub struct RawImu {
    pub accel: [i16; 3],  // [x, y, z] in raw ADC counts (two's complement)
    pub gyro: [i16; 3],   // [x, y, z] in raw ADC counts (two's complement)
}

/// Driver errors
#[derive(Debug)]
pub enum Error {
    I2cError,
    InvalidChipId,
    WriteFailed,
    ReadFailed,
}

/// ICM-20948 driver
pub struct Icm20948<I2C, DELAY> {
    i2c: I2C,
    address: u8,
    delay: DELAY,
}

impl<I2C, DELAY> Icm20948<I2C, DELAY>
where
    I2C: I2c,
    DELAY: DelayNs,
{
    /// Create a new driver instance
    pub fn new(i2c: I2C, address: u8, delay: DELAY) -> Self {
        Self { i2c, address, delay }
    }

    /// Initialize the sensor
    /// - Performs reset and waits 100ms
    /// - Sets auto clock source
    /// - Enables accel and gyro
    /// - Configures default ranges: ±2g accel, ±250 dps gyro
    /// - Verifies chip ID (WHO_AM_I = 0xEA)
    pub fn init(&mut self) -> Result<(), Error> {
        // Reset device
        self.write_register(registers::PWR_MGMT_1, 0x80).map_err(|_| Error::WriteFailed)?;
        self.delay.delay_ms(100u32);

        // Verify chip ID
        let chip_id = self.read_register(registers::WHO_AM_I).map_err(|_| Error::ReadFailed)?;
        if chip_id != 0xEA {
            return Err(Error::InvalidChipId);
        }

        // Wake from sleep and set clock to auto-select
        self.write_register(registers::PWR_MGMT_1, 0x01).map_err(|_| Error::WriteFailed)?;

        // Enable accel and gyro (all axes)
        self.write_register(registers::PWR_MGMT_2, 0x00).map_err(|_| Error::WriteFailed)?;

        // Switch to Bank 2 for config
        self.write_register(registers::REG_BANK_SEL, 0x20).map_err(|_| Error::WriteFailed)?;

        // Configure gyro: ±250 dps (GYRO_FS_SEL=00), FCHOICE=1 (DLPF enabled)
        self.write_register(registers::GYRO_CONFIG_1, 0x01).map_err(|_| Error::WriteFailed)?; // Bit 0=1 for FCHOICE, FS_SEL=00

        // Configure accel: ±2g (ACCEL_FS_SEL=00), FCHOICE=1
        self.write_register(registers::ACCEL_CONFIG, 0x01).map_err(|_| Error::WriteFailed)?; // Bit 0=1, FS_SEL=00

        // Switch back to Bank 0
        self.write_register(registers::REG_BANK_SEL, 0x00).map_err(|_| Error::WriteFailed)?;

        Ok(())
    }

    /// Read raw accelerometer and gyroscope data
    /// - Performs a burst read of 12 bytes starting from ACCEL_XOUT_H
    /// - Parses into i16 values (MSB << 8 | LSB, signed two's complement)
    pub fn read_raw(&mut self) -> Result<RawImu, Error> {
        let mut buffer = [0u8; 12];
        self.i2c
            .write_read(self.address, &[registers::ACCEL_XOUT_H], &mut buffer)
            .map_err(|_| Error::ReadFailed)?;

        let raw = RawImu {
            accel: [
                i16::from_be_bytes([buffer[0], buffer[1]]),
                i16::from_be_bytes([buffer[2], buffer[3]]),
                i16::from_be_bytes([buffer[4], buffer[5]]),
            ],
            gyro: [
                i16::from_be_bytes([buffer[6], buffer[7]]),
                i16::from_be_bytes([buffer[8], buffer[9]]),
                i16::from_be_bytes([buffer[10], buffer[11]]),
            ],
        };

        Ok(raw)
    }

    /// Write a single register
    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), ()> {
        self.i2c.write(self.address, &[reg, value]).map_err(|_| ())
    }

    /// Read a single register
    fn read_register(&mut self, reg: u8) -> Result<u8, ()> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(self.address, &[reg], &mut buffer).map_err(|_| ())?;
        Ok(buffer[0])
    }
}