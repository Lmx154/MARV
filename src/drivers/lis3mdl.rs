//! LIS3MDL Magnetometer Driver
//!
//! This driver provides raw data as i16 values per axis for the LIS3MDL 3-axis magnetometer.
//! Based on the LIS3MDL datasheet:
//! - I2C addresses: 0x1C (SDO/SA1 low) or 0x1E (SDO/SA1 high)
//! - Initialization: Verify chip ID, configure performance mode, output data rate, full scale, and continuous conversion mode.
//! - Raw data: 16-bit two's complement, LSB at lower address (little-endian by default).
//! - Sensitivity (for reference, not implemented): ±4 gauss full scale = 6842 LSB/gauss.
//!
//! Usage Notes:
//! - Connect via the same I2C bus as the ICM20948.
//! - In the main code, read raw data and append to UART output as "; MAG1: x y z".
//! - Delays use the provided Delay trait implementation.
//! - Burst read for efficiency: Read 6 bytes from OUT_X_L (0x28) to OUT_Z_H (0x2D).

use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;

pub const LIS3MDL_ADDR: u8 = 0x1C; // SDO/SA1 low, as detected in hardware.

/// Register addresses
pub mod registers {
    pub const WHO_AM_I: u8 = 0x0F;      // Expected: 0x3D
    pub const CTRL_REG1: u8 = 0x20;
    pub const CTRL_REG2: u8 = 0x21;
    pub const CTRL_REG3: u8 = 0x22;
    pub const CTRL_REG4: u8 = 0x23;
    pub const OUT_X_L: u8 = 0x28;
    // Sequential: OUT_X_H=0x29, Y_L=0x2A, Y_H=0x2B, Z_L=0x2C, Z_H=0x2D
}

/// Driver errors
#[derive(Debug)]
pub enum Error {
    I2cError,
    InvalidChipId,
    WriteFailed,
    ReadFailed,
    Timeout,
}

/// LIS3MDL driver
pub struct Lis3mdl<DELAY> {
    pub delay: DELAY,
    address: u8,
}

impl<DELAY> Lis3mdl<DELAY>
where
    DELAY: DelayNs,
{
    /// Create a new driver instance
    pub fn new(delay: DELAY, address: u8) -> Self {
        Self { delay, address }
    }

    /// Initialize the sensor
    pub fn init<I2C>(&mut self, i2c: &mut I2C) -> Result<(), Error>
    where
        I2C: I2c,
    {
        // Verify chip ID
        let chip_id = self.read_register(i2c, registers::WHO_AM_I).map_err(|_| Error::ReadFailed)?;
        if chip_id != 0x3D {
            return Err(Error::InvalidChipId);
        }

        // Configure full scale ±4 gauss
        self.write_register(i2c, registers::CTRL_REG2, 0x00).map_err(|_| Error::WriteFailed)?;
        self.delay.delay_ms(10u32);

        // Configure UHP mode, 10 Hz ODR, no temp
        self.write_register(i2c, registers::CTRL_REG1, 0x70).map_err(|_| Error::WriteFailed)?;
        self.delay.delay_ms(10u32);

        // Continuous conversion mode
        self.write_register(i2c, registers::CTRL_REG3, 0x00).map_err(|_| Error::WriteFailed)?;
        self.delay.delay_ms(10u32);

        // UHP for Z-axis, little-endian
        self.write_register(i2c, registers::CTRL_REG4, 0x0C).map_err(|_| Error::WriteFailed)?;
        self.delay.delay_ms(10u32);

        Ok(())
    }

    /// Read raw magnetometer data
    pub fn read_raw<I2C>(&mut self, i2c: &mut I2C) -> Result<[i16; 3], Error>
    where
        I2C: I2c,
    {
        let mut buffer = [0u8; 6];
        if i2c.write_read(self.address, &[registers::OUT_X_L], &mut buffer).is_err() {
            return Err(Error::ReadFailed);
        }

        let raw_mag = [
            i16::from_le_bytes([buffer[0], buffer[1]]),
            i16::from_le_bytes([buffer[2], buffer[3]]),
            i16::from_le_bytes([buffer[4], buffer[5]]),
        ];

        Ok(raw_mag)
    }

    pub fn write_register<I2C>(&mut self, i2c: &mut I2C, reg: u8, value: u8) -> Result<(), Error>
    where
        I2C: I2c,
    {
        i2c.write(self.address, &[reg, value]).map_err(|_| Error::I2cError)
    }

    pub fn read_register<I2C>(&mut self, i2c: &mut I2C, reg: u8) -> Result<u8, Error>
    where
        I2C: I2c,
    {
        let mut buffer = [0u8; 1];
        i2c.write_read(self.address, &[reg], &mut buffer).map_err(|_| Error::I2cError)?;
        Ok(buffer[0])
    }
}