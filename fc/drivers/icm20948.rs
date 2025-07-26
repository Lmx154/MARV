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
//! - Connect via I2C0 (GP20 SDA, GP21 SCL as per hardware.md correction).
//! - In main code, read raw data and format as text for UART output (e.g., "Accel: x y z Gyro: x y z").
//! - Delays in init use the provided Delay trait implementation.
//! - Burst read for efficiency: Read 12 bytes from ACCEL_XOUT_H (0x2D) to GYRO_ZOUT_L (0x38).
//!
//! Error Handling: Basic I2C errors and chip ID verification.

use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;
use rp235x_hal as hal;
use hal::gpio::bank0;
use hal::gpio::{Pin, FunctionI2C, PullUp};
use super::sensor_trait::SensorDriver;

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

    // Magnetometer-related registers
    pub const I2C_MST_CTRL: u8 = 0x01;       // Bank 3
    pub const I2C_MST_DELAY_CTRL: u8 = 0x02; // Bank 3
    pub const I2C_SLV0_ADDR: u8 = 0x03;      // Bank 3
    pub const I2C_SLV0_REG: u8 = 0x04;       // Bank 3
    pub const I2C_SLV0_CTRL: u8 = 0x05;      // Bank 3
    pub const I2C_SLV0_DO: u8 = 0x06;        // Bank 3
    pub const I2C_SLV4_ADDR: u8 = 0x13;      // Bank 3
    pub const I2C_SLV4_REG: u8 = 0x14;       // Bank 3
    pub const I2C_SLV4_DO: u8 = 0x15;        // Bank 3
    pub const I2C_SLV4_CTRL: u8 = 0x16;      // Bank 3
    pub const I2C_SLV4_DI: u8 = 0x17;        // Bank 3
    pub const EXT_SENS_DATA_00: u8 = 0x3B;   // Bank 0, for mag data
}

/// AK09916 magnetometer constants
mod ak09916 {
    pub const ADDR: u8 = 0x0C;
    pub const WHO_AM_I: u8 = 0x01;           // Expected: 0x09
    pub const ST1: u8 = 0x10;
    pub const HXL: u8 = 0x11;                // Mag data start
    pub const ST2: u8 = 0x18;                // End after HZ H
    pub const CNTL2: u8 = 0x31;              // Mode control
    pub const MODE_100HZ: u8 = 0x08;         // Continuous 100 Hz
}

/// Raw IMU data structure
#[derive(Debug, Clone, Copy, Default)]
pub struct RawImu {
    pub accel: [i16; 3],  // [x, y, z] in raw ADC counts (two's complement)
    pub gyro: [i16; 3],   // [x, y, z] in raw ADC counts (two's complement)
    pub mag: [i16; 3],    // [x, y, z] in raw ADC counts (two's complement)
}

/// Driver errors
#[derive(Debug)]
pub enum Error {
    I2cError,
    InvalidChipId,
    WriteFailed,
    ReadFailed,
    MagInitFailed,
    MagDataNotReady,
    Timeout,
}

/// ICM-20948 driver
pub struct Icm20948<DELAY> {
    pub delay: DELAY,
    address: u8,
}

impl<DELAY> Icm20948<DELAY>
where
    DELAY: DelayNs,
{
    /// Create a new driver instance
    pub fn new(delay: DELAY, address: u8) -> Self {
        Self { delay, address }
    }

    /// Initialize the sensor including magnetometer
    pub fn init<I2C>(&mut self, i2c: &mut I2C) -> Result<(), Error>
    where
        I2C: I2c,
    {
        // Reset device
        self.write_register(i2c, registers::PWR_MGMT_1, 0x80).map_err(|_| Error::WriteFailed)?;
        self.delay.delay_ms(100u32);

        // Verify chip ID
        let chip_id = self.read_register(i2c, registers::WHO_AM_I).map_err(|_| Error::ReadFailed)?;
        if chip_id != 0xEA {
            return Err(Error::InvalidChipId);
        }

        // Wake from sleep and set clock to auto-select
        self.write_register(i2c, registers::PWR_MGMT_1, 0x01).map_err(|_| Error::WriteFailed)?;

        // Enable accel and gyro (all axes)
        self.write_register(i2c, registers::PWR_MGMT_2, 0x00).map_err(|_| Error::WriteFailed)?;

        // Switch to Bank 2 for config
        self.write_register(i2c, registers::REG_BANK_SEL, 0x20).map_err(|_| Error::WriteFailed)?;

        // Configure gyro
        self.write_register(i2c, registers::GYRO_CONFIG_1, 0x01).map_err(|_| Error::WriteFailed)?;

        // Configure accel
        self.write_register(i2c, registers::ACCEL_CONFIG, 0x01).map_err(|_| Error::WriteFailed)?;

        // Switch to Bank 0
        self.write_register(i2c, registers::REG_BANK_SEL, 0x00).map_err(|_| Error::WriteFailed)?;

        // Enable I2C master
        self.write_register(i2c, registers::USER_CTRL, 0x20).map_err(|_| Error::WriteFailed)?;

        // Reset I2C master
        self.write_register(i2c, registers::USER_CTRL, 0x20 | 0x80).map_err(|_| Error::WriteFailed)?;
        self.delay.delay_ms(10u32);
        self.write_register(i2c, registers::USER_CTRL, 0x20).map_err(|_| Error::WriteFailed)?;

        // Switch to Bank 3
        self.write_register(i2c, registers::REG_BANK_SEL, 0x30).map_err(|_| Error::WriteFailed)?;

        // Set I2C master clock
        self.write_register(i2c, registers::I2C_MST_CTRL, 0x07).map_err(|_| Error::WriteFailed)?;

        // Set mag to 100 Hz mode
        let _ = self.slv4_write(i2c, ak09916::ADDR, ak09916::CNTL2, ak09916::MODE_100HZ);

        // Config SLV0 for mag data read
        let _ = self.write_register(i2c, registers::I2C_SLV0_ADDR, ak09916::ADDR | 0x80);
        let _ = self.write_register(i2c, registers::I2C_SLV0_REG, ak09916::ST1);
        let _ = self.write_register(i2c, registers::I2C_SLV0_CTRL, 0x80 | 0x09);

        // Switch to Bank 0
        self.write_register(i2c, registers::REG_BANK_SEL, 0x00).map_err(|_| Error::WriteFailed)?;

        // Verify mag ID (optional, continue if fails)
        self.write_register(i2c, registers::REG_BANK_SEL, 0x30).map_err(|_| Error::WriteFailed)?;
        let mag_id = self.slv4_read(i2c, ak09916::ADDR, ak09916::WHO_AM_I);
        self.write_register(i2c, registers::REG_BANK_SEL, 0x00).map_err(|_| Error::WriteFailed)?;
        if let Ok(id) = mag_id {
            if id != 0x09 {
                // Log but continue
            }
        }

        Ok(())
    }

    fn slv4_write<I2C>(&mut self, i2c: &mut I2C, addr: u8, reg: u8, value: u8) -> Result<(), Error>
    where
        I2C: I2c,
    {
        self.write_register(i2c, registers::I2C_SLV4_ADDR, addr).map_err(|_| Error::WriteFailed)?;
        self.write_register(i2c, registers::I2C_SLV4_REG, reg).map_err(|_| Error::WriteFailed)?;
        self.write_register(i2c, registers::I2C_SLV4_DO, value).map_err(|_| Error::WriteFailed)?;
        self.write_register(i2c, registers::I2C_SLV4_CTRL, 0x80).map_err(|_| Error::WriteFailed)?;

        let mut timeout = 100u32;
        while timeout > 0 {
            self.delay.delay_ms(1u32);
            let ctrl = self.read_register(i2c, registers::I2C_SLV4_CTRL).map_err(|_| Error::ReadFailed)?;
            if (ctrl & 0x80) == 0 {
                if (ctrl & 0x01) != 0 {
                    return Err(Error::I2cError); // NACK
                }
                return Ok(());
            }
            timeout -= 1;
        }
        Err(Error::Timeout)
    }

    fn slv4_read<I2C>(&mut self, i2c: &mut I2C, addr: u8, reg: u8) -> Result<u8, Error>
    where
        I2C: I2c,
    {
        self.write_register(i2c, registers::I2C_SLV4_ADDR, addr | 0x80).map_err(|_| Error::WriteFailed)?;
        self.write_register(i2c, registers::I2C_SLV4_REG, reg).map_err(|_| Error::WriteFailed)?;
        self.write_register(i2c, registers::I2C_SLV4_CTRL, 0x80).map_err(|_| Error::WriteFailed)?;

        let mut timeout = 100u32;
        while timeout > 0 {
            self.delay.delay_ms(1u32);
            let ctrl = self.read_register(i2c, registers::I2C_SLV4_CTRL).map_err(|_| Error::ReadFailed)?;
            if (ctrl & 0x80) == 0 {
                if (ctrl & 0x01) != 0 {
                    return Err(Error::I2cError); // NACK
                }
                break;
            }
            timeout -= 1;
        }
        if timeout == 0 {
            return Err(Error::Timeout);
        }

        self.read_register(i2c, registers::I2C_SLV4_DI).map_err(|_| Error::ReadFailed)
    }

    /// Read raw accelerometer, gyroscope, and magnetometer data
    pub fn read_raw<I2C>(&mut self, i2c: &mut I2C) -> Result<RawImu, Error>
    where
        I2C: I2c,
    {
        let mut buffer = [0u8; 12];
        if i2c.write_read(self.address, &[registers::ACCEL_XOUT_H], &mut buffer).is_err() {
            return Err(Error::ReadFailed);
        }

        let mut raw = RawImu {
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
            mag: [0; 3],
        };

        let mut mag_buffer = [0u8; 9];
        if i2c.write_read(self.address, &[registers::EXT_SENS_DATA_00], &mut mag_buffer).is_ok() {
            let st1 = mag_buffer[0];
            if st1 & 0x01 != 0 {
                raw.mag = [
                    i16::from_le_bytes([mag_buffer[1], mag_buffer[2]]),
                    i16::from_le_bytes([mag_buffer[3], mag_buffer[4]]),
                    i16::from_le_bytes([mag_buffer[5], mag_buffer[6]]),
                ];
                let st2 = mag_buffer[8];
                if st2 & 0x08 != 0 {
                    // HOFL set, overflow - handle if needed
                }
            }
        }

        Ok(raw)
    }

    fn write_register<I2C>(&mut self, i2c: &mut I2C, reg: u8, value: u8) -> Result<(), Error>
    where
        I2C: I2c,
    {
        i2c.write(self.address, &[reg, value]).map_err(|_| Error::I2cError)
    }

    fn read_register<I2C>(&mut self, i2c: &mut I2C, reg: u8) -> Result<u8, Error>
    where
        I2C: I2c,
    {
        let mut buffer = [0u8; 1];
        i2c.write_read(self.address, &[reg], &mut buffer).map_err(|_| Error::I2cError)?;
        Ok(buffer[0])
    }
}

impl<DELAY> SensorDriver for Icm20948<DELAY>
where
    DELAY: DelayNs,
{
    type Bus = hal::i2c::I2C<hal::pac::I2C0, (Pin<bank0::Gpio20, FunctionI2C, PullUp>, Pin<bank0::Gpio21, FunctionI2C, PullUp>)>;
    type RawData = RawImu;
    type ParsedData = RawImu;
    type Error = Error;

    fn read_raw(&mut self, bus: &mut Self::Bus) -> Result<Self::RawData, Self::Error> {
        self.read_raw(bus)
    }

    fn parse(&self, raw: Self::RawData) -> Result<Self::ParsedData, Self::Error> {
        Ok(raw)
    }
}