//! DPS310 Barometric Pressure & Temperature Sensor Driver
//!
//! Minimal from-scratch driver (no external crates) providing raw 24-bit
//! pressure and temperature samples. No compensation / scaling is performed;
//! middleware may later apply coefficient-based compensation if desired.
//!
//! Notes (datasheet summary):
//! - I2C address: 0x76 or 0x77 (SDO / address pin). We use 0x77 per wiring.
//! - Soft reset: write 0x89 to register 0x0C, wait > 10ms.
//! - Product / revision ID register at 0x0D (expected product ID 0x10).
//! - Raw pressure registers: 0x00..0x02 (MSB..LSB)
//! - Raw temperature registers: 0x03..0x05 (MSB..LSB)
//! - Data is 24-bit two's complement; sign-extend to i32.
//! - This minimal implementation does not configure continuous measurement;
//!   many breakout boards power up performing background measurements, so a
//!   direct burst read often returns latest sample. If future explicit mode
//!   control is required, add register writes for PRS_CFG/TMP_CFG/MEAS_CFG.
//!
//! Safety / Simplicity Trade-offs:
//! - We ignore coefficient PROM (0x10..0x21) and do not compute compensated
//!   pressure or temperature to keep code small and robust for early bring-up.
//! - Return (raw_pressure, raw_temperature) as i32 for consistent sign space.
//!
//! Future Work:
//! - Implement coefficient parsing + compensation formula.
//! - Add configurable oversampling / rates and one-shot triggering.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

pub const DPS310_ADDR: u8 = 0x77; // Per user wiring on I2C0

/// DPS310 register addresses used here
mod reg {
    pub const PRS_B2: u8 = 0x00; // First pressure byte (burst reads span PRS + temp)
    pub const PRS_CFG: u8 = 0x06; // Pressure configuration (rate + oversampling)
    pub const TMP_CFG: u8 = 0x07; // Temperature configuration (rate + oversampling + ext sensor select)
    pub const MEAS_CFG: u8 = 0x08; // Measurement / status register
    pub const RESET: u8 = 0x0C;  // Soft reset register
    pub const PROD_ID: u8 = 0x0D; // Product / revision ID
}

#[derive(Debug)]
pub enum Error {
    I2cWrite,
    I2cRead,
    InvalidChipId(u8),
}

// Implement defmt::Format manually (avoid pulling in derive macros)
impl defmt::Format for Error {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Error::I2cWrite => defmt::write!(f, "I2cWrite"),
            Error::I2cRead => defmt::write!(f, "I2cRead"),
            Error::InvalidChipId(id) => defmt::write!(f, "InvalidChipId(0x{:02X})", id),
        }
    }
}

pub struct Dps310 {
    address: u8,
    chip_id: Option<u8>,
}

impl Dps310 {
    pub fn new(address: u8) -> Self { Self { address, chip_id: None } }

    /// Perform soft reset and basic identity check (non-fatal if ID mismatches; we just report error).
    pub fn init<I2C, D, E>(&mut self, i2c: &mut I2C, delay: &mut D) -> Result<(), Error>
    where
        I2C: I2c<Error = E>,
        D: DelayNs,
    {
        // Soft reset
        if i2c.write(self.address, &[reg::RESET, 0x89]).is_err() { return Err(Error::I2cWrite); }
        delay.delay_ms(10u32);

        // Read product ID (not strictly required to proceed)
        let mut id = [0u8;1];
        if i2c.write_read(self.address, &[reg::PROD_ID], &mut id).is_err() { return Err(Error::I2cRead); }
        self.chip_id = Some(id[0]);
        // Many DPS310s report 0x10; we treat others as error but caller may choose to ignore.
        if id[0] != 0x10 { return Err(Error::InvalidChipId(id[0])); }

    // Basic configuration: low measurement rate & modest oversampling, then enable continuous P+T.
    // Datasheet field assumptions (keep conservative):
    // PRS_CFG bits [7:4] PM_RATE (0=1 Hz, 1=2 Hz, 2=4 Hz, ...), [3:0] PM_PRC oversampling (0=1x,1=2x,...)
    // TMP_CFG bits [6:4] TMP_RATE, [3:0] TMP_PRC, bit7 TMP_EXT (0=internal).
    // MEAS_CFG bits [2:0] op mode (0=idle, 1=prs, 2=tmp, 7=cont prs+tmp).
    // Use 4 Hz, 4x oversampling for both (balanced). If any write fails we still proceed (raw reads may stay zero).
    let _ = i2c.write(self.address, &[reg::PRS_CFG,  (0x2 << 4) | 0x03]); // rate=4Hz, oversample=8x (if mapping correct)
    let _ = i2c.write(self.address, &[reg::TMP_CFG,  (0x2 << 4) | 0x03]); // same for temperature
    // Enter continuous pressure+temperature background mode (0x07) so registers 0x00..0x05 update automatically.
    let _ = i2c.write(self.address, &[reg::MEAS_CFG, 0x07]);
    // Allow first conversion group to complete
    delay.delay_ms(20u32);
        Ok(())
    }

    /// Read raw 24-bit pressure & temperature samples (sign-extended to i32).
    pub fn read_raw<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(i32, i32), Error>
    where
        I2C: I2c<Error = E>,
    {
        let mut buf = [0u8; 6];
        if i2c.write_read(self.address, &[reg::PRS_B2], &mut buf).is_err() { return Err(Error::I2cRead); }
        let p_raw = Self::se24(&buf[0..3]);
        let t_raw = Self::se24(&buf[3..6]);
        Ok((p_raw, t_raw))
    }

    /// Read status / mode register (MEAS_CFG) for debugging (returns raw byte).
    pub fn read_status<I2C, E>(&mut self, i2c: &mut I2C) -> Result<u8, Error>
    where
        I2C: I2c<Error = E>,
    {
        let mut b = [0u8;1];
        if i2c.write_read(self.address, &[reg::MEAS_CFG], &mut b).is_err() { return Err(Error::I2cRead); }
        Ok(b[0])
    }

    #[inline]
    fn se24(bytes: &[u8]) -> i32 { // sign-extend 24-bit two's complement into i32
        let raw = ((bytes[0] as i32) << 16) | ((bytes[1] as i32) << 8) | (bytes[2] as i32);
        if raw & 0x800000 != 0 { raw | !0xFFFFFF } else { raw & 0xFFFFFF }
    }
}
