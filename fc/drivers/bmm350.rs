
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

/// Key registers from datasheet Section 8.
const REG_CHIP_ID: u8 = 0x00;
const REG_ERR_REG: u8 = 0x02;
const REG_PAD_CTRL: u8 = 0x03;
const REG_PMU_CMD_AGGR_SET: u8 = 0x04;
const REG_PMU_CMD_AXIS_EN: u8 = 0x05;
const REG_PMU_CMD: u8 = 0x06;
const REG_PMU_CMD_STATUS_0: u8 = 0x07;
const REG_I2C_WDT_SET: u8 = 0x0A;
const REG_INT_STATUS: u8 = 0x30;
const REG_MAG_DATA_START: u8 = 0x31; // Burst read start for mag data.
const REG_SENSORTIME_XLSB: u8 = 0x3D; // Sensortime for data update check.
const REG_OTP_CMD_REG: u8 = 0x50;
const REG_OTP_DATA_MSB_REG: u8 = 0x52;
const REG_OTP_DATA_LSB_REG: u8 = 0x53;
const REG_OTP_STATUS_REG: u8 = 0x55;
const REG_CMD: u8 = 0x7E;

/// Expected CHIP_ID value.
const CHIP_ID_EXPECTED: u8 = 0x33;

/// Raw magnetometer data struct.
#[derive(Debug, Copy, Clone)]
pub struct RawMag {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

/// Custom error types.
#[derive(Debug, Copy, Clone)]
pub enum Bmm350Error<E> {
    I2cError(E),
    InvalidChipId(u8),
    PmuBusy,
    PmuError(u8),
    OtpBusy,
    ConfigMismatch { reg: u8, expected: u8, actual: u8 },
    DataNotReady,
}

/// BMM350 driver struct.
pub struct Bmm350 {
    addr: u8,
}

impl Bmm350 {
    /// Create a new driver instance.
    pub fn new(addr: u8) -> Self {
        Self { addr }
    }

    /// Initialize the sensor following datasheet and API sequence.
    pub fn init<I2C, E, D, W>(
        &self,
        i2c: &mut I2C,
        delay: &mut D,
        uart: &mut W,
    ) -> Result<(), Bmm350Error<E>>
    where
        I2C: I2c<Error = E>,
        D: DelayNs,
        W: core::fmt::Write,
    {
        // Step 1: Power-on delay.
        delay.delay_ms(200u32);
        writeln!(uart, "BMM350 power-on delay completed").ok();

        // Step 2: Verify CHIP_ID with retries at both addresses.
        let mut chip_id = [0u8];
        let addresses = [self.addr, if self.addr == 0x14 { 0x15 } else { 0x14 }];
        let mut success = false;
        for &addr in addresses.iter() {
            let mut retry = 5u8;
            while retry > 0 {
                match self.read_reg_at(i2c, addr, REG_CHIP_ID, &mut chip_id) {
                    Ok(()) if chip_id[0] == CHIP_ID_EXPECTED => {
                        writeln!(uart, "BMM350 CHIP_ID verified at 0x{:02X}: 0x{:02X}", addr, chip_id[0]).ok();
                        success = true;
                        break;
                    }
                    Ok(()) => {
                        writeln!(uart, "BMM350 CHIP_ID read at 0x{:02X} failed: got 0x{:02X}, retrying ({}/5)", addr, chip_id[0], 6 - retry).ok();
                    }
                    Err(e) => {
                        writeln!(uart, "BMM350 CHIP_ID read at 0x{:02X} failed with I2C error: {:?}", addr, e).ok();
                    }
                }
                retry -= 1;
                if retry > 0 {
                    // Attempt soft reset.
                    let _ = self.write_reg_at(i2c, addr, REG_CMD, 0xB6);
                    delay.delay_ms(200u32);
                    let _ = self.write_reg_at(i2c, addr, REG_CMD, 0x00);
                    delay.delay_ms(200u32);
                }
                delay.delay_ms(100u32);
            }
            if success {
                self.addr = addr; // Update address if successful.
                break;
            }
        }
        if !success {
            writeln!(uart, "BMM350 CHIP_ID failed after retries").ok();
            return Err(Bmm350Error::InvalidChipId(chip_id[0]));
        }

        // Step 3: Soft reset.
        self.write_reg(i2c, REG_CMD, 0xB6)?;
        delay.delay_ms(200u32);
        self.write_reg(i2c, REG_CMD, 0x00)?;
        delay.delay_ms(200u32);
        let mut status = [0u8];
        self.read_reg(i2c, REG_ERR_REG, &mut status)?;
        if status[0] != 0 {
            writeln!(uart, "BMM350 ERR_REG after reset: 0x{:02X}", status[0]).ok();
            return Err(Bmm350Error::PmuError(status[0]));
        }
        writeln!(uart, "BMM350 soft reset completed").ok();

        // Step 4: Load OTP (32 words; discard for raw data).
        let mut _otp_data: [u16; 32] = [0; 32];
        for word in 0..32u8 {
            self.write_reg(i2c, REG_OTP_CMD_REG, (word << 3) | 0x02)?;
            delay.delay_ms(10u32);
            let mut timeout = 50u32;
            loop {
                self.read_reg(i2c, REG_OTP_STATUS_REG, &mut status)?;
                if status[0] & 0x01 == 0 {
                    break;
                }
                timeout -= 1;
                if timeout == 0 {
                    writeln!(uart, "BMM350 OTP busy timeout at word {}", word).ok();
                    return Err(Bmm350Error::OtpBusy);
                }
                delay.delay_ms(10u32);
            }
            let mut lsb = [0u8];
            let mut msb = [0u8];
            self.read_reg(i2c, REG_OTP_DATA_LSB_REG, &mut lsb)?;
            self.read_reg(i2c, REG_OTP_DATA_MSB_REG, &mut msb)?;
            _otp_data[word as usize] = ((msb[0] as u16) << 8) | lsb[0] as u16;
        }
        writeln!(uart, "BMM350 OTP loaded").ok();

        // Step 5: Configure settings.
        self.write_reg(i2c, REG_PAD_CTRL, 0x07)?;
        self.read_reg(i2c, REG_PAD_CTRL, &mut status)?;
        if status[0] != 0x07 {
            writeln!(uart, "BMM350 PAD_CTRL mismatch: expected 0x07, got 0x{:02X}", status[0]).ok();
            return Err(Bmm350Error::ConfigMismatch {
                reg: REG_PAD_CTRL,
                expected: 0x07,
                actual: status[0],
            });
        }
        self.write_reg(i2c, REG_I2C_WDT_SET, 0x01)?;
        self.read_reg(i2c, REG_I2C_WDT_SET, &mut status)?;
        if status[0] != 0x01 {
            writeln!(uart, "BMM350 I2C_WDT_SET mismatch: expected 0x01, got 0x{:02X}", status[0]).ok();
            return Err(Bmm350Error::ConfigMismatch {
                reg: REG_I2C_WDT_SET,
                expected: 0x01,
                actual: status[0],
            });
        }

        // Set ODR/avg for regular preset (100 Hz, avg=2).
        self.write_reg(i2c, REG_PMU_CMD_AGGR_SET, 0x14)?;
        self.read_reg(i2c, REG_PMU_CMD_AGGR_SET, &mut status)?;
        if status[0] != 0x14 {
            writeln!(uart, "BMM350 PMU_CMD_AGGR_SET mismatch: expected 0x14, got 0x{:02X}", status[0]).ok();
            return Err(Bmm350Error::ConfigMismatch {
                reg: REG_PMU_CMD_AGGR_SET,
                expected: 0x14,
                actual: status[0],
            });
        }
        self.write_reg(i2c, REG_PMU_CMD, 0x02)?;
        delay.delay_ms(200u32);
        let mut timeout = 50u32;
        loop {
            self.read_reg(i2c, REG_PMU_CMD_STATUS_0, &mut status)?;
            if status[0] & 0x01 == 0 {
                break;
            }
            timeout -= 1;
            if timeout == 0 {
                writeln!(uart, "BMM350 PMU busy timeout after ODR/avg update").ok();
                return Err(Bmm350Error::PmuBusy);
            }
            delay.delay_ms(10u32);
        }
        self.read_reg(i2c, REG_ERR_REG, &mut status)?;
        if status[0] != 0 {
            writeln!(uart, "BMM350 ERR_REG after ODR/avg: 0x{:02X}", status[0]).ok();
            return Err(Bmm350Error::PmuError(status[0]));
        }
        writeln!(uart, "BMM350 ODR/avg configured").ok();

        // Enable all axes.
        self.write_reg(i2c, REG_PMU_CMD_AXIS_EN, 0x07)?;
        self.read_reg(i2c, REG_PMU_CMD_AXIS_EN, &mut status)?;
        if status[0] != 0x07 {
            writeln!(uart, "BMM350 PMU_CMD_AXIS_EN mismatch: expected 0x07, got 0x{:02X}", status[0]).ok();
            return Err(Bmm350Error::ConfigMismatch {
                reg: REG_PMU_CMD_AXIS_EN,
                expected: 0x07,
                actual: status[0],
            });
        }
        self.read_reg(i2c, REG_ERR_REG, &mut status)?;
        if status[0] != 0 {
            writeln!(uart, "BMM350 ERR_REG after axis enable: 0x{:02X}", status[0]).ok();
            return Err(Bmm350Error::PmuError(status[0]));
        }
        writeln!(uart, "BMM350 axes enabled").ok();

        // Step 6: Set normal mode.
        self.write_reg(i2c, REG_PMU_CMD, 0x01)?;
        delay.delay_ms(200u32);
        timeout = 50u32;
        loop {
            self.read_reg(i2c, REG_PMU_CMD_STATUS_0, &mut status)?;
            if (status[0] & 0x01 == 0) && (status[0] & 0x08 != 0) {
                break;
            }
            timeout -= 1;
            if timeout == 0 {
                writeln!(uart, "BMM350 PMU busy timeout entering normal mode").ok();
                return Err(Bmm350Error::PmuBusy);
            }
            delay.delay_ms(10u32);
        }
        self.read_reg(i2c, REG_ERR_REG, &mut status)?;
        if status[0] != 0 {
            writeln!(uart, "BMM350 ERR_REG after normal mode: 0x{:02X}", status[0]).ok();
            return Err(Bmm350Error::PmuError(status[0]));
        }
        writeln!(uart, "BMM350 entered normal mode").ok();

        // Verify data ready.
        timeout = 50u32;
        loop {
            self.read_reg(i2c, REG_INT_STATUS, &mut status)?;
            if status[0] & 0x04 != 0 {
                writeln!(uart, "BMM350 data ready confirmed: INT_STATUS=0x{:02X}", status[0]).ok();
                break;
            }
            timeout -= 1;
            if timeout == 0 {
                writeln!(uart, "BMM350 data not ready: INT_STATUS=0x{:02X}", status[0]).ok();
                return Err(Bmm350Error::DataNotReady);
            }
            delay.delay_ms(10u32);
        }

        Ok(())
    }

    /// Read raw mag X/Y/Z as RawMag struct, checking data ready and sensortime.
    pub fn read_raw<I2C, E, W>(
        &self,
        i2c: &mut I2C,
        uart: &mut W,
    ) -> Result<RawMag, Bmm350Error<E>>
    where
        I2C: I2c<Error = E>,
        W: core::fmt::Write,
    {
        // Check data ready (INT_STATUS bit 2).
        let mut status = [0u8];
        self.read_reg(i2c, REG_INT_STATUS, &mut status)?;
        if status[0] & 0x04 == 0 {
            writeln!(uart, "BMM350 data not ready: INT_STATUS=0x{:02X}", status[0]).ok();
            return Err(Bmm350Error::DataNotReady);
        }

        // Read sensortime to verify data updates.
        let mut time_buf = [0u8; 3];
        self.read_reg(i2c, REG_SENSORTIME_XLSB, &mut time_buf)?;
        let sensortime = u32::from_le_bytes([time_buf[0], time_buf[1], time_buf[2], 0]);
        writeln!(uart, "BMM350 data ready: INT_STATUS=0x{:02X}, sensortime={}", status[0], sensortime).ok();

        let mut buf = [0u8; 9];
        self.read_reg(i2c, REG_MAG_DATA_START, &mut buf)?;

        // Parse 24-bit signed (sign-extend).
        let raw_x = i32::from_le_bytes([buf[0], buf[1], buf[2], if buf[2] & 0x80 != 0 { 0xFF } else { 0 }]);
        let raw_y = i32::from_le_bytes([buf[3], buf[4], buf[5], if buf[5] & 0x80 != 0 { 0xFF } else { 0 }]);
        let raw_z = i32::from_le_bytes([buf[6], buf[7], buf[8], if buf[8] & 0x80 != 0 { 0xFF } else { 0 }]);

        Ok(RawMag { x: raw_x, y: raw_y, z: raw_z })
    }

    /// Helper: Write single register with retry at specified address.
    fn write_reg_at<I2C, E>(&self, i2c: &mut I2C, addr: u8, reg: u8, val: u8) -> Result<(), Bmm350Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        let buf = [reg, val];
        let mut retry = 3u8;
        loop {
            match i2c.write(addr, &buf) {
                Ok(()) => return Ok(()),
                Err(e) => {
                    retry -= 1;
                    if retry == 0 {
                        return Err(Bmm350Error::I2cError(e));
                    }
                }
            }
        }
    }

    /// Helper: Read into buffer (single or burst) with retry at specified address.
    fn read_reg_at<I2C, E>(&self, i2c: &mut I2C, addr: u8, reg: u8, buf: &mut [u8]) -> Result<(), Bmm350Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        let mut retry = 3u8;
        loop {
            match i2c.write(addr, &[reg]) {
                Ok(()) => match i2c.read(addr, buf) {
                    Ok(()) => return Ok(()),
                    Err(e) => {
                        retry -= 1;
                        if retry == 0 {
                            return Err(Bmm350Error::I2cError(e));
                        }
                    }
                },
                Err(e) => {
                    retry -= 1;
                    if retry == 0 {
                        return Err(Bmm350Error::I2cError(e));
                    }
                }
            }
        }
    }

    /// Helper: Write single register with retry.
    fn write_reg<I2C, E>(&self, i2c: &mut I2C, reg: u8, val: u8) -> Result<(), Bmm350Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        self.write_reg_at(i2c, self.addr, reg, val)
    }

    /// Helper: Read into buffer (single or burst) with retry.
    fn read_reg<I2C, E>(&self, i2c: &mut I2C, reg: u8, buf: &mut [u8]) -> Result<(), Bmm350Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        self.read_reg_at(i2c, self.addr, reg, buf)
    }
}