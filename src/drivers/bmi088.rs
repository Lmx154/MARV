//! BMI088 IMU Driver (Accelerometer + Gyroscope)
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;
use defmt::{debug, info, warn, Format};

/// BMI088 Accelerometer register map (subset)
mod acc {
    pub const CHIP_ID: u8 = 0x00;      // Expect 0x1E
    pub const ACC_X_L: u8 = 0x12;      // Burst sequence: XL, XH, YL, YH, ZL, ZH
    pub const SOFTRESET: u8 = 0x7E;    // Write 0xB6 to reset accel core
    pub const PWR_CONF: u8 = 0x7C;     // Set to 0x00 for active (disable suspend)
    pub const PWR_CTRL: u8 = 0x7D;     // Bit 2 (0x04) enables accelerometer
    pub const ACC_RANGE: u8 = 0x41;    // 0x01 => ±6g
    pub const ACC_CONF: u8 = 0x40;     // Bandwidth / ODR config (0xA8 -> ~100Hz, filtered)
}

/// BMI088 Gyroscope register map (subset)
mod gyr {
    pub const CHIP_ID: u8 = 0x00;      // Expect 0x0F
    pub const SOFTRESET: u8 = 0x14;    // Write 0xB6 to reset gyro core
    pub const RATE_X_L: u8 = 0x02;     // Burst start: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    pub const RANGE: u8 = 0x0F;        // 0x00 => 2000 dps
    pub const BW: u8 = 0x10;           // ODR/Bandwidth (0x07 => 100Hz, 32Hz BW)
    pub const LPM1: u8 = 0x15;         // Normal mode: set to 0x00
}

#[derive(Debug, Format)]
#[allow(dead_code)] // Suppress warning for unused variant
pub enum Error {
    Bus,
    Cs,
    ChipIdAccel(u8),
    ChipIdGyro(u8),
    Unknown,
}

/// Raw accelerometer + gyro frame (gyro stubbed)
#[derive(Debug, Clone, Copy, Default)]
pub struct Bmi088Raw {
    pub accel: [i16; 3],
    #[allow(dead_code)] // Suppress warning for unused field
    pub gyro: [i16; 3], // Stubbed as [0;3]
}

pub struct Bmi088<BUS, CSACC, CSGYR, DELAY>
where
    BUS: SpiBus,
    CSACC: OutputPin,
    CSGYR: OutputPin,
    DELAY: DelayNs,
{
    bus: BUS,
    cs_accel: CSACC,
    cs_gyro: CSGYR,
    delay: DELAY,
}

impl<BUS, CSACC, CSGYR, DELAY> Bmi088<BUS, CSACC, CSGYR, DELAY>
where
    BUS: SpiBus,
    CSACC: OutputPin,
    CSGYR: OutputPin,
    DELAY: DelayNs,
{
    pub fn new(bus: BUS, cs_accel: CSACC, cs_gyro: CSGYR, delay: DELAY) -> Self {
        Self { bus, cs_accel, cs_gyro, delay }
    }

    pub fn init(&mut self) -> Result<(), Error> {
        info!("BMI088: proper initialization sequence");

        let _ = self.cs_accel.set_high();
        let _ = self.cs_gyro.set_high();
        self.delay.delay_ms(10);

    info!("BMI088: resetting gyroscope first");
    let _ = self.write_gyr(gyr::SOFTRESET, 0xB6);
    self.delay.delay_ms(50);

        info!("BMI088: resetting accelerometer");
        self.write_acc(acc::SOFTRESET, 0xB6)?;
        self.delay.delay_ms(100);

        let _ = self.read_acc(acc::CHIP_ID); // Dummy read post-reset
        self.delay.delay_ms(10);

        info!("BMI088: configuring power management");
        self.write_acc(acc::PWR_CONF, 0x00)?;
        self.delay.delay_ms(5);
        self.write_acc(acc::PWR_CTRL, 0x04)?;
    self.delay.delay_ms(20);

    info!("BMI088: configuring gyroscope power + range/bw");
    // Exit suspend -> normal mode
    let _ = self.write_gyr(gyr::LPM1, 0x00); // normal mode
    self.delay.delay_ms(30);
    let _ = self.write_gyr(gyr::RANGE, 0x00); // 2000 dps
    let _ = self.write_gyr(gyr::BW, 0x07);    // ~100 Hz ODR
    self.delay.delay_ms(10);

        self.delay.delay_ms(50);

    let mut acc_id = 0u8;
    let mut gyr_id = 0u8;
        let mut success = false;

        for attempt in 0..10 {
            self.delay.delay_ms(20);

            if let Ok(id) = self.read_acc(acc::CHIP_ID) {
                acc_id = id;
                if id == 0x1E {
                    info!("BMI088: accelerometer ID found on attempt {}", attempt);
                }
            }
            if let Ok(alt_id) = self.read_acc_alt(acc::CHIP_ID) {
                if alt_id == 0x1E && acc_id != 0x1E {
                    acc_id = alt_id;
                    info!("BMI088: accelerometer ID found via alternate read");
                }
            }

            if let Ok(id) = self.read_gyr(gyr::CHIP_ID) {
                gyr_id = id;
                if id == 0x0F {
                    info!("BMI088: gyroscope ID found on attempt {}", attempt);
                }
            }

            debug!("BMI088: attempt {} IDs acc=0x{:02X} gyr=0x{:02X}", attempt, acc_id, gyr_id);

            if acc_id == 0x1E && gyr_id == 0x0F {
                success = true;
                break;
            }
        }

        if !success {
            let mut alt_acc = 0xFFu8;
            let mut alt_gyr = 0xFFu8;
            if let Ok(v) = self.read_acc_alt(acc::CHIP_ID) { alt_acc = v; }
            if let Ok(vg) = self.read_gyr_alt(gyr::CHIP_ID) { alt_gyr = vg; }
            warn!("BMI088: ID mismatch primary acc=0x{:02X} gyr=0x{:02X} alt acc=0x{:02X} gyr=0x{:02X}", acc_id, gyr_id, alt_acc, alt_gyr);

            if let Some(raw) = self.read_raw_bytes_acc(0x00, 8) {
                debug!("BMI088: raw acc bytes @0x00 = {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
                    raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7]);
            }

            if acc_id != 0x1E { return Err(Error::ChipIdAccel(acc_id)); }
            if gyr_id != 0x0F { return Err(Error::ChipIdGyro(gyr_id)); }
        }

    self.write_acc(acc::ACC_RANGE, 0x01)?; // ±6g
    self.write_acc(acc::ACC_CONF, 0xA8)?;  // ~100Hz
        self.delay.delay_ms(2);
        info!("BMI088: accelerometer configured range=±6g bw=0xA8");
        info!("BMI088: gyroscope configured range=2000dps bw=0x07");
        // Diagnostic: read first 6 gyro bytes after config
        if let Some(raw) = self.read_raw_bytes_gyr(gyr::RATE_X_L, 6) {
            debug!("BMI088: gyro raw bytes post-config {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}", raw[0],raw[1],raw[2],raw[3],raw[4],raw[5]);
        }
        Ok(())
    }

    pub fn read_raw(&mut self) -> Result<Bmi088Raw, Error> {
        let accel = self.read_accel()?;
        let gyro = self.read_gyro().unwrap_or([0;3]);
        Ok(Bmi088Raw { accel, gyro })
    }

    fn read_accel(&mut self) -> Result<[i16; 3], Error> {
        let mut buf = [0u8; 8];
        buf[0] = acc::ACC_X_L | 0x80;
        self.cs_accel.set_low().map_err(|_| Error::Cs)?;
        if self.bus.transfer_in_place(&mut buf).is_err() { self.cs_accel.set_high().ok(); return Err(Error::Bus); }
        self.cs_accel.set_high().map_err(|_| Error::Cs)?;
        let x = i16::from_le_bytes([buf[2], buf[3]]);
        let y = i16::from_le_bytes([buf[4], buf[5]]);
        let z = i16::from_le_bytes([buf[6], buf[7]]);
        Ok([x, y, z])
    }

    fn read_acc(&mut self, reg: u8) -> Result<u8, Error> {
        let mut frame = [reg | 0x80, 0x00, 0x00];
        self.cs_accel.set_low().map_err(|_| Error::Cs)?;
        if self.bus.transfer_in_place(&mut frame).is_err() { self.cs_accel.set_high().ok(); return Err(Error::Bus); }
        self.cs_accel.set_high().map_err(|_| Error::Cs)?;
        Ok(frame[2])
    }

    fn write_acc(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.cs_accel.set_low().map_err(|_| Error::Cs)?;
        let bytes = [reg & 0x7F, val];
        if self.bus.write(&bytes).is_err() { self.cs_accel.set_high().ok(); return Err(Error::Bus); }
        self.cs_accel.set_high().map_err(|_| Error::Cs)?;
        Ok(())
    }

    fn read_gyr(&mut self, reg: u8) -> Result<u8, Error> {
    // Use alternate method internally for reliability
    self.read_gyr_alt(reg)
    }

    fn write_gyr(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.cs_gyro.set_low().map_err(|_| Error::Cs)?;
        let bytes = [reg & 0x7F, val];
        if self.bus.write(&bytes).is_err() { self.cs_gyro.set_high().ok(); return Err(Error::Bus); }
        self.cs_gyro.set_high().map_err(|_| Error::Cs)?;
        Ok(())
    }

    fn read_acc_alt(&mut self, reg: u8) -> Result<u8, Error> {
        let addr = reg | 0x80;
        let mut rx = [0u8; 2];
        self.cs_accel.set_low().map_err(|_| Error::Cs)?;
        if self.bus.write(&[addr]).is_err() { self.cs_accel.set_high().ok(); return Err(Error::Bus); }
        if self.bus.read(&mut rx).is_err() { self.cs_accel.set_high().ok(); return Err(Error::Bus); }
        self.cs_accel.set_high().map_err(|_| Error::Cs)?;
        Ok(rx[1])
    }

    fn read_gyr_alt(&mut self, reg: u8) -> Result<u8, Error> {
        let addr = reg | 0x80;
    let mut rx = [0u8; 1];
        self.cs_gyro.set_low().map_err(|_| Error::Cs)?;
        if self.bus.write(&[addr]).is_err() { self.cs_gyro.set_high().ok(); return Err(Error::Bus); }
        if self.bus.read(&mut rx).is_err() { self.cs_gyro.set_high().ok(); return Err(Error::Bus); }
        self.cs_gyro.set_high().map_err(|_| Error::Cs)?;
        Ok(rx[0])
    }

    fn read_raw_bytes_acc(&mut self, start: u8, len: usize) -> Option<[u8; 8]> {
        if len > 8 { return None; }
        let mut buf = [0u8; 10];
        buf[0] = start | 0x80;
        self.cs_accel.set_low().ok()?;
        if self.bus.transfer_in_place(&mut buf[..len + 2]).is_err() { self.cs_accel.set_high().ok(); return None; }
        self.cs_accel.set_high().ok()?;
        let mut out = [0u8; 8];
        out[..len].copy_from_slice(&buf[2..len + 2]);
        Some(out)
    }

    fn read_raw_bytes_gyr(&mut self, start: u8, len: usize) -> Option<[u8; 8]> {
        if len > 8 { return None; }
        let mut buf = [0u8; 10];
        buf[0] = start | 0x80;
        self.cs_gyro.set_low().ok()?;
        if self.bus.transfer_in_place(&mut buf[..len + 1]).is_err() { self.cs_gyro.set_high().ok(); return None; }
        self.cs_gyro.set_high().ok()?;
        let mut out = [0u8; 8];
        out[..len].copy_from_slice(&buf[1..len + 1]);
        Some(out)
    }

    fn read_gyro(&mut self) -> Result<[i16;3], Error> {
        // Burst read: address + 6 data bytes (no extra dummy beyond address)
        let mut buf = [0u8; 7];
        buf[0] = gyr::RATE_X_L | 0x80;
        self.cs_gyro.set_low().map_err(|_| Error::Cs)?;
        if self.bus.transfer_in_place(&mut buf).is_err() { self.cs_gyro.set_high().ok(); return Err(Error::Bus); }
        self.cs_gyro.set_high().map_err(|_| Error::Cs)?;
        let x = i16::from_le_bytes([buf[1], buf[2]]);
        let y = i16::from_le_bytes([buf[3], buf[4]]);
        let z = i16::from_le_bytes([buf[5], buf[6]]);
        Ok([x,y,z])
    }
}