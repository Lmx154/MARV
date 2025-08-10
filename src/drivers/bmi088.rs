//! BMI088 IMU Driver (Accelerometer + Gyroscope)
//!
//! Minimal bootstrap driver focused on reading raw accelerometer data (x,y,z) so
//! it can be displayed alongside existing sensors. Gyro stub included for future
//! expansion but not yet used by middleware print path.
//!
//! References: Bosch BMI088 Datasheet (public). This code keeps only a tiny
//! subset of the register map sufficient for: reset, power-up, range config,
//! bandwidth config, and burst read of accelerometer axes.
//!
//! Design notes:
//! - The BMI088 exposes TWO separate SPI targets with distinct chip-selects:
//!   * Accelerometer (ACC) 8-bit registers, CHIP_ID = 0x1E
//!   * Gyroscope     (GYR) 8-bit registers, CHIP_ID = 0x0F
//! - Each target is accessed by asserting its own CS pin; the same SPI bus is
//!   otherwise shared (SCK/MOSI/MISO per `PINOUT.md`).
//! - SPI mode: CPOL=0, CPHA=0 (Mode 0). Max clock during init kept low (e.g. 1 MHz)
//!   then can be increased (datasheet allows higher – left for later tuning).
//! - Read protocol: Set MSB=1 of register address for reads (standard Bosch IMU pattern).
//!   Writes use MSB=0. Multi-byte reads auto-increment address.
//!
//! Scaling: Raw accel values are left as i16 (two's complement). With default
//! ±6g range (acc_range = 0x01) the LSB sensitivity is 32768 / 6 g ≈ 5461 LSB/g.
//! Actual scaling can be applied later.
//!
//! Safety / Robustness: This is a blocking, best-effort driver intended for
//! low-rate status output. No concurrency / arbitration layer yet (planned
//! SPI0 bus manager + SD DMA per design doc).
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;
use defmt::{Format, debug, info, warn};
/// BMI088 Accelerometer register map (subset)
mod acc {
    pub const CHIP_ID: u8 = 0x00;      // Expect 0x1E
    pub const ERR_REG: u8 = 0x02;      // Error flags (not used yet)
    pub const STATUS: u8 = 0x03;       // Data ready bits
    pub const ACC_X_L: u8 = 0x12;      // Burst sequence: XL, XH, YL, YH, ZL, ZH
    pub const SOFTRESET: u8 = 0x7E;    // Write 0xB6 to reset accel core
    pub const PWR_CONF: u8 = 0x7C;     // Set to 0x00 for active (disable suspend)
    pub const PWR_CTRL: u8 = 0x7D;     // Bit 2 (0x04) enables accelerometer
    pub const ACC_RANGE: u8 = 0x41;    // 0x01 => ±6g (default chosen)
    pub const ACC_CONF: u8 = 0x40;     // Bandwidth / ODR config (basic: 0xA8 -> ~100Hz, filtered)
}
/// BMI088 Gyroscope register map (subset)
mod gyr {
    pub const CHIP_ID: u8 = 0x00;      // Expect 0x0F
    pub const RATE_X_L: u8 = 0x02;     // Burst data base (6 bytes)
    pub const RANGE: u8 = 0x0F;        // Range config (not yet set)
    pub const BW: u8 = 0x10;           // Bandwidth / ODR
    pub const LPM1: u8 = 0x11;         // Power mode
    pub const SOFTRESET: u8 = 0x14;    // Write 0xB6 to reset gyro core
}
#[derive(Debug)]
pub enum Error {
    Bus,
    Cs,
    ChipIdAccel(u8),
    ChipIdGyro(u8),
    AccelNotReady,
    Unknown,
}
impl Format for Error {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Error::Bus => defmt::write!(f, "Bus"),
            Error::Cs => defmt::write!(f, "Cs"),
            Error::ChipIdAccel(id) => defmt::write!(f, "ChipIdAccel(0x{:02X})", id),
            Error::ChipIdGyro(id) => defmt::write!(f, "ChipIdGyro(0x{:02X})", id),
            Error::AccelNotReady => defmt::write!(f, "AccelNotReady"),
            Error::Unknown => defmt::write!(f, "Unknown"),
        }
    }
}
/// Raw accelerometer + (future) gyro frame
#[derive(Debug, Clone, Copy, Default)]
pub struct Bmi088Raw {
    pub accel: [i16; 3],
    pub gyro: [i16; 3], // presently zeroed (not yet read by middleware)
}
/// BMI088 driver over a shared SPI bus with two CS lines
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
    /// Initialise accelerometer (and soft-reset gyro for presence check). Gyro range left default.
    pub fn init(&mut self) -> Result<(), Error> {
        info!("BMI088: proper initialization sequence");
       
        // Ensure CS lines are high initially
        let _ = self.cs_accel.set_high();
        let _ = self.cs_gyro.set_high();
        self.delay.delay_ms(10);
        // BMI088 specific power-up sequence from datasheet
        // Step 1: Start with gyro reset (datasheet recommendation)
        info!("BMI088: resetting gyroscope first");
        let _ = self.write_gyr(gyr::SOFTRESET, 0xB6);
        self.delay.delay_ms(100); // Gyro needs more time after reset
       
        // Step 2: Reset accelerometer
        info!("BMI088: resetting accelerometer");
        self.write_acc(acc::SOFTRESET, 0xB6)?;
        self.delay.delay_ms(100); // Longer delay for accelerometer reset
       
        // Dummy read to switch back to SPI mode after reset
        let _ = self.read_acc(acc::CHIP_ID); // Ignore result, as it may be invalid
        self.delay.delay_ms(10); // Short stability delay

        // Step 3: Critical BMI088 accelerometer power-up sequence
        // MUST disable suspend mode BEFORE enabling accelerometer
        info!("BMI088: configuring power management");
        self.write_acc(acc::PWR_CONF, 0x00)?; // Active mode (exit suspend)
        self.delay.delay_ms(5);
        self.write_acc(acc::PWR_CTRL, 0x04)?; // Enable accelerometer
        self.delay.delay_ms(50); // Critical delay after power-on
       
        // Step 4: Additional stability delay
        self.delay.delay_ms(50);
        // Step 5: Try multiple read approaches
        let mut acc_id = 0u8;
        let mut gyr_id = 0u8;
        let mut success = false;
       
        for attempt in 0..10 { // More attempts
            self.delay.delay_ms(20); // Longer delay between attempts
           
            // Try both read methods
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
            // Final diagnostic attempt
            let mut alt_acc = 0xFFu8; let mut alt_gyr = 0xFFu8;
            if let Ok(v) = self.read_acc_alt(acc::CHIP_ID) { alt_acc = v; }
            if let Ok(vg) = self.read_gyr_alt(gyr::CHIP_ID) { alt_gyr = vg; }
            warn!("BMI088: ID mismatch primary acc=0x{:02X} gyr=0x{:02X} alt acc=0x{:02X} gyr=0x{:02X}", acc_id, gyr_id, alt_acc, alt_gyr);
           
            // Raw burst frame attempt for accel starting at 0x00 (read 8 bytes)
            if let Some(raw) = self.read_raw_bytes_acc(0x00, 8) {
                debug!("BMI088: raw acc bytes @0x00 = {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
                       raw[0],raw[1],raw[2],raw[3],raw[4],raw[5],raw[6],raw[7]);
            }
           
            if acc_id != 0x1E { return Err(Error::ChipIdAccel(acc_id)); }
            if gyr_id != 0x0F { return Err(Error::ChipIdGyro(gyr_id)); }
        }
        // Configure accel range & bandwidth
        self.write_acc(acc::ACC_RANGE, 0x01)?; // ±6g
        self.write_acc(acc::ACC_CONF, 0xA8)?;  // ODR/filtered (~100Hz)
        self.delay.delay_ms(2);
        info!("BMI088: accelerometer configured range=±6g bw=0xA8");
        Ok(())
    }
    /// Read an accelerometer sample (blocking). Returns raw i16 counts per axis.
    pub fn read_accel(&mut self) -> Result<[i16;3], Error> {
        // BMI088 accelerometer requires dummy byte after register address for burst reads
    let mut buf = [0u8; 8]; // addr + dummy + 6 data bytes
    buf[0] = acc::ACC_X_L | 0x80;
    self.cs_accel.set_low().map_err(|_| Error::Cs)?;
    if self.bus.transfer_in_place(&mut buf).is_err() { self.cs_accel.set_high().ok(); return Err(Error::Bus); }
    self.cs_accel.set_high().map_err(|_| Error::Cs)?;
    // Data starts from buf[2] (skip addr and dummy byte)
    let x = i16::from_le_bytes([buf[2], buf[3]]);
    let y = i16::from_le_bytes([buf[4], buf[5]]);
    let z = i16::from_le_bytes([buf[6], buf[7]]);
        Ok([x,y,z])
    }
    /// Combined read (future expansion) currently returns accel + zero gyro.
    pub fn read_raw(&mut self) -> Result<Bmi088Raw, Error> {
        let accel = self.read_accel()?;
        Ok(Bmi088Raw { accel, gyro: [0;3] })
    }
    fn read_acc(&mut self, reg: u8) -> Result<u8, Error> {
    // BMI088 accelerometer requires dummy byte after register address
    let mut frame = [reg | 0x80, 0x00, 0x00];
    self.cs_accel.set_low().map_err(|_| Error::Cs)?;
    if self.bus.transfer_in_place(&mut frame).is_err() { self.cs_accel.set_high().ok(); return Err(Error::Bus); }
    self.cs_accel.set_high().map_err(|_| Error::Cs)?;
    Ok(frame[2]) // Data is in the third byte, not second
    }
    fn write_acc(&mut self, reg: u8, val: u8) -> Result<(), Error> {
    self.cs_accel.set_low().map_err(|_| Error::Cs)?;
    let bytes = [reg & 0x7F, val];
    if self.bus.write(&bytes).is_err() { self.cs_accel.set_high().ok(); return Err(Error::Bus); }
    self.cs_accel.set_high().map_err(|_| Error::Cs)?;
        Ok(())
    }
    fn read_gyr(&mut self, reg: u8) -> Result<u8, Error> {
    let mut frame = [reg | 0x80, 0x00];
    self.cs_gyro.set_low().map_err(|_| Error::Cs)?;
    if self.bus.transfer_in_place(&mut frame).is_err() { self.cs_gyro.set_high().ok(); return Err(Error::Bus); }
    self.cs_gyro.set_high().map_err(|_| Error::Cs)?;
    Ok(frame[1])
    }
    fn write_gyr(&mut self, reg: u8, val: u8) -> Result<(), Error> {
    self.cs_gyro.set_low().map_err(|_| Error::Cs)?;
    let bytes = [reg & 0x7F, val];
    if self.bus.write(&bytes).is_err() { self.cs_gyro.set_high().ok(); return Err(Error::Bus); }
    self.cs_gyro.set_high().map_err(|_| Error::Cs)?;
        Ok(())
    }
    // Alternate read: split write then read (some SPI setups prefer this)
    fn read_acc_alt(&mut self, reg: u8) -> Result<u8, Error> {
        // BMI088 accelerometer alternative read with dummy byte
        let addr = reg | 0x80;
        let mut rx = [0u8; 2]; // dummy + actual data
        self.cs_accel.set_low().map_err(|_| Error::Cs)?;
        if self.bus.write(&[addr]).is_err() { self.cs_accel.set_high().ok(); return Err(Error::Bus); }
        if self.bus.read(&mut rx).is_err() { self.cs_accel.set_high().ok(); return Err(Error::Bus); }
        self.cs_accel.set_high().map_err(|_| Error::Cs)?;
        Ok(rx[1]) // Second byte contains the actual data
    }
    fn read_gyr_alt(&mut self, reg: u8) -> Result<u8, Error> {
        let addr = reg | 0x80;
        let mut rx = [0u8;1];
        self.cs_gyro.set_low().map_err(|_| Error::Cs)?;
        if self.bus.write(&[addr]).is_err() { self.cs_gyro.set_high().ok(); return Err(Error::Bus); }
        if self.bus.read(&mut rx).is_err() { self.cs_gyro.set_high().ok(); return Err(Error::Bus); }
        self.cs_gyro.set_high().map_err(|_| Error::Cs)?;
        Ok(rx[0])
    }
    fn read_raw_bytes_acc(&mut self, start: u8, len: usize) -> Option<[u8;8]> {
        if len > 8 { return None; }
        // BMI088 accelerometer needs dummy byte, so buffer needs to be len+2 (addr + dummy + data)
        let mut buf = [0u8;10]; // addr + dummy + up to 8 data bytes
        buf[0] = start | 0x80;
        self.cs_accel.set_low().ok()?;
        if self.bus.transfer_in_place(&mut buf[..len+2]).is_err() { self.cs_accel.set_high().ok(); return None; }
        self.cs_accel.set_high().ok()?;
        let mut out = [0u8;8];
        out[..len].copy_from_slice(&buf[2..len+2]); // Skip addr and dummy byte
        Some(out)
    }
}