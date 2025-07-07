//! PCF8563 Real-Time Clock Driver
//!
//! Driver for the NXP PCF8563 I2C Real-Time Clock
//! 
//! Features:
//! - Read/write date and time
//! - Low power consumption
//! - Alarm functionality
//! - Timer functionality
//! - Clock output



use embedded_hal::i2c::I2c;
use defmt::*;
use core::fmt::Write;

/// PCF8563 I2C address
pub const PCF8563_ADDRESS: u8 = 0x51;

/// PCF8563 register addresses
#[allow(dead_code)]
mod registers {
    pub const CONTROL_STATUS_1: u8 = 0x00;
    pub const CONTROL_STATUS_2: u8 = 0x01;
    pub const VL_SECONDS: u8 = 0x02;
    pub const MINUTES: u8 = 0x03;
    pub const HOURS: u8 = 0x04;
    pub const DAYS: u8 = 0x05;
    pub const WEEKDAYS: u8 = 0x06;
    pub const CENTURY_MONTHS: u8 = 0x07;
    pub const YEARS: u8 = 0x08;
    pub const MINUTE_ALARM: u8 = 0x09;
    pub const HOUR_ALARM: u8 = 0x0A;
    pub const DAY_ALARM: u8 = 0x0B;
    pub const WEEKDAY_ALARM: u8 = 0x0C;
    pub const CLKOUT_CONTROL: u8 = 0x0D;
    pub const TIMER_CONTROL: u8 = 0x0E;
    pub const TIMER: u8 = 0x0F;
}

/// Date and time structure compatible with PCF8563
#[derive(Debug, Clone, Copy, Default)]
pub struct DateTime {
    pub year: u16,    // 2000-2099
    pub month: u8,    // 1-12
    pub day: u8,      // 1-31
    pub weekday: u8,  // 0-6 (0=Sunday)
    pub hour: u8,     // 0-23
    pub minute: u8,   // 0-59
    pub second: u8,   // 0-59
}

impl DateTime {
    /// Create a new DateTime with the current date/time from GPS
    pub fn from_gps(gps_data: &crate::sensors::gps::GpsData) -> Self {
        Self {
            year: gps_data.year,
            month: gps_data.month,
            day: gps_data.day,
            weekday: 0, // Will be calculated
            hour: gps_data.hour,
            minute: gps_data.minute,
            second: gps_data.second,
        }
    }

    /// Calculate day of week (0=Sunday, 1=Monday, etc.)
    pub fn calculate_weekday(&mut self) {
        // Zeller's congruence algorithm
        let mut year = self.year as i32;
        let mut month = self.month as i32;
        
        if month < 3 {
            month += 12;
            year -= 1;
        }
        
        let k = year % 100;
        let j = year / 100;
        
        let h = (self.day as i32 + (13 * (month + 1)) / 5 + k + k / 4 + j / 4 - 2 * j) % 7;
        
        // Convert to 0=Sunday format
        self.weekday = ((h + 5) % 7) as u8;
    }

    /// Format as a readable string
    pub fn format(&self) -> heapless::String<32> {
        let mut output = heapless::String::new();
        let _ = core::write!(output, "{:02}/{:02}/{:04} {:02}:{:02}:{:02}", 
                           self.month, self.day, self.year,
                           self.hour, self.minute, self.second);
        output
    }

    /// Format as individual components for defmt
    pub fn format_for_defmt(&self) -> (&str, u8, u8, u16, u8, u8, u8) {
        ("PCF8563", self.month, self.day, self.year, self.hour, self.minute, self.second)
    }
}

/// PCF8563 RTC driver errors
#[derive(Debug)]
#[derive(defmt::Format)]
pub enum Error {
    I2cError,
    InvalidData,
    ClockNotRunning,
}

/// PCF8563 Real-Time Clock driver
pub struct Pcf8563<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> Pcf8563<I2C>
where
    I2C: I2c,
{
    /// Create a new PCF8563 driver instance
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            address: PCF8563_ADDRESS,
        }
    }

    /// Initialize the RTC
    pub fn init(&mut self) -> Result<(), Error> {
        // Read control status to check if RTC is running
        let status = self.read_register(registers::CONTROL_STATUS_1)
            .map_err(|_| Error::I2cError)?;
        
        info!("PCF8563 Control Status 1: 0x{:02X}", status);
        
        // Read control status 2 for additional info
        let status2 = self.read_register(registers::CONTROL_STATUS_2)
            .map_err(|_| Error::I2cError)?;
        
        info!("PCF8563 Control Status 2: 0x{:02X}", status2);
        
        // Check if clock is running (bit 7 of VL_SECONDS should be 0)
        let seconds_reg = self.read_register(registers::VL_SECONDS)
            .map_err(|_| Error::I2cError)?;
        
        info!("PCF8563 VL_SECONDS register: 0x{:02X}", seconds_reg);
        
        if seconds_reg & 0x80 != 0 {
            warn!("PCF8563 clock integrity not guaranteed (VL bit set)");
            
            // Try to force-clear the VL bit by writing a valid time
            info!("Attempting to initialize RTC with default time to clear VL bit");
            let default_time = DateTime {
                year: 2025,
                month: 7,
                day: 7,
                weekday: 1, // Monday
                hour: 12,
                minute: 0,
                second: 0,
            };
            
            // Try to write default time to clear VL bit
            if let Err(e) = self.write_datetime(&default_time) {
                error!("Failed to write default time to PCF8563: {:?}", e);
                return Err(e);
            }
            
            // Re-check VL bit after write
            let seconds_reg_after = self.read_register(registers::VL_SECONDS)
                .map_err(|_| Error::I2cError)?;
            
            info!("PCF8563 VL_SECONDS after init write: 0x{:02X}", seconds_reg_after);
            
            if seconds_reg_after & 0x80 != 0 {
                error!("PCF8563 VL bit still set after initialization - possible hardware issue");
                return Err(Error::ClockNotRunning);
            }
        }
        
        info!("PCF8563 initialized successfully");
        Ok(())
    }

    /// Read the current date and time from the RTC
    pub fn read_datetime(&mut self) -> Result<DateTime, Error> {
        // Read all time/date registers in one go (7 bytes starting from VL_SECONDS)
        let mut buffer = [0u8; 7];
        self.read_registers(registers::VL_SECONDS, &mut buffer)
            .map_err(|_| Error::I2cError)?;

        // Check if clock is running
        if buffer[0] & 0x80 != 0 {
            warn!("PCF8563 VL bit is set - clock integrity may be compromised");
            // Let's try to read the time anyway and see what we get
            // return Err(Error::ClockNotRunning);
        }

        let datetime = DateTime {
            second: bcd_to_bin(buffer[0] & 0x7F),
            minute: bcd_to_bin(buffer[1] & 0x7F),
            hour: bcd_to_bin(buffer[2] & 0x3F),
            day: bcd_to_bin(buffer[3] & 0x3F),
            weekday: buffer[4] & 0x07,
            month: bcd_to_bin(buffer[5] & 0x1F),
            year: 2000 + bcd_to_bin(buffer[6]) as u16,
        };

        // Validate the time values are reasonable
        if datetime.second > 59 || datetime.minute > 59 || datetime.hour > 23 ||
           datetime.day == 0 || datetime.day > 31 || datetime.month == 0 || datetime.month > 12 {
            warn!("PCF8563 returned invalid time values - possible corruption");
            if buffer[0] & 0x80 != 0 {
                return Err(Error::ClockNotRunning);
            } else {
                return Err(Error::InvalidData);
            }
        }

        Ok(datetime)
    }

    /// Write date and time to the RTC
    pub fn write_datetime(&mut self, datetime: &DateTime) -> Result<(), Error> {
        let mut dt = *datetime;
        dt.calculate_weekday(); // Ensure weekday is calculated

        // Prepare the data buffer (7 bytes)
        let buffer = [
            bin_to_bcd(dt.second) & 0x7F,    // VL_SECONDS with VL bit explicitly cleared (bit 7 = 0)
            bin_to_bcd(dt.minute),           // MINUTES
            bin_to_bcd(dt.hour),             // HOURS
            bin_to_bcd(dt.day),              // DAYS
            dt.weekday,                      // WEEKDAYS
            bin_to_bcd(dt.month),            // MONTHS (century bit is handled automatically)
            bin_to_bcd((dt.year - 2000) as u8), // YEARS
        ];

        // Write all registers starting from VL_SECONDS
        self.write_registers(registers::VL_SECONDS, &buffer)
            .map_err(|_| Error::I2cError)?;

        let (_, month, day, year, hour, minute, second) = dt.format_for_defmt();
        info!("PCF8563 datetime updated: {:02}/{:02}/{:04} {:02}:{:02}:{:02}", 
              month, day, year, hour, minute, second);
        
        // Verify the write was successful by checking if the clock is now running
        match self.is_running() {
            Ok(true) => {
                info!("PCF8563 clock is now running after sync");
            }
            Ok(false) => {
                warn!("PCF8563 clock is still not running after sync");
            }
            Err(e) => {
                warn!("Failed to verify PCF8563 status after sync: {:?}", e);
            }
        }
        
        Ok(())
    }

    /// Check if the RTC is running
    pub fn is_running(&mut self) -> Result<bool, Error> {
        let seconds_reg = self.read_register(registers::VL_SECONDS)
            .map_err(|_| Error::I2cError)?;
        
        // Clock is running if VL bit (bit 7) is 0
        Ok(seconds_reg & 0x80 == 0)
    }

    /// Read a single register
    fn read_register(&mut self, register: u8) -> Result<u8, ()> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(self.address, &[register], &mut buffer).map_err(|_| ())?;
        Ok(buffer[0])
    }

    /// Read multiple registers
    fn read_registers(&mut self, start_register: u8, buffer: &mut [u8]) -> Result<(), ()> {
        self.i2c.write_read(self.address, &[start_register], buffer).map_err(|_| ())
    }

    /// Write multiple registers
    fn write_registers(&mut self, start_register: u8, data: &[u8]) -> Result<(), ()> {
        let mut write_buffer = [0u8; 8]; // Max 7 data bytes + 1 register byte
        write_buffer[0] = start_register;
        write_buffer[1..=data.len()].copy_from_slice(data);
        
        self.i2c.write(self.address, &write_buffer[..=data.len()]).map_err(|_| ())
    }

    /// Perform diagnostic read of PCF8563 registers
    pub fn diagnostic_read(&mut self) -> Result<(), Error> {
        info!("=== PCF8563 Diagnostic Read ===");
        
        // Read control registers
        let ctrl1 = self.read_register(registers::CONTROL_STATUS_1)
            .map_err(|_| Error::I2cError)?;
        let ctrl2 = self.read_register(registers::CONTROL_STATUS_2)
            .map_err(|_| Error::I2cError)?;
        
        info!("Control Status 1: 0x{:02X}", ctrl1);
        info!("Control Status 2: 0x{:02X}", ctrl2);
        
        // Read all time registers
        let mut time_buffer = [0u8; 7];
        self.read_registers(registers::VL_SECONDS, &mut time_buffer)
            .map_err(|_| Error::I2cError)?;
        
        info!("VL_SECONDS: 0x{:02X} (VL bit: {})", time_buffer[0], if time_buffer[0] & 0x80 != 0 { "SET" } else { "CLEAR" });
        info!("MINUTES: 0x{:02X}", time_buffer[1]);
        info!("HOURS: 0x{:02X}", time_buffer[2]);
        info!("DAYS: 0x{:02X}", time_buffer[3]);
        info!("WEEKDAYS: 0x{:02X}", time_buffer[4]);
        info!("MONTHS: 0x{:02X}", time_buffer[5]);
        info!("YEARS: 0x{:02X}", time_buffer[6]);
        
        // Attempt to decode time if VL bit is clear
        if time_buffer[0] & 0x80 == 0 {
            let datetime = DateTime {
                second: bcd_to_bin(time_buffer[0] & 0x7F),
                minute: bcd_to_bin(time_buffer[1] & 0x7F),
                hour: bcd_to_bin(time_buffer[2] & 0x3F),
                day: bcd_to_bin(time_buffer[3] & 0x3F),
                weekday: time_buffer[4] & 0x07,
                month: bcd_to_bin(time_buffer[5] & 0x1F),
                year: 2000 + bcd_to_bin(time_buffer[6]) as u16,
            };
            
            let (_, month, day, year, hour, minute, second) = datetime.format_for_defmt();
            info!("Decoded time: {:02}/{:02}/{:04} {:02}:{:02}:{:02}", 
                  month, day, year, hour, minute, second);
        } else {
            warn!("Cannot decode time - VL bit is set");
        }
        
        info!("=== End Diagnostic ===");
        Ok(())
    }
}

/// Convert BCD to binary
fn bcd_to_bin(bcd: u8) -> u8 {
    (bcd & 0x0F) + ((bcd >> 4) * 10)
}

/// Convert binary to BCD
fn bin_to_bcd(bin: u8) -> u8 {
    ((bin / 10) << 4) | (bin % 10)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bcd_conversion() {
        assert_eq!(bcd_to_bin(0x23), 23);
        assert_eq!(bcd_to_bin(0x59), 59);
        assert_eq!(bin_to_bcd(23), 0x23);
        assert_eq!(bin_to_bcd(59), 0x59);
    }

    #[test]
    fn test_weekday_calculation() {
        let mut dt = DateTime {
            year: 2025,
            month: 7,
            day: 7,
            weekday: 0,
            hour: 18,
            minute: 30,
            second: 0,
        };
        
        dt.calculate_weekday();
        assert_eq!(dt.weekday, 1); // July 7, 2025 is a Monday
    }
}
