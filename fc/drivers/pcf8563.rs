// fc/drivers/pcf8563.rs
use embedded_hal::i2c::I2c;
use core::fmt::Write;
use heapless::String as HeaplessString;

/// PCF8563 I2C address
pub const PCF8563_ADDRESS: u8 = 0x51;

/// PCF8563 register addresses
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
    pub fn from_gps(gps_data: &crate::ublox_neom9n::GpsData) -> Self {
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
        
        let h = (self.day as i32 + (13 * (month + 1)) / 5 + k + k / 4 + j / 4 + 5 * j) % 7;
        
        self.weekday = ((h + 5) % 7 + 1) as u8;
    }

    /// Format as a readable string
    pub fn format(&self) -> HeaplessString<32> {
        let mut output = HeaplessString::new();
        let _ = core::write!(output, "{:02}/{:02}/{:04} {:02}:{:02}:{:02}", 
                           self.month, self.day, self.year,
                           self.hour, self.minute, self.second);
        output
    }
}

/// PCF8563 RTC driver errors
#[derive(Debug)]
pub enum Error {
    I2cError,
    InvalidData,
    ClockNotRunning,
}

/// PCF8563 Real-Time Clock driver
pub struct Pcf8563;

impl Pcf8563 {
    /// Read the current date and time from the RTC
    pub fn read_datetime<I: I2c>(&self, i2c: &mut I) -> Result<DateTime, Error> {
        let mut buffer = [0u8; 7];
        i2c.write_read(PCF8563_ADDRESS, &[registers::VL_SECONDS], &mut buffer).map_err(|_| Error::I2cError)?;

        if buffer[0] & 0x80 != 0 {
            return Err(Error::ClockNotRunning);
        }

        let mut datetime = DateTime {
            second: bcd_to_bin(buffer[0] & 0x7F),
            minute: bcd_to_bin(buffer[1] & 0x7F),
            hour: bcd_to_bin(buffer[2] & 0x3F),
            day: bcd_to_bin(buffer[3] & 0x3F),
            weekday: buffer[4] & 0x07,
            month: bcd_to_bin(buffer[5] & 0x1F),
            year: 2000 + bcd_to_bin(buffer[6]) as u16,
        };

        if datetime.second > 59 || datetime.minute > 59 || datetime.hour > 23 ||
           datetime.day == 0 || datetime.day > 31 || datetime.month == 0 || datetime.month > 12 {
            return Err(Error::InvalidData);
        }

        datetime.calculate_weekday();
        Ok(datetime)
    }

    /// Write date and time to the RTC
    pub fn write_datetime<I: I2c>(&self, datetime: &DateTime, i2c: &mut I) -> Result<(), Error> {
        let mut dt = *datetime;
        dt.calculate_weekday();

        let buffer = [
            bin_to_bcd(dt.second) & 0x7F,
            bin_to_bcd(dt.minute),
            bin_to_bcd(dt.hour),
            bin_to_bcd(dt.day),
            dt.weekday,
            bin_to_bcd(dt.month),
            bin_to_bcd((dt.year - 2000) as u8),
        ];

        i2c.write(PCF8563_ADDRESS, &[registers::VL_SECONDS]).map_err(|_| Error::I2cError)?;
        i2c.write(PCF8563_ADDRESS, &buffer).map_err(|_| Error::I2cError)?;

        Ok(())
    }

    /// Check if the RTC is running
    pub fn is_running<I: I2c>(&self, i2c: &mut I) -> Result<bool, Error> {
        let mut buffer = [0u8; 1];
        i2c.write_read(PCF8563_ADDRESS, &[registers::VL_SECONDS], &mut buffer).map_err(|_| Error::I2cError)?;
        Ok(buffer[0] & 0x80 == 0)
    }
}

impl SensorDriver for Pcf8563 {
    type Bus = impl I2c;
    type RawData = [u8; 7];
    type ParsedData = DateTime;
    type Error = Error;

    fn read_raw(&mut self, bus: &mut impl I2c) -> Result<Self::RawData, Self::Error> {
        let mut buffer = [0u8; 7];
        bus.write_read(PCF8563_ADDRESS, &[registers::VL_SECONDS], &mut buffer).map_err(|_| Error::I2cError)?;
        Ok(buffer)
    }

    fn parse(&self, raw: Self::RawData) -> Result<Self::ParsedData, Self::Error> {
        if raw[0] & 0x80 != 0 {
            return Err(Error::ClockNotRunning);
        }

        let mut datetime = DateTime {
            second: bcd_to_bin(raw[0] & 0x7F),
            minute: bcd_to_bin(raw[1] & 0x7F),
            hour: bcd_to_bin(raw[2] & 0x3F),
            day: bcd_to_bin(raw[3] & 0x3F),
            weekday: raw[4] & 0x07,
            month: bcd_to_bin(raw[5] & 0x1F),
            year: 2000 + bcd_to_bin(raw[6]) as u16,
        };

        if datetime.second > 59 || datetime.minute > 59 || datetime.hour > 23 ||
           datetime.day == 0 || datetime.day > 31 || datetime.month == 0 || datetime.month > 12 {
            return Err(Error::InvalidData);
        }

        datetime.calculate_weekday();
        Ok(datetime)
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