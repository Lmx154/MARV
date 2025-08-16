//! SD card middleware API
//!
//! Provides a tiny wrapper for writing a line to a status file.

use embedded_sdmmc as sdmmc;

pub struct SdApi<'a, D> {
    driver: &'a D,
}

impl<'a, D> SdApi<'a, D> {
    pub fn new(driver: &'a D) -> Self { Self { driver } }
}

impl<'a, SPI, DELAY> SdApi<'a, crate::drivers::sd::SdDriver<SPI, DELAY>>
where
    SPI: embedded_hal::spi::SpiDevice,
    DELAY: embedded_hal::delay::DelayNs,
{
    pub fn init(&self) -> Result<(), sdmmc::Error<sdmmc::SdCardError>> {
        self.driver.init()?;
        Ok(())
    }

    /// Append a line to STATUS.TXT in the root directory.
    pub fn write_status_line(&self, line: &str) -> Result<(), sdmmc::Error<sdmmc::SdCardError>> {
        self.driver.append_text("STATUS.TXT", line.as_bytes())?;
        Ok(())
    }
}
