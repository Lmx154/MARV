//! SD card driver over SPI1 using embedded-sdmmc
//!
//! Pins per docs/PINOUT.md (SPI1):
//! - SCK = GP10
//! - MOSI = GP11
//! - MISO = GP12
//! - CS   = GP13
//!
//! This is a minimal, blocking driver to mount the first FAT volume and
//! create/append to a text file.

use core::cell::RefCell;
use defmt::info;
use embedded_hal::delay::DelayNs;
use embedded_sdmmc as sdmmc;
use sdmmc::{Error as FsError, Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};

/// Simple time source using RV-8803 if available, otherwise a fixed timestamp.
pub struct SdTimeSource {
    // Provide a way to inject a time if needed later.
}
impl TimeSource for SdTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        // Fallback to a constant compile-time-ish value; FAT requires non-zero date.
        Timestamp { year_since_1970: 55, zero_indexed_month: 0, zero_indexed_day: 0, hours: 0, minutes: 0, seconds: 0 }
    }
}

/// Minimal SD card driver wrapper.
pub struct SdDriver<SPI, D>
where
    SPI: embedded_hal::spi::SpiDevice,
    D: DelayNs,
{
    vm: RefCell<VolumeManager<sdmmc::SdCard<SPI, D>, SdTimeSource, 4, 4, 1>>,
}

impl<SPI, D> SdDriver<SPI, D>
where
    SPI: embedded_hal::spi::SpiDevice,
    D: DelayNs,
{
    pub fn new(spi_dev: SPI, delay: D) -> Result<Self, FsError<sdmmc::SdCardError>> {
    let sd = sdmmc::SdCard::new(spi_dev, delay);
    let bytes = sd.num_bytes()?;
        info!("SD: card size {} bytes", bytes);
        let ts = SdTimeSource {};
        let vm: VolumeManager<_, _, 4, 4, 1> = VolumeManager::new(sd, ts);
        Ok(Self { vm: RefCell::new(vm) })
    }

    /// Initialize the card and verify we can open volume 0.
    pub fn init(&self) -> Result<(), FsError<sdmmc::SdCardError>> {
    let vm = self.vm.borrow_mut();
    let _v = vm.open_volume(VolumeIdx(0))?;
        Ok(())
    }

    /// Create or append a text file in root and write the provided data.
    pub fn append_text(&self, path8_3: &str, data: &[u8]) -> Result<(), FsError<sdmmc::SdCardError>> {
    let vm = self.vm.borrow_mut();
    let vol = vm.open_volume(VolumeIdx(0))?;
    let root = vol.open_root_dir()?;
    let file = root.open_file_in_dir(path8_3, Mode::ReadWriteCreateOrAppend)?;
        file.write(data)?;
        file.flush()?;
        Ok(())
    }
}
