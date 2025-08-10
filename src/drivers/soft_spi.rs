//! Very small blocking bit-banged SPI (Mode0) for low-speed sensor bring-up.
//! Not timing precise; intended only for initial BMI088 verification until
//! hardware SPI setup is implemented.
//!
//! Assumptions:
//! - CPOL=0, CPHA=0
//! - Data captured on rising edge, output changes on falling edge.
//! - Delay loops approximate; adjust CYCLES if needed for slower clock.

use embedded_hal::digital::{OutputPin, InputPin};
use super::bmi088::SimpleBmiSpi;

pub struct SoftSpi<SCK, MOSI, MISO> {
    sck: SCK,
    mosi: MOSI,
    miso: MISO,
}

#[derive(Debug)]
pub enum SoftSpiError { Pin }

impl<SCK, MOSI, MISO> SoftSpi<SCK, MOSI, MISO>
where
    SCK: OutputPin,
    MOSI: OutputPin,
    MISO: InputPin,
{
    pub fn new(mut sck: SCK, mosi: MOSI, miso: MISO) -> Result<Self, SoftSpiError> {
        sck.set_low().map_err(|_| SoftSpiError::Pin)?;
        Ok(Self { sck, mosi, miso })
    }

    fn delay(&self) { for _ in 0..40 { cortex_m::asm::nop(); } } // crude ~ few hundred ns

    pub fn write(&mut self, bytes: &[u8]) -> Result<(), SoftSpiError> {
        for &b in bytes { self.transfer_byte(b)?; }
        Ok(())
    }

    pub fn read(&mut self, buf: &mut [u8]) -> Result<(), SoftSpiError> {
        for b in buf { *b = self.transfer_byte(0x00)?; }
        Ok(())
    }

    fn transfer_byte(&mut self, mut val: u8) -> Result<u8, SoftSpiError> {
        let mut rx: u8 = 0;
        for bit in (0..8).rev() {
            // set MOSI
            if (val & (1 << bit)) != 0 { self.mosi.set_high().map_err(|_| SoftSpiError::Pin)?; } else { self.mosi.set_low().map_err(|_| SoftSpiError::Pin)?; }
            self.delay();
            // clock high
            self.sck.set_high().map_err(|_| SoftSpiError::Pin)?;
            self.delay();
            // read MISO
            if self.miso.is_high().map_err(|_| SoftSpiError::Pin)? { rx |= 1 << bit; }
            // clock low
            self.sck.set_low().map_err(|_| SoftSpiError::Pin)?;
            self.delay();
        }
        Ok(rx)
    }
}

impl<SCK, MOSI, MISO> SimpleBmiSpi for SoftSpi<SCK, MOSI, MISO>
where
    SCK: OutputPin,
    MOSI: OutputPin,
    MISO: InputPin,
{
    type Error = SoftSpiError;
    fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> { self.write(data) }
    fn read(&mut self, data: &mut [u8]) -> Result<(), Self::Error> { self.read(data) }
}
