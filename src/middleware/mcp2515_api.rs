//! Middleware for MCP2515 CAN controller
//! Provides higher-level send/receive routines while delegating raw SPI/register access to driver.

use crate::drivers::mcp2515::{Mcp2515, Mcp2515Error, CanMessage};
use embedded_hal::digital::OutputPin; // For potential future abstractions
use embedded_hal::delay::DelayNs;

pub struct Mcp2515Middleware<'d, SPI, CS, T>
where
    SPI: crate::drivers::mcp2515::SimpleSpi,
    CS: OutputPin,
    T: DelayNs,
{
    driver: &'d mut Mcp2515<SPI, CS, T>,
}

impl<'d, SPI, CS, T> Mcp2515Middleware<'d, SPI, CS, T>
where
    SPI: crate::drivers::mcp2515::SimpleSpi,
    CS: OutputPin,
    T: DelayNs,
{
    pub fn new(driver: &'d mut Mcp2515<SPI, CS, T>) -> Self { Self { driver } }

    pub fn init(&mut self) -> Result<(), Mcp2515Error> { self.driver.init() }

    pub fn send(&mut self, msg: &CanMessage) -> Result<(), Mcp2515Error> { self.driver.send_message(msg) }

    pub fn poll_rx(&mut self) -> Result<Option<CanMessage>, Mcp2515Error> { self.driver.receive_message() }
}
