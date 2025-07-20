#![allow(dead_code)]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

// Simple SPI trait to avoid external dependencies
pub trait SimpleSpi {
    type Error;
    fn transfer_in_place(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error>;
}

// MCP2515 Register Addresses
const RXF0SIDH: u8 = 0x00;
const RXF0SIDL: u8 = 0x01;
const CANSTAT: u8 = 0x0E;
const CANCTRL: u8 = 0x0F;
const TXB0CTRL: u8 = 0x30;
const TXB0SIDH: u8 = 0x31;
const TXB0SIDL: u8 = 0x32;
const TXB0DLC: u8 = 0x35;
const TXB0DATA: u8 = 0x36;
const RXB0CTRL: u8 = 0x60;
const RXB0SIDH: u8 = 0x61;
const RXB0SIDL: u8 = 0x62;
const RXB0DLC: u8 = 0x65;
const RXB0DATA: u8 = 0x66;
const CNF1: u8 = 0x2A;
const CNF2: u8 = 0x29;
const CNF3: u8 = 0x28;
const CANINTE: u8 = 0x2B;
const CANINTF: u8 = 0x2C;
const EFLG: u8 = 0x2D;

// MCP2515 SPI Commands
const CMD_RESET: u8 = 0xC0;
const CMD_READ: u8 = 0x03;
const CMD_WRITE: u8 = 0x02;
const CMD_RTS: u8 = 0x80; // Request to Send
const CMD_READ_STATUS: u8 = 0xA0;
const CMD_BIT_MODIFY: u8 = 0x05;

// Control Register Modes
const MODE_NORMAL: u8 = 0x00;
const MODE_SLEEP: u8 = 0x20;
const MODE_LOOPBACK: u8 = 0x40;
const MODE_LISTEN_ONLY: u8 = 0x60;
const MODE_CONFIG: u8 = 0x80;

// Status bits
const TXREQ: u8 = 0x08; // Message transmit request bit
const TXB0CNTRL_TXREQ: u8 = 0x08;

pub struct Mcp2515<SPI, CS, TIMER>
where
    SPI: SimpleSpi,
    CS: OutputPin,
    TIMER: DelayNs,
{
    spi: SPI,
    cs: CS,
    timer: TIMER,
}

#[derive(Debug)]
pub enum Mcp2515Error {
    SpiError,
    CsError,
    InitError,
    NotReady,
    InvalidData,
}

impl<SPI, CS, TIMER> Mcp2515<SPI, CS, TIMER>
where
    SPI: SimpleSpi,
    CS: OutputPin,
    TIMER: DelayNs,
{
    pub fn new(spi: SPI, cs: CS, timer: TIMER) -> Self {
        Self { spi, cs, timer }
    }

    pub fn init(&mut self) -> Result<(), Mcp2515Error> {
        // Reset the MCP2515
        self.reset()?;
        
        // Wait for reset to complete
        self.timer.delay_ms(10);

        // Check if we can communicate with the device
        if !self.check_communication()? {
            return Err(Mcp2515Error::InitError);
        }

        // Set configuration mode
        self.set_mode(MODE_CONFIG)?;

        // Configure bit timing for 125kbps @ 8MHz
        // These values assume 8MHz crystal on MCP2515
        self.write_register(CNF1, 0x03)?; // BRP = 3, SJW = 1
        self.write_register(CNF2, 0x90)?; // BTLMODE = 1, SAM = 0, PHSEG1 = 1, PRSEG = 0
        self.write_register(CNF3, 0x02)?; // PHSEG2 = 2

        // Configure interrupts (disable all for now)
        self.write_register(CANINTE, 0x00)?;

        // Clear interrupt flags
        self.write_register(CANINTF, 0x00)?;

        // Set normal mode
        self.set_mode(MODE_NORMAL)?;

        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Mcp2515Error> {
        self.cs.set_low().map_err(|_| Mcp2515Error::CsError)?;
        
        let mut buf = [CMD_RESET];
        self.spi.transfer_in_place(&mut buf).map_err(|_| Mcp2515Error::SpiError)?;
        
        self.cs.set_high().map_err(|_| Mcp2515Error::CsError)?;
        Ok(())
    }

    pub fn check_communication(&mut self) -> Result<bool, Mcp2515Error> {
        // Try to read CANSTAT register - should return a valid mode
        let canstat = self.read_register(CANSTAT)?;
        
        // Check if the mode bits make sense (should be in CONFIG mode after reset)
        let mode = canstat & 0xE0;
        Ok(mode == MODE_CONFIG)
    }

    pub fn set_mode(&mut self, mode: u8) -> Result<(), Mcp2515Error> {
        self.bit_modify(CANCTRL, 0xE0, mode)?;
        
        // Wait for mode change to take effect
        let mut timeout = 100;
        while timeout > 0 {
            let canstat = self.read_register(CANSTAT)?;
            if (canstat & 0xE0) == mode {
                return Ok(());
            }
            self.timer.delay_ms(1);
            timeout -= 1;
        }
        
        Err(Mcp2515Error::NotReady)
    }

    pub fn read_register(&mut self, address: u8) -> Result<u8, Mcp2515Error> {
        self.cs.set_low().map_err(|_| Mcp2515Error::CsError)?;
        
        let mut buf = [CMD_READ, address, 0x00];
        self.spi.transfer_in_place(&mut buf).map_err(|_| Mcp2515Error::SpiError)?;
        
        self.cs.set_high().map_err(|_| Mcp2515Error::CsError)?;
        
        Ok(buf[2])
    }

    pub fn write_register(&mut self, address: u8, data: u8) -> Result<(), Mcp2515Error> {
        self.cs.set_low().map_err(|_| Mcp2515Error::CsError)?;
        
        let mut buf = [CMD_WRITE, address, data];
        self.spi.transfer_in_place(&mut buf).map_err(|_| Mcp2515Error::SpiError)?;
        
        self.cs.set_high().map_err(|_| Mcp2515Error::CsError)?;
        
        Ok(())
    }

    pub fn bit_modify(&mut self, address: u8, mask: u8, data: u8) -> Result<(), Mcp2515Error> {
        self.cs.set_low().map_err(|_| Mcp2515Error::CsError)?;
        
        let mut buf = [CMD_BIT_MODIFY, address, mask, data];
        self.spi.transfer_in_place(&mut buf).map_err(|_| Mcp2515Error::SpiError)?;
        
        self.cs.set_high().map_err(|_| Mcp2515Error::CsError)?;
        
        Ok(())
    }

    pub fn read_status(&mut self) -> Result<u8, Mcp2515Error> {
        self.cs.set_low().map_err(|_| Mcp2515Error::CsError)?;
        
        let mut buf = [CMD_READ_STATUS, 0x00];
        self.spi.transfer_in_place(&mut buf).map_err(|_| Mcp2515Error::SpiError)?;
        
        self.cs.set_high().map_err(|_| Mcp2515Error::CsError)?;
        
        Ok(buf[1])
    }

    pub fn send_test_frame(&mut self) -> Result<(), Mcp2515Error> {
        // Check if TX buffer 0 is free
        let status = self.read_register(TXB0CTRL)?;
        if status & TXREQ != 0 {
            return Err(Mcp2515Error::NotReady);
        }

        // Set up a test CAN frame
        // Standard ID: 0x123
        self.write_register(TXB0SIDH, 0x24)?; // ID bits 10-3
        self.write_register(TXB0SIDL, 0x60)?; // ID bits 2-0, no extended frame

        // Data length: 8 bytes
        self.write_register(TXB0DLC, 0x08)?;

        // Test data
        let test_data = [0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE];
        for (i, &byte) in test_data.iter().enumerate() {
            self.write_register(TXB0DATA + i as u8, byte)?;
        }

        // Request transmission
        self.write_register(TXB0CTRL, TXB0CNTRL_TXREQ)?;

        Ok(())
    }

    pub fn check_transmission_complete(&mut self) -> Result<bool, Mcp2515Error> {
        let status = self.read_register(TXB0CTRL)?;
        Ok((status & TXREQ) == 0)
    }

    pub fn get_error_flags(&mut self) -> Result<u8, Mcp2515Error> {
        self.read_register(EFLG)
    }

    pub fn get_interrupt_flags(&mut self) -> Result<u8, Mcp2515Error> {
        self.read_register(CANINTF)
    }

    pub fn test_basic_functionality(&mut self) -> Result<(), Mcp2515Error> {
        // Test 1: Check communication
        if !self.check_communication()? {
            return Err(Mcp2515Error::InitError);
        }

        // Test 2: Set loopback mode for self-test
        self.set_mode(MODE_LOOPBACK)?;

        // Test 3: Send a test frame
        self.send_test_frame()?;

        // Test 4: Wait for transmission to complete
        let mut timeout = 100;
        while timeout > 0 && !self.check_transmission_complete()? {
            self.timer.delay_ms(1);
            timeout -= 1;
        }

        if timeout == 0 {
            return Err(Mcp2515Error::NotReady);
        }

        // Test 5: Check for errors
        let error_flags = self.get_error_flags()?;
        if error_flags != 0 {
            return Err(Mcp2515Error::InvalidData);
        }

        // Return to normal mode
        self.set_mode(MODE_NORMAL)?;

        Ok(())
    }
}
