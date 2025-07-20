// fc/mcp2515_driver.rs
#![allow(dead_code)]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

pub trait SimpleSpi {
    type Error;
    fn transfer_in_place(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error>;
}

#[derive(Debug, Clone)]
pub struct CanMessage {
    pub id: u32,
    pub dlc: u8,
    pub data: [u8; 8],
    pub is_extended: bool,
}

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

const CMD_RESET: u8 = 0xC0;
const CMD_READ: u8 = 0x03;
const CMD_WRITE: u8 = 0x02;
const CMD_RTS: u8 = 0x80;
const CMD_READ_STATUS: u8 = 0xA0;
const CMD_BIT_MODIFY: u8 = 0x05;

const MODE_NORMAL: u8 = 0x00;
const MODE_LOOPBACK: u8 = 0x40;
const MODE_CONFIG: u8 = 0x80;

const TXREQ: u8 = 0x08;
const RX0IF: u8 = 0x01;
const RX1IF: u8 = 0x02;
const RXB0CTRL_RXM: u8 = 0x60;

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

#[derive(Debug, defmt::Format)]
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
        self.reset()?;
        self.timer.delay_ms(10);
        if !self.check_communication()? {
            return Err(Mcp2515Error::InitError);
        }
        self.set_mode(MODE_CONFIG)?;
        self.write_register(CNF1, 0x03)?;
        self.write_register(CNF2, 0x90)?;
        self.write_register(CNF3, 0x02)?;
        self.write_register(CANINTE, RX0IF)?;
        self.write_register(CANINTF, 0x00)?;
        self.bit_modify(CANCTRL, 0x08, 0x08)?; // Enable one-shot mode
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
        let canstat = self.read_register(CANSTAT)?;
        let mode = canstat & 0xE0;
        Ok(mode == MODE_CONFIG)
    }

    pub fn set_mode(&mut self, mode: u8) -> Result<(), Mcp2515Error> {
        self.bit_modify(CANCTRL, 0xE0, mode)?;
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

    pub fn send_message(&mut self, msg: &CanMessage) -> Result<(), Mcp2515Error> {
        if msg.is_extended || msg.dlc > 8 {
            return Err(Mcp2515Error::InvalidData);
        }
        let status = self.read_register(TXB0CTRL)?;
        if status & TXREQ != 0 {
            return Err(Mcp2515Error::NotReady);
        }
        let sidh = ((msg.id >> 3) & 0xFF) as u8;
        let sidl = ((msg.id & 0x07) << 5) as u8;
        self.write_register(TXB0SIDH, sidh)?;
        self.write_register(TXB0SIDL, sidl)?;
        self.write_register(TXB0DLC, msg.dlc)?;
        for (i, &byte) in msg.data.iter().enumerate().take(msg.dlc as usize) {
            self.write_register(TXB0DATA + i as u8, byte)?;
        }
        self.write_register(TXB0CTRL, TXREQ)?;
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

    pub fn clear_interrupt_flag(&mut self, flag: u8) -> Result<(), Mcp2515Error> {
        self.bit_modify(CANINTF, flag, 0x00)
    }

    pub fn has_message(&mut self) -> Result<bool, Mcp2515Error> {
        let flags = self.get_interrupt_flags()?;
        Ok(flags & (RX0IF | RX1IF) != 0)
    }

    pub fn receive_message(&mut self) -> Result<Option<CanMessage>, Mcp2515Error> {
        let flags = self.get_interrupt_flags()?;
        if flags & RX0IF != 0 {
            let id_high = self.read_register(RXB0SIDH)?;
            let id_low = self.read_register(RXB0SIDL)?;
            let dlc_reg = self.read_register(RXB0DLC)?;
            let id = ((id_high as u16) << 3) | ((id_low as u16) >> 5);
            let dlc = dlc_reg & 0x0F;
            let mut data = [0u8; 8];
            for i in 0..core::cmp::min(dlc as usize, 8) {
                data[i] = self.read_register(RXB0DATA + i as u8)?;
            }
            self.clear_interrupt_flag(RX0IF)?;
            Ok(Some(CanMessage { id: id as u32, dlc, data, is_extended: false }))
        } else {
            Ok(None)
        }
    }

    pub fn configure_receive_all(&mut self) -> Result<(), Mcp2515Error> {
        self.write_register(RXB0CTRL, RXB0CTRL_RXM)?;
        Ok(())
    }

    pub fn test_basic_functionality(&mut self) -> Result<(), Mcp2515Error> {
        if !self.check_communication()? {
            return Err(Mcp2515Error::InitError);
        }
        self.set_mode(MODE_LOOPBACK)?;
        let test_msg = CanMessage {
            id: 0x123,
            dlc: 8,
            data: [0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE],
            is_extended: false,
        };
        self.send_message(&test_msg)?;
        let mut timeout = 100;
        while timeout > 0 && !self.check_transmission_complete()? {
            self.timer.delay_ms(1);
            timeout -= 1;
        }
        if timeout == 0 {
            return Err(Mcp2515Error::NotReady);
        }
        let error_flags = self.get_error_flags()?;
        if error_flags != 0 {
            return Err(Mcp2515Error::InvalidData);
        }
        self.set_mode(MODE_NORMAL)?;
        Ok(())
    }
}