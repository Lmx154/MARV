// fc/drivers/bus_managers.rs
use rp235x_hal as hal;
use hal::i2c::{I2C as HalI2C, Error as I2cError};
use hal::i2c::I2cDevice as I2cInstance;
use hal::gpio::{Pin, PinId, FunctionI2C, PullUp, FunctionUart, PullNone};
use hal::spi::{Spi as HalSpi, Enabled as SpiEnabled};
use hal::spi::SpiDevice as SpiInstance;
use hal::uart::{UartPeripheral, Enabled as UartEnabled, ValidPinTx, ValidPinRx};
use hal::uart::UartDevice as UartInstance;

pub struct I2cBusManager<P: I2cInstance, SDA: PinId, SCL: PinId> {
    i2c: HalI2C<P, (Pin<SDA, FunctionI2C, PullUp>, Pin<SCL, FunctionI2C, PullUp>)>,
    in_use: bool,
}

impl<P: I2cInstance, SDA: PinId, SCL: PinId> I2cBusManager<P, SDA, SCL> {
    pub fn new(i2c: HalI2C<P, (Pin<SDA, FunctionI2C, PullUp>, Pin<SCL, FunctionI2C, PullUp>)>) -> Self {
        Self { i2c, in_use: false }
    }

    pub fn acquire(&mut self) -> Option<&mut HalI2C<P, (Pin<SDA, FunctionI2C, PullUp>, Pin<SCL, FunctionI2C, PullUp>)>> {
        if self.in_use {
            None
        } else {
            self.in_use = true;
            Some(&mut self.i2c)
        }
    }

    pub fn release(&mut self) {
        self.in_use = false;
    }

    pub fn recover(&mut self) -> Result<(), I2cError> {
        Ok(())
    }
}

pub struct UartBusManager<U: UartInstance, TX: PinId, RX: PinId>
where
    Pin<TX, FunctionUart, PullNone>: ValidPinTx<U>,
    Pin<RX, FunctionUart, PullNone>: ValidPinRx<U>,
{
    uart: UartPeripheral<UartEnabled, U, (Pin<TX, FunctionUart, PullNone>, Pin<RX, FunctionUart, PullNone>)>,
    in_use: bool,
}

impl<U: UartInstance, TX: PinId, RX: PinId> UartBusManager<U, TX, RX>
where
    Pin<TX, FunctionUart, PullNone>: ValidPinTx<U>,
    Pin<RX, FunctionUart, PullNone>: ValidPinRx<U>,
{
    pub fn new(uart: UartPeripheral<UartEnabled, U, (Pin<TX, FunctionUart, PullNone>, Pin<RX, FunctionUart, PullNone>)>) -> Self {
        Self { uart, in_use: false }
    }

    pub fn acquire(&mut self) -> Option<&mut UartPeripheral<UartEnabled, U, (Pin<TX, FunctionUart, PullNone>, Pin<RX, FunctionUart, PullNone>)>> {
        if self.in_use {
            None
        } else {
            self.in_use = true;
            Some(&mut self.uart)
        }
    }

    pub fn release(&mut self) {
        self.in_use = false;
    }
}