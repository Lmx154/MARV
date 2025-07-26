// fc/drivers/bus_managers.rs
use rp235x_hal as hal;
use hal::i2c::{I2C as HalI2C, Error as I2cError};
use hal::i2c::I2cDevice as I2cInstance;
use hal::gpio::{Pin, PinId, FunctionI2C, PullUp};

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