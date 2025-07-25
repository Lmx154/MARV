use rp235x_hal as hal;
use hal::i2c::I2c as HalI2c;
use hal::i2c::Instance as I2cInstance;
use hal::spi::{Spi as HalSpi, Enabled as SpiEnabled};
use hal::spi::SpiDevice as SpiInstance;
use hal::uart::{UartPeripheral, Enabled as UartEnabled};
use hal::uart::UartDevice as UartInstance;

/// Generic bus manager for I2C peripherals.
/// Ensures exclusive access; follows O(1) acquire/release pattern.
pub struct I2cBusManager<P: I2cInstance, SDA, SCL>
where
    SDA: hal::gpio::PinId,
    SCL: hal::gpio::PinId,
{
    i2c: HalI2c<P, (hal::gpio::Pin<SDA, hal::gpio::FunctionI2C, hal::gpio::PullUp>, hal::gpio::Pin<SCL, hal::gpio::FunctionI2C, hal::gpio::PullUp>)>,
    in_use: bool,
}

impl<P: I2cInstance, SDA: hal::gpio::PinId, SCL: hal::gpio::PinId> I2cBusManager<P, SDA, SCL> {
    pub fn new(i2c: HalI2c<P, (hal::gpio::Pin<SDA, hal::gpio::FunctionI2C, hal::gpio::PullUp>, hal::gpio::Pin<SCL, hal::gpio::FunctionI2C, hal::gpio::PullUp>)>) -> Self {
        Self { i2c, in_use: false }
    }

    pub fn acquire(&mut self) -> Option<&mut HalI2c<P, (hal::gpio::Pin<SDA, hal::gpio::FunctionI2C, hal::gpio::PullUp>, hal::gpio::Pin<SCL, hal::gpio::FunctionI2C, hal::gpio::PullUp>)>> {
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

    /// Recovery for stuck I2C bus (e.g., clock stretching issues).
    pub fn recover(&mut self) -> Result<(), hal::i2c::Error> {
        self.i2c.reset(); // Placeholder; extend with rp235x-hal recovery if needed.
        Ok(())
    }
}

// Similar refinements for SpiBusManager and UartBusManager (omitted for brevity; add recover methods as applicable).
// ...existing code for SPI and UART managers...

/// Generic bus manager for UART peripherals.
/// 
/// Although UARTs (e.g., UART0 for GPS) are typically not shared, this manager
/// provides consistency with other buses and allows for future extensions like
/// DMA RX buffering. Debug UART may not need management.
/// 
/// Aligns with the project's modular design, using the same acquire/release pattern.
pub struct UartBusManager<U: UartInstance, PINS> {
    uart: UartPeripheral<UartEnabled, U, PINS>,
    in_use: bool,
}

impl<U: UartInstance, PINS> UartBusManager<U, PINS> {
    /// Creates a new UART bus manager.
    pub fn new(uart: UartPeripheral<UartEnabled, U, PINS>) -> Self {
        Self { uart, in_use: false }
    }

    /// Acquires exclusive access to the UART peripheral.
    /// Returns Some(&mut UartPeripheral) if available, None if in use.
    /// 
    /// For GPS parsing, acquire the bus, read/parse data, then release.
    pub fn acquire(&mut self) -> Option<&mut UartPeripheral<UartEnabled, U, PINS>> {
        if self.in_use {
            None
        } else {
            self.in_use = true;
            Some(&mut self.uart)
        }
    }

    /// Releases the UART peripheral for other users.
    pub fn release(&mut self) {
        self.in_use = false;
    }
}
