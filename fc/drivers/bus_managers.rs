use rp235x_hal as hal;
use hal::i2c::I2c as HalI2c;
use hal::i2c::Instance as I2cInstance;
use hal::spi::{Spi as HalSpi, Enabled as SpiEnabled};
use hal::spi::SpiDevice as SpiInstance;
use hal::uart::{UartPeripheral, Enabled as UartEnabled};
use hal::uart::UartDevice as UartInstance;

/// Generic bus manager for I2C peripherals.
/// 
/// This manager ensures exclusive access to the I2C bus, preventing conflicts
/// when multiple devices share the same bus (e.g., BMM350 and ICM-20948 on I2C0).
/// It follows the pseudo code pattern from the implementation document, using
/// an in_use flag for O(1) acquire/release operations.
/// 
/// Usage in RTIC: Place instances in shared resources (e.g., one for I2C0, one for I2C1).
/// In tasks: lock the manager, acquire the bus, perform operations, then release.
pub struct I2cBusManager<P: I2cInstance, PINS> {
    i2c: HalI2c<P, PINS>,
    in_use: bool,
}

impl<P: I2cInstance, PINS> I2cBusManager<P, PINS> {
    /// Creates a new I2C bus manager.
    pub fn new(i2c: HalI2c<P, PINS>) -> Self {
        Self { i2c, in_use: false }
    }

    /// Acquires exclusive access to the I2C peripheral.
    /// Returns Some(&mut I2C) if available, None if in use.
    /// 
    /// Note: To handle potential stuck slaves, users should implement timeouts
    /// in the calling code and call release() if needed, per rp235x-hal guidelines.
    /// Future enhancements could include automatic recovery (e.g., bus reset).
    pub fn acquire(&mut self) -> Option<&mut HalI2c<P, PINS>> {
        if self.in_use {
            None
        } else {
            self.in_use = true;
            Some(&mut self.i2c)
        }
    }

    /// Releases the I2C peripheral for other users.
    pub fn release(&mut self) {
        self.in_use = false;
    }
}

/// Generic bus manager for SPI peripherals.
/// 
/// This manager ensures exclusive access to the SPI bus, important for shared buses
/// like SPI0 (BMI088 gyro/accel with separate CS). SD card and CAN may use separate
/// instances, but the manager pattern remains consistent.
/// 
/// Follows the implementation document's approach for O(1) operations and RTIC integration.
pub struct SpiBusManager<SPI: SpiInstance, PINS, const SIZE: usize> {
    spi: HalSpi<SpiEnabled, SPI, PINS, SIZE>,
    in_use: bool,
}

impl<SPI: SpiInstance, PINS, const SIZE: usize> SpiBusManager<SPI, PINS, SIZE> {
    /// Creates a new SPI bus manager.
    pub fn new(spi: HalSpi<SpiEnabled, SPI, PINS, SIZE>) -> Self {
        Self { spi, in_use: false }
    }

    /// Acquires exclusive access to the SPI peripheral.
    /// Returns Some(&mut Spi) if available, None if in use.
    /// 
    /// For DMA support (as per architecture document), users can initiate DMA transfers
    /// after acquiring the bus. Check status flags on completion to handle overflows.
    pub fn acquire(&mut self) -> Option<&mut HalSpi<SpiEnabled, SPI, PINS, SIZE>> {
        if self.in_use {
            None
        } else {
            self.in_use = true;
            Some(&mut self.spi)
        }
    }

    /// Releases the SPI peripheral for other users.
    pub fn release(&mut self) {
        self.in_use = false;
    }
}

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
