#![no_std]
#![no_main]

use rp235x_hal as hal;
use hal::clocks::Clock;
use hal::entry;
use core::fmt::Write;
use panic_halt as _;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, PullUp};
use embedded_hal::delay::DelayNs;

// Import the ICM20948 driver
mod drivers;
use drivers::icm20948::{Icm20948, ICM20948_ADDR_AD0_HIGH};
mod tools;

// Boot ROM image definition (required for RP2350 boot process)
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[entry]
fn main() -> ! {
    // Take ownership of peripherals
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Initialize watchdog and clocks
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        12_000_000u32,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // Reset peripherals
    let mut resets = pac.RESETS;

    // Initialize SIO and pins
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut resets,
    );

    // Configure UART1 pins: GP4 (TX), GP5 (RX)
    let tx_pin = pins.gpio4.into_function::<hal::gpio::FunctionUart>();
    let rx_pin = pins.gpio5.into_function::<hal::gpio::FunctionUart>();

    // Initialize and enable UART1 with default configuration
    let mut uart = hal::uart::UartPeripheral::new(pac.UART1, (tx_pin, rx_pin), &mut resets).enable(
        hal::uart::UartConfig::default(),
        clocks.peripheral_clock.freq(),
    )
    .unwrap();

    // Initialize timer for delays
    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut resets, &clocks);

    // Delay for stabilization after power-on
    timer.delay_ms(200u32);

    // Configure I2C0 pins: GP20 (SDA), GP21 (SCL)
    let i2c0_sda = pins.gpio20.into_pull_type::<PullUp>().into_function::<FunctionI2C>();
    let i2c0_scl = pins.gpio21.into_pull_type::<PullUp>().into_function::<FunctionI2C>();

    // Initialize I2C0 at 100 kHz
    let mut i2c0 = hal::i2c::I2C::i2c0(
        pac.I2C0,
        i2c0_sda,
        i2c0_scl,
        100u32.kHz(),
        &mut resets,
        clocks.system_clock.freq(),
    );

    // Configure I2C1 pins: GP2 (SDA), GP3 (SCL)
    let i2c1_sda = pins.gpio2.into_pull_type::<PullUp>().into_function::<FunctionI2C>();
    let i2c1_scl = pins.gpio3.into_pull_type::<PullUp>().into_function::<FunctionI2C>();

    // Initialize I2C1 at 100 kHz
    let mut i2c1 = hal::i2c::I2C::i2c1(
        pac.I2C1,
        i2c1_sda,
        i2c1_scl,
        100u32.kHz(),
        &mut resets,
        clocks.system_clock.freq(),
    );

    // Scan I2C buses to confirm devices
    tools::i2cscanner::scan_i2c_bus(&mut i2c0, &mut uart, "I2C0");
    tools::i2cscanner::scan_i2c_bus(&mut i2c1, &mut uart, "I2C1");

    // Initialize ICM20948 with timer as delay provider
    let mut icm = Icm20948::new(timer, ICM20948_ADDR_AD0_HIGH);

    // Initialize the IMU
    match icm.init(&mut i2c0) {
        Ok(()) => {
            let _ = writeln!(uart, "ICM20948 IMU initialized\r\n");
        }
        Err(e) => {
            let _ = writeln!(uart, "ICM20948 IMU init failed: {:?}\r\n", e);
            // Continue despite failure to allow partial operation
        }
    }

    icm.delay.delay_ms(200u32); // Delay after init

    // Main loop: Read and send IMU data
    loop {
        match icm.read_raw(&mut i2c0) {
            Ok(raw) => {
                let _ = writeln!(
                    uart,
                    "IMU1: accel {}, {}, {}; gyro {}, {}, {}; mag {}, {}, {}\r\n",
                    raw.accel[0], raw.accel[1], raw.accel[2],
                    raw.gyro[0], raw.gyro[1], raw.gyro[2],
                    raw.mag[0], raw.mag[1], raw.mag[2]
                );
            }
            Err(e) => {
                let _ = writeln!(uart, "Failed to read IMU: {:?}\r\n", e);
            }
        }

        // Delay for 1 second
        icm.delay.delay_ms(1000u32);
    }
}