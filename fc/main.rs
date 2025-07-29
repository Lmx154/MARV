
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

mod drivers;
use drivers::bmm350::Bmm350;
use drivers::bus_managers::I2cBusManager;
mod tools;

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
    let i2c0 = hal::i2c::I2C::i2c0(
        pac.I2C0,
        i2c0_sda,
        i2c0_scl,
        100u32.kHz(),
        &mut resets,
        clocks.system_clock.freq(),
    );

    // Wrap in bus manager
    let mut i2c0_manager = I2cBusManager::<hal::pac::I2C0, hal::gpio::bank0::Gpio20, hal::gpio::bank0::Gpio21>::new(i2c0);

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
    let mut bus = i2c0_manager.acquire().unwrap();
    tools::i2cscanner::scan_i2c_bus(&mut bus, &mut uart, "I2C0");
    i2c0_manager.release();
    tools::i2cscanner::scan_i2c_bus(&mut i2c1, &mut uart, "I2C1");

    // Initialize BMM350
    let mut bmm = Bmm350::new(0x14);
    let mut bus = i2c0_manager.acquire().unwrap();
    match bmm.init(&mut bus, &mut timer, &mut uart) {
        Ok(()) => {
            let _ = writeln!(uart, "BMM350 initialized successfully\r\n");
        }
        Err(e) => {
            let _ = writeln!(uart, "BMM350 init failed: {:?}", e);
            loop {} // Halt for debugging
        }
    }
    i2c0_manager.release();

    // Main loop: Read and send sensor data at ~100 Hz
    let mut scan_counter: u32 = 0;
    loop {
        let mut bus = i2c0_manager.acquire().unwrap();
        match bmm.read_raw(&mut bus, &mut uart) {
            Ok(raw) => {
                let _ = writeln!(
                    uart,
                    "MAG1: {}, {}, {}\r\n",
                    raw.x, raw.y, raw.z
                );
            }
            Err(e) => {
                let _ = writeln!(uart, "Failed to read BMM350: {:?}", e);
            }
        }
        i2c0_manager.release();

        // Delay for ~100 Hz (10ms)
        timer.delay_ms(10u32);

        // Periodic I2C scan every 5 seconds
        scan_counter += 1;
        if scan_counter >= 500 { // 500 * 10ms = 5s
            let mut bus = i2c0_manager.acquire().unwrap();
            tools::i2cscanner::scan_i2c_bus(&mut bus, &mut uart, "I2C0 (periodic)");
            i2c0_manager.release();
            scan_counter = 0;
        }
    }
}