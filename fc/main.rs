// fc/main.rs (Updated)

#![no_std]
#![no_main]

use rp235x_hal as hal;
use hal::clocks::Clock;
use hal::entry;
use core::fmt::Write;
use panic_halt as _;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, PullUp};
use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;

// Import the BMM350 driver
mod drivers;
use drivers::bmm350::{Bmm350, RawMag};
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

    // Extended delay for stabilization after power-on
    timer.delay_ms(500u32);

    // Configure I2C0 pins: GP20 (SDA), GP21 (SCL)
    let i2c0_sda = pins.gpio20.into_pull_type::<PullUp>().into_function::<FunctionI2C>();
    let i2c0_scl = pins.gpio21.into_pull_type::<PullUp>().into_function::<FunctionI2C>();

    // Initialize I2C0 at reduced 50 kHz for stability
    let i2c0 = hal::i2c::I2C::i2c0(
        pac.I2C0,
        i2c0_sda,
        i2c0_scl,
        50u32.kHz(),
        &mut resets,
        clocks.system_clock.freq(),
    );

    // Wrap in bus manager
    let mut i2c0_manager = I2cBusManager::<hal::pac::I2C0, hal::gpio::bank0::Gpio20, hal::gpio::bank0::Gpio21>::new(i2c0);

    // Configure I2C1 pins: GP2 (SDA), GP3 (SCL) - kept for completeness
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

    // Initialize BMM350
    let mut bmm = Bmm350::new(0x14u8);

    // Initialize the sensor using bus manager
    let mut bus = i2c0_manager.acquire().unwrap();
    match bmm.init(&mut bus, &mut timer, &mut uart) {
        Ok(()) => {
            let _ = writeln!(uart, "BMM350 initialized\r\n");
        }
        Err(e) => {
            let _ = writeln!(uart, "BMM350 init failed: {:?}\r\n", e);
        }
    }

    // Detailed troubleshooting outputs: Read key registers post-init
    let mut buf = [0u8; 1];
    // CHIP_ID (should be 0x33)
    let chip_id_result = bus.write_read(0x14u8, &[0x00u8], &mut buf);
    match chip_id_result {
        Ok(()) => {
            let _ = writeln!(uart, "BMM350 CHIP_ID: 0x{:02X}\r\n", buf[0]);
        }
        Err(_) => {
            let _ = writeln!(uart, "BMM350 CHIP_ID read failed\r\n");
        }
    }
    // ERR_REG (should be 0x00)
    let err_reg_result = bus.write_read(0x14u8, &[0x02u8], &mut buf);
    match err_reg_result {
        Ok(()) => {
            let _ = writeln!(uart, "BMM350 ERR_REG: 0x{:02X}\r\n", buf[0]);
        }
        Err(_) => {
            let _ = writeln!(uart, "BMM350 ERR_REG read failed\r\n");
        }
    }
    // PMU_CMD_AGGR_SET (should be 0x14)
    let aggr_set_result = bus.write_read(0x14u8, &[0x04u8], &mut buf);
    match aggr_set_result {
        Ok(()) => {
            let _ = writeln!(uart, "BMM350 PMU_CMD_AGGR_SET: 0x{:02X}\r\n", buf[0]);
        }
        Err(_) => {
            let _ = writeln!(uart, "BMM350 PMU_CMD_AGGR_SET read failed\r\n");
        }
    }
    // PMU_CMD_AXIS_EN (should be 0x07)
    let axis_en_result = bus.write_read(0x14u8, &[0x05u8], &mut buf);
    match axis_en_result {
        Ok(()) => {
            let _ = writeln!(uart, "BMM350 PMU_CMD_AXIS_EN: 0x{:02X}\r\n", buf[0]);
        }
        Err(_) => {
            let _ = writeln!(uart, "BMM350 PMU_CMD_AXIS_EN read failed\r\n");
        }
    }
    // PMU_CMD_STATUS_0 (busy bit should be 0)
    let status_0_result = bus.write_read(0x14u8, &[0x07u8], &mut buf);
    match status_0_result {
        Ok(()) => {
            let _ = writeln!(uart, "BMM350 PMU_CMD_STATUS_0: 0x{:02X}\r\n", buf[0]);
        }
        Err(_) => {
            let _ = writeln!(uart, "BMM350 PMU_CMD_STATUS_0 read failed\r\n");
        }
    }
    i2c0_manager.release();

    timer.delay_ns(200_000_000u32); // Delay after init (200ms)

    // Main loop: Read and send sensor data at 1 Hz
    let mut scan_counter: u32 = 0;
    loop {
        let mut bus = i2c0_manager.acquire().unwrap();
        // Check INT_STATUS for data ready (for debugging)
        let mut status_buf = [0u8; 1];
        let int_status_result = bus.write_read(0x14u8, &[0x30u8], &mut status_buf);
        match int_status_result {
            Ok(()) => {
                let _ = writeln!(uart, "BMM350 INT_STATUS: 0x{:02X}\r\n", status_buf[0]);
            }
            Err(_) => {
                let _ = writeln!(uart, "BMM350 INT_STATUS read failed\r\n");
            }
        }

        match bmm.read_raw(&mut bus, &mut uart) {
            Ok(raw) => {
                let _ = writeln!(
                    uart,
                    "MAG1: {}, {}, {}\r\n",
                    raw.x, raw.y, raw.z
                );
            }
            Err(e) => {
                let _ = writeln!(uart, "Failed to read BMM350: {:?}\r\n", e);
            }
        }
        i2c0_manager.release();

        // Delay for 1 second (1 Hz reporting)
        timer.delay_ms(1000u32);

        // Increment counter (scan disabled)
        scan_counter += 1;
        if scan_counter >= 5 {
            scan_counter = 0;
        }
    }
}