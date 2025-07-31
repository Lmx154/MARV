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
use embedded_hal::i2c::I2c;

// Import the ICM20948 driver
mod drivers;
use drivers::icm20948::{Icm20948, ICM20948_ADDR_AD0_HIGH};
use drivers::lis3mdl::{Lis3mdl, LIS3MDL_ADDR};
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

    // Initialize timers for delays
    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut resets, &clocks);
    let mut timer1 = hal::Timer::new_timer1(pac.TIMER1, &mut resets, &clocks);

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
    tools::i2cscanner::scan_i2c_bus(&mut i2c0_manager.acquire().unwrap(), &mut uart, "I2C0");
    i2c0_manager.release();
    tools::i2cscanner::scan_i2c_bus(&mut i2c1, &mut uart, "I2C1");

    // Initialize ICM20948 with timer as delay provider
    let mut icm = Icm20948::new(timer, ICM20948_ADDR_AD0_HIGH);

    // Initialize the IMU using bus manager
    let mut bus = i2c0_manager.acquire().unwrap();
    match icm.init(&mut bus) {
        Ok(()) => {
            let _ = writeln!(uart, "ICM20948 IMU initialized\r\n");
        }
        Err(e) => {
            let _ = writeln!(uart, "ICM20948 IMU init failed: {:?}\r\n", e);
            // Continue despite failure to allow partial operation
        }
    }

    // Diagnostic: Read mag WHO_AM_I directly
    let mut mag_id_buf = [0u8; 1];
    let mag_id_result = bus.write_read(drivers::icm20948::ak09916::ADDR, &[drivers::icm20948::ak09916::WHO_AM_I], &mut mag_id_buf);
    match mag_id_result {
        Ok(()) => {
            let _ = writeln!(uart, "Mag WHO_AM_I: 0x{:02X}\r\n", mag_id_buf[0]);
        }
        Err(_) => {
            let _ = writeln!(uart, "Mag WHO_AM_I read failed\r\n");
        }
    }
    i2c0_manager.release();

    // Verify key registers post-init
    let mut bus = i2c0_manager.acquire().unwrap();
    let pwr_mgmt_1 = icm.read_register(&mut bus, drivers::icm20948::registers::PWR_MGMT_1).unwrap_or(0xFF);
    let _ = writeln!(uart, "PWR_MGMT_1 read-back: 0x{:02X}\r\n", pwr_mgmt_1);  // Should be 0x01
    // Switch to Bank 2 to check configs
    let _ = icm.write_register(&mut bus, drivers::icm20948::registers::REG_BANK_SEL, 0x20);
    let gyro_config_1 = icm.read_register(&mut bus, drivers::icm20948::registers::GYRO_CONFIG_1).unwrap_or(0xFF);
    let _ = writeln!(uart, "GYRO_CONFIG_1 read-back: 0x{:02X}\r\n", gyro_config_1);  // Should be 0x00
    let accel_config = icm.read_register(&mut bus, drivers::icm20948::registers::ACCEL_CONFIG).unwrap_or(0xFF);
    let _ = writeln!(uart, "ACCEL_CONFIG read-back: 0x{:02X}\r\n", accel_config);  // Should be 0x00
    // Back to Bank 0
    let _ = icm.write_register(&mut bus, drivers::icm20948::registers::REG_BANK_SEL, 0x00);
    i2c0_manager.release();

    // Initialize LIS3MDL with second timer
    let mut lis = Lis3mdl::new(timer1, LIS3MDL_ADDR);
    let mut bus = i2c0_manager.acquire().unwrap();
    match lis.init(&mut bus) {
        Ok(()) => {
            let _ = writeln!(uart, "LIS3MDL initialized\r\n");
        }
        Err(e) => {
            let _ = writeln!(uart, "LIS3MDL init failed: {:?}\r\n", e);
            // Continue despite failure
        }
    }
    i2c0_manager.release();

    icm.delay.delay_ms(200u32); // Delay after init

    // Main loop: Read and send IMU data, scan I2C0 every 5 seconds
    let mut scan_counter: u32 = 0;
    loop {
        let mut bus = i2c0_manager.acquire().unwrap();
        // Check data ready status
        let int_status = icm.read_register(&mut bus, 0x1A).unwrap_or(0x00);  // INT_STATUS (Bank 0, 0x1A), bit 0 = RAW_DATA_RDY
        let _ = writeln!(uart, "Data ready status: 0x{:02X}\r\n", int_status);
        if int_status & 0x01 == 0 {
            let _ = writeln!(uart, "Warning: No new data ready\r\n");
        }

        match icm.read_raw(&mut bus) {
            Ok(raw) => {
                let mag1_res = lis.read_raw(&mut bus);
                match mag1_res {
                    Ok(mag1) => {
                        let _ = writeln!(
                            uart,
                            "IMU1: accel {}, {}, {}; gyro {}, {}, {}; mag {}, {}, {} ; MAG1: {}, {}, {}\r\n",
                            raw.accel[0], raw.accel[1], raw.accel[2],
                            raw.gyro[0], raw.gyro[1], raw.gyro[2],
                            raw.mag[0], raw.mag[1], raw.mag[2],
                            mag1[0], mag1[1], mag1[2]
                        );
                    }
                    Err(_) => {
                        let _ = writeln!(
                            uart,
                            "IMU1: accel {}, {}, {}; gyro {}, {}, {}; mag {}, {}, {}\r\n",
                            raw.accel[0], raw.accel[1], raw.accel[2],
                            raw.gyro[0], raw.gyro[1], raw.gyro[2],
                            raw.mag[0], raw.mag[1], raw.mag[2]
                        );
                        let _ = writeln!(uart, "Failed to read LIS3MDL\r\n");
                    }
                }
            }
            Err(e) => {
                let _ = writeln!(uart, "Failed to read IMU: {:?}\r\n", e);
            }
        }
        i2c0_manager.release();

        // Delay for 1 second
        icm.delay.delay_ms(1000u32);

        // Increment counter and scan every 5 iterations
        scan_counter += 1;
        if scan_counter >= 5 {
            let mut bus = i2c0_manager.acquire().unwrap();
            tools::i2cscanner::scan_i2c_bus(&mut bus, &mut uart, "I2C0 (periodic)");
            i2c0_manager.release();
            scan_counter = 0;
        }
    }
}