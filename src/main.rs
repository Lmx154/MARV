//! Raspberry Pi Pico 2 Example (optional BMI088)
//!
//! Preserves working RTT setup while allowing BMI088 via feature flag.
#![no_std]
#![no_main]

use panic_halt as _;
use rp235x_hal as hal;
use embedded_hal::digital::OutputPin;
use hal::Clock;
use defmt::*;
use defmt_rtt as _;
#[cfg(feature = "bmi088")] mod drivers;
#[cfg(feature = "bmi088")] use drivers::bmi088::Bmi088;

// Boot metadata (keep layout stable for RTT)
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 3] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"RustyPico Example (BMI088 opt)"),
];

#[hal::entry]
fn main() -> ! {
    info!("Boot start (bmi088 feature: {} )", cfg!(feature = "bmi088"));

    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    info!("Watchdog initialized");

    let clocks = hal::clocks::init_clocks_and_plls(
        12_000_000,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();
    info!("Clocks configured successfully");

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure on-board LED for blinking to confirm execution
    let mut led_pin = pins.gpio25.into_push_pull_output();
    led_pin.set_high().ok(); // Initial state

    #[cfg(feature = "bmi088")] {
        let mut ps_pin = pins.gpio5.into_push_pull_output();
        ps_pin.set_low().ok();
        info!("GPIO5 low (gyro PS)");
    }

    #[cfg(feature = "bmi088")] let mut bmi_driver_opt = {
        use embedded_hal::spi::MODE_0;
        let mosi = pins.gpio19.into_function::<hal::gpio::FunctionSpi>();
        let miso = pins.gpio16.into_function::<hal::gpio::FunctionSpi>();
        let sck = pins.gpio18.into_function::<hal::gpio::FunctionSpi>();
        let mut cs_accel = pins.gpio17.into_push_pull_output();
        let mut cs_gyro = pins.gpio22.into_push_pull_output();
        cs_accel.set_high().ok();
        cs_gyro.set_high().ok();
        let spi_freq = hal::fugit::HertzU32::from_raw(200_000);
        let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            spi_freq,
            &MODE_0,
        );
        struct BmiDelay;
        impl embedded_hal::delay::DelayNs for BmiDelay {
            fn delay_ns(&mut self, _: u32) {}
            fn delay_us(&mut self, us: u32) { for _ in 0..(us * 25) { cortex_m::asm::nop(); } }
            fn delay_ms(&mut self, ms: u32) { for _ in 0..ms { self.delay_us(1000); } }
        }
        let delay = BmiDelay;
        let mut drv = Bmi088::new(spi, cs_accel, cs_gyro, delay);
        match drv.init() {
            Ok(()) => info!("BMI088 init OK"),
            Err(e) => warn!("BMI088 init FAIL err={:?}", e),
        }
        Some(drv)
    };

    let mut counter = 0u32;
    let mut led_on = true;

    loop {
        counter += 1;
        // Blink LED to confirm loop is running
        if counter % 1000000 == 0 {
            if led_on { led_pin.set_low().ok(); } else { led_pin.set_high().ok(); }
            led_on = !led_on;
            info!("Loop tick");
        }

        #[cfg(feature = "bmi088")] if let Some(drv) = &mut bmi_driver_opt {
            if let Ok(frame) = drv.read_raw() {
                // Raw -> approximate units (see BMI088 datasheet)
                // Accel ±6g => 6g / 32768 ≈ 0.000183105g per LSB
                // Gyro 2000 dps => 2000 / 32768 ≈ 0.061035 dps per LSB
                let ax = frame.accel[0];
                let ay = frame.accel[1];
                let az = frame.accel[2];
                let gx = frame.gyro[0];
                let gy = frame.gyro[1];
                let gz = frame.gyro[2];
                info!("BMI088 Accel(raw): x={}, y={}, z={} | Gyro(raw): x={}, y={}, z={}", ax, ay, az, gx, gy, gz);
            }
        }
        // Simple delay
        for _ in 0..1_000_000 { cortex_m::asm::nop(); }
    }
}


defmt::timestamp!("{=u64:us}", {
    static mut TIMESTAMP: u64 = 0;
    unsafe {
        TIMESTAMP += 1;
        TIMESTAMP
    }
});