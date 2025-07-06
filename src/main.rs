//! # GPIO 'Blinky' Example with Debug Output
//!
//! This application demonstrates how to control a GPIO pin on the RP2350
//! and send debug messages via probe-rs/defmt.
//! If there is an LED connected to GPIO 25 (as on the Pico 2), it will blink.

#![no_std]
#![no_main]

// Pick one of these panic handlers:
// `panic-halt` will cause the processor to halt/stop on panic.
use panic_halt as _;

// Provide an alias for our HAL crate
use rp235x_hal as hal;

// Some things we need
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

// Import defmt for debug output
use defmt::*;
use defmt_rtt as _;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
#[hal::entry]
fn main() -> ! {
    info!("Starting RustyPi on RP2350!");
    info!("Hello, World! 🦀");
    
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    info!("Watchdog initialized");

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();
    info!("Clocks configured successfully");

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();
    info!("GPIO25 (LED) configured as output");
    
    let mut blink_count = 0u32;
    
    loop {
        info!("LED ON - Blink #{}", blink_count);
        led_pin.set_high().unwrap();
        timer.delay_ms(500);
        
        info!("LED OFF");
        led_pin.set_low().unwrap();
        timer.delay_ms(500);
        
        blink_count += 1;
        
        // Print a special message every 10 blinks
        if blink_count % 10 == 0 {
            info!("🎉 Completed {} blink cycles!", blink_count);
        }
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"RustyPico LED Blinky Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// Define a defmt timestamp (required for defmt-rtt)
defmt::timestamp!("{=u64:us}", {
    // This is a simple timestamp implementation
    // In a real application, you might use a hardware timer
    static mut TIMESTAMP: u64 = 0;
    // NOTE(unsafe) single-core, single context
    unsafe {
        TIMESTAMP += 1;
        TIMESTAMP
    }
});

// End of file