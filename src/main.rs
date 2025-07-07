//! # Basic Raspberry Pi Pico 2 Example
//!
//! This application demonstrates basic functionality on the RP2350
//! including LED blinking and system time tracking.
//!
//! ## Status:
//! - ✅ no_std embedded environment
//! - ✅ Basic LED blinking functionality
//! - ✅ System time tracking

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
use hal::Clock;

// Import defmt for debug output
use defmt::*;
use defmt_rtt as _;

// Import our hardware configuration
mod hardware;
use hardware::{HARDWARE, constants};

// Import sensors module
mod sensors;
use sensors::gps::GpsModule;

// Import scripts module
mod scripts;
use scripts::i2c_scanner::I2cScanner;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

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
        HARDWARE.xtal_frequency(),
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

    // System time tracking (simulating RTC)
    let mut system_seconds = 0u32; // Seconds since boot
    
    info!("System timer initialized");

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO25 as an output for LED
    let mut led_pin = pins.gpio25.into_push_pull_output();
    info!("GPIO{} (LED) configured as output", HARDWARE.led_pin());
    
    // Initialize GPS module
    let mut gps = GpsModule::new();
    
    // Initialize GPS with UART0 on GP0 (TX) and GP1 (RX)
    match gps.init(pac.UART0, pins.gpio0, pins.gpio1, &mut pac.RESETS, &clocks) {
        Ok(()) => info!("GPS module initialized successfully on GP0/GP1"),
        Err(e) => {
            error!("Failed to initialize GPS module: {:?}", e);
        }
    }
    
    // Initialize I2C Scanner
    let mut i2c_scanner = I2cScanner::new();
    
    // Validate I2C pin configuration matches hardware config
    if let Err(e) = I2cScanner::validate_pin_configuration() {
        error!("I2C pin configuration error: {}", e);
    }
    
    // Get I2C pin configuration from hardware config
    let (i2c0_sda_pin, i2c0_scl_pin) = HARDWARE.i2c0_pins();
    let (i2c1_sda_pin, i2c1_scl_pin) = HARDWARE.i2c1_pins();
    
    // Initialize I2C0 bus using hardware configuration
    match i2c_scanner.init_i2c0(
        pac.I2C0, 
        pins.gpio4,  // This should match i2c0_sda_pin
        pins.gpio5,  // This should match i2c0_scl_pin
        &mut pac.RESETS,
        clocks.peripheral_clock.freq()
    ) {
        Ok(()) => info!("I2C0 bus initialized successfully on GP{}/GP{}", i2c0_sda_pin, i2c0_scl_pin),
        Err(e) => error!("Failed to initialize I2C0 bus: {:?}", e),
    }
    
    // Initialize I2C1 bus using hardware configuration
    match i2c_scanner.init_i2c1(
        pac.I2C1,
        pins.gpio6,  // This should match i2c1_sda_pin
        pins.gpio7,  // This should match i2c1_scl_pin
        &mut pac.RESETS,
        clocks.peripheral_clock.freq()
    ) {
        Ok(()) => info!("I2C1 bus initialized successfully on GP{}/GP{}", i2c1_sda_pin, i2c1_scl_pin),
        Err(e) => error!("Failed to initialize I2C1 bus: {:?}", e),
    }
    
    // Perform initial I2C scan
    info!("Performing initial I2C scan...");
    i2c_scanner.scan_and_print();
    
    let mut counter = 0u32;
    
    info!("� System initialized, starting main loop...");
    
    loop {
        counter += 1;
        
        // Update system time every second (approximate)
        if counter % constants::TIME_UPDATE_INTERVAL == 0 { // Roughly every second based on iterations
            system_seconds += 1;
        }
        
        // Poll GPS for new data
        gps.update();
        
        // Blink LED every 1000 iterations
        if counter % constants::LED_BLINK_INTERVAL == 0 {
            if (counter / constants::LED_BLINK_INTERVAL) % 2 == 0 {
                led_pin.set_high().unwrap();
            } else {
                led_pin.set_low().unwrap();
            }
        }
        
        // Print system status every 10,000 iterations (approximately 1 second)
        if counter % constants::STATUS_PRINT_INTERVAL == 0 {
            print_system_status(system_seconds, &gps);
        }
        
        // Perform I2C scan every 30 seconds (300,000 iterations)
        if counter % (constants::STATUS_PRINT_INTERVAL * 30) == 0 && counter > 0 {
            info!("Performing periodic I2C scan...");
            i2c_scanner.scan_and_print();
        }
        
        // Small delay to prevent excessive polling
        timer.delay_us(constants::MAIN_LOOP_DELAY_US);
    }
}

/// Print system status including GPS information
/// Format: "SYS RTC: MM/DD/YYYY, HH:MM:SS GPS: MM/DD/YYYY, HH:MM:SS, LAT, LONG, ALT, SATS, FIX"
fn print_system_status(system_seconds: u32, gps: &GpsModule) {
    // Calculate RTC time from system seconds (starting from boot time)
    // Base time: July 6, 2025, 14:30:00
    let base_hour = 14;
    let base_minute = 30;
    let base_second = 0;
    
    let total_seconds = base_second + (base_minute * 60) + (base_hour * 3600) + system_seconds;
    let rtc_hour = (total_seconds / 3600) % 24;
    let rtc_minute = (total_seconds / 60) % 60;
    let rtc_second = total_seconds % 60;
    
    let rtc_month = 7;
    let rtc_day = 6;
    let rtc_year = 2025;
    
    let gps_data = gps.get_last_data();
    
    // Format GPS coordinates in human-readable format
    let lat_whole = gps_data.latitude / 10_000_000;
    let lat_frac = (gps_data.latitude % 10_000_000).abs();
    let lon_whole = gps_data.longitude / 10_000_000;
    let lon_frac = (gps_data.longitude % 10_000_000).abs();
    let alt_m = gps_data.altitude / 1000; // Convert mm to meters
    
    info!(
        "SYS RTC: {:02}/{:02}/{:04}, {:02}:{:02}:{:02} GPS: {:02}/{:02}/{:04}, {:02}:{:02}:{:02}, {}.{:07}, {}.{:07}, {}m, {} sats, fix:{}", 
        rtc_month, rtc_day, rtc_year, rtc_hour, rtc_minute, rtc_second,
        gps_data.month, gps_data.day, gps_data.year, gps_data.hour, gps_data.minute, gps_data.second,
        lat_whole, lat_frac, lon_whole, lon_frac, alt_m, gps_data.satellites, gps_data.fix_type
    );
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"RustyPico Basic Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// Define a defmt timestamp (required for defmt-rtt)
defmt::timestamp!("{=u64:us}", {
    // Simple timestamp implementation for no_std environment
    // In a real application, you might use a hardware timer
    static mut TIMESTAMP: u64 = 0;
    // NOTE(unsafe) single-core, single context
    unsafe {
        TIMESTAMP += 1;
        TIMESTAMP
    }
});

// End of file