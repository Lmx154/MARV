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
use sensors::pcf8563::{Pcf8563, DateTime};

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
    
    // Validate hardware configuration first
    if let Err(e) = HARDWARE.validate_configuration() {
        error!("Hardware configuration error: {}", e);
        cortex_m::asm::udf(); // Trigger undefined instruction exception
    }
    
    // Print hardware configuration
    HARDWARE.print_configuration();
    
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

    // System time tracking - now will use hardware RTC as primary source
    let mut _system_seconds = 0u32; // Seconds since boot (for timing intervals)
    let mut rtc_synced_to_gps = false; // Flag to track if RTC has been synced with GPS
    
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

    // Configure LED as output using hardware configuration
    let led_pin_num = HARDWARE.led_pin();
    let mut led_pin = pins.gpio25.into_push_pull_output(); // Currently hardcoded to match HARDWARE.led_pin()
    info!("GPIO{} (LED) configured as output", led_pin_num);
    
    // Initialize GPS module using hardware configuration
    let mut gps = GpsModule::new();
    let (uart_tx_pin, uart_rx_pin) = HARDWARE.uart0_pins();
    
    // Initialize GPS with UART0 - pins are configured in hardware config
    match gps.init(pac.UART0, pins.gpio0, pins.gpio1, &mut pac.RESETS, &clocks) {
        Ok(()) => info!("GPS module initialized successfully on GP{}/GP{}", uart_tx_pin, uart_rx_pin),
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
    
    // Note: We'll initialize PCF8563 RTC after I2C scanner to avoid conflicts
    // For now, using I2C scanner to detect the RTC at address 0x51
    
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
    
    // Note: PCF8563 RTC detected at address 0x51 on I2C0
    // We'll initialize a separate I2C instance for the RTC to avoid conflicts with the scanner
    
    // Perform initial I2C scan
    info!("Performing initial I2C scan...");
    i2c_scanner.scan_and_print();
    
    // Test common device addresses specifically
    info!("Testing common I2C device addresses...");
    i2c_scanner.test_common_addresses();
    
    // Initialize PCF8563 RTC using I2C0 (detected at address 0x51)
    let mut rtc = match i2c_scanner.take_i2c0() {
        Some(i2c0) => {
            let mut pcf8563 = Pcf8563::new(i2c0);
            match pcf8563.init() {
                Ok(()) => {
                    info!("PCF8563 RTC initialized successfully on I2C0");
                    
                    // Perform diagnostic read to understand the current state
                    if let Err(e) = pcf8563.diagnostic_read() {
                        warn!("Failed to perform PCF8563 diagnostic: {:?}", e);
                    }
                    
                    // Check if RTC is running and read current time
                    match pcf8563.is_running() {
                        Ok(true) => {
                            match pcf8563.read_datetime() {
                                Ok(datetime) => {
                                    let (_, month, day, year, hour, minute, second) = datetime.format_for_defmt();
                                    info!("PCF8563 current time: {:02}/{:02}/{:04} {:02}:{:02}:{:02}", 
                                          month, day, year, hour, minute, second);
                                }
                                Err(e) => {
                                    warn!("Failed to read PCF8563 time: {:?}", e);
                                }
                            }
                        }
                        Ok(false) => {
                            warn!("PCF8563 clock is not running - will be set from GPS when available");
                        }
                        Err(e) => {
                            warn!("Failed to check PCF8563 status: {:?}", e);
                        }
                    }
                    
                    Some(pcf8563)
                }
                Err(e) => {
                    error!("Failed to initialize PCF8563 RTC: {:?}", e);
                    None
                }
            }
        }
        None => {
            error!("I2C0 not available for PCF8563 RTC");
            None
        }
    };
    
    // System time tracking - now using PCF8563 as source of truth
    // Keep system_seconds for timing intervals only
    let mut _system_seconds = 0u32; // Seconds since boot (for timing intervals)
    
    let mut counter = 0u32;
    
    info!("� System initialized, starting main loop...");
    
    loop {
        counter += 1;
        
        // Update system time every second (approximate) - keep for timing intervals
        if counter % constants::TIME_UPDATE_INTERVAL == 0 { // Roughly every second based on iterations
            _system_seconds += 1;
        }
        
        // Poll GPS for new data
        gps.update();
        
        // Sync RTC with GPS time when we have a good fix and haven't synced yet
        let gps_data = gps.get_last_data();
        if !rtc_synced_to_gps && gps_data.fix_type == 3 && gps_data.year != 0 {
            if let Some(ref mut rtc_device) = rtc {
                // Create DateTime from GPS data
                let mut gps_datetime = DateTime::from_gps(gps_data);
                gps_datetime.calculate_weekday();
                
                match rtc_device.write_datetime(&gps_datetime) {
                    Ok(()) => {
                        rtc_synced_to_gps = true;
                        info!("PCF8563 RTC synchronized with GPS time! GPS: {:02}/{:02}/{:04} {:02}:{:02}:{:02}", 
                              gps_data.month, gps_data.day, gps_data.year,
                              gps_data.hour, gps_data.minute, gps_data.second);
                        
                        // Give RTC a moment to stabilize after write
                        timer.delay_us(10_000); // 10ms delay
                        
                        // Perform diagnostic read after GPS sync to see what happened
                        info!("=== Post-GPS Sync Diagnostic ===");
                        if let Err(e) = rtc_device.diagnostic_read() {
                            warn!("Failed to perform post-sync diagnostic: {:?}", e);
                        }
                    }
                    Err(e) => {
                        error!("Failed to sync RTC with GPS: {:?}", e);
                    }
                }
            }
        }
        
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
            print_system_status(&mut rtc, &gps);
        }
        
        // Small delay to prevent excessive polling
        timer.delay_us(constants::MAIN_LOOP_DELAY_US);
    }
}

/// Print system status including GPS and RTC information
/// Format: "SYS RTC: MM/DD/YYYY, HH:MM:SS GPS: MM/DD/YYYY, HH:MM:SS, LAT, LONG, ALT, SATS, FIX"
fn print_system_status(rtc: &mut Option<Pcf8563<hal::I2C<hal::pac::I2C0, (hal::gpio::Pin<hal::gpio::bank0::Gpio4, hal::gpio::FunctionI2C, hal::gpio::PullUp>, hal::gpio::Pin<hal::gpio::bank0::Gpio5, hal::gpio::FunctionI2C, hal::gpio::PullUp>)>>>, gps: &GpsModule) {
    let gps_data = gps.get_last_data();
    
    // Try to read current time from hardware RTC
    let (rtc_month, rtc_day, rtc_year, rtc_hour, rtc_minute, rtc_second) = if let Some(ref mut rtc_device) = rtc {
        match rtc_device.read_datetime() {
            Ok(datetime) => {
                (datetime.month, datetime.day, datetime.year, datetime.hour, datetime.minute, datetime.second)
            }
            Err(_) => {
                // RTC read failed, use fallback time
                warn!("Failed to read RTC, using fallback time");
                (7, 7, 2025, 0, 0, 0) // Current date with 00:00:00 time
            }
        }
    } else {
        // No RTC available, use fallback time
        (7, 7, 2025, 0, 0, 0) // Current date with 00:00:00 time
    };
    
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