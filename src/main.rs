//! # GPS Example with u-blox NEO M9N
//!
//! This application demonstrates how to parse UBX packets from a GPS module
//! and display the data in a readable format including RTC time.
//! GPS is connected to UART0: GP0 (TX), GP1 (RX)
//!
//! ## Status:
//! - ✅ GPS UBX packet parser implemented
//! - ✅ Real UART GPS communication active
//! - ✅ Display format: "RTC Time: mm/dd/yyyy, hh:mm:ss, latitude, longitude, altitude, satellites"
//! - ✅ no_std embedded environment
//! - ✅ Basic LED blinking functionality

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
use embedded_hal_nb::serial::Read;
use hal::fugit::RateExtU32;

// Import defmt for debug output
use defmt::*;
use defmt_rtt as _;

// Import our GPS module
mod sensors;
use sensors::{GpsData, UbxParser};

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
#[hal::entry]
fn main() -> ! {
    info!("Starting RustyPi on RP2350 with GPS!");
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
    info!("GPIO25 (LED) configured as output");
    
    // Configure UART0 for GPS communication
    // GP0 = UART0 TX, GP1 = UART0 RX
    let uart_pins = (
        pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
        pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
    );
    
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            hal::uart::UartConfig::new(
                9600_u32.Hz(),
                hal::uart::DataBits::Eight,
                None,
                hal::uart::StopBits::One,
            ),
            125_000_000_u32.Hz(), // System clock frequency
        )
        .unwrap();
    
    info!("UART0 configured for GPS (9600 baud, 8N1)");
    
    // Initialize GPS parser
    let mut gps_parser = UbxParser::new();
    let mut last_gps_data = GpsData::default();
    let mut counter = 0u32;
    
    info!("🛰️ GPS system initialized, waiting for data...");
    
    loop {
        counter += 1;
        
        // Update system time every second (approximate)
        if counter % 10000 == 0 { // Roughly every second based on iterations
            system_seconds += 1;
        }
        
        // Check for incoming GPS data
        match uart.read() {
            Ok(byte) => {
                if let Some(gps_data) = gps_parser.process_byte(byte) {
                    last_gps_data = gps_data;
                    // GPS data updated silently
                }
            }
            Err(_) => {
                // No data available, continue
            }
        }
        
        // Blink LED every 1000 iterations
        if counter % 1000 == 0 {
            if (counter / 1000) % 2 == 0 {
                led_pin.set_high().unwrap();
            } else {
                led_pin.set_low().unwrap();
            }
        }
        
        // Print GPS status every 10,000 iterations (approximately 1 second)
        if counter % 10_000 == 0 {
            print_gps_status(&last_gps_data, system_seconds);
        }
        
        // Small delay to prevent excessive polling
        timer.delay_us(100);
    }
}

/// Print GPS status in the requested format
/// Format: "RTC Time: mm/dd/yyyy, hh:mm:ss, GPS Time: mm/dd/yyyy, hh:mm:ss, latitude, longitude, altitude, satellites"
fn print_gps_status(gps_data: &GpsData, system_seconds: u32) {
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
    
    if gps_data.fix_valid {
        // Use GPS coordinates and GPS time when we have a fix
        let lat_int = (gps_data.latitude * 1000000.0) as i32;
        let lon_int = (gps_data.longitude * 1000000.0) as i32;
        
        info!(
            "RTC Time: {:02}/{:02}/{:04}, {:02}:{:02}:{:02}, GPS Time: {:02}/{:02}/{:04}, {:02}:{:02}:{:02}, {}.{:06}, {}.{:06}, {}, {}", 
            rtc_month, rtc_day, rtc_year, rtc_hour, rtc_minute, rtc_second,
            gps_data.month, gps_data.day, gps_data.year, gps_data.hour, gps_data.minute, gps_data.second,
            lat_int / 1000000, 
            (lat_int % 1000000).abs(),
            lon_int / 1000000,
            (lon_int % 1000000).abs(),
            gps_data.altitude / 1000, // Convert mm to m
            gps_data.satellites
        );
    } else {
        // Show zeros when no fix, but still show RTC time and satellite count
        info!(
            "RTC Time: {:02}/{:02}/{:04}, {:02}:{:02}:{:02}, GPS Time: 00/00/0000, 00:00:00, 0.000000, 0.000000, 0, {}", 
            rtc_month, rtc_day, rtc_year, rtc_hour, rtc_minute, rtc_second,
            gps_data.satellites
        );
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"RustyPico GPS UBX Parser Example"),
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