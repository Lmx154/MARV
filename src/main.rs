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

// Import defmt for debug output
use defmt::*;
use defmt_rtt as _;

// Import our hardware configuration
mod hardware;
use hardware::{HARDWARE, constants};

// Import sensors module
mod sensors;
use sensors::gps::GpsModule;

// Drivers
mod drivers;
use drivers::rgb_led::{RgbLed, Polarity};
use embedded_hal::pwm::SetDutyCycle;
use hal::pwm::Slices;

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

    // Configure GPIO25 as an output for on-board LED
    let mut led_pin = pins.gpio25.into_push_pull_output();
    info!("GPIO{} (LED) configured as output", HARDWARE.led_pin());

    // Configure external RGB LED pins: R=GP14, G=GP15, B=GP27
    let r_pin = pins.gpio14.into_push_pull_output();
    let g_pin = pins.gpio15.into_push_pull_output();
    let b_pin = pins.gpio27.into_push_pull_output();
    let mut rgb = RgbLed::new(r_pin, g_pin, b_pin, Polarity::ActiveLow);
    rgb.off();
    info!("RGB LED configured on GP14 (R), GP15 (G), GP27 (B) - Common Anode");

    // --- Passive buzzer on GP26 using PWM slice 5 channel A ---
    // Goal: 1 kHz square wave @ 50% duty toggled on/off every second
    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
    let mut pwm5 = pwm_slices.pwm5; // Slice 5 supports GPIO26 (channel A)
    pwm5.default_config();
    // System clock ~125MHz. Divide by 125 -> 1MHz tick. TOP=999 -> 1kHz.
    pwm5.set_div_int(125); // integer divider
    pwm5.set_div_frac(0);
    pwm5.set_top(999); // counts 0..999 (1000 cycles)
    // Enable slice before configuring channel outputs
    pwm5.enable();
    // Attach channel A to GPIO26 (channel A corresponds to even pin in slice group containing GPIO26)
    let mut ch_a = pwm5.channel_a; // take channel ownership
    let _buzz_pin = ch_a.output_to(pins.gpio26); // move pin into PWM function
    // Prepare 50% duty (will be enabled when tone_on true)
    let _ = ch_a.set_duty_cycle(500); // half of (TOP+1) = 1000
    let mut tone_on = false; // start silent
    // Ensure silent initially by setting duty 0
    let _ = ch_a.set_duty_cycle(0);
    
    // Initialize GPS module
    let mut gps = GpsModule::new();
    
    // Initialize GPS with UART0 on GP0 (TX) and GP1 (RX)
    match gps.init(pac.UART0, pins.gpio0, pins.gpio1, &mut pac.RESETS, &clocks) {
        Ok(()) => info!("GPS module initialized successfully on GP0/GP1"),
        Err(e) => {
            error!("Failed to initialize GPS module: {:?}", e);
        }
    }
    
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
        
        // Blink on-board LED every 1000 iterations
        if counter % constants::LED_BLINK_INTERVAL == 0 {
            if (counter / constants::LED_BLINK_INTERVAL) % 2 == 0 {
                led_pin.set_high().unwrap();
            } else {
                led_pin.set_low().unwrap();
            }
            // Toggle buzzer once per LED blink interval (approx 0.1s currently?)
            // For "very basic test" make it on/off every status print (~1s). Accumulate using tone_on.
        }

        // Turn tone on/off roughly every second using STATUS_PRINT_INTERVAL
        if counter % constants::STATUS_PRINT_INTERVAL == 0 {
            tone_on = !tone_on;
            if tone_on {
                let _ = ch_a.set_duty_cycle(500); // 50%
            } else {
                let _ = ch_a.set_duty_cycle(0); // silence
            }
        }

        // Cycle RGB LED: Red -> Green -> Blue, each for ~500ms
        if counter % (constants::STATUS_PRINT_INTERVAL / 2) == 0 {
            let phase = (counter / (constants::STATUS_PRINT_INTERVAL / 2)) % 3;
            match phase {
                0 => {
                    rgb.red();
                }
                1 => {
                    rgb.green();
                }
                _ => {
                    rgb.blue();
                }
            }
        }
        
        // Print system status every 10,000 iterations (approximately 1 second)
        if counter % constants::STATUS_PRINT_INTERVAL == 0 {
            print_system_status(system_seconds, &gps);
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
        "SYS RTC: {:02}/{:02}/{:04}, {:02}:{:02}:{:02} GPS: {:02}/{:02}/{:04}, {:02}:{:02}:{:02}, {}.{:07}°, {}.{:07}°, {}m, {} sats, fix:{}", 
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