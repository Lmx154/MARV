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
use embedded_hal::digital::OutputPin;
use hal::Clock;

// Import defmt for debug output
use defmt::*;
use defmt_rtt as _;

// Import our hardware configuration
mod hardware;
use hardware::{HARDWARE, constants};

// GPS driver + middleware
use drivers::gps::GpsDriver;
use drivers::gps::GpsData;
use middleware::gps_api::GpsApi;

// Drivers
mod drivers;
mod middleware;
use drivers::rgb_led::{RgbLed, Polarity};
use drivers::buzzer::Buzzer;
use middleware::buzzer_api::BuzzerController;
use middleware::rgb_led_api::{RgbLedController, LedColor};
use hal::pwm::Slices;
mod tools;
use drivers::ms5611::{Ms5611, MS5611_ADDR};
// Delay trait used in custom MsDelay implementation
use middleware::ms5611_api::Ms5611Middleware;
use drivers::lis3mdl::Lis3mdl;
use middleware::lis3mdl_api::Lis3MdlMiddleware;
use drivers::icm20948::Icm20948;
use middleware::icm20948_api::Icm20948Middleware;
use drivers::bmi088::Bmi088;
use middleware::bmi088_api::Bmi088Middleware;
use hal::spi::Spi;

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

    // Timers: dedicate TIMER0 to ICM20948, TIMER1 to LIS3MDL (mirrors previously working pattern)
    let timer_icm = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    let timer_lis = hal::Timer::new_timer1(pac.TIMER1, &mut pac.RESETS, &clocks);

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

    // --- I2C SCANNER SETUP ---
    // I2C0: SCL=GP21, SDA=GP20
    let sda0 = pins.gpio20.into_pull_up_input().into_function::<hal::gpio::FunctionI2C>();
    let scl0 = pins.gpio21.into_pull_up_input().into_function::<hal::gpio::FunctionI2C>();
    let mut i2c0 = hal::i2c::I2C::i2c0(
        pac.I2C0,
        sda0,
        scl0,
        hal::fugit::HertzU32::from_raw(100_000),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );
    // I2C1: SCL=GP3, SDA=GP2
    let sda1 = pins.gpio2.into_pull_up_input().into_function::<hal::gpio::FunctionI2C>();
    let scl1 = pins.gpio3.into_pull_up_input().into_function::<hal::gpio::FunctionI2C>();
    let mut i2c1 = hal::i2c::I2C::i2c1(
        pac.I2C1,
        sda1,
        scl1,
        hal::fugit::HertzU32::from_raw(100_000),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );
    // UART1 for debug output (GP4 TX, GP5 RX) to keep UART0 free for GPS
    let tx1 = pins.gpio4.into_function::<hal::gpio::FunctionUart>();
    let rx1 = pins.gpio5.into_function::<hal::gpio::FunctionUart>();
    let mut uart_dbg = hal::uart::UartPeripheral::new(pac.UART1, (tx1, rx1), &mut pac.RESETS)
        .enable(
            hal::uart::UartConfig::new(
                hal::fugit::HertzU32::from_raw(115_200),
                hal::uart::DataBits::Eight,
                None,
                hal::uart::StopBits::One,
            ),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    // Scan both I2C busses at startup
    use tools::i2cscanner::scan_i2c_bus_summary;
    scan_i2c_bus_summary(&mut i2c1, &mut uart_dbg, "I2C1");
    scan_i2c_bus_summary(&mut i2c0, &mut uart_dbg, "I2C0");
    // Short settle delay after scanning before sensor init (coarse busy-wait ~ few hundred microseconds)
    for _ in 0..10_000 { cortex_m::asm::nop(); }

    // --- Initialize sensors (best-effort) ---

    // ICM20948 on I2C0 address 0x68
    // Use AD0=1 address 0x69 if that's what scan showed
    let mut icm_driver = Icm20948::new(timer_icm, drivers::icm20948::ICM20948_ADDR_AD0_HIGH);
    let mut icm = Icm20948Middleware::new(&mut icm_driver);
    let icm_ok = icm.init(&mut i2c0).is_ok();
    if icm_ok { info!("ICM20948 init OK"); } else { warn!("ICM20948 init FAIL"); }

    // --- BMI088 (SPI0) Bring-up (simplified minimal path) ---
    // Pins per PINOUT (do NOT change / swap per user wiring):
    //   GP19 MOSI (SPI0 TX), GP16 MISO (SPI0 RX), GP18 SCK, GP17 CS_ACCEL, GP22 CS_GYRO
    // Pre-SPI electrical test: temporarily drive GPIO16 (intended MISO) high then low to confirm we physically
    // observe the pin toggling (helps detect miswire / short). After this we reconfigure it as SPI MISO.
    // Allocate option holders for later assignment
    let (mosi, miso, sck);
    // Probe and then configure
    let mut miso_probe = pins.gpio16.into_push_pull_output();
    miso_probe.set_high().ok();
    for _ in 0..50_000 { cortex_m::asm::nop(); }
    info!("BMI088 MISO probe: drove GPIO16 HIGH (measure ~3.3V if wiring correct)");
    miso_probe.set_low().ok();
    for _ in 0..50_000 { cortex_m::asm::nop(); }
    info!("BMI088 MISO probe: drove GPIO16 LOW (measure ~0V)");
    // Convert to SPI function pins
    let miso_pin = miso_probe.into_function::<hal::gpio::FunctionSpi>();
    let mosi_pin = pins.gpio19.into_function::<hal::gpio::FunctionSpi>();
    let sck_pin  = pins.gpio18.into_function::<hal::gpio::FunctionSpi>();
    mosi = mosi_pin; miso = miso_pin; sck = sck_pin;
    use embedded_hal::spi::SpiBus as _; // bring transfer_in_place into scope
    let mut cs_accel = pins.gpio17.into_push_pull_output();
    let mut cs_gyro  = pins.gpio22.into_push_pull_output();
    cs_accel.set_high().ok();
    cs_gyro.set_high().ok();
    // Accel requires a defined low->high after power for SPI (shuttle board); enforce once.
    cs_accel.set_low().ok(); for _ in 0..20_000 { cortex_m::asm::nop(); } cs_accel.set_high().ok();
    // Create SPI0 in MODE_0 at a conservative 200 kHz.
    use embedded_hal::spi::MODE_0;
    let spi_freq = hal::fugit::HertzU32::from_raw(200_000);
    let spi0 = Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        spi_freq,
        &MODE_0,
    );
    // Quick raw ID probe (gyro then accel) before driver init.
    let mut spi0 = spi0; // make mutable
    let mut frame_g = [0x80u8, 0x00]; // gyro chip ID register | 0x80 (read)
    cs_gyro.set_low().ok();
    let gyro_xfer_ok = spi0.transfer_in_place(&mut frame_g).is_ok();
    cs_gyro.set_high().ok();
    let mut frame_a = [0x80u8, 0x00]; // accel chip ID register (same address 0x00)
    cs_accel.set_low().ok();
    let accel_xfer_ok = spi0.transfer_in_place(&mut frame_a).is_ok();
    cs_accel.set_high().ok();
    info!("BMI088 raw IDs gyro(ok={}):0x{:02X} accel(ok={}):0x{:02X}", gyro_xfer_ok, frame_g[1], accel_xfer_ok, frame_a[1]);
    // Extra electrical diagnostics: send dummy frames with varying first byte to see if any MISO bits ever go high.
    // If every returned second byte stays 0x00 across patterns, MISO is likely stuck low / not connected / device unpowered or not in SPI mode.
    let test_patterns: [u8;5] = [0xFF, 0x00, 0xAA, 0x55, 0x3C];
    for p in test_patterns {
        let mut f = [p, 0x00];
        cs_gyro.set_low().ok();
        let _ = spi0.transfer_in_place(&mut f);
        cs_gyro.set_high().ok();
        info!("BMI088 SPI line test pattern=0x{:02X} resp=0x{:02X}", p, f[1]);
        // small delay so a scope/LA can capture distinct bursts
        for _ in 0..30_000 { cortex_m::asm::nop(); }
    }
    // (Swapped CS test removed at user request; pins are fixed.)
    // Driver delay provider
    struct BmiDelay; impl embedded_hal::delay::DelayNs for BmiDelay { fn delay_ns(&mut self,_:u32){} fn delay_us(&mut self,us:u32){ for _ in 0..(us*25){ cortex_m::asm::nop(); } } fn delay_ms(&mut self,ms:u32){ for _ in 0..ms { self.delay_us(1000);} } }
    let mut bmi_driver = Bmi088::new(spi0, cs_accel, cs_gyro, BmiDelay);
    let mut bmi = Bmi088Middleware::new(&mut bmi_driver);
    let bmi_ok = match bmi.init() { Ok(()) => { info!("BMI088 init OK (MODE0)"); true }, Err(e) => { warn!("BMI088 init FAIL err={:?}", e); false } };

    // LIS3MDL on I2C0 address 0x1C
    let mut lis3_driver = Lis3mdl::new(timer_lis, drivers::lis3mdl::LIS3MDL_ADDR);
    let mut lis3 = Lis3MdlMiddleware::new(&mut lis3_driver);
    let lis3_ok = lis3.init(&mut i2c0).is_ok();
    if lis3_ok { info!("LIS3MDL init OK"); } else { warn!("LIS3MDL init FAIL"); }

    // MS5611 on I2C1 address 0x76 (explicit per your wiring)
    let mut ms5611_driver = Ms5611::new(MS5611_ADDR);
    let mut ms5611 = Ms5611Middleware::new(&mut ms5611_driver);
    struct BaroDelay;
    impl embedded_hal::delay::DelayNs for BaroDelay {
        fn delay_ns(&mut self, _ns: u32) {}
        fn delay_us(&mut self, us: u32) { for _ in 0..(us * 25) { cortex_m::asm::nop(); } }
        fn delay_ms(&mut self, ms: u32) { for _ in 0..ms { self.delay_us(1000); } }
    }
    let mut baro_delay = BaroDelay;
    let ms5611_ok = ms5611.init(&mut i2c1, &mut baro_delay).is_ok();
    if ms5611_ok {
        if let Ok(c1) = ms5611.read_prom(&mut i2c1, 0) { info!("MS5611 C1={=u16}", c1); }
        if let Ok(c2) = ms5611.read_prom(&mut i2c1, 1) { info!("MS5611 C2={=u16}", c2); }
        info!("MS5611 init OK (0x76 I2C1)");
    } else { warn!("MS5611 init FAIL (0x76 I2C1)"); }

    // Configure external RGB LED pins: R=GP14, G=GP15, B=GP27
    let r_pin = pins.gpio14.into_push_pull_output();
    let g_pin = pins.gpio15.into_push_pull_output();
    let b_pin = pins.gpio27.into_push_pull_output();
    let rgb_driver = RgbLed::new(r_pin, g_pin, b_pin, Polarity::ActiveLow);
    let mut rgb = RgbLedController::new(rgb_driver);
    rgb.set_color(LedColor::Off);
    info!("RGB LED configured on GP14 (R), GP15 (G), GP27 (B) - Common Anode");

    // --- Passive buzzer on GP26 using PWM slice 5 channel A ---
    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
    let mut pwm5 = pwm_slices.pwm5; // Slice 5 supports GPIO26 (channel A)
    pwm5.default_config();
    pwm5.set_div_int(125); // integer divider: 125MHz / 125 = 1MHz
    pwm5.set_div_frac(0);
    pwm5.set_top(999); // Revert to 1 kHz tone (1MHz tick / 1000)
    pwm5.enable();
    let mut ch_a = pwm5.channel_a; // take channel ownership
    let _buzz_pin = ch_a.output_to(pins.gpio26); // move pin into PWM function
    let mut buzzer = Buzzer::new(ch_a, 500);
    buzzer.off();
    let mut buzzer_ctl = BuzzerController::new();
    let mut loop_counter: u32 = 0; // global loop count for pattern timing
    let loops_per_second = (1_000_000 / constants::MAIN_LOOP_DELAY_US) as u32; // approx
    let mut next_demo_event = loops_per_second * 2; // start arm after ~2s
    let mut demo_phase = 0u8; // 0 -> schedule arm, 1 -> wait complete, 2 -> schedule ack

    // Initialize GPS
    let mut gps_driver = GpsDriver::new();
    // Consume UART0 and pins GP0/GP1 for GPS driver (moves ownership)
    match gps_driver.init(
        pac.UART0, pins.gpio0, pins.gpio1, &mut pac.RESETS, &clocks
    ) {
        Ok(()) => info!("GPS driver initialized (UART0 GP0/GP1)"),
        Err(_e) => { error!("GPS init failed"); }
    }
    let mut gps_api = GpsApi::new();
    
    let mut counter = 0u32;
    
    info!("� System initialized, starting main loop...");
    
    loop {
        counter += 1;
        loop_counter = loop_counter.wrapping_add(1);
        
        // Update system time every second (approximate)
        if counter % constants::TIME_UPDATE_INTERVAL == 0 { // Roughly every second based on iterations
            system_seconds += 1;
        }
        
        // Update GPS middleware
        gps_api.update(&mut gps_driver);
        
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

        // Demo pattern sequencing now via middleware controller
        if loop_counter == next_demo_event {
            match demo_phase {
                0 => { buzzer_ctl.start_arm(loop_counter, loops_per_second, 5, &mut buzzer); demo_phase = 1; }
                2 => { buzzer_ctl.start_ack(loop_counter, loops_per_second, &mut buzzer); demo_phase = 3; }
                _ => {}
            }
        }
        if buzzer_ctl.update(loop_counter, &mut buzzer) {
            if demo_phase == 1 { demo_phase = 2; next_demo_event = loop_counter + loops_per_second; }
            else if demo_phase == 3 { demo_phase = 255; }
        }
        rgb.update(loop_counter);

        // Turn tone on/off roughly every second using STATUS_PRINT_INTERVAL
        if counter % constants::STATUS_PRINT_INTERVAL == 0 {
            // Disable old toggle behavior to avoid interfering with patterns
            // if buzzer.toggle() { info!("Buzzer ON"); } else { info!("Buzzer OFF"); }
        }

        // Cycle RGB LED through extended palette every ~0.5s
        if counter % (constants::STATUS_PRINT_INTERVAL / 2) == 0 {
            let phase = (counter / (constants::STATUS_PRINT_INTERVAL / 2)) % 6;
            use middleware::rgb_led_api::LedColor::*;
            let color = match phase { 0=>Red,1=>Orange,2=>Green,3=>Cyan,4=>Blue,_=>Purple };
            rgb.set_color(color);
        }
        
        // Print system + sensor status every STATUS_PRINT_INTERVAL iterations (~1s)
        if counter % constants::STATUS_PRINT_INTERVAL == 0 {
            // Read sensors (non-blocking / best-effort) prior to print
            let mut mag_line: Option<[i16;3]> = None;
            if lis3_ok { if let Ok(m) = lis3.read(&mut i2c0) { mag_line = Some([m.x,m.y,m.z]); } }
            let mut imu_acc: Option<[i16;3]> = None;
            let mut imu_gyro: Option<[i16;3]> = None;
            let mut imu_mag: Option<[i16;3]> = None;
            if icm_ok { if let Ok(frame) = icm.read_frame(&mut i2c0) { imu_acc=Some(frame.accel); imu_gyro=Some(frame.gyro); imu_mag=Some(frame.mag);} }
            let mut baro: Option<u32> = None;
            if ms5611_ok {
                if let Ok(p) = ms5611.read_pressure(&mut i2c1, &mut baro_delay) {
                    if p.d1 != 0 { baro = Some(p.d1); } else { info!("MS5611 D1=0"); }
                } else { info!("MS5611 read error"); }
            }
            // BMI088 accel read
            let mut bmi_acc: Option<[i16;3]> = None;
            let mut bmi_gyro: Option<[i16;3]> = None;
            if bmi_ok { if let Ok(frame) = bmi.read() { bmi_acc = Some(frame.accel); bmi_gyro = Some(frame.gyro);} }
            print_system_status(system_seconds, gps_api.last(), mag_line, imu_acc, imu_gyro, imu_mag, baro, bmi_acc, bmi_gyro);
        }
        
    // Small delay to prevent excessive polling (use ICM timer now owned by driver)
    // Simple loop delay (~ coarse) to pace main loop
    for _ in 0..(constants::MAIN_LOOP_DELAY_US * 8) { cortex_m::asm::nop(); }
    }
}

/// Print system status including GPS information
/// Format: "GPS: MM/DD/YYYY, HH:MM:SS, LAT, LONG, ALT, SATS, FIX"
fn print_system_status(
    _system_seconds: u32,
    gps_data: &GpsData,
    mag_lis3: Option<[i16;3]>,
    imu_acc: Option<[i16;3]>,
    imu_gyro: Option<[i16;3]>,
    imu_mag: Option<[i16;3]>,
    baro_raw: Option<u32>,
    bmi_acc: Option<[i16;3]>,
    bmi_gyro: Option<[i16;3]>,
) {
    // Only print GPS time and data
    let lat_whole = gps_data.latitude / 10_000_000;
    let lat_frac = (gps_data.latitude % 10_000_000).abs();
    let lon_whole = gps_data.longitude / 10_000_000;
    let lon_frac = (gps_data.longitude % 10_000_000).abs();
    let alt_m = gps_data.altitude / 1000;
    info!("GPS: {:02}/{:02}/{:04} {:02}:{:02}:{:02} lat {}.{:07} lon {}.{:07} alt {}m sats {} fix {}",
        gps_data.month, gps_data.day, gps_data.year, gps_data.hour, gps_data.minute, gps_data.second,
        lat_whole, lat_frac, lon_whole, lon_frac, alt_m, gps_data.satellites, gps_data.fix_type);
    // Show a simple presence marker instead of a blank line now that raw D1 is hidden
    if let Some(raw) = baro_raw { info!("MS5611: {}", raw); } else { info!("MS5611: --"); }
    if let Some(m) = mag_lis3 { info!("LIS3MDL: x={} y={} z={}", m[0], m[1], m[2]); } else { info!("LIS3MDL: --"); }
    if let Some(a) = imu_acc { if let (Some(g), Some(mg)) = (imu_gyro, imu_mag) { info!("ICM20948: Acc[{} {} {}] Gyro[{} {} {}] Mag[{} {} {}]", a[0],a[1],a[2], g[0],g[1],g[2], mg[0],mg[1],mg[2]); } else { info!("ICM20948: Acc[{} {} {}] Gyro/Mag --", a[0],a[1],a[2]); } } else { info!("ICM20948: --"); }
    if let Some(a) = bmi_acc { info!("BMI088 Acc: {},{},{}", a[0], a[1], a[2]); } else { info!("BMI088 Acc: --"); }
    if let Some(g) = bmi_gyro { info!("BMI088 Gyro: {},{},{}", g[0], g[1], g[2]); } else { info!("BMI088 Gyro: --"); }
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