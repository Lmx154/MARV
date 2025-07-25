// fc/main.rs
#![no_std]
#![no_main]

use panic_halt as _;
use rp235x_hal as hal;
use rtic::app;
use hal::gpio::{FunctionSioOutput, Pin, bank0::Gpio25, PullNone, FunctionSpi, PullUp, FunctionUart, FunctionI2C};
use hal::timer::{Timer, CopyableTimer0, CopyableTimer1, Alarm, Interrupt};
use hal::spi::Spi;
use hal::fugit::{HertzU32, MicrosDurationU32};
use hal::i2c::I2c;
mod drivers;
use drivers::bus_managers::{I2cBusManager, UartBusManager};
use drivers::pcf8563::{Pcf8563, DateTime};
use drivers::ublox_neom9n::{GpsModule, GpsData};
use drivers::sensor_trait::SensorDriver;
use embedded_hal::digital::OutputPin;
use embedded_hal::delay::DelayNs;
use hal::uart::{self, UartConfig, UartPeripheral};
use hal::clocks::Clock;
mod mcp2515_driver;
use mcp2515_driver::{Mcp2515, SimpleSpi, CanMessage};

pub struct SpiWrapper<S> {
    spi: S,
}

impl<S> SpiWrapper<S> {
    fn new(spi: S) -> Self {
        Self { spi }
    }
}

impl<S> SimpleSpi for SpiWrapper<S>
where
    S: embedded_hal::spi::SpiBus,
{
    type Error = S::Error;
    fn transfer_in_place(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.spi.transfer_in_place(buffer)
    }
}

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[app(device = hal::pac, peripherals = true, dispatchers = [TIMER0_IRQ_1])]
mod app {
    use super::*;
    use core::fmt::{self, Write};
    use embedded_hal_nb::serial::Write as SerialWrite;

    struct UartWriter<'a, UART: SerialWrite<u8>> {
        uart: &'a mut UART,
    }

    impl<'a, UART: SerialWrite<u8>> fmt::Write for UartWriter<'a, UART> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            for byte in s.bytes() {
                nb::block!(self.uart.write(byte)).map_err(|_| fmt::Error)?;
            }
            Ok(())
        }
    }

    macro_rules! info {
        ($uart:expr, $($arg:tt)*) => { /* unchanged */ };
    }

    macro_rules! warn {
        ($uart:expr, $($arg:tt)*) => { /* unchanged */ };
    }

    macro_rules! error_macro {
        ($uart:expr, $($arg:tt)*) => { /* unchanged */ };
    }

    #[shared]
    struct Shared {
        mcp2515: Mcp2515<SpiWrapper<Spi<hal::spi::Enabled, hal::pac::SPI1, (Pin<hal::gpio::bank0::Gpio11, FunctionSpi, PullUp>, Pin<hal::gpio::bank0::Gpio12, FunctionSpi, PullUp>, Pin<hal::gpio::bank0::Gpio10, FunctionSpi, PullUp>), 8>>, Pin<hal::gpio::bank0::Gpio13, FunctionSioOutput, PullUp>, Timer<CopyableTimer1>>,
        token: bool,
        i2c_manager: I2cBusManager<hal::pac::I2C1, Gpio2, Gpio3>,
        uart_manager: UartBusManager<hal::pac::UART0, (Pin<Gpio0, FunctionUart, PullNone>, Pin<Gpio1, FunctionUart, PullNone>)>,
    }

    #[local]
    struct Local {
        timer: Timer<CopyableTimer0>,
        led_pin: Pin<Gpio25, FunctionSioOutput, PullNone>,
        error_count: u32,
        hold_timer: u32,
        uart: UartPeripheral<uart::Enabled, hal::pac::UART1, (Pin<hal::gpio::bank0::Gpio4, FunctionUart, PullNone>, Pin<hal::gpio::bank0::Gpio5, FunctionUart, PullNone>)>,
        rtc: Pcf8563,
        gps: GpsModule,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut resets = cx.device.RESETS;
        let mut watchdog = hal::Watchdog::new(cx.device.WATCHDOG);

        let clocks = hal::clocks::init_clocks_and_plls(
            12_000_000u32,
            cx.device.XOSC,
            cx.device.CLOCKS,
            cx.device.PLL_SYS,
            cx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .unwrap();

        let sio = hal::Sio::new(cx.device.SIO);
        let pins = hal::gpio::Pins::new(
            cx.device.IO_BANK0,
            cx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let tx_pin = pins.gpio4.into_pull_type::<PullNone>().into_function::<FunctionUart>();
        let rx_pin = pins.gpio5.into_pull_type::<PullNone>().into_function::<FunctionUart>();
        let uart_pins = (tx_pin, rx_pin);

        let mut uart = UartPeripheral::new(cx.device.UART1, uart_pins, &mut resets).enable(
            UartConfig::default(),
            clocks.peripheral_clock.freq(),
        ).unwrap();

        info!(&mut uart, "Early log test");

        let mut led_pin = pins.gpio25.into_pull_type::<PullNone>().into_push_pull_output();
        led_pin.set_high().unwrap(); // Start with LED on (holding token)
        let spi_sclk = pins.gpio10.into_pull_type::<PullUp>().into_function::<FunctionSpi>();
        let spi_mosi = pins.gpio11.into_pull_type::<PullUp>().into_function::<FunctionSpi>();
        let spi_miso = pins.gpio12.into_pull_type::<PullUp>().into_function::<FunctionSpi>();
        let spi = Spi::<_, _, _, 8>::new(cx.device.SPI1, (spi_mosi, spi_miso, spi_sclk))
            .init(&mut resets, &clocks.peripheral_clock, HertzU32::from_raw(1_000_000), embedded_hal::spi::MODE_0);
        let spi_wrapper = SpiWrapper::new(spi);
        let mcp2515_cs = pins.gpio13.into_pull_type::<PullUp>().into_push_pull_output();
        
        // Create a separate timer instance for MCP2515
        let mcp2515_timer = Timer::new_timer1(cx.device.TIMER1, &mut resets, &clocks);
        let mut mcp2515 = Mcp2515::new(spi_wrapper, mcp2515_cs, mcp2515_timer);

        info!(&mut uart, "Initializing MCP2515 CAN controller...");
        match mcp2515.init() {
            Ok(()) => {
                info!(&mut uart, "MCP2515 initialization successful");
                match mcp2515.configure_receive_all() {
                    Ok(()) => info!(&mut uart, "MCP2515 configured to receive all messages"),
                    Err(e) => error_macro!(&mut uart, "Failed to configure MCP2515 receive mode: {:?}", e),
                }
                match mcp2515.test_basic_functionality() {
                    Ok(()) => {
                        info!(&mut uart, "MCP2515 basic functionality test passed");
                        for _ in 0..3 {
                            led_pin.set_high().unwrap();
                            // ...existing code...
                            led_pin.set_low().unwrap();
                            // ...existing code...
                        }
                    }
                    Err(e) => {
                        warn!(&mut uart, "MCP2515 basic functionality test failed: {:?}", e);
                        for _ in 0..2 {
                            led_pin.set_high().unwrap();
                            // ...existing code...
                            led_pin.set_low().unwrap();
                            // ...existing code...
                        }
                    }
                }
            }
            Err(e) => {
                error_macro!(&mut uart, "MCP2515 initialization failed: {:?}", e);
                for _ in 0..5 {
                    led_pin.set_high().unwrap();
                    // ...existing code...
                    led_pin.set_low().unwrap();
                    // ...existing code...
                }
            }
        }

        // Add I2C1 for RTC (100 kHz)
        let i2c_sda = pins.gpio2.into_function::<FunctionI2C>().into_pull_up();
        let i2c_scl = pins.gpio3.into_function::<FunctionI2C>().into_pull_up();
        let i2c = I2c::i2c1(cx.device.I2C1, i2c_sda, i2c_scl, HertzU32::kHz(100), &mut resets, &clocks.system_clock.freq());
        let i2c_manager = I2cBusManager::new(i2c);

        // Add UART0 for GPS (38400 baud)
        let gps_tx = pins.gpio0.into_function::<FunctionUart>().into_pull_type::<PullNone>();
        let gps_rx = pins.gpio1.into_function::<FunctionUart>().into_pull_type::<PullNone>();
        let gps_uart = UartPeripheral::new(cx.device.UART0, (gps_tx, gps_rx), &mut resets).enable(
            UartConfig::new(HertzU32::Hz(38_400), uart::DataBits::Eight, None, uart::StopBits::One),
            clocks.peripheral_clock.freq(),
        ).unwrap();
        let uart_manager = UartBusManager::new(gps_uart);

        let rtc = Pcf8563;
        let gps = GpsModule::new();

        // Setup timer interrupt for sensor reads (1-second period)
        let mut timer = Timer::new_timer0(cx.device.TIMER0, &mut resets, &clocks);
        timer.enable_interrupt(Interrupt::Irq0);
        timer.arm_alarm(hal::timer::Alarm::Alarm0, MicrosDurationU32::secs(1));

        info!(&mut uart, "FC initialization complete - starting main loop");
        (Shared { mcp2515, token: true, i2c_manager, uart_manager }, Local { timer, led_pin, error_count: 0, hold_timer: 0, uart, rtc, gps })
    }
    #[task(binds = TIMER0_IRQ_0, priority = 2, shared = [i2c_manager, uart_manager], local = [timer, uart, rtc, gps])]
    fn sensor_read(mut cx: sensor_read::Context) {
        cx.local.timer.clear_interrupt(Interrupt::Irq0);

        // Read RTC.
        let mut read_ok = false;
        cx.shared.i2c_manager.lock(|manager| {
            if let Some(i2c) = manager.acquire() {
                if let Ok(dt) = cx.local.rtc.read_datetime(i2c) {
                    info!(cx.local.uart, "RTC Time: {}", dt.format());
                    read_ok = true;
                }
                manager.release();
            }
        });
        if !read_ok {
            warn!(cx.local.uart, "Failed to read RTC");
        }

        // Read GPS.
        read_ok = false;
        cx.shared.uart_manager.lock(|manager| {
            if let Some(uart) = manager.acquire() {
                cx.local.gps.update(uart);
                let data = cx.local.gps.get_last_data();
                info!(cx.local.uart, "GPS Data: {}", data.format_csv());
                read_ok = true;
                manager.release();
            }
        });
        if !read_ok {
            warn!(cx.local.uart, "Failed to read GPS");
        }

        // Re-arm alarm for next read.
        cx.local.timer.arm_alarm(hal::timer::Alarm::Alarm0, MicrosDurationU32::secs(1));
    }

    #[idle(shared = [mcp2515, token, i2c_manager, uart_manager], local = [timer, led_pin, error_count, hold_timer, uart])]
    fn idle(mut cx: idle::Context) -> ! {
        // Existing CAN monitoring loop unchanged.
        let timer = cx.local.timer;
        let led_pin = cx.local.led_pin;
        let error_count = cx.local.error_count;
        let hold_timer = cx.local.hold_timer;
        let uart = cx.local.uart;

        info!(uart, "FC entering main loop - monitoring CAN bus");
        loop {
            cx.shared.token.lock(|token| {
                if *token {
                    led_pin.set_high().unwrap();
                    *hold_timer += 1;
                    if *hold_timer >= 40 { // 2 seconds (40 * 50ms)
                        *hold_timer = 0;
                        cx.shared.mcp2515.lock(|mcp2515| {
                            let msg = CanMessage {
                                id: 0x300, // Pass from FC to Radio
                                dlc: 0,
                                data: [0; 8],
                                is_extended: false,
                            };
                            match mcp2515.send_message(&msg) {
                                Ok(()) => {
                                    info!(uart, "FC: Passed token to Radio");
                                    *token = false;
                                    led_pin.set_low().unwrap();
                                }
                                Err(e) => {
                                    error_macro!(uart, "FC: Failed to pass token: {:?}", e);
                                    *error_count += 1;
                                }
                            }
                        });
                    }
                } else {
                    led_pin.set_low().unwrap();
                }
            });

            cx.shared.mcp2515.lock(|mcp2515| {
                match mcp2515.get_error_flags() {
                    Ok(errors) => {
                        if errors != 0 {
                            warn!(uart, "FC: CAN Error flags: 0x{:02X}", errors);
                            *error_count += 1;
                            if *error_count > 10 {
                                error_macro!(uart, "FC: Too many errors, resetting MCP2515");
                                if let Err(e) = mcp2515.reset() {
                                    error_macro!(uart, "FC: Reset failed: {:?}", e);
                                } else if let Err(e) = mcp2515.init() {
                                    error_macro!(uart, "FC: Re-init failed: {:?}", e);
                                } else {
                                    *error_count = 0;
                                    info!(uart, "FC: MCP2515 reset and re-initialized");
                                }
                            }
                        }
                    }
                    Err(e) => error_macro!(uart, "FC: Failed to read error flags: {:?}", e),
                }
                match mcp2515.has_message() {
                    Ok(true) => {
                        match mcp2515.receive_message() {
                            Ok(Some(msg)) => {
                                if msg.id == 0x400 { // Pass from Radio to FC
                                    info!(uart, "FC: Received token from Radio");
                                    cx.shared.token.lock(|token| *token = true);
                                    *hold_timer = 0; // Reset hold timer on receive
                                }
                            }
                            Ok(None) => {}
                            Err(e) => {
                                error_macro!(uart, "FC: Error receiving CAN message: {:?}", e);
                                *error_count += 1;
                            }
                        }
                    }
                    Ok(false) => {}
                    Err(e) => {
                        error_macro!(uart, "FC: Error checking for CAN messages: {:?}", e);
                        *error_count += 1;
                    }
                }
            });

            timer.delay_ms(50);
        }
    }
}