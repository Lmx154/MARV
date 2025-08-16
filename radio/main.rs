// radio/main.rs
#![no_std]
#![no_main]

use panic_halt as _;
// Link the defmt RTT logger for this binary and import macros
use defmt_rtt as _;
use defmt as _;
use rp235x_hal as hal;
use rtic::app;
use hal::gpio::{FunctionSioOutput, Pin, bank0::Gpio25, PullNone, FunctionSpi, PullUp, FunctionUart};
use hal::timer::{Timer, CopyableTimer0, CopyableTimer1};
use hal::spi::Spi;
use hal::fugit::HertzU32;
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

#[app(device = hal::pac, peripherals = true, dispatchers = [TIMER0_IRQ_0])]
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
        ($uart:expr, $($arg:tt)*) => {
            {
                let mut writer = UartWriter { uart: $uart };
                let _ = write!(writer, "[INFO] ");
                let _ = write!(writer, $($arg)*);
                let _ = writer.write_str("\r\n");
            }
        }
    }

    macro_rules! warn {
        ($uart:expr, $($arg:tt)*) => {
            {
                let mut writer = UartWriter { uart: $uart };
                let _ = write!(writer, "[WARN] ");
                let _ = write!(writer, $($arg)*);
                let _ = writer.write_str("\r\n");
            }
        }
    }

    macro_rules! error_macro {
        ($uart:expr, $($arg:tt)*) => {
            {
                let mut writer = UartWriter { uart: $uart };
                let _ = write!(writer, "[ERROR] ");
                let _ = write!(writer, $($arg)*);
                let _ = writer.write_str("\r\n");
            }
        }
    }

    #[shared]
    struct Shared {
        mcp2515: Mcp2515<SpiWrapper<Spi<hal::spi::Enabled, hal::pac::SPI0, (Pin<hal::gpio::bank0::Gpio19, FunctionSpi, PullUp>, Pin<hal::gpio::bank0::Gpio16, FunctionSpi, PullUp>, Pin<hal::gpio::bank0::Gpio18, FunctionSpi, PullUp>), 8>>, Pin<hal::gpio::bank0::Gpio17, FunctionSioOutput, PullUp>, Timer<CopyableTimer1>>,
        token: bool, // Shared token state
    }

    #[local]
    struct Local {
        timer: Timer<CopyableTimer0>,
        led_pin: Pin<Gpio25, FunctionSioOutput, PullNone>,
        error_count: u32,
        hold_timer: u32,
        uart: UartPeripheral<uart::Enabled, hal::pac::UART1, (Pin<hal::gpio::bank0::Gpio4, FunctionUart, PullNone>, Pin<hal::gpio::bank0::Gpio5, FunctionUart, PullNone>)>,
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

        let mut timer = Timer::new_timer0(cx.device.TIMER0, &mut resets, &clocks);
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

        let mut led_pin = pins.gpio25.into_pull_type::<PullNone>().into_push_pull_output();
        led_pin.set_low().unwrap(); // Start with LED off (not holding token)
        let spi_sclk = pins.gpio18.into_pull_type::<PullUp>().into_function::<FunctionSpi>();
        let spi_mosi = pins.gpio19.into_pull_type::<PullUp>().into_function::<FunctionSpi>();
        let spi_miso = pins.gpio16.into_pull_type::<PullUp>().into_function::<FunctionSpi>();
        let spi = Spi::<_, _, _, 8>::new(cx.device.SPI0, (spi_mosi, spi_miso, spi_sclk))
            .init(&mut resets, &clocks.peripheral_clock, HertzU32::from_raw(1_000_000), embedded_hal::spi::MODE_0);
        let spi_wrapper = SpiWrapper::new(spi);
        let mcp2515_cs = pins.gpio17.into_pull_type::<PullUp>().into_push_pull_output();
        
        // Create a separate timer instance for MCP2515
        let mcp2515_timer = Timer::new_timer1(cx.device.TIMER1, &mut resets, &clocks);
        let mut mcp2515 = Mcp2515::new(spi_wrapper, mcp2515_cs, mcp2515_timer);

    defmt::info!("Radio boot: RTT alive (defmt)");
    info!(&mut uart, "Initializing MCP2515 CAN controller for Radio...");
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
                            timer.delay_ms(100);
                            led_pin.set_low().unwrap();
                            timer.delay_ms(100);
                        }
                    }
                    Err(e) => {
                        warn!(&mut uart, "MCP2515 basic functionality test failed: {:?}", e);
                        for _ in 0..2 {
                            led_pin.set_high().unwrap();
                            timer.delay_ms(100);
                            led_pin.set_low().unwrap();
                            timer.delay_ms(100);
                        }
                    }
                }
            }
            Err(e) => {
                error_macro!(&mut uart, "MCP2515 initialization failed: {:?}", e);
                for _ in 0..5 {
                    led_pin.set_high().unwrap();
                    timer.delay_ms(50);
                    led_pin.set_low().unwrap();
                    timer.delay_ms(50);
                }
            }
        }

        for _ in 0..4 {
            led_pin.set_high().unwrap();
            timer.delay_ms(75);
            led_pin.set_low().unwrap();
            timer.delay_ms(75);
        }

    defmt::info!("Radio init complete (defmt)");
    info!(&mut uart, "Radio initialization complete - starting main loop");
        (Shared { mcp2515, token: false }, Local { timer, led_pin, error_count: 0, hold_timer: 0, uart })
    }

    #[idle(shared = [mcp2515, token], local = [timer, led_pin, error_count, hold_timer, uart])]
    fn idle(mut cx: idle::Context) -> ! {
        let timer = cx.local.timer;
        let led_pin = cx.local.led_pin;
        let error_count = cx.local.error_count;
        let hold_timer = cx.local.hold_timer;
        let uart = cx.local.uart;

    defmt::info!("Radio entering main loop (defmt)");
    info!(uart, "Radio entering main loop - monitoring CAN bus");
        
        let mut heartbeat_counter = 0u32;
        
        loop {
            // Add heartbeat every 500 loops (5 seconds at 10ms per loop)
            heartbeat_counter += 1;
            if heartbeat_counter >= 500 {
                info!(uart, "Radio: Heartbeat - loop iteration {}", heartbeat_counter);
                heartbeat_counter = 0;
            }
            cx.shared.token.lock(|token| {
                if *token {
                    led_pin.set_high().unwrap();
                    *hold_timer += 1;
                    if *hold_timer >= 200 { // 2 seconds (200 * 10ms)
                        *hold_timer = 0;
                        cx.shared.mcp2515.lock(|mcp2515| {
                            let msg = CanMessage {
                                id: 0x400, // Pass from Radio to FC
                                dlc: 0,
                                data: [0; 8],
                                is_extended: false,
                            };
                            match mcp2515.send_message(&msg) {
                                Ok(()) => {
                                    info!(uart, "Radio: Passed token to FC");
                                    *token = false;
                                    led_pin.set_low().unwrap();
                                }
                                Err(e) => {
                                    error_macro!(uart, "Radio: Failed to pass token: {:?}", e);
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
                            warn!(uart, "Radio: CAN Error flags: 0x{:02X}", errors);
                            *error_count += 1;
                            if *error_count > 10 {
                                error_macro!(uart, "Radio: Too many errors, resetting MCP2515");
                                if let Err(e) = mcp2515.reset() {
                                    error_macro!(uart, "Radio: Reset failed: {:?}", e);
                                } else if let Err(e) = mcp2515.init() {
                                    error_macro!(uart, "Radio: Re-init failed: {:?}", e);
                                } else {
                                    *error_count = 0;
                                    info!(uart, "Radio: MCP2515 reset and re-initialized");
                                }
                            }
                        }
                    }
                    Err(e) => error_macro!(uart, "Radio: Failed to read error flags: {:?}", e),
                }
                match mcp2515.has_message() {
                    Ok(true) => {
                        match mcp2515.receive_message() {
                            Ok(Some(msg)) => {
                                if msg.id == 0x300 { // Pass from FC to Radio
                                    info!(uart, "Radio: Received token from FC");
                                    cx.shared.token.lock(|token| *token = true);
                                    *hold_timer = 0; // Reset hold timer on receive
                                }
                            }
                            Ok(None) => {}
                            Err(e) => {
                                error_macro!(uart, "Radio: Error receiving CAN message: {:?}", e);
                                *error_count += 1;
                            }
                        }
                    }
                    Ok(false) => {}
                    Err(e) => {
                        error_macro!(uart, "Radio: Error checking for CAN messages: {:?}", e);
                        *error_count += 1;
                    }
                }
            });

            // Shorter delay to allow more frequent UART flushing
            timer.delay_ms(10);
        }
    }
}