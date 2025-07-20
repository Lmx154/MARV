#![no_std]
#![no_main]

// Use panic-probe as the panic handler for debugging
use panic_probe as _;

// Provide an alias for our HAL crate
use rp235x_hal as hal;

// Some things we need
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use rtic::app;

// Additional imports for corrected types
use hal::gpio::{FunctionSioOutput, Pin, bank0::Gpio25, PullNone};
use hal::timer::{Timer, CopyableTimer0};

// SPI and GPIO imports for MCP2515
use hal::gpio::{FunctionSpi, PullUp};
use hal::spi::Spi;
use hal::fugit::HertzU32;

// Debugging
use defmt_rtt as _;

// Timestamp implementation for defmt
defmt::timestamp!("{=u64}", {
    // Just return 0 for now - you could implement a real timestamp using a timer
    0
});

// MCP2515 CAN controller driver
mod mcp2515_driver;
use mcp2515_driver::{Mcp2515, SimpleSpi, CanMessage};

// SPI wrapper for the HAL SPI
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

// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// RTIC application configuration
#[app(device = hal::pac, peripherals = true, dispatchers = [TIMER0_IRQ_0])]
mod app {
    use crate::{SpiWrapper, Mcp2515, SimpleSpi, CanMessage};
    use rp235x_hal as hal;
    use embedded_hal::delay::DelayNs;
    use embedded_hal::digital::OutputPin;
    use hal::gpio::{FunctionSioOutput, Pin, bank0::Gpio25, PullNone, FunctionSpi, PullUp};
    use hal::timer::{Timer, CopyableTimer0};
    use hal::spi::Spi;
    use hal::fugit::HertzU32;
    use defmt::{info, warn, error};

    #[shared]
    struct Shared {
        mcp2515: Mcp2515<SpiWrapper<Spi<hal::spi::Enabled, hal::pac::SPI1, (Pin<hal::gpio::bank0::Gpio11, FunctionSpi, PullUp>, Pin<hal::gpio::bank0::Gpio12, FunctionSpi, PullUp>, Pin<hal::gpio::bank0::Gpio10, FunctionSpi, PullUp>), 8>>, Pin<hal::gpio::bank0::Gpio13, FunctionSioOutput, PullUp>, Timer<CopyableTimer0>>,
    }

    #[local]
    struct Local {
        timer: Timer<CopyableTimer0>,
        led_pin: Pin<Gpio25, FunctionSioOutput, PullNone>,
        send_counter: u32,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut resets = cx.device.RESETS;
        let mut watchdog = hal::Watchdog::new(cx.device.WATCHDOG);

        let clocks = hal::clocks::init_clocks_and_plls(
            12_000_000u32, // 12 MHz external crystal
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

        let mut led_pin = pins.gpio25.into_pull_type::<PullNone>().into_push_pull_output();

        // Configure SPI1 for MCP2515 (GP10=SCK, GP11=MOSI, GP12=MISO)
        let spi_sclk = pins.gpio10.into_pull_type::<PullUp>().into_function::<FunctionSpi>();
        let spi_mosi = pins.gpio11.into_pull_type::<PullUp>().into_function::<FunctionSpi>();
        let spi_miso = pins.gpio12.into_pull_type::<PullUp>().into_function::<FunctionSpi>();
        
        let spi = Spi::<_, _, _, 8>::new(cx.device.SPI1, (spi_mosi, spi_miso, spi_sclk))
            .init(&mut resets, &clocks.peripheral_clock, HertzU32::from_raw(1_000_000), embedded_hal::spi::MODE_0);

        let spi_wrapper = SpiWrapper::new(spi);
        
        // CS pin for MCP2515 (GP13)
        let mcp2515_cs = pins.gpio13.into_pull_type::<PullUp>().into_push_pull_output();
        
        // Create MCP2515 instance
        let mut mcp2515 = Mcp2515::new(spi_wrapper, mcp2515_cs, timer);
        
        info!("Initializing MCP2515 CAN controller...");
        
        // Initialize MCP2515 - this will test basic communication
        match mcp2515.init() {
            Ok(()) => {
                info!("MCP2515 initialization successful");
                
                // Configure to receive all messages (no filtering)
                match mcp2515.configure_receive_all() {
                    Ok(()) => info!("MCP2515 configured to receive all messages"),
                    Err(_) => error!("Failed to configure MCP2515 receive mode"),
                }
                
                // Test basic functionality in loopback mode
                match mcp2515.test_basic_functionality() {
                    Ok(()) => {
                        info!("MCP2515 basic functionality test passed");
                        // LED blink pattern: 3 fast blinks = full success
                        for _ in 0..3 {
                            led_pin.set_high().unwrap();
                            timer.delay_ms(100);
                            led_pin.set_low().unwrap();
                            timer.delay_ms(100);
                        }
                    }
                    Err(_) => {
                        warn!("MCP2515 basic functionality test failed");
                        // LED blink pattern: 2 fast blinks = init success, test fail
                        for _ in 0..2 {
                            led_pin.set_high().unwrap();
                            timer.delay_ms(100);
                            led_pin.set_low().unwrap();
                            timer.delay_ms(100);
                        }
                    }
                }
            }
            Err(_) => {
                error!("MCP2515 initialization failed");
                // LED blink pattern: 5 fast blinks = init error
                for _ in 0..5 {
                    led_pin.set_high().unwrap();
                    timer.delay_ms(50);
                    led_pin.set_low().unwrap();
                    timer.delay_ms(50);
                }
            }
        }

        info!("FC initialization complete - starting main loop");

        (Shared { mcp2515 }, Local { timer, led_pin, send_counter: 0 })
    }

    #[idle(shared = [mcp2515], local = [timer, led_pin, send_counter])]
    fn idle(mut cx: idle::Context) -> ! {
        let timer = cx.local.timer;
        let led_pin = cx.local.led_pin;
        let send_counter = cx.local.send_counter;

        info!("FC entering main loop - monitoring CAN bus");
        let mut message_count = 0u32;
        let mut send_timer = 0u32; // Counter for periodic message sending

        // Main loop - check for CAN messages and periodically send test messages
        loop {
            // Send a test message to radio every 5 loop cycles (roughly every 500ms)
            send_timer += 1;
            if send_timer >= 5 {
                send_timer = 0;
                *send_counter += 1;
                
                cx.shared.mcp2515.lock(|mcp2515| {
                    match mcp2515.send_fc_test_message(*send_counter) {
                        Ok(()) => {
                            info!("FC: Sent test message #{} to Radio", *send_counter);
                            // Single long blink to indicate message sent
                            led_pin.set_high().unwrap();
                            timer.delay_ms(200);
                            led_pin.set_low().unwrap();
                        }
                        Err(_) => {
                            error!("FC: Failed to send test message to Radio");
                        }
                    }
                });
            }

            // Check for CAN messages (responses from radio)
            cx.shared.mcp2515.lock(|mcp2515| {
                match mcp2515.has_message() {
                    Ok(true) => {
                        match mcp2515.receive_message() {
                            Ok(Some(msg)) => {
                                message_count += 1;
                                info!("FC: Got response #{}, ID={}", 
                                    message_count,
                                    msg.id
                                );
                                
                                // Double blink to indicate message received
                                for _ in 0..2 {
                                    led_pin.set_high().unwrap();
                                    timer.delay_ms(100);
                                    led_pin.set_low().unwrap();
                                    timer.delay_ms(100);
                                }
                            }
                            Ok(None) => {
                                // No message available (shouldn't happen if has_message returned true)
                            }
                            Err(_) => {
                                error!("FC: Error receiving CAN message");
                            }
                        }
                    }
                    Ok(false) => {
                        // No messages waiting
                    }
                    Err(_) => {
                        error!("FC: Error checking for CAN messages");
                    }
                }
                
                // Check CAN controller status periodically
                if message_count % 1000 == 0 && message_count > 0 {
                    match mcp2515.get_error_flags() {
                        Ok(errors) => {
                            if errors != 0 {
                                warn!("FC: CAN Error flags: 0x{:02X}", errors);
                            }
                        }
                        Err(_) => {
                            error!("FC: Failed to read CAN error flags");
                        }
                    }
                }
            });

            // Short delay to prevent busy-waiting, but no LED blink
            timer.delay_ms(20);
        }
    }
}
