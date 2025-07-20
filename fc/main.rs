#![no_std]
#![no_main]

// Use panic-halt as the panic handler
use panic_halt as _;

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

// MCP2515 CAN controller driver
mod mcp2515_driver;
use mcp2515_driver::{Mcp2515, SimpleSpi};

// SPI wrapper for the HAL SPI
struct SpiWrapper<S> {
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
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        timer: Timer<CopyableTimer0>,
        led_pin: Pin<Gpio25, FunctionSioOutput, PullNone>,
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
        
        // Create a temporary MCP2515 instance for testing during init
        let mut mcp2515 = Mcp2515::new(spi_wrapper, mcp2515_cs, timer);
        
        // Initialize MCP2515 - this will test basic communication
        match mcp2515.init() {
            Ok(()) => {
                // Test basic functionality in loopback mode
                match mcp2515.test_basic_functionality() {
                    Ok(()) => {
                        // LED blink pattern: 3 fast blinks = full success
                        for _ in 0..3 {
                            led_pin.set_high().unwrap();
                            timer.delay_ms(100);
                            led_pin.set_low().unwrap();
                            timer.delay_ms(100);
                        }
                    }
                    Err(_) => {
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
                // LED blink pattern: 5 fast blinks = init error
                for _ in 0..5 {
                    led_pin.set_high().unwrap();
                    timer.delay_ms(50);
                    led_pin.set_low().unwrap();
                    timer.delay_ms(50);
                }
            }
        }

        (Shared {}, Local { timer, led_pin })
    }

    #[idle(local = [timer, led_pin])]
    fn idle(cx: idle::Context) -> ! {
        let timer = cx.local.timer;
        let led_pin = cx.local.led_pin;

        // Main loop - blink the LED
        loop {
            // Turn LED on
            led_pin.set_high().unwrap();
            // Wait for 500ms
            timer.delay_ms(500);

            // Turn LED off
            led_pin.set_low().unwrap();
            // Wait for 500ms
            timer.delay_ms(500);
        }
    }
}
