// fc/main.rs
#![no_std]
#![no_main]

use panic_probe as _;
use defmt_rtt as _; // global logger
use rp235x_hal as hal;
use rtic::app;
use hal::gpio::{FunctionSioOutput, Pin, bank0::Gpio25, PullNone, FunctionSpi, PullUp};
use hal::timer::{Timer, CopyableTimer0, CopyableTimer1};
use hal::spi::Spi;
use hal::fugit::HertzU32;
use defmt::{info, warn, error};
use embedded_hal::digital::OutputPin;
use embedded_hal::delay::DelayNs;
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

defmt::timestamp!("{=u64}", {
    0
});

#[app(device = hal::pac, peripherals = true, dispatchers = [TIMER0_IRQ_0])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        mcp2515: Mcp2515<SpiWrapper<Spi<hal::spi::Enabled, hal::pac::SPI1, (Pin<hal::gpio::bank0::Gpio11, FunctionSpi, PullUp>, Pin<hal::gpio::bank0::Gpio12, FunctionSpi, PullUp>, Pin<hal::gpio::bank0::Gpio10, FunctionSpi, PullUp>), 8>>, Pin<hal::gpio::bank0::Gpio13, FunctionSioOutput, PullUp>, Timer<CopyableTimer1>>,
        token: bool, // Shared token state
    }

    #[local]
    struct Local {
        timer: Timer<CopyableTimer0>,
        led_pin: Pin<Gpio25, FunctionSioOutput, PullNone>,
        error_count: u32,
        hold_timer: u32,
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

        info!("Initializing MCP2515 CAN controller...");
        match mcp2515.init() {
            Ok(()) => {
                info!("MCP2515 initialization successful");
                match mcp2515.configure_receive_all() {
                    Ok(()) => info!("MCP2515 configured to receive all messages"),
                    Err(e) => error!("Failed to configure MCP2515 receive mode: {:?}", e),
                }
                match mcp2515.test_basic_functionality() {
                    Ok(()) => {
                        info!("MCP2515 basic functionality test passed");
                        for _ in 0..3 {
                            led_pin.set_high().unwrap();
                            timer.delay_ms(100);
                            led_pin.set_low().unwrap();
                            timer.delay_ms(100);
                        }
                    }
                    Err(e) => {
                        warn!("MCP2515 basic functionality test failed: {:?}", e);
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
                error!("MCP2515 initialization failed: {:?}", e);
                for _ in 0..5 {
                    led_pin.set_high().unwrap();
                    timer.delay_ms(50);
                    led_pin.set_low().unwrap();
                    timer.delay_ms(50);
                }
            }
        }

        info!("FC initialization complete - starting main loop");
        (Shared { mcp2515, token: true }, Local { timer, led_pin, error_count: 0, hold_timer: 0 })
    }

    #[idle(shared = [mcp2515, token], local = [timer, led_pin, error_count, hold_timer])]
    fn idle(mut cx: idle::Context) -> ! {
        let timer = cx.local.timer;
        let led_pin = cx.local.led_pin;
        let error_count = cx.local.error_count;
        let hold_timer = cx.local.hold_timer;

        info!("FC entering main loop - monitoring CAN bus");
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
                                    info!("FC: Passed token to Radio");
                                    *token = false;
                                    led_pin.set_low().unwrap();
                                }
                                Err(e) => {
                                    error!("FC: Failed to pass token: {:?}", e);
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
                            warn!("FC: CAN Error flags: 0x{:02X}", errors);
                            *error_count += 1;
                            if *error_count > 10 {
                                error!("FC: Too many errors, resetting MCP2515");
                                if let Err(e) = mcp2515.reset() {
                                    error!("FC: Reset failed: {:?}", e);
                                } else if let Err(e) = mcp2515.init() {
                                    error!("FC: Re-init failed: {:?}", e);
                                } else {
                                    *error_count = 0;
                                    info!("FC: MCP2515 reset and re-initialized");
                                }
                            }
                        }
                    }
                    Err(e) => error!("FC: Failed to read error flags: {:?}", e),
                }
                match mcp2515.has_message() {
                    Ok(true) => {
                        match mcp2515.receive_message() {
                            Ok(Some(msg)) => {
                                if msg.id == 0x400 { // Pass from Radio to FC
                                    info!("FC: Received token from Radio");
                                    cx.shared.token.lock(|token| *token = true);
                                    *hold_timer = 0; // Reset hold timer on receive
                                }
                            }
                            Ok(None) => {}
                            Err(e) => {
                                error!("FC: Error receiving CAN message: {:?}", e);
                                *error_count += 1;
                            }
                        }
                    }
                    Ok(false) => {}
                    Err(e) => {
                        error!("FC: Error checking for CAN messages: {:?}", e);
                        *error_count += 1;
                    }
                }
            });

            timer.delay_ms(50);
        }
    }
}