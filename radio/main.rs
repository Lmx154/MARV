// radio/main.rs
#![no_std]
#![no_main]

use panic_probe as _;
use defmt_rtt as _; // global logger
use rp235x_hal as hal;
use rtic::app;
use hal::gpio::{FunctionSioOutput, Pin, bank0::Gpio24, PullNone, FunctionSpi, PullUp};
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
        mcp2515: Mcp2515<SpiWrapper<Spi<hal::spi::Enabled, hal::pac::SPI0, (Pin<hal::gpio::bank0::Gpio19, FunctionSpi, PullUp>, Pin<hal::gpio::bank0::Gpio16, FunctionSpi, PullUp>, Pin<hal::gpio::bank0::Gpio18, FunctionSpi, PullUp>), 8>>, Pin<hal::gpio::bank0::Gpio17, FunctionSioOutput, PullUp>, Timer<CopyableTimer1>>,
    }

    #[local]
    struct Local {
        timer: Timer<CopyableTimer0>,
        led_pin: Pin<Gpio24, FunctionSioOutput, PullNone>,
        receive_counter: u32,
        error_count: u32,
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

        let mut led_pin = pins.gpio24.into_pull_type::<PullNone>().into_push_pull_output();
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

        info!("Initializing MCP2515 CAN controller for Radio...");
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

        for _ in 0..4 {
            led_pin.set_high().unwrap();
            timer.delay_ms(75);
            led_pin.set_low().unwrap();
            timer.delay_ms(75);
        }

        info!("Radio initialization complete - starting main loop");
        (Shared { mcp2515 }, Local { timer, led_pin, receive_counter: 0, error_count: 0 })
    }

    #[idle(shared = [mcp2515], local = [timer, led_pin, receive_counter, error_count])]
    fn idle(mut cx: idle::Context) -> ! {
        let timer = cx.local.timer;
        let led_pin = cx.local.led_pin;
        let receive_counter = cx.local.receive_counter;
        let error_count = cx.local.error_count;

        info!("Radio entering main loop - monitoring CAN bus");
        loop {
            cx.shared.mcp2515.lock(|mcp2515| {
                match mcp2515.get_error_flags() {
                    Ok(errors) => {
                        if errors != 0 {
                            warn!("Radio: CAN Error flags: 0x{:02X}", errors);
                            *error_count += 1;
                            if *error_count > 10 {
                                error!("Radio: Too many errors, resetting MCP2515");
                                if let Err(e) = mcp2515.reset() {
                                    error!("Radio: Reset failed: {:?}", e);
                                } else if let Err(e) = mcp2515.init() {
                                    error!("Radio: Re-init failed: {:?}", e);
                                } else {
                                    *error_count = 0;
                                    info!("Radio: MCP2515 reset and re-initialized");
                                }
                            }
                        }
                    }
                    Err(e) => error!("Radio: Failed to read error flags: {:?}", e),
                }
                match mcp2515.has_message() {
                    Ok(true) => {
                        match mcp2515.receive_message() {
                            Ok(Some(msg)) => {
                                *receive_counter += 1;
                                let received_counter = (msg.data[0] as u32) | ((msg.data[1] as u32) << 8) | ((msg.data[2] as u32) << 16) | ((msg.data[3] as u32) << 24);
                                info!("Radio: Received message #{}, ID=0x{:x}, counter={}", *receive_counter, msg.id, received_counter);
                                let response_counter = received_counter + 1;
                                let response_msg = CanMessage {
                                    id: 0x200,
                                    dlc: 4,
                                    data: [response_counter as u8, (response_counter >> 8) as u8, (response_counter >> 16) as u8, (response_counter >> 24) as u8, 0, 0, 0, 0],
                                    is_extended: false,
                                };
                                match mcp2515.send_message(&response_msg) {
                                    Ok(()) => {
                                        info!("Radio: Sent response with counter={}", response_counter);
                                        for _ in 0..3 {
                                            led_pin.set_high().unwrap();
                                            timer.delay_ms(100);
                                            led_pin.set_low().unwrap();
                                            timer.delay_ms(100);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Radio: Failed to send response: {:?}", e);
                                        *error_count += 1;
                                    }
                                }
                            }
                            Ok(None) => {}
                            Err(e) => {
                                error!("Radio: Error receiving CAN message: {:?}", e);
                                *error_count += 1;
                            }
                        }
                    }
                    Ok(false) => {}
                    Err(e) => {
                        error!("Radio: Error checking for CAN messages: {:?}", e);
                        *error_count += 1;
                    }
                }
            });

            timer.delay_ms(50); // Reduced for faster polling
        }
    }
}