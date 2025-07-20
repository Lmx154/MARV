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
use hal::gpio::{FunctionSioOutput, Pin, bank0::Gpio24, PullNone};
use hal::timer::{Timer, CopyableTimer0};

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
        led_pin: Pin<Gpio24, FunctionSioOutput, PullNone>, // Different LED pin for radio
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

        // Radio uses GPIO24 as LED instead of GPIO25
        let mut led_pin = pins.gpio24.into_pull_type::<PullNone>().into_push_pull_output();

        // Radio initialization pattern: 4 quick blinks
        for _ in 0..4 {
            led_pin.set_high().unwrap();
            timer.delay_ms(75);
            led_pin.set_low().unwrap();
            timer.delay_ms(75);
        }

        (Shared {}, Local { timer, led_pin })
    }

    #[idle(local = [timer, led_pin])]
    fn idle(cx: idle::Context) -> ! {
        let timer = cx.local.timer;
        let led_pin = cx.local.led_pin;

        // Radio has different blink pattern - faster blinks
        loop {
            // Turn LED on
            led_pin.set_high().unwrap();
            // Wait for 250ms (faster than FC)
            timer.delay_ms(250);

            // Turn LED off
            led_pin.set_low().unwrap();
            // Wait for 250ms
            timer.delay_ms(250);
        }
    }
}
