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

        let timer = Timer::new_timer0(cx.device.TIMER0, &mut resets, &clocks);

        let sio = hal::Sio::new(cx.device.SIO);

        let pins = hal::gpio::Pins::new(
            cx.device.IO_BANK0,
            cx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let led_pin = pins.gpio25.into_pull_type::<PullNone>().into_push_pull_output();

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
            timer.delay_ms(100);

            // Turn LED off
            led_pin.set_low().unwrap();
            // Wait for 500ms
            timer.delay_ms(100);
        }
    }
}
