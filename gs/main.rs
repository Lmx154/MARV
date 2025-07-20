#![no_std]
#![no_main]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use rp235x_hal as hal;
use rtic::app;

use hal::gpio::{FunctionSioOutput, Pin, bank0::Gpio25, PullNone};
use hal::timer::{Timer, CopyableTimer0};

// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

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

        // Configure GPIO25 as output for LED
        let mut led_pin = pins.gpio25.into_pull_type::<PullNone>().into_push_pull_output();

        // Ground Station startup sequence - 3 quick blinks
        for _ in 0..3 {
            led_pin.set_high().unwrap();
            timer.delay_ms(100);
            led_pin.set_low().unwrap();
            timer.delay_ms(100);
        }

        (Shared {}, Local { timer, led_pin })
    }

    #[idle(local = [timer, led_pin])]
    fn idle(cx: idle::Context) -> ! {
        let timer = cx.local.timer;
        let led_pin = cx.local.led_pin;

        loop {
            // Ground Station - slow blink pattern (500ms on/off)
            led_pin.set_high().unwrap();
            timer.delay_ms(500);
            led_pin.set_low().unwrap();
            timer.delay_ms(500);
        }
    }
}
