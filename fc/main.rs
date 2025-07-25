// fc/main.rs
#![no_std]
#![no_main]

use panic_halt as _;
use rp235x_hal as hal;
use rtic::app;
use hal::gpio::{Pin, bank0::{Gpio4, Gpio5, Gpio20, Gpio21, Gpio25}, PullNone, PullUp, FunctionUart, FunctionI2C, FunctionSioOutput};
use hal::timer::{Timer, CopyableTimer0, Alarm};
use hal::fugit::{HertzU32, MicrosDurationU32};
use hal::clocks::Clock;
use hal::i2c::I2C;
mod drivers;
use drivers::bus_managers::I2cBusManager;
use drivers::icm20948::{Icm20948, RawImu, ICM20948_ADDR_AD0_LOW};
use drivers::sensor_trait::SensorDriver;
use core::fmt::Write;

#[app(device = hal::pac, peripherals = true, dispatchers = [TIMER0_IRQ_1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        i2c_manager: I2cBusManager<hal::pac::I2C0, bank0::Gpio20, bank0::Gpio21>,
        icm20948: Icm20948<Timer<CopyableTimer0>>,
        timer: Timer<CopyableTimer0>,
        debug_uart: hal::uart::UartPeripheral<hal::uart::Enabled, hal::pac::UART1, (Pin<bank0::Gpio4, FunctionUart, PullNone>, Pin<bank0::Gpio5, FunctionUart, PullNone>)>,
    }

    #[local]
    struct Local {}

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
        ).unwrap();

        let mut timer = Timer::new_timer0(cx.device.TIMER0, &mut resets, &clocks);
        let pins = hal::gpio::Pins::new(
            cx.device.IO_BANK0,
            cx.device.PADS_BANK0,
            hal::Sio::new(cx.device.SIO).gpio_bank0,
            &mut resets,
        );

        let tx_pin = pins.gpio4.into_pull_type::<PullNone>().into_function::<FunctionUart>();
        let rx_pin = pins.gpio5.into_pull_type::<PullNone>().into_function::<FunctionUart>();
        let uart_pins = (tx_pin, rx_pin);

        let mut debug_uart = hal::uart::UartPeripheral::new(cx.device.UART1, uart_pins, &mut resets).enable(
            hal::uart::UartConfig::default(),
            clocks.peripheral_clock.freq(),
        ).unwrap();

        let i2c_sda = pins.gpio20.into_pull_type::<PullUp>().into_function::<FunctionI2C>();
        let i2c_scl = pins.gpio21.into_pull_type::<PullUp>().into_function::<FunctionI2C>();
        let i2c = I2C::i2c0(cx.device.I2C0, i2c_sda, i2c_scl, HertzU32::kHz(100), &mut resets, clocks.system_clock.freq());
        let i2c_manager = I2cBusManager::new(i2c);

        let mut icm20948 = Icm20948::new(timer.clone(), ICM20948_ADDR_AD0_LOW);
        {
            if let Some(i2c_ref) = i2c_manager.acquire() {
                match icm20948.init(i2c_ref) {
                    Ok(()) => {
                        let _ = write!(debug_uart, "ICM20948 IMU initialized\r\n");
                    }
                    Err(e) => {
                        let _ = write!(debug_uart, "ICM20948 IMU init failed: {:?}\r\n", e);
                    }
                }
                i2c_manager.release();
            } else {
                let _ = write!(debug_uart, "Could not acquire I2C for IMU init\r\n");
            }
        }

        let mut alarm = timer.alarm_0().unwrap();
        alarm.schedule(MicrosDurationU32::secs(1)).unwrap();
        alarm.enable_interrupt();

        (Shared { i2c_manager, icm20948, timer, debug_uart }, Local {})
    }

    #[task(binds = TIMER0_IRQ_0, priority = 2, shared = [i2c_manager, icm20948, timer, debug_uart])]
    fn sensor_read(mut cx: sensor_read::Context) {
        cx.shared.timer.lock(|timer| {
            let mut alarm = timer.alarm_0().unwrap();
            alarm.clear_interrupt();
        });

        let mut raw_data: Option<RawImu> = None;
        cx.shared.i2c_manager.lock(|manager| {
            if let Some(i2c) = manager.acquire() {
                cx.shared.icm20948.lock(|imu| {
                    match imu.read_raw(i2c) {
                        Ok(raw) => {
                            raw_data = Some(raw);
                        }
                        Err(e) => {
                            cx.shared.debug_uart.lock(|uart| {
                                let _ = write!(uart, "Failed to read IMU: {:?}\r\n", e);
                            });
                        }
                    }
                });
                manager.release();
            } else {
                cx.shared.debug_uart.lock(|uart| {
                    let _ = write!(uart, "Could not acquire I2C for IMU read\r\n");
                });
            }
        });

        if let Some(raw) = raw_data {
            cx.shared.debug_uart.lock(|uart| {
                let _ = write!(uart, "(IMU1: accel: {}, {}, {} ; gyro: {}, {}, {} ;)\r\n",
                    raw.accel[0], raw.accel[1], raw.accel[2],
                    raw.gyro[0], raw.gyro[1], raw.gyro[2]);
            });
        } else {
            cx.shared.debug_uart.lock(|uart| {
                let _ = write!(uart, "IMU not read\r\n");
            });
        }

        cx.shared.timer.lock(|timer| {
            let mut alarm = timer.alarm_0().unwrap();
            alarm.schedule(MicrosDurationU32::secs(1)).unwrap();
        });
    }
}