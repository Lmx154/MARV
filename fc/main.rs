// fc/main.rs
#![no_std]
#![no_main]

use panic_halt as _;
use rp235x_hal as hal;
use rtic::app;
use hal::gpio::{FunctionSioOutput, Pin, bank0::{Gpio0, Gpio1, Gpio2, Gpio3, Gpio4, Gpio5, Gpio10, Gpio11, Gpio12, Gpio13, Gpio25}, PullNone, PullUp, FunctionSpi, FunctionUart, FunctionI2C};
use hal::timer::{Timer, CopyableTimer0, CopyableTimer1, Event};
use hal::spi::Spi;
use hal::fugit::{HertzU32, MicrosDurationU32};
use embedded_hal::digital::OutputPin;
use embedded_hal::delay::DelayNs;
use hal::uart::{self, UartConfig, UartPeripheral, DataBits, StopBits};
use hal::clocks::Clock;
use hal::i2c::I2C;
mod mcp2515_driver;
use mcp2515_driver::{Mcp2515, SimpleSpi, CanMessage, Mcp2515Error};
mod drivers;
use drivers::bus_managers::{I2cBusManager, UartBusManager};
use drivers::pcf8563::{Pcf8563, DateTime as RtcDateTime, Error as RtcError};
use drivers::ublox_neom9n::{GpsModule, GpsData};
use drivers::sensor_trait::SensorDriver;
use drivers::icm20948::{Icm20948, RawImu, ICM20948_ADDR_AD0_LOW};
use core::fmt::{self, Write};
use embedded_hal_nb::serial::Write as SerialWrite;
use heapless::String as HeaplessString;

pub struct SpiWrapper<S> {
    spi: S,
}

// fc/main.rs
#![no_std]
#![no_main]

use panic_halt as _;
use rp235x_hal as hal;
use rtic::app;
use hal::gpio::{Pin, bank0::{Gpio2, Gpio3, Gpio4, Gpio5}, PullUp, PullNone, FunctionUart, FunctionI2C};
use hal::timer::{Timer, CopyableTimer0, Event};
use hal::fugit::{HertzU32, MicrosDurationU32};
use hal::uart::{UartConfig, UartPeripheral};
use hal::clocks::Clock;
use hal::i2c::I2C;
mod drivers;
use drivers::icm20948::{Icm20948, ICM20948_ADDR_AD0_LOW};
use core::fmt::Write;

#[app(device = hal::pac, peripherals = true, dispatchers = [TIMER0_IRQ_1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        icm20948: Option<Icm20948<I2C<hal::pac::I2C1, Gpio2, Gpio3>, Timer<CopyableTimer0>>>,
        timer: Timer<CopyableTimer0>,
        debug_uart: UartPeripheral<hal::uart::Enabled, hal::pac::UART1, (Pin<Gpio4, FunctionUart, PullNone>, Pin<Gpio5, FunctionUart, PullNone>)>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, ()) {
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

        let mut debug_uart = UartPeripheral::new(cx.device.UART1, uart_pins, &mut resets).enable(
            UartConfig::default(),
            clocks.peripheral_clock.freq(),
        ).unwrap();

        let i2c_sda = pins.gpio2.into_pull_type::<PullUp>().into_function::<FunctionI2C>();
        let i2c_scl = pins.gpio3.into_pull_type::<PullUp>().into_function::<FunctionI2C>();
        let mut i2c = I2C::i2c1(cx.device.I2C1, i2c_sda, i2c_scl, HertzU32::kHz(100), &mut resets, clocks.system_clock.freq());

        let mut icm20948: Option<Icm20948<I2C<hal::pac::I2C1, Gpio2, Gpio3>, Timer<CopyableTimer0>>> = None;
        {
            let mut imu = Icm20948::new(&mut i2c, ICM20948_ADDR_AD0_LOW, timer.clone());
            match imu.init() {
                Ok(()) => {
                    let _ = write!(debug_uart, "ICM20948 IMU initialized\r\n");
                    icm20948 = Some(imu);
                }
                Err(e) => {
                    let _ = write!(debug_uart, "ICM20948 IMU init failed: {:?}\r\n", e);
                }
            }
        }

        let mut alarm = timer.alarm_0().unwrap();
        alarm.schedule(MicrosDurationU32::secs(1)).unwrap();
        timer.listen(Event::Alarm0);

        (Shared { icm20948, timer, debug_uart }, ())
    }

    #[task(binds = TIMER0_IRQ_0, priority = 2, shared = [icm20948, timer, debug_uart])]
    fn sensor_read(mut cx: sensor_read::Context) {
        cx.shared.timer.lock(|timer| {
            let mut alarm = timer.alarm_0().unwrap();
            alarm.clear_interrupt();
            timer.unlisten(Event::Alarm0);
        });

        cx.shared.icm20948.lock(|imu_opt| {
            if let Some(imu) = imu_opt {
                match imu.read_raw() {
                    Ok(raw) => {
                        cx.shared.debug_uart.lock(|uart| {
                            let _ = write!(uart, "IMU Accel: [{}, {}, {}] Gyro: [{}, {}, {}]\r\n", raw.accel[0], raw.accel[1], raw.accel[2], raw.gyro[0], raw.gyro[1], raw.gyro[2]);
                        });
                    }
                    Err(e) => {
                        cx.shared.debug_uart.lock(|uart| {
                            let _ = write!(uart, "Failed to read IMU: {:?}\r\n", e);
                        });
                    }
                }
            } else {
                cx.shared.debug_uart.lock(|uart| {
                    let _ = write!(uart, "IMU not initialized\r\n");
                });
            }
        });

        cx.shared.timer.lock(|timer| {
            let mut alarm = timer.alarm_0().unwrap();
            alarm.schedule(MicrosDurationU32::secs(1)).unwrap();
            timer.listen(Event::Alarm0);
        });
    }
}
        {
            // Try to acquire I2C for IMU init
            if let Some(i2c_ref) = i2c_manager.acquire() {
                let mut imu = Icm20948::new(i2c_ref, ICM20948_ADDR_AD0_LOW, timer.clone());
                match imu.init() {
                    Ok(()) => {
                        info!(&mut debug_uart, "ICM20948 IMU initialized");
                        icm20948 = Some(imu);
                    }
                    Err(e) => {
                        warn!(&mut debug_uart, "ICM20948 IMU init failed: {:?}", e);
                    }
                }
                i2c_manager.release();
            } else {
                warn!(&mut debug_uart, "Could not acquire I2C for IMU init");
            }
        }

        let gps_tx = pins.gpio0.into_pull_type::<PullNone>().into_function::<FunctionUart>();
        let gps_rx = pins.gpio1.into_pull_type::<PullNone>().into_function::<FunctionUart>();
        let gps_uart = UartPeripheral::new(cx.device.UART0, (gps_tx, gps_rx), &mut resets).enable(
            UartConfig::new(HertzU32::Hz(38400), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        ).unwrap();
        let uart_manager = UartBusManager::new(gps_uart);

        let rtc = Pcf8563;
        let gps = GpsModule::new();

        let mut alarm = timer.alarm_0().unwrap();
        alarm.schedule(MicrosDurationU32::secs(1)).unwrap();
        timer.listen(Event::Alarm0);

        info!(&mut debug_uart, "FC initialization complete - starting main loop");
        (Shared { mcp2515, token: true, i2c_manager, uart_manager, timer, debug_uart, icm20948 }, Local { led_pin, error_count: 0, hold_timer: 0, rtc, gps })
    }

    #[task(binds = TIMER0_IRQ_0, priority = 2, shared = [i2c_manager, uart_manager, timer, debug_uart, icm20948], local = [rtc, gps])]
    fn sensor_read(mut cx: sensor_read::Context) {
        cx.shared.timer.lock(|timer| {
            let mut alarm = timer.alarm_0().unwrap();
            alarm.clear_interrupt();
            timer.unlisten(Event::Alarm0);
        });

        let mut read_ok = false;
        cx.shared.i2c_manager.lock(|manager| {
            if let Some(i2c) = manager.acquire() {
                if let Ok(dt) = cx.local.rtc.read_datetime(i2c) {
                    cx.shared.debug_uart.lock(|uart| {
                        info!(uart, "RTC Time: {}", dt.format());
                    });
                    read_ok = true;
                }
                manager.release();
            }
        });
        if !read_ok {
            cx.shared.debug_uart.lock(|uart| {
                warn!(uart, "Failed to read RTC");
            });
        }

        // Read ICM20948 IMU raw data
        let mut imu_read_ok = false;
        cx.shared.icm20948.lock(|imu_opt| {
            if let Some(imu) = imu_opt {
                match imu.read_raw() {
                    Ok(raw) => {
                        cx.shared.debug_uart.lock(|uart| {
                            info!(uart, "IMU Accel: [{}, {}, {}] Gyro: [{}, {}, {}]", raw.accel[0], raw.accel[1], raw.accel[2], raw.gyro[0], raw.gyro[1], raw.gyro[2]);
                        });
                        imu_read_ok = true;
                    }
                    Err(e) => {
                        cx.shared.debug_uart.lock(|uart| {
                            warn!(uart, "Failed to read IMU: {:?}", e);
                        });
                    }
                }
            } else {
                cx.shared.debug_uart.lock(|uart| {
                    warn!(uart, "IMU not initialized");
                });
            }
        });

        read_ok = false;
        cx.shared.uart_manager.lock(|manager| {
            if let Some(uart) = manager.acquire() {
                cx.local.gps.update(uart);
                let data = cx.local.gps.get_last_data();
                cx.shared.debug_uart.lock(|uart| {
                    info!(uart, "GPS Data: {}", data.format_csv());
                });
                read_ok = true;
                manager.release();
            }
        });
        if !read_ok {
            cx.shared.debug_uart.lock(|uart| {
                warn!(uart, "Failed to read GPS");
            });
        }

        cx.shared.timer.lock(|timer| {
            let mut alarm = timer.alarm_0().unwrap();
            alarm.schedule(MicrosDurationU32::secs(1)).unwrap();
            timer.listen(Event::Alarm0);
        });
    }

    #[idle(shared = [mcp2515, token, i2c_manager, uart_manager, timer, debug_uart], local = [led_pin, error_count, hold_timer])]
    fn idle(mut cx: idle::Context) -> ! {
        let led_pin = cx.local.led_pin;
        let error_count = cx.local.error_count;
        let hold_timer = cx.local.hold_timer;

        cx.shared.debug_uart.lock(|uart| {
            info!(uart, "FC entering main loop - monitoring CAN bus");
        });

        loop {
            cx.shared.token.lock(|token| {
                if *token {
                    led_pin.set_high().unwrap();
                    *hold_timer += 1;
                    if *hold_timer >= 40 {
                        *hold_timer = 0;
                        cx.shared.mcp2515.lock(|mcp2515| {
                            let msg = CanMessage {
                                id: 0x300,
                                dlc: 0,
                                data: [0; 8],
                                is_extended: false,
                            };
                            match mcp2515.send_message(&msg) {
                                Ok(()) => {
                                    cx.shared.debug_uart.lock(|uart| {
                                        info!(uart, "FC: Passed token to Radio");
                                    });
                                    *token = false;
                                    led_pin.set_low().unwrap();
                                }
                                Err(e) => {
                                    cx.shared.debug_uart.lock(|uart| {
                                        error_macro!(uart, "FC: Failed to pass token: {:?}", e);
                                    });
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
                            cx.shared.debug_uart.lock(|uart| {
                                warn!(uart, "FC: CAN Error flags: 0x{:02X}", errors);
                            });
                            *error_count += 1;
                            if *error_count > 10 {
                                cx.shared.debug_uart.lock(|uart| {
                                    error_macro!(uart, "FC: Too many errors, resetting MCP2515");
                                });
                                if let Err(e) = mcp2515.reset() {
                                    cx.shared.debug_uart.lock(|uart| {
                                        error_macro!(uart, "FC: Reset failed: {:?}", e);
                                    });
                                } else if let Err(e) = mcp2515.init() {
                                    cx.shared.debug_uart.lock(|uart| {
                                        error_macro!(uart, "FC: Re-init failed: {:?}", e);
                                    });
                                } else {
                                    *error_count = 0;
                                    cx.shared.debug_uart.lock(|uart| {
                                        info!(uart, "FC: MCP2515 reset and re-initialized");
                                    });
                                }
                            }
                        }
                    }
                    Err(e) => cx.shared.debug_uart.lock(|uart| {
                        error_macro!(uart, "FC: Failed to read error flags: {:?}", e);
                    }),
                }
                match mcp2515.has_message() {
                    Ok(true) => {
                        match mcp2515.receive_message() {
                            Ok(Some(msg)) => {
                                if msg.id == 0x400 {
                                    cx.shared.debug_uart.lock(|uart| {
                                        info!(uart, "FC: Received token from Radio");
                                    });
                                    cx.shared.token.lock(|token| *token = true);
                                    *hold_timer = 0;
                                }
                            }
                            Ok(None) => {}
                            Err(e) => {
                                cx.shared.debug_uart.lock(|uart| {
                                    error_macro!(uart, "FC: Error receiving CAN message: {:?}", e);
                                });
                                *error_count += 1;
                            }
                        }
                    }
                    Ok(false) => {}
                    Err(e) => {
                        cx.shared.debug_uart.lock(|uart| {
                            error_macro!(uart, "FC: Error checking for CAN messages: {:?}", e);
                        });
                        *error_count += 1;
                    }
                }
            });

            cx.shared.timer.lock(|timer| {
                timer.delay_ms(50);
            });
        }
    }
}