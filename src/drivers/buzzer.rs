//! Passive buzzer driver wrapping a PWM channel.
//!
//! This mirrors the simple working code you had in `main.rs` but encapsulates
//! it so the main loop only deals with `Buzzer` methods.
//!
//! You still configure the PWM slice (frequency, divider, TOP) outside, then
//! route the GPIO to the channel and hand the channel into `Buzzer::new` with
//! an "on" duty value (e.g. 500 for 50% when TOP=999).
//!
//! Example:
//! ```ignore
//! let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
//! let mut pwm5 = pwm_slices.pwm5;
//! pwm5.default_config();
//! pwm5.set_div_int(125);
//! pwm5.set_div_frac(0);
//! pwm5.set_top(999); // 1 kHz
//! pwm5.enable();
//! let mut ch_a = pwm5.channel_a;
//! let _pin = ch_a.output_to(pins.gpio26);
//! let mut buzzer = Buzzer::new(ch_a, 500); // 50% duty when ON
//! buzzer.on();
//! ```

use embedded_hal::pwm::SetDutyCycle; // bring trait into scope for method

pub struct Buzzer<C: SetDutyCycle> {
    channel: C,
    on_duty: u16,
    is_on: bool,
}

impl<C: SetDutyCycle> Buzzer<C> {
    pub fn new(mut channel: C, on_duty: u16) -> Self {
        let _ = channel.set_duty_cycle(0); // start silent
        Self { channel, on_duty, is_on: false }
    }

    pub fn on(&mut self) { let _ = self.channel.set_duty_cycle(self.on_duty); self.is_on = true; }
    pub fn off(&mut self) { let _ = self.channel.set_duty_cycle(0); self.is_on = false; }
    pub fn toggle(&mut self) -> bool { if self.is_on { self.off(); } else { self.on(); } self.is_on }
    pub fn set_on_duty(&mut self, duty: u16) { self.on_duty = duty; if self.is_on { let _ = self.channel.set_duty_cycle(self.on_duty); } }
}
