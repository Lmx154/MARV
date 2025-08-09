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
    pattern: PatternState,
}

/// Simple predefined tone duty presets (assuming frequency set outside).
/// These just change how "strong" the tone is (duty cycle intensity).
#[allow(dead_code)]
pub enum Tone {
    /// Silence (turn off)
    Off,
    /// Soft beep (~25% duty)
    Soft,
    /// Normal (~50%)
    Normal,
    /// Loud (~75%)
    Loud,
    /// ARM siren (frequency sweep – call `arm_siren_step` each loop while on)
    Arm,
}

impl<C: SetDutyCycle> Buzzer<C> {
    pub fn new(mut channel: C, on_duty: u16) -> Self {
        let _ = channel.set_duty_cycle(0); // start silent
    Self { channel, on_duty, is_on: false, pattern: PatternState::Idle }
    }

    pub fn on(&mut self) { let _ = self.channel.set_duty_cycle(self.on_duty); self.is_on = true; }
    pub fn off(&mut self) { let _ = self.channel.set_duty_cycle(0); self.is_on = false; }
    pub fn toggle(&mut self) -> bool { if self.is_on { self.off(); } else { self.on(); } self.is_on }
    #[allow(dead_code)]
    pub fn set_on_duty(&mut self, duty: u16) { self.on_duty = duty; if self.is_on { let _ = self.channel.set_duty_cycle(self.on_duty); } }
    pub fn is_on(&self) -> bool { self.is_on }

    /// Apply a predefined tone duty. This does NOT modify frequency.
    #[allow(dead_code)]
    pub fn set_tone(&mut self, tone: Tone) {
        match tone {
            Tone::Off => { self.off(); return; }
            Tone::Soft => self.set_on_duty(self.on_duty.saturating_mul(25).saturating_div(50)), // scale to ~25%
            Tone::Normal => {/* keep existing on_duty */}
            Tone::Loud => {
                // push toward 75% (approx) by scaling
                let new = (self.on_duty as u32 * 3 / 2).min(u16::MAX as u32) as u16;
                self.set_on_duty(new);
            }
            Tone::Arm => { self.on(); }
        }
    }

    /// Siren-style sweeping helper: adjust the provided PWM TOP value between
    /// `min_top` and `max_top` in a triangle wave pattern based on `step`.
    /// Call this frequently (e.g. every loop) while the buzzer is ON.
    ///
    /// You must pass a closure that actually updates the hardware slice TOP.
    /// Example from main (with `pwm5` slice still in scope):
    /// ```ignore
    /// if buzzer_is_armed { buzzer.arm_siren_step(step, 300, 1200, |t| pwm5.set_top(t)); }
    /// ```
    /// ARM siren style modulation by varying duty cycle (amplitude) between
    /// min_percent and max_percent (inclusive). Does not alter frequency so
    /// it avoids needing the PWM slice after moving the channel out.
    /// Call every loop with an increasing step counter while the buzzer is ON.
    pub fn arm_siren_modulate(&mut self, step: u32, min_percent: u8, max_percent: u8) {
        if !self.is_on { return; }
        if min_percent >= max_percent { return; }
        let min_p = min_percent as u32;
        let max_p = max_percent as u32;
        let span = max_p - min_p;
        let pos = step % (2 * span);
        let rel = if pos < span { pos } else { 2 * span - pos };
        let current_percent = min_p + rel; // in [min_p, max_p]
        // Convert percent to duty relative to original on_duty baseline (50% default)
        // We'll scale up to original TOP-relative on_duty * (current_percent/100)
        let scaled = (self.on_duty as u32 * current_percent) / 100;
        let _ = self.channel.set_duty_cycle(scaled as u16);
    }

    /* ---------------- Pattern API ----------------

       We provide two timed patterns managed by calling update(now_counter) each loop:
       - ARM: sweeping siren for a fixed duration (in loop counts) then auto stop.
       - ACK: long beep then two short beeps.

       Call start_arm(now, duration_loops) or start_ack(now, loops_per_second).
       Then call update(now) every iteration. When a pattern completes, update returns true.
    */

    /// Start ARM pattern for `total_seconds`: 1.0s ON (with modulation), 0.5s OFF repeating.
    pub fn start_arm(&mut self, now: u32, loops_per_second: u32, total_seconds: u32) {
        let total_loops = loops_per_second * total_seconds;
        self.pattern = PatternState::Arm { start: now, duration: total_loops, lps: loops_per_second };
        self.on();
    }

    /// Start ACK pattern: faster long beep + two quick beeps.
    pub fn start_ack(&mut self, now: u32, loops_per_second: u32) {
        // New faster timing: long = 0.4s, gap = 0.08s, short = 0.1s
        let long = loops_per_second * 2 / 5;          // 0.4s
        let gap  = loops_per_second / 12;             // ~0.083s
        let short = loops_per_second / 10;            // 0.1s
        self.pattern = PatternState::Ack {
            start: now,
            long,
            gap,
            short,
        };
        self.on();
    }

    /// Update pattern state. Call every loop with a monotonically increasing counter.
    /// Returns true exactly once when a pattern finishes.
    pub fn update(&mut self, now: u32) -> bool {
        match self.pattern {
            PatternState::Idle => return false,
            PatternState::Arm { start, duration, lps } => {
                let elapsed = now - start;
                if elapsed >= duration { // done
                    self.off();
                    self.pattern = PatternState::Idle;
                    return true;
                }
                let on_loops = lps;        // 1.0s ON
                let off_loops = lps / 2;    // 0.5s OFF
                let cycle = on_loops + off_loops;
                let in_cycle = elapsed % cycle;
                if in_cycle < on_loops {
                    self.on();
                    self.arm_siren_modulate(elapsed, 40, 100); // stronger modulation range
                } else {
                    self.off();
                }
            }
            PatternState::Ack { start, long, gap, short } => {
                let elapsed = now - start;
                let p1_end = long;                          // long beep ON
                let p2_end = p1_end + gap;                  // gap1 OFF
                let p3_end = p2_end + short;                // short beep1 ON
                let p4_end = p3_end + gap;                  // gap2 OFF
                let p5_end = p4_end + short;                // short beep2 ON

                if elapsed < p1_end { // long beep
                    self.on();
                } else if elapsed < p2_end { // gap
                    self.off();
                } else if elapsed < p3_end { // beep1
                    self.on();
                } else if elapsed < p4_end { // gap2
                    self.off();
                } else if elapsed < p5_end { // beep2
                    self.on();
                } else { // finished
                    self.off();
                    self.pattern = PatternState::Idle;
                    return true;
                }
            }
        }
        false
    }
}

#[derive(Copy, Clone)]
enum PatternState {
    Idle,
    Arm { start: u32, duration: u32, lps: u32 },
    Ack { start: u32, long: u32, gap: u32, short: u32 },
}
