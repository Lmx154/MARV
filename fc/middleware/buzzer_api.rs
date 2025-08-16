//! High-level buzzer middleware: predefined patterns (ARM, ACK) built atop the minimal driver.
//! Keeps timing/state separate from low-level PWM duty management.

use crate::drivers::buzzer::Buzzer;

/// Active pattern kinds.
#[derive(Copy, Clone, Eq, PartialEq)]
pub enum BuzzerPatternKind { Arm, Ack }

/// Internal pattern state machine.
pub struct BuzzerPattern {
    kind: BuzzerPatternKind,
    start: u32,
    loops_per_sec: u32,
    // Derived timing for Ack
    long: u32,
    gap: u32,
    short: u32,
    // Total duration for Arm
    total_loops: u32,
}

impl BuzzerPattern {
    fn new_arm(now: u32, loops_per_sec: u32, total_seconds: u32) -> Self {
        Self { kind: BuzzerPatternKind::Arm, start: now, loops_per_sec, long: 0, gap: 0, short: 0, total_loops: loops_per_sec * total_seconds }
    }
    fn new_ack(now: u32, loops_per_sec: u32) -> Self {
        // Faster timing: 0.4s, gap 0.08s, short 0.1s
        let long = loops_per_sec * 2 / 5;  // 0.4s
        let gap  = loops_per_sec / 12;     // ~0.083s
        let short= loops_per_sec / 10;     // 0.1s
        Self { kind: BuzzerPatternKind::Ack, start: now, loops_per_sec, long, gap, short, total_loops: 0 }
    }
}

/// Controller that owns no hardware; it drives a provided buzzer driver each update.
pub struct BuzzerController {
    pattern: Option<BuzzerPattern>,
}

impl BuzzerController {
    pub fn new() -> Self { Self { pattern: None } }

    pub fn start_arm(&mut self, now: u32, loops_per_sec: u32, total_seconds: u32, buzzer: &mut Buzzer<impl embedded_hal::pwm::SetDutyCycle>) {
        self.pattern = Some(BuzzerPattern::new_arm(now, loops_per_sec, total_seconds));
        buzzer.on();
    }

    pub fn start_ack(&mut self, now: u32, loops_per_sec: u32, buzzer: &mut Buzzer<impl embedded_hal::pwm::SetDutyCycle>) {
        self.pattern = Some(BuzzerPattern::new_ack(now, loops_per_sec));
        buzzer.on();
    }

    /// Update buzzer according to active pattern. Returns true when a pattern finishes.
    pub fn update(&mut self, now: u32, buzzer: &mut Buzzer<impl embedded_hal::pwm::SetDutyCycle>) -> bool {
        let Some(p) = &self.pattern else { return false; };
        match p.kind {
            BuzzerPatternKind::Arm => {
                let elapsed = now - p.start;
                if elapsed >= p.total_loops {
                    buzzer.off();
                    self.pattern = None;
                    return true;
                }
                // 1s ON / 0.5s OFF cycle
                let on_loops = p.loops_per_sec;
                let off_loops = p.loops_per_sec / 2;
                let cycle = on_loops + off_loops;
                let in_cycle = elapsed % cycle;
                if in_cycle < on_loops {
                    buzzer.on();
                    // Simple amplitude modulation by tweaking duty fractionally (40..100%)
                    let span = 60; // percent span
                    let pos = (elapsed / 4) % (2 * span); // slower ramp
                    let rel = if pos < span { pos } else { 2 * span - pos };
                    let percent = 40 + rel; // 40..100
                    let duty = (buzzer.base_duty() as u32 * percent) / 100;
                    let _ = buzzer.inner_channel_mut().set_duty_cycle(duty as u16);
                } else {
                    buzzer.off();
                }
            }
            BuzzerPatternKind::Ack => {
                let elapsed = now - p.start;
                let p1_end = p.long;
                let p2_end = p1_end + p.gap;
                let p3_end = p2_end + p.short;
                let p4_end = p3_end + p.gap;
                let p5_end = p4_end + p.short;
                if elapsed < p1_end { buzzer.on(); }
                else if elapsed < p2_end { buzzer.off(); }
                else if elapsed < p3_end { buzzer.on(); }
                else if elapsed < p4_end { buzzer.off(); }
                else if elapsed < p5_end { buzzer.on(); }
                else {
                    buzzer.off();
                    self.pattern = None;
                    return true;
                }
            }
        }
        false
    }

    pub fn is_active(&self) -> bool { self.pattern.is_some() }
}
