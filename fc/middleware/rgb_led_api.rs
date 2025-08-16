//! RGB LED middleware: high-level named colors & simple patterns built atop minimal driver.
//! The low-level driver just flips pins; here we offer semantic colors & blink/fade patterns.

use crate::drivers::rgb_led::{RgbLed, Polarity};
use embedded_hal::digital::OutputPin;

/// Logical color (boolean channels only).
#[derive(Copy, Clone)]
pub enum LedColor { Off, Red, Green, Blue, Yellow, Cyan, Magenta, White, Orange, Pink, Purple, Turquoise }

impl LedColor {
    pub fn channels(self) -> (bool, bool, bool) {
        use LedColor::*;
        match self {
            Off => (false,false,false),
            Red => (true,false,false),
            Green => (false,true,false),
            Blue => (false,false,true),
            Yellow | Orange => (true,true,false),
            Cyan | Turquoise => (false,true,true),
            Magenta | Pink | Purple => (true,false,true),
            White => (true,true,true),
        }
    }
}

/// High level LED controller providing pattern playback.
pub struct RgbLedController<R: OutputPin, G: OutputPin, B: OutputPin> {
    led: RgbLed<R,G,B>,
    pattern: Option<LedPattern>,
}

impl<R: OutputPin, G: OutputPin, B: OutputPin> RgbLedController<R,G,B> {
    pub fn new(led: RgbLed<R,G,B>) -> Self { Self { led, pattern: None } }
    pub fn inner_mut(&mut self) -> &mut RgbLed<R,G,B> { &mut self.led }

    pub fn set_color(&mut self, color: LedColor) { let (r,g,b)=color.channels(); self.led_set(r,g,b); }

    pub fn start_cycle(&mut self, now: u32, interval_loops: u32) { self.pattern = Some(LedPattern { start: now, interval: interval_loops }); }

    pub fn update(&mut self, now: u32) { if let Some(p)=self.pattern { let phase = ( (now - p.start) / p.interval ) % 6; match phase { 0=>self.set_color(LedColor::Red), 1=>self.set_color(LedColor::Orange), 2=>self.set_color(LedColor::Green), 3=>self.set_color(LedColor::Cyan), 4=>self.set_color(LedColor::Blue), _=>self.set_color(LedColor::Purple) } } }

    fn led_set(&mut self, r_on: bool, g_on: bool, b_on: bool) {
        match self.led.polarity() {
            Polarity::ActiveHigh => {
                let _ = if r_on { self.led.r_high() } else { self.led.r_low() };
                let _ = if g_on { self.led.g_high() } else { self.led.g_low() };
                let _ = if b_on { self.led.b_high() } else { self.led.b_low() };
            }
            Polarity::ActiveLow => {
                let _ = if r_on { self.led.r_low() } else { self.led.r_high() };
                let _ = if g_on { self.led.g_low() } else { self.led.g_high() };
                let _ = if b_on { self.led.b_low() } else { self.led.b_high() };
            }
        }
    }
}

#[derive(Copy, Clone)]
struct LedPattern { start: u32, interval: u32 }
