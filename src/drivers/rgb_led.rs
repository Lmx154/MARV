//! Simple RGB LED driver for common-anode or common-cathode RGB LEDs.
//!
//! This driver manages three GPIO pins for Red, Green, and Blue channels.
//! It supports both active-high (common cathode) and active-low (common anode)
//! wiring by inverting logic as needed.

use embedded_hal::digital::OutputPin;

/// RGB LED polarity
#[derive(Copy, Clone)]
pub enum Polarity {
    /// Active-high (set_high turns the LED channel on). Typical for common-cathode.
    ActiveHigh,
    /// Active-low (set_low turns the LED channel on). Typical for common-anode.
    ActiveLow,
}

/// Common named colors (additive RGB, boolean channels only).
/// For more granular colors (with PWM), a future enhancement would accept duty values.
#[derive(Copy, Clone)]
pub enum Color {
    Off,
    Red,
    Green,
    Blue,
    Yellow,   // R + G
    Cyan,     // G + B
    Magenta,  // R + B
    White,    // R + G + B
    Orange,   // R + (G) ~ approximate using R+G
    Pink,     // Magenta approximation
    Purple,   // Magenta alias
    // Additional readable aliases
    Turquoise, // Cyan alias
}

/// A simple RGB LED composed of three output-capable pins.
/// The pins can be any type implementing `embedded_hal::digital::OutputPin`.
pub struct RgbLed<R, G, B>
where
    R: OutputPin,
    G: OutputPin,
    B: OutputPin,
{
    r: R,
    g: G,
    b: B,
    polarity: Polarity,
}

impl<R, G, B> RgbLed<R, G, B>
where
    R: OutputPin,
    G: OutputPin,
    B: OutputPin,
{
    pub fn new(r: R, g: G, b: B, polarity: Polarity) -> Self {
        Self { r, g, b, polarity }
    }

    /// Turn all channels off.
    pub fn off(&mut self) {
        match self.polarity {
            Polarity::ActiveHigh => {
                let _ = self.r.set_low();
                let _ = self.g.set_low();
                let _ = self.b.set_low();
            }
            Polarity::ActiveLow => {
                let _ = self.r.set_high();
                let _ = self.g.set_high();
                let _ = self.b.set_high();
            }
        }
    }

    /// Set via enum Color for quick use.
    pub fn set_color(&mut self, color: Color) {
        use Color::*;
        match color {
            Off => self.set(false, false, false),
            Red => self.set(true, false, false),
            Green => self.set(false, true, false),
            Blue => self.set(false, false, true),
            Yellow => self.set(true, true, false),
            Cyan | Turquoise => self.set(false, true, true),
            Magenta | Pink | Purple => self.set(true, false, true),
            White => self.set(true, true, true),
            Orange => self.set(true, true, false), // same as yellow for boolean pins
        }
    }

    /// Set the LED to red only.
    pub fn red(&mut self) { self.set_color(Color::Red); }

    /// Set the LED to green only.
    pub fn green(&mut self) { self.set_color(Color::Green); }

    /// Set the LED to blue only.
    pub fn blue(&mut self) { self.set_color(Color::Blue); }

    /// Set the LED to yellow (red + green).
    pub fn yellow(&mut self) { self.set_color(Color::Yellow); }

    /// Set the LED to magenta/pink (red + blue).
    pub fn magenta(&mut self) { self.set_color(Color::Magenta); }

    /// Set the LED to cyan/turquoise (green + blue).
    pub fn cyan(&mut self) { self.set_color(Color::Cyan); }

    /// Set the LED to white (all colors).
    pub fn white(&mut self) { self.set_color(Color::White); }

    /// Convenience: orange (alias of yellow with discrete pins)
    pub fn orange(&mut self) { self.set_color(Color::Orange); }

    /// Convenience: pink (magenta alias)
    pub fn pink(&mut self) { self.set_color(Color::Pink); }

    /// Convenience: purple (magenta alias)
    pub fn purple(&mut self) { self.set_color(Color::Purple); }

    /// Internal helper to set individual channels.
    fn set(&mut self, r_on: bool, g_on: bool, b_on: bool) {
        match self.polarity {
            Polarity::ActiveHigh => {
                let _ = if r_on { self.r.set_high() } else { self.r.set_low() };
                let _ = if g_on { self.g.set_high() } else { self.g.set_low() };
                let _ = if b_on { self.b.set_high() } else { self.b.set_low() };
            }
            Polarity::ActiveLow => {
                let _ = if r_on { self.r.set_low() } else { self.r.set_high() };
                let _ = if g_on { self.g.set_low() } else { self.g.set_high() };
                let _ = if b_on { self.b.set_low() } else { self.b.set_high() };
            }
        }
    }

    /// Consume and return the underlying pins.
    pub fn release(self) -> (R, G, B) { (self.r, self.g, self.b) }
}
