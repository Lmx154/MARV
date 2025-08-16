use embedded_hal::pwm::SetDutyCycle;

/// Minimal passive buzzer driver: only controls duty (on/off) for a configured PWM channel.
pub struct Buzzer<C: SetDutyCycle> {
    channel: C,
    duty_on: u16,
    is_on: bool,
}

impl<C: SetDutyCycle> Buzzer<C> {
    pub fn new(mut channel: C, duty_on: u16) -> Self {
        let _ = channel.set_duty_cycle(0);
        Self { channel, duty_on, is_on: false }
    }
    pub fn on(&mut self) { let _ = self.channel.set_duty_cycle(self.duty_on); self.is_on = true; }
    pub fn off(&mut self) { let _ = self.channel.set_duty_cycle(0); self.is_on = false; }
    pub fn is_on(&self) -> bool { self.is_on }
    pub fn base_duty(&self) -> u16 { self.duty_on }
    pub fn set_on_duty(&mut self, duty: u16) { self.duty_on = duty; if self.is_on { let _ = self.channel.set_duty_cycle(self.duty_on); } }
    pub fn toggle(&mut self) -> bool { if self.is_on { self.off(); } else { self.on(); } self.is_on }
    pub fn inner_channel_mut(&mut self) -> &mut C { &mut self.channel }
}
