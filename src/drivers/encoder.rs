//! Rotary encoder driver with quadrature decoding
//!
//! Handles the rotary encoder on GPIO 8-10 with debouncing.

use embassy_rp::gpio::{Input, Pin, Pull};
use embassy_rp::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant};

/// Minimum time between encoder events (debounce)
const DEBOUNCE_MS: u64 = 5;

/// Encoder events
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum EncoderEvent {
    /// Rotated clockwise (increment)
    Increment,
    /// Rotated counter-clockwise (decrement)
    Decrement,
    /// Button pressed
    Press,
    /// Button released
    Release,
    /// Button held for long press
    LongPress,
}

/// Rotary encoder state
pub struct RotaryEncoder<'d> {
    pin_a: Input<'d>,
    pin_b: Input<'d>,
    pin_btn: Input<'d>,
    last_a: bool,
    last_b: bool,
    last_event_time: Instant,
    btn_press_time: Option<Instant>,
}

impl<'d> RotaryEncoder<'d> {
    /// Create a new rotary encoder driver
    ///
    /// Uses internal pull-ups (can be changed to Pull::None if external pull-ups are present)
    pub fn new(
        pin_a: Peri<'d, impl Pin>,
        pin_b: Peri<'d, impl Pin>,
        pin_btn: Peri<'d, impl Pin>,
    ) -> Self {
        let pin_a = Input::new(pin_a, Pull::Up);
        let pin_b = Input::new(pin_b, Pull::Up);
        let pin_btn = Input::new(pin_btn, Pull::Up);

        let last_a = pin_a.is_high();
        let last_b = pin_b.is_high();

        Self {
            pin_a,
            pin_b,
            pin_btn,
            last_a,
            last_b,
            last_event_time: Instant::now(),
            btn_press_time: None,
        }
    }

    /// Poll the encoder and return any events
    ///
    /// Uses quadrature decoding to determine direction.
    /// Returns None if no event or within debounce period.
    pub fn poll(&mut self) -> Option<EncoderEvent> {
        let now = Instant::now();

        // Check button state
        let btn_pressed = self.pin_btn.is_low(); // Active low with pull-up

        if let Some(press_time) = self.btn_press_time {
            if !btn_pressed {
                // Button released
                self.btn_press_time = None;
                let held_duration = now - press_time;
                if held_duration > Duration::from_millis(500) {
                    return Some(EncoderEvent::LongPress);
                } else {
                    return Some(EncoderEvent::Release);
                }
            }
        } else if btn_pressed {
            self.btn_press_time = Some(now);
            return Some(EncoderEvent::Press);
        }

        // Debounce check for rotation
        if now - self.last_event_time < Duration::from_millis(DEBOUNCE_MS) {
            return None;
        }

        // Read current state
        let a = self.pin_a.is_high();
        let b = self.pin_b.is_high();

        // Quadrature decoding
        // On A edge: if A == B -> CW, else CCW
        let event = if a != self.last_a {
            self.last_event_time = now;
            if a == b {
                Some(EncoderEvent::Increment)
            } else {
                Some(EncoderEvent::Decrement)
            }
        } else {
            None
        };

        self.last_a = a;
        self.last_b = b;

        event
    }

    /// Wait for any edge on encoder pins (async)
    pub async fn wait_for_edge(&mut self) {
        // Wait for any change on the encoder pins
        embassy_futures::select::select3(
            self.pin_a.wait_for_any_edge(),
            self.pin_b.wait_for_any_edge(),
            self.pin_btn.wait_for_any_edge(),
        )
        .await;
    }
}

/// Global encoder event signal for async notification
pub static ENCODER_SIGNAL: Signal<CriticalSectionRawMutex, EncoderEvent> = Signal::new();
