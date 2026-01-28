//! GUI widgets for the OLED display

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle, RoundedRectangle},
    text::{Alignment, Text},
};
use heapless::String;
use core::fmt::Write;

use super::{DISPLAY_WIDTH, DISPLAY_HEIGHT, text_style, filled_style, outline_style};

/// Audio input source
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AudioSource {
    /// Analog line input via PCM1822 ADC
    LineIn,
    /// S/PDIF digital input
    Spdif,
    /// USB Audio Class 2
    Usb,
    /// No active source
    None,
}

impl AudioSource {
    pub fn name(&self) -> &'static str {
        match self {
            AudioSource::LineIn => "LINE IN",
            AudioSource::Spdif => "S/PDIF",
            AudioSource::Usb => "USB",
            AudioSource::None => "---",
        }
    }

    pub fn short_name(&self) -> &'static str {
        match self {
            AudioSource::LineIn => "LIN",
            AudioSource::Spdif => "SPD",
            AudioSource::Usb => "USB",
            AudioSource::None => "---",
        }
    }
}

/// Volume bar widget
///
/// Displays a horizontal volume bar with percentage and dB value
pub struct VolumeBar {
    /// Current volume (0-100%)
    pub volume: u8,
    /// Whether audio is muted
    pub muted: bool,
}

impl VolumeBar {
    pub fn new(volume: u8, muted: bool) -> Self {
        Self { volume, muted }
    }

    /// Draw the volume bar at the specified position
    ///
    /// The widget is 120x24 pixels
    pub fn draw<D>(&self, display: &mut D, position: Point) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        let bar_width = 100u32;
        let bar_height = 12u32;
        let bar_x = position.x + 14;
        let bar_y = position.y + 8;

        // Draw "VOL" label
        Text::new(
            "VOL",
            Point::new(position.x, position.y + 16),
            text_style(),
        )
        .draw(display)?;

        // Draw outer border
        RoundedRectangle::with_equal_corners(
            Rectangle::new(
                Point::new(bar_x, bar_y),
                Size::new(bar_width + 4, bar_height + 4),
            ),
            Size::new(3, 3),
        )
        .into_styled(outline_style())
        .draw(display)?;

        if self.muted {
            // Draw "MUTE" text centered in bar
            Text::with_alignment(
                "MUTE",
                Point::new(bar_x + (bar_width as i32 / 2) + 2, bar_y + 11),
                text_style(),
                Alignment::Center,
            )
            .draw(display)?;
        } else {
            // Draw filled portion based on volume
            let fill_width = (self.volume as u32 * bar_width) / 100;
            if fill_width > 0 {
                Rectangle::new(
                    Point::new(bar_x + 2, bar_y + 2),
                    Size::new(fill_width, bar_height),
                )
                .into_styled(filled_style())
                .draw(display)?;
            }
        }

        // Draw percentage value
        let mut vol_str: String<8> = String::new();
        if self.muted {
            let _ = write!(vol_str, "---");
        } else {
            let _ = write!(vol_str, "{}%", self.volume);
        }
        Text::with_alignment(
            &vol_str,
            Point::new(position.x + 120, position.y + 16),
            text_style(),
            Alignment::Right,
        )
        .draw(display)?;

        Ok(())
    }
}

/// Source indicator widget
///
/// Displays the current audio source with an icon
pub struct SourceIndicator {
    pub source: AudioSource,
    pub signal_detected: bool,
}

impl SourceIndicator {
    pub fn new(source: AudioSource, signal_detected: bool) -> Self {
        Self {
            source,
            signal_detected,
        }
    }

    /// Draw the source indicator
    ///
    /// Widget is approximately 60x12 pixels
    pub fn draw<D>(&self, display: &mut D, position: Point) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        // Draw source name
        Text::new(
            self.source.name(),
            Point::new(position.x, position.y + 10),
            text_style(),
        )
        .draw(display)?;

        // Draw signal indicator (small circle)
        let indicator_x = position.x + 50;
        let indicator_y = position.y + 4;

        if self.signal_detected {
            // Filled circle for signal present
            Rectangle::new(
                Point::new(indicator_x, indicator_y),
                Size::new(6, 6),
            )
            .into_styled(filled_style())
            .draw(display)?;
        } else {
            // Empty circle for no signal
            Rectangle::new(
                Point::new(indicator_x, indicator_y),
                Size::new(6, 6),
            )
            .into_styled(outline_style())
            .draw(display)?;
        }

        Ok(())
    }
}

/// Status bar widget for the top of the screen
pub struct StatusBar {
    pub source: AudioSource,
    pub sample_rate: u32,
    pub signal_present: bool,
}

impl StatusBar {
    pub fn new(source: AudioSource, sample_rate: u32, signal_present: bool) -> Self {
        Self {
            source,
            sample_rate,
            signal_present,
        }
    }

    /// Draw the status bar at the top of the screen
    ///
    /// Full width, 12 pixels high
    pub fn draw<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        // Draw source on the left
        Text::new(
            self.source.short_name(),
            Point::new(2, 10),
            text_style(),
        )
        .draw(display)?;

        // Draw sample rate in center
        let mut rate_str: String<12> = String::new();
        let _ = write!(rate_str, "{}kHz", self.sample_rate / 1000);
        Text::with_alignment(
            &rate_str,
            Point::new(64, 10),
            text_style(),
            Alignment::Center,
        )
        .draw(display)?;

        // Draw signal indicator on right
        if self.signal_present {
            Text::with_alignment(
                "SIG",
                Point::new(126, 10),
                text_style(),
                Alignment::Right,
            )
            .draw(display)?;
        }

        // Draw separator line
        Rectangle::new(
            Point::new(0, 13),
            Size::new(DISPLAY_WIDTH, 1),
        )
        .into_styled(filled_style())
        .draw(display)?;

        Ok(())
    }
}

/// Large volume display for the home screen
pub struct LargeVolumeDisplay {
    pub volume: u8,
    pub muted: bool,
}

impl LargeVolumeDisplay {
    pub fn new(volume: u8, muted: bool) -> Self {
        Self { volume, muted }
    }

    /// Draw large centered volume display
    pub fn draw<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        use embedded_graphics::mono_font::ascii::FONT_10X20;

        let large_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

        // Volume text - big and centered
        let mut vol_str: String<8> = String::new();
        if self.muted {
            let _ = write!(vol_str, "MUTE");
        } else {
            let _ = write!(vol_str, "{}%", self.volume);
        }

        Text::with_alignment(
            &vol_str,
            Point::new(64, 42),
            large_style,
            Alignment::Center,
        )
        .draw(display)?;

        Ok(())
    }
}

/// Simple progress bar for loading screens
pub struct ProgressBar {
    pub progress: u8, // 0-100
    pub label: &'static str,
}

impl ProgressBar {
    pub fn new(progress: u8, label: &'static str) -> Self {
        Self { progress, label }
    }

    pub fn draw<D>(&self, display: &mut D, position: Point) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        let bar_width = 100u32;
        let bar_height = 8u32;

        // Draw label
        Text::with_alignment(
            self.label,
            Point::new(64, position.y),
            text_style(),
            Alignment::Center,
        )
        .draw(display)?;

        // Draw bar outline
        let bar_x = (DISPLAY_WIDTH - bar_width - 4) as i32 / 2;
        let bar_y = position.y + 4;

        Rectangle::new(
            Point::new(bar_x, bar_y),
            Size::new(bar_width + 4, bar_height + 4),
        )
        .into_styled(outline_style())
        .draw(display)?;

        // Draw fill
        let fill_width = (self.progress as u32 * bar_width) / 100;
        if fill_width > 0 {
            Rectangle::new(
                Point::new(bar_x + 2, bar_y + 2),
                Size::new(fill_width, bar_height),
            )
            .into_styled(filled_style())
            .draw(display)?;
        }

        Ok(())
    }
}
