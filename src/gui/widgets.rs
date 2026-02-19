//! GUI widgets for the OLED display

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, ascii::FONT_6X9, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle, RoundedRectangle},
    text::{Alignment, Text},
};
use heapless::String;

use super::{filled_style, outline_style, text_style, DISPLAY_HEIGHT, DISPLAY_WIDTH};

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
            AudioSource::LineIn => "LINE",
            AudioSource::Spdif => "S/PDIF",
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
        Text::new("VOL", Point::new(position.x, position.y + 16), text_style()).draw(display)?;

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
            Rectangle::new(Point::new(indicator_x, indicator_y), Size::new(6, 6))
                .into_styled(filled_style())
                .draw(display)?;
        } else {
            // Empty circle for no signal
            Rectangle::new(Point::new(indicator_x, indicator_y), Size::new(6, 6))
                .into_styled(outline_style())
                .draw(display)?;
        }

        Ok(())
    }
}

/// Signal status for the status bar
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum SignalStatus {
    /// No signal (connected but receiving silence)
    NoSignal,
    /// Signal present and OK
    Ok,
    /// Signal is clipping
    Clip,
}

/// Status bar widget for the top of the screen
pub struct StatusBar {
    pub source: AudioSource,
    pub sample_rate: u32,
    pub signal_status: SignalStatus,
}

impl StatusBar {
    pub fn new(source: AudioSource, sample_rate: u32, signal_status: SignalStatus) -> Self {
        Self {
            source,
            sample_rate,
            signal_status,
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
        Text::new(self.source.short_name(), Point::new(2, 10), text_style()).draw(display)?;

        // Draw sample rate in center
        let mut rate_str: String<12> = String::new();
        if self.sample_rate == 44100 {
            let _ = write!(rate_str, "44.1kHz");
        } else {
            let _ = write!(rate_str, "{}kHz", self.sample_rate / 1000);
        }
        Text::with_alignment(
            &rate_str,
            Point::new(64, 10),
            text_style(),
            Alignment::Center,
        )
        .draw(display)?;

        // Draw signal status on right
        let status_text = match self.signal_status {
            SignalStatus::NoSignal => "NO SIG",
            SignalStatus::Ok => "SIG OK",
            SignalStatus::Clip => "CLIP",
        };
        Text::with_alignment(
            status_text,
            Point::new(126, 10),
            text_style(),
            Alignment::Right,
        )
        .draw(display)?;

        // Draw separator line
        Rectangle::new(Point::new(0, 13), Size::new(DISPLAY_WIDTH, 1))
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

    /// Draw large centered volume display at default position
    pub fn draw<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        self.draw_at(display, 42)
    }

    /// Draw large centered volume display at a custom Y position
    pub fn draw_at<D>(&self, display: &mut D, y: i32) -> Result<(), D::Error>
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

        Text::with_alignment(&vol_str, Point::new(64, y), large_style, Alignment::Center)
            .draw(display)?;

        Ok(())
    }
}

/// Level meter widget with bar graph and peak hold
///
/// Draws two horizontal bars (L and R) between the status bar and volume display.
/// Each bar shows the current level as a filled rectangle and the peak hold as
/// a 1px-wide vertical line that decays slowly.
pub struct LevelMeter {
    pub left_level: u8,  // 0-100
    pub right_level: u8, // 0-100
    pub left_peak: u8,   // 0-100
    pub right_peak: u8,  // 0-100
}

impl LevelMeter {
    /// Bar area: starts at x=10 (after label), 114px wide
    const BAR_X: i32 = 10;
    const BAR_WIDTH: u32 = 114;
    const BAR_HEIGHT: u32 = 4;

    pub fn new(left_level: u8, right_level: u8, left_peak: u8, right_peak: u8) -> Self {
        Self {
            left_level,
            right_level,
            left_peak,
            right_peak,
        }
    }

    /// Draw at a given Y position. Uses ~14px of vertical space.
    pub fn draw<D>(&self, display: &mut D, y: i32) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        // Left channel
        self.draw_channel(display, "L", self.left_level, self.left_peak, y)?;
        // Right channel
        self.draw_channel(display, "R", self.right_level, self.right_peak, y + 7)?;
        Ok(())
    }

    /// Draw only the bar fills and peak holds (no outlines or labels).
    ///
    /// Use this for fast partial updates — caller should clear the bar
    /// interiors first. Only dirties pages containing the fill pixels.
    pub fn draw_fills<D>(&self, display: &mut D, y: i32) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        self.draw_fill(display, self.left_level, self.left_peak, y)?;
        self.draw_fill(display, self.right_level, self.right_peak, y + 7)?;
        Ok(())
    }

    fn draw_fill<D>(&self, display: &mut D, level: u8, peak: u8, y: i32) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        // Filled bar (current level) — inside the 1px outline
        let fill_width = (level as u32 * Self::BAR_WIDTH) / 100;
        if fill_width > 0 {
            Rectangle::new(
                Point::new(Self::BAR_X + 1, y + 1),
                Size::new(fill_width, Self::BAR_HEIGHT),
            )
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(display)?;
        }

        // Peak hold indicator (1px wide vertical line)
        if peak > 0 {
            let peak_x = Self::BAR_X
                + 1
                + (peak as u32 * Self::BAR_WIDTH / 100).min(Self::BAR_WIDTH - 1) as i32;
            Rectangle::new(Point::new(peak_x, y + 1), Size::new(1, Self::BAR_HEIGHT))
                .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
                .draw(display)?;
        }

        Ok(())
    }

    fn draw_channel<D>(
        &self,
        display: &mut D,
        label: &str,
        level: u8,
        peak: u8,
        y: i32,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        let small_style = MonoTextStyle::new(&FONT_6X9, BinaryColor::On);

        // Label
        Text::new(label, Point::new(2, y + 5), small_style).draw(display)?;

        // Bar outline
        Rectangle::new(
            Point::new(Self::BAR_X, y),
            Size::new(Self::BAR_WIDTH + 2, Self::BAR_HEIGHT + 2),
        )
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(display)?;

        // Filled bar (current level)
        let fill_width = (level as u32 * Self::BAR_WIDTH) / 100;
        if fill_width > 0 {
            Rectangle::new(
                Point::new(Self::BAR_X + 1, y + 1),
                Size::new(fill_width, Self::BAR_HEIGHT),
            )
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(display)?;
        }

        // Peak hold indicator (1px wide vertical line)
        if peak > 0 {
            let peak_x = Self::BAR_X
                + 1
                + (peak as u32 * Self::BAR_WIDTH / 100).min(Self::BAR_WIDTH - 1) as i32;
            Rectangle::new(Point::new(peak_x, y + 1), Size::new(1, Self::BAR_HEIGHT))
                .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
                .draw(display)?;
        }

        Ok(())
    }
}

/// CLIP warning overlay
///
/// Draws a large flashing "CLIP" warning centered on the display,
/// replacing the volume readout.
pub struct ClipWarning;

impl ClipWarning {
    /// Draw the CLIP warning at the specified Y center position
    pub fn draw<D>(display: &mut D, y_center: i32) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        use embedded_graphics::mono_font::ascii::FONT_10X20;

        // Draw filled background rectangle for contrast
        let rect_w: u32 = 60;
        let rect_h: u32 = 24;
        let rect_x = (super::DISPLAY_WIDTH as i32 - rect_w as i32) / 2;
        let rect_y = y_center - rect_h as i32 / 2;

        Rectangle::new(Point::new(rect_x, rect_y), Size::new(rect_w, rect_h))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(display)?;

        // Draw "CLIP" text in inverted color
        let clip_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::Off);
        Text::with_alignment(
            "CLIP",
            Point::new(super::DISPLAY_WIDTH as i32 / 2, y_center + 6),
            clip_style,
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
