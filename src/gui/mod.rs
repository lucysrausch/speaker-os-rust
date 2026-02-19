//! GUI system using embedded-graphics
//!
//! Provides widgets and screens for the OLED display menu system.

pub mod display;
pub mod menu;
pub mod screens;
pub mod widgets;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, ascii::FONT_9X15_BOLD, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
};

pub use menu::{Menu, MenuAction, MenuItem};
pub use screens::ScreenId;
pub use widgets::{SourceIndicator, StatusBar, VolumeBar};

/// Display dimensions for SSD1306 128x64
pub const DISPLAY_WIDTH: u32 = 128;
pub const DISPLAY_HEIGHT: u32 = 64;

/// Common text styles
pub fn title_style() -> MonoTextStyle<'static, BinaryColor> {
    MonoTextStyle::new(&FONT_9X15_BOLD, BinaryColor::On)
}

pub fn text_style() -> MonoTextStyle<'static, BinaryColor> {
    MonoTextStyle::new(&FONT_6X10, BinaryColor::On)
}

pub fn text_style_inverted() -> MonoTextStyle<'static, BinaryColor> {
    MonoTextStyle::new(&FONT_6X10, BinaryColor::Off)
}

/// Common primitive styles
pub fn filled_style() -> PrimitiveStyle<BinaryColor> {
    PrimitiveStyle::with_fill(BinaryColor::On)
}

pub fn outline_style() -> PrimitiveStyle<BinaryColor> {
    PrimitiveStyle::with_stroke(BinaryColor::On, 1)
}

/// Clear the entire display
pub fn clear_display<D>(display: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = BinaryColor>,
{
    Rectangle::new(Point::zero(), Size::new(DISPLAY_WIDTH, DISPLAY_HEIGHT))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(display)
}
