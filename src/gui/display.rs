//! Minimal SH1106 OLED driver with dirty-page tracking and async I2C
//!
//! Uses `embedded_hal_async::i2c::I2c` so that each I2C write yields to the
//! embassy executor. This prevents display updates from blocking USB and S/PDIF
//! audio tasks on Core 0.
//!
//! Only the pages that were drawn to since the last flush are sent over I2C,
//! reducing display update time from ~20ms (full) to ~2.5ms per dirty page.

use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
use embedded_hal_async::i2c::I2c;

/// SH1106 display width/height
const WIDTH: u32 = 128;
const HEIGHT: u32 = 64;
const PAGES: usize = (HEIGHT as usize) / 8;
const BUF_SIZE: usize = WIDTH as usize * PAGES; // 1024 bytes

/// SH1106 column offset (132-column controller, 128-column display)
const COL_OFFSET: u8 = 2;

/// SH1106 OLED display with page-level dirty tracking
pub struct Sh1106<I2C> {
    i2c: I2C,
    addr: u8,
    buffer: [u8; BUF_SIZE],
    /// Bitmask of pages that need flushing (bit 0 = page 0, etc.)
    dirty: u8,
    ///
    enabled: bool,
}

impl<I2C: I2c> Sh1106<I2C> {
    /// Create a new display driver. Call `init()` before use.
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            buffer: [0; BUF_SIZE],
            dirty: 0xFF, // All pages dirty initially
            enabled: false,
        }
    }

    /// Send a command (1-2 bytes) to the SH1106
    async fn cmd(&mut self, bytes: &[u8]) -> Result<(), I2C::Error> {
        match bytes.len() {
            1 => self.i2c.write(self.addr, &[0x00, bytes[0]]).await,
            2 => self.i2c.write(self.addr, &[0x00, bytes[0], bytes[1]]).await,
            _ => Ok(()),
        }
    }

    /// Initialize the SH1106 display controller (128x64, rotated 180°)
    pub async fn init(&mut self) -> Result<(), I2C::Error> {
        self.cmd(&[0xAE]).await?; // Display OFF
        self.cmd(&[0xD5, 0x80]).await?; // Set display clock divide ratio
        self.cmd(&[0xA8, 0x3F]).await?; // Set multiplex ratio (63 = 64 rows)
        self.cmd(&[0xD3, 0x00]).await?; // Set display offset: 0
        self.cmd(&[0x40]).await?; // Set start line: 0
        self.cmd(&[0x8D, 0x14]).await?; // Charge pump enabled
        self.cmd(&[0x20, 0x02]).await?; // Page addressing mode
        self.cmd(&[0xA0]).await?; // Segment remap off (normal)
        self.cmd(&[0xC0]).await?; // COM scan normal
        self.cmd(&[0xDA, 0x12]).await?; // Set COM pins configuration
        self.cmd(&[0x81, 0xCF]).await?; // Set contrast
        self.cmd(&[0xD9, 0xF1]).await?; // Set pre-charge period
        self.cmd(&[0xDB, 0x40]).await?; // Set VCOMH deselect level
        self.cmd(&[0xA4]).await?; // Display follows RAM content
        self.cmd(&[0xA6]).await?; // Normal display (not inverted)
        self.cmd(&[0xAF]).await?; // Display ON
        self.enabled = true;
        Ok(())
    }

    /// Initialize the SH1106 display controller (128x64, rotated 180°)
    pub async fn disable_display(&mut self) -> Result<(), I2C::Error> {
        if self.enabled {
            self.enabled = false;
            self.cmd(&[0xAE]).await?; // Display OFF
        }
        Ok(())
    }

    pub async fn enable_display(&mut self) -> Result<(), I2C::Error> {
        if !self.enabled {
            self.enabled = true;
            self.cmd(&[0xAF]).await?; // Display ON
        }
        Ok(())
    }

    /// Flush only dirty pages to the display (async — yields between pages).
    ///
    /// Each page is 128 bytes = ~2.5ms at 400kHz I2C.
    /// The executor can run other tasks between page writes.
    pub async fn flush(&mut self) -> Result<(), I2C::Error> {
        let dirty = self.dirty;
        if dirty == 0 {
            return Ok(());
        }
        self.dirty = 0;

        for page in 0..PAGES {
            if dirty & (1 << page) != 0 {
                self.flush_page(page as u8).await?;
            }
        }
        Ok(())
    }

    /// Flush at most one dirty page to the display.
    ///
    /// Returns `true` if a page was flushed (more may remain dirty).
    /// Returns `false` if no pages are dirty.
    ///
    /// Call this once per main loop tick to spread I2C writes over time,
    /// preventing audio stutter from bursty I2C activity.
    pub async fn flush_one_page(&mut self) -> Result<bool, I2C::Error> {
        if self.dirty == 0 {
            return Ok(false);
        }
        // Find lowest dirty page
        for page in 0..PAGES {
            if self.dirty & (1 << page) != 0 {
                self.dirty &= !(1 << page);
                self.flush_page(page as u8).await?;
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Check if any pages are dirty (need flushing)
    pub fn has_dirty_pages(&self) -> bool {
        self.dirty != 0
    }

    /// Flush all pages regardless of dirty state.
    pub async fn flush_all(&mut self) -> Result<(), I2C::Error> {
        self.dirty = 0xFF;
        self.flush().await
    }

    /// Clear a rectangular region in the framebuffer and mark those pages dirty
    pub fn clear_region(&mut self, x: u32, y: u32, width: u32, height: u32) {
        for row in y..y + height {
            if row >= HEIGHT {
                break;
            }
            let page = (row / 8) as usize;
            let bit = row % 8;
            let mask = !(1u8 << bit);
            for col in x..x + width {
                if col >= WIDTH {
                    break;
                }
                self.buffer[page * WIDTH as usize + col as usize] &= mask;
            }
            self.dirty |= 1 << page;
        }
    }

    /// Write a single page (128 bytes) to the display
    async fn flush_page(&mut self, page: u8) -> Result<(), I2C::Error> {
        // Set page address and column
        let cmd = [
            0x00,                     // Control byte: commands follow
            0xB0 | page,              // Set page address
            COL_OFFSET & 0x0F,        // Lower column address nibble
            0x10 | (COL_OFFSET >> 4), // Upper column address nibble
        ];
        self.i2c.write(self.addr, &cmd).await?;

        // Send page data with 0x40 prefix (data mode)
        let mut data = [0u8; 129];
        data[0] = 0x40; // Control byte: data follows
        let offset = page as usize * WIDTH as usize;
        data[1..129].copy_from_slice(&self.buffer[offset..offset + WIDTH as usize]);
        self.i2c.write(self.addr, &data).await
    }

    /// Fill the entire framebuffer with a color and mark all pages dirty
    pub fn clear_all(&mut self, color: BinaryColor) {
        let byte = match color {
            BinaryColor::On => 0xFF,
            BinaryColor::Off => 0x00,
        };
        self.buffer.fill(byte);
        self.dirty = 0xFF;
    }
}

/// DrawTarget writes to the in-memory framebuffer only (no I2C, stays sync).
/// Call flush() or flush_all() afterward to send dirty pages to the display.
impl<I2C: I2c> DrawTarget for Sh1106<I2C> {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels {
            let x = coord.x;
            let y = coord.y;
            if x < 0 || x >= WIDTH as i32 || y < 0 || y >= HEIGHT as i32 {
                continue;
            }
            let x = x as usize;
            let y = y as usize;
            let page = y / 8;
            let bit = y % 8;
            let idx = page * WIDTH as usize + x;

            match color {
                BinaryColor::On => self.buffer[idx] |= 1 << bit,
                BinaryColor::Off => self.buffer[idx] &= !(1 << bit),
            }
            self.dirty |= 1 << page;
        }
        Ok(())
    }
}

impl<I2C: I2c> OriginDimensions for Sh1106<I2C> {
    fn size(&self) -> Size {
        Size::new(WIDTH, HEIGHT)
    }
}
