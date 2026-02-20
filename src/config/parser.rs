//! No-std, zero-alloc INI-like config parser.
//!
//! Parses `[section]`, `key = value`, and `# comments` line-by-line
//! from a `&[u8]` config file into a [`DspConfigFile`].

use super::DspConfigFile;
use crate::drivers::tas5830::{FilterType, Gain923, MixerGains, MixerMode};

/// Parse error with line number context.
#[derive(Debug, defmt::Format)]
pub struct ParseError {
    pub line: u16,
    pub kind: ParseErrorKind,
}

#[derive(Debug, defmt::Format)]
pub enum ParseErrorKind {
    InvalidSection,
    InvalidKeyValue,
    InvalidNumber,
    InvalidFilterType,
    InvalidMixerMode,
    TooManyProcessingPairs,
}

/// Parse a config file from raw bytes into a `DspConfigFile`.
///
/// Returns `Ok(config)` on success, or `Err(error)` with the first parse error.
/// Invalid lines are logged via defmt and skipped where possible.
pub fn parse(data: &[u8]) -> Result<DspConfigFile, ParseError> {
    let mut config = DspConfigFile::default();
    let mut section = Section::None;
    let mut line_num: u16 = 0;

    for line in LineIter::new(data) {
        line_num += 1;
        let line = trim(line);

        // Skip empty lines and comments
        if line.is_empty() || line[0] == b'#' {
            continue;
        }

        // Section header
        if line[0] == b'[' {
            section = parse_section(line);
            if matches!(section, Section::Unknown) {
                defmt::warn!("Config line {}: unknown section", line_num);
            }
            continue;
        }

        // Key = value
        let Some((key, value)) = split_kv(line) else {
            defmt::warn!("Config line {}: invalid key=value", line_num);
            continue;
        };

        match section {
            Section::Device => parse_device(key, value, &mut config),
            Section::Mixer => parse_mixer(key, value, &mut config),
            Section::ChannelVolume => parse_channel_volume(key, value, &mut config),
            Section::Volume => parse_volume(key, value, &mut config),
            Section::Eq => parse_eq_global(key, value, &mut config),
            Section::EqBand(n) => parse_eq_band(n, key, value, &mut config),
            Section::Processing => {
                if let Err(e) = parse_processing(key, value, &mut config) {
                    defmt::warn!("Config line {}: {:?}", line_num, e.kind);
                }
            }
            Section::None | Section::Unknown => {
                defmt::warn!("Config line {}: key outside section", line_num);
            }
        }
    }

    Ok(config)
}

// ── Section parsing ─────────────────────────────────────────────────────────

#[derive(Clone, Copy)]
enum Section {
    None,
    Unknown,
    Device,
    Mixer,
    ChannelVolume,
    Volume,
    Eq,
    EqBand(u8),
    Processing,
}

fn parse_section(line: &[u8]) -> Section {
    // Strip [ and ]
    let end = match line.iter().position(|&b| b == b']') {
        Some(pos) => pos,
        None => return Section::Unknown,
    };
    let inner = &line[1..end];

    // Match known sections
    if eq_bytes(inner, b"device") {
        Section::Device
    } else if eq_bytes(inner, b"mixer") {
        Section::Mixer
    } else if eq_bytes(inner, b"channel_volume") {
        Section::ChannelVolume
    } else if eq_bytes(inner, b"volume") {
        Section::Volume
    } else if eq_bytes(inner, b"eq") {
        Section::Eq
    } else if eq_bytes(inner, b"processing") {
        Section::Processing
    } else if starts_with(inner, b"eq.") {
        // [eq.N] — bands 0-11 = tweeter/left, 12-23 = woofer/right
        let num_bytes = &inner[3..];
        match parse_u8(num_bytes) {
            Some(n) if n < 24 => Section::EqBand(n),
            _ => Section::Unknown,
        }
    } else {
        Section::Unknown
    }
}

// ── Per-section key-value handlers ──────────────────────────────────────────

fn parse_device(key: &[u8], value: &[u8], config: &mut DspConfigFile) {
    if eq_bytes(key, b"sample_rate") {
        if let Some(v) = parse_u32(value) {
            config.sample_rate = v;
        }
    }
}

fn parse_mixer(key: &[u8], value: &[u8], config: &mut DspConfigFile) {
    if eq_bytes(key, b"mode") {
        config.mixer = if eq_bytes(value, b"stereo") {
            MixerMode::Stereo
        } else if eq_bytes(value, b"mono") {
            MixerMode::Mono
        } else if eq_bytes(value, b"left_only") {
            MixerMode::LeftOnly
        } else if eq_bytes(value, b"right_only") {
            MixerMode::RightOnly
        } else {
            defmt::warn!("Unknown mixer mode, defaulting to stereo");
            MixerMode::Stereo
        };
    } else if eq_bytes(key, b"l2l") {
        if let Some(db) = parse_i16(value) {
            config.mixer_gains = Some(config.mixer_gains.unwrap_or(MixerGains::STEREO));
            if let Some(ref mut g) = config.mixer_gains {
                g.l2l = Gain923::from_db(db);
            }
        }
    } else if eq_bytes(key, b"r2l") {
        if let Some(db) = parse_i16(value) {
            config.mixer_gains = Some(config.mixer_gains.unwrap_or(MixerGains::STEREO));
            if let Some(ref mut g) = config.mixer_gains {
                g.r2l = Gain923::from_db(db);
            }
        }
    } else if eq_bytes(key, b"l2r") {
        if let Some(db) = parse_i16(value) {
            config.mixer_gains = Some(config.mixer_gains.unwrap_or(MixerGains::STEREO));
            if let Some(ref mut g) = config.mixer_gains {
                g.l2r = Gain923::from_db(db);
            }
        }
    } else if eq_bytes(key, b"r2r") {
        if let Some(db) = parse_i16(value) {
            config.mixer_gains = Some(config.mixer_gains.unwrap_or(MixerGains::STEREO));
            if let Some(ref mut g) = config.mixer_gains {
                g.r2r = Gain923::from_db(db);
            }
        }
    }
}

fn parse_channel_volume(key: &[u8], value: &[u8], config: &mut DspConfigFile) {
    if eq_bytes(key, b"left_db") || eq_bytes(key, b"tweeter_db") {
        if let Some(db) = parse_i16(value) {
            config.channel_volume_left_db = db;
        }
    } else if eq_bytes(key, b"right_db") || eq_bytes(key, b"woofer_db") {
        if let Some(db) = parse_i16(value) {
            config.channel_volume_right_db = db;
        }
    }
}

fn parse_volume(key: &[u8], value: &[u8], config: &mut DspConfigFile) {
    if eq_bytes(key, b"digital_db") {
        if let Some(db) = parse_i16(value) {
            // 0x00 = 0 dB, each step = -0.5 dB, so register = -2 * dB
            let reg = ((-db) * 2).clamp(0, 255) as u8;
            config.digital_volume = reg;
        }
    } else if eq_bytes(key, b"analog_gain") {
        if let Some(v) = parse_u8(value) {
            config.analog_gain = v.min(31);
        }
    }
}

fn parse_eq_global(key: &[u8], value: &[u8], config: &mut DspConfigFile) {
    if eq_bytes(key, b"bypass") {
        config.eq_bypass = eq_bytes(value, b"true");
    }
}

fn parse_eq_band(band: u8, key: &[u8], value: &[u8], config: &mut DspConfigFile) {
    let b = band as usize;
    let filter = &mut config.eq_bands[b];

    if eq_bytes(key, b"type") {
        *filter = if eq_bytes(value, b"peaking") {
            FilterType::Peaking {
                freq_hz: 1000.0,
                gain_db: 0.0,
                q: 1.41,
            }
        } else if eq_bytes(value, b"low_pass") {
            FilterType::LowPass {
                freq_hz: 1000.0,
                q: 0.707,
            }
        } else if eq_bytes(value, b"high_pass") {
            FilterType::HighPass {
                freq_hz: 1000.0,
                q: 0.707,
            }
        } else if eq_bytes(value, b"low_shelf") {
            FilterType::LowShelf {
                freq_hz: 1000.0,
                gain_db: 0.0,
                q: 0.707,
            }
        } else if eq_bytes(value, b"high_shelf") {
            FilterType::HighShelf {
                freq_hz: 1000.0,
                gain_db: 0.0,
                q: 0.707,
            }
        } else if eq_bytes(value, b"band_pass") {
            FilterType::BandPass {
                freq_hz: 1000.0,
                q: 0.707,
            }
        } else if eq_bytes(value, b"notch") {
            FilterType::Notch {
                freq_hz: 1000.0,
                q: 0.707,
            }
        } else if eq_bytes(value, b"all_pass") {
            FilterType::AllPass {
                freq_hz: 1000.0,
                q: 0.707,
            }
        } else {
            FilterType::Bypass
        };
    } else if eq_bytes(key, b"freq") {
        if let Some(f) = parse_f32(value) {
            set_filter_freq(filter, f);
        }
    } else if eq_bytes(key, b"gain_db") {
        if let Some(g) = parse_f32(value) {
            set_filter_gain(filter, g);
        }
    } else if eq_bytes(key, b"q") {
        if let Some(q) = parse_f32(value) {
            set_filter_q(filter, q);
        }
    }
}

fn parse_processing(
    key: &[u8],
    value: &[u8],
    config: &mut DspConfigFile,
) -> Result<(), ParseError> {
    // Both key and value are hex bytes: reg = val
    let reg = parse_hex_u8(key).ok_or(ParseError {
        line: 0,
        kind: ParseErrorKind::InvalidNumber,
    })?;
    let val = parse_hex_u8(value).ok_or(ParseError {
        line: 0,
        kind: ParseErrorKind::InvalidNumber,
    })?;

    if config.processing.is_full() {
        return Err(ParseError {
            line: 0,
            kind: ParseErrorKind::TooManyProcessingPairs,
        });
    }
    let _ = config.processing.push((reg, val));
    Ok(())
}

// ── Filter field setters ────────────────────────────────────────────────────

fn set_filter_freq(filter: &mut FilterType, f: f32) {
    match filter {
        FilterType::Peaking { freq_hz, .. }
        | FilterType::LowPass { freq_hz, .. }
        | FilterType::HighPass { freq_hz, .. }
        | FilterType::LowShelf { freq_hz, .. }
        | FilterType::HighShelf { freq_hz, .. }
        | FilterType::BandPass { freq_hz, .. }
        | FilterType::Notch { freq_hz, .. }
        | FilterType::AllPass { freq_hz, .. } => *freq_hz = f,
        FilterType::Bypass => {}
    }
}

fn set_filter_gain(filter: &mut FilterType, g: f32) {
    match filter {
        FilterType::Peaking { gain_db, .. }
        | FilterType::LowShelf { gain_db, .. }
        | FilterType::HighShelf { gain_db, .. } => *gain_db = g,
        _ => {}
    }
}

fn set_filter_q(filter: &mut FilterType, q_val: f32) {
    match filter {
        FilterType::Peaking { q, .. }
        | FilterType::LowPass { q, .. }
        | FilterType::HighPass { q, .. }
        | FilterType::LowShelf { q, .. }
        | FilterType::HighShelf { q, .. }
        | FilterType::BandPass { q, .. }
        | FilterType::Notch { q, .. }
        | FilterType::AllPass { q, .. } => *q = q_val,
        FilterType::Bypass => {}
    }
}

// ── Low-level byte parsing utilities ────────────────────────────────────────

/// Line iterator over `&[u8]`, splitting on `\n` (handles `\r\n` too).
struct LineIter<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> LineIter<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }
}

impl<'a> Iterator for LineIter<'a> {
    type Item = &'a [u8];

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos >= self.data.len() {
            return None;
        }

        let start = self.pos;
        while self.pos < self.data.len() && self.data[self.pos] != b'\n' {
            self.pos += 1;
        }

        let mut end = self.pos;
        // Strip trailing \r
        if end > start && self.data[end - 1] == b'\r' {
            end -= 1;
        }

        // Skip the \n
        if self.pos < self.data.len() {
            self.pos += 1;
        }

        Some(&self.data[start..end])
    }
}

/// Trim leading and trailing ASCII whitespace from a byte slice.
fn trim(s: &[u8]) -> &[u8] {
    let start = s.iter().position(|&b| b != b' ' && b != b'\t').unwrap_or(s.len());
    let end = s.iter().rposition(|&b| b != b' ' && b != b'\t').map_or(start, |p| p + 1);
    &s[start..end]
}

/// Split `key = value` (or `key=value`), returning trimmed key and value.
fn split_kv(line: &[u8]) -> Option<(&[u8], &[u8])> {
    let eq_pos = line.iter().position(|&b| b == b'=')?;
    let key = trim(&line[..eq_pos]);
    let value = trim(&line[eq_pos + 1..]);
    if key.is_empty() {
        return None;
    }
    Some((key, value))
}

/// Compare two byte slices for equality (case-sensitive).
fn eq_bytes(a: &[u8], b: &[u8]) -> bool {
    a.len() == b.len() && a.iter().zip(b.iter()).all(|(x, y)| x == y)
}

/// Check if `a` starts with `prefix`.
fn starts_with(a: &[u8], prefix: &[u8]) -> bool {
    a.len() >= prefix.len() && &a[..prefix.len()] == prefix
}

/// Parse an ASCII byte slice as a `u8`.
fn parse_u8(s: &[u8]) -> Option<u8> {
    parse_u32(s).and_then(|v| u8::try_from(v).ok())
}

/// Parse an ASCII byte slice as a `u32`.
fn parse_u32(s: &[u8]) -> Option<u32> {
    if s.is_empty() {
        return None;
    }
    let mut result: u32 = 0;
    for &b in s {
        if b < b'0' || b > b'9' {
            return None;
        }
        result = result.checked_mul(10)?.checked_add((b - b'0') as u32)?;
    }
    Some(result)
}

/// Parse an ASCII byte slice as an `i16` (supports negative values and decimals).
/// Truncates decimal part (e.g., "-24.5" → -24).
fn parse_i16(s: &[u8]) -> Option<i16> {
    if s.is_empty() {
        return None;
    }
    let (neg, start) = if s[0] == b'-' {
        (true, 1)
    } else {
        (false, 0)
    };

    let mut result: i32 = 0;
    for &b in &s[start..] {
        if b == b'.' {
            break; // Truncate decimal
        }
        if b < b'0' || b > b'9' {
            return None;
        }
        result = result * 10 + (b - b'0') as i32;
    }

    if neg {
        result = -result;
    }
    i16::try_from(result).ok()
}

/// Parse a simple floating-point number from ASCII bytes.
///
/// Supports: `123`, `1.5`, `-3.14`, `0.707`
/// Does NOT support scientific notation.
fn parse_f32(s: &[u8]) -> Option<f32> {
    if s.is_empty() {
        return None;
    }

    let (neg, start) = if s[0] == b'-' {
        (true, 1)
    } else {
        (false, 0)
    };

    let mut integer_part: u32 = 0;
    let mut frac_part: u32 = 0;
    let mut frac_divisor: u32 = 1;
    let mut in_frac = false;

    for &b in &s[start..] {
        if b == b'.' {
            if in_frac {
                return None; // Double decimal
            }
            in_frac = true;
            continue;
        }
        if b < b'0' || b > b'9' {
            return None;
        }
        if in_frac {
            frac_part = frac_part * 10 + (b - b'0') as u32;
            frac_divisor *= 10;
        } else {
            integer_part = integer_part * 10 + (b - b'0') as u32;
        }
    }

    let mut result = integer_part as f32 + frac_part as f32 / frac_divisor as f32;
    if neg {
        result = -result;
    }
    Some(result)
}

/// Parse a hex byte like `0x2A` or `2A` or `2a`.
fn parse_hex_u8(s: &[u8]) -> Option<u8> {
    let s = if s.len() >= 2 && s[0] == b'0' && (s[1] == b'x' || s[1] == b'X') {
        &s[2..]
    } else {
        s
    };

    if s.is_empty() || s.len() > 2 {
        return None;
    }

    let mut result: u8 = 0;
    for &b in s {
        let digit = match b {
            b'0'..=b'9' => b - b'0',
            b'a'..=b'f' => b - b'a' + 10,
            b'A'..=b'F' => b - b'A' + 10,
            _ => return None,
        };
        result = result * 16 + digit;
    }
    Some(result)
}
