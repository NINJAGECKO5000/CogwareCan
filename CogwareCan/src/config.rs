// SPDX-License-Identifier: GPL-3.0-only
//! Settings a display holds, described well enough for an editor to render
//! them without knowing what any of them mean.
//!
//! Each `Setting` carries its range, its default, its unit and a line of
//! documentation, so `CogwareCopilot` builds an editor from `ALL_SETTINGS`
//! rather than from a hardcoded form. Values are fixed point in the setting's
//! unit, exactly as gauges are: `WARN_COOLANT` is 1050 for 105 °C because
//! `Unit::CELSIUS.scale == 10`.
//!
//! A whole table travels as one self-describing blob, which is what `xfer`
//! carries as `kind::CONFIG`:
//!
//! ```ignore
//! let n = config::encode(&mut buf)?;              // editor, or a node saving
//! let applied = config::decode(&buf[..n])?;       // node receiving one
//! config::describe(&mut json)?;                   // editor, to build the form
//! ```
//!
//! Keys travel as text rather than as an index or a hash, so a blob written
//! by a newer editor still applies on an older node: a key this build does
//! not have is skipped, not an error.

use crate::units::Unit;
use core::cell::Cell;
use core::fmt::Write;
use critical_section::Mutex;

/// Magic at the head of a config blob.
pub const MAGIC: [u8; 4] = *b"CFG1";
/// Blob layout version, bumped only if the record framing changes.
pub const VERSION: u8 = 1;
/// Bytes before the first record.
pub const HEADER_LEN: usize = 8;
/// Longest key the record framing can carry.
pub const KEY_MAX: usize = 31;

/// How an editor should present a setting.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// 0 or 1, drawn as a switch.
    Toggle,
    /// A number between `min` and `max`, in the setting's unit.
    Number,
    /// An index into `choices`.
    Choice,
}

impl Kind {
    /// The name an editor matches on, lowercase.
    pub const fn name(self) -> &'static str {
        match self {
            Kind::Toggle => "toggle",
            Kind::Number => "number",
            Kind::Choice => "choice",
        }
    }
}

/// Why a blob could not be applied.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfigError {
    /// The first four bytes are not `MAGIC`.
    BadMagic,
    /// The blob was written to a layout this build does not read.
    BadVersion(u8),
    /// A record runs past the end of the blob.
    Truncated,
    /// The buffer handed to `encode` is smaller than `blob_len`.
    TooSmall { needed: usize, got: usize },
}

/// What `decode` did with a blob.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Applied {
    /// Records that named a setting this build has.
    pub applied: usize,
    /// Records naming a setting this build does not have.
    pub skipped: usize,
    /// Records whose value fell outside the setting's range and was clamped.
    pub clamped: usize,
}

/// One editable setting. The value is live: reading it is what the firmware
/// does every frame, and an editor writing it takes effect immediately.
pub struct Setting {
    /// Stable name, the one that travels in a blob.
    pub key: &'static str,
    /// How an editor should draw it.
    pub kind: Kind,
    /// Unit the fixed-point value is in; `Unit::RAW` for a plain count.
    pub unit: Unit,
    /// Lowest accepted value, inclusive.
    pub min: i32,
    /// Highest accepted value, inclusive.
    pub max: i32,
    /// Value a fresh node starts at.
    pub default: i32,
    /// Labels for a `Choice`, indexed by value; empty for every other kind.
    pub choices: &'static [&'static str],
    /// One line for whoever is deciding what to set it to.
    pub doc: &'static str,
    value: Mutex<Cell<i32>>,
}

impl Setting {
    /// Build a table entry. The value starts at `default`, so a setting is
    /// never unset the way a gauge can be.
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        key: &'static str,
        kind: Kind,
        unit: Unit,
        min: i32,
        max: i32,
        default: i32,
        choices: &'static [&'static str],
        doc: &'static str,
    ) -> Self {
        Setting {
            key,
            kind,
            unit,
            min,
            max,
            default,
            choices,
            doc,
            value: Mutex::new(Cell::new(default)),
        }
    }

    /// The fixed-point value as it stands.
    pub fn get(&self) -> i32 {
        critical_section::with(|cs| self.value.borrow(cs).get())
    }

    /// The value in physical units, for a display that wants a float.
    pub fn as_f32(&self) -> f32 {
        self.get() as f32 / self.unit.scale as f32
    }

    /// True if this setting is a `Toggle` that is on.
    pub fn is_on(&self) -> bool {
        self.get() != 0
    }

    /// The label of the selected choice, or `None` unless this is a `Choice`.
    pub fn choice(&self) -> Option<&'static str> {
        self.choices.get(self.get().max(0) as usize).copied()
    }

    /// Store `value`, clamped into range. Returns true if it was clamped, so
    /// a caller can tell an editor its number did not survive intact.
    pub fn set(&self, value: i32) -> bool {
        let clamped = value.clamp(self.min, self.max);
        critical_section::with(|cs| self.value.borrow(cs).set(clamped));
        clamped != value
    }

    /// Put it back to `default`.
    pub fn reset(&self) {
        critical_section::with(|cs| self.value.borrow(cs).set(self.default));
    }

    /// True if it has not been moved off its default.
    pub fn is_default(&self) -> bool {
        self.get() == self.default
    }
}

impl core::fmt::Debug for Setting {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{} = {}{}", self.key, self.get(), self.unit.symbol)
    }
}

/// Generates the `static` for each setting and `ALL_SETTINGS`, from one row
/// per setting.
macro_rules! settings {
    ($(
        $name:ident = $key:literal : $kind:ident, $unit:ident,
        $min:literal ..= $max:literal, $default:literal
        $(, [$($choice:literal),+ $(,)?])? ;
        $doc:literal
    )+) => {
        $(
            #[doc = $doc]
            pub static $name: Setting = Setting::new(
                $key, Kind::$kind, Unit::$unit, $min, $max, $default,
                &[$($($choice),+)?], $doc,
            );
            // A key travels as a length-prefixed byte string.
            const _: () = assert!($key.len() <= KEY_MAX, "setting key is too long");
            const _: () = assert!($min <= $default && $default <= $max, "default is out of range");
        )+

        /// Every setting in table order, which is the order an editor lists them.
        pub static ALL_SETTINGS: &[&'static Setting] = &[$(&$name),+];
    };
}

// Ranges are fixed point in the row's unit: a PERCENT row runs 0..=1000 for
// 0..=100 %, because `Unit::PERCENT.scale` is 10.
settings! {
    BRIGHTNESS = "brightness": Number, PERCENT, 0..=1000, 800;
        "Backlight level."
    SCREEN_ROTATION = "screen_rotation": Choice, RAW, 0..=3, 0, ["0", "90", "180", "270"];
        "Quarter turns clockwise from the panel's native orientation."
    STARTUP_MODE = "startup_mode": Number, RAW, 0..=255, 0;
        "Mode this node assumes at power-on until the master says otherwise."
    UNITS_TEMP = "units_temp": Choice, RAW, 0..=2, 0, ["Celsius", "Fahrenheit", "Kelvin"];
        "Temperature unit the display renders in."
    UNITS_PRESSURE = "units_pressure": Choice, RAW, 0..=3, 0, ["kPa", "psi", "bar", "inHg"];
        "Pressure unit the display renders in."
    UNITS_SPEED = "units_speed": Choice, RAW, 0..=1, 0, ["km/h", "mph"];
        "Speed unit the display renders in."
    UNITS_DISTANCE = "units_distance": Choice, RAW, 0..=1, 0, ["km", "miles"];
        "Distance unit the odometer renders in."
    SHIFT_LIGHT_RPM = "shift_light_rpm": Number, RPM, 0..=12000, 6500;
        "Engine speed that lights SHIFT_LIGHT; 0 turns the lamp off entirely."
    REV_LIMIT_RPM = "rev_limit_rpm": Number, RPM, 0..=12000, 7000;
        "Engine speed that lights REV_LIMIT."
    WARN_COOLANT = "warn_coolant": Number, CELSIUS, 0..=1500, 1050;
        "Coolant temperature that lights COOLANT_TEMP."
    WARN_OIL_PRESSURE = "warn_oil_pressure": Number, KPA, 0..=10000, 1000;
        "Oil pressure below which OIL_PRESSURE lights."
    WARN_OIL_TEMP = "warn_oil_temp": Number, CELSIUS, 0..=2000, 1300;
        "Oil temperature that lights OVERHEAT."
    WARN_BATTERY = "warn_battery": Number, VOLT, 0..=20000, 12000;
        "Battery voltage below which BATTERY lights."
    LOW_FUEL = "low_fuel": Number, PERCENT, 0..=1000, 150;
        "Fuel level below which LOW_FUEL lights."
    BUS_TIMEOUT = "bus_timeout": Number, MS, 0..=10000000, 2000000;
        "Silence from the master after which the display treats the bus as down."
    SUBSCRIBE_RETRY = "subscribe_retry": Number, MS, 0..=10000000, 250000;
        "Wait between subscribe attempts while gauges are still unacknowledged."
    DEMO_MODE = "demo_mode": Toggle, RAW, 0..=1, 0;
        "Sweep every gauge through its range instead of reading the bus."
}

/// The setting called `key`, or `None` if this build has no such setting.
pub fn by_key(key: &str) -> Option<&'static Setting> {
    ALL_SETTINGS.iter().copied().find(|s| s.key == key)
}

/// Put every setting back to its default.
pub fn reset_all() {
    for s in ALL_SETTINGS {
        s.reset();
    }
}

/// How many settings are off their defaults.
pub fn changed_count() -> usize {
    ALL_SETTINGS.iter().filter(|s| !s.is_default()).count()
}

/// Exactly how many bytes `encode` will write.
pub fn blob_len() -> usize {
    HEADER_LEN + ALL_SETTINGS.iter().map(|s| 5 + s.key.len()).sum::<usize>()
}

/// Write every setting into `out` as a config blob, returning its length.
pub fn encode(out: &mut [u8]) -> Result<usize, ConfigError> {
    let needed = blob_len();
    if out.len() < needed {
        return Err(ConfigError::TooSmall { needed, got: out.len() });
    }
    out[..4].copy_from_slice(&MAGIC);
    out[4] = VERSION;
    out[5] = 0;
    out[6..8].copy_from_slice(&(ALL_SETTINGS.len() as u16).to_le_bytes());

    let mut at = HEADER_LEN;
    for s in ALL_SETTINGS {
        let key = s.key.as_bytes();
        out[at] = key.len() as u8;
        out[at + 1..at + 1 + key.len()].copy_from_slice(key);
        at += 1 + key.len();
        out[at..at + 4].copy_from_slice(&s.get().to_le_bytes());
        at += 4;
    }
    Ok(at)
}

/// Apply a config blob. A key this build does not have is skipped rather than
/// failing the whole blob, so an older node still takes what it understands
/// from a newer editor.
pub fn decode(blob: &[u8]) -> Result<Applied, ConfigError> {
    if blob.len() < HEADER_LEN || blob[..4] != MAGIC {
        return Err(ConfigError::BadMagic);
    }
    if blob[4] != VERSION {
        return Err(ConfigError::BadVersion(blob[4]));
    }
    let count = u16::from_le_bytes([blob[6], blob[7]]) as usize;

    let mut out = Applied::default();
    let mut at = HEADER_LEN;
    for _ in 0..count {
        let key_len = *blob.get(at).ok_or(ConfigError::Truncated)? as usize;
        at += 1;
        let end = at + key_len;
        if end + 4 > blob.len() {
            return Err(ConfigError::Truncated);
        }
        let value = i32::from_le_bytes([blob[end], blob[end + 1], blob[end + 2], blob[end + 3]]);
        // A key that is not valid UTF-8 cannot name a setting, so it is
        // skipped exactly as an unknown one is.
        match core::str::from_utf8(&blob[at..end]).ok().and_then(by_key) {
            Some(s) => {
                out.applied += 1;
                if s.set(value) {
                    out.clamped += 1;
                }
            }
            None => out.skipped += 1,
        }
        at = end + 4;
    }
    Ok(out)
}

/// Write a JSON description of every setting, for an editor to build a form
/// from. Uses `core::fmt::Write`, so it costs no dependency and works on a
/// node as well as in a tool.
pub fn describe<W: Write>(w: &mut W) -> core::fmt::Result {
    w.write_char('[')?;
    for (i, s) in ALL_SETTINGS.iter().enumerate() {
        if i > 0 {
            w.write_char(',')?;
        }
        write!(w, r#"{{"key":"{}","kind":"{}","unit":""#, s.key, s.kind.name())?;
        escape(w, s.unit.symbol)?;
        write!(
            w,
            r#"","scale":{},"min":{},"max":{},"default":{},"value":{},"choices":["#,
            s.unit.scale, s.min, s.max, s.default, s.get()
        )?;
        for (j, c) in s.choices.iter().enumerate() {
            if j > 0 {
                w.write_char(',')?;
            }
            w.write_char('"')?;
            escape(w, c)?;
            w.write_char('"')?;
        }
        w.write_str(r#"],"doc":""#)?;
        escape(w, s.doc)?;
        w.write_str(r#""}"#)?;
    }
    w.write_char(']')
}

/// Write `s` with the two characters JSON cannot carry raw escaped. Control
/// characters cannot appear: every string here is a Rust literal from the
/// table above.
fn escape<W: Write>(w: &mut W, s: &str) -> core::fmt::Result {
    for c in s.chars() {
        match c {
            '"' => w.write_str("\\\"")?,
            '\\' => w.write_str("\\\\")?,
            _ => w.write_char(c)?,
        }
    }
    Ok(())
}
