//! Table-driven decoding shared by every ECU converter.
//!
//! Adding a data source means writing tables, not code:
//!
//! ```ignore
//! // A CAN-broadcast ECU: one `Field` per channel, grouped by frame.
//! pub static SOURCE: CanSource = CanSource {
//!     name: "MyEcu",
//!     default_base: 0x600,
//!     frames: &[
//!         CanFrameMap { id_offset: 0, fields: &[
//!             field!(0, U16BE, RPM, raw),
//!             field!(2, U16BE, MAP, x10),
//!         ]},
//!     ],
//! };
//! // then, per received frame:  SOURCE.feed(base, &frame)
//! ```
//!
//! A serial ECU uses the same `Field` table with `apply_fields`; see `speeduino`.

use crate::gauge::GaugeData;
use embedded_hal_0_2::can::{Frame, Id};
use mcp2515::frame::CanFrame;

/// How a raw channel is packed in a byte buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Raw {
    U8,
    S8,
    /// Little-endian 16-bit.
    U16,
    S16,
    /// Big-endian 16-bit (Haltech, Megasquirt).
    U16BE,
    S16BE,
    U32,
    U32BE,
}

impl Raw {
    /// How many bytes this width occupies on the wire.
    // No `is_empty` beside it: a width is one, two or four bytes and never
    // none, so the method clippy asks for could only ever return false.
    #[allow(clippy::len_without_is_empty)]
    pub const fn len(self) -> usize {
        match self {
            Raw::U8 | Raw::S8 => 1,
            Raw::U16 | Raw::S16 | Raw::U16BE | Raw::S16BE => 2,
            Raw::U32 | Raw::U32BE => 4,
        }
    }

    /// Read the channel at `off`, or `None` if the buffer is too short.
    pub fn read(self, data: &[u8], off: usize) -> Option<i32> {
        let b = data.get(off..off + self.len())?;
        Some(match self {
            Raw::U8 => b[0] as i32,
            Raw::S8 => b[0] as i8 as i32,
            Raw::U16 => u16::from_le_bytes([b[0], b[1]]) as i32,
            Raw::S16 => i16::from_le_bytes([b[0], b[1]]) as i32,
            Raw::U16BE => u16::from_be_bytes([b[0], b[1]]) as i32,
            Raw::S16BE => i16::from_be_bytes([b[0], b[1]]) as i32,
            Raw::U32 => u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as i32,
            Raw::U32BE => u32::from_be_bytes([b[0], b[1], b[2], b[3]]) as i32,
        })
    }
}

/// One channel: where it sits, how it is packed, which gauge it feeds, and
/// the conversion from raw counts to that gauge's canonical fixed point.
pub struct Field {
    pub off: usize,
    pub raw: Raw,
    pub gauge: &'static GaugeData,
    pub conv: fn(i32) -> i32,
}

/// Build a `Field` from `(offset, Raw variant, gauge static, conversion fn)`.
#[macro_export]
macro_rules! field {
    ($off:literal, $raw:ident, $gauge:ident, $conv:expr) => {
        $crate::decode::Field {
            off: $off,
            raw: $crate::decode::Raw::$raw,
            gauge: &$crate::$gauge,
            conv: $conv,
        }
    };
}

/// Write every field present in `data` into its gauge. Fields that fall past
/// the end of `data` are skipped. Returns how many gauges were written.
pub fn apply_fields(fields: &[Field], data: &[u8]) -> usize {
    let mut written = 0;
    for f in fields {
        if let Some(v) = f.raw.read(data, f.off) {
            f.gauge.set((f.conv)(v));
            written += 1;
        }
    }
    written
}

/// The channels carried by one CAN frame of a broadcast protocol.
pub struct CanFrameMap {
    /// Frame ID minus the protocol's base ID.
    pub id_offset: u16,
    pub fields: &'static [Field],
}

/// A CAN-broadcast ECU protocol.
pub struct CanSource {
    pub name: &'static str,
    /// Base ID the ECU ships with; pass a different one to `feed` if reconfigured.
    pub default_base: u16,
    pub frames: &'static [CanFrameMap],
}

impl CanSource {
    /// Decode `frame` if it belongs to this protocol at `base`.
    /// Returns the number of gauges written, or `None` if the ID is not ours.
    pub fn feed(&self, base: u16, frame: &CanFrame) -> Option<usize> {
        let id = match frame.id() {
            Id::Standard(s) => s.as_raw(),
            Id::Extended(e) => u16::try_from(e.as_raw()).ok()?,
        };
        let offset = id.checked_sub(base)?;
        let map = self.frames.iter().find(|m| m.id_offset == offset)?;
        Some(apply_fields(map.fields, &frame.data()[..frame.dlc()]))
    }

    /// Decode with the protocol's default base ID.
    pub fn feed_default(&self, frame: &CanFrame) -> Option<usize> {
        self.feed(self.default_base, frame)
    }

    /// Every CAN ID this protocol uses at `base`.
    pub fn ids(&self, base: u16) -> impl Iterator<Item = u16> + '_ {
        self.frames.iter().map(move |m| base + m.id_offset)
    }
}

/// Raw-to-canonical conversions shared by the converters. Each is named for
/// the raw scaling it undoes; the target scale is the gauge's `Unit::scale`.
pub mod conv {
    /// Raw already matches the canonical scale.
    pub fn raw(v: i32) -> i32 {
        v
    }
    /// Whole units into a tenths unit (%, °, kPa, km/h).
    pub fn x10(v: i32) -> i32 {
        v * 10
    }
    /// Whole units into a hundredths unit.
    pub fn x100(v: i32) -> i32 {
        v * 100
    }
    /// Whole units into a thousandths unit (V into mV, ms into µs).
    pub fn x1000(v: i32) -> i32 {
        v * 1000
    }
    /// 0.5-unit steps into a tenths unit.
    pub fn half(v: i32) -> i32 {
        v * 5
    }
    /// 2-unit steps into a tenths unit.
    pub fn x20(v: i32) -> i32 {
        v * 20
    }
    /// 5-unit steps into a tenths unit (rusEFI EGT).
    pub fn x50(v: i32) -> i32 {
        v * 50
    }
    /// Raw in hundredths into a thousandths unit (0.01 V into mV, 0.01 ms into µs).
    pub fn hundredths_to_thousandths(v: i32) -> i32 {
        v * 10
    }
    /// Raw in tenths into a hundredths unit (AFR sent as AFR*10).
    pub fn tenths_to_hundredths(v: i32) -> i32 {
        v * 10
    }
    /// Raw in tenths into a thousandths unit (0.1 V into mV, 0.1 ms into µs).
    pub fn tenths_to_thousandths(v: i32) -> i32 {
        v * 100
    }
    /// Raw in hundredths into a tenths unit (rounded).
    pub fn hundredths_to_tenths(v: i32) -> i32 {
        div_round(v, 10)
    }
    /// Raw in thousandths into a hundredths unit (rounded).
    pub fn thousandths_to_hundredths(v: i32) -> i32 {
        div_round(v, 10)
    }
    /// Whole °C sent with a +40 offset, into tenths of °C.
    pub fn celsius_plus40(v: i32) -> i32 {
        (v - 40) * 10
    }
    /// Kelvin in tenths into tenths of °C.
    pub fn kelvin_tenths(v: i32) -> i32 {
        div_round(v * 10 - 27_315, 10)
    }
    /// Fahrenheit in tenths into tenths of °C.
    pub fn fahrenheit_tenths(v: i32) -> i32 {
        div_round((v - 320) * 5, 9)
    }
    /// Whole percent centred on 100 (Speeduino corrections) into tenths of trim, 0-centred.
    pub fn centred100_to_trim_tenths(v: i32) -> i32 {
        (v - 100) * 10
    }
    /// Tenths of percent centred on 100 (Megasquirt) into tenths of trim, 0-centred.
    pub fn centred1000_to_trim_tenths(v: i32) -> i32 {
        v - 1000
    }
    /// Absolute pressure in tenths of kPa into gauge pressure (Haltech).
    pub fn kpa_abs_tenths_to_gauge(v: i32) -> i32 {
        v - 1013
    }
    /// Metres per second in tenths into tenths of km/h (Megasquirt VSS1).
    pub fn mps_tenths_to_kmh_tenths(v: i32) -> i32 {
        div_round(v * 36, 10)
    }
    /// Whole psi into tenths of kPa.
    pub fn psi(v: i32) -> i32 {
        div_round(v * 6895, 100)
    }
    /// psi in tenths into tenths of kPa.
    pub fn psi_tenths(v: i32) -> i32 {
        div_round(v * 6895, 1000)
    }
    /// Lambda in thousandths into hundredths of AFR (gasoline stoich 14.7).
    pub fn lambda_thousandths(v: i32) -> i32 {
        div_round(v * 147, 100)
    }
    /// Lambda in hundredths into hundredths of AFR.
    pub fn lambda_hundredths(v: i32) -> i32 {
        div_round(v * 147, 10)
    }
    /// Lambda in ten-thousandths into hundredths of AFR.
    pub fn lambda_ten_thousandths(v: i32) -> i32 {
        div_round(v * 147, 1000)
    }

    /// Integer division rounding to nearest, away from zero on ties.
    pub fn div_round(v: i32, d: i32) -> i32 {
        let half = d / 2;
        if v >= 0 {
            (v + half) / d
        } else {
            (v - half) / d
        }
    }
}
