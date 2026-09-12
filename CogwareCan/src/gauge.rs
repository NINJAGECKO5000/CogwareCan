// SPDX-License-Identifier: GPL-3.0-only
//! The gauge specification: every value the displays can show, its CAN ID,
//! its canonical unit, and how it travels on the wire.
//!
//! This table is the single source of truth. Converters (see `speeduino`)
//! normalise whatever an ECU sends into the canonical unit and store it here;
//! displays read from here and never need to know which ECU produced it.
//!
//! Stored values are fixed point: `physical value * unit.scale`. A coolant
//! reading of 87.5 °C is stored as 875 because `Unit::CELSIUS.scale == 10`.
//! Use `GaugeData::as_f32` when a float is wanted for display.

use crate::units::{Reading, Unit};
use core::cell::Cell;
use critical_section::Mutex;
use embedded_hal_0_2::can::{Frame, Id, StandardId};
use mcp2515::frame::CanFrame;
use paste::paste;

/// How a value is packed into a CAN frame: little endian, exactly this many bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wire {
    U8,
    I8,
    U16,
    I16,
    I32,
}

impl Wire {
    /// How many bytes this width occupies in a frame's payload.
    // No `is_empty` beside it: a width is one, two or four bytes and never
    // none, so the method clippy asks for could only ever return false.
    #[allow(clippy::len_without_is_empty)]
    pub const fn len(self) -> usize {
        match self {
            Wire::U8 | Wire::I8 => 1,
            Wire::U16 | Wire::I16 => 2,
            Wire::I32 => 4,
        }
    }

    /// Pack `value` into `out` and return how many bytes were used.
    /// Values outside the wire range are clamped rather than wrapped.
    pub fn encode(self, value: i32, out: &mut [u8; 4]) -> usize {
        let n = self.len();
        let clamped = match self {
            Wire::U8 => value.clamp(0, u8::MAX as i32),
            Wire::I8 => value.clamp(i8::MIN as i32, i8::MAX as i32),
            Wire::U16 => value.clamp(0, u16::MAX as i32),
            Wire::I16 => value.clamp(i16::MIN as i32, i16::MAX as i32),
            Wire::I32 => value,
        };
        out[..n].copy_from_slice(&clamped.to_le_bytes()[..n]);
        n
    }

    /// Unpack a payload. Returns `None` unless `data.len()` matches exactly.
    pub fn decode(self, data: &[u8]) -> Option<i32> {
        if data.len() != self.len() {
            return None;
        }
        Some(match self {
            Wire::U8 => data[0] as i32,
            Wire::I8 => data[0] as i8 as i32,
            Wire::U16 => u16::from_le_bytes([data[0], data[1]]) as i32,
            Wire::I16 => i16::from_le_bytes([data[0], data[1]]) as i32,
            Wire::I32 => i32::from_le_bytes([data[0], data[1], data[2], data[3]]),
        })
    }
}

/// Which kind of source can populate a gauge. A display should only rely on
/// a gauge being present if the attached source is at least this specific.
/// Ordered, so `gauge.source <= Source::Standalone` is a valid check.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Source {
    /// Available from a stock car over OBD2 mode 01 as well as every standalone.
    Common,
    /// Standalone ECUs (Speeduino, Megasquirt, rusEFI, Haltech, MaxxECU,
    /// ECUMaster, ...) broadcast it; OBD2 has no PID for it.
    Standalone,
    /// Speeduino-only: status bitfields, firmware diagnostics, and the
    /// individual fuel-correction terms other ECUs only send summed.
    Speeduino,
    /// Bus housekeeping, produced by the server itself.
    Protocol,
}

pub struct GaugeData {
    pub name: &'static str,
    pub id: u16,
    pub wire: Wire,
    pub unit: Unit,
    pub source: Source,
    value: Mutex<Cell<Option<i32>>>,
}

impl core::fmt::Debug for GaugeData {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}(0x{:03X}) = {:?}{}", self.name, self.id, self.get(), self.unit.symbol)
    }
}

/// Two gauges are equal only if they are the same table entry.
impl PartialEq for GaugeData {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self, other)
    }
}
impl Eq for GaugeData {}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// The payload length does not match the gauge's wire width.
    BadLength { expected: usize, got: usize },
}

impl GaugeData {
    pub const fn new(name: &'static str, id: u16, wire: Wire, unit: Unit, source: Source) -> Self {
        GaugeData {
            name,
            id,
            wire,
            unit,
            source,
            value: Mutex::new(Cell::new(None)),
        }
    }

    /// Fixed-point value, or `None` if nothing has written this gauge yet.
    pub fn get(&self) -> Option<i32> {
        critical_section::with(|cs| self.value.borrow(cs).get())
    }

    pub fn get_or(&self, default: i32) -> i32 {
        self.get().unwrap_or(default)
    }

    /// Value in physical units (counts divided by `unit.scale`).
    pub fn as_f32(&self) -> Option<f32> {
        self.reading().map(|r| r.value())
    }

    /// Value with its unit attached, for conversion: `MAP.reading()?.psi()`.
    pub fn reading(&self) -> Option<Reading> {
        self.get().map(|counts| Reading { counts, unit: self.unit })
    }

    /// Value converted to `target`: `MAP.to(Unit::PSI)`.
    /// `None` if unset or if `target` is a different physical dimension.
    pub fn to(&self, target: Unit) -> Option<f32> {
        self.reading()?.to(target)
    }

    pub fn set(&self, value: i32) {
        critical_section::with(|cs| self.value.borrow(cs).set(Some(value)));
    }

    pub fn clear(&self) {
        critical_section::with(|cs| self.value.borrow(cs).set(None));
    }

    pub fn is_set(&self) -> bool {
        self.get().is_some()
    }

    /// Frame carrying this gauge, or `None` if it has never been set.
    pub fn to_frame(&self) -> Option<CanFrame> {
        let value = self.get()?;
        let mut buf = [0u8; 4];
        let n = self.wire.encode(value, &mut buf);
        CanFrame::new(Id::Standard(StandardId::new(self.id)?), &buf[..n])
    }

    /// Store the payload of a frame addressed to this gauge.
    pub fn set_from_frame(&self, frame: &CanFrame) -> Result<i32, FrameError> {
        let data = &frame.data()[..frame.dlc()];
        let value = self.wire.decode(data).ok_or(FrameError::BadLength {
            expected: self.wire.len(),
            got: data.len(),
        })?;
        self.set(value);
        Ok(value)
    }
}

/// Generates, from one row per gauge: the `static` for each gauge, the
/// `Gauge` enum with matching discriminants, ID lookup, and `ALL_GAUGES`.
macro_rules! gauges {
    ($($name:ident = $id:literal : $wire:ident, $unit:ident, $source:ident;)+) => {
        $(
            pub static $name: GaugeData =
                GaugeData::new(stringify!($name), $id, Wire::$wire, Unit::$unit, Source::$source);
            // The client request protocol carries gauge IDs as single bytes.
            const _: () = assert!($id <= 0xFF, "gauge ID must fit in one request byte");
        )+

        /// Every gauge in table order.
        pub static ALL_GAUGES: &[&'static GaugeData] = &[$(&$name),+];

        paste! {
            #[repr(u16)]
            #[derive(Clone, Copy, Debug, PartialEq, Eq)]
            pub enum Gauge {
                $([<$name:camel>] = $id,)+
            }

            impl Gauge {
                pub const fn data(self) -> &'static GaugeData {
                    match self {
                        $(Gauge::[<$name:camel>] => &$name,)+
                    }
                }

                pub const fn id(self) -> u16 {
                    self as u16
                }

                pub const fn from_id(id: u16) -> Option<Gauge> {
                    match id {
                        $($id => Some(Gauge::[<$name:camel>]),)+
                        _ => None,
                    }
                }
            }
        }
    };
}

/// Look up a gauge by its CAN ID.
pub fn gauge_by_id(id: u16) -> Option<&'static GaugeData> {
    Gauge::from_id(id).map(Gauge::data)
}

// CAN ID map. 0x000..=0x01F is reserved for protocol frames (see `protocol`).
// Lower IDs win bus arbitration, so more urgent gauges belong at lower IDs.
gauges! {
    STA_TIME            = 0x20: U8,  SECONDS,       Common;
    STA_STATUS1         = 0x21: U8,  RAW,           Speeduino;
    STA_ENG             = 0x22: U8,  RAW,           Speeduino;
    DWELL               = 0x23: U16, MS,            Standalone;
    MAP                 = 0x24: U16, KPA,           Common;
    IAT                 = 0x25: I16, CELSIUS,       Common;
    CLNT                = 0x26: I16, CELSIUS,       Common;
    BAT_CORRECT         = 0x27: U16, PERCENT,       Speeduino;
    BAT_VOL             = 0x28: U16, VOLT,          Common;
    AFR_PRI             = 0x29: U16, AFR,           Common;
    // Closed-loop fuel trim, 0 = no correction (Speeduino's 100-centred value is shifted).
    EGO_CORRECT         = 0x2A: I16, PERCENT,       Common;
    IAT_CORRECT         = 0x2B: U16, PERCENT,       Speeduino;
    WUE_CORRECT         = 0x2C: U16, PERCENT,       Speeduino;
    RPM                 = 0x2D: U16, RPM,           Common;
    ACCEL_ENRICH        = 0x2E: U16, PERCENT,       Speeduino;
    GAMME_E             = 0x2F: U16, PERCENT,       Speeduino;
    VE                  = 0x30: U16, PERCENT,       Standalone;
    AFR_TARGET          = 0x31: U16, AFR,           Common;
    PULSE_WIDTH1        = 0x32: U16, MS,            Standalone;
    TPS_DOT             = 0x33: I16, PERCENT_PER_S, Standalone;
    CUR_SPARK_ADVANCE   = 0x34: I16, DEGREES,       Common;
    TPS                 = 0x35: U16, PERCENT,       Common;
    LOOP_PS             = 0x36: U16, HZ,            Speeduino;
    FREE_MEM            = 0x37: U16, BYTES,         Speeduino;
    BOOST_TARGET        = 0x38: U16, KPA,           Standalone;
    BOOST_PWM           = 0x39: U16, PERCENT,       Standalone;
    STA_SPARK           = 0x3A: U8,  RAW,           Speeduino;
    RPM_DOT             = 0x3B: I16, RPM_PER_S,     Standalone;
    ETHANOL_PERCENT     = 0x3C: U16, PERCENT,       Common;
    FLEX_CORRECT        = 0x3D: U16, PERCENT,       Speeduino;
    FLEX_IGN_CORRECT    = 0x3E: I16, DEGREES,       Speeduino;
    IDLE_LOAD           = 0x3F: U8,  RAW,           Standalone;
    TEST_OUTPUTS        = 0x40: U8,  RAW,           Speeduino;
    AFR_SEC             = 0x41: U16, AFR,           Standalone;
    BARO                = 0x42: U16, KPA,           Common;
    TPS_ADC             = 0x43: U8,  RAW,           Speeduino;
    NEXT_ERROR          = 0x44: U8,  RAW,           Common;
    STA_LAUNCH_CORRECT  = 0x45: U16, PERCENT,       Standalone;
    PULSE_WIDTH2        = 0x46: U16, MS,            Standalone;
    PULSE_WIDTH3        = 0x47: U16, MS,            Standalone;
    PULSE_WIDTH4        = 0x48: U16, MS,            Standalone;
    STA_STATUS2         = 0x49: U8,  RAW,           Speeduino;
    ENG_PROTECT_STA     = 0x4A: U8,  RAW,           Speeduino;
    FUEL_LOAD           = 0x4B: I16, RAW,           Common;
    IGN_LOAD            = 0x4C: I16, RAW,           Common;
    INJ_ANGLE           = 0x4D: U16, DEGREES,       Standalone;
    IDLE_DUTY           = 0x4E: U16, PERCENT,       Standalone;
    CL_IDLE_TARGET      = 0x4F: U16, RPM,           Standalone;
    MAP_DOT             = 0x50: I16, KPA_PER_S,     Standalone;
    VVT_ANGLE           = 0x51: I16, DEGREES,       Standalone;
    VVT_TARGET_ANGLE    = 0x52: U16, DEGREES,       Standalone;
    VVT_DUTY            = 0x53: U16, PERCENT,       Standalone;
    FLEX_BOOST_CORRECT  = 0x54: I16, KPA,           Speeduino;
    BARO_CORRECTION     = 0x55: U16, PERCENT,       Speeduino;
    ASE                 = 0x56: U16, PERCENT,       Speeduino;
    VSS                 = 0x57: U16, KMH,           Common;
    GEAR                = 0x58: U8,  RAW,           Common;
    FUEL_PRES           = 0x59: U16, KPA,           Common;
    OIL_PRES            = 0x5A: U16, KPA,           Standalone;
    WMI_PW              = 0x5B: U16, PERCENT,       Standalone;
    STA_STATUS4         = 0x5C: U8,  RAW,           Speeduino;
    VVT_ANGLE2          = 0x5D: I16, DEGREES,       Standalone;
    VVT_TARGET_ANGLE2   = 0x5E: U16, DEGREES,       Standalone;
    VVT_DUTY2           = 0x5F: U16, PERCENT,       Standalone;
    STATUS_OUT_STA      = 0x60: U8,  RAW,           Speeduino;
    FLEX_FUEL_TEMP      = 0x61: I16, CELSIUS,       Standalone;
    FUEL_TEMP_CORRECT   = 0x62: U16, PERCENT,       Speeduino;
    VE1                 = 0x63: U16, PERCENT,       Standalone;
    VE2                 = 0x64: U16, PERCENT,       Standalone;
    ADVANCE1            = 0x66: I16, DEGREES,       Standalone;
    ADVANCE2            = 0x67: I16, DEGREES,       Standalone;
    NITRO_STA           = 0x68: U8,  RAW,           Speeduino;
    SD_STA              = 0x69: U8,  RAW,           Speeduino;
    MASTERALIVE         = 0x70: U8,  RAW,           Protocol;
    // Common channels other ECUs broadcast that Speeduino does not.
    OIL_TEMP            = 0x71: I16, CELSIUS,       Common;
    EGT1                = 0x72: I16, CELSIUS,       Common;
    FUEL_LEVEL          = 0x73: U16, PERCENT,       Common;
    COOLANT_PRES        = 0x74: U16, KPA,           Standalone;
    PEDAL               = 0x75: U16, PERCENT,       Common;
    TRANS_TEMP          = 0x76: I16, CELSIUS,       Standalone;
    INJ_DUTY            = 0x77: U16, PERCENT,       Standalone;
    WASTEGATE_PRES      = 0x78: U16, KPA,           Standalone;
    ERROR_COUNT         = 0x79: U16, RAW,           Common;
    // Fuel remaining in litres; FUEL_LEVEL is the percent form.
    FUEL_VOLUME         = 0x7A: U16, LITRES,        Standalone;
}
