//! ECUMaster EMU Black CAN stream converter (manual 1.4, firmware 2.169+).
//!
//! Eight little-endian frames at a configurable base, 0x600 by default,
//! 1 Mbit/s (500 kbit/s selectable). Encodings are mixed: pulse width is
//! ms/62, dwell ms/20, lambda λ/128, oil and fuel pressure bar/16, battery
//! V*0.027, several 0.5-step percent and angle bytes. EMU Classic sends the
//! same stream shape; per-field identity there is unverified.
//!
//! Not mapped: analogue inputs, DBW position, traction, output flags.

use crate::decode::{conv::*, CanFrameMap, CanSource, Field};
use crate::field;

pub const DEFAULT_BASE: u16 = 0x600;

/// ms/62 into microseconds.
fn ms_div62(v: i32) -> i32 {
    div_round(v * 1000, 62)
}
/// ms/20 into microseconds.
fn ms_div20(v: i32) -> i32 {
    v * 50
}
/// λ/128 into hundredths of AFR.
fn lambda_div128(v: i32) -> i32 {
    div_round(v * 1470, 128)
}
/// bar/16 into tenths of a kPa.
fn bar_div16(v: i32) -> i32 {
    div_round(v * 1000, 16)
}
/// V*0.027 into millivolts.
fn volt_x27(v: i32) -> i32 {
    v * 27
}

static F0: &[Field] = &[
    field!(0, U16, RPM, raw),
    field!(2, U8, TPS, half),
    field!(3, S8, IAT, x10),
    field!(4, U16, MAP, x10),
    field!(6, U16, PULSE_WIDTH1, ms_div62),
];

static F2: &[Field] = &[
    field!(0, U16, VSS, x10),
    field!(2, U8, BARO, x10),
    field!(3, U8, OIL_TEMP, x10),
    field!(4, U8, OIL_PRES, bar_div16),
    field!(5, U8, FUEL_PRES, bar_div16),
    field!(6, S16, CLNT, x10),
];

static F3: &[Field] = &[
    field!(0, S8, CUR_SPARK_ADVANCE, half),
    field!(1, U8, DWELL, ms_div20),
    field!(2, U8, AFR_PRI, lambda_div128),
    field!(3, U8, EGO_CORRECT, half), // assumed 0-centred; unverified
    field!(4, U16, EGT1, x10),
];

static F4: &[Field] = &[
    field!(0, U8, GEAR, raw),
    field!(2, U16, BAT_VOL, volt_x27),
    field!(7, U8, ETHANOL_PERCENT, x10),
];

static F7: &[Field] = &[
    field!(0, U16, BOOST_TARGET, x10),
    field!(4, U8, AFR_TARGET, lambda_hundredths),
];

pub static SOURCE: CanSource = CanSource {
    name: "ECUMaster",
    default_base: DEFAULT_BASE,
    frames: &[
        CanFrameMap { id_offset: 0, fields: F0 },
        CanFrameMap { id_offset: 2, fields: F2 },
        CanFrameMap { id_offset: 3, fields: F3 },
        CanFrameMap { id_offset: 4, fields: F4 },
        CanFrameMap { id_offset: 7, fields: F7 },
    ],
};
