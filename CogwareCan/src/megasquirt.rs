//! Megasquirt "Simplified Dash Broadcasting" converter (MS2/Extra 3.4.x and
//! MS3 1.4.x, document dated 2016-02-17).
//!
//! Five big-endian frames starting at a configurable base ID, 1512 (0x5E8)
//! by default. Temperatures are °F*10, pressures kPa*10, percentages %*10,
//! pulse widths in µs, advance deg*10, AFR*10, battery V*10, VSS1 in m/s*10.
//! `egt1` and `VSS1` are MS3-only and read zero on MS2.
//!
//! Not mapped: sequential pulse width, GPIO ADC inputs, knock retard,
//! traction and launch timing. The larger "Advanced" broadcast at base 1520
//! is a different layout and is not handled here.

use crate::decode::{conv::*, CanFrameMap, CanSource, Field};
use crate::field;

pub const DEFAULT_BASE: u16 = 1512;

static F0: &[Field] = &[
    field!(0, S16BE, MAP, raw),
    field!(2, U16BE, RPM, raw),
    field!(4, S16BE, CLNT, fahrenheit_tenths),
    field!(6, S16BE, TPS, raw),
];

static F1: &[Field] = &[
    field!(0, U16BE, PULSE_WIDTH1, raw),
    field!(2, U16BE, PULSE_WIDTH2, raw),
    field!(4, S16BE, IAT, fahrenheit_tenths),
    field!(6, S16BE, CUR_SPARK_ADVANCE, raw),
];

static F2: &[Field] = &[
    field!(0, U8, AFR_TARGET, tenths_to_hundredths),
    field!(1, U8, AFR_PRI, tenths_to_hundredths),
    field!(2, S16BE, EGO_CORRECT, centred1000_to_trim_tenths),
    field!(4, S16BE, EGT1, fahrenheit_tenths),
];

static F3: &[Field] = &[field!(0, S16BE, BAT_VOL, tenths_to_thousandths)];

static F4: &[Field] = &[field!(0, U16BE, VSS, mps_tenths_to_kmh_tenths)];

pub static SOURCE: CanSource = CanSource {
    name: "Megasquirt",
    default_base: DEFAULT_BASE,
    frames: &[
        CanFrameMap { id_offset: 0, fields: F0 },
        CanFrameMap { id_offset: 1, fields: F1 },
        CanFrameMap { id_offset: 2, fields: F2 },
        CanFrameMap { id_offset: 3, fields: F3 },
        CanFrameMap { id_offset: 4, fields: F4 },
    ],
};
