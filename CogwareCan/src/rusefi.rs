// SPDX-License-Identifier: GPL-3.0-only
//! rusEFI CAN broadcast converter ("rusEFI verbose" dash output).
//!
//! Layout verified against `firmware/controllers/can/can_verbose.cpp` and
//! `rusEFI_CAN_verbose.dbc` on rusEFI master (2025). Frames are little-endian
//! at a configurable base ID, 0x200 by default (11-bit; a 29-bit variant with
//! the same low bits exists). `scaled_channel` encodings: pressure kPa*30,
//! angle deg*50, percent %*100, temperature °C+40 in a byte, voltage mV,
//! lambda λ*10000, pulse width ms*300, cam angles whole degrees in a signed
//! byte, EGT 5 °C per unsigned byte.
//!
//! Not mapped: TPS2, AUX/MCU temperatures, airflow, fuel consumption, knock,
//! and the status flag byte (its bit meanings differ from Speeduino's).

use crate::decode::{conv::*, CanFrameMap, CanSource, Field};
use crate::field;

pub const DEFAULT_BASE: u16 = 0x200;

/// kPa*30 into tenths of a kPa.
fn kpa_x30(v: i32) -> i32 {
    div_round(v, 3)
}
/// deg*50 into tenths of a degree.
fn deg_x50(v: i32) -> i32 {
    div_round(v, 5)
}
/// %*2 into tenths of a percent.
fn pct_x2(v: i32) -> i32 {
    v * 5
}
/// ms*300 into microseconds.
fn ms_x300(v: i32) -> i32 {
    div_round(v * 10, 3)
}

// base+0 Status
static STATUS: &[Field] = &[
    field!(0, U16, ERROR_COUNT, raw),
    field!(2, U16, NEXT_ERROR, raw),
    field!(5, U8, GEAR, raw),
];

// base+1 Speeds (offset 5 is ignition coil duty; no gauge for it)
static SPEEDS: &[Field] = &[
    field!(0, U16, RPM, raw),
    field!(2, S16, CUR_SPARK_ADVANCE, deg_x50),
    field!(4, U8, INJ_DUTY, pct_x2),
    field!(6, U8, VSS, x10),
    field!(7, U8, ETHANOL_PERCENT, x10),
];

// base+2 PedalAndTps (offset 6 is wastegate valve position, shown as boost duty)
static PEDAL_AND_TPS: &[Field] = &[
    field!(0, S16, PEDAL, hundredths_to_tenths),
    field!(2, S16, TPS, hundredths_to_tenths),
    field!(6, S16, BOOST_PWM, hundredths_to_tenths),
];

// base+3 Sensors1
static SENSORS1: &[Field] = &[
    field!(0, U16, MAP, kpa_x30),
    field!(2, U8, CLNT, celsius_plus40),
    field!(3, U8, IAT, celsius_plus40),
    field!(7, U8, FUEL_LEVEL, pct_x2),
];

// base+4 Sensors2 (offsets 0-1 are padding)
static SENSORS2: &[Field] = &[
    field!(2, U16, OIL_PRES, kpa_x30),
    field!(4, U8, OIL_TEMP, celsius_plus40),
    field!(5, U8, FLEX_FUEL_TEMP, celsius_plus40),
    field!(6, U16, BAT_VOL, raw),
];

// base+5 Fueling
static FUELING: &[Field] = &[field!(4, S16, PULSE_WIDTH1, ms_x300)];

// base+6 Fueling2 (fuel trim bank 1)
static FUELING2: &[Field] = &[field!(4, S16, EGO_CORRECT, hundredths_to_tenths)];

// base+7 Fueling3
static FUELING3: &[Field] = &[
    field!(0, U16, AFR_PRI, lambda_ten_thousandths),
    field!(2, U16, AFR_SEC, lambda_ten_thousandths),
    field!(4, U16, FUEL_PRES, kpa_x30),
];

// base+8 Cams: bank1 intake actual/target, exhaust actual/target, then bank 2
static CAMS: &[Field] = &[
    field!(0, S8, VVT_ANGLE, x10),
    field!(1, S8, VVT_TARGET_ANGLE, x10),
    field!(4, S8, VVT_ANGLE2, x10),
    field!(5, S8, VVT_TARGET_ANGLE2, x10),
];

// base+9 Egts: eight bytes, 5 °C each
static EGTS: &[Field] = &[field!(0, U8, EGT1, x50)];

pub static SOURCE: CanSource = CanSource {
    name: "rusEFI",
    default_base: DEFAULT_BASE,
    frames: &[
        CanFrameMap { id_offset: 0, fields: STATUS },
        CanFrameMap { id_offset: 1, fields: SPEEDS },
        CanFrameMap { id_offset: 2, fields: PEDAL_AND_TPS },
        CanFrameMap { id_offset: 3, fields: SENSORS1 },
        CanFrameMap { id_offset: 4, fields: SENSORS2 },
        CanFrameMap { id_offset: 5, fields: FUELING },
        CanFrameMap { id_offset: 6, fields: FUELING2 },
        CanFrameMap { id_offset: 7, fields: FUELING3 },
        CanFrameMap { id_offset: 8, fields: CAMS },
        CanFrameMap { id_offset: 9, fields: EGTS },
    ],
};
