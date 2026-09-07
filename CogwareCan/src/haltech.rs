//! Haltech CAN Broadcast Protocol converter (Elite and Nexus, protocol V2.35).
//!
//! Every channel is a big-endian 16-bit word at a fixed 11-bit ID (the base is
//! not configurable). Temperatures are Kelvin*10, pressures kPa*10 with
//! sensor pressures sent *absolute* (subtract 101.3 kPa for gauge pressure),
//! percentages %*10, angles deg*10, wideband lambda*1000, battery V*10,
//! speed km/h*10.
//!
//! Fuel level arrives in litres and feeds `FUEL_VOLUME`, not `FUEL_LEVEL`.
//! Not mapped: injection stages 2-4, wheel speeds, launch and traction data.

use crate::decode::{conv::*, CanFrameMap, CanSource, Field};
use crate::field;

/// IDs are fixed, so the base is zero and each `id_offset` is the absolute ID.
pub const DEFAULT_BASE: u16 = 0;

static F360: &[Field] = &[
    field!(0, U16BE, RPM, raw),
    field!(2, U16BE, MAP, raw),
    field!(4, U16BE, TPS, raw),
    field!(6, U16BE, COOLANT_PRES, kpa_abs_tenths_to_gauge),
];

// offset 4 is Engine Demand, not pedal position
static F361: &[Field] = &[
    field!(0, U16BE, FUEL_PRES, kpa_abs_tenths_to_gauge),
    field!(2, U16BE, OIL_PRES, kpa_abs_tenths_to_gauge),
    field!(6, U16BE, WASTEGATE_PRES, kpa_abs_tenths_to_gauge),
];

static F362: &[Field] = &[
    field!(0, U16BE, INJ_DUTY, raw),
    field!(4, S16BE, CUR_SPARK_ADVANCE, raw),
];

static F368: &[Field] = &[
    field!(0, U16BE, AFR_PRI, lambda_thousandths),
    field!(2, U16BE, AFR_SEC, lambda_thousandths),
];

static F36F: &[Field] = &[field!(2, U16BE, BOOST_PWM, raw)];

// offsets 2-3 are undefined
static F370: &[Field] = &[
    field!(0, U16BE, VSS, raw),
    field!(4, S16BE, VVT_ANGLE, raw),
    field!(6, S16BE, VVT_ANGLE2, raw),
];

// offsets 2-3 are undefined
static F372: &[Field] = &[
    field!(0, U16BE, BAT_VOL, tenths_to_thousandths),
    field!(4, U16BE, BOOST_TARGET, raw),
    field!(6, U16BE, BARO, raw),
];

static F373: &[Field] = &[field!(0, U16BE, EGT1, kelvin_tenths)];

static F3E0: &[Field] = &[
    field!(0, U16BE, CLNT, kelvin_tenths),
    field!(2, U16BE, IAT, kelvin_tenths),
    field!(4, U16BE, FLEX_FUEL_TEMP, kelvin_tenths),
    field!(6, U16BE, OIL_TEMP, kelvin_tenths),
];

static F3E1: &[Field] = &[
    field!(0, U16BE, TRANS_TEMP, kelvin_tenths),
    field!(4, U16BE, ETHANOL_PERCENT, raw),
];

// short-term fuel trim bank 1
static F3E2: &[Field] = &[field!(0, U16BE, FUEL_VOLUME, raw)];

static F3E3: &[Field] = &[field!(0, S16BE, EGO_CORRECT, raw)];

static F3E9: &[Field] = &[field!(4, U16BE, AFR_TARGET, lambda_thousandths)];

static F3EB: &[Field] = &[
    field!(4, S16BE, ADVANCE1, raw),
    field!(6, S16BE, ADVANCE2, raw),
];

// byte 7: -1 reverse, 0 neutral, 1.. forward
static F470: &[Field] = &[field!(7, S8, GEAR, raw)];

static F471: &[Field] = &[field!(2, U16BE, PEDAL, raw)];

pub static SOURCE: CanSource = CanSource {
    name: "Haltech",
    default_base: DEFAULT_BASE,
    frames: &[
        CanFrameMap { id_offset: 0x360, fields: F360 },
        CanFrameMap { id_offset: 0x361, fields: F361 },
        CanFrameMap { id_offset: 0x362, fields: F362 },
        CanFrameMap { id_offset: 0x368, fields: F368 },
        CanFrameMap { id_offset: 0x36F, fields: F36F },
        CanFrameMap { id_offset: 0x370, fields: F370 },
        CanFrameMap { id_offset: 0x372, fields: F372 },
        CanFrameMap { id_offset: 0x373, fields: F373 },
        CanFrameMap { id_offset: 0x3E0, fields: F3E0 },
        CanFrameMap { id_offset: 0x3E1, fields: F3E1 },
        CanFrameMap { id_offset: 0x3E2, fields: F3E2 },
        CanFrameMap { id_offset: 0x3E3, fields: F3E3 },
        CanFrameMap { id_offset: 0x3E9, fields: F3E9 },
        CanFrameMap { id_offset: 0x3EB, fields: F3EB },
        CanFrameMap { id_offset: 0x470, fields: F470 },
        CanFrameMap { id_offset: 0x471, fields: F471 },
    ],
};
