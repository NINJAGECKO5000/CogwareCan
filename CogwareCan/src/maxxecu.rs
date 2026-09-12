// SPDX-License-Identifier: GPL-3.0-only
//! MaxxECU default CAN output converter (protocol V1.3, 2020-09-29).
//!
//! Every channel is a little-endian `int16` at a fixed 11-bit ID, 500 kbit/s.
//! Most channels are already in tenths (kPa*10, %*10, °C*10, deg*10, km/h*10)
//! and map straight onto the gauge table; lambda is λ*1000, battery V*100,
//! pulse width ms*100, EGT whole °C.
//!
//! Not mapped: wheel speeds, knock, accelerometers.

use crate::decode::{conv::*, CanFrameMap, CanSource, Field};
use crate::field;

pub const DEFAULT_BASE: u16 = 0;

static F520: &[Field] = &[
    field!(0, S16, RPM, raw),
    field!(2, S16, TPS, raw),
    field!(4, S16, MAP, raw),
];

static F521: &[Field] = &[
    field!(0, S16, AFR_PRI, lambda_thousandths),
    field!(2, S16, AFR_SEC, lambda_thousandths),
    field!(4, S16, CUR_SPARK_ADVANCE, raw),
];

static F522: &[Field] = &[
    field!(0, S16, PULSE_WIDTH1, hundredths_to_thousandths),
    field!(2, S16, INJ_DUTY, raw),
    field!(6, S16, VSS, raw),
];

// lambda correction bank A
static F524: &[Field] = &[field!(2, S16, EGO_CORRECT, raw)];

static F527: &[Field] = &[field!(6, S16, AFR_TARGET, lambda_thousandths)];

static F530: &[Field] = &[
    field!(0, S16, BAT_VOL, hundredths_to_thousandths),
    field!(2, S16, BARO, raw),
    field!(4, S16, IAT, raw),
    field!(6, S16, CLNT, raw),
];

static F531: &[Field] = &[
    field!(2, S16, ETHANOL_PERCENT, raw),
    field!(6, S16, EGT1, x10),
];

static F534: &[Field] = &[field!(4, S16, ERROR_COUNT, raw)];

static F536: &[Field] = &[
    field!(0, S16, GEAR, raw),
    field!(2, S16, BOOST_PWM, raw),
    field!(4, S16, OIL_PRES, raw),
    field!(6, S16, OIL_TEMP, raw),
];

static F537: &[Field] = &[
    field!(0, S16, FUEL_PRES, raw),
    field!(2, S16, WASTEGATE_PRES, raw),
    field!(4, S16, COOLANT_PRES, raw),
    field!(6, S16, BOOST_TARGET, raw),
];

static F540: &[Field] = &[
    field!(2, S16, FUEL_VOLUME, raw),
    field!(4, S16, TRANS_TEMP, raw),
];

// intake cam 1 position / target
static F541: &[Field] = &[field!(0, S16, VVT_ANGLE, raw)];
static F542: &[Field] = &[
    field!(0, S16, VVT_TARGET_ANGLE, raw),
    field!(4, S16, NEXT_ERROR, raw),
];

pub static SOURCE: CanSource = CanSource {
    name: "MaxxECU",
    default_base: DEFAULT_BASE,
    frames: &[
        CanFrameMap { id_offset: 0x520, fields: F520 },
        CanFrameMap { id_offset: 0x521, fields: F521 },
        CanFrameMap { id_offset: 0x522, fields: F522 },
        CanFrameMap { id_offset: 0x524, fields: F524 },
        CanFrameMap { id_offset: 0x527, fields: F527 },
        CanFrameMap { id_offset: 0x530, fields: F530 },
        CanFrameMap { id_offset: 0x531, fields: F531 },
        CanFrameMap { id_offset: 0x534, fields: F534 },
        CanFrameMap { id_offset: 0x536, fields: F536 },
        CanFrameMap { id_offset: 0x537, fields: F537 },
        CanFrameMap { id_offset: 0x540, fields: F540 },
        CanFrameMap { id_offset: 0x541, fields: F541 },
        CanFrameMap { id_offset: 0x542, fields: F542 },
    ],
};
